# SPDX-License-Identifier: GPL-2.0-only

"""Offline saved car topology, indexed source attribution and approach replacements.

Uses the importer's existing 32 m SegmentIndex. Cost is O(vertices + edges +
local grid candidates), plus stable sorting; no simulation-tick work.
"""

from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sqlite3

from .conflate import SegmentIndex
from .geometry import distance, length


def read_save(path):
    with sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True) as c:
        nodes = {i: [x, y, z] for i, x, y, z in c.execute('SELECT node_id,x,y,z FROM network_nodes')}
        shapes = defaultdict(list)
        for i, x, y, z in c.execute('SELECT edge_id,x,y,z FROM network_edge_geometry WHERE physical=0 ORDER BY edge_id,point_index'):
            shapes[i].append([x, y, z])
        edges = [dict(zip(('id', 'a', 'b', 'fwd', 'bkw', 'class', 'median_style', 'median_width', 'length_m', 'start_clip', 'end_clip'), row))
                 for row in c.execute('SELECT edge_id,start_node,end_node,fwd_lanes,bkw_lanes,class,median_style,median_width,physical_length,start_clip,end_clip FROM network_edges WHERE fwd_lanes+bkw_lanes>0')]
        tables = {r[0] for r in c.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        marked = set(c.execute('SELECT node_id,edge_id FROM road_crosswalks WHERE marked=1')) if 'road_crosswalks' in tables else set()
        tapers = defaultdict(dict)
        if 'road_lane_tapers' in tables:
            for eid, end, run, left, right, fwd, bkw in c.execute('SELECT edge_id,end_index,length_m,left_pocket,right_pocket,from_fwd,from_bkw FROM road_lane_tapers'):
                tapers[eid][end] = {'length_m': run, 'pocket': bool(left or right), 'from_fwd': fwd, 'from_bkw': bkw}
        for e in edges:
            e['tapers'] = tapers[e['id']]
            e['points'] = shapes[e['id']]
            e['marked_nodes'] = [n for n in (e['a'], e['b']) if (n, e['id']) in marked]
    return nodes, edges


def topology(nodes, edges):
    parents = {n: n for e in edges for n in (e['a'], e['b'])}
    def root(n):
        while parents[n] != n:
            parents[n] = parents[parents[n]]
            n = parents[n]
        return n
    for e in edges:
        a, b = root(e['a']), root(e['b'])
        parents[max(a, b)] = min(a, b)
    groups, incident = defaultdict(list), defaultdict(list)
    for e in edges:
        e['component'] = root(e['a'])
        groups[e['component']].append(e)
        for n in (e['a'], e['b']):
            incident[n].append(e)
    return groups, incident


def directed_components(edges):
    """Count strongly connected node groups without recursive traversal."""
    forward, backward = defaultdict(list), defaultdict(list)
    for e in edges:
        for a, b, enabled in ((e['a'], e['b'], e['fwd']), (e['b'], e['a'], e['bkw'])):
            forward.setdefault(a, [])
            backward.setdefault(a, [])
            if enabled:
                forward[a].append(b)
                backward[b].append(a)
    visited, order = set(), []
    for seed in sorted(forward):
        if seed in visited:
            continue
        visited.add(seed)
        stack = [(seed, iter(forward[seed]))]
        while stack:
            node, neighbors = stack[-1]
            nxt = next(neighbors, None)
            if nxt is None:
                order.append(node)
                stack.pop()
            elif nxt not in visited:
                visited.add(nxt)
                stack.append((nxt, iter(forward[nxt])))
    visited, sizes = set(), []
    for seed in reversed(order):
        if seed in visited:
            continue
        visited.add(seed)
        stack, size = [seed], 0
        while stack:
            node = stack.pop()
            size += 1
            for nxt in backward[node]:
                if nxt not in visited:
                    visited.add(nxt)
                    stack.append(nxt)
        sizes.append(size)
    return {'strong_components': len(sizes), 'largest_strong_component_nodes': max(sizes, default=0),
            'strong_component_sizes': sorted(sizes, reverse=True)}


def source_index(plan):
    return SegmentIndex((s['id'], s['points_xz'], s) for s in plan['strokes'] if s['fwd_lanes']+s['bkw_lanes'])


def attribute(edges, index):
    for e in edges:
        pts = e['points']
        a, b = max(zip(pts, pts[1:]), key=lambda ab: distance((ab[0][0], ab[0][2]), (ab[1][0], ab[1][2])))
        p = [(a[j]+b[j])/2 for j in (0, 2)]
        hit = index.nearest(p, 35)
        e['source'] = hit[3][3] if hit else {}
        # Persisted edges may run opposite to their attributed source section.
        # Directional permissions must follow the local source tangent, not IDs.
        e['source_forward'] = bool(hit and sum((b[j]-a[j])*(hit[3][2][k]-hit[3][1][k])
                                              for k, j in enumerate((0, 2))) >= 0)


def analyse(path, plan):
    nodes, edges = read_save(path)
    groups, incident = topology(nodes, edges)
    attribute(edges, source_index(plan))
    main = max(groups, key=lambda k: sum(e['length_m'] for e in groups[k]))
    index = SegmentIndex((e['id'], [(p[0], p[2]) for p in e['points']], e) for e in edges)
    records, candidates = [], []
    for cid, members in sorted(groups.items(), key=lambda kv: (-sum(e['length_m'] for e in kv[1]), kv[0])):
        nearest = None
        for n in sorted({n for e in members for n in (e['a'], e['b'])}):
            if len(incident[n]) != 1:
                continue
            e = incident[n][0]
            p = nodes[n][::2]
            hit = index.nearest(p, 120, lambda s: s[3]['component'] != cid)
            if not hit:
                continue
            item = {'node': n, 'edge': e['id'], 'target_edge': hit[3][3]['id'],
                    'target_component': hit[3][3]['component'], 'gap_m': hit[0], 'point_xz': p, 'target_xz': hit[2]}
            if nearest is None or (item['gap_m'], n) < (nearest['gap_m'], nearest['node']):
                nearest = item
            if e['class'] != 0 or hit[3][3]['class'] != 0 or hit[0] > 45 or hit[0] < .05:
                continue
            # Only reconnect an authored source junction or an interrupted source stroke.
            own, target = e['source'], hit[3][3]['source']
            authored = own.get('id') == target.get('id')
            authored |= any(distance(q, hit[2]) <= 18 for q in own.get('junctions_xz', []))
            authored |= any(distance(q, hit[2]) <= 18 for q in (own.get('points_xz') or [])[::max(1, len(own.get('points_xz', []))-1)])
            if not authored:
                continue
            target_edge = hit[3][3]
            q = hit[2]
            # Reuse a nearby junction rather than creating a second mouth inside its apron.
            ends = sorted((distance(q, nodes[k][::2]), k) for k in (target_edge['a'], target_edge['b']))
            if ends[0][0] <= 30:
                q = nodes[ends[0][1]][::2]
            points = [(p[0], p[2]) for p in e['points']]
            at_start = n == e['a']
            ordered = points if at_start else points[::-1]
            chain = [e]
            remote = e['b'] if at_start else e['a']
            while length(ordered) < 70 and len(incident[remote]) == 2 and len(chain) < 8:
                other = next(x for x in incident[remote] if x['id'] != chain[-1]['id'])
                if other in chain or other['class'] != e['class'] or any(other[k] != e[k] for k in ('fwd', 'bkw', 'median_style', 'median_width')):
                    break
                if other['a'] != remote and other['fwd'] != other['bkw']:
                    break
                part = [(p[0], p[2]) for p in other['points']]
                if other['b'] == remote:
                    part.reverse()
                ordered.extend(part[1:])
                chain.append(other)
                remote = other['b'] if other['a'] == remote else other['a']
            tip = ordered[0]
            # Keep the remote geometry; remove short controls inside the replacement throat.
            while len(ordered) > 2 and distance(tip, ordered[1]) < 24:
                ordered.pop(1)
            ordered[0] = q
            points = ordered if at_start else ordered[::-1]
            candidates.append({**item, 'points_xz': points, 'old_points': e['points'], 'edges': [x['id'] for x in chain],
                               'fwd_lanes': e['fwd'], 'bkw_lanes': e['bkw'],
                               'median_style': e['median_style'], 'median_width_m': e['median_width'],
                               'names': own.get('names', []), 'source_id': own.get('id'),
                               'reason': 'restore_source_junction_or_stroke'})
        records.append({'component': cid, 'main': cid == main, 'edges': len(members),
                        'length_m': sum(e['length_m'] for e in members),
                        'names': sorted({name for e in members for name in e['source'].get('names', [])}),
                        'classes': dict(Counter(e['source'].get('road_class', 'unattributed') for e in members)),
                        'edge_ids': sorted(e['id'] for e in members), 'nearest_other_component': nearest,
                        'isolation': 'main_network' if cid == main else ('unplaced_junction_or_trimmed_approach' if nearest else 'no_other_car_component_within_120m')})
    classes = defaultdict(float)
    for e in edges:
        classes[e['source'].get('road_class', 'unattributed')] += e['length_m']
    return {**directed_components(edges), 'components': len(groups), 'car_edges': len(edges), 'car_length_m': sum(e['length_m'] for e in edges),
            'main_length_m': sum(e['length_m'] for e in groups[main]), 'by_class_physical_m': dict(sorted(classes.items())),
            'component_details': records, 'candidates': sorted(candidates, key=lambda c: (c['gap_m'], c['node'], c['target_edge']))}


def coalesced_gap_actions(path, plan):
    """Join source-backed, same-section dead ends as one continuous road.

    A short terminal span has no space for two overlapping junction aprons. The
    existing degree-two chain is coalesced on both sides (at most eight edges
    each), preserving directional order and refusing incompatible sections.
    """
    from .curves import smooth
    from .geometry import simplify
    report = analyse(path, plan)
    nodes, edges = read_save(path)
    _, incident = topology(nodes, edges)
    lookup = {e['id']: e for e in edges}
    actions, used = [], set()
    def chain(edge, node):
        ordered, ids = [], []
        counts = (edge['fwd'], edge['bkw']) if edge['a'] == node else (edge['bkw'], edge['fwd'])
        while len(ids) < 8:
            forward = edge['a'] == node
            if ((edge['fwd'], edge['bkw']) if forward else (edge['bkw'], edge['fwd'])) != counts:
                break
            part = [p[::2] for p in (edge['points'] if forward else edge['points'][::-1])]
            ordered.extend(part if not ordered else part[1:])
            ids.append(edge['id'])
            node = edge['b'] if forward else edge['a']
            if length(ordered) >= 100 or len(incident[node]) != 2:
                break
            other = next(e for e in incident[node] if e['id'] != edge['id'])
            if other['id'] in ids or any(other[k] != edge[k] for k in ('class', 'median_style', 'median_width')):
                break
            edge = other
        return ordered, ids, counts
    for c in report['candidates']:
        own, target = lookup[c['edge']], lookup[c['target_edge']]
        tips = [k for k in (target['a'], target['b']) if len(incident[k]) == 1]
        if not tips:
            continue
        tip = min(tips, key=lambda k: distance(nodes[k][::2], c['target_xz']))
        if distance(nodes[tip][::2], c['target_xz']) > 30:
            continue
        a, ai, ac = chain(own, c['node'])
        b, bi, bc = chain(target, tip)
        if ac[::-1] != bc or any(own[k] != target[k] for k in ('class', 'median_style', 'median_width')):
            continue
        if set(ai+bi) & used:
            continue
        for part in (a, b):
            while len(part) > 2 and distance(part[0], part[1]) < 24:
                part.pop(1)
        points = smooth(simplify(a[::-1]+b, 1.0))
        actions.append({**c, 'kind': 'replace', 'edges': ai+bi, 'points_xz': points,
                        'fwd_lanes': ac[1], 'bkw_lanes': ac[0],
                        'reason': 'coalesce_source_backed_same_section_gap'})
        used.update(ai+bi)
    return {'actions': actions}


def whole_stroke_actions(path, plan):
    """Rebuild fragmented authored strokes after all neighboring roads exist.

    Matching section and standard-road class are mandatory. The player compiler
    validates the complete source geometry; rejection restores every old piece.
    """
    nodes, edges = read_save(path)
    groups, _ = topology(nodes, edges)
    main = max(groups, key=lambda k: sum(e['length_m'] for e in groups[k]))
    attribute(edges, source_index(plan))
    by_source = defaultdict(list)
    for e in edges:
        if e['source']:
            by_source[e['source']['id']].append(e)
    actions = []
    for sid, members in by_source.items():
        source = members[0]['source']
        if source.get('bridge') or source.get('tunnel') or not 60 <= source['length_m'] <= 1500:
            continue
        if len({e['component'] for e in members}) < 2 and (not source.get('names') or members[0]['component'] == main):
            continue
        if any(e['class'] != 0 or any(e[a] != source[b] for a, b in
               (('fwd', 'fwd_lanes'), ('bkw', 'bkw_lanes'), ('median_style', 'median_style'))) for e in members):
            continue
        if source.get('names') == ['Savilahdentie'] and any(distance(p, (-999.179, 1829.554)) < .1 for p in source['points_xz']):
            continue
        first = members[0]
        actions.append({**source, 'kind': 'replace', 'edge': first['id'],
                        'edges': sorted(e['id'] for e in members), 'node': first['a'],
                        'point_xz': nodes[first['a']][::2], 'require_dead_end': False,
                        'reason': 'whole_authored_stroke_after_neighbors',
                        'source_components': len({e['component'] for e in members})})
    actions.sort(key=lambda a: (-a['source_components'], -bool(a.get('names')), -a['length_m'], a['id']))
    return {'actions': actions}


if __name__ == '__main__':
    import sys
    result = (whole_stroke_actions if '--whole' in sys.argv else coalesced_gap_actions if '--joins' in sys.argv else analyse)(sys.argv[1], json.loads(Path(sys.argv[2]).read_text()))
    Path(sys.argv[3]).write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('candidates', 'component_details')}, indent=2))
