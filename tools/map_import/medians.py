# SPDX-License-Identifier: GPL-2.0-only

"""Deterministic offline carriageway pairing using the existing segment grid.

Directed source chains ignore lane-pocket boundaries. Opposing samples require
same identity, class, layer and a <=30 degree reversed heading. Work is linear
in sampled length plus local indexed candidates and sorting, never a game tick.
"""

from collections import defaultdict, deque
from copy import deepcopy
import math
import statistics

from .geometry import distance, roadbed_half_width, simplify
from .mapping import POLICY


def median_section(separation, forward, backward, paved=False):
    gap = separation - 1.75*(forward+backward)
    style = 1 if gap < 1 else 3 if gap < 2 or paved else 2
    low, high = {1: (.5, 2), 2: (2, 12), 3: (1, 3)}[style]
    return {'median_style': style, 'median_width_m': round(max(low, min(high, gap)), 2)}


def source_through_count(feature):
    p = feature['properties']
    directional = p.get('lanes_forward' if p['oneway'] == 1 else 'lanes_backward')
    return int(p.get('through_lanes') or directional or p.get('lanes') or 1)


def infer_carriageway_identities(roads):
    """Recover an unnamed one-way continuation only from unambiguous source incidence."""
    candidates = {f['id']: f for f, m in roads if m['action'] == 'road'
                  and f['properties'].get('oneway') and not f['properties']['road_class'].endswith('_link')}
    incidence = defaultdict(set)
    for sid, f in candidates.items():
        for n in (f['node_ids'][0], f['node_ids'][-1]):
            incidence[n, f['properties']['road_class']].add(sid)
    visited, audit = set(), []
    for seed in sorted(candidates):
        if seed in visited or candidates[seed]['properties'].get('name') or candidates[seed]['properties'].get('ref'):
            continue
        pending, members, identities = [seed], [], set()
        visited.add(seed)
        while pending:
            sid = pending.pop()
            f = candidates[sid]
            members.append(f)
            for n in (f['node_ids'][0], f['node_ids'][-1]):
                for peer in sorted(incidence[n, f['properties']['road_class']]):
                    g = candidates[peer]
                    fn = f['node_ids'][::f['properties']['oneway']]
                    gn = g['node_ids'][::g['properties']['oneway']]
                    if not ((fn[-1] == n == gn[0]) or (gn[-1] == n == fn[0])):
                        continue
                    fp = f['points'][::f['properties']['oneway']]
                    gp = g['points'][::g['properties']['oneway']]
                    fa = fp[-2:] if fn[-1] == n else fp[:2]
                    ga = gp[:2] if gn[0] == n else gp[-2:]
                    u = fa[1][0]-fa[0][0], fa[1][1]-fa[0][1]
                    v = ga[1][0]-ga[0][0], ga[1][1]-ga[0][1]
                    if u[0]*v[0]+u[1]*v[1] < .5*math.hypot(*u)*math.hypot(*v):
                        continue
                    identity = g['properties'].get('name') or g['properties'].get('ref')
                    if identity:
                        identities.add(identity)
                    elif peer not in visited:
                        visited.add(peer)
                        pending.append(peer)
        if len(identities) != 1:
            continue
        identity = next(iter(identities))
        for f in members:
            f['properties']['name'] = identity
        audit.append({'source_ids': sorted(f['id'] for f in members), 'identity': identity,
                      'authority': 'unambiguous_source_carriageway_continuation'})
    return audit


def carriageway_chains(roads):
    candidates = [(f, m) for f, m in roads if m['action'] == 'road'
                  and f['properties'].get('oneway') and not f['properties']['road_class'].endswith('_link')
                  and (f['properties'].get('name') or f['properties'].get('ref'))]
    ends = defaultdict(list)
    for i, (f, _) in enumerate(candidates):
        p = f['properties']
        key = (p.get('name') or p.get('ref'), p['road_class'],
               p.get('layer', 0), p.get('bridge', False), p.get('tunnel', False),
               p.get('survey_cross_section'))
        ids = f['node_ids'][::p['oneway']]
        ends[key, ids[0], 'out'].append(i)
        ends[key, ids[-1], 'in'].append(i)
    used, chains = set(), []
    for seed, (f, _) in enumerate(candidates):
        if seed in used:
            continue
        p = f['properties']
        key = (p.get('name') or p.get('ref'), p['road_class'],
               p.get('layer', 0), p.get('bridge', False), p.get('tunnel', False),
               p.get('survey_cross_section'))
        members = deque([seed])
        used.add(seed)
        for start in (False, True):
            while True:
                g = candidates[members[0 if start else -1]][0]
                ids = g['node_ids'][::g['properties']['oneway']]
                node = ids[0 if start else -1]
                ins, outs = ends[key, node, 'in'], ends[key, node, 'out']
                if len(ins) != 1 or len(outs) != 1:
                    break
                nxt = ins[0] if start else outs[0]
                if nxt in used:
                    break
                current = g['points'][::g['properties']['oneway']]
                candidate = candidates[nxt][0]
                other = candidate['points'][::candidate['properties']['oneway']]
                a, b = (other[-2:], current[:2]) if start else (current[-2:], other[:2])
                u = a[1][0]-a[0][0], a[1][1]-a[0][1]
                v = b[1][0]-b[0][0], b[1][1]-b[0][1]
                if u[0]*v[0]+u[1]*v[1] < .5*math.hypot(*u)*math.hypot(*v):
                    break
                used.add(nxt)
                members.appendleft(nxt) if start else members.append(nxt)
        fs = [candidates[i][0] for i in members]
        pts = [p for j, g in enumerate(fs) for p in g['points'][::g['properties']['oneway']][int(j > 0):]]
        # Through-only lane tags take precedence. The minimum along a chain
        # removes local widening/turn pockets which slice B cannot represent.
        counts = [source_through_count(g) for g in fs]
        chains.append({'id': min(g['id'] for g in fs), 'key': key, 'features': fs,
                       'points': pts, 'lanes': min(counts)})
    return sorted(chains, key=lambda c: c['id'])


def carriageway_retention(chains, active, measured, axes):
    """Orient opposing pairs consistently through unique source continuations.

    Opposing sampled chains have opposite parity. An unambiguous directed
    source-node continuation has the same parity, even across names/structures.
    A contradictory component keeps its original one-way sources for audit.
    """
    links = {cid: set() for cid in active}
    def link(a, b, opposite):
        if a != b and a in links and b in links:
            links[a].add((b, opposite))
            links[b].add((a, opposite))
    ends = defaultdict(lambda: {'in': [], 'out': []})
    lookup = {c['id']: c for c in chains}
    for c in chains:
        if c['id'] not in active:
            continue
        for other, _ in measured[c['id']][2]:
            link(c['id'], other, True)
        first, last = c['features'][0], c['features'][-1]
        a = first['node_ids'][::first['properties']['oneway']][0]
        b = last['node_ids'][::last['properties']['oneway']][-1]
        ends[c['key'][1], a]['out'].append(c['id'])
        ends[c['key'][1], b]['in'].append(c['id'])
    for sides in ends.values():
        if len(sides['in']) == len(sides['out']) == 1:
            incoming, outgoing = sides['in'][0], sides['out'][0]
            a, b = lookup[incoming]['points'][-2:], lookup[outgoing]['points'][:2]
            u = a[1][0]-a[0][0], a[1][1]-a[0][1]
            v = b[1][0]-b[0][0], b[1][1]-b[0][1]
            # Use the existing chain-continuation angle contract. Opposing ways
            # can meet at an undivided convergence without continuing as one side.
            if u[0]*v[0]+u[1]*v[1] >= .5*math.hypot(*u)*math.hypot(*v):
                link(incoming, outgoing, False)
    retained = {}
    conflicts = set()
    for seed in sorted(active):
        if seed in retained:
            continue
        c = lookup[seed]
        axis = axes[c['key'][:2]]
        retained[seed] = c['points'][-1][axis] >= c['points'][0][axis]
        pending = [seed]
        component, invalid = [], False
        while pending:
            current = pending.pop()
            component.append(current)
            for other, opposite in sorted(links[current]):
                keep = retained[current] != opposite
                if other in retained:
                    invalid |= retained[other] != keep
                else:
                    retained[other] = keep
                    pending.append(other)
        if invalid:
            conflicts.update(component)
    return {cid: keep for cid, keep in retained.items() if cid not in conflicts}, conflicts


def merge_carriageways(roads, decisions):
    # Import here: conflate owns the shared index and calls this pass.
    from .conflate import SegmentIndex, aligned, samples
    chains = carriageway_chains(roads)
    by_source = {f['id']: c for c in chains for f in c['features']}
    chain_lookup = {c['id']: c for c in chains}
    mappings = {f['id']: m for f, m in roads}
    index = SegmentIndex((f['id'], f['points'][::f['properties']['oneway']], (c, f))
                         for c in chains for f in c['features'])
    measured = {}
    feature_opponents = defaultdict(set)
    for c in chains:
        hits, total, matched = [], 0., 0.
        for f in c['features']:
            for p, u, run in samples(f['points'][::f['properties']['oneway']], POLICY['sample_m']):
                total += run
                hit = index.nearest(p, POLICY['carriageway_distance_m'],
                    lambda s: s[3][0]['id'] != c['id'] and s[3][0]['key'] == c['key']
                    and s[3][1]['properties']['layer'] == f['properties']['layer']
                    and aligned(u, s[1], s[2], True))
                if hit:
                    matched += run
                    hits.append((hit[3][3][0]['id'], hit[0]))
                    feature_opponents[f['id']].add(hit[3][3][0]['id'])
        measured[c['id']] = total, matched, hits
    extents = defaultdict(list)
    for c in chains:
        extents[c['key'][:2]].extend(c['points'])
    axes = {key: int(max(p[1] for p in pts)-min(p[1] for p in pts) > max(p[0] for p in pts)-min(p[0] for p in pts))
            for key, pts in extents.items()}
    active = {c['id'] for c in chains if measured[c['id']][0] and measured[c['id']][1]/measured[c['id']][0] >= POLICY['parallel_fraction']}
    # Only reciprocal chain evidence can remove an entire carriageway.
    active = {cid for cid in active if any(other in active for other, _ in measured[cid][2])}
    orientation, conflicts = carriageway_retention(chains, active, measured, axes)
    active.difference_update(conflicts)
    paired_features = {fid for fid, opponents in feature_opponents.items() if opponents & active}
    for c in chains:
        if c['id'] in active:
            # Cross-median connectors and unpaired approach heads are not evidence
            # of the parallel carriageway's cross-section. Keep those source ways
            # separately; only actual paired features supply the shared lane counts.
            c['lanes'] = min(source_through_count(f) for f in c['features'] if f['id'] in paired_features)
    retained, audit, shifted = [], [], set()
    for c in chains:
        total, matched, hits = measured[c['id']]
        if c['id'] not in active:
            audit.append({'name': c['key'][0], 'road_class': c['key'][1], 'source_ids': [f['id'] for f in c['features']], 'status': 'fallback',
                          'reason': 'inconsistent_carriageway_pairing_topology' if c['id'] in conflicts else 'irregular_or_unpaired_carriageway', 'paired_m': round(matched, 3), 'length_m': round(total, 3)})
            continue
        keep = orientation[c['id']]
        opponents = [chain_lookup[cid] for cid in sorted({cid for cid, _ in hits}) if cid in active]
        opposing = min(o['lanes'] for o in opponents)
        paved = any(f['properties'].get('median_surface') in ('paved', 'paving_stones', 'concrete', 'asphalt')
                    for o in [c]+opponents for f in o['features'])
        section = median_section(statistics.median(d for _, d in hits), c['lanes'], opposing, paved)
        sources = sorted({f['id'] for o in [c]+opponents for f in o['features']})
        audit.append({'name': c['key'][0], 'road_class': c['key'][1], 'source_ids': [f['id'] for f in c['features']], 'opposing_source_ids': [s for s in sources if s not in {f['id'] for f in c['features']}],
                      'status': 'merged' if keep else 'suppressed', 'paired_m': round(matched, 3),
                      'length_m': round(total, 3), 'separation_m': round(statistics.median(d for _, d in hits), 3),
                      'fwd_lanes': c['lanes'], 'bkw_lanes': opposing, **section,
                      'reason': 'paired_source_sections_with_native_transitions'})
        shifted.update(n for f in c['features'] if f['id'] in paired_features for n in f['node_ids'])
        if not keep:
            continue
        for f in c['features']:
            if f['id'] not in paired_features:
                continue
            pts = f['points'][::f['properties']['oneway']]
            dense = list(samples(pts, POLICY['sample_m']))
            dense.append((pts[-1], dense[-1][1], 0))
            moved, paired_ends, local_sources = [], [], {f['id']}
            peer_counts, peer_bus_masks = [], []
            for sample_index, (p, u, run) in enumerate(dense):
                hit = index.nearest(p, POLICY['carriageway_distance_m'],
                    lambda s: s[3][0]['id'] != c['id'] and s[3][0]['key'] == c['key']
                    and s[3][0]['id'] in active and s[3][1]['properties']['layer'] == f['properties']['layer']
                    and aligned(u, s[1], s[2], True))
                if hit:
                    local_sources.add(hit[3][0])
                    if run > 0:
                        peer_counts.append(source_through_count(hit[3][3][1]))
                        peer_bus_masks.append(mappings[hit[3][3][1]['id']].get('bus_masks', [0, 0])[0])
                if sample_index in (0, len(dense)-1):
                    paired_ends.append(hit is not None)
                q = tuple(round((p[j]+hit[2][j])/2, 3) for j in range(2)) if hit else tuple(p)
                if not moved or q != moved[-1]:
                    moved.append(q)
            g = deepcopy(f)
            g['points'] = simplify(moved, .5)
            g['node_ids'] = [f'median/{p[0]:.3f}/{p[1]:.3f}' for p in g['points']]
            g['source_ids'] = sorted(local_sources)
            g['properties']['oneway'] = 0
            g['_source_end_nodes'] = [f['node_ids'][::f['properties']['oneway']][i] for i in (0, -1)]
            g['_paired_endpoints'] = paired_ends
            m = mappings[f['id']]
            forward, backward = source_through_count(f), min(peer_counts)
            masks = [m.get('bus_masks', [0, 0])[0], max(peer_bus_masks, default=0)]
            if any(masks):
                section = {**section, 'bus_masks': masks}
            if f['properties'].get('survey_cross_section') == 'university_2026_10_22':
                section = {'median_style': 3, 'median_width_m': 2.0, 'bus_masks': [4, 4]}
            retained.append((g, {**m, 'oneway': 0, 'fwd_lanes': forward, 'bkw_lanes': backward, **section}))
            audit[-1].setdefault('sections', []).append({'source_id': f['id'], 'fwd_lanes': forward,
                                                        'bkw_lanes': backward, **section})
    # One named-road axis keeps the retained direction consistent through short
    # structures. Shared source endpoint IDs then supply exact joints across levels;
    # independent closest-point midlines can disagree at those same source nodes.
    joints = {}
    for f, _ in sorted(retained, key=lambda r: r[0]['id']):
        for node, point, paired in zip(f['_source_end_nodes'], (f['points'][0], f['points'][-1]), f.pop('_paired_endpoints')):
            # A section boundary can extend beyond one opposing chain. Prefer
            # measured midline evidence over that chain's unshifted source end.
            identity = (f['properties'].get('name') or f['properties'].get('ref'), f['properties']['road_class'])
            if node not in joints or (paired and not joints[node][1] and identity == joints[node][2]):
                joints[node] = point, paired, identity
    for f, m in retained:
        for end, node in zip((0, -1), f.pop('_source_end_nodes')):
            q = joints[node][0]
            moved = distance(f['points'][end], q) > .01
            f['points'][end] = q
            f['node_ids'][end] = f'median/{q[0]:.3f}/{q[1]:.3f}'
            if moved:
                # Reattachment replaces the old carriageway mouth. Its controls
                # inside the new roadbed footprint cannot define a separate bend
                # before the retained outside alignment resumes.
                half = roadbed_half_width({'kind': 'road', **m})
                adjacent = 1 if end == 0 else -2
                while len(f['points']) > 2 and distance(f['points'][adjacent], q) < half:
                    f['points'].pop(adjacent)
                    f['node_ids'].pop(adjacent)
    median_index = SegmentIndex((f['id'], f['points'], f) for f, _ in retained)
    for f, m in roads:
        chain = by_source.get(f['id'])
        if chain and chain['id'] in active and f['id'] in paired_features:
            continue
        for i, node in enumerate(f['node_ids']):
            if node in shifted:
                hit = median_index.nearest(f['points'][i], POLICY['carriageway_distance_m'])
                if hit:
                    q = tuple(round(v, 3) for v in hit[2])
                    f['points'][i], f['node_ids'][i] = q, f'median/{q[0]:.3f}/{q[1]:.3f}'
        retained.append((f, m))
    return sorted(retained, key=lambda r: r[0]['id']), audit

