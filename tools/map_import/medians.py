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

from .geometry import distance, simplify
from .mapping import POLICY


def median_section(separation, forward, backward, paved=False):
    gap = separation - 1.75*(forward+backward)
    style = 1 if gap < 1 else 3 if gap < 2 or paved else 2
    low, high = {1: (.5, 2), 2: (2, 12), 3: (1, 3)}[style]
    return {'median_style': style, 'median_width_m': round(max(low, min(high, gap)), 2)}


def carriageway_chains(roads):
    candidates = [(f, m) for f, m in roads if m['action'] == 'road'
                  and f['properties'].get('oneway') and not f['properties']['road_class'].endswith('_link')
                  and (f['properties'].get('name') or f['properties'].get('ref'))]
    ends = defaultdict(list)
    for i, (f, _) in enumerate(candidates):
        p = f['properties']
        key = p.get('name') or p.get('ref'), p['road_class']
        ids = f['node_ids'][::p['oneway']]
        ends[key, ids[0], 'out'].append(i)
        ends[key, ids[-1], 'in'].append(i)
    used, chains = set(), []
    for seed, (f, _) in enumerate(candidates):
        if seed in used:
            continue
        key = f['properties'].get('name') or f['properties'].get('ref'), f['properties']['road_class']
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
        counts = []
        for g in fs:
            p = g['properties']
            directional = p.get('lanes_forward' if p['oneway'] == 1 else 'lanes_backward')
            counts.append(int(p.get('through_lanes') or directional or p.get('lanes') or 1))
        chains.append({'id': min(g['id'] for g in fs), 'key': key, 'features': fs,
                       'points': pts, 'lanes': min(counts)})
    return sorted(chains, key=lambda c: c['id'])


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
        measured[c['id']] = total, matched, hits
    extents = defaultdict(list)
    for c in chains:
        extents[c['key']].extend(c['points'])
    axes = {key: int(max(p[1] for p in pts)-min(p[1] for p in pts) > max(p[0] for p in pts)-min(p[0] for p in pts))
            for key, pts in extents.items()}
    active = {c['id'] for c in chains if measured[c['id']][0] and measured[c['id']][1]/measured[c['id']][0] >= POLICY['parallel_fraction']}
    # Only reciprocal chain evidence can remove an entire carriageway.
    active = {cid for cid in active if any(other in active for other, _ in measured[cid][2])}
    retained, audit, shifted = [], [], set()
    for c in chains:
        total, matched, hits = measured[c['id']]
        if c['id'] not in active:
            audit.append({'name': c['key'][0], 'road_class': c['key'][1], 'source_ids': [f['id'] for f in c['features']], 'status': 'fallback',
                          'reason': 'irregular_or_unpaired_carriageway', 'paired_m': round(matched, 3), 'length_m': round(total, 3)})
            continue
        axis = axes[c['key']]
        keep = c['points'][-1][axis] >= c['points'][0][axis]
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
                      'reason': 'lane_pockets_simplified_to_chain_through_lanes'})
        shifted.update(n for f in c['features'] for n in f['node_ids'])
        if not keep:
            continue
        for f in c['features']:
            pts = f['points'][::f['properties']['oneway']]
            dense = list(samples(pts, POLICY['sample_m']))
            dense.append((pts[-1], dense[-1][1], 0))
            moved, local_sources = [], {f['id']}
            for p, u, _ in dense:
                hit = index.nearest(p, POLICY['carriageway_distance_m'],
                    lambda s: s[3][0]['id'] != c['id'] and s[3][0]['key'] == c['key']
                    and s[3][0]['id'] in active and s[3][1]['properties']['layer'] == f['properties']['layer']
                    and aligned(u, s[1], s[2], True))
                if hit:
                    local_sources.add(hit[3][0])
                q = tuple(round((p[j]+hit[2][j])/2, 3) for j in range(2)) if hit else tuple(p)
                if not moved or q != moved[-1]:
                    moved.append(q)
            g = deepcopy(f)
            g['points'] = simplify(moved, .5)
            g['node_ids'] = [f'median/{p[0]:.3f}/{p[1]:.3f}' for p in g['points']]
            g['source_ids'] = sorted(local_sources)
            g['properties']['oneway'] = 0
            m = mappings[f['id']]
            retained.append((g, {**m, 'oneway': 0, 'fwd_lanes': c['lanes'], 'bkw_lanes': opposing, **section}))
    median_index = SegmentIndex((f['id'], f['points'], f) for f, _ in retained)
    for f, m in roads:
        chain = by_source.get(f['id'])
        if chain and chain['id'] in active:
            continue
        for i, node in enumerate(f['node_ids']):
            if node in shifted:
                hit = median_index.nearest(f['points'][i], POLICY['carriageway_distance_m'])
                if hit:
                    q = tuple(round(v, 3) for v in hit[2])
                    f['points'][i], f['node_ids'][i] = q, f'median/{q[0]:.3f}/{q[1]:.3f}'
        retained.append((f, m))
    return sorted(retained, key=lambda r: r[0]['id']), audit


def simplify_prisma(roads, centre, decisions):
    """Surveyed four-arm simplification; source IDs make all removed slips auditable."""
    slips = {'5074383', '1366553666', '1366321596', '335739398', '1001374963', '1001374964', '359210529'}
    east_arm = {'335739979', '613918840'}
    audit, result = [], []
    node = 'junction/prisma'
    # A junction's arms need a shared straight throat before the surveyed
    # curves resume. Fit the main axis to the local midline; this also avoids
    # encoding the removed turn-pocket offsets as a kink inside the mouth.
    local = [p for f, _ in roads if f['properties']['name'] == 'Savilahdentie'
             for p in f['points'] if 45 <= distance(p, centre) <= 100]
    xx = sum((p[0]-centre[0])**2 for p in local)
    zz = sum((p[1]-centre[1])**2 for p in local)
    xz = sum((p[0]-centre[0])*(p[1]-centre[1]) for p in local)
    angle = .5*math.atan2(2*xz, xx-zz)
    axis = math.cos(angle), math.sin(angle)
    for f, m in roads:
        sid = f['id'].split('/')[-1]
        if sid in slips:
            decisions[f['id'], 'road'] = {**m, 'action': 'skip', 'reason': 'prisma_slip_road_simplified_slice_B'}
            audit.append({'source_id': f['id'], 'reason': 'prisma_slip_road_simplified_slice_B'})
            continue
        if sid in east_arm:
            m = {**m, 'oneway': 0, 'fwd_lanes': 1, 'bkw_lanes': 1, 'median_style': 3, 'median_width_m': 1.5}
        if f['properties']['name'] in ('Savilahdentie', 'Sarastuskaari') or sid in east_arm:
            pts, ids = [], []
            for p, n in zip(f['points'], f['node_ids']):
                if distance(p, centre) < 38:
                    p, n = centre, node
                elif f['properties']['name'] == 'Savilahdentie' and distance(p, centre) < 75:
                    station = (p[0]-centre[0])*axis[0]+(p[1]-centre[1])*axis[1]
                    p = tuple(round(centre[j]+station*axis[j], 3) for j in range(2))
                    n = f'median/{p[0]:.3f}/{p[1]:.3f}'
                if not pts or p != pts[-1]:
                    pts.append(p); ids.append(n)
            f['points'], f['node_ids'] = pts, ids
            if len(pts) < 2:
                audit.append({'source_id': f['id'], 'reason': 'prisma_internal_junction_segment_collapsed'})
                continue
        result.append((f, m))
    audit.append({'reason': 'prisma_shared_straight_throat', 'centre_xz': centre, 'radius_m': 75, 'axis_xz': axis})
    return result, audit
