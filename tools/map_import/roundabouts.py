# SPDX-License-Identifier: GPL-2.0-only

"""Recognize source motor rings without place names or surveyed way IDs.

Tagged rings retain their source authority. Untagged rings must be bounded,
compact, directed faces of the source graph, with at least three external arms.
A divided ordinary junction encloses a block too, but its angular rectangular
face fails the circularity contract. Parking/service circuits are never inferred.
The embedding visits each half-edge once: O(V + E log maximum degree), O(V+E)
scratch during offline preparation, with no cycle enumeration or gameplay work.
"""

from collections import defaultdict
import math

from .geometry import distance
from .mapping import POLICY, map_role


def detect_roundabouts(features):
    """Tag complete source ways forming qualifying faces; return inference evidence.

    A face containing only a section of a way is reported, not silently allowed
    to consume the rest of that way. Explicit tags are handled by the existing
    split-way ring assembler, including incomplete extracts and circular tags.
    """
    roads = [f for f in features if 'road' in f['roles'] and len(f['points']) > 1
             and (f['properties'].get('roundabout') or map_role(f, 'road')['action'] == 'road')]
    adjacency, positions, owners, directed = defaultdict(set), {}, defaultdict(set), set()
    lookup = {f['id']: f for f in roads}
    for f in roads:
        p = f['properties']
        if p.get('road_class') == 'service':
            continue
        positions.update(zip(f['node_ids'], f['points']))
        for a, b in zip(f['node_ids'], f['node_ids'][1:]):
            if a == b or distance(positions[a], positions[b]) < .01:
                continue
            adjacency[a].add(b)
            adjacency[b].add(a)
            owners[a, b].add(f['id'])
            owners[b, a].add(f['id'])
            if p.get('oneway'):
                directed.add((a, b) if p['oneway'] > 0 else (b, a))
    successors = {}
    for b, neighbors in adjacency.items():
        ns = sorted(neighbors, key=lambda a: (math.atan2(positions[a][1]-positions[b][1],
                                                       positions[a][0]-positions[b][0]), a))
        for i, a in enumerate(ns):
            successors[a, b] = b, ns[(i+1) % len(ns)]
    visited, audit = set(), []
    for seed in sorted(successors):
        if seed in visited:
            continue
        edge, edges = seed, []
        while edge not in visited:
            visited.add(edge)
            edges.append(edge)
            edge = successors[edge]
        nodes = [a for a, _ in edges]
        if edge != seed or len(nodes) < 3 or len(set(nodes)) != len(nodes):
            continue
        points = [positions[n] for n in nodes]
        lo = [min(p[j] for p in points) for j in (0, 1)]
        hi = [max(p[j] for p in points) for j in (0, 1)]
        diameter = max(hi[j]-lo[j] for j in (0, 1)) + 3.5
        # World z runs south. A Finnish counterclockwise motor ring has negative area.
        area = sum(positions[a][0]*positions[b][1]-positions[b][0]*positions[a][1]
                   for a, b in edges)/2
        if area >= 0 or diameter > POLICY['roundabout_max_diameter_m'] or any(e not in directed for e in edges):
            continue
        sources = sorted({s for e in edges for s in owners[e]})
        if all(lookup[s]['properties'].get('roundabout') for s in sources):
            continue
        perimeter = sum(distance(positions[a], positions[b]) for a, b in edges)
        circularity = 4*math.pi*abs(area)/perimeter**2
        if circularity < POLICY['roundabout_min_circularity']:
            continue
        arms = sum(len(adjacency[n])-2 for n in nodes)
        ring_edges = {frozenset(e) for e in edges}
        reason = ''
        if arms < 3:
            reason = 'turning_loop_fewer_than_three_external_arms'
        elif len({lookup[s]['properties'].get('layer', 0) for s in sources}) != 1:
            reason = 'ring_changes_source_layer'
        elif any(frozenset((a, b)) not in ring_edges for s in sources
                 for a, b in zip(lookup[s]['node_ids'], lookup[s]['node_ids'][1:])):
            reason = 'ring_is_only_part_of_source_way'
        evidence = {'source_ids': sources, 'centre_xz': [round((a+b)/2, 3) for a, b in zip(lo, hi)],
                    'diameter_m': round(diameter, 3), 'circularity': round(circularity, 6),
                    'external_source_arms': arms, 'authority': 'closed_oneway_motor_face',
                    'status': 'refused' if reason else 'detected', 'reason': reason}
        audit.append(evidence)
        if not reason:
            for s in sources:
                lookup[s]['properties'].update(roundabout=True, roundabout_authority='closed_oneway_motor_face')
    return audit
