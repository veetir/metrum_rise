# SPDX-License-Identifier: GPL-2.0-only

"""Keep unnamed service circulation out of public through-road reconstruction.

Source parking aisles and driveways are already excluded by mapping. Untagged
service components which leave and return to just one named public street are
local circulation, not another through street. Preserve single-access spurs,
named service streets and links between different public streets. Incidence and
component traversal are O(source vertices + roads), offline only.
"""

from collections import defaultdict


def filter_service_circulation(roads, decisions):
    services = {f['id']: (f, m) for f, m in roads if m['action'] == 'road'
                and not f['properties'].get('bus_only')
                and f['properties']['road_class'] == 'service' and not f['properties'].get('name')}
    incidence, public = defaultdict(set), defaultdict(set)
    for f, m in roads:
        if m['action'] != 'road':
            continue
        if f['id'] in services:
            for n in f['node_ids']:
                incidence[n].add(f['id'])
        else:
            identity = f['properties'].get('name') or f['properties'].get('ref') or f['id']
            for n in f['node_ids']:
                public[n].add(identity)
    visited, excluded, audit = set(), set(), []
    for seed in sorted(services):
        if seed in visited:
            continue
        pending, component, attachments = [seed], [], {}
        visited.add(seed)
        while pending:
            current = pending.pop()
            component.append(current)
            for n in services[current][0]['node_ids']:
                if public[n]:
                    attachments[n] = public[n]
                for other in sorted(incidence[n]):
                    if other not in visited:
                        visited.add(other)
                        pending.append(other)
        streets = {s for names in attachments.values() for s in names}
        if len(attachments) < 2 or len(streets) != 1:
            continue
        for sid in component:
            decisions[sid, 'road'] = {**services[sid][1], 'action': 'skip', 'reason': 'local_service_circulation'}
        excluded.update(component)
        audit.append({'source_ids': sorted(component), 'reason': 'local_service_circulation',
                      'through_street': next(iter(streets)), 'source_mouths': sorted(attachments)})
    return [(f, m) for f, m in roads if f['id'] not in excluded], audit
