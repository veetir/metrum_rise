# SPDX-License-Identifier: GPL-2.0-only

"""Deterministic feature accounting; unrun apply results are null, never successes."""

from collections import Counter, defaultdict


def build_report(features, decisions, plan, preparation, outcome=None):
    road_links, zone_links = defaultdict(list), defaultdict(list)
    for s in plan['strokes']:
        for source in s['source_ids']:
            road_links[source].append(s['id'])
    for zone in plan['zoning']:
        zone_links[zone['source_id']].append(zone['id'])
    discarded = {source: item['reason'] for item in preparation['discarded'] for source in item['source_ids']}
    records = []
    by_class, skipped = defaultdict(Counter), Counter()
    road_results = {r['id']: r for r in outcome['roads']} if outcome else {}
    zone_results = {r['id']: r for r in outcome['zoning']} if outcome else {}
    for f in features:
        for role in f['roles']:
            mapping = decisions[(f['id'], role)]
            action = mapping['action']
            links = road_links[f['id']] if role == 'road' else zone_links[f['id']] if role in ('building', 'landuse') else []
            reason = mapping['reason']
            if action == 'retain':
                status = 'retained'
            elif action == 'skip':
                status = 'skipped'
            elif not links:
                status = 'skipped'
                reason = discarded.get(f['id'], 'outside_target' if role == 'road' else 'no_frontage_intent')
            elif not outcome:
                status = 'prepared'
            else:
                results = road_results if role == 'road' else zone_results
                states = {results[i]['status'] for i in links if i in results}
                if len([i for i in links if i in results]) != len(links):
                    states.add('unattempted')
                status = next(iter(states)) if len(states) == 1 else 'partial'
            source_class = (f['properties']['road_class'] if role == 'road' else
                            f['properties']['building_use'] if role == 'building' else
                            f['properties']['landuse'] if role == 'landuse' else role)
            records.append({'id': f['id'], 'role': role, 'class': source_class, 'status': status,
                            'reason': reason, 'products': links, 'issues': f['issues'],
                            'outcomes': [(road_results if role == 'road' else zone_results)[i]
                                         for i in links if i in (road_results if role == 'road' else zone_results)]})
            by_class[f'{role}:{source_class}'][status] += 1
            if status == 'skipped':
                skipped[reason] += 1
    totals = {}
    for kind in ('road', 'footpath'):
        ids = {s['id'] for s in plan['strokes'] if s['kind'] == kind}
        lengths = {s['id']: s['length_m'] for s in plan['strokes'] if s['kind'] == kind}
        results = [r for r in road_results.values() if r['id'] in ids]
        totals[kind] = {'prepared': len(ids), 'prepared_length_m': round(sum(s['length_m'] for s in plan['strokes'] if s['kind'] == kind), 3),
                        'attempted': len(results), 'accepted': sum(r['status'] == 'accepted' for r in results) if outcome else None,
                        'partial': sum(r['status'] == 'partial' for r in results) if outcome else None,
                        'accepted_length_m': round(sum(r.get('accepted_length_m', lengths[r['id']] if r['status'] == 'accepted' else 0) for r in results), 3) if outcome else None,
                        'attempt_refusals_by_reason': dict(sorted(Counter(a['reason'] for r in results for a in r.get('attempts', []) if a['reason'] and a.get('by') != 'importer').items())),
                        'importer_retries_by_reason': dict(sorted(Counter(a['reason'] for r in results for a in r.get('attempts', []) if a['reason'] and a.get('by') == 'importer').items())),
                        'failed_pieces_by_reason': dict(sorted(Counter(a['reason'] for r in results for a in r.get('failed_pieces', [])).items())),
                        'refused': sum(r['status'] == 'refused' for r in results) if outcome else None,
                        'refused_by_reason': dict(sorted(Counter(r['reason'] for r in results if r['status'] == 'refused').items())),
                        'unattempted': len(ids)-len(results)}
    totals['zoning'] = {'prepared_by_type': dict(sorted(Counter(z['zone_type'] for z in plan['zoning']).items())),
                        'nominal_prepared_area_m2': dict(sorted(_sum_areas(plan['zoning'], 'zone_type', 'nominal_area_m2').items())),
                        'attempted': len(zone_results), 'outcomes': dict(sorted(Counter(r['status'] for r in zone_results.values()).items())),
                        'committed_parcels_by_type': dict(sorted(Counter(p['zone_type'] for p in outcome['parcels']).items())) if outcome else None,
                        'committed_area_m2': _sum_areas(outcome['parcels'], 'zone_type', 'area_m2') if outcome else None}
    return {'schema_version': 1, 'apply_status': 'completed' if outcome and outcome.get('complete') else 'incomplete' if outcome else 'not_run',
            'save_path': outcome.get('save_path') if outcome and outcome.get('saved') else None,
            'source_sha256': plan['source_sha256'], 'world_sha256': plan['world_sha256'],
            'world_transform': plan['world_transform'], 'target_box_xz': plan['target_box_xz'],
            'totals': totals, 'skipped_by_reason': dict(sorted(skipped.items())),
            'totals_per_class': {k: dict(sorted(v.items())) for k, v in sorted(by_class.items())},
            'preparation': preparation, 'features': records,
            'apply_results': outcome}


def _sum_areas(items, key, value):
    result = defaultdict(float)
    for item in items:
        result[item[key]] += item[value]
    return {k: round(v, 3) for k, v in sorted(result.items())}


def summary(report):
    lines = [f"Map import: apply {report['apply_status']}; save: {report['save_path'] or 'not created'}"]
    for kind in ('road', 'footpath'):
        t = report['totals'][kind]
        lines.append(f"{kind}: {t['prepared']} prepared ({t['prepared_length_m']:.1f} m), {t['attempted']} attempted, "
                     f"accepted={t['accepted']}, partial={t['partial']}, refused={t['refused']}; {t['unattempted']} unattempted; accepted metres={t['accepted_length_m']}")
        for reason, count in t['refused_by_reason'].items():
            lines.append(f"  unplaced stroke reason {reason}: {count}")
    z = report['totals']['zoning']
    lines.append(f"Zoning intents: {z['prepared_by_type']}; nominal area m2: {z['nominal_prepared_area_m2']}")
    lines.append(f"Committed parcels: {z['committed_parcels_by_type']}; actual area m2: {z['committed_area_m2']}")
    lines.append('Skipped feature roles: ' + ', '.join(f'{k}={v}' for k, v in report['skipped_by_reason'].items()))
    lines.append(f"Input junction warnings (not game refusals): {len(report['preparation']['junctions'])}")
    lines.append(f"Input radius warnings (not game refusals): {len(report['preparation'].get('radius_warnings', []))}")
    if report['apply_results']:
        lines.append(f"Connected car components (game CCH log): {report['apply_results'].get('car_components', 'unknown')}")
        lines.append(f"Forest clearing: {report['apply_results'].get('forest', {})}")
        lines.append(f"Population: {report['apply_results'].get('population', 'unknown')}; buildings: {report['apply_results'].get('buildings', 'unknown')}; saved speed: {report['apply_results'].get('saved_speed', 'unknown')}")
        lines.append(f"Growth: {report['apply_results'].get('growth', {})}")
        for kind in ('road', 'footpath'):
            lines.append(f"{kind} attempt refusals: {report['totals'][kind]['attempt_refusals_by_reason']}; failed pieces: {report['totals'][kind]['failed_pieces_by_reason']}")
            lines.append(f"{kind} importer decisions: {report['totals'][kind]['importer_retries_by_reason']}")
    return '\n'.join(lines) + '\n'
