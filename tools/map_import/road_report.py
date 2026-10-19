# SPDX-License-Identifier: GPL-2.0-only

"""Road/street accounting and local CSV handoff; refusals include every retry."""

from collections import Counter, defaultdict
import csv
import json
import sqlite3
from pathlib import Path
from .geometry import clip_line, length
from .mapping import map_role
from .render import render


def analyse(plan, outcome, features=(), preparation=None):
    records={r['id']:r for r in outcome['roads']}
    classes=defaultdict(lambda:{'prepared':0,'placed':0,'partial':0,'missing':0,'prepared_m':0.,'placed_m':0.})
    streets=defaultdict(list)
    for s in plan['strokes']:
        r=records.get(s['id'],{})
        row=classes[s['road_class']]
        row['prepared']+=1;row['prepared_m']+=s['length_m']
        state={'accepted':'placed','partial':'partial'}.get(r.get('status'),'missing')
        row[state]+=1
        row['placed_m']+=r.get('accepted_length_m',0)
        for name in s.get('names',[]):
            streets[name].append((s,r))
        if s['road_class']=='motorway_link':streets['E63 ramps'].append((s,r))
    named=[]
    for name,items in sorted(streets.items()):
        states=[r.get('status','unattempted') for _,r in items]
        state='placed' if all(s=='accepted' for s in states) else 'missing' if not any(s in ('accepted','partial') for s in states) else 'partly placed'
        reasons=Counter(p['reason'] for _,r in items for p in r.get('failed_pieces',[]))
        reasons.update('unattempted' for _, r in items if not r)
        reasons.update('lane_fit_after_refusal' for _, r in items for a in r.get('attempts', [])
                       if a.get('strategy') == 'lane_fit_after_refusal' and not a['reason'])
        named.append({'street':name,'status':state,'strokes':len(items),'prepared_km':round(sum(s['length_m'] for s,_ in items)/1000,3),'placed_km':round(sum(r.get('accepted_length_m',0) for _,r in items)/1000,3),'reasons':dict(sorted(reasons.items()))})
    # Include named eligible source roads lost before stroke construction, not
    # only names that happened to survive into the plan. Paths remain separate.
    known = {row['street'] for row in named}
    missing = defaultdict(list)
    for f in features:
        name = f['properties'].get('name')
        if not name or name in known or 'road' not in f['roles']:
            continue
        mapping = map_role(f, 'road')
        car_classes = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link',
                       'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'residential',
                       'service', 'living_street', 'unclassified', 'road', 'construction', 'proposed'}
        if f['properties'].get('road_class') not in car_classes:
            continue
        clipped = clip_line(f['points'], ['']*max(0, len(f['points'])-1), plan['target_box_xz'])
        if clipped:
            reason = mapping['reason'] if mapping['action'] == 'skip' else 'no_prepared_stroke'
            missing[name].append((f, sum(length(points) for points, _ in clipped), reason))
    for name, items in sorted(missing.items()):
        named.append({'street': name, 'status': 'missing', 'strokes': 0,
                      'prepared_km': 0., 'placed_km': 0.,
                      'reasons': dict(sorted(Counter(reason for _, _, reason in items).items()))})
    named.sort(key=lambda row: row['street'])
    return {'complete':outcome.get('complete',False),'saved':outcome.get('saved',False),
            'preparation_simplifications': (preparation or {}).get('conflation', {}),
            'by_class':dict(sorted(classes.items())),'named_streets':named,
            'named_status_counts':dict(Counter(s['status'] for s in named)),
            'car_components':outcome.get('car_components'),
            'lane_fit_pieces': [{'stroke_id': r['id'], 'fwd_lanes': a['fwd_lanes'], 'bkw_lanes': a['bkw_lanes'],
                                 'points_xz': a['points_xz']} for r in records.values() for a in r['attempts']
                                if a.get('strategy') == 'lane_fit_after_refusal' and not a['reason']],
            'repair_accepted': sum(r['status'] == 'accepted' for r in outcome.get('repairs', [])),
            'repair_refusals': dict(sorted(Counter(r['reason'] for r in outcome.get('repairs', []) if r['reason']).items())),
            'attempt_refusals':dict(sorted(Counter(a['reason'] for r in records.values() for a in r['attempts'] if a['reason']).items())),
            'remaining_refusals':dict(sorted(Counter(a['reason'] for r in records.values() for a in r.get('failed_pieces',[])).items())),
            'unplaced_strokes':[{'id':s['id'],'names':s.get('names',[]),'class':s['road_class'],'source_ids':s['source_ids'],'length_m':s['length_m'],'status':records.get(s['id'],{}).get('status','unattempted'),'reasons':dict(Counter(p['reason'] for p in records.get(s['id'],{}).get('failed_pieces',[])))} for s in plan['strokes'] if records.get(s['id'],{}).get('status')!='accepted']}


def save_stats(path):
    """Inspect canonical persisted IDs, without relying on positional snapping."""
    with sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True) as conn:
        columns = {r[1] for r in conn.execute('PRAGMA table_info(network_edges)')}
        median = 'median_style' if 'median_style' in columns else '0'
        edges = conn.execute(f'SELECT start_node,end_node,physical_length,{median} FROM network_edges WHERE fwd_lanes+bkw_lanes>0').fetchall()
        speed = conn.execute('SELECT speed_multiplier FROM time_state').fetchone()[0]
    parents = {n: n for row in edges for n in row[:2]}
    def root(n):
        while parents[n] != n:
            parents[n] = parents[parents[n]]
            n = parents[n]
        return n
    for a, b, _, _ in edges:
        parents[root(a)] = root(b)
    styles = defaultdict(lambda: {'edges': 0, 'km': 0.})
    for _, _, run, style in edges:
        styles[style]['edges'] += 1
        styles[style]['km'] += run/1000
    return {'car_edges': len(edges), 'car_nodes': len(parents),
            'car_km': sum(row[2] for row in edges)/1000,
            'car_components': len({root(n) for n in parents}),
            'median_styles': dict(sorted(styles.items())), 'saved_speed': speed}


def write(root, run):
    root,run=Path(root),Path(run)
    plan=json.loads((root/'plan.json').read_text());outcome=json.loads((run/'apply_results.json').read_text())
    from .cli import digest
    if outcome.get('plan_sha256') != digest(root/'plan.json'):
        raise ValueError('Road report outcome does not match the plan')
    neutral = root/'neutral.json'
    features = json.loads(neutral.read_text())['features'] if neutral.exists() else json.loads((root/'features.json').read_text())
    preparation = json.loads((root/'report.json').read_text()).get('preparation', {}) if (root/'report.json').exists() else {}
    report=analyse(plan,outcome,features,preparation)
    save_path = Path(outcome.get('save_path') or run/'city.sqlite')
    if outcome.get('saved') and save_path.exists():
        report['saved_network'] = save_stats(save_path)
        report['car_components'] = report['saved_network']['car_components']
    (run/'roads_report.json').write_text(json.dumps(report,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
    with (run/'streets.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['street','status','strokes','prepared_km','placed_km','reasons']);writer.writeheader();writer.writerows(report['named_streets'])
    shots = root/'shots'
    if (shots/'views.json').exists():
        render(shots,features,[p for r in outcome['roads'] for p in r['pieces']],prefix=run.name+'_placed')
    return report


if __name__=='__main__':
    import sys
    result=write(sys.argv[1],sys.argv[2]);print(json.dumps({k:v for k,v in result.items() if k not in ('unplaced_strokes','named_streets')},indent=2))
