# SPDX-License-Identifier: GPL-2.0-only

"""Road/street accounting and local CSV handoff; refusals include every retry."""

from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
from .geometry import length
from .render import render


def analyse(plan, outcome):
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
        named.append({'street':name,'status':state,'strokes':len(items),'prepared_km':round(sum(s['length_m'] for s,_ in items)/1000,3),'placed_km':round(sum(r.get('accepted_length_m',0) for _,r in items)/1000,3),'reasons':dict(sorted(reasons.items()))})
    return {'complete':outcome.get('complete',False),'saved':outcome.get('saved',False),
            'by_class':dict(sorted(classes.items())),'named_streets':named,
            'named_status_counts':dict(Counter(s['status'] for s in named)),
            'car_components':outcome.get('car_components'),
            'attempt_refusals':dict(sorted(Counter(a['reason'] for r in records.values() for a in r['attempts'] if a['reason']).items())),
            'remaining_refusals':dict(sorted(Counter(a['reason'] for r in records.values() for a in r.get('failed_pieces',[])).items())),
            'unplaced_strokes':[{'id':s['id'],'names':s.get('names',[]),'class':s['road_class'],'source_ids':s['source_ids'],'length_m':s['length_m'],'status':records.get(s['id'],{}).get('status','unattempted'),'reasons':dict(Counter(p['reason'] for p in records.get(s['id'],{}).get('failed_pieces',[])))} for s in plan['strokes'] if records.get(s['id'],{}).get('status')!='accepted']}


def write(root, run):
    root,run=Path(root),Path(run)
    plan=json.loads((root/'plan.json').read_text());outcome=json.loads((run/'apply_results.json').read_text())
    report=analyse(plan,outcome)
    (run/'roads_report.json').write_text(json.dumps(report,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
    with (run/'streets.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['street','status','strokes','prepared_km','placed_km','reasons']);writer.writeheader();writer.writerows(report['named_streets'])
    features=json.loads((root/'features.json').read_text())
    render(root/'shots',features,[p for r in outcome['roads'] for p in r['pieces']],prefix=run.name+'_placed')
    return report


if __name__=='__main__':
    import sys
    result=write(sys.argv[1],sys.argv[2]);print(json.dumps({k:v for k,v in result.items() if k not in ('unplaced_strokes','named_streets')},indent=2))
