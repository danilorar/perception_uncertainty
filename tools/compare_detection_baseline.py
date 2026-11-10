"""Compare the migrated 1203-run HiDrive grid with the preserved Studio measurements."""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]


def table(path):
    with path.open(encoding='utf-8-sig',newline='') as stream:return list(csv.DictReader(stream))


def number(value):return None if value in ('',None) else float(value)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grid',type=Path,required=True)
    parser.add_argument('--reference',type=Path,default=ROOT/'tests/data/hidrive_detection_20261103.csv')
    parser.add_argument('--output',type=Path,default=ROOT/'output/validation/pipeline_migration.json')
    args=parser.parse_args()
    old=table(args.reference)
    actual=table(args.grid/'results.csv')
    def old_key(row):return row['scenario'],row['group'],int(row['seed'])
    def new_key(row):
        group=row['group']
        if group=='no_error':group='ideal'
        elif group=='markov':group=f'markov_{float(row["errors.detection.miss_mean_s"]):g}s'
        return row['scenario'],group,int(row['seed'])
    expected={old_key(row):row for row in old if row['role']=='main'}
    found={new_key(row):row for row in actual}
    if len(found)!=len(actual) or set(found)!=set(expected):
        raise SystemExit('Run keys differ from the preserved baseline; comparison refused.')
    fields={'first_trigger_time_s':'first_trigger_time_s','first_brake_time_s':'first_brake_time_s',
            'minimum_gap_m':'minimum_gap_m','collision_relative_speed_mps':'relative_impact_speed_kph'}
    differences=[];maximum=defaultdict(float);groups=defaultdict(lambda:{'runs':0,'old_collisions':0,'new_collisions':0})
    for key,reference in expected.items():
        row=found[key]
        if row['status'] not in ('completed','completed_with_warnings'):
            differences.append({'run':key,'field':'status','actual':row['status']});continue
        old_hit=reference['collision'].lower()=='true';new_hit=row['collision'].lower()=='true'
        group=groups[key[:2]];group['runs']+=1;group['old_collisions']+=old_hit;group['new_collisions']+=new_hit
        if old_hit!=new_hit:differences.append({'run':key,'field':'collision','old':old_hit,'new':new_hit})
        for old_field,new_field in fields.items():
            before,after=number(reference[old_field]),number(row[new_field])
            if after is not None and old_field=='collision_relative_speed_mps':after/=3.6
            if before is None or after is None:
                if before!=after:differences.append({'run':key,'field':old_field,'old':before,'new':after})
            else:
                delta=abs(before-after);maximum[old_field]=max(maximum[old_field],delta)
                if delta>1e-9:differences.append({'run':key,'field':old_field,'old':before,'new':after,'absolute_difference':delta})
    commits=sorted({row['commit'] for row in actual})
    dirty=sorted({int(row['local_changes']) for row in actual})
    report={'reference':str(args.reference.resolve()),'controller':'HiDrive',
            'runs_compared':len(expected),'all_metrics_match':not differences,
            'absolute_tolerance':1e-9,'maximum_absolute_difference':dict(maximum),
            'executed_commits':commits,'local_change_counts':dirty,
            'groups':[{'scenario':key[0],'group':key[1],**value} for key,value in groups.items()],
            'differences':differences}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('groups','differences')},indent=2))
    if differences:raise SystemExit(1)


if __name__=='__main__':main()
