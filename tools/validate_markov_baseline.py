"""Compare pipeline draws with a compiled copy of the original public C++ generator.

Only three named perception files are read from the preserved Git baseline.
No HiDrive controller or private build log is opened or hashed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pipeline.errors.frames import iid_frames, markov_frames

PREFIX = 'experiments/esmini-perception/simulation/acc_cpp/'
PROBE = r'''
#include "accsim/perception.hpp"
#include <iostream>
int main() {
  for (int id : {1,7}) for (int seed=0;seed<100;++seed) for(int family=0;family<4;++family) {
    double mean = family==1 ? 0.2 : family==3 ? 1.0 : 0.5;
    accsim::DetectionDropout iid(0.1,seed);
    accsim::MarkovDropout markov(0.1,mean,0.05,seed,{id},true);
    std::cout << id << '\t' << seed << '\t' << family << '\t';
    for(int i=0;i<200;++i) {
      accsim::PerceptionFrame frame;
      frame.sequence=i;frame.measurement_time_s=i*0.05;frame.delivery_time_s=frame.measurement_time_s;
      accsim::Observation target;target.track_id=id;frame.objects.push_back(target);
      const auto result=family==0 ? iid.apply(frame) : markov.apply(frame);
      std::cout << (result.dropped_ids.empty()?'0':'1');
    }
    std::cout << '\n';
  }
}
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',default='baseline/native-markov-20261006')
    parser.add_argument('--output',type=Path,default=ROOT/'output/validation/markov_cpp')
    parser.add_argument('--write-fixture',type=Path)
    args=parser.parse_args()
    folder=args.output.resolve();folder.mkdir(parents=True,exist_ok=True)
    names=['src/perception.cpp','include/accsim/perception.hpp','include/accsim/types.hpp']
    hashes={}
    for name in names:
        content=subprocess.check_output(['git','show',args.baseline+':'+PREFIX+name],cwd=ROOT)
        path=folder/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(content)
        hashes[name]=hashlib.sha256(content).hexdigest()
    source=folder/'probe.cpp';source.write_text(PROBE)
    compiler=shutil.which('clang++') or shutil.which('c++')
    if not compiler:raise SystemExit('A C++17 compiler is required to validate the original generator.')
    executable=folder/'probe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(folder/'include'),str(source),
                    str(folder/'src/perception.cpp'),'-o',str(executable)],check=True)
    rows=subprocess.check_output([str(executable)],text=True).splitlines()
    fixtures=[]
    for line in rows:
        ident,seed,family,bits=line.split('\t');ident,seed,family=map(int,[ident,seed,family])
        mean={0:None,1:0.2,2:0.5,3:1.0}[family]
        expected=bytes(int(bit) for bit in bits)
        actual=bytes(iid_frames(.1,seed,ident,200) if family==0 else markov_frames(.1,mean,.05,seed,ident,200))
        if actual!=expected:raise RuntimeError(f'Generator mismatch: target {ident}, seed {seed}, family {family}')
        fixtures.append({'target_id':ident,'seed':seed,'model':'iid' if family==0 else 'markov',
                         'mean_missing_s':mean,'sha256':hashlib.sha256(expected).hexdigest()})
    report={'baseline_commit':subprocess.check_output(['git','rev-parse',args.baseline],cwd=ROOT,text=True).strip(),
            'reference':'Actual compiled original C++ DetectionDropout / MarkovDropout',
            'public_source_sha256':hashes,'miss_probability':.1,'period_s':.05,'frames':200,
            'sequences_compared':len(fixtures),'frames_compared':len(fixtures)*200,'all_identical':True,
            'fixtures':fixtures}
    (folder/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.write_fixture:
        args.write_fixture.parent.mkdir(parents=True,exist_ok=True)
        args.write_fixture.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='fixtures'},indent=2))


if __name__=='__main__':main()
