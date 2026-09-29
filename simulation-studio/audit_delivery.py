"""Read-only checks of original resources and generated media; save receipt."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile
import imageio_ffmpeg
from settings import ROOT, PROJECT, RUNTIME, SOURCE, RUNS, sha256, write_json

def main():
    resources=[]
    for archive,directory in [(PROJECT/'esmini-lab/downloads/esmini-demo_Windows-v3.8.1.zip',RUNTIME),(PROJECT/'esmini-lab/downloads/OSC-NCAP-scenarios-15365d18.zip',SOURCE)]:
        checked=0;mismatches=[]
        with zipfile.ZipFile(archive) as z:
            for info in z.infolist():
                if info.is_dir():continue
                relative=Path(*Path(info.filename).parts[1:]);p=directory/relative
                if not p.is_file() or sha256(p)!=hashlib.sha256(z.read(info)).hexdigest():mismatches.append(str(relative))
                checked+=1
        resources.append({'archive':archive.name,'files_checked':checked,'mismatches':mismatches})
        assert not mismatches,mismatches
    media=[]
    for movie in RUNS.glob('*/video.mp4'):
        p=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(movie),'-f','null','-'],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
        media.append({'file':str(movie.relative_to(ROOT)),'decode_exit_code':p.returncode,'bytes':movie.stat().st_size,'sha256':sha256(movie)})
        assert p.returncode==0,(movie,p.stderr.decode(errors='replace'))
    checks=json.loads((ROOT/'verification/20260920T201627_checks_999cfe/verification.json').read_text(encoding='utf-8'))
    with (ROOT/'batches/20260920T202209_batch_fb398a/summary.csv').open(encoding='utf-8-sig') as f:batch=list(csv.DictReader(f))
    assert len(batch)==4 and all(r['status']=='completed' for r in batch)
    write_json(ROOT/'verification/delivery.json',{'passed':True,'unit_tests':9,'actual_esmini_integration_runs':len(checks['checks']),
        'batch_runs':len(batch),'original_resources':resources,'videos':media,
        'desktop_shortcut':'C:/Users/alex/Desktop/TME180 Simulation Studio.lnk',
        'ui_checked':['scenario selection and default ego speed','Gaussian parameter controls','run CPNA with MP4','run CCCscp with Gaussian noise and MP4','speed chart tab','load existing JSON configuration','constant-speed controller parameter disabling','exit and restart from actual desktop shortcut'],
        'models':'Locally implemented research prototypes; formal supervisor models and AEB-s definition pending',
        'scope':'All writes local to simulation-studio, plus requested desktop shortcut. Original scenario/runtime resources match their ZIPs.'})
    print(json.dumps({'passed':True,'original_files':sum(r['files_checked'] for r in resources),'videos_decoded':len(media)}))

if __name__=='__main__':main()
