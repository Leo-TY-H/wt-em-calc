"""Audit saved EM samples; optionally compute the user's default aircraft case."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
from urllib.request import urlretrieve
import zipfile

ROOT=Path(__file__).resolve().parents[1]
PHYSICAL=frozenset(('post-stall','control authority','wing force limit','IAS limit','Mach limit','VNE','sweep unavailable','flap automatic retraction','flap IAS limit','Flap IAS limit','Flap automatic IAS limit','Flap automatic Mach limit','Instructor pitch limit','reversed pitch response','propeller shaft stopped'))
NUMERICAL=frozenset(('trim did not converge','propulsion did not settle','propulsion cycle unresolved','vertical step did not close','pitch response unresolved','Instructor boundary unresolved'))
SEARCH=frozenset(('chart load search limit',))


def summarize(data):
    output=dict(denominator='Saved aircraft.points entries, including not-evaluated exclusions; adaptive samples are not uniformly distributed or independent solver attempts.',reason_counts='Each reason counts at most once per rejected point; multiple reasons can overlap.',settings=data.get('settings'),aircraft=[])
    for aircraft in data['aircraft']:
        points=aircraft.get('points',[]);rejected=[p for p in points if not p['valid']]
        counts=Counter(reason for p in rejected for reason in set(p.get('reasons',[])))
        categories=Counter()
        for p in rejected:
            reasons=set(p.get('reasons',[]));groups=set()
            if reasons&PHYSICAL:groups.add('physical')
            if reasons&NUMERICAL:groups.add('numerical')
            if reasons&SEARCH:groups.add('search')
            if reasons-(PHYSICAL|NUMERICAL|SEARCH) or not reasons:groups.add('unclassified')
            categories['+'.join(sorted(groups))]+=1
        n=len(points);r=len(rejected)
        examples={reason:[dict(speed_kmh=p.get('speed_kmh'),load_g=p.get('load_g'),converged=p.get('converged'),not_evaluated=p.get('not_evaluated',False),reasons=p.get('reasons'),force_error_g=p.get('force_error_g'),angular_error_rad_s2=p.get('angular_error_rad_s2')) for p in rejected if reason in p.get('reasons',[])][:3] for reason in counts}
        output['aircraft'].append(dict(id=aircraft.get('id',aircraft.get('name')),name=aircraft.get('name'),sample_count=n,rejected_count=r,rejection_percent=100*r/n if n else None,converged_rejected_count=sum(bool(p.get('converged')) for p in rejected),not_evaluated_rejected_count=sum(bool(p.get('not_evaluated')) for p in rejected),categories=dict(sorted(categories.items())),reasons={reason:dict(count=count,percent_of_all_samples=100*count/n if n else None,percent_of_rejected=100*count/r if r else None) for reason,count in counts.most_common()},numerical_boundary_count=len(aircraft.get('numerical_boundaries',[])),numerical_gap_count=len(aircraft.get('numerical_gaps',[])),examples=examples))
    return output


def download_data(directory):
    directory.mkdir(parents=True,exist_ok=True)
    base='https://github.com/Leo-TY-H/wt-em-calc/releases/download/v2026.09.30.2/'
    archive=directory/'neothunderism-windows-x64.zip';checksum=archive.with_suffix('.zip.sha256')
    urlretrieve(base+archive.name,archive);urlretrieve(base+checksum.name,checksum)
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest!=checksum.read_text().strip().split()[0]:raise ValueError('Release archive checksum mismatch')
    files=0
    with zipfile.ZipFile(archive) as bundle:
        for entry in bundle.infolist():
            parts=Path(entry.filename.replace('\\','/')).parts
            if 'references' not in parts or entry.is_dir():continue
            relative=Path(*parts[parts.index('references'):])
            target=(ROOT/relative).resolve()
            if not target.is_relative_to(ROOT/'references'):raise ValueError('Archive path outside references')
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(bundle.read(entry));files+=1
    return dict(release='v2026.09.30.2',archive_sha256=digest,reference_files=files)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path)
    parser.add_argument('--compute-defaults',action='store_true')
    parser.add_argument('--download-data',action='store_true')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'.qa/rejections')
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    provenance=download_data(args.output_dir) if args.download_data else None
    if args.compute_defaults:
        from em_solver import compute
        from aircraft_catalog import catalog
        identifiers=[]
        for requested in ('F_2A','saab_jas39c'):
            matches=[key for key in catalog() if key.casefold()==requested.casefold()]
            if len(matches)!=1:raise ValueError('Aircraft ID not found: '+requested)
            identifiers.append(matches[0])
        last=[0.]
        def progress(row):
            now=time.monotonic()
            if now-last[0]>15:
                print('EM_PROGRESS '+json.dumps(row),flush=True);last[0]=now
        data=compute(dict(aircraft=identifiers),progress=progress)
        import orjson
        (args.output_dir/'results.json').write_bytes(orjson.dumps(data,option=orjson.OPT_SERIALIZE_NUMPY))
    elif args.input:data=json.loads(args.input.read_text(encoding='utf-8'))
    else:parser.error('Choose --input or --compute-defaults')
    report=summarize(data);report['provenance']=provenance
    (args.output_dir/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('EM_REJECTION_SUMMARY '+json.dumps(report),flush=True)


if __name__=='__main__':main()
