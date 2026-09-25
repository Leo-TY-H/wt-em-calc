"""Real E-M API/Unicode/browser benchmarks. No flight telemetry or maneuver replay."""
import argparse
import csv
import io
import json
from pathlib import Path
import time
from urllib.request import Request,urlopen

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',default='http://127.0.0.1:8767')
    parser.add_argument('--cases',default='f16v,f16xl,j6k1-flaps30,comparison')
    parser.add_argument('--out',type=Path,default=ROOT/'outputs/validation')
    parser.add_argument('--require-no-gaps',action='store_true')
    args=parser.parse_args();out=args.out;out.mkdir(parents=True,exist_ok=True)
    from instructor_steady_aoa import REVISION
    def request(path,data=None,raw=False):
        req=Request(args.base+path,data=None if data is None else json.dumps(data).encode('utf-8'),
                    headers={'Content-Type':'application/json'})
        with urlopen(req,timeout=120) as response:content=response.read()
        return content if raw else json.loads(content)
    original={'aircraft': ['f_16a_block_72v_china'], 'altitude_m': 0.0, 'fuel_percent': 30.0, 'throttle': 1.1, 'afterburner': True, 'torque_gyro': False, 'engine_control_mode': 'automatic', 'trim_mode': 'optimized', 'trim_limit': 1.0, 'fixed_trim': [0.0, 0.0, 0.0], 'extra_mass_kg': 0.0, 'speed_min_kmh': 100.0, 'speed_max_kmh': 1300.0, 'max_load_g': None, 'speed_samples': 9, 'load_samples': 9, 'structural_limits': True, 'timestep_hz': 48.0, 'sampling': 'adaptive', 'sep_tolerance_mps': 0.5, 'surface_resolution': 601, 'sweep_percent': 0.0, 'flaps_percent': 0.0, 'instructor': True, 'aircraft_settings': {}, 'compare_instructor': False, 'entries': [{'id': 'entry_1', 'aircraft_id': 'f_16a_block_72v_china', 'settings': {'afterburner': True, 'altitude_m': 0.0, 'engine_control_mode': 'automatic', 'extra_mass_kg': 0.0, 'fixed_trim': [0.0, 0.0, 0.0], 'flaps_percent': 0.0, 'fuel_percent': 30.0, 'instructor': True, 'structural_limits': True, 'sweep_percent': 0.0, 'throttle': 1.1, 'timestep_hz': 48.0, 'torque_gyro': False, 'trim_limit': 1.0, 'trim_mode': 'optimized'}}]}
    cases=[('f16v',original),
           ('f16xl-full',dict(original,aircraft=['f_16xl'],entries=None,aircraft_settings={},speed_min_kmh=150.,speed_max_kmh=1400.)),
           ('fa18e-full',dict(original,aircraft=['fa_18e_block_2'],entries=None,aircraft_settings={},speed_min_kmh=150.,speed_max_kmh=1300.)),
           ('fa18e-flaps30',dict(original,aircraft=['fa_18e_block_2'],entries=None,aircraft_settings={},flaps_percent=30.,speed_min_kmh=150.,speed_max_kmh=900.)),
           ('fa18e-altitude',dict(original,aircraft=['fa_18e_block_2'],entries=None,aircraft_settings={},altitude_m=2000.,fuel_percent=70.,speed_min_kmh=150.,speed_max_kmh=1300.)),
           ('fa18e',dict(original,aircraft=['fa_18e_block_2'],entries=None,aircraft_settings={},speed_min_kmh=150.,speed_max_kmh=1000.)),
           ('f16xl',dict(original,aircraft=['f_16xl'],entries=None,aircraft_settings={},speed_min_kmh=150.,speed_max_kmh=900.)),
           ('j6k1-flaps30',dict(original,aircraft=['j6k1'],entries=None,aircraft_settings={},flaps_percent=30.,speed_min_kmh=150.,speed_max_kmh=700.)),
           ('comparison',{'aircraft': ['f_16a_block_72v_china', 'f_16xl'], 'altitude_m': 0.0, 'fuel_percent': 30.0, 'throttle': 1.1, 'afterburner': True, 'torque_gyro': False, 'engine_control_mode': 'automatic', 'trim_mode': 'optimized', 'trim_limit': 1.0, 'fixed_trim': [0.0, 0.0, 0.0], 'extra_mass_kg': 0.0, 'speed_min_kmh': 100.0, 'speed_max_kmh': 1300.0, 'max_load_g': None, 'speed_samples': 9, 'load_samples': 9, 'structural_limits': True, 'timestep_hz': 48.0, 'sampling': 'adaptive', 'sep_tolerance_mps': 0.5, 'surface_resolution': 601, 'sweep_percent': 0.0, 'flaps_percent': 0.0, 'instructor': True, 'aircraft_settings': {}, 'compare_instructor': False, 'entries': [{'id': 'entry_1', 'aircraft_id': 'f_16a_block_72v_china', 'settings': {'afterburner': True, 'altitude_m': 0.0, 'engine_control_mode': 'automatic', 'extra_mass_kg': 0.0, 'fixed_trim': [0.0, 0.0, 0.0], 'flaps_percent': 0.0, 'fuel_percent': 30.0, 'instructor': True, 'structural_limits': True, 'sweep_percent': 0.0, 'throttle': 1.1, 'timestep_hz': 48.0, 'torque_gyro': False, 'trim_limit': 1.0, 'trim_mode': 'optimized'}}, {'id': 'entry_2', 'aircraft_id': 'f_16xl', 'settings': {'afterburner': True, 'altitude_m': 0.0, 'engine_control_mode': 'automatic', 'extra_mass_kg': 0.0, 'fixed_trim': [0.0, 0.0, 0.0], 'flaps_percent': 0.0, 'fuel_percent': 30.0, 'instructor': True, 'structural_limits': True, 'sweep_percent': 0.0, 'throttle': 1.1, 'timestep_hz': 48.0, 'torque_gyro': False, 'trim_limit': 1.0, 'trim_mode': 'optimized'}}]})]
    cases.extend((label, dict(original, aircraft=[name], entries=None, aircraft_settings={}))
                 for label, name in (('j10a', 'j_10a'), ('jas39c', 'saab_jas39c'),
                                     ('ef2000', 'ef_2000_block_10')))
    cases=[case for case in cases if case[0] in args.cases.split(',')]
    reports=[]
    for label,config in cases:
        submitted=time.monotonic();key=request('/api/jobs',config)['id'];start=time.monotonic();last=0
        submission_seconds=start-submitted
        while True:
            job=request('/api/jobs/'+key)
            if job['status'] in ('error','cancelled'):raise AssertionError(job)
            if job['status']=='complete':break
            if time.monotonic()-last>15:
                print(label,job['status'],job.get('first_preview_s'),job.get('progress'),flush=True);last=time.monotonic()
            if time.monotonic()-start>300:raise AssertionError('API timeout')
            time.sleep(.5)
        data=request(f'/api/jobs/{key}/data.json');chart=request(f'/api/jobs/{key}/chart.json')
        checks=[]
        for aircraft in data['aircraft']:
            assert aircraft['instructor_approximation']['kind']=='steady AoA schedule'
            assert 'continuous_pull_method' not in aircraft
            accepted=[p for p in aircraft['points'] if p['valid']]
            assert accepted
            assert all(p['converged'] and p['instructor']['history_independent'] for p in accepted)
            assert all(p['instructor']['static_trim']['success'] or
                       p['instructor']['control_authority_trim_independent'] for p in accepted)
            assert all(p['instructor']['model_revision']==REVISION for p in accepted)
            assert all(p['instructor']['margin']>=-2e-7 for p in accepted)
            assert all(p['stall_margin_deg']>=0. for p in accepted)
            assert all(p['force_error_g']<=2e-4 and p['angular_error_rad_s2']<=5e-5 for p in accepted)
            edge=[c['boundary'] for c in aircraft['boundary_columns'] if c['boundary_status']=='verified limit']
            assert edge and all(p['valid'] for p in edge)
            unresolved=[c['speed_kmh'] for c in aircraft['boundary_columns'] if 'unresolved' in c['boundary_status']]
            if args.require_no_gaps:
                assert not unresolved,(label,'unresolved boundary columns',unresolved)
                assert not aircraft['interpolation']['unresolved_speed_intervals'],(label,aircraft['interpolation']['unresolved_speed_intervals'])
                assert not aircraft['numerical_gaps'],(label,'numerical plot gaps')
                assert not aircraft['numerical_boundaries'],(label,'unresolved plot boundaries')
                for column in aircraft['columns']:
                    assert not column.get('unresolved_load_intervals'),(label,column['speed_kmh'],'load intervals')
                    assert not column.get('interior_failures'),(label,column['speed_kmh'],'interior failures')
                    assert not column.get('numerical_gap_brackets'),(label,column['speed_kmh'],'gap brackets')
            checks.append(dict(aircraft=aircraft['id'],accepted_points=len(accepted),verified_boundary_points=len(edge),
                unresolved_boundary_speeds_kmh=unresolved,
                force_error_max_g=max(p['force_error_g'] for p in accepted),
                angular_error_max_rad_s2=max(p['angular_error_rad_s2'] for p in accepted),
                unresolved_intervals=aircraft['interpolation']['unresolved_speed_intervals'],
                peak_turn_dps=max(p['turn_dps'] for p in edge)))
        content=request(f'/api/jobs/{key}/samples.csv',raw=True).decode('utf-8')
        rows=list(csv.DictReader(io.StringIO(content)))
        if label=='f16v':assert '\u2417' in content and any('\u2417' in r['aircraft_name'] for r in rows)
        assert request('/api/jobs',config)['id']==key
        png=request(f'/api/jobs/{key}/diagram.png',raw=True);assert png.startswith(b'\x89PNG')
        (out/(label+'.png')).write_bytes(png)
        reports.append(dict(case=label,job=key,settings=data['settings'],solve_seconds=data['elapsed_s'],
            cache_hit=job.get('cached',False),submission_seconds=submission_seconds,
            equations_fingerprint=data.get('equations_fingerprint'),
            first_preview_seconds=job.get('first_preview_s'),complete_seconds=job.get('elapsed_s'),checks=checks))
        print('PASS',label,reports[-1]['solve_seconds'],checks,flush=True)
    errors=[]
    from playwright.sync_api import sync_playwright
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(channel='chrome',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1100})
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(args.base,wait_until='networkidle')
        page.wait_for_function('state.meta!==null')
        for report in reports:
            page.evaluate('(id)=>loadData(id,true)',report['job'])
            page.wait_for_function('(id)=>state.job===id || state.data?.aircraft?.[0]?.instructor_approximation?.kind==="steady AoA schedule"',arg=report['job'])
            page.wait_for_timeout(700)
            assert page.locator('#stale').is_hidden()
            assert page.evaluate("""() => {const a=state.data.aircraft[0];return document.getElementById('chart').data.some(t=>t.mode==='lines'&&t.x?.length===a.boundary.length&&t.x.every((x,i)=>x===a.boundary[i].speed_kmh));}""")
            page.screenshot(path=str(out/(report['case']+'-browser.png')),full_page=True)
        browser.close()
    assert not errors,errors
    report=dict(passed=True,require_no_gaps=args.require_no_gaps,scope='Steady approximation: numerical constraints, actual API, UTF-8 exports and browser plotting; live-game response not certified',
        data_version={k:v for k,v in json.loads((ROOT/'references/data-version.json').read_bytes()).items() if k!='files'},
        cases=reports,browser_errors=errors)
    (out/'validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('ALL PASS',out,flush=True)


if __name__=='__main__':main()
