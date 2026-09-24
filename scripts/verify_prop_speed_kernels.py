"""Check scalar compilation against the readable kernels for every prop graph."""
import importlib.util
import json
import random
from pathlib import Path
from em_backend import activate

assert activate() == 'compiled'
from propeller_model import blade_forces
from component_assembly import f32
from verify_propulsion_general import differences
from propulsion_general import step
from prop_steady import initial_state

ROOT = Path(__file__).resolve().parents[1]


def reference(name):
    spec = importlib.util.spec_from_file_location('reference_' + name, ROOT/'scripts'/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(report_path=None):
    polar = reference('polar_runtime')
    force = reference('polar_f32')
    blade = reference('propeller_model')
    blade.evaluate = polar.evaluate
    blade.calc_cl, blade.calc_cd = force.calc_cl, force.calc_cd
    prop = reference('propeller_general')
    prop.blade_forces = blade.blade_forces
    piston = reference('piston_general')
    owner = reference('propulsion_general')
    owner.propeller_step, owner.piston_step = prop.step, piston.step
    rng = random.Random(20260920)
    def v(a, b): return f32(rng.uniform(a, b))
    cases = graphs = owner_cases = 0
    for path in sorted((ROOT/'references/prop-propulsion').glob('*.json')):
        if path.name == 'manifest.json':
            continue
        properties = json.loads(path.read_text())['properties']
        props = properties['propellers']
        for prop in props:
            p = prop['properties']
            for i in range(24):
                args = [p, v(0., 500.), v(p['pitch_min']-.1, p['pitch_max']+.1),
                        v(-50., 400.), v(-40., 40.), v(-10., 10.), v(-1., 1.),
                        v(.1, 1.4), v(270., 350.), v(-5., 20.)]
                a, b = blade_forces(*args), blade.blade_forces(*args)
                assert not differences(a, b), (path.name, i, a, b)
                cases += 1
        for mode in ['automatic', 'optimized']:
            state = initial_state(properties, [100., 0., 0.], 3000., engine_control_mode=mode)
            for i in range(4):
                kw = dict(velocity=[v(40., 200.), v(-5., 5.), v(-3., 3.)],
                          height=v(0., 12000.), body_omega=[v(-.1, .1) for _ in range(3)],
                          dt=f32(1/[30,48,60,120][i]), nitro=10., seed=state['seed'])
                a, b = step(properties, state, **kw), owner.step(properties, state, **kw)
                assert not differences(a, b), (path.name, mode, i, differences(a, b))
                state = a
                owner_cases += 1
        graphs += 1
    out = Path(report_path) if report_path else ROOT/'analysis/prop-speed/kernel-validation.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(status='PASS', graphs=graphs, blade_cases=cases, owner_cases=owner_cases,
        scope='Exact compiled/readable polar, blade, propeller, piston and drivetrain equality on every pinned propulsion graph; both engine control modes.'), indent=2)+'\n')
    print('PASS', graphs, 'graphs', cases, 'blade cases', owner_cases, 'owner cases')


if __name__ == '__main__':
    main()
