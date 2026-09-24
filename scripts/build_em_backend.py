"""Compile the existing FM ports without changing float rounding/grouping.

Source is generated from the readable Python reference. Scalar declarations
avoid boxed arithmetic while preserving explicit float32 helpers; fast-math
and fused multiply-add contraction are disabled.
"""
import argparse
import ast
import json
import os
import re
from pathlib import Path

from em_backend import ROOT,DIRECTORY,MODULES,signature

COMPILE_ARGS = ['/O2', '/fp:strict'] if os.name == 'nt' else ['-O3', '-ffp-contract=off', '-fno-fast-math']

HELPERS='''
cpdef inline double f32(double x):
    return <float>x
cpdef inline double add(double a, double b):
    return <float>(a+b)
cpdef inline double sub(double a, double b):
    return <float>(a-b)
cpdef inline double mul(double a, double b):
    return <float>(a*b)
'''

MATH_HELPERS = """
from libc.math cimport sin as _c_sin, cos as _c_cos, tan as _c_tan, sqrt as _c_sqrt
from libc.math cimport asin as _c_asin, acos as _c_acos, atan as _c_atan, atan2 as _c_atan2, isinf as _c_isinf
cdef inline double _math_sqrt(double x) except *:
    if x < 0.: raise ValueError('math domain error')
    return _c_sqrt(x)
cdef inline double _math_sin(double x) except *:
    if _c_isinf(x): raise ValueError('math domain error')
    return _c_sin(x)
cdef inline double _math_cos(double x) except *:
    if _c_isinf(x): raise ValueError('math domain error')
    return _c_cos(x)
cdef inline double _math_tan(double x) except *:
    if _c_isinf(x): raise ValueError('math domain error')
    return _c_tan(x)
cdef inline double _math_asin(double x) except *:
    if x < -1. or x > 1.: raise ValueError('math domain error')
    return _c_asin(x)
cdef inline double _math_acos(double x) except *:
    if x < -1. or x > 1.: raise ValueError('math domain error')
    return _c_acos(x)
cdef inline double _math_atan(double x):
    return _c_atan(x)
cdef inline double _math_atan2(double y, double x):
    return _c_atan2(y,x)
"""

# Scalar temporaries otherwise remain boxed Python floats even in Cython.
# These declarations preserve the readable equations and every explicit f32
# rounding operation. Keep Python math calls and disable contraction/fast-math.
SCALARS={
    'piston_model':{
        'div(a, b)':('cpdef double div(double a, double b)', ''),
        'pressure_at_height(height, pressure0=101300.0, ceiling=18300.0)':
            ('cpdef double pressure_at_height(double height, double pressure0=101300.0, double ceiling=18300.0)', 'h, p, c'),
        'inlet_pressure(height, body_u, recovery)':
            ('cpdef double inlet_pressure(double height, double body_u, double recovery)', 'rho, ram'),
        'rpm_torque(p, omega, throttle, torque_multiplier=1.0, afterburner=False, gear=0, nitro=0.0)':
            ('def rpm_torque(p, double omega, double throttle, double torque_multiplier=1.0, afterburner=False, gear=0, double nitro=0.0)',
             'effective, wref, x, shape, scale, torque, tboost'),
        'mixture(p, inlet, command, *, rich_accumulator=0.0, automatic=False)':
            ('def mixture(p, double inlet, double command, *, double rich_accumulator=0.0, automatic=False)',
             'pressure, accum, supplied, limit, factor, inv'),
    },
    'propeller_step':{
        'sign(x)':('cpdef inline double sign(double x)', ''),
        'clamp(x, lo, hi)':('cpdef inline double clamp(double x, double lo, double hi)', ''),
        'tiny(x)':('cpdef inline double tiny(double x)', ''),
    },
    'structural_limits':{
        'interval(x, x0, y0, x1, y1)':
            ('cpdef inline double interval(double x, double x0, double y0, double x1, double y1)', ''),
    },
    'piston_compressor':{
        'speed_factor(p, omega)':('cpdef double speed_factor(p, double omega)', 'x'),
        'requested_pressure(p, throttle, height)':('cpdef double requested_pressure(p, double throttle, double height)', ''),
        'low_rpm(manifold, omega, inlet)':('cpdef double low_rpm(double manifold, double omega, double inlet)', ''),
        'candidate(p, s, i, speed, inlet, requested, throttle, afterburner, nitro)':
            ('def candidate(p, s, int i, double speed, double inlet, double requested, double throttle, afterburner, double nitro)',
             'pb, flow, critical, flat, ceiling, line, scale, baseline, shape, x, potential, score, offset'),
        'step(p, omega, throttle, inlet, dt, *, gear=0, old_gear=0, regulator=-1.0, afterburner=False, nitro=0.0, turbo=0.0, turbo_command=1.0, automatic_turbo=True, assisted=False, height=0.0)':
            ('def step(p, double omega, double throttle, double inlet, double dt, *, gear=0, old_gear=0, double regulator=-1.0, afterburner=False, double nitro=0.0, double turbo=0.0, double turbo_command=1.0, automatic_turbo=True, assisted=False, double height=0.0)',
             'requested, speed, shape, base, upper, allowed, target, fraction, potential, top, ratio, manifold, line, factor, best, gain'),
    },
    'engine_supply':{
        'mechanical_multiplier(p, omega, health, cylinders, previous, extra, torque, friction, dt, seed, disabled=False, enabled=True)':
            ('def mechanical_multiplier(p, double omega, double health, cylinders, double previous, double extra, double torque, double friction, double dt, seed, disabled=False, enabled=True)',
             'rate, amplitude, loss, damage, shaft, a, b, c, u, threshold, value'),
    },
    'polar_f32':{
        'sin(x)':('cpdef inline double sin(double x)', ''),
        'div(a, b)':('cpdef inline double div(double a, double b)', ''),
        'calc_cl(p, a)':('cpdef double calc_cl(p, double a)',
            's, crit, cy, after, da, x, maxang, sa, pa, h, den, coeff, local_sign, local_angle, one, two, correction, wave'),
        'calc_cd(p, a)':('cpdef double calc_cd(p, double a)', 'line, delta, cd, bound'),
        'calc_c(p, a, angle, cl_add=0.0, cd_coeff=1.0)':
            ('def calc_c(p, double a, double angle, double cl_add=0.0, double cd_coeff=1.0)', 'cd, cl, radians, sn, cs'),
    },
    'polar_runtime':{
        'mach_value(runtime, mach, index)':('cpdef double mach_value(runtime, double mach, int index)',
            'a, b, high, slope, limit, c0, c1, c2, c3, value'),
        'evaluate(runtime, mach, cy_mult=1.0)':('def evaluate(runtime, double mach, double cy_mult=1.0)',
            'lam, slope, parab, decline, maxdist, cdafter, clafterl, clafterh, cl0, ah, al, ch, cl, cd, span, area, '
            'ind, focus, cm0, cm1, kq, clkq, original_cl0, mm, value, effective_slope, inv, dh, dl, lineh, low, linel, highden, lowden, ph, pl'),
    },
    'propeller_model':{
        'blade_forces(p, omega, pitch, axial, tangential, crossflow, axial_gradient, density, sound_speed, angular_flow=0.0)':
            ('def blade_forces(p, double omega, double pitch, double axial, double tangential, double crossflow, '
             'double axial_gradient, double density, double sound_speed, double angular_flow=0.0)',
             'thrust, torque, area_unit, half_density, cross_sq, station, twist, width, r, u, v, phi, alpha, vt, speed_sq, speed, force_unit, lift, drag, inv_speed, un, vn'),
    },
    'propeller_general':{
        'curve3(rows, x)':('def curve3(rows, double x)', 't'),
        'governor(p, pitch, gp, omega, previous, target, command, auto, boost, neutral, dt)':
            ('def governor(p, double pitch, double gp, double omega, double previous, double target, double command, auto, boost, double neutral, double dt)',
             'lo, hi, speed, maximum, delivered, gain, derivative, now, error, minimum, limit, reported, delta, rate'),
        'step(p, state, velocity=(100.0, 0.0, 0.0), body_omega=(0.0, 0.0, 0.0), cg=(0.0, 0.0, 0.0), omega=180.0, previous_omega=None, target_omega=270.0, command=1.0, auto=False, density=1.225, sound_speed=340.0, dt=1 / 60, afterburner=False, torque_gyro=True)':
            ('def step(p, state, velocity=(100.0, 0.0, 0.0), body_omega=(0.0, 0.0, 0.0), cg=(0.0, 0.0, 0.0), double omega=180.0, previous_omega=None, double target_omega=270.0, double command=1.0, auto=False, double density=1.225, double sound_speed=340.0, double dt=1 / 60, afterburner=False, torque_gyro=True)',
             'previous, target, transverse_sq, transverse, pitch, gp, disc, rho_disc, ambient, relax, axial, thrust, torque, first, x, y, z, u, flux, half, divisor, dz, deflection, axial_force, direction, reaction, engine_omega, old_engine, normalized, damp, momentum, neutral, reported, shake, a, b, c, d'),
    },
    'propulsion_general':{
        'step(config, state, velocity=(100.0, 0.0, 0.0), height=0.0, body_omega=(0.0, 0.0, 0.0), cg=(0.0, 0.0, 0.0), dt=1 / 48, seed=12345, nitro=0.0, torque_gyro=True)':
            ('def step(config, state, velocity=(100.0, 0.0, 0.0), double height=0.0, body_omega=(0.0, 0.0, 0.0), cg=(0.0, 0.0, 0.0), double dt=1 / 48, seed=12345, double nitro=0.0, torque_gyro=True)', 'wash'),
        'transmission_step(t, engines, props, state, engine_states, prop_states, velocity, height, body_omega, cg, dt, seed, nitro, torque_gyro)':
            ('def transmission_step(t, engines, props, state, engine_states, prop_states, velocity, double height, body_omega, cg, double dt, seed, double nitro, torque_gyro)',
             'omega, previous, target, load, prop_inertia, prop_friction, wash, ratio, inertia, axial, torque, engine_inertia, friction, limit, inverse, accel, proposed, drag, sound, next_omega'),
    },
    'piston_general':{
        'step(p, s, velocity=(100.0, 0.0, 0.0), height=0.0, dt=1 / 48, seed=12345, torque_multiplier=1.0, nitro=0.0)':
            ('def step(p, s, velocity=(100.0, 0.0, 0.0), double height=0.0, double dt=1 / 48, seed=12345, double torque_multiplier=1.0, double nitro=0.0)',
             'omega, throttle, inlet, tq, modulation, mechanical, power, x, rate, consumption, manifold'),
    },
}


# Export the native frame drivers through their C entry points as well.
# Keyword calls keep their values/defaults; generated wrappers remain callable
# from Python. No equation AST or float32 operation is changed.
DIRECT_CALLS={
    'piston_model':('rpm_torque','mixture'),
    'piston_compressor':('candidate','step'),
    'piston_general':('step',),
    'engine_supply':('mechanical_multiplier',),
    'propulsion_general':('transmission_step','step'),
    'propeller_general':('step','governor','curve3'),
}
for module,names in DIRECT_CALLS.items():
    for header,(typed,locals_) in list(SCALARS[module].items()):
        if header.split('(')[0] in names:
            SCALARS[module][header]=(typed.replace('def ','cpdef ',1).replace(', *,',','),locals_)
SCALARS['piston_model']['boost_active(p, throttle, afterburner, gear, nitro=0.0)']=(
    'cpdef bint boost_active(p, double throttle, afterburner, gear, double nitro=0.0)','')


class Rewrite(ast.NodeTransformer):
    def visit_ImportFrom(self,node):
        if node.module=='component_assembly':
            node.names=[a for a in node.names if a.name not in ('f32','add','sub','mul')]
            if not node.names:return None
        return node
    def visit_FunctionDef(self,node):
        if node.name in ('f32','add','sub','mul'):return None
        return self.generic_visit(node)


def hoist_axial_flow(tree):
    """Make the captured wake helper callable directly from the C frame loop."""
    frame=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='step')
    helper=next(n for n in ast.walk(frame) if isinstance(n,ast.FunctionDef) and n.name=='axial_flow')
    helper.name='_axial_flow'
    helper.args.args.extend([ast.arg(arg='rho_disc'),ast.arg(arg='airflow')])
    class Calls(ast.NodeTransformer):
        def visit_FunctionDef(self,node):
            if node is helper:return None
            return self.generic_visit(node)
        def visit_Call(self,node):
            if isinstance(node.func,ast.Name) and node.func.id=='axial_flow':
                node.func.id='_axial_flow'
                node.args.extend([ast.Name(id='rho_disc',ctx=ast.Load()),ast.Name(id='airflow',ctx=ast.Load())])
            return self.generic_visit(node)
    Calls().visit(frame)
    tree.body.insert(tree.body.index(frame),helper)
    return ast.fix_missing_locations(tree)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--force',action='store_true');args=parser.parse_args()
    expected=signature();manifest=DIRECTORY/'manifest.json'
    if not args.force and manifest.exists() and json.loads(manifest.read_text()).get('signature')==expected:
        print('Compiled EM engine is up to date.');return
    from setuptools import setup,Extension
    from Cython.Build import cythonize
    # Windows keeps loaded extension DLLs locked while a plotter is open.
    # Publish a new generation without overwriting the running application's
    # libraries. The manifest selects it only after all extensions build.
    library=('lib/'+expected[:16]) if os.name=='nt' else 'lib'
    src=DIRECTORY/'src';lib=DIRECTORY/library;src.mkdir(parents=True,exist_ok=True);lib.mkdir(parents=True,exist_ok=True)
    extensions=[]
    def write_source(path,text):
        if not path.exists() or path.read_text()!=text:path.write_text(text)
    # Export tiny scalar kernels to Cython callers without Python dispatch.
    write_source(src/'polar_f32.pxd','cpdef double calc_cl(object p, double a)\ncpdef double calc_cd(object p, double a)\n')
    write_source(src/'polar_runtime.pxd','cpdef double mach_value(object runtime, double mach, int index)\n')
    exports={
        'piston_model':{'div':'cpdef double div(double a, double b)',
                        'pressure_at_height':'cpdef double pressure_at_height(double height, double pressure0=*, double ceiling=*)',
                        'inlet_pressure':'cpdef double inlet_pressure(double height, double body_u, double recovery)'},
        'propeller_step':{'sign':'cpdef double sign(double x)', 'clamp':'cpdef double clamp(double x, double lo, double hi)',
                          'tiny':'cpdef double tiny(double x)'},
    }
    for module,names in dict(DIRECT_CALLS,piston_model=(*DIRECT_CALLS['piston_model'],'boost_active')).items():
        for header,(typed,_) in SCALARS[module].items():
            name=header.split('(')[0]
            if name not in names:continue
            declaration=re.sub(r'=(\([^()]*\)|[^,)]+)', '=*', typed)
            exports.setdefault(module,{})[name]=declaration
    for module,functions in exports.items():write_source(src/(module+'.pxd'),'\n'.join(functions.values())+'\n')
    from em_compiled_blades import generate
    blade_text=generate(ROOT,HELPERS,MATH_HELPERS,SCALARS)
    blade_path=src/'propeller_fast.pyx'
    if not blade_path.exists() or blade_path.read_text()!=blade_text:blade_path.write_text(blade_text)
    write_source(src/'propeller_fast.pxd','cpdef blade_forces(object p, double omega, double pitch, double axial, double tangential, double crossflow, double axial_gradient, double density, double sound_speed, double angular_flow=*)\n')
    extensions.append(Extension('propeller_fast',[str(blade_path)],extra_compile_args=COMPILE_ARGS))
    from em_compiled_propeller import generate as generate_propeller
    frame_path=src/'propeller_frame.pyx'
    write_source(frame_path,generate_propeller(ROOT,HELPERS,MATH_HELPERS,SCALARS,hoist_axial_flow))
    write_source(src/'propeller_frame.pxd',exports['propeller_general']['step']+'\n')
    extensions.append(Extension('propeller_frame',[str(frame_path)],extra_compile_args=COMPILE_ARGS))
    from em_compiled_piston import generate as generate_piston
    piston_path=src/'piston_frame.pyx'
    write_source(piston_path,generate_piston(ROOT,HELPERS,MATH_HELPERS,SCALARS))
    write_source(src/'piston_frame.pxd',exports['piston_general']['step']+'\n')
    extensions.append(Extension('piston_frame',[str(piston_path)],extra_compile_args=COMPILE_ARGS))
    for name in MODULES:
        tree=ast.parse((ROOT/'scripts'/f'{name}.py').read_text())
        tree=ast.fix_missing_locations(Rewrite().visit(tree))
        if name=='propeller_general':tree=hoist_axial_flow(tree)
        text=HELPERS+'\n'+MATH_HELPERS+'\n'+ast.unparse(tree)+'\n'
        for function in ('sin','cos','tan','sqrt','asin','acos','atan','atan2'):
            text=text.replace('math.'+function+'(', '_math_'+function+'(')
        for header,(typed,locals_) in SCALARS.get(name,{}).items():
            original='def '+header+':'
            if original not in text:raise ValueError('Scalar kernel signature changed: '+original)
            text=text.replace(original,typed+':'+('\n    cdef double '+locals_ if locals_ else ''))
        if name=='instructor_protection':
            # Compile the readable, exact recurrence as a scalar C loop.
            # Keep its body and all explicit float32 operations unchanged.
            header='def authority_factor_run(authority_factor, peak, critical_high, recovery_reference, dt, adaptation_rates, max_steps):'
            typed='def authority_factor_run(double authority_factor, double peak, double critical_high, double recovery_reference, double dt, adaptation_rates, long max_steps):\n    cdef double rate, change, increment, value, previous, updated\n    cdef long steps'
            if header not in text:raise ValueError('Authority recurrence signature changed')
            text=text.replace(header,typed)
            header='def history_repeats(records, period, tolerance):'
            typed='def history_repeats(records, long period, double tolerance):\n    cdef long k, i\n    cdef double delta'
            if header not in text:raise ValueError('History convergence signature changed')
            text=text.replace(header,typed)
        for module,functions in exports.items():
            if name==module:continue
            def import_scalars(match):
                names=match.group(1).split(', ');direct=[n for n in names if n.split(' as ')[0] in functions]
                if not direct:return match.group(0)
                rest=[n for n in names if n.split(' as ')[0] not in functions]
                return 'from '+module+' cimport '+', '.join(direct)+'\n'+('from '+module+' import '+', '.join(rest)+'\n' if rest else '')
            text=re.sub(r'^from '+module+r' import (.+)\n',import_scalars,text,flags=re.MULTILINE)
        text=text.replace('from polar_f32 import calc_cl, calc_cd\n','from polar_f32 cimport calc_cl, calc_cd\n')
        if name=='propulsion_general':
            text=text.replace('from piston_general cimport step as piston_step\n',
                'from piston_general cimport step as reference_piston_step\n'
                'from piston_frame cimport step as typed_piston_step\n'
                'import os\n_TYPED_PISTON=os.environ.get("WT_EM_TYPED_PISTON", "1")=="1"\n')
            text=text.replace('from propeller_general cimport step as propeller_step\n',
                'from propeller_general cimport step as reference_propeller_step\n'
                'from propeller_frame cimport step as typed_propeller_step\n'
                'import os\n_TYPED_PROPELLER=os.environ.get("WT_EM_TYPED_PROP", "1")=="1"\n')
            # A diagnostic switch compares the generated representation with
            # the existing implementation on exactly the same trajectories.
            call='r = propeller_step('
            lines=text.splitlines()
            for index,line in enumerate(lines):
                if call in line:
                    reference=line.replace('propeller_step(', 'reference_propeller_step(').strip()
                    typed=line.replace('propeller_step(', 'typed_propeller_step(').strip()
                    indent=line[:len(line)-len(line.lstrip())]
                    lines[index]=indent+'if _TYPED_PROPELLER:\n'+indent+'    '+typed+'\n'+indent+'else:\n'+indent+'    '+reference
            text='\n'.join(lines)+'\n'
            # Spell the intervening default explicitly. Cython's fallback for
            # an aliased cpdef call with a skipped optional argument resolves
            # the unaliased Python name, which is another function here.
            text=text.replace('piston_step(p, s, velocity, height, dt, seed, nitro=nitro)',
                              '(typed_piston_step(p, s, velocity, height, dt, seed, 1.0, nitro) if _TYPED_PISTON else reference_piston_step(p, s, velocity, height, dt, seed, 1.0, nitro))')
        if name=='piston_general':
            original="compressor(p, omega, throttle, inlet, dt, gear=s.get('gear', 0) if p['manual_compressor'] and (not s.get('automatic_compressor', False)) else None, old_gear=s.get('gear', 0), regulator=s.get('regulator', -1.0), afterburner=s.get('afterburner', False), nitro=nitro, turbo=s.get('turbo', 0.0), turbo_command=s.get('turbo_command', 1.0), automatic_turbo=s.get('automatic_turbo', True), height=height)"
            direct="compressor(p, omega, throttle, inlet, dt, s.get('gear', 0) if p['manual_compressor'] and (not s.get('automatic_compressor', False)) else None, s.get('gear', 0), s.get('regulator', -1.0), s.get('afterburner', False), nitro, s.get('turbo', 0.0), s.get('turbo_command', 1.0), s.get('automatic_turbo', True), False, height)"
            assert original in text
            text=text.replace(original,direct)
            text=text.replace("mixture(p, inlet, s.get('mixture', 0.5), automatic=s.get('automatic_mixture', False))",
                              "mixture(p, inlet, s.get('mixture', 0.5), 0.0, s.get('automatic_mixture', False))")
        if name=='propeller_general':
            # Both retained axial-flow deltas have exactly two coordinates.
            # Preserve max order and any's short circuit while removing the
            # Python generator closures from this C-callable frame driver.
            text=text.replace('max((abs(a) for a in delta))',
                              'max(abs(delta[0]), abs(delta[1]))')
            # ast.unparse on Python 3.11 omits parentheses around the tuple
            # target; newer Python versions may include them. Both represent
            # the same two-element short-circuit predicate.
            for target in ('a, b', '(a, b)'):
                text=text.replace('any((mul(a, b) < 0.0 for '+target+' in zip(delta, last_delta)))',
                                  '(mul(delta[0], last_delta[0]) < 0.0 or mul(delta[1], last_delta[1]) < 0.0)')
            if 'any((mul(a, b)' in text:
                raise ValueError('Propeller reversal predicate was not lowered')
            text=text.replace('def _axial_flow(t, u, rho_disc, airflow):',
                'cdef double _axial_flow(double t, double u, double rho_disc, airflow):\n    cdef double value')
            text=text.replace('from propeller_model import blade_forces\n','from propeller_fast cimport blade_forces\n')
            # A starred call needs a Python callable. The nine prepared blade
            # inputs have fixed order; pass them directly to the C entry.
            assert text.count('blade_forces(p, *args)')==2
            text=text.replace('blade_forces(p, *args)','blade_forces(p, '+', '.join('args['+str(i)+']' for i in range(9))+')')
        if name=='prop_steady':
            # Store each actual native output once in a contiguous buffer.
            # Repeated convergence checks can view it directly instead of
            # re-boxing/converting every preceding frame. Keep the reference
            # output lists and returned phase certificates unchanged.
            text='import numpy as np\n'+text
            header=next(line for line in text.splitlines() if line.startswith('def settled_cycle('))
            text=text.replace(header,header+'\n    cdef double[:, ::1] frame_view\n    cdef long frame_channel')
            target='    warm_start = state is not None and (not cold_start)'
            assert text.count(target)==1
            text=text.replace(target,
                '    frame_array = np.empty((max(1, round(max_seconds / dt)), 11), dtype=np.float64)\n'
                '    frame_view = frame_array\n'+target)
            target='        outputs.append(row)'
            assert text.count(target)==1
            text=text.replace(target,target+'\n        for frame_channel in range(11):\n'
                '            frame_view[index, frame_channel] = row[frame_channel]')
            for function in ('aligned_window','aircraft_window'):
                target=function+'(outputs,'
                assert text.count(target)==1
                text=text.replace(target,function+'(frame_array[:index + 1],')
        path=src/f'{name}.pyx' 
        if not path.exists() or path.read_text()!=text:path.write_text(text)
        extensions.append(Extension(name,[str(path)],extra_compile_args=COMPILE_ARGS))
    setup(name='wt-em-local-kernels',ext_modules=cythonize(extensions,nthreads=min(4,os.cpu_count() or 1),
          compiler_directives={'language_level':3,'infer_types':None,'binding':True}),
          script_args=['build_ext','--build-lib',str(lib),'--build-temp',str(DIRECTORY/'objects'),'-j','4'])
    temporary=manifest.with_suffix('.tmp')
    temporary.write_text(json.dumps({'signature':expected,'modules':MODULES,'library':library,
        'compiler':'Cython; '+' '.join(COMPILE_ARGS)},indent=2)+'\n')
    temporary.replace(manifest)
    print('Compiled EM engine ready.')


if __name__=='__main__':main()
