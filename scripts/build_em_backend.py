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


SCALARS = {'instructor_aoa_balance': {'required_acceleration(model, ip, state, delivered_pitch)': ('def required_acceleration(model, ip, state, double delivered_pitch)', 'working, sn, cs, vx, vy, axial, swirl, qvx, wash, k, spin, dynamic, taq, center, positive, negative, command, a, e, cladd, bias, angle, vstab_drag, other, extra, flow, td, required, tail_cl, tail_force, lever, acceleration')}, 'windows_instructor_source': {'fixed_source(model, state)': ('def fixed_source(model, state)', 'distance, denominator, slope, numerator')}, 'piston_model': {'div(a, b)': ('cpdef double div(double a, double b)', ''), 'pressure_at_height(height, pressure0=101300.0, ceiling=18300.0)': ('cpdef double pressure_at_height(double height, double pressure0=101300.0, double ceiling=18300.0)', 'h, p, c'), 'inlet_pressure(height, body_u, recovery)': ('cpdef double inlet_pressure(double height, double body_u, double recovery)', 'rho, ram'), 'rpm_torque(p, omega, throttle, torque_multiplier=1.0, afterburner=False, gear=0, nitro=0.0)': ('def rpm_torque(p, double omega, double throttle, double torque_multiplier=1.0, afterburner=False, gear=0, double nitro=0.0)', 'effective, wref, x, shape, scale, torque, tboost'), 'mixture(p, inlet, command, *, rich_accumulator=0.0, automatic=False)': ('def mixture(p, double inlet, double command, *, double rich_accumulator=0.0, automatic=False)', 'pressure, accum, supplied, limit, factor, inv')}, 'structural_limits': {'interval(x, x0, y0, x1, y1)': ('cpdef inline double interval(double x, double x0, double y0, double x1, double y1)', '')}, 'piston_compressor': {'speed_factor(p, omega)': ('cpdef double speed_factor(p, double omega)', 'x'), 'requested_pressure(p, throttle, height)': ('cpdef double requested_pressure(p, double throttle, double height)', ''), 'low_rpm(manifold, omega, inlet)': ('cpdef double low_rpm(double manifold, double omega, double inlet)', ''), 'candidate(p, s, i, speed, inlet, requested, throttle, afterburner, nitro)': ('def candidate(p, s, int i, double speed, double inlet, double requested, double throttle, afterburner, double nitro)', 'pb, flow, critical, flat, ceiling, line, scale, baseline, shape, x, potential, score, offset'), 'step(p, omega, throttle, inlet, dt, *, gear=0, old_gear=0, regulator=-1.0, afterburner=False, nitro=0.0, turbo=0.0, turbo_command=1.0, automatic_turbo=True, assisted=False, height=0.0)': ('def step(p, double omega, double throttle, double inlet, double dt, *, gear=0, old_gear=0, double regulator=-1.0, afterburner=False, double nitro=0.0, double turbo=0.0, double turbo_command=1.0, automatic_turbo=True, assisted=False, double height=0.0)', 'requested, speed, shape, base, upper, allowed, target, fraction, potential, top, ratio, manifold, line, factor, best, gain')}, 'engine_supply': {'mechanical_multiplier(p, omega, health, cylinders, previous, extra, torque, friction, dt, seed, disabled=False, enabled=True)': ('def mechanical_multiplier(p, double omega, double health, cylinders, double previous, double extra, double torque, double friction, double dt, seed, disabled=False, enabled=True)', 'rate, amplitude, loss, damage, shaft, a, b, c, u, threshold, value')}, 'polar_f32': {'sin(x)': ('cpdef inline double sin(double x)', ''), 'div(a, b)': ('cpdef inline double div(double a, double b)', ''), 'calc_cl(p, a)': ('cpdef double calc_cl(p, double a)', 's, crit, cy, after, da, x, maxang, sa, pa, h, den, coeff, local_sign, local_angle, one, two, correction, wave'), 'calc_cd(p, a)': ('cpdef double calc_cd(p, double a)', 'line, delta, cd, bound'), 'calc_c(p, a, angle, cl_add=0.0, cd_coeff=1.0)': ('def calc_c(p, double a, double angle, double cl_add=0.0, double cd_coeff=1.0)', 'cd, cl, radians, sn, cs')}, 'polar_runtime': {'mach_value(runtime, mach, index)': ('cpdef double mach_value(runtime, double mach, int index)', 'a, b, high, slope, limit, c0, c1, c2, c3, value'), 'evaluate(runtime, mach, cy_mult=1.0)': ('def evaluate(runtime, double mach, double cy_mult=1.0)', 'lam, slope, parab, decline, maxdist, cdafter, clafterl, clafterh, cl0, ah, al, ch, cl, cd, span, area, ind, focus, cm0, cm1, kq, clkq, original_cl0, mm, value, effective_slope, inv, dh, dl, lineh, low, linel, highden, lowden, ph, pl')}, 'piston_general': {'step(p, s, velocity=(100.0, 0.0, 0.0), height=0.0, dt=1 / 48, seed=12345, torque_multiplier=1.0, nitro=0.0)': ('def step(p, s, velocity=(100.0, 0.0, 0.0), double height=0.0, double dt=1 / 48, seed=12345, double torque_multiplier=1.0, double nitro=0.0)', 'omega, throttle, inlet, tq, modulation, mechanical, power, x, rate, consumption, manifold')}}


# The steady propeller model deliberately uses binary64 polars. Keep these
# separate from the binary32 aircraft polar kernels and retain strict FP flags.
SCALARS['polar_model'] = {
    'safe_div(a, b)': ('cpdef double safe_div(double a, double b)', ''),
    'mach_multiplier(props, mach, index)': ('def mach_multiplier(props, double mach, int index)', 'low, a, b, high, slope, limit, value, t'),
    'calc_cl(p, a)': ('cpdef double calc_cl(p, double a)', 's, crit, after, cy, max_ang, da, h, max_da, need, local_sign, local_aoa, correction'),
    'calc_cd(p, a)': ('cpdef double calc_cd(p, double a)', 'linear, s, crit, cd'),
}
SCALARS['prop_quasisteady'] = {
    'blade_forces(p, omega, pitch, axial, transverse, density, sound, angular_flow=0.0)':
        ('def blade_forces(p, double omega, double pitch, double axial, double transverse, double density, double sound, double angular_flow=0.0)',
         'thrust, torque, area_unit, station, twist, width, radius, tangential, phi, alpha, speed, force_unit, lift, drag'),
}

DIRECT_CALLS = {'piston_model': ('rpm_torque', 'mixture'), 'piston_compressor': ('candidate', 'step'), 'piston_general': ('step',), 'engine_supply': ('mechanical_multiplier',)}
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


    library=('lib/'+expected[:16]) if os.name=='nt' else 'lib'
    src=DIRECTORY/('src-'+expected[:16]);lib=DIRECTORY/library;src.mkdir(parents=True,exist_ok=True);lib.mkdir(parents=True,exist_ok=True)
    extensions=[]
    def write_source(path,text):
        if not path.exists() or path.read_text()!=text:path.write_text(text)

    write_source(src/'polar_f32.pxd','cpdef double calc_cl(object p, double a)\ncpdef double calc_cd(object p, double a)\n')
    write_source(src/'polar_runtime.pxd','cpdef double mach_value(object runtime, double mach, int index)\n')
    exports={
        'polar_model': {'calc_cl':'cpdef double calc_cl(object p, double a)',
                        'calc_cd':'cpdef double calc_cd(object p, double a)'},
        'piston_model':{'div':'cpdef double div(double a, double b)',
                        'pressure_at_height':'cpdef double pressure_at_height(double height, double pressure0=*, double ceiling=*)',
                        'inlet_pressure':'cpdef double inlet_pressure(double height, double body_u, double recovery)'},
    }
    for module,names in dict(DIRECT_CALLS,piston_model=(*DIRECT_CALLS['piston_model'],'boost_active')).items():
        for header,(typed,_) in SCALARS[module].items():
            name=header.split('(')[0]
            if name not in names:continue
            declaration=re.sub(r'=(\([^()]*\)|[^,)]+)', '=*', typed)
            exports.setdefault(module,{})[name]=declaration
    for module,functions in exports.items():write_source(src/(module+'.pxd'),'\n'.join(functions.values())+'\n')
    for name in MODULES:
        tree=ast.parse((ROOT/'scripts'/f'{name}.py').read_text())
        tree=ast.fix_missing_locations(Rewrite().visit(tree))
        text=HELPERS+'\n'+MATH_HELPERS+'\n'+ast.unparse(tree)+'\n'
        for function in ('sin','cos','tan','sqrt','asin','acos','atan','atan2'):
            text=text.replace('math.'+function+'(', '_math_'+function+'(')
        for header,(typed,locals_) in SCALARS.get(name,{}).items():
            original='def '+header+':'
            if original not in text:raise ValueError('Scalar kernel signature changed: '+original)
            text=text.replace(original,typed+':'+('\n    cdef double '+locals_ if locals_ else ''))
        if name=='instructor_protection':


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
                # These calls use sparse keyword arguments. Cython's C-import
                # fallback loses the aliased name for such calls; retain the
                # compiled function's Python wrapper instead.
                if name=='prop_quasisteady' and module in ('piston_general','piston_compressor'):
                    return match.group(0)
                names=match.group(1).split(', ');direct=[n for n in names if n.split(' as ')[0] in functions]
                if not direct:return match.group(0)
                rest=[n for n in names if n.split(' as ')[0] not in functions]
                return 'from '+module+' cimport '+', '.join(direct)+'\n'+('from '+module+' import '+', '.join(rest)+'\n' if rest else '')
            text=re.sub(r'^from '+module+r' import (.+)\n',import_scalars,text,flags=re.MULTILINE)
        text=text.replace('from polar_f32 import calc_cl, calc_cd\n','from polar_f32 cimport calc_cl, calc_cd\n')
        if name=='piston_general':
            original="compressor(p, omega, throttle, inlet, dt, gear=s.get('gear', 0) if p['manual_compressor'] and (not s.get('automatic_compressor', False)) else None, old_gear=s.get('gear', 0), regulator=s.get('regulator', -1.0), afterburner=s.get('afterburner', False), nitro=nitro, turbo=s.get('turbo', 0.0), turbo_command=s.get('turbo_command', 1.0), automatic_turbo=s.get('automatic_turbo', True), height=height)"
            direct="compressor(p, omega, throttle, inlet, dt, s.get('gear', 0) if p['manual_compressor'] and (not s.get('automatic_compressor', False)) else None, s.get('gear', 0), s.get('regulator', -1.0), s.get('afterburner', False), nitro, s.get('turbo', 0.0), s.get('turbo_command', 1.0), s.get('automatic_turbo', True), False, height)"
            assert original in text
            text=text.replace(original,direct)
            text=text.replace("mixture(p, inlet, s.get('mixture', 0.5), automatic=s.get('automatic_mixture', False))",
                              "mixture(p, inlet, s.get('mixture', 0.5), 0.0, s.get('automatic_mixture', False))")
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
