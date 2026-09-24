"""Generate a typed held-condition propeller from the readable native equations.

Only representation and loop-invariant placement change. The readable owner
remains the reference; every frame still updates its shaft/governor/wake state.
"""
import ast
import copy


def generate(root, helpers, math_helpers, scalar_declarations, hoist_axial_flow):
    tree = ast.parse((root/'scripts/propeller_general.py').read_text())
    tree = hoist_axial_flow(tree)
    functions = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
    scalars = ('cyclic differential_pitch active_pitch_2d pitch_1d_count governor '
               'pitch_min pitch_max governor_speed boost_omega max_omega min_omega '
               'reduction governor_fast pitch_command_report iterative coaxial '
               'diameter thrust_deflection max_deflection direction inertia '
               'inv_max_omega momentum_scale neutral_radius mean_twist torque_gyro_always').split()
    integral = set('cyclic differential_pitch active_pitch_2d pitch_1d_count governor governor_fast '
                   'pitch_command_report iterative coaxial direction torque_gyro_always'.split())
    objects = ('polar',)
    vectors = dict(basis=9, position=3, damping_speed=4, pitch_damping=4, yaw_damping=4, shake=4)
    text = helpers+'\n'+math_helpers+'''
import math
from collections import OrderedDict
from piston_model cimport div
from propeller_step cimport sign, clamp, tiny
from structural_limits import interval
from propeller_fast cimport blade_forces
from propeller_general import local_flow, curve3

cdef class PropellerCondition:
    cdef object source, polar
    cdef double disc, rho_disc, ambient, transverse_sq, transverse, damp
    cdef double density, sound_speed, dt
    cdef double local[3], local_w[3], airflow[3], arm[3]
'''
    for kind in ('double', 'long'):
        fields = [k for k in scalars if (k in integral) == (kind == 'long')]
        text += '    cdef '+kind+' '+', '.join(fields)+'\n'
    for name, size in vectors.items():
        text += '    cdef double '+name+'['+str(size)+']\n'
    text += '''
    def __init__(self, source, velocity, body_omega, cg, density, sound_speed, dt):
        cdef int i
        self.source=source
        self.density=f32(density); self.sound_speed=f32(sound_speed); self.dt=f32(dt)
'''
    for field in scalars+list(objects):
        text += "        self."+field+"=source['"+field+"']\n"
    for field, size in vectors.items():
        text += '        for i in range('+str(size)+'): self.'+field+"[i]=source['"+field+"'][i]\n"
    # These statements are the corresponding original step expressions, with
    # inputs fixed for the entire native settling trajectory.
    text += '''
        local, local_w, transverse_sq, transverse=local_flow(tuple(source['basis']), tuple(source['position']),
            tuple(map(f32,velocity)), tuple(map(f32,body_omega)))
        for i in range(3):
            self.local[i]=local[i]; self.local_w[i]=local_w[i]
            self.arm[i]=sub(source['position'][i],f32(cg[i]))
        self.transverse_sq=transverse_sq; self.transverse=transverse
        self.disc=mul(mul(source['diameter'],source['diameter']),f32(.7853981852531433))
        self.rho_disc=mul(self.density,self.disc)
        airflow=curve3(source['airflow'],local[0])
        for i in range(3): self.airflow[i]=airflow[i]
        self.ambient=add(airflow[0],local[0])
        self.damp=interval(local[0],*source['damping_speed'])

_contexts=OrderedDict()

cdef PropellerCondition condition(p, velocity, body_omega, cg, double density, double sound_speed, double dt):
    key=(id(p),tuple(velocity),tuple(body_omega),tuple(cg),density,sound_speed,dt)
    result=_contexts.get(key)
    if result is None:
        result=PropellerCondition(p,velocity,body_omega,cg,density,sound_speed,dt)
        if len(_contexts)>=256: _contexts.popitem(last=False)
        _contexts[key]=result
    return result

def clear_contexts():
    _contexts.clear()

'''

    class Fields(ast.NodeTransformer):
        def visit_Subscript(self, node):
            if isinstance(node.value, ast.Name) and node.value.id == 'p':
                assert isinstance(node.slice, ast.Constant), ast.unparse(node)
                key = node.slice.value
                assert key in scalars or key in objects or key in vectors, key
                return ast.Attribute(value=ast.Name(id='p', ctx=ast.Load()), attr=key, ctx=node.ctx)
            return self.generic_visit(node)

    def cpp_function(name, signature, locals_=''):
        node = copy.deepcopy(functions[name])
        node.decorator_list = []
        header = ast.unparse(node).splitlines()[0]
        code = ast.unparse(ast.fix_missing_locations(Fields().visit(node)))
        return code.replace(header, signature+(':'+locals_ if locals_ else ':'))+'\n\n'

    # Preserve the basis transform's multiplication/addition ordering.
    text += cpp_function('transform', 'cdef object transform(double *b, object v)')
    text += cpp_function('_axial_flow', 'cdef double _axial_flow(double t, double u, double rho_disc, double *airflow)',
                         '\n    cdef double value')
    key = next(k for k in scalar_declarations['propeller_general'] if k.startswith('governor('))
    declaration, temps = scalar_declarations['propeller_general'][key]
    declaration = declaration.replace('cpdef governor(p,', 'cdef object governor(PropellerCondition p,')
    text += cpp_function('governor', declaration, '\n    cdef double '+temps+'\n    cdef long kind')

    node = copy.deepcopy(functions['step'])
    key = next(k for k in scalar_declarations['propeller_general'] if k.startswith('step('))
    header, temps = scalar_declarations['propeller_general'][key]
    header = header.replace('cpdef step(p,', 'cdef object _step(PropellerCondition p,')
    # Keep all native scalar equations, but take invariant values from the
    # prepared condition. No rounded quantity is replaced by an approximation.
    replacements = {
        "v = list(map(f32, velocity))": '',
        "w = list(map(f32, body_omega))": '',
        "r = p['position']": '',
        "basis = p['basis']": '',
        'local, local_w, transverse_sq, transverse = local_flow(tuple(basis), tuple(r), tuple(v), tuple(w))': '',
        "disc = mul(mul(p['diameter'], p['diameter']), f32(0.7853981852531433))": '',
        'rho_disc = mul(density, disc)': '',
        "airflow = curve3(p['airflow'], local[0])": '',
        'ambient = add(airflow[0], local[0])': '',
        "damp = interval(local[0], *p['damping_speed'])": '',
        'arm = [sub(x, y) for x, y in zip(r, map(f32, cg))]': '',
        "flow = list(map(f32, state.get('flow', [0.0, 0.0, 0.0])))":
            "flow_input = state.get('flow', [0.0, 0.0, 0.0])\nfor i in range(3): flow[i] = f32(flow_input[i])",
        'last_delta = [0.0, 0.0]': 'last_delta[0] = 0.0\nlast_delta[1] = 0.0',
        'delta = [clamp(sub(a, b), -10.0, 10.0) for a, b in zip([x, y], flow[:2])]':
            'delta[0] = clamp(sub(x, flow[0]), -10.0, 10.0)\ndelta[1] = clamp(sub(y, flow[1]), -10.0, 10.0)',
        'flow = [x, y, z]': 'flow[0] = x\nflow[1] = y\nflow[2] = z',
        'flow = [add(mul(relax, a), b) for a, b in zip(delta + [dz], flow)]':
            'flow[0] = add(mul(relax, delta[0]), flow[0])\nflow[1] = add(mul(relax, delta[1]), flow[1])\nflow[2] = add(mul(relax, dz), flow[2])',
        'last_delta = delta': 'last_delta[0] = delta[0]\nlast_delta[1] = delta[1]',
    }
    replacements = {ast.unparse(ast.parse(k).body[0]): v for k, v in replacements.items()}
    found = set()

    class Prepare(ast.NodeTransformer):
        def visit_Assign(self, node):
            original = ast.unparse(node)
            if original in replacements:
                found.add(original)
                return ast.parse(replacements[original]).body
            return self.generic_visit(node)
    node = Prepare().visit(node)
    assert set(replacements) == found, set(replacements)-found
    node = Fields().visit(node)
    invariant_names = {'basis', 'local', 'local_w', 'transverse_sq', 'transverse',
                       'disc', 'rho_disc', 'airflow', 'ambient', 'damp', 'arm'}

    class Invariants(ast.NodeTransformer):
        def visit_Name(self, node):
            if node.id in invariant_names:
                return ast.Attribute(value=ast.Name(id='p', ctx=ast.Load()), attr=node.id, ctx=node.ctx)
            return node
    node = Invariants().visit(node)
    code = ast.unparse(ast.fix_missing_locations(node))
    first = code.splitlines()[0]
    code = code.replace(first, header+':\n    cdef double '+temps+
                        '\n    cdef double flow[3], delta[2], last_delta[2]\n    cdef int i, iteration')
    assert code.count('blade_forces(p, *args)') == 2
    code = code.replace('blade_forces(p, *args)', 'blade_forces(p.source, '+', '.join('args['+str(i)+']' for i in range(9))+')')
    code = code.replace('max((abs(a) for a in delta))', 'max(abs(delta[0]), abs(delta[1]))')
    code = code.replace('any((mul(a, b) < 0.0 for a, b in zip(delta, last_delta)))',
                        '(mul(delta[0], last_delta[0]) < 0.0 or mul(delta[1], last_delta[1]) < 0.0)')
    code = code.replace('any((mul(a, b) < 0.0 for (a, b) in zip(delta, last_delta)))',
                        '(mul(delta[0], last_delta[0]) < 0.0 or mul(delta[1], last_delta[1]) < 0.0)')
    code = code.replace('[mul(momentum, c) for c in p.basis[:3]]', '[mul(momentum, p.basis[i]) for i in range(3)]')
    for field in ('pitch_damping','yaw_damping'):
        code = code.replace('*p.'+field, ', '.join('p.'+field+'['+str(i)+']' for i in range(4)))
    shake_assignment = ast.unparse(ast.parse('a,b,c,d = p.shake').body[0])
    code = code.replace(shake_assignment, 'a, b, c, d = p.shake[0], p.shake[1], p.shake[2], p.shake[3]')
    code = code.replace('flow=flow,', 'flow=[flow[0], flow[1], flow[2]],')
    text += code+'\n\n'
    # Keep the public native step API and its complete diagnostics intact.
    text += '''
cpdef step(p, state, velocity=(100.,0.,0.), body_omega=(0.,0.,0.), cg=(0.,0.,0.),
           double omega=180., previous_omega=None, double target_omega=270., double command=1., auto=False,
           double density=1.225, double sound_speed=340., double dt=1/60, afterburner=False, torque_gyro=True):
    cdef PropellerCondition context=condition(p,velocity,body_omega,cg,density,sound_speed,dt)
    return _step(context,state,velocity,body_omega,cg,omega,previous_omega,target_omega,command,auto,
                 density,sound_speed,dt,afterburner,torque_gyro)
'''
    for name in ('sin', 'cos', 'tan', 'sqrt', 'asin', 'acos', 'atan', 'atan2'):
        text = text.replace('math.'+name+'(', '_math_'+name+'(')
    return text
