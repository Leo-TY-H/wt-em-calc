"""Build larger exact numerical sections from the readable reference modules.

Only scalar storage and call boundaries change. Every binary32 helper rounds
both operands, retaining the original summation order and strict FP compilation.
"""
import ast
import re
from pathlib import Path

MODULES = ('aero_vectors', 'body_integration', 'control_frame', 'acceleration_control',
           'radar_tracking', 'radar_modes', 'radar_provider_manager', 'geometric_radar',
           'optical_point_flight', 'radar_provider_controls', 'optical_manager_controls', 'radar_point_modes',
           'shared_seeker', 'optical_tracking', 'radar_tracking_filters')

HELPERS = '''
from libc.math cimport sqrt as _sqrt
cdef inline double _math_sqrt(double x) except *:
    if x < 0.: raise ValueError('math domain error')
    return _sqrt(x)
cpdef inline double f32(double x):
    return <float>x
cpdef inline double add(double a, double b):
    cdef double x = <float>a, y = <float>b
    return <float>(x+y)
cpdef inline double sub(double a, double b):
    cdef double x = <float>a, y = <float>b
    return <float>(x-y)
cpdef inline double mul(double a, double b):
    cdef double x = <float>a, y = <float>b
    return <float>(x*y)
cpdef inline double div(double a, double b):
    cdef double x = <float>a, y = <float>b
    if y == 0.: raise ZeroDivisionError('float division by zero')
    return <float>(x/y)
'''

# Explicit scalar sets avoid inferring C integer arithmetic, changing overflow,
# or accidentally coercing optional values such as a missing AoA limit.
LOCALS = {
    'aero_vectors': {
        'dot': '',
        'composed_quaternion': 'x y z w a b c d',
        'columns': 'x y z w',
        'forces': 'cm_speed64 speed ww arm offset cosine_offset sine_offset bx by bz mach z poly pressure k ki cx_total cx_mach cy qgeom dt fx fy fz x y w ex ey ez zx yw ey2 plus_x plus_z mass inverse n2',
        'evaluate': 'cosine sin2 lift_coefficient cap cd lift_scalar',
    },
    'body_integration': {
        'half_angle_trig': 'h quadrant_input t t2 c s',
        'orientation': 'sx sy sz cx cy cz dx dy dz dw x y z w norm2',
        'integrate': 'dt half_dt2 vx vy vz distance_step elapsed',
    },
    'control_frame': {
        'length': 'a b c',
        'normalized': 'n inv',
        'matrix_quaternion': 'fx fy fz ux uy uz rx ry rz trace_x trace_yz best sx sy sz sw x y z w xy wz wy xz yz wx a b c d e f error',
        'frame': 'x y z w speed inv fx fy fz ux uy uz rx ry rz positive negative',
    },
    'acceleration_control': {
        'lateral_axes': 'x y z w hx t u hy common hz vx vy vz',
        'project': 'x y z hx hy hz vx vy vz',
        'density_polynomial': 'z a c',
        'atmosphere_values': 'height ceiling z density poly c sound',
        'pid_coefficients': 'time fraction',
        'update': 'squared vx vy vz speed_squared limit mass cy_mult factor limit_squared reduction scale_squared rho scale mx my mz kp ki kd ilim integral derivative proportional differential value',
    },
    'radar_tracking': {'update': 'dt projected_position projected_velocity previous_speed acceleration'},
    'radar_modes': {'update': 'dt'},
    'radar_provider_manager': {'update': 'elapsed dt distance projection tgo range_squared closure'},
    'optical_point_flight': {'cap_velocity': 'end_speed c squared factor',
                           'update': 'dt target_time motor_dt sub_dt sub_time remaining elapsed'},
    'radar_point_modes': {'update': 'limit_age los_check_timeout dt minimum maximum half'},
    'shared_seeker': {
        'properties': 'radians default_gate',
        'normalize': 'x y z squared length reciprocal',
        'world_residual': 'x y z w mx my mz a b c d e f ww xx yy zz',
        'coast_body': 'x y z w ux uy uz a b nw d e c f ww xx yy zz',
        'slew': 'dt x y z step limit previous target change',
        'update': 'dt gate step gain',
    },
    'optical_tracking': {
        'properties': 'half_degree',
        'pre_slew_body': 'x y z w',
        'reseed': 'x y z limit',
        'update': 'old_time new_time los_check_timeout dt half_inverse_distance',
    },
    'radar_tracking_filters': {
        'coefficients': 'interval n',
        'clamp_channel': 'value half',
        # Disabled channels may receive no observation. Keep observed boxed
        # until the original present-channel guard performs its conversion.
        'update': 'dt limit_age gap interval value rate alpha beta predicted residual change next_rate limit',
        'coast': 'dt value',
    },
}


class Rewrite(ast.NodeTransformer):
    def visit_ImportFrom(self, node):
        if node.module == 'kernels':
            node.names = [v for v in node.names if v.name not in ('f32', 'add', 'sub', 'mul', 'div')]
            if not node.names:
                return None
        if node.module == 'copy':
            for alias in node.names:
                if alias.name != 'deepcopy':
                    raise ValueError('Unexpected copy import in numerical kernel')
            node.module = '_missile_accel'
        return node


def source(name, model):
    tree = Rewrite().visit(ast.parse((model / (name + '.py')).read_text()))
    if name == 'control_frame':
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'matrix_quaternion')
        loop = next(n for n in ast.walk(function) if isinstance(n, ast.For) and
                    isinstance(n.target, ast.Name) and n.target.id == 'sx')
        if ast.literal_eval(loop.iter) != (1., -1.):
            raise ValueError('Quaternion sign-enumeration reference changed')
        # The second eight candidates negate all four components of candidates
        # already visited. Every tested matrix entry is a product of two
        # components, so its value AND rounding are identical under that full
        # negation. The (sz == sw) summation branch is also identical. Strict '<'
        # tie selection therefore can never select a second-half candidate.
        loop.iter = ast.Tuple(elts=[ast.Constant(1.)], ctx=ast.Load())
    ast.fix_missing_locations(tree)
    text = ast.unparse(tree)
    # Insert declarations after each signature, including the small nested
    # aerodynamic evaluator. ast.unparse keeps each signature on one line.
    for function, scalars in LOCALS.get(name, {}).items():
        if not scalars:
            continue
        lines = text.splitlines()
        found = False
        for index, line in enumerate(lines):
            if line.lstrip().startswith('def ' + function + '('):
                indent = len(line) - len(line.lstrip())
                node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == function)
                arguments = {a.arg for a in (*node.args.args, *node.args.kwonlyargs)}
                locals_ = []
                for scalar in scalars.split():
                    if scalar in arguments:
                        lines[index] = re.sub(r'\b' + scalar + r'(?=\s*[,=)])', 'double ' + scalar, lines[index])
                    else:
                        locals_.append(scalar)
                if locals_:
                    lines.insert(index + 1, ' ' * (indent + 4) + 'cdef double ' + ', '.join(locals_))
                found = True
                break
        if not found:
            raise ValueError('Numerical function missing: ' + name + '.' + function)
        text = '\n'.join(lines)
    return HELPERS + '\n' + text.replace('math.sqrt(', '_math_sqrt(') + '\n'


def functions(name, model):
    return [node.name for node in ast.parse((model / (name + '.py')).read_text()).body
            if isinstance(node, ast.FunctionDef)]
