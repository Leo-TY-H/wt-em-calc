"""Optional dependency-free Rust kernels; no compilation during app startup.

WT_NUMERIC_BACKEND=auto uses direct Rust builtins on GIL-enabled CPython, including
inside compiled modules. Other interpreters use portable ctypes calls and keep
compiled EM kernels preferred. 'python' disables Rust; 'rust' requires a build.
"""
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / '.native_rust'
FIELDS = ('cl0 cd0 indCoeff clLineCoeff cyCritH cyCritL aoaCritH aoaCritL '
          'aoaLineH aoaLineL parabCyCoeffH parabCyCoeffL aerCenterOffset '
          'clToCm0 clToCm1 parabAngle declineCoeff maxDistAng cdAfterCoeff '
          'clAfterCritL clAfterCritH kq clKq cyMult').split()
_library = None
_attempted = False
D = ctypes.c_double
P = ctypes.POINTER(D)


def signature():
    digest = hashlib.sha256()
    for file in (ROOT/'native/Cargo.toml', ROOT/'native/Cargo.lock',
                 *sorted((ROOT/'native/src').rglob('*.rs')), Path(__file__)):
        digest.update(file.read_bytes())
    digest.update((sys.platform + platform.machine().lower()).encode())
    return digest.hexdigest()


def load(module_file):
    global _library, _attempted
    mode = os.environ.get('WT_NUMERIC_BACKEND', 'auto')
    if mode not in ('auto', 'rust', 'python'):
        raise ValueError('Unknown WT_NUMERIC_BACKEND: ' + mode)
    if mode == 'python':
        return None
    if not _attempted:
        _attempted = True
        try:
            manifest = json.loads((DIRECTORY/'manifest.json').read_text())
            if manifest['signature'] != signature():
                raise ValueError('Rust kernel source/platform mismatch')
            name = manifest['binary']
            if Path(name).name != name:
                raise ValueError('Invalid Rust library path')
            path = DIRECTORY/name
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['sha256']:
                raise ValueError('Rust library hash mismatch')
            library = ctypes.CDLL(str(path))
            library.wt_numeric_abi.argtypes = []
            library.wt_numeric_abi.restype = ctypes.c_uint32
            if library.wt_numeric_abi() != 2:
                raise ValueError('Unsupported Rust numeric ABI')
            library.wt_polar.argtypes = [P, D, D, D, D, ctypes.c_uint32, P]
            library.wt_polar.restype = ctypes.c_uint32
            library.wt_polar_batch.argtypes = [P, P, ctypes.c_size_t, P]
            library.wt_polar_batch.restype = ctypes.c_uint32
            library.wt_atmosphere.argtypes = [D, P]
            library.wt_atmosphere.restype = None
            library.wt_orientation.argtypes = [P, P]
            library.wt_orientation.restype = ctypes.c_uint32
            library.wt_matrix_quaternion.argtypes = [P, P]
            library.wt_matrix_quaternion.restype = ctypes.c_uint32
            library.wt_vector.argtypes = [P, ctypes.c_uint32, P]
            library.wt_vector.restype = ctypes.c_uint32
            library.wt_integrate.argtypes = [P, P]
            library.wt_integrate.restype = ctypes.c_uint32
            library.wt_controller.argtypes = [P, P, ctypes.c_size_t, P]
            library.wt_controller.restype = ctypes.c_uint32
            library.wt_seeker.argtypes = [P, P]
            library.wt_seeker.restype = ctypes.c_uint32
            library.wt_slew.argtypes = [P, P]
            library.wt_slew.restype = ctypes.c_uint32
            library.wt_aero.argtypes = [P, P, P, ctypes.c_size_t, P]
            library.wt_aero.restype = ctypes.c_uint32
            for name in ('wt_force', 'wt_moment'):
                function = getattr(library, name)
                function.argtypes = [P, P]
                function.restype = ctypes.c_uint32
            library._python = _python_interface(library)
            _library = library
        except (OSError, ValueError, KeyError, AttributeError) as error:
            if mode == 'rust':
                raise RuntimeError('Rust kernels unavailable; run scripts/build_rust_backend.py') from error
    if mode == 'rust' and _library is None:
        raise RuntimeError('Rust kernels unavailable; run scripts/build_rust_backend.py')
    if mode == 'auto' and not module_file.endswith('.py') and (_library is None or not _library._python):
        return None
    return _library


def _python_interface(library):
    """Register GIL-held native builtins using the public CPython Stable ABI.

    No Python object layouts or version-specific symbols are used. Other
    interpreters and free-threaded builds keep the portable ctypes interface.
    The builtins own their key tuple; the library remains owned by this module.
    """
    if os.environ.get('WT_RUST_INTERFACE') == 'ctypes' or sys.implementation.name != 'cpython' or (hasattr(sys, '_is_gil_enabled') and not sys._is_gil_enabled()):
        return {}
    names = ('Py_IncRef', 'Py_DecRef', 'PyTuple_GetItem', 'PyFloat_AsDouble',
             'PyErr_Occurred', 'PyDict_GetItemWithError', 'PyList_Size',
             'PyList_GetItem', 'PyErr_Clear', 'PyTuple_Size', 'PyFloat_FromDouble',
             'PyList_New', 'PyList_SetItem', 'PyDict_New', 'PyDict_SetItemString',
             'PyLong_FromLongLong', 'PyCFunction_NewEx', 'PyErr_ExceptionMatches', 'PyObject_Type',
             'PyTuple_New','PyTuple_SetItem','PyObject_CallObject',
             'PyDict_Next','PyDict_SetItem','PyList_Append','PyLong_FromVoidPtr','PyDict_Size','PyObject_Call')
    addresses = (ctypes.c_size_t * len(names))(*(ctypes.cast(getattr(ctypes.pythonapi,name),ctypes.c_void_p).value for name in names))
    keys = (*FIELDS, 'left_wing','right_wing','left_hstab','right_hstab','vstab','fuselage','chute','parasite',
            'position','velocity','omega','quaternion','time','clocks','distance','water_distance','water')
    from copy import deepcopy
    library.wt_python_field_names.argtypes=[]
    library.wt_python_field_names.restype=ctypes.c_char_p
    field_names=library.wt_python_field_names().decode('ascii').split()
    context = (*map(sys.intern,keys), *([None]*6), None, MemoryError, list, tuple, dict, library,
               float,int,str,bool,bytes,type(None),deepcopy,True,False,*map(sys.intern,field_names))
    # PYFUNCTYPE retains the GIL. ctypes consumes the returned new reference
    # when converting a py_object function result (do not decrement it again).
    initialize = ctypes.PYFUNCTYPE(ctypes.py_object,ctypes.POINTER(ctypes.c_size_t),ctypes.c_size_t,ctypes.py_object,ctypes.c_uint32)(('wt_python_init',library))
    return initialize(addresses,len(names),context,_fast_layout_available())


def _fast_layout_available():
    """Probe the explicitly supported CPython layouts; never guess future ABIs.

    Header declarations: CPython 3.11/3.12 Include/cpython/{list,tuple,float,dict}object.h.
    Stable-ABI builtins remain the fallback, including for trace-ref/debug layouts.
    """
    if os.environ.get('WT_RUST_INTERFACE')=='stable' or sys.version_info[:2] not in ((3,11),(3,12)) or hasattr(sys,'getobjects') or ctypes.sizeof(ctypes.c_void_p)!=8:
        return False
    class Head(ctypes.Structure):
        _fields_=[('refs',ctypes.c_ssize_t),('kind',ctypes.c_void_p)]
    class Var(ctypes.Structure):
        _fields_=[('head',Head),('size',ctypes.c_ssize_t)]
    class Float(ctypes.Structure):
        _fields_=[('head',Head),('value',ctypes.c_double)]
    class List(ctypes.Structure):
        _fields_=[('base',Var),('items',ctypes.POINTER(ctypes.c_void_p)),('allocated',ctypes.c_ssize_t)]
    class Dict(ctypes.Structure):
        _fields_=[('head',Head),('used',ctypes.c_ssize_t),('version',ctypes.c_uint64)]
    value=42.25
    if Head.from_address(id(value)).kind!=id(float) or Float.from_address(id(value)).value!=value:
        return False
    items=[value,None,True]
    raw=List.from_address(id(items))
    if raw.base.head.kind!=id(list) or raw.base.size!=len(items) or not raw.items:
        return False
    if list(raw.items[:len(items)])!=list(map(id,items)):
        return False
    row=tuple(items)
    if Var.from_address(id(row)).head.kind!=id(tuple) or Var.from_address(id(row)).size!=len(row):
        return False
    raw_items=ctypes.cast(id(row)+ctypes.sizeof(Var),ctypes.POINTER(ctypes.c_void_p))
    if list(raw_items[:len(row)])!=list(map(id,row)):
        return False
    mapping={'probe':value}
    raw_dict=Dict.from_address(id(mapping))
    if raw_dict.head.kind!=id(dict) or raw_dict.used!=len(mapping):
        return False
    version=raw_dict.version
    mapping['probe']=value+1
    return version!=0 and raw_dict.version!=version


def scalar_functions(library, references):
    """Bind masked binary32 missile operations; invalid inputs call the reference."""
    if not library._python:
        return references
    bind=ctypes.PYFUNCTYPE(ctypes.py_object,ctypes.c_uint32,ctypes.py_object)(('wt_python_scalar',library))
    return tuple(bind(i,(reference,library)) for i,reference in enumerate(references))


def bind_native(library, reference, kind):
    if library is None or not library._python:
        return reference
    bind=ctypes.PYFUNCTYPE(ctypes.py_object,ctypes.c_uint32,ctypes.py_object)(('wt_python_bound',library))
    return bind(kind,(library._python['_context'],reference,library,(0.,)*4))


def copy_function(library, reference):
    if not library._python:
        return reference
    native=library._python['deepcopy']
    def deepcopy(value,memo=None):
        return native(value,memo)
    return deepcopy


def finite_function(library, reference, string_keys=False):
    if not library._python:
        return reference
    native=library._python['finite_graph']
    def finite(value,*args,**kwargs):
        if native(value,string_keys):
            return None
        return reference(value,*args,**kwargs)
    return finite


def _array(values):
    values = tuple(values)
    # Preserve the reference's struct.pack overflow and trig domain errors.
    if any(not math.isfinite(v) or abs(v) > 3.4028234663852886e38 for v in values):
        return None
    return (D * len(values))(*values)


def polar(library, p, a, angle=0., cl_add=0., cd_coeff=1., mode=2):
    if library._python:
        return library._python['polar'](p,a,angle,cl_add,cd_coeff,mode)
    try:
        packed = _array(p[key] for key in FIELDS)
    except KeyError:
        return None
    if packed is None or _array((a, angle, cl_add, cd_coeff)) is None:
        return None
    result = (D * 2)()
    if not library.wt_polar(packed, a, angle, cl_add, cd_coeff, mode, result):
        return None
    return list(result) if all(math.isfinite(v) for v in result) else None


def assembly(library, values, moment=False):
    if library._python:
        return library._python['assembly'](tuple(values),moment)
    packed = _array(values)
    if packed is None:
        return None
    expected = 45 if moment else 24
    if len(packed) != expected:
        return None
    result = (D * 3)()
    if not (library.wt_moment if moment else library.wt_force)(packed, result):
        return None
    return list(result) if all(math.isfinite(v) for v in result) else None


def polar_batch(p, rows):
    """Evaluate [angle, rotation, added lift, drag multiplier] rows in one call."""
    library = load(__file__)
    if library is not None and library._python:
        result = library._python['batch'](p,rows)
        if result is not None:
            return result
    rows = [tuple(row) for row in rows]
    if any(len(row) != 4 for row in rows):
        raise ValueError('Each polar row must contain four values')
    if library is not None and library._python:
        result = library._python['batch'](p,rows)
        if result is not None:
            return result
    packed = _array(p[key] for key in FIELDS)
    inputs = _array(v for row in rows for v in row)
    if library is None or packed is None or inputs is None:
        from polar_f32 import calc_c
        return [calc_c(p, *row) for row in rows]
    result = (D * (len(rows)*2))()
    valid = library.wt_polar_batch(packed, inputs, len(rows), result)
    if not valid or not all(math.isfinite(v) for v in result):
        from polar_f32 import calc_c
        return [calc_c(p, *row) for row in rows]
    return [list(result[i:i+2]) for i in range(0, len(result), 2)]


def atmosphere_function(library, reference):
    if library._python:
        return bind_native(library,reference,8)
    def atmosphere(height):
        if not math.isfinite(height) or abs(height)>3.4028234663852886e38:
            return reference(height)
        result = (D * 3)()
        library.wt_atmosphere(height, result)
        if not all(math.isfinite(v) for v in result):
            return reference(height)
        return dict(zip(('density', 'sound_speed', 'pressure'), result))
    return atmosphere


def orientation_function(library, reference):
    if library._python:
        return bind_native(library,reference,9)
    def orientation(quaternion, increment):
        values = (*quaternion, *increment)
        packed = _array(values)
        if packed is None or len(quaternion)!=4 or len(increment)!=3:
            return reference(quaternion, increment)
        result = (D * 24)()
        if not library.wt_orientation(packed, result) or not all(math.isfinite(v) for v in result):
            return reference(quaternion, increment)
        trig = [dict(sine=result[i],cosine=result[i+1],quadrant=int(result[i+2]),reduced=result[i+3]) for i in (0,4,8)]
        return dict(trig=trig,delta=list(result[12:16]),raw=list(result[16:20]),quaternion=list(result[20:24]))
    return orientation

def aero_function(library, reference):
    if library._python:
        return bind_native(library,reference,12)
    fields=('stabilizer_arm','cx','cx_aoa','cy','cy_limit','front_area','side_area','fins_hor','fins_ver','fin_pressure_limit','damping_geometry','mass')
    def forces(props,height,velocity,q,omega,**kwargs):
        defaults=dict(wind=(0.,0.,0.),fins=(0.,0.),additional_cx=0.,additional_lever=0.,dt=1/48,torque=(0.,0.,0.),force=(0.,0.,0.),mass_lost=0.,gravity=True,use_cxi=True,mass_term=0.,angular_environment=(0.,0.,0.),perturbation=0.,body_random=0.)
        if kwargs.keys()-defaults.keys():return reference(props,height,velocity,q,omega,**kwargs)
        defaults.update(kwargs);k=defaults
        try:
            vectors=(props['axis_quaternion'],props['inertia'],props['angular_damping'],velocity,q,omega,k['wind'],k['fins'],k['torque'],k['force'],k['angular_environment'])
            if tuple(map(len,vectors))!=(4,3,3,3,4,3,3,2,3,3,3):raise ValueError('vector shape')
            rows=props['cy_table']
            if any(len(row)!=3 for row in rows):raise ValueError('table shape')
            packed=_array([*(props[key] for key in fields),*vectors[0],*vectors[1],*vectors[2]])
            inputs=_array([height,*velocity,*q,*omega,*k['wind'],*k['fins'],k['additional_cx'],k['additional_lever'],k['dt'],*k['torque'],*k['force'],k['mass_lost'],float(k['gravity']),float(k['use_cxi']),k['mass_term'],*k['angular_environment'],k['perturbation'],k['body_random']])
            table=_array(v for row in rows for v in row)
        except (KeyError,TypeError,ValueError):
            return reference(props,height,velocity,q,omega,**kwargs)
        result=(D*85)()
        if packed is None or inputs is None or table is None or not library.wt_aero(packed,inputs,table,len(rows),result):
            return reference(props,height,velocity,q,omega,**kwargs)
        r=list(result)
        def evaluated(i):return dict(drag=r[i:i+3],lift=r[i+3:i+6],force=r[i+6:i+9],cosine=r[i+9],cd=r[i+10],cy=r[i+11])
        baseline=evaluated(27);flow=r[24:27];active=bool(r[39])
        return dict(frame=[r[i:i+3] for i in (0,3,6)],axes=[r[i:i+3] for i in (9,12,15)],cm_speed=r[18],mach=r[19],pressure=r[20],local_flow=r[21:24],flow=flow,baseline=baseline,fin_active=active,fin_limited=bool(r[40]),deflection=r[41:43],fin_flow=r[43:46] if active else flow,steering=evaluated(46) if active else baseline,moment=r[58:61],damping=r[61:64],damping_clipped=list(map(bool,r[64:67])),angular_acceleration_before_environment=r[67:70],angular_acceleration=r[70:73],acceleration=r[73:76],effective_mass=r[76],perturbation=dict(zip(('force_scale','angle','cosine','sine','lever_fraction'),r[77:82])),perturbed_lever=r[82:85])
    return forces


def matrix_quaternion_function(library, reference):
    if library._python:
        return bind_native(library,reference,10)
    def matrix_quaternion(forward,up,right):
        if any(len(row)!=3 for row in (forward,up,right)):
            return reference(forward,up,right)
        packed=_array((*forward,*up,*right))
        result=(D*4)()
        if packed is None or not library.wt_matrix_quaternion(packed,result):
            return reference(forward,up,right)
        return list(result)
    return matrix_quaternion


def vector_function(library, reference, mode):
    if library._python:
        return bind_native(library,reference,5+mode)
    def vector(q,value,*rest):
        if len(q)!=4 or len(value)!=3 or (mode!=1 and rest) or (mode==1 and (len(rest)!=1 or len(rest[0])!=3)):
            return reference(q,value,*rest)
        packed=_array((*q,*value,*(rest[0] if mode==1 else (0.,0.,0.))))
        result=(D*3)()
        if packed is None or not library.wt_vector(packed,mode,result):
            return reference(q,value,*rest)
        return list(result)
    return vector


def integrate_function(library, reference):
    if library._python:
        return bind_native(library,reference,11)
    def integrate(state,acceleration,angular_acceleration,dt,absolute_time,clock_rates=(0.,)*4):
        args=(state,acceleration,angular_acceleration,dt,absolute_time,clock_rates)
        try:
            vectors=(state['position'],state['velocity'],state['omega'],state['quaternion'],state['clocks'],acceleration,angular_acceleration,clock_rates)
            if tuple(map(len,vectors))!=(3,3,3,4,4,3,3,4):return reference(*args)
            packed=_array((*vectors[0],*vectors[1],*vectors[2],*vectors[3],state['time'],*vectors[4],state['distance'],state['water_distance'],float(state['water']),*acceleration,*angular_acceleration,dt,absolute_time,*clock_rates))
        except (KeyError,TypeError,ValueError):
            return reference(*args)
        result=(D*51)()
        if packed is None or not library.wt_integrate(packed,result):return reference(*args)
        r=list(result);rotation=r[27:];quaternion=rotation[20:24]
        trig=[dict(sine=rotation[i],cosine=rotation[i+1],quadrant=int(rotation[i+2]),reduced=rotation[i+3]) for i in (0,4,8)]
        return dict(state=dict(position=r[:3],velocity=r[3:6],omega=r[6:9],quaternion=quaternion,time=r[13],clocks=r[14:18],distance=r[18],water_distance=r[19],immersion=r[20]),displacement=r[21:24],increment=r[24:27],rotation=dict(trig=trig,delta=rotation[12:16],raw=rotation[16:20],quaternion=quaternion))
    return integrate


def controller_function(library, reference):
    import acceleration_control
    def update(p,aero,motor,state,request,measured,q,velocity,height,time,dt,*,environment=None,matrix_velocity_frame=True):
        args=(p,aero,motor,state,request,measured,q,velocity,height,time,dt)
        kwargs=dict(environment=environment,matrix_velocity_frame=matrix_velocity_frame)
        try:
            coefficients=acceleration_control.pid_coefficients(p,time)
            if coefficients is None:return reference(*args,**kwargs)
            frame=acceleration_control.frame(q,velocity,p['velocity_frame'],matrix=matrix_velocity_frame)
            env=dict(height_scale=18300.,reference_density=1.225,sea_density=1.225,sea_temperature=288.16)|(environment or {})
            if tuple(map(len,(frame,request,measured,velocity,state,*state,coefficients)))!=(4,3,3,3,2,8,8,4):return reference(*args,**kwargs)
            rows=aero['cy_table'] if aero is not None and p['limit_aoa'] else []
            if any(len(row)!=3 for row in rows):return reference(*args,**kwargs)
            av=[aero[key] for key in ('mass','cy','cy_limit','side_area')] if aero is not None and p['limit_aoa'] else [0.]*4
            motor=motor or dict(thrust=0.,mass_lost=0.)
            packed=_array((*frame,*request,*measured,*velocity,height,dt,p['max_accel'],float(p['limit_aoa']),p['aoa_max'],p['base_speed_squared'],*(env[key] for key in ('height_scale','reference_density','sea_density','sea_temperature')),*av,motor['thrust'],motor['mass_lost'],*state[0],*state[1],*coefficients,float(aero is not None)))
            table=_array(x for row in rows for x in row)
        except (KeyError,TypeError,ValueError):return reference(*args,**kwargs)
        result=(D*25)()
        if packed is None or table is None or not library.wt_controller(packed,table,len(rows),result):return reference(*args,**kwargs)
        r=list(result)
        return dict(frame=frame,fins=r[:2],state=[r[2:10],r[10:18]],errors=r[18:20],limited_request=r[20:22],aoa_limit=r[22] if aero is not None and p['limit_aoa'] else None,scale=r[23],scale_squared=r[24],coefficients=coefficients)
    return update


def seeker_functions(library, reference_update, reference_slew):
    fields=('angle_max','lock_angle_max','rate_max','alpha','beta','gate_rate')
    def slew(p,desired,angles,dt,lock_limit,authored_rate):
        if len(desired)!=3 or len(angles)!=2:return reference_slew(p,desired,angles,dt,lock_limit,authored_rate)
        packed=_array((*(p[k] for k in fields),*desired,*angles,dt,float(lock_limit),float(authored_rate)))
        r=(D*2)()
        if packed is None or not library.wt_slew(packed,r):return reference_slew(p,desired,angles,dt,lock_limit,authored_rate)
        return list(r)
    def update(p,quaternion,measurement,state,dt,*,lock_limit=False,authored_rate=True,coast=False):
        args=(p,quaternion,measurement,state,dt);kwargs=dict(lock_limit=lock_limit,authored_rate=authored_rate,coast=coast)
        try:
            desired=(0.,0.,0.) if coast else measurement
            if tuple(map(len,(quaternion,desired,state['angles'],state['direction'],state['angular_rate'])))!=(4,3,2,3,3):return reference_update(*args,**kwargs)
            packed=_array((*(p[k] for k in fields),*quaternion,*desired,*state['angles'],*state['direction'],*state['angular_rate'],dt,float(lock_limit),float(authored_rate),float(coast)))
        except (KeyError,TypeError,ValueError):return reference_update(*args,**kwargs)
        r=(D*9)()
        if packed is None or not library.wt_seeker(packed,r):return reference_update(*args,**kwargs)
        return dict(angles=list(r[:2]),direction=list(r[2:5]),angular_rate=list(r[5:8]),accepted=None if coast else bool(r[8]))
    return update,slew
