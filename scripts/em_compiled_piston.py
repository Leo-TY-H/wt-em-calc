"""Typed representation of the unchanged healthy piston and compressor kernels.

Generate from the reference AST; retain every original branch, rounded operation,
and returned diagnostic. Only immutable property reads and inlet preparation move.
"""
import ast
import copy


def generate(root,helpers,math_helpers,declarations):
    selected={
        'piston_model':('boost_active','rpm_torque','mixture'),
        'piston_compressor':('speed_factor','requested_pressure','low_rpm','candidate','step'),
        'piston_general':('reservoir_torque','step'),
    }
    nodes=[]
    for module,names in selected.items():
        functions={n.name:n for n in ast.parse((root/'scripts'/(module+'.py')).read_text()).body if isinstance(n,ast.FunctionDef)}
        nodes.extend((module,copy.deepcopy(functions[name])) for name in names)
    fields=sorted({n.slice.value for _,node in nodes for n in ast.walk(node)
                   if isinstance(n,ast.Subscript) and isinstance(n.value,ast.Name) and n.value.id=='p' and isinstance(n.slice,ast.Constant)})
    objects={'stages','ata','consumption'}
    integral={'family','carburetor','manual_compressor','compressor_type','mixer_type','boost_type','ata_enabled','exact_altitudes','cylinders'}
    assert objects<=set(fields)
    text=helpers+'\n'+math_helpers+'''
import math
from collections import OrderedDict
from piston_model cimport div, inlet_pressure, pressure_at_height
from control_mixer import curve
from structural_limits import interval
from engine_supply cimport mechanical_multiplier

cdef class PistonCondition:
    cdef object source
    cdef double inlet
'''
    for kind,keys in [('object',objects),('long',integral),('double',set(fields)-objects-integral)]:
        text+='    cdef '+kind+' '+', '.join(sorted(keys))+'\n'
    text+='\n    def __init__(self, source, axial, height):\n        self.source=source\n'
    for field in fields:text+="        self."+field+"=source['"+field+"']\n"
    text+='        self.inlet=inlet_pressure(height,axial,self.ram_recovery)\n'
    text+='''
_contexts=OrderedDict()

cdef PistonCondition condition(p, velocity, double height):
    key=(id(p),velocity[0],height)
    result=_contexts.get(key)
    if result is None:
        result=PistonCondition(p,velocity[0],height)
        if len(_contexts)>=256: _contexts.popitem(last=False)
        _contexts[key]=result
    return result

def clear_contexts():
    _contexts.clear()

'''
    class Fields(ast.NodeTransformer):
        def visit_Subscript(self,node):
            if isinstance(node.value,ast.Name) and node.value.id=='p':
                assert isinstance(node.slice,ast.Constant) and node.slice.value in fields
                return ast.Attribute(value=ast.Name(id='p',ctx=ast.Load()),attr=node.slice.value,ctx=node.ctx)
            return self.generic_visit(node)
    for module,node in nodes:
        original_name=node.name
        key=next((key for key in declarations.get(module,{}) if key.startswith(original_name+'(')),None)
        if key:
            header,temps=declarations[module][key]
            header=header.replace('cpdef ', 'cdef ',1) if header.startswith('cpdef ') else header.replace('def ','cdef ',1)
            # An omitted return type in the existing Cython interface is object.
            if header.startswith('cdef '+original_name+'('):header=header.replace('cdef ','cdef object ',1)
        else:
            assert original_name=='reservoir_torque'
            header='cdef double reservoir_torque(PistonCondition p, double omega, double throttle, double multiplier, double reservoir, afterburner, gear, double nitro)'
            temps='effective, x, shape, half, fuel, tq'
        header=header.replace('(p,','(PistonCondition p,').replace(', *,',',')
        if module=='piston_compressor' and original_name=='step':header=header.replace(' step(', ' compressor(')
        if module=='piston_general' and original_name=='step':header=header.replace(' step(', ' _step(')
        code=ast.unparse(ast.fix_missing_locations(Fields().visit(node)))
        code=code.replace(code.splitlines()[0],header+':'+('\n    cdef double '+temps if temps else ''))
        if module=='piston_general' and original_name=='step':
            target='inlet = inlet_pressure(height, velocity[0], p.ram_recovery)'
            assert target in code
            code=code.replace(target,'inlet = p.inlet')
            code=code.replace('mechanical_multiplier(p,', 'mechanical_multiplier(p.source,')
            code=code.replace("automatic_turbo=s.get('automatic_turbo', True), height=height)",
                              "automatic_turbo=s.get('automatic_turbo', True), assisted=False, height=height)")
            code=code.replace("mixture(p, inlet, s.get('mixture', 0.5), automatic=",
                              "mixture(p, inlet, s.get('mixture', 0.5), rich_accumulator=0., automatic=")
        text+=code+'\n\n'
    text+='''
cpdef step(p, s, velocity=(100.,0.,0.), double height=0., double dt=1/48,
           seed=12345, double torque_multiplier=1., double nitro=0.):
    cdef PistonCondition context=condition(p,velocity,height)
    return _step(context,s,velocity,height,dt,seed,torque_multiplier,nitro)
'''
    return text
