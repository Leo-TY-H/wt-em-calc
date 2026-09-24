"""Generate a typed blade consumer from the unchanged readable FM equations.

Only data representation changes: prepared polar fields live in a Cython
record, avoiding a new Python dictionary and string lookups at each of four
blade stations. Keep this transformation separate from the reference physics.
"""
import ast


def generate(root,helpers,math_helpers,scalar_declarations):
    def function(module,name):
        tree=ast.parse((root/'scripts'/(module+'.py')).read_text())
        return next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    tree=ast.parse((root/'scripts/polar_model.py').read_text())
    field_node=next(n.value for n in tree.body if isinstance(n,ast.Assign)
                    and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in n.targets))
    assert isinstance(field_node,ast.Call) and isinstance(field_node.func,ast.Attribute) and field_node.func.attr=='split'
    fields=ast.literal_eval(field_node.func.value).split()
    class TypedFields(ast.NodeTransformer):
        def visit_Subscript(self,node):
            if not isinstance(node.value,ast.Name) or node.value.id!='p':return self.generic_visit(node)
            def field(key):
                if isinstance(key,ast.Constant) and key.value in fields:
                    return ast.Attribute(value=ast.Name(id='p',ctx=ast.Load()),attr=key.value,ctx=ast.Load())
                if isinstance(key,ast.IfExp):return ast.IfExp(test=key.test,body=field(key.body),orelse=field(key.orelse))
                if (isinstance(key,ast.BinOp) and isinstance(key.op,ast.Add) and isinstance(key.left,ast.Constant)
                    and isinstance(key.right,ast.Name) and key.right.id=='suffix'):
                    return ast.IfExp(test=ast.Name(id='positive',ctx=ast.Load()),
                        body=field(ast.Constant(key.left.value+'H')),orelse=field(ast.Constant(key.left.value+'L')))
                raise ValueError('Unexpected blade polar access: '+ast.unparse(node))
            return ast.copy_location(field(node.slice),node)
    polar=function('polar_runtime','evaluate')
    assert isinstance(polar.body[-1],ast.Return) and ast.unparse(polar.body[-1].value)=='dict(zip(FIELDS, values))'
    values=next(n.value.elts for n in polar.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='values' for t in n.targets))
    polar.body=[n for n in polar.body if not (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='values' for t in n.targets))][:-1]
    assignments='p = BladePolar.__new__(BladePolar)\n'+'\n'.join('p.'+k+' = '+ast.unparse(v) for k,v in zip(fields,values))+'\nreturn p'
    polar.body.extend(ast.parse(assignments).body)
    polar.name='_polar'
    text=helpers+'\n'+math_helpers+'''\nimport math
from collections import OrderedDict
from functools import lru_cache
from piston_model cimport div
from polar_runtime cimport mach_value
from control_mixer import curve

cdef class BladePolar:
    cdef double '''+', '.join(fields)+'\n\n'
    text+=ast.unparse(ast.fix_missing_locations(polar))+'\n\n'
    header,locals_=scalar_declarations['polar_runtime']['evaluate(runtime, mach, cy_mult=1.0)']
    text=text.replace('def _polar(runtime, mach, cy_mult=1.0):',
                      'cdef BladePolar _polar(runtime, double mach, double cy_mult=1.0):\n    cdef BladePolar p\n    cdef double '+locals_)
    for name in ('calc_cl','calc_cd'):
        node=TypedFields().visit(function('polar_f32',name));node.name='_'+name
        code=ast.unparse(ast.fix_missing_locations(node))
        _,locals_=scalar_declarations['polar_f32'][name+'(p, a)']
        code=code.replace('def _'+name+'(p, a):','cdef inline double _'+name+'(BladePolar p, double a):\n    cdef double '+locals_)
        if name=='calc_cl':code=code.replace('    cdef double ', '    cdef bint positive\n    cdef double ',1)
        text+=code+'\n\n'
    text+='cdef inline double sin(double x):\n    return f32(_math_sin(x))\n\n'
    shape=function('propeller_model','legacy_stall_shape');shape.decorator_list=[];shape.name='_legacy_stall_shape'
    code=ast.unparse(shape).replace("polar = evaluate(dict(base=base, mode=0), 0.0)","polar = _polar(dict(base=base, mode=0), 0.0)")
    code=code.replace("return {k: polar[k] for k in ['aoaLineH', 'aoaLineL', 'parabCyCoeffH', 'parabCyCoeffL']}",
                      'return (polar.aoaLineH, polar.aoaLineL, polar.parabCyCoeffH, polar.parabCyCoeffL)')
    code=code.replace('def _legacy_stall_shape():','@lru_cache(maxsize=1)\ndef _legacy_stall_shape():\n    cdef BladePolar polar')
    text+=code+'\n\n_blade_polars = OrderedDict()\n\n'
    cached=function('propeller_model','blade_polar');cached.name='_blade_polar'
    code=ast.unparse(cached).replace('polar = evaluate(runtime, mach)','polar = _polar(runtime, mach)')
    code=code.replace('polar.update(legacy_stall_shape())','polar.aoaLineH, polar.aoaLineL, polar.parabCyCoeffH, polar.parabCyCoeffL = _legacy_stall_shape()')
    code=code.replace('def _blade_polar(runtime, mach):','cdef BladePolar _blade_polar(runtime, double mach):\n    cdef BladePolar polar')
    text+=code+'\n\n'
    blades=function('propeller_model','blade_forces');code=ast.unparse(blades)
    key=next(iter(scalar_declarations['propeller_model']));header,locals_=scalar_declarations['propeller_model'][key]
    original='def '+key+':'
    assert original in code
    code=code.replace(original,header.replace('def blade_forces','cpdef blade_forces')+':\n    cdef BladePolar polar\n    cdef double '+locals_)
    code=code.replace('blade_polar(', '_blade_polar(').replace('calc_cl(', '_calc_cl(').replace('calc_cd(', '_calc_cd(')
    code=code.replace("polar['clKq']",'polar.clKq').replace("polar['kq']",'polar.kq')
    text+=code+'\n'
    for name in ('sin','cos','tan','sqrt','asin','acos','atan','atan2'):
        text=text.replace('math.'+name+'(', '_math_'+name+'(')
    return text
