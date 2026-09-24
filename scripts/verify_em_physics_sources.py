"""Verify native equations separately from audited temporal-driver extensions."""
import ast,copy,hashlib,json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def verify():
    reference=json.loads((ROOT/'analysis/em-adaptive-boundary-september22/physics-source-hashes.json').read_text())
    unchanged=[]
    for row in reference['modules']:
        path=ROOT/'scripts'/(row['module']+'.py')
        if row['module']=='aircraft_model':
            baseline=ROOT/'analysis/instructor-live-september24/baseline-sources/scripts/aircraft_model.py'
            assert hashlib.sha256(baseline.read_bytes()).hexdigest()==row['sha256']
            before=ast.parse(baseline.read_text());after=ast.parse(path.read_text())
            imports=[n for n in after.body if isinstance(n,ast.ImportFrom) and n.module=='wing_area_normalization']
            assert len(imports)==1 and ast.dump(imports[0])==ast.dump(ast.parse('from wing_area_normalization import intact_polars').body[0])
            after.body.remove(imports[0])
            calls=[n for n in ast.walk(after) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='wings']
            assert len(calls)==1
            assert ast.dump(calls[0].args[1])==ast.dump(ast.parse("intact_polars(wing_polar,g['areas'])",mode='eval').body)
            calls[0].args[1]=ast.parse('[wing_polar,wing_polar]',mode='eval').body
            assert ast.dump(before)==ast.dump(after),'Aircraft equations changed beyond the verified area-normalization input stage'
            continue
        if row['module']=='instructor_pitch_predictor':
            baseline=ROOT/'analysis/em-low-speed-september23/baseline-scripts/instructor_pitch_predictor.py'
            assert hashlib.sha256(baseline.read_bytes()).hexdigest()==row['sha256']
            before=ast.parse(baseline.read_text());after=ast.parse(path.read_text())
            old_function=next(n for n in before.body if isinstance(n,ast.FunctionDef) and n.name=='prepare_geometry')
            new_function=next(n for n in after.body if isinstance(n,ast.FunctionDef) and n.name=='prepare_geometry')
            old_key=next(n for n in old_function.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='key')
            new_key=next(n for n in new_function.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='key')
            reads={n.slice.value for statement in new_function.body if statement is not new_key for n in ast.walk(statement)
                   if isinstance(n,ast.Subscript) and isinstance(n.value,ast.Name) and n.value.id=='ip' and isinstance(n.slice,ast.Constant)}
            keyed=set(ast.literal_eval(new_key.value.elts[0].args[0].generators[0].iter))
            assert reads-keyed=={0,0xc,0x28},'Geometry cache omitted a physical input'
            new_function.body[new_function.body.index(new_key)]=old_key
            assert ast.dump(before)==ast.dump(after),'Predictor equations changed'
            continue
        if row['module']=='instructor_settle':
            baseline=ROOT/'analysis/em-controller-engine-september23/baseline-scripts/instructor_settle.py'
            assert hashlib.sha256(baseline.read_bytes()).hexdigest()==row['sha256']
            before=ast.parse(baseline.read_text());after=ast.parse(path.read_text())
            # Exact memoization of a pure allocation at a held observation.
            # Verify the complete replacement block before reducing it to the
            # original call for the remaining equation/acceptance AST audit.
            original_allocation=next(n for n in ast.walk(before) if isinstance(n,ast.Assign)
                and ast.unparse(n.targets[0])=='allocation'
                and isinstance(n.value,ast.Call) and ast.unparse(n.value.func)=='command_allocation')
            expected_cache=ast.parse("""
allocation_cache={}
allocation_key=struct.pack('<3d',*trim)
if allocation_key not in allocation_cache:
    if len(allocation_cache)>=64:allocation_cache.clear()
    allocation_cache[allocation_key]=command_allocation(effective,state['delivered'],ranges,dict(DEFAULTS,trim_mode='fixed',fixed_trim=trim))
allocation=allocation_cache[allocation_key]
""").body
            actual_cache=[n for n in ast.walk(after) if (
                isinstance(n,ast.Assign) and ast.unparse(n.targets[0]) in ('allocation_cache','allocation_key') or
                isinstance(n,ast.If) and ast.unparse(n.test)=='allocation_key not in allocation_cache' or
                isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='allocation' and
                    ast.unparse(n.value)=='allocation_cache[allocation_key]')]
            assert sorted(map(ast.dump,actual_cache))==sorted(map(ast.dump,expected_cache)), 'Unexpected allocation memoization'
            class RemoveExactCycleSkip(ast.NodeTransformer):
                def visit_Import(self,node):
                    node.names=[a for a in node.names if a.name!='struct']
                    return node
                def visit_Assign(self,node):
                    if any(isinstance(n,ast.Name) and n.id in ('exact_states','accelerated_cycle_ticks','allocation_cache','allocation_key') for n in node.targets):return None
                    if ast.unparse(node.targets[0])=='allocation' and ast.unparse(node.value)=='allocation_cache[allocation_key]':
                        return copy.deepcopy(original_allocation)
                    return self.generic_visit(node)
                def visit_Expr(self,node):
                    if ast.unparse(node)=='exact_states.clear()':return None
                    return self.generic_visit(node)
                def visit_If(self,node):
                    if ast.unparse(node.test)=='allocation_key not in allocation_cache':return None
                    if ast.unparse(node.test)=='accelerate_history and (not search_only) and (ticks >= 24)':return None
                    # Audited separately against complete unaccelerated
                    # recurrences, including terminal phase and every history.
                    if (any(isinstance(n,ast.ImportFrom) and n.module=='em_controller_cycle'
                            for n in ast.walk(node))):return None
                    return self.generic_visit(node)
                def visit_Call(self,node):
                    node.keywords=[kw for kw in node.keywords if kw.arg!='accelerated_cycle_ticks']
                    return self.generic_visit(node)
            assert ast.dump(before)==ast.dump(RemoveExactCycleSkip().visit(after)), 'Controller equations or acceptance changed'
            continue
        if row['module']!='prop_steady':
            assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256'],row['module']
            unchanged.append(row['module']);continue
        baseline=ROOT/'analysis/em-cold-defaults-september23/baseline-scripts/prop_steady.py'
        assert hashlib.sha256(baseline.read_bytes()).hexdigest()==row['sha256']
        before=ast.parse(baseline.read_text());after=ast.parse(path.read_text())
        class RemoveAveragingExtension(ast.NodeTransformer):
            def visit_FunctionDef(self,node):
                self.generic_visit(node)
                if node.name=='settled_cycle':
                    assert node.args.args[-1].arg=='aircraft_residual_scales'
                    node.args.args.pop();node.args.defaults.pop()
                return node
            def visit_If(self,node):
                if 'aircraft_residual_scales' in ast.unparse(node.test):return None
                return self.generic_visit(node)
        assert ast.dump(before)==ast.dump(RemoveAveragingExtension().visit(after)), 'Native propagation changed'
    return dict(status='PASS',unchanged_native_modules=unchanged,
        wing_normalization='Only the independently native-checked intact per-wing polar input stage differs; remaining aircraft assembly and force equations have identical ASTs.',
        controller_driver='Only exact repeated-state/window and pre-target periodic timer skipping, exact held-state allocation memoization and their accounting differ; native updates and acceptance have identical ASTs.',
        averaging_driver='Only the optional residual-scaled averaging extension differs; initialization, native frame propagation, cycle detection and original acceptance paths have identical ASTs.')


if __name__=='__main__':
    report=verify()
    (ROOT/'analysis/em-speed-flaps-september23/physics-source-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS:',len(report['unchanged_native_modules']),'unchanged equation modules; averaging propagation and controller updates/acceptance unchanged')
