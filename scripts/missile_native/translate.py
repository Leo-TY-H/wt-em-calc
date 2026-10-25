"""Build-time translation of the pinned game-model subset to standalone C++.

The emitted program uses only the native value runtime.  This module is a build
tool, never imported by the calculation executable. Unsupported syntax is a
build error with a source location; it cannot fall back to Python execution.
"""
from __future__ import annotations

import argparse
import ast
import json
import pathlib
from collections.abc import Mapping


class TranslationError(ValueError):
    pass


def quote(value):
    text = str(value)
    if text.isascii():
        return json.dumps(text, ensure_ascii=True)
    # Universal character names use the compiler's execution character set.
    # Byte escapes instead preserve UTF-8 even with MSVC's Windows code page.
    # Separate literals terminate each escape run before an ASCII hex digit.
    parts = []
    run = []
    ascii_run = None
    for character in text:
        is_ascii = ord(character) < 128
        if ascii_run is not None and is_ascii != ascii_run:
            chunk = ''.join(run)
            parts.append(json.dumps(chunk, ensure_ascii=True) if ascii_run else
                         '"' + ''.join(f'\\x{byte:02x}' for byte in chunk.encode('utf-8')) + '"')
            run = []
        ascii_run = is_ascii
        run.append(character)
    chunk = ''.join(run)
    parts.append(json.dumps(chunk, ensure_ascii=True) if ascii_run else
                 '"' + ''.join(f'\\x{byte:02x}' for byte in chunk.encode('utf-8')) + '"')
    return ' '.join(parts)


class Scope:
    def __init__(self, env='e', *, globals_=(), nonlocals=(), class_env=None,
                 method_class=None, synchronous_yield=False):
        self.env = env
        self.globals = set(globals_)
        self.nonlocals = set(nonlocals)
        self.class_env = class_env
        self.method_class = method_class
        self.synchronous_yield = synchronous_yield
        self.cached_locals = {}


class Translator:
    def __init__(self):
        self.lines = []
        self.indent = 0
        self.counter = 0
        self.scope = Scope()
        self.filename = '<source>'
        self.module = ''
        self.context = []
        self.loops = []

    def fail(self, node, message=None):
        raise TranslationError(f'{self.filename}:{getattr(node, "lineno", 0)}: '
                               f'{message or "unsupported " + type(node).__name__}')

    def emit(self, line=''):
        self.lines.append('  ' * self.indent + line)

    def fresh(self, stem='v'):
        self.counter += 1
        return f'_{stem}{self.counter}'

    def value(self, expression):
        name = self.fresh()
        self.emit(f'V {name} = {expression};')
        return name

    def block(self, statements):
        for node in statements:
            self.stmt(node)

    def literal(self, node):
        value = node.value
        if value is None:
            return 'V()'
        if isinstance(value, bool):
            return f'V({str(value).lower()})'
        if isinstance(value, int):
            if not -(1 << 63) <= value < 1 << 63:
                self.fail(node, 'integer literal exceeds native signed 64-bit range')
            if value == -(1 << 63):
                return 'V(INT64_MIN)'
            return f'V(int64_t({value}))'
        if isinstance(value, float):
            return f'V({value.hex()})'
        if isinstance(value, str):
            return f'V({quote(value)})'
        if isinstance(value, bytes):
            encoded = ''.join(f'\\x{x:02x}' for x in value)
            return f'make_bytes(std::string("{encoded}", {len(value)}))'
        self.fail(node)

    def expr(self, node):
        if isinstance(node, ast.Constant):
            return self.value(self.literal(node))
        if isinstance(node, ast.Name):
            if node.id in self.scope.cached_locals:
                return self.value(f'read_local({self.scope.env}, {quote(node.id)}, {self.scope.cached_locals[node.id]})')
            return self.value(f'get({self.scope.env}, {quote(node.id)})')
        if isinstance(node, ast.Attribute):
            base = self.expr(node.value)
            return self.value(f'attr({base}, {quote(node.attr)})')
        if isinstance(node, ast.Subscript):
            base = self.expr(node.value)
            if isinstance(node.slice, ast.Slice):
                bounds = [self.expr(x) if x is not None else 'V()'
                          for x in (node.slice.lower, node.slice.upper, node.slice.step)]
                return self.value(f'slice({base}, {", ".join(bounds)})')
            key = self.expr(node.slice)
            return self.value(f'item({base}, {key})')
        if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
            args = self.fresh('args')
            self.emit(f'Args {args};')
            for element in node.elts:
                if isinstance(element, ast.Starred):
                    value = self.expr(element.value)
                    self.emit(f'for (const V& x : iter({value})) {args}.push_back(x);')
                else:
                    self.emit(f'{args}.push_back({self.expr(element)});')
            kind = {ast.List: 'list', ast.Tuple: 'tuple', ast.Set: 'set'}[type(node)]
            return self.value(f'make_{kind}(std::move({args}))')
        if isinstance(node, ast.Dict):
            result = self.value('make_dict()')
            for key, value in zip(node.keys, node.values):
                if key is None:
                    self.emit(f'dict_merge({result}, {self.expr(value)});')
                else:
                    k = self.expr(key)
                    v = self.expr(value)
                    self.emit(f'dict_set({result}, {k}, {v});')
            return result
        if isinstance(node, ast.BinOp):
            left = self.expr(node.left)
            right = self.expr(node.right)
            return self.value(f'binary({quote(type(node.op).__name__)}, {left}, {right})')
        if isinstance(node, ast.UnaryOp):
            value = self.expr(node.operand)
            return self.value(f'unary({quote(type(node.op).__name__)}, {value})')
        if isinstance(node, ast.BoolOp):
            result = self.expr(node.values[0])
            for value in node.values[1:]:
                condition = f'truth({result})'
                if isinstance(node.op, ast.Or):
                    condition = '!' + condition
                self.emit(f'if ({condition}) {{')
                self.indent += 1
                other = self.expr(value)
                self.emit(f'{result} = {other};')
                self.indent -= 1
                self.emit('}')
            return result
        if isinstance(node, ast.Compare):
            previous = self.expr(node.left)
            result = self.value('V(true)')
            for op, comparator in zip(node.ops, node.comparators):
                self.emit(f'if (truth({result})) {{')
                self.indent += 1
                other = self.expr(comparator)
                self.emit(f'{result} = V(compare({quote(type(op).__name__)}, {previous}, {other}));')
                self.emit(f'{previous} = {other};')
                self.indent -= 1
                self.emit('}')
            return result
        if isinstance(node, ast.IfExp):
            result = self.value('V()')
            test = self.expr(node.test)
            self.emit(f'if (truth({test})) {{')
            self.indent += 1
            body = self.expr(node.body)
            self.emit(f'{result} = {body};')
            self.indent -= 1
            self.emit('} else {')
            self.indent += 1
            other = self.expr(node.orelse)
            self.emit(f'{result} = {other};')
            self.indent -= 1
            self.emit('}')
            return result
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id == 'super' and not node.args and not node.keywords:
                if not self.scope.method_class:
                    self.fail(node, 'zero-argument super outside method')
                selfv = self.value(f'get({self.scope.env}, "self")')
                clsv = self.value(f'get({self.scope.env}, {quote(self.scope.method_class)})')
                return self.value(f'super_of({clsv}, {selfv})')
            fn = self.expr(node.func)
            if not node.keywords and len(node.args) in (1, 2) and not any(isinstance(x, ast.Starred) for x in node.args):
                arguments = [self.expr(argument) for argument in node.args]
                return self.value(f'call{len(arguments)}({fn}, {", ".join(arguments)})')
            args, kwargs = self.fresh('args'), self.fresh('kw')
            self.emit(f'Args {args}; Kw {kwargs};')
            if node.args:
                self.emit(f'{args}.reserve({len(node.args)});')
            if node.keywords:
                self.emit(f'{kwargs}.reserve({len(node.keywords)});')
            for argument in node.args:
                if isinstance(argument, ast.Starred):
                    value = self.expr(argument.value)
                    self.emit(f'for (const V& x : iter({value})) {args}.push_back(x);')
                else:
                    value = self.expr(argument)
                    self.emit(f'{args}.push_back({value});')
            for keyword in node.keywords:
                value = self.expr(keyword.value)
                if keyword.arg is None:
                    self.emit(f'for (const V& k : iter({value})) {kwargs}.emplace_back(k.as_string(), item({value}, k));')
                else:
                    self.emit(f'{kwargs}.emplace_back({quote(keyword.arg)}, {value});')
            return self.value(f'call({fn}, {args}, {kwargs})')
        if isinstance(node, ast.Lambda):
            return self.function(node, '<lambda>', [ast.Return(value=node.body)])
        if isinstance(node, (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)):
            return self.comprehension(node)
        if isinstance(node, ast.JoinedStr):
            result = self.value('V("")')
            for child in node.values:
                value = self.expr(child)
                self.emit(f'{result} = binary("Add", {result}, {value});')
            return result
        if isinstance(node, ast.FormattedValue):
            value = self.expr(node.value)
            if node.conversion in (114, 97):
                value = self.value(f'V(repr({value}))')
            elif node.conversion == 115:
                value = self.value(f'V(str({value}))')
            if node.format_spec is not None:
                spec = self.expr(node.format_spec)
                return self.value(f'call(get({self.scope.env}, "format"), Args{{{value}, {spec}}})')
            return self.value(f'V(str({value}))')
        if isinstance(node, ast.NamedExpr):
            value = self.expr(node.value)
            self.assign(node.target, value)
            return value
        if isinstance(node, ast.Yield):
            if not self.scope.synchronous_yield:
                self.fail(node, 'yield requires a supported generator frame')
            value = self.expr(node.value) if node.value is not None else 'V()'
            return self.value(f'call(get({self.scope.env}, "__yield__"), Args{{{value}}})')
        if isinstance(node, ast.YieldFrom):
            if not self.scope.synchronous_yield:
                self.fail(node, 'yield from is only supported in synchronous Ray search')
            return self.expr(node.value)
        self.fail(node)

    def set_name(self, name, value):
        if name in self.scope.nonlocals:
            self.emit(f'set_outer({self.scope.env}, {quote(name)}, {value});')
        elif name in self.scope.globals:
            self.emit(f'set(root_env({self.scope.env}), {quote(name)}, {value});')
        elif name in self.scope.cached_locals:
            self.emit(f'write_local({self.scope.env}, {quote(name)}, {self.scope.cached_locals[name]}, {value});')
        else:
            self.emit(f'set({self.scope.env}, {quote(name)}, {value});')

    def assign(self, target, value):
        if isinstance(target, ast.Name):
            self.set_name(target.id, value)
        elif isinstance(target, (ast.Tuple, ast.List)):
            values = self.fresh('unpack')
            self.emit(f'Args {values} = iter({value});')
            self.emit(f'if ({values}.size() != {len(target.elts)}) throw PyError("ValueError", "unpack length mismatch");')
            for i, element in enumerate(target.elts):
                self.assign(element, f'{values}[{i}]')
        elif isinstance(target, ast.Attribute):
            base = self.expr(target.value)
            self.emit(f'setattr({base}, {quote(target.attr)}, {value});')
        elif isinstance(target, ast.Subscript):
            base = self.expr(target.value)
            if isinstance(target.slice, ast.Slice):
                bounds = [self.expr(x) if x is not None else 'V()'
                          for x in (target.slice.lower, target.slice.upper, target.slice.step)]
                self.emit(f'setslice({base}, {", ".join(bounds)}, {value});')
            else:
                key = self.expr(target.slice)
                self.emit(f'setitem({base}, {key}, {value});')
        else:
            self.fail(target, 'unsupported assignment target')

    @staticmethod
    def declarations(body):
        globals_, nonlocals = set(), set()
        class Visit(ast.NodeVisitor):
            def visit_Global(self, node): globals_.update(node.names)
            def visit_Nonlocal(self, node): nonlocals.update(node.names)
            def visit_FunctionDef(self, node): pass
            def visit_Lambda(self, node): pass
            def visit_ClassDef(self, node): pass
        visitor = Visit()
        for node in body:
            visitor.visit(node)
        return globals_, nonlocals

    @staticmethod
    def own_yields(body):
        found = []
        class Visit(ast.NodeVisitor):
            def visit_Yield(self, node): found.append(node)
            def visit_YieldFrom(self, node): found.append(node)
            def visit_FunctionDef(self, node): pass
            def visit_Lambda(self, node): pass
            def visit_ClassDef(self, node): pass
        visitor = Visit()
        for node in body:
            visitor.visit(node)
        return found

    @staticmethod
    def local_names(node, body):
        names = {a.arg for a in node.args.posonlyargs + node.args.args + node.args.kwonlyargs}
        for argument in (node.args.vararg, node.args.kwarg):
            if argument: names.add(argument.arg)
        class Visit(ast.NodeVisitor):
            def visit_Name(self, item):
                if isinstance(item.ctx, (ast.Store, ast.Del)): names.add(item.id)
            def visit_FunctionDef(self, item): names.add(item.name)
            def visit_ClassDef(self, item): names.add(item.name)
            def visit_Lambda(self, item): pass
            def visit_ListComp(self, item): pass
            def visit_SetComp(self, item): pass
            def visit_DictComp(self, item): pass
            def visit_GeneratorExp(self, item): pass
            def visit_Import(self, item):
                names.update(alias.asname or alias.name.split('.')[0] for alias in item.names)
            def visit_ImportFrom(self, item):
                names.update(alias.asname or alias.name for alias in item.names)
            def visit_ExceptHandler(self, item):
                if item.name: names.add(item.name)
                self.generic_visit(item)
        visitor = Visit()
        for statement in body: visitor.visit(statement)
        return names

    def function(self, node, name, body):
        # Defaults are evaluated once, in the enclosing scope, in source order.
        defaults = self.fresh('defaults')
        self.emit(f'Kw {defaults};')
        positional = node.args.posonlyargs + node.args.args
        pairs = list(zip(positional[len(positional)-len(node.args.defaults):], node.args.defaults))
        pairs += [(a, d) for a, d in zip(node.args.kwonlyargs, node.args.kw_defaults) if d is not None]
        for parameter, default in pairs:
            value = self.expr(default)
            self.emit(f'{defaults}.emplace_back({quote(parameter.arg)}, {value});')
        result = self.fresh()
        closure = self.scope.class_env or self.scope.env
        self.emit(f'V {result} = make_function({quote(name)}, {closure}, [{defaults}](E e, const Args& args, const Kw& kw) -> V {{')
        self.indent += 1
        old_scope, old_loops = self.scope, self.loops
        globals_, nonlocals = self.declarations(body)
        synchronous = old_scope.synchronous_yield or ('Ray' in self.context)
        self.scope = Scope(globals_=globals_, nonlocals=nonlocals,
                           method_class=old_scope.method_class,
                           synchronous_yield=synchronous)
        self.loops = []
        self.context.append(name)
        names = lambda sequence: '{' + ', '.join(quote(a.arg) for a in sequence) + '}'
        vararg = node.args.vararg.arg if node.args.vararg else ''
        kwarg = node.args.kwarg.arg if node.args.kwarg else ''
        if not self.own_yields(body):
            locals_ = sorted(self.local_names(node, body) - globals_ - nonlocals)
            self.scope.cached_locals = {name: self.fresh('local') for name in locals_}
            self.emit(f'e->values.reserve({len(locals_)});')
        parameters, keywords = self.fresh('parameters'), self.fresh('keywords')
        self.emit(f'static const std::vector<std::string> {parameters}{names(positional)};')
        self.emit(f'static const std::vector<std::string> {keywords}{names(node.args.kwonlyargs)};')
        self.emit(f'nt::bind(e, args, kw, {parameters}, {len(node.args.posonlyargs)}, '
                  f'{keywords}, {defaults}, {quote(vararg)}, {quote(kwarg)});')
        for name, pointer in self.scope.cached_locals.items():
            self.emit(f'V* {pointer} = nullptr;')
        if self.own_yields(body) and not synchronous:
            self.generator_function(body, node)
        else:
            self.block(body)
            self.emit('return V();')
        self.context.pop()
        self.loops, self.scope = old_loops, old_scope
        self.indent -= 1
        self.emit('});')
        return result

    def generator_function(self, body, node):
        """Lower the model's single-loop provider generator without eager calls."""
        body = [x for x in body if not isinstance(x, ast.Expr) or not isinstance(x.value, ast.Constant) or not isinstance(x.value.value, str)]
        if len(body) != 1 or not isinstance(body[0], ast.For):
            self.fail(node, 'generator function must contain one provider loop')
        loop = body[0]
        if loop.orelse or not loop.body or not isinstance(loop.body[-1], ast.Expr) or not isinstance(loop.body[-1].value, ast.Yield):
            self.fail(node, 'provider generator must yield at the end of its loop')
        if any(self.own_yields([statement]) for statement in loop.body[:-1]):
            self.fail(node, 'provider generator has multiple suspension points')
        values = self.fresh('provider_iterator')
        yielded = self.fresh('yielded')
        self.emit(f'auto {values} = std::make_shared<V>();')
        self.emit('track_capture(e);')
        self.emit(f'return make_generator([e, {values}](V& {yielded}) -> bool {{')
        self.indent += 1
        self.emit(f'if ({values}->is_none()) {{')
        self.indent += 1
        iterable = self.expr(loop.iter)
        self.emit(f'*{values} = iterator({iterable});')
        self.indent -= 1
        self.emit('}')
        item = self.fresh('item')
        self.emit(f'V {item};')
        self.emit(f'if (!try_next(*{values}, {item})) return false;')
        self.assign(loop.target, item)
        self.block(loop.body[:-1])
        value = self.expr(loop.body[-1].value.value)
        self.emit(f'{yielded} = {value};')
        self.emit('return true;')
        self.indent -= 1
        self.emit('});')

    def comprehension(self, node):
        generators = node.generators
        if any(x.is_async for x in generators):
            self.fail(node, 'async comprehensions are unsupported')
        # The first iterable is evaluated outside the comprehension scope.
        source = self.expr(generators[0].iter)
        old_scope = self.scope
        frame = self.fresh('comp')
        self.emit(f'E {frame} = child({old_scope.env});')
        self.scope = Scope(frame, synchronous_yield=old_scope.synchronous_yield)
        if isinstance(node, ast.GeneratorExp):
            state = self.fresh('state')
            state_type = self.fresh('GeneratorState')
            self.emit(f'struct {state_type} {{ E env; Args iterators; int depth=0; bool done=false; }};')
            self.emit(f'auto {state} = std::make_shared<{state_type}>();')
            self.emit(f'{state}->env = {frame}; {state}->iterators.resize({len(generators)});')
            self.emit(f'{state}->iterators[0] = iterator({source});')
            self.emit(f'track_capture({frame});')
            result = self.fresh()
            yielded = self.fresh('yielded')
            self.emit(f'V {result} = make_generator([{state}](V& {yielded}) -> bool {{')
            self.indent += 1
            self.emit(f'E {frame} = {state}->env;')
            self.emit(f'while (!{state}->done) {{')
            self.indent += 1
            self.emit(f'int depth = {state}->depth;')
            self.emit('V value;')
            self.emit(f'if (!try_next({state}->iterators[depth], value)) {{')
            self.indent += 1
            self.emit(f'if (--{state}->depth < 0) {state}->done = true;')
            self.emit('continue;')
            self.indent -= 1
            self.emit('}')
            self.emit('switch (depth) {')
            for depth, generator in enumerate(generators):
                self.emit(f'case {depth}: {{')
                self.indent += 1
                self.assign(generator.target, 'value')
                for condition in generator.ifs:
                    predicate = self.expr(condition)
                    self.emit(f'if (!truth({predicate})) continue;')
                if depth + 1 < len(generators):
                    iterable = self.expr(generators[depth+1].iter)
                    self.emit(f'{state}->iterators[{depth+1}] = iterator({iterable});')
                    self.emit(f'{state}->depth = {depth+1};')
                    self.emit('continue;')
                else:
                    value = self.expr(node.elt)
                    self.emit(f'{yielded} = {value};')
                    self.emit('return true;')
                self.indent -= 1
                self.emit('}')
            self.emit('default: throw PyError("RuntimeError", "invalid generator state");')
            self.emit('}')
            self.indent -= 1
            self.emit('}')
            self.emit('return false;')
            self.indent -= 1
            self.emit('});')
        else:
            result = self.value('make_dict()' if isinstance(node, ast.DictComp) else 'make_list()')
            def loop(depth, iterable):
                generator = generators[depth]
                item = self.fresh('item')
                iterator = self.value(f'iterator({iterable})')
                self.emit('while (true) {')
                self.indent += 1
                self.emit(f'V {item};')
                self.emit(f'if (!try_next({iterator}, {item})) break;')
                self.assign(generator.target, item)
                for condition in generator.ifs:
                    predicate = self.expr(condition)
                    self.emit(f'if (!truth({predicate})) continue;')
                if depth + 1 < len(generators):
                    loop(depth+1, self.expr(generators[depth+1].iter))
                elif isinstance(node, ast.DictComp):
                    key = self.expr(node.key)
                    value = self.expr(node.value)
                    self.emit(f'dict_set({result}, {key}, {value});')
                else:
                    value = self.expr(node.elt)
                    # The result is an unexposed, newly allocated native list.
                    self.emit(f'{result}.node()->seq.push_back({value});')
                self.indent -= 1
                self.emit('}')
            loop(0, source)
            if isinstance(node, ast.SetComp):
                result = self.value(f'make_set(iter({result}))')
        self.scope = old_scope
        return result

    def stmt(self, node):
        if isinstance(node, ast.Expr):
            if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
                return
            self.expr(node.value)
        elif isinstance(node, ast.Assign):
            value = self.expr(node.value)
            for target in node.targets:
                self.assign(target, value)
        elif isinstance(node, ast.AnnAssign):
            if node.value is not None:
                self.assign(node.target, self.expr(node.value))
        elif isinstance(node, ast.AugAssign):
            target = node.target
            if isinstance(target, ast.Name):
                old = self.expr(target)
                write = lambda value: self.set_name(target.id, value)
            elif isinstance(target, ast.Attribute):
                base = self.expr(target.value)
                old = self.value(f'attr({base}, {quote(target.attr)})')
                write = lambda value: self.emit(f'setattr({base}, {quote(target.attr)}, {value});')
            elif isinstance(target, ast.Subscript) and not isinstance(target.slice, ast.Slice):
                base, key = self.expr(target.value), self.expr(target.slice)
                old = self.value(f'item({base}, {key})')
                write = lambda value: self.emit(f'setitem({base}, {key}, {value});')
            else:
                self.fail(target, 'unsupported augmented assignment target')
            right = self.expr(node.value)
            value = self.value(f'inplace({quote(type(node.op).__name__)}, {old}, {right})')
            write(value)
        elif isinstance(node, ast.Return):
            value = self.expr(node.value) if node.value is not None else 'V()'
            self.emit(f'return {value};')
        elif isinstance(node, ast.If):
            value = self.expr(node.test)
            self.emit(f'if (truth({value})) {{')
            self.indent += 1
            self.block(node.body)
            self.indent -= 1
            if node.orelse:
                self.emit('} else {')
                self.indent += 1
                self.block(node.orelse)
                self.indent -= 1
            self.emit('}')
        elif isinstance(node, (ast.For, ast.While)):
            broke = self.fresh('broke')
            self.emit(f'bool {broke} = false;')
            self.loops.append(broke)
            if isinstance(node, ast.For):
                source = self.expr(node.iter)
                item = self.fresh('item')
                iterator = self.value(f'iterator({source})')
                self.emit('while (true) {')
                self.indent += 1
                self.emit(f'V {item};')
                self.emit(f'if (!try_next({iterator}, {item})) break;')
                self.assign(node.target, item)
            else:
                self.emit('while (true) {')
                self.indent += 1
                test = self.expr(node.test)
                self.emit(f'if (!truth({test})) break;')
            self.block(node.body)
            self.indent -= 1
            self.emit('}')
            self.loops.pop()
            if node.orelse:
                self.emit(f'if (!{broke}) {{')
                self.indent += 1
                self.block(node.orelse)
                self.indent -= 1
                self.emit('}')
        elif isinstance(node, ast.Break):
            if not self.loops: self.fail(node, 'break outside loop')
            self.emit(f'{self.loops[-1]} = true; break;')
        elif isinstance(node, ast.Continue):
            self.emit('continue;')
        elif isinstance(node, ast.FunctionDef):
            decorators = [self.expr(x) for x in node.decorator_list]
            value = self.function(node, node.name, node.body)
            for decorator in reversed(decorators):
                value = self.value(f'call({decorator}, Args{{{value}}})')
            self.set_name(node.name, value)
        elif isinstance(node, ast.ClassDef):
            if node.keywords: self.fail(node, 'class keywords unsupported')
            bases = [self.expr(x) for x in node.bases]
            outer = self.scope
            frame = self.fresh('class')
            self.emit(f'E {frame} = child({outer.env});')
            self.scope = Scope(frame, class_env=outer.env, method_class=node.name,
                               synchronous_yield=outer.synchronous_yield)
            self.context.append(node.name)
            self.block(node.body)
            self.context.pop()
            self.scope = outer
            result = self.value(f'make_class({quote(node.name)}, Args{{{", ".join(bases)}}}, {frame})')
            for decorator in reversed(node.decorator_list):
                name = decorator.func.id if isinstance(decorator, ast.Call) and isinstance(decorator.func, ast.Name) else decorator.id if isinstance(decorator, ast.Name) else ''
                if name == 'dataclass':
                    fields = [x for x in node.body if isinstance(x, ast.AnnAssign) and isinstance(x.target, ast.Name)]
                    defaults = self.fresh('fields')
                    self.emit(f'Kw {defaults};')
                    for field in fields:
                        if field.value is not None:
                            self.emit(f'{defaults}.emplace_back({quote(field.target.id)}, get({frame}, {quote(field.target.id)}));')
                    frozen = isinstance(decorator, ast.Call) and any(k.arg == 'frozen' and isinstance(k.value, ast.Constant) and k.value.value for k in decorator.keywords)
                    field_names = ', '.join(quote(x.target.id) for x in fields)
                    result = self.value(f'dataclass({result}, {{{field_names}}}, {defaults}, {str(bool(frozen)).lower()})')
                else:
                    function = self.expr(decorator)
                    result = self.value(f'call({function}, Args{{{result}}})')
            self.set_name(node.name, result)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                module = self.value(f'import_module({quote(alias.name)})')
                if not alias.asname and '.' in alias.name:
                    module = self.value(f'import_module({quote(alias.name.split(".")[0])})')
                self.set_name(alias.asname or alias.name.split('.')[0], module)
        elif isinstance(node, ast.ImportFrom):
            if node.module == '__future__': return
            if node.level: self.fail(node, 'relative imports must be resolved at build time')
            module = self.value(f'import_module({quote(node.module)})')
            for alias in node.names:
                if alias.name == '*': self.fail(node, 'star imports unsupported')
                value = self.value(f'attr({module}, {quote(alias.name)})')
                self.set_name(alias.asname or alias.name, value)
        elif isinstance(node, ast.Raise):
            if node.exc is None:
                self.emit('throw;')
            else:
                value = self.expr(node.exc)
                self.emit(f'raise({value});')
        elif isinstance(node, ast.Assert):
            value = self.expr(node.test)
            self.emit(f'if (!truth({value})) {{')
            self.indent += 1
            message = self.expr(node.msg) if node.msg else 'V("")'
            self.emit(f'throw PyError("AssertionError", str({message}));')
            self.indent -= 1
            self.emit('}')
        elif isinstance(node, ast.Try):
            if node.finalbody: self.fail(node, 'try/finally infrastructure must be supplied natively')
            success = self.fresh('success')
            self.emit(f'bool {success} = false;')
            self.emit('try {')
            self.indent += 1
            self.block(node.body)
            self.emit(f'{success} = true;')
            self.indent -= 1
            self.emit('} catch (const PyError& error) {')
            self.indent += 1
            matched = self.fresh('matched')
            self.emit(f'bool {matched} = false;')
            for handler in node.handlers:
                self.emit(f'if (!{matched}) {{')
                self.indent += 1
                test = self.expr(handler.type) if handler.type else None
                condition = f'exception_matches(error, {test})' if test else 'true'
                self.emit(f'if ({condition}) {{')
                self.indent += 1
                self.emit(f'{matched} = true;')
                if handler.name: self.set_name(handler.name, 'exception_value(error)')
                self.block(handler.body)
                self.indent -= 1
                self.emit('}')
                self.indent -= 1
                self.emit('}')
            self.emit(f'if (!{matched}) throw;')
            self.indent -= 1
            self.emit('}')
            if node.orelse:
                self.emit(f'if ({success}) {{')
                self.indent += 1
                self.block(node.orelse)
                self.indent -= 1
                self.emit('}')
        elif isinstance(node, (ast.Pass, ast.Global, ast.Nonlocal)):
            pass
        else:
            self.fail(node)

    def module_code(self, name, source, kernel_hook):
        self.module = name
        if isinstance(source, pathlib.Path):
            self.filename = str(source.resolve())
            tree = ast.parse(source.read_text(encoding='utf-8'), filename=self.filename)
        elif isinstance(source, ast.Module):
            tree = source
            self.filename = getattr(tree, 'source_path', name + '.py')
        else:
            self.filename = name + '.py'
            tree = ast.parse(source, filename=self.filename)
        self.scope = Scope()
        self.context = []
        self.emit(f'register_module({quote(name)}, [](E e) {{')
        self.indent += 1
        self.emit(f'set(e, "__file__", V({quote(self.filename)}));')
        self.emit(f'set(e, "__name__", V({quote(name)}));')
        self.block(tree.body)
        if kernel_hook:
            self.emit(f'apply_kernel_overrides({quote(name)}, e);')
        self.indent -= 1
        self.emit('});')


def generate(modules: Mapping[str, str | pathlib.Path | ast.Module], *, kernel_hook=True) -> str:
    """Return C++ for explicitly selected modules; no generated file is implicit."""
    translator = Translator()
    translator.emit('// Generated at build time from pinned game-model source; no Python runtime.')
    translator.emit('#include "runtime.h"')
    if kernel_hook:
        translator.emit('#include "engine_kernels.h"')
    translator.emit('using namespace nt;')
    translator.emit('void register_generated_modules() {')
    translator.indent += 1
    for name, source in modules.items():
        translator.module_code(name, source, kernel_hook)
    translator.indent -= 1
    translator.emit('}')
    return '\n'.join(translator.lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    parser.add_argument('--no-kernel-hook', action='store_true')
    parser.add_argument('sources', nargs='+', help='module=source.py')
    options = parser.parse_args()
    modules = {}
    for spec in options.sources:
        name, separator, path = spec.partition('=')
        if not separator: parser.error('sources must be module=source.py')
        modules[name] = pathlib.Path(path)
    text = generate(modules, kernel_hook=not options.no_kernel_hook)
    options.output.write_text(text, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()
