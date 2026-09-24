"""Conservative linear alias reconnaissance; findings require assembly review.

Client allocator 104e28c50 stores object+208 in object+538. This interior
pointer matters: direct positive-displacement scans miss accesses via it.
This is a locator, not a control-flow/data-flow proof or an absence proof.
"""
import json
from pathlib import Path
from capstone import Cs,CS_ARCH_X86,CS_MODE_64,CS_OP_MEM,CS_OP_REG,CS_OP_IMM
from capstone.x86 import X86_REG_RIP
from macho_scan import MachO,disassemble


def main():
    m=MachO();refs=json.loads(Path('analysis/instructor-full/client-global-refs.json').read_text())
    cs=Cs(CS_ARCH_X86,CS_MODE_64);cs.detail=True;found=[]
    def reg(ins,r):
        s=ins.reg_name(r) or ''
        if s in ['eax','ax','al']:return 'rax'
        if s in ['ebx','bx','bl']:return 'rbx'
        if s in ['ecx','cx','cl']:return 'rcx'
        if s in ['edx','dx','dl']:return 'rdx'
        if s in ['edi','di','dil']:return 'rdi'
        if s in ['esi','si','sil']:return 'rsi'
        return s[:-1] if s.startswith('r') and s[-1:] in ['d','b','w'] else s
    for fn in sorted({int(r['function'],16) for r in refs}):
        aliases={};spills={}
        for ins in cs.disasm(m.read(fn,m.function_end(fn)-fn),fn):
            ops=ins.operands
            def address(op):
                if op.type!=CS_OP_MEM or op.mem.index:return None
                if op.mem.base==X86_REG_RIP:
                    return ('slot',0) if ins.address+ins.size+op.mem.disp==0x107f5fda8 else None
                alias=aliases.get(reg(ins,op.mem.base))
                return (alias[0],alias[1]+op.mem.disp) if alias else None
            for op in ops:
                loc=address(op)
                if loc and loc[0]=='object':
                    targets=[x for x in [0x34,0xe8,0x30a] if loc[1]<=x<loc[1]+op.size]
                    if targets:found.append(dict(function=hex(fn),address=hex(ins.address),field=[hex(x) for x in targets],instruction=ins.mnemonic+' '+ins.op_str))
            if not ops:continue
            value=None
            if ins.mnemonic in ['mov','lea'] and len(ops)>1:
                src=ops[1];loc=address(src)
                if src.type==CS_OP_REG:value=aliases.get(reg(ins,src.reg))
                elif src.type==CS_OP_MEM:
                    if ins.mnemonic=='lea':value=loc
                    elif loc==('slot',0):value=('object',0)
                    elif loc==('object',0x538):value=('object',0x208)
                    elif ins.reg_name(src.mem.base)=='rbp':value=spills.get(src.mem.disp)
            elif ins.mnemonic in ['add','sub'] and len(ops)>1 and ops[0].type==CS_OP_REG and ops[1].type==CS_OP_IMM:
                old=aliases.get(reg(ins,ops[0].reg))
                if old:value=(old[0],old[1]+ops[1].imm*(1 if ins.mnemonic=='add' else -1))
            if ins.mnemonic=='call':
                if aliases.get('rdi') and aliases['rdi'][0]=='object':
                    found.append(dict(function=hex(fn),address=hex(ins.address),receiver=hex(aliases['rdi'][1]),instruction=ins.mnemonic+' '+ins.op_str))
                for r in ['rax','rcx','rdx','rsi','rdi','r8','r9','r10','r11']:aliases.pop(r,None)
            elif ops[0].type==CS_OP_REG and ops[0].access&2:
                r=reg(ins,ops[0].reg)
                if value:aliases[r]=value
                else:aliases.pop(r,None)
            elif ops[0].type==CS_OP_MEM and ins.reg_name(ops[0].mem.base)=='rbp' and ops[0].access&2:
                if value:spills[ops[0].mem.disp]=value
                else:spills.pop(ops[0].mem.disp,None)
    Path('analysis/instructor-full/client-interior-refs.json').write_text(json.dumps(found,indent=2)+'\n')
    print(json.dumps(found,indent=2))
    for fn in sorted({int(r['function'],16) for r in found}):disassemble(m,fn,Path('analysis/instructor-runtime/disassembly'))


if __name__=='__main__':main()
