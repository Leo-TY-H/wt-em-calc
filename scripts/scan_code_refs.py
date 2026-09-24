"""Validate direct call/jump references to selected native addresses."""
import argparse,json,re,struct
from pathlib import Path
from capstone import Cs,CS_ARCH_X86,CS_MODE_64,CS_OP_IMM,CS_OP_MEM
from capstone.x86 import X86_REG_RIP
from macho_scan import MachO,disassemble

def refs(m,targets):
    s=next(s for s in m.sections if s['name']=='__text');base=s['va'];d=m.read(base,s['size']);functions=set()
    for match in re.finditer(rb'[\xe8\xe9]....',d,re.S):
        p=match.start();target=base+p+5+struct.unpack_from('<i',d,p+1)[0]
        if target in targets:functions.add(m.function(base+p))
    # Overlapping candidates are necessary: regex matching consumes five bytes.
    for opcode in [b'\xe8',b'\xe9']:
        p=0
        while True:
            p=d.find(opcode,p)
            if p<0 or p+5>len(d):break
            target=base+p+5+struct.unpack_from('<i',d,p+1)[0]
            if target in targets:functions.add(m.function(base+p))
            p+=1
    cs=Cs(CS_ARCH_X86,CS_MODE_64);cs.detail=True;result=[]
    for fn in sorted(functions):
        for ins in cs.disasm(m.read(fn,m.function_end(fn)-fn),fn):
            if ins.mnemonic in ['call','jmp'] and ins.operands[0].type==CS_OP_IMM and ins.operands[0].imm in targets:
                result.append(dict(function=hex(fn),address=hex(ins.address),target=hex(ins.operands[0].imm),instruction=ins.mnemonic))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('targets',nargs='+',type=lambda x:int(x,0));p.add_argument('--out',required=True);p.add_argument('--disassemble',action='store_true');a=p.parse_args()
    m=MachO();r=refs(m,set(a.targets));Path(a.out).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
    if a.disassemble:
        for fn in sorted({int(x['function'],0) for x in r}):disassemble(m,fn,Path('analysis/disassembly'))
