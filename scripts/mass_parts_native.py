"""Native intact advanced-mass record producer on loaded collision geometry.

DataBlock lookup, temporary strings and name maps are adapters. The entire
101990300 producer, mesh traversal and component mass arithmetic are native.
"""
import json
import struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from collision_native import CollisionNative

class MassPartsNative(CollisionNative):
    def __init__(self):
        super().__init__()
        self.blocks = {}; self.block_names = []; self.block_children = {}; self.block_labels = {}; self.name_maps = {}
        # Original callers substitute this installed empty DataBlock for a
        # failed nullable child lookup.
        self.blocks[0x107f0b2c0] = {}
        self.block_children[0x107f0b2c0] = []
        self.block_labels[0x107f0b2c0] = ''
        self.getter_log = []
        for a in [0x102e6d5e0, 0x102e6bc20, 0x102e6dd50, 0x102e718c0, 0x102e71720,
                  0x102e71620, 0x102e714c0, 0x102e71f80, 0x102e725c0, 0x102e6ea50,
                  0x102e6e4b0, 0x102e6f000, 0x102e6b3c0, 0x106d421b0, 0x106d3f730,
                  0x1000225f0, 0x100022860, 0x100027f30,0x102e71140,0x102e6aa20]:
            self.u.hook_add(UC_HOOK_CODE, self.provider, begin=a, end=a)

    def block(self, value, label=''):
        a = self.alloc(32); self.blocks[a] = value
        self.block_labels[a] = label
        children=[]
        for k,v in value.items():
            if isinstance(v,dict):children.append((k,self.block(v,k)))
            elif isinstance(v,list) and v and isinstance(v[0],dict):
                children.extend((k,self.block(child,k)) for child in v)
        self.block_children[a] = children
        self.u.mem_write(a+0xe, struct.pack('<H', len(children)))
        self.u.mem_write(a+8, struct.pack('<I',1))
        # Materialize native dynamic parameter records. This lets native
        # parameter search/type/index accessors execute instead of silently
        # seeing an empty parameter table in source-backed blocks.
        rows=bytearray(); pool=bytearray()
        for k,v in value.items():
            if isinstance(v,dict) or isinstance(v,list) and v and isinstance(v[0],dict):continue
            if isinstance(v,list) and v and isinstance(v[0],list):v=v[0]
            if k not in self.block_names: self.block_names.append(k)
            name_id=self.block_names.index(k)
            if isinstance(v,list) and v and isinstance(v[0],bool): v=v[0]
            if isinstance(v,bool): kind=7; raw=struct.pack('<I',int(v))
            elif isinstance(v,int): kind=2; raw=struct.pack('<i',v)
            elif isinstance(v,float): kind=3; raw=struct.pack('<f',v)
            elif isinstance(v,list) and len(v) in [2,3,4]:
                kind=len(v)+2;raw=struct.pack('<I',len(pool));pool+=struct.pack('<'+'f'*len(v),*v)
            elif isinstance(v,str):
                kind=1; raw=struct.pack('<I',len(pool)); pool+=v.encode()+b'\0'
                pool+=bytes((-len(pool))%4)
            else: raise ValueError('Unsupported source parameter '+k)
            rows+=struct.pack('<I',name_id|(kind<<24))+raw
        count=len(rows)//8
        data=self.alloc(len(rows)+len(pool));self.u.mem_write(data,bytes(rows+pool))
        container=self.alloc(32);self.qword(container,data)
        self.u.mem_write(a+0xc,struct.pack('<H',count))
        self.u.mem_write(a+0x14,struct.pack('<I',0xffffffff));self.qword(a+0x18,container)
        return a

    def provider(self, u, a, size, data):
        self.services[hex(a)] += 1
        x, y, z, w = [u.reg_read(r) for r in [UC_X86_REG_RDI, UC_X86_REG_RSI, UC_X86_REG_RDX, UC_X86_REG_RCX]]
        result = None
        if a in [0x102e6d5e0, 0x102e6bc20, 0x102e6dd50]:
            name = self.cstr(y)
            default = self.read_xmm(0)[0] if a == 0x102e6d5e0 else z
            value = self.blocks.get(x, {}).get(name, default)
            # BLKX represents repeated scalar parameters as a list. Named
            # DataBlock getters use the first occurrence; vector getters below
            # retain the complete vector.
            if isinstance(value, list): value = value[0]
            self.getter_log.append(dict(block=x, name=name, default=default, value=value))
            if a == 0x102e6d5e0: self.xmm(0, [value])
            else: result = int(value) & 0xffffffff
        elif a == 0x102e6b3c0:
            name = self.cstr(y)
            value = self.blocks.get(x, {}).get(name)
            self.getter_log.append(dict(block=x, name=name, default=self.cstr(z) if z else None, value=value))
            if value is None: result = z
            else:
                raw = value.encode() + b'\0'; result = self.alloc(len(raw)); u.mem_write(result, raw)
        elif a == 0x106d3f730:
            # Native strings use the shared DataBlock string pool, unlike
            # vector values. Supply this indexed source getter explicitly.
            values = [v for v in self.blocks[x].values() if not isinstance(v,dict)]
            value = values[y] if y < len(values) else None
            result = 0
            if isinstance(value,str):
                raw = value.encode()+b'\0'; result=self.alloc(len(raw));u.mem_write(result,raw)
        elif a == 0x102e718c0:
            name = self.cstr(y)
            if name not in self.block_names: self.block_names.append(name)
            result = self.block_names.index(name)
        elif a in [0x102e71620, 0x102e714c0]:
            # Both indexed-name finders return null when absent. Only the
            # named getBlockByNameEx below supplies an empty fallback block.
            result = dict(self.block_children.get(x,[])).get(self.block_names[y], 0)
        elif a == 0x102e71720:
            name = self.cstr(y)
            result = dict(self.block_children.get(x,[])).get(name)
            if result is None: result = self.block({}, name)
        elif a == 0x102e71f80: result = self.block_children[x][y][1]
        elif a == 0x102e71140:
            start=struct.unpack('<i',struct.pack('<I',z&0xffffffff))[0]
            result=next((i for i,(name,_) in enumerate(self.block_children[x]) if i>start and name==self.block_names[y]),0xffffffff)
        elif a == 0x102e6aa20:result=self.block_children[x][y][1]
        elif a == 0x102e725c0:
            raw = self.block_labels[x].encode()+b'\0'; result=self.alloc(len(raw));u.mem_write(result,raw)
        elif a == 0x102e6ea50:
            value = self.blocks[x].get(self.cstr(y), self.read(z))
            if value and isinstance(value[0],list):value=value[0]
            self.xmm(0, value[:2]); self.xmm(1, value[2:])
        elif a == 0x106d421b0:
            value = self.blocks[x][self.cstr(y)]
            if value and isinstance(value[0],list):value=value[0]
            assert isinstance(value,list) and len(value)==4
            self.xmm(0,value[:2]);self.xmm(1,value[2:])
        elif a in [0x102e6e4b0, 0x102e6f000]:
            n = 2 if a == 0x102e6e4b0 else 4
            value = self.blocks[x].get(self.cstr(y), self.read(z,n))
            if value and isinstance(value[0],list):value=value[0]
            self.xmm(0, value[:2])
            if n == 4: self.xmm(1, value[2:])
        elif a in [0x1000225f0, 0x100022860]:
            name = self.cstr(y); names = self.name_maps.setdefault(x, [])
            if a == 0x100022860 and name not in names: names.append(name)
            result = names.index(name) if name in names else 0xffffffff
        elif a == 0x100027f30:
            fmt = self.cstr(z)
            count = u.reg_read(UC_X86_REG_R8)
            args=[]
            for i in range(count):
                tag=self.read(w+16*i,1,'I')[0];value=self.read(w+16*i+8,1,'Q')[0]
                assert tag in [1,3], (fmt,tag)
                args.append(self.cstr(value) if tag==1 else struct.unpack('<i',struct.pack('<I',value&0xffffffff))[0])
            raw = (fmt % tuple(args)).encode() + b'\0'; p = self.alloc(len(raw))
            u.mem_write(p, raw); self.qword(x, p)
            u.mem_write(x+16, struct.pack('<II', len(raw), len(raw)))
        else: raise RuntimeError(hex(a))
        self.ret(result)

    def produce(self, fm, fuel_names=tuple('tank%d_dm'%i for i in range(1,17)), extra=None):
        mass = self.alloc(0x600)
        self.run(0x10198fae0,[mass,self.allocator])
        for off in [0x88,0xa0]:self.qword(mass+off,self.allocator)
        names = self.alloc(64); self.name_maps[names] = list(fuel_names)
        p = self.block(fm['Mass'])
        if extra is None:
            from propeller_model import properties
            prop = properties(fm)
            extra = dict(masses={'engine1_dm':fm['Engine0']['Main']['Mass']},
                         parts={'prop0_dm':dict(mass=prop['mass'],pos=prop['position'])})
        extra_names = list(extra.get('parts',{}))
        extra = self.block(extra)
        self.getter_log = []
        self.run(0x101993720, [mass, p, 1, self.resource, names, extra],limit=50000000)
        ptr = self.read(mass+0x60, 1, 'Q')[0]; count = self.read(mass+0x70, 1, 'I')[0]
        dump = self.read(self.resource+0x78,1,'Q')[0]
        nodeofs, nc = self.read(dump+4,2,'I'); namebase=dump+self.read(dump+0x4c,1,'I')[0]
        node_names = extra_names+[self.cstr(namebase+self.read(dump+nodeofs+i*64+0x3c,1,'I')[0]) for i in range(nc) if self.u.mem_read(dump+nodeofs+i*64+3,1)[0] != 1]
        records = []
        for i in range(count):
            a = ptr+36*i
            rec = dict(name=node_names[i],mass=self.read(a,1)[0],weight=self.read(a+4,1)[0],specific_inertia=self.read(a+8),position=self.read(a+20),fuel_tank=self.u.mem_read(a+33,1)[0],box_mass=bool(self.u.mem_read(a+34,1)[0]))
            if rec['fuel_tank'] == 255: rec['fuel_tank'] = None
            records.append(rec)
        tank_count=self.read(mass+0x10,1,'I')[0]
        tanks=[dict(index=i,capacity=self.read(mass+0xb0+16*i,1)[0],
                    system=self.read(mass+0xb4+16*i,1,'I')[0],
                    priority=self.read(mass+0xb8+16*i,1,'I')[0],
                    external=bool(self.u.mem_read(mass+0xbc+16*i,1)[0])) for i in range(tank_count)]
        systems=[dict(capacity=self.read(mass+0x1b0+28*i,1)[0],
                      external_capacity=self.read(mass+0x1b4+28*i,1)[0],
                      reservoir_capacity=self.read(mass+0x1b8+28*i,1)[0],
                      priority_count=self.read(mass+0x1bc+28*i,1,'I')[0])
                 for i in range(self.read(mass+8,1,'I')[0])]
        dp=self.read(mass+0x80,1,'Q')[0];dn=self.read(mass+0x90,1,'I')[0]
        damage_records=[dict(mass=self.read(dp+36*i,1)[0],specific_inertia=self.read(dp+36*i+8),
                             position=self.read(dp+36*i+20),fuel_tank=None) for i in range(dn)]
        return dict(records=records,damage_records=damage_records,tanks=tanks,systems=systems,tank_count=tank_count,
                    tank_capacity=self.read(mass+0xb0,1)[0],raw_records=bytes(self.u.mem_read(ptr,36*count)).hex())

if __name__ == '__main__':
    m=MassPartsNative();m.load(Path('references/collision/bf_109f_4_collision.dump').read_bytes())
    result=m.produce(json.loads(Path('references/fm-2.59.0.13/bf-109f-4.blkx').read_text()))
    Path('analysis/bf-109f-4-native-mass-parts.json').write_text(json.dumps(result,indent=2)+'\n')
    print('records',len(result['records']),'sum mass',sum(p['mass'] for p in result['records']),'tank capacity',result['tank_capacity'])
