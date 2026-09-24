"""Independent engine snapshot restore used by Instructor look-ahead rewind.

Original1019f9f20 plus thermal-history copy1019eb400. Raw typed records preserve
unwritten state. This restores history; it does not simulate heating or damage.
"""
import struct
from component_assembly import sub
from instructor_protection import divide


def restore_engine(engine,properties,snapshot):
    if len(engine)<0x320 or len(properties)<0x2bc or len(snapshot)!=0xb8:
        raise ValueError('Engine/property/snapshot record size')
    e=bytearray(engine);p=properties;s=snapshot
    def read(buf,off,fmt='f'):return struct.unpack_from('<'+fmt,buf,off)[0]
    def write(off,value,fmt='f'):struct.pack_into('<'+fmt,e,off,value)
    def copy(dest,src,size):e[dest:dest+size]=s[src:src+size]
    flags=s[0]
    copy(0x1c,1,2);copy(0x20,8,4);copy(0x24,4,4)
    write(0x1e,(flags>>1)&1,'B');write(0x70,int(not flags&4),'B');copy(0xd0,0x20,1)
    copy(0x40,0xc,4);write(0x44,read(s,0x10) if flags&1 else 0.)
    if read(p,0x2b4,'i')==3:write(0x98,sub(read(p,0x2b8),read(s,0x14)))
    copy(0x28,0x18,4);copy(0x50,0x1c,4)
    # Thermal helper copies its header, clears both interleaved history lanes,
    # then restores each lane's own byte-counted number of float entries.
    copy(0x2b0,0x24,8)
    capacity=read(e,0x2b8,'I')
    if capacity>8 or max(s[0x6c],s[0x6d])>capacity:raise ValueError('Prepared thermal-history capacity')
    e[0x2bc:0x2bc+capacity*8]=bytes(capacity*8)
    for lane in range(2):
        for i in range(s[0x6c+lane]):copy(0x2bc+lane*4+i*8,0x2c+lane*4+i*8,4)
    copy(0x58,0x70,4);write(0x6c,(read(p,0x244,'I')-s[0x74])&0xffffffff,'I')
    for dest,src in [(0x54,0x75),(0x12,0x76),(0x78,0x77),(0x81,0x78),(0x80,0x79)]:copy(dest,src,1)
    for dest,src in [(0x7c,0x7c),(0x84,0x80),(0xa4,0x84),(0xc4,0x88),(0x90,0x8c)]:copy(dest,src,4)
    write(0xbc,(flags>>4)&1,'B');write(0xb4,s[0x91],'I');write(0x8c,s[0x90],'I')
    if flags&0x20:
        copy(0xe0,0xac,4);copy(0xec,0xb0,4);write(0xf0,divide(read(s,0xb4),read(e,0xe4)))
    oil=read(s,0x98) if read(s,0x94,'i') else 0.
    write(0x9c,oil)
    water_mode=read(s,0x9c,'i')
    # Native mode3 keeps XMM0 from the oil channel rather than loading the
    # water channel's saved value (1019fa0cf..175).
    write(0xa0,0. if water_mode==0 else oil if water_mode==3 else read(s,0xa0))
    copy(0xac,0xa8,4);write(0xb0,s[0xa4],'I')
    return bytes(e)


def restore_jet_engines(engines,properties,records):
    """101a15340 jet/general-engine loop; no propeller/transmission records.

    Owner and snapshot counts limit the number restored. Engines beyond that
    prefix retain their advanced state. Record representation is already decoded.
    """
    out=list(engines)
    for i in range(min(len(engines),len(records))):out[i]=restore_engine(engines[i],properties[i],records[i])
    return out


def restore_propeller(propeller,snapshot):
    """Original101a08cf0: restore retained state, leaving outputs untouched.

    The packed flags select optional commands. Shaft reaction/force/wash output
    records at +60 remain from prediction, even though pitch/flow are rewound.
    """
    if len(propeller)<0x60 or len(snapshot)!=0x4c:raise ValueError('Propeller snapshot record size')
    p=bytearray(propeller);s=snapshot;flags=s[0]
    p[0x10:0x44]=s[4:0x38]
    p[0x44:0x46]=s[1:3]
    p[0x4d]=(flags>>1)&1;p[0x4c]=(flags>>2)&1
    p[0x48:0x4c]=s[0x38:0x3c] if flags&1 else struct.pack('<f',1.)
    p[0x50:0x58]=s[0x3c:0x44] if flags&8 else bytes(8)
    p[0x58:0x5c]=s[0x44:0x48] if flags&16 else bytes(4)
    p[0x5c:0x60]=s[0x48:0x4c] if flags&32 else bytes(4)
    return bytes(p)


def restore_transmission(transmission,snapshot):
    """Original101a10ca0; current/previous RPM and delivered link state.

    Counts clamp to four. Entries outside the restored prefix and calculated
    outputs at +70 retain the advanced record's bytes.
    """
    if len(transmission)<0x70 or len(snapshot)!=0x68:raise ValueError('Transmission snapshot record size')
    t=bytearray(transmission);s=snapshot
    t[8:16]=s[:8];t[0x10:0x14]=s[0xc:0x10]
    struct.pack_into('<I',t,0x14,s[8]);t[0x18:0x1c]=s[0x10:0x14];t[0x1c]=s[0x14]
    for count_src,count_dst,src,dst in [(0x18,0x20,0x1c,0x24),(0x40,0x48,0x44,0x4c)]:
        count=min(4,struct.unpack_from('<I',s,count_src)[0]);struct.pack_into('<I',t,count_dst,count)
        for i in range(count):
            struct.pack_into('<I',t,dst+8*i+4,s[src+8*i]);t[dst+8*i:dst+8*i+4]=s[src+8*i+4:src+8*i+8]
    return bytes(t)
