"""Read an installed GRP entry and decompress its version-3 collision stream.

No installed file is modified. The installed GRP and extracted bytes are hashed
separately from the pinned datamined FM; their patch-version equality is not
assumed. Geometry itself is decoded by collision_native.py, not this script.
"""
import argparse
import hashlib
import json
import struct
from pathlib import Path
import zstandard

DEFAULT_GRP=Path('/Applications/WarThunderLauncher.app/Contents/WarThunder.app/Contents/Resources/game/content/base/res/aircrafts/germ_aircraft_logic.grp')

def extract(path, name):
    data=path.read_bytes()
    label,desc,full,rest=struct.unpack_from('<4I',data)
    assert data[:4]==b'GRP2' and rest+16==len(data)
    no,nc=struct.unpack_from('<2I',data,16);ro,rc=struct.unpack_from('<2I',data,32)
    names=[]
    for i in range(nc):
        offset=struct.unpack_from('<I',data,no+4*i)[0]
        names.append(data[offset:data.index(0,offset)].decode())
    rows=[struct.unpack_from('<IIHH',data,ro+12*i) for i in range(rc)]
    matches=[r for r in rows if names[r[2]]==name]
    assert len(matches)==1
    cls,start,rid,reserved=matches[0]
    assert cls==0xace50000
    end=min([r[1] for r in rows if r[1]>start]+[len(data)])
    packed=data[start:end];magic,tag=struct.unpack_from('<II',packed)
    assert magic==0xace50003 and tag>>30==1
    length=tag&0x3fffffff
    stream=zstandard.ZstdDecompressor().decompress(packed[8:8+length])
    sha=lambda x:hashlib.sha256(x).hexdigest()
    provenance=dict(grp_path=str(path.resolve()),grp_sha256=sha(data),grp_size=len(data),
                    resource=name,resource_class=hex(cls),offset=start,end=end,
                    packed_sha256=sha(packed),packed_size=len(packed),
                    stream_sha256=sha(stream),stream_size=len(stream),format=hex(magic),
                    fm_commit='28e93b1b1f7f0ec30644bfbd25c7f82673106012',
                    version_boundary='Installed asset hashes are pinned independently; no unverified assertion that installed assets equal datamine patch 2.59.0.13.',
                    grp_format_source='https://raw.githubusercontent.com/GaijinEntertainment/DagorEngine/75723669297e48e200a0dc67b18c1629e0975daf/prog/engine/gameRes/grpData.h')
    return packed,stream,provenance

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--grp',type=Path,default=DEFAULT_GRP)
    ap.add_argument('--resource',default='bf_109f_4_collision');a=ap.parse_args()
    packed,stream,provenance=extract(a.grp,a.resource)
    out=Path('references/collision');out.mkdir(exist_ok=True,parents=True)
    for suffix,data in [('.bin',packed),('.dump',stream)]: (out/(a.resource+suffix)).write_bytes(data)
    (out/(a.resource+'-provenance.json')).write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(provenance,indent=2))

if __name__=='__main__':main()
