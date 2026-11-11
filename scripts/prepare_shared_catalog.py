"""Prepare a new immutable input catalog offline, before making it selectable.

This is an input publication tool, never part of an HTTP calculation request.
"""
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import em_server as em
import orjson

output=Path(sys.argv[1])
version=em.game_data()['version']
assert all(part.isdecimal() for part in version.split('.')) and len(version.split('.'))==4
raw=io.BytesIO();em.save_start_cache(raw);snapshot=raw.getvalue()
from engine_performance import without_wep
meta=dict(aircraft=dict(em.AIRCRAFT),defaults=em.DEFAULTS,latest=None,
          no_wep=list(without_wep()),game_data=em.game_data())
encoded=gzip.compress(orjson.dumps(meta,option=orjson.OPT_SERIALIZE_NUMPY),compresslevel=1)
assert len(snapshot)+len(encoded)<5000000,'Catalog exceeds the finite 5 MB publication limit'
for suffix,data in (('.gz',snapshot),('.json.gz',encoded)):
    path=output/(version+suffix);path.write_bytes(data);path.chmod(0o644)
print(json.dumps(dict(version=version,key=em.start_cache_key(),
    snapshot_sha256=hashlib.sha256(snapshot).hexdigest(),meta_sha256=hashlib.sha256(encoded).hexdigest())))
