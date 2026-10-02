from functools import lru_cache
from collections.abc import Mapping
import json
from pathlib import Path
import jet_catalog
import prop_catalog
REFERENCE=jet_catalog.REFERENCE
_worker_snapshot=None
EXCLUSIONS=Path(__file__).resolve().parents[1]/'references/aircraft-exclusions.json'


class LazyCatalog(Mapping):
    def __getitem__(self,key):return catalog()[key]
    def __iter__(self):return iter(catalog())
    def __len__(self):return len(catalog())


def install_worker_catalog(snapshot):
    global _worker_snapshot
    _worker_snapshot=snapshot
    catalog.cache_clear()

    prop_catalog._worker_snapshot={k:v for k,v in snapshot.items() if v['propulsion'] not in ('jet','rocket')}
    prop_catalog.catalog.cache_clear()


@lru_cache(maxsize=1)
def catalog():
    if _worker_snapshot is not None:return _worker_snapshot
    result={k:dict(v) for k,v in jet_catalog.catalog().items()}
    result.update(prop_catalog.catalog())


    excluded=json.loads(EXCLUSIONS.read_text(encoding='utf-8'))['aircraft']
    for name in excluded:result.pop(name,None)
    from aircraft_upgrades import profile
    for name, row in result.items():
        try:
            row['upgrades']=profile(name)
            if row['propulsion'] in ('jet','rocket') and row['upgrades']['applied']:
                raise ValueError('Positive direct-thrust upgrade requires a validated engine consumer')
        except ValueError as error:
            row.update(supported=False,reason=str(error))
    from aircraft_ammunition import profile as ammunition_profile
    for name, row in result.items():
        row['ammunition']=ammunition_profile(name)
    from vehicle_names import apply_names
    apply_names(result)
    from flap_model import profile as flap_profile
    for name, row in result.items():
        path = prop_catalog.fm_source(row['fm_id']) if 'fm_id' in row else jet_catalog.source(name)
        row['flaps'] = flap_profile(json.loads(path.read_text()))
    return result


def is_prop(name):return catalog()[name]['propulsion'] not in ('jet','rocket')


def load(name):return prop_catalog.load(name) if is_prop(name) else jet_catalog.load(name)


def source(name):
    return prop_catalog.fm_source(catalog()[name]['fm_id']) if is_prop(name) else jet_catalog.source(name)


def asset_sources():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    from aircraft_ammunition import PROFILE
    from aircraft_upgrades import source_paths
    from vehicle_names import source_paths as name_sources
    return (sorted((root/'references/prop-propulsion').glob('*.json'))+
            sorted((root/'references/prop-mass').glob('*.json'))+source_paths()+
            name_sources()+([PROFILE] if PROFILE.exists() else [])+[EXCLUSIONS,root/'app/fonts/wt-symbols.ttf',root/'references/prop-native-config.json',
             root/'references/body-gameplay.blkx',root/'references/body-gameparams.blkx'])
