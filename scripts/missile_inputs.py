"""Select a complete managed input set, with packaged inputs as bootstrap."""
from pathlib import Path


def directory(scripts=None):
    scripts = Path(scripts) if scripts is not None else Path(__file__).resolve().parent
    managed = scripts.parent / 'references/missile-inputs'
    if (managed / 'launch-profiles/index.json').is_file():
        if not (managed / 'radar-launch-properties.json').is_file():
            raise ValueError('Managed missile inputs are incomplete')
        return managed
    return scripts / 'missile_model'
