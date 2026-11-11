"""Select a complete managed input set, with packaged inputs as bootstrap."""
from pathlib import Path


def directory(scripts=None):
    scripts = Path(scripts) if scripts is not None else Path(__file__).resolve().parent
    from input_paths import REFERENCES
    managed = (scripts.parent / 'references' if scripts != Path(__file__).resolve().parent else REFERENCES) / 'missile-inputs'
    if (managed / 'launch-profiles/index.json').is_file():
        if not (managed / 'radar-launch-properties.json').is_file():
            raise ValueError('Managed missile inputs are incomplete')
        return managed
    return scripts / 'missile_model'
