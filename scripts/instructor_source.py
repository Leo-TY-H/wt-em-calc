"""Typed boolean source adapter for Instructor research, separate from the app."""
import copy
from aircraft_catalog import load as catalog_load

BOOLEAN_FIELDS=('InvertElevator','AllowStrongControlsRestrictions','ConvertAoa',
                'ConvertAoaAI','RollLeveling','AllowModsToChangeLongidutialBalance')


def load(name):
    fm=copy.deepcopy(catalog_load(name))
    for key in BOOLEAN_FIELDS:
        value=fm.get(key)
        if isinstance(value,list):
            if not value or not all(isinstance(x,bool) for x in value):raise ValueError('Invalid boolean '+key)
            fm[key]=value[0]  # original named parameter search stops at first match
    return fm
