"""Altitude/sweep dependent VNE and MNE display limits, without trim solves."""
import math
from air_state import cache,speed_of_sound
from wing_sweep import prepare,select


def speed_limits(fm,config):
    strength=select(prepare(fm),config['sweep_percent']/100.)['geometry']['strength']
    # The vertical chart redline uses conventional total-speed IAS. The native
    # pointwise longitudinal-IAS check remains in the FM and is not substituted
    # for this explicit plot mask (it would be AoA-dependent, not vertical).
    factor=cache([1.,0.,0.],config['altitude_m'])['ias_u']
    vne=strength['ias']/factor*3.6
    mne=strength['mach']*speed_of_sound(config['altitude_m'])*3.6
    if not all(math.isfinite(x) and x>0 for x in (vne,mne)):
        raise ValueError('Aircraft speed redlines must be positive and finite')
    limit=min(vne,mne)
    # One millimetre/second of inward margin avoids a rounded body-air Mach
    # just above the critical value at a quaternion rounding boundary.
    inside=max(0.,limit-.0036)
    return dict(speed_kmh=limit,sample_speed_kmh=inside,
        kind='VNE' if vne<=mne else 'Mach limit',vne_ias_kmh=strength['ias']*3.6,
        vne_tas_kmh=vne,mne_tas_kmh=mne,mne=strength['mach'],
        enforced=bool(config['structural_limits']),
        convention='Vertical total-speed IAS redline converted to TAS; lower of VNE and MNE; native pointwise checks retained')
