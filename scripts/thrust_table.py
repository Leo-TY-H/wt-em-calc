"""Steady max-dry/full-afterburner table evaluator for the two selected jets.

Outputs kgf as displayed by Statshark. This is a datamine/Statshark reconstruction,
not a recovered engine transient/nozzle implementation. Only points within the
configured altitude/TAS grid and exact 1.0 / 1.1 throttle modes are supported.
"""
from bisect import bisect_right

def axis(table,prefix):
    return [table[f'{prefix}_{i}'] for i in range(sum(k.startswith(prefix+'_') for k in table))]

def bracket(grid,value):
    if not grid[0]<=value<=grid[-1]:raise ValueError('Requested point is outside the verified table domain')
    i=min(bisect_right(grid,value)-1,len(grid)-2)
    return i,(value-grid[i])/(grid[i+1]-grid[i])

def table_coefficient(table,field,altitude,tas_kmh):
    i,u=bracket(axis(table,'Altitude'),altitude);j,v=bracket(axis(table,'Velocity'),tas_kmh)
    lo=table[f'{field}_{i}_{j}']*(1-v)+table[f'{field}_{i}_{j+1}']*v
    hi=table[f'{field}_{i+1}_{j}']*(1-v)+table[f'{field}_{i+1}_{j+1}']*v
    return lo*(1-u)+hi*u

def steady_thrust(main,altitude,tas_kmh,afterburner=False):
    table=main['ThrustMax']
    if table['VelocityType']!='TAS':raise ValueError('Only TAS grids are supported')
    throttle=1.1 if afterburner else 1.0
    mode=next(v for k,v in main.items() if k.startswith('Mode') and v.get('Throttle')==throttle)
    result=table['ThrustMax0']*table_coefficient(table,'ThrustMaxCoeff',altitude,tas_kmh)*mode['ThrustMult']
    if afterburner:result*=main['AfterburnerBoost']*table_coefficient(table,'ThrAftMaxCoeff',altitude,tas_kmh)
    return result
