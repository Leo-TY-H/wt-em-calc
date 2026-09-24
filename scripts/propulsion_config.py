"""Portable complete propulsion topology at the original property boundary."""
from piston_config import loaded_properties as piston_properties
from propeller_config import loaded_properties as propeller_properties
from turbine_config import turbine,nozzles


def decode(m, address):
    config=m.describe(address)
    for e in config['engines']:
        instance=address+0x7710+e['index']*0x19b0
        p=address+8+e['type_id']*0x770
        e['properties']=piston_properties(m,p)
        e['mass']=m.read(p+4,1)[0]
        e['maximum_direct_thrust']=m.read(p+0x10,1)[0]
        e['fuel_system']=m.read(instance+0x1c,1,'I')[0]
        e['control_group']=m.read(instance+0x20,1,'I')[0]
        if e['family'] in (2,5,6):
            e['turbine']=turbine(m,p+0x578);e['nozzles']=nozzles(m,instance)
    for prop in config['propellers']:
        links=[l for t in config['transmissions'] for l in t['propellers'] if l['index']==prop['index']]
        if len(links)!=1:raise ValueError('Expected one transmission per propeller')
        prop['properties']=propeller_properties(m,address+0x21218+prop['type_id']*0x370,
            address+0x2491c+prop['index']*0x38,links[0]['ratio'])
    return config
