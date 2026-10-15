"""Local propulsion tangents for trim proposals, with full-solve fallback.

These samples are never accepted aircraft states or retained certificates.
The line search and final certification always use ordinary propulsion solves.
"""
from component_assembly import f32
from em_operating import flight_condition
from em_trim_work import charge
from prop_sensitivity import PropulsionSensitivity, SensitivityUnavailable


def sampler(solver, speed, load, x, base, fallback, search):
    if not solver.is_prop or not solver.engine.quasi_steady or not base['propulsion']['converged']:
        return fallback
    engine = solver.engine
    velocity, _, geometry, _ = flight_condition(solver,speed,load,x)
    try:
        sensitivity = PropulsionSensitivity(engine.properties,tuple(map(f32,velocity)),
            solver.config['altitude_m'],tuple(map(f32,geometry['omega'])),solver.mass['cog'],
            solver.dt,solver.mass['nitro_mass'],engine.fixed_controls or engine.automatic_controls,
            solver.config['torque_gyro'],state=base['propulsion']['state'])
    except SensitivityUnavailable:
        return fallback

    def sample(q):
        velocity, _, geometry, _ = flight_condition(solver,speed,load,q)
        try:
            prediction = sensitivity.predict(tuple(map(f32,velocity)),solver.config['altitude_m'],
                                             tuple(map(f32,geometry['omega'])))
        except SensitivityUnavailable:
            charge('propulsion_derivative_fallbacks')
            return fallback(q)
        row = prediction['outputs'].tolist()
        propulsion = dict(sensitivity.result,force=row[:3],moment=row[3:6],
                          angular_momentum=row[6:9],wash=row[9:],cycle_samples=[row])
        charge('propulsion_derivative_predictions')
        return solver.operating_point(speed,load,q,propulsion_sample=propulsion,_search=search)
    return sample
