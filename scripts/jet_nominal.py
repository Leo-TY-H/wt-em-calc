"""Declared healthy, frozen-supply jet model; no spool or mechanical history."""
from component_assembly import f32, mul
from control_mixer import curve
from engine_supply import available_fuel
from jet_model import scalar_update
from structural_limits import interval

REVISION = 'capped-nominal-jet-v1'


def evaluate(properties, jet, fuel, system_fuel, accumulator, density, body_u,
             dt, throttle, afterburner):
    # The retained owner computes its neutral control limits at IAS zero.
    # The force/nozzle owner separately receives the actual longitudinal IAS.
    vtol = min(interval(0., *properties['ias_vtol']), 0.)
    effective = min(curve(properties['vtol_throttle'], vtol, 1)[0],
                    interval(0., *properties['reverse_throttle']), f32(throttle))
    normalized = min(f32(effective / f32(
        1.1 if properties['throttle_boost'] > 1. else 1.)), 1.)
    supply = available_fuel(fuel, system_fuel, accumulator, 1., dt)
    inputs = dict(afterburner=afterburner, fuel_available=supply, health=1., running=2)
    # Target selection never reads omega. Reevaluate output at the capped target;
    # the first call's thrust and both calls' next_omega are intentionally unused.
    target = scalar_update(jet, density, body_u, 0., normalized, dt, **inputs)
    if not target['active']:
        raise ValueError('Nominal running-jet supply branch unavailable; not an absence certificate')
    omega = min(max(target['target_omega'], 0.), mul(properties['omega_limit'], f32(1.)))
    output = scalar_update(jet, density, body_u, omega, normalized, dt,
                           shaft_fraction=mul(f32(omega), properties['inverse_omega']), **inputs)
    return dict(omega=omega, mechanical=1., effective_throttle=effective,
                effective_vtol=vtol, fuel_available=supply, scalar=output,
                revision=REVISION)
