"""Small recovered engine providers used by aerodynamic assembly."""
from component_assembly import f32,add,mul


def automatic_radiator(engine_type,current,oil_current,water_rate,oil_rate,dt,common=True):
    """1019fb860 / 1019fb8b0. Property engine type: Inline=0, Radial=1, Jet=2.

    Returns requested water/oil fractions before command snapshot quantization.
    Common-radiator jets request zero regardless of temperature rates/state.
    Non-common jet oil returns its previous value, not necessarily zero.
    """
    current,oil_current,water_rate,oil_rate,dt=map(f32,(current,oil_current,water_rate,oil_rate,dt))
    water=0. if engine_type>1 else min(1.,max(0.,add(current,mul(max(water_rate,oil_rate) if common else water_rate,dt))))
    oil=water if common else oil_current if engine_type>1 else min(1.,max(0.,add(oil_current,mul(oil_rate,dt))))
    return [water,oil]


def spin_engine_factor(runtime_state,health,coefficient):
    """1019f7780: engine runtime-state byte is NOT the engine property type."""
    return mul(f32(health),f32(coefficient)) if 1<=runtime_state<=7 else 0.
