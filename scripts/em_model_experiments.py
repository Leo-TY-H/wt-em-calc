"""Optional model simplifications; disabled settings recover the retained model.

These are modeling experiments, not root-coverage or physical-absence proofs.
Body axes are x forward, y up, z lateral; pitching moment is component 2.
"""
REVISION = 'instantaneous-aero-coordinated-trim-v1'
MODES = ('full', 'zero_sideslip', 'symmetric')


def description(config):
    return dict(revision=REVISION,
                instantaneous_aerodynamics=config.get('experimental_instantaneous_aero', False),
                lateral_trim=config.get('experimental_lateral_trim', 'full'))


def label(config):
    parts = []
    if config.get('experimental_instantaneous_aero', False):parts.append('Instantaneous aero')
    lateral = config.get('experimental_lateral_trim', 'full')
    if lateral == 'zero_sideslip':parts.append('Zero sideslip')
    if lateral == 'symmetric':parts.append('Ideal symmetric')
    return ' / '.join(parts)


def reference_wing_angles(g, polars, velocity, cog, controls, flaps, density,
                          body_aoa, engine_x, engine_swirl):
    """Explicit geometry input: current CG flow, wash, incidence and controls.

    No previous angle or moving application point enters this reference flow.
    Forces still use local rotational flow at the resulting application points.
    This deliberately replaces the implicit moving-center/history relation.
    """
    from wing_model import local_inputs, coefficients
    angles = []
    for side in range(2):
        flow = local_inputs(side, cog, cog, velocity, (0., 0., 0.),
                            (0., 0., 0.), g['dihedral'], engine_x, engine_swirl)
        c = coefficients(polars[side], flow, controls[side], g['incidence'],
                         flaps[side], body_aoa=body_aoa)
        angles.append(c['aoa'])
    return angles


def symmetric_result(result, mass, target_omega, dt):
    """Ideal lateral constraint, preserving longitudinal force and pitch moment.

    Discard lateral resultant force and supply ideal roll/yaw constraint moments
    to follow the prescribed turn. Record the discarded quantities and reaction;
    never present them as roll/yaw trim achieved by physical controls.
    """
    force = list(result['force'])
    moment = list(result['stored_moment'])
    original = dict(force=force.copy(), moment=moment.copy())
    force[2] = 0.
    wx, wy, wz = result['omega_for_flow']
    ix, iy, iz = mass['inertia']
    for axis, cross in ((0, wy*wz*(iz-iy)), (1, wz*wx*(ix-iz))):
        moment[axis] = (target_omega[axis]-result['omega_for_flow'][axis])*mass['inertia'][axis]/dt-cross
    return dict(result, force=force, stored_moment=moment,
                symmetry_projection=dict(unconstrained=original,
                    removed_side_force=original['force'][2],
                    ideal_constraint_moment=[moment[i]-original['moment'][i] for i in range(3)]))
