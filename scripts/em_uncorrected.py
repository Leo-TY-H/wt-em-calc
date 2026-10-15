"""Single evaluations of explicit trial states, with no equilibrium correction."""
import math

import numpy as np


def require_chart_correction(config):
    """Reject off mode before scheduling any chart or boundary searches."""
    from em_solver import aircraft_settings
    conditions = ([e['settings'] for e in config['entries']] if config.get('entries')
                  else [aircraft_settings(config, name) for name in config['aircraft']])
    if any(c['trim_solver_mode'] == 'off' for c in conditions):
        raise ValueError('Correction is off. Evaluate a trial state, or select a solver to calculate an equilibrium chart.')


def evaluate(solver, speed_kmh, load, *, attitude=None, coordinates=None, request_indices=None):
    from em_stationary import JointProblem, prepare
    from em_model_experiments import description
    for name, value, low, high in [('Speed', speed_kmh, 100., 2600.), ('Load', load, 1., 64.)]:
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be between {low:g} and {high:g}')
    if solver.config['trim_solver_mode'] != 'off':
        raise ValueError('Select correction off to evaluate an uncorrected trial')
    attitude = solver.initial_guess(speed_kmh / 3.6, load) if attitude is None else attitude
    if len(attitude) != 5 or not np.isfinite(attitude).all():
        raise ValueError('Trial attitude requires five finite coordinates')
    if coordinates is not None:
        coordinates = np.asarray(coordinates, dtype=float)
        axes = [0, 1, 3] if solver.config['experimental_lateral_trim']=='symmetric' else list(range(5))
        if coordinates.ndim != 1 or len(coordinates) < len(axes) or not np.isfinite(coordinates).all():
            raise ValueError('Trial coordinates require a finite coordinate array for the selected model')
        # The requested-RPM branch must be prepared at the supplied attitude,
        # not at an unrelated automatic starting guess.
        attitude = [0.] * 5
        for axis, coordinate in zip(axes, coordinates):attitude[axis] = float(coordinate)
    requests = prepare(solver, speed_kmh / 3.6, load, attitude)[1] if solver.is_prop else []
    indices = [0] * len(requests) if request_indices is None else request_indices
    if len(indices) != len(requests) or any(isinstance(i, bool) or not isinstance(i, int) or i < 0 for i in indices):
        raise ValueError('Invalid trial request indices')
    if any(r['governed'] and i >= len(r['candidates']) for r, i in zip(requests, indices)):
        raise ValueError('No requested-RPM candidate for this trial branch; no search performed')
    problem = JointProblem(solver, speed_kmh, load, attitude, indices)
    x = problem.initial if coordinates is None else coordinates
    if x.shape != problem.initial.shape or not np.isfinite(x).all():
        raise ValueError(f'Trial coordinates require {len(problem.initial)} finite numbers: '+', '.join(problem.coordinate_names))
    if np.any(x < problem.lower) or np.any(x > problem.upper):
        raise ValueError('Trial coordinates lie outside the declared search bounds')
    residual = problem.residual(x)
    value = problem.last
    return dict(status='uncorrected', correction_enabled=False, valid=False,
        chart_usable=False, local_valid=False, admission_checked=False,
        speed_kmh=speed_kmh, load_g=load, solution=problem.attitude(x),
        stationary_coordinates=x.tolist(), stationary_coordinate_names=problem.coordinate_names,
        stationary_request_indices=list(indices), stationary_residual=residual.tolist(),
        force_error_g=float(value['force_error_g']),
        angular_error_rad_s2=float(max(abs(value['rate_residual']))),
        history_iterations=value['history_iterations'], evaluations=problem.calls,
        model_experiments=description(solver.config), mach_curve_mode=solver.config['mach_curve_mode'],
        reasons=['Correction disabled; trial state only. Equilibrium and physical admission not checked.'])


def request(body):
    from em_solver import TrimSolver, settings
    config = settings(body['settings'])
    if config['entries'] is not None or len(config['aircraft']) != 1 or config['aircraft_settings']:
        raise ValueError('A trial requires one aircraft with explicit settings')
    return evaluate(TrimSolver(config['aircraft'][0], config), body['speed_kmh'], body['load_g'],
        coordinates=body.get('coordinates'), request_indices=body.get('request_indices'))
