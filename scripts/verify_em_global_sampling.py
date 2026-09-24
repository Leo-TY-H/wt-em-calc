"""Focused regression checks for EM branch seams and automatic prop limits."""
import json

from em_sampling import predict_limit, sample_column
from em_solver import TrimSolver, settings


def column(name, speed, **options):
    config = settings(dict(aircraft=[name], **options))
    return sample_column((name, json.dumps(config, sort_keys=True), float(speed), None))


def main():
    f5 = column('f_5e', 500., instructor=False)
    assert f5['boundary_status'] == 'verified limit', f5['boundary_status']
    assert f5['boundary_reason'] == 'stall', f5['boundary_reason']
    assert f5['boundary']['load_g'] > 5., f5['boundary']['load_g']
    assert f5['branch_selection']['narrow_numerical_seams']

    f104 = column('f_104c', 291.)
    assert f104['boundary_status'] == 'verified limit', f104['boundary_status']
    assert f104['boundary_reason'] == 'stall', f104['boundary_reason']
    assert f104['boundary']['load_g'] > 1.1, f104['boundary']['load_g']

    tornado_config = settings(dict(aircraft=['tornado_gr1'], speed_min_kmh=250.,
                                   speed_max_kmh=1300., speed_samples=7))
    tornado_key = json.dumps(tornado_config, sort_keys=True)
    adjacent = sample_column(('tornado_gr1', tornado_key, 582.7364594075522, None))
    tornado = sample_column(('tornado_gr1', tornado_key, 582.7952156250001,
                             [p for p in adjacent['points'] if p['valid']]))
    assert tornado['boundary_status'] == 'verified limit', tornado['boundary_status']
    assert tornado['boundary_reason'] == 'stall', tornado['boundary_reason']

    prop = TrimSolver('p-38j', settings(dict(aircraft=['p-38j'])))
    level = prop.solve(300., 1., exhaustive=False)
    assert level['valid'], level['reasons']
    limit = predict_limit(prop, 300., level, None)
    assert limit and limit['envelope_limit']['kind'] == 'stall'
    assert 2.8 < limit['load_g'] < 3., limit['load_g']

    p38 = column('p-38j', 450., speed_min_kmh=375., speed_max_kmh=525.)
    assert p38['boundary_status'] == 'verified limit', p38['boundary_status']
    assert p38['boundary_reason'] == 'stall', p38['boundary_reason']
    p38_low = column('p-38j', 300., speed_min_kmh=300., speed_max_kmh=525.)
    assert p38_low['boundary_status'] == 'verified limit', p38_low['boundary_status']
    assert not p38_low['numerical_gap_brackets'], p38_low['numerical_gap_brackets']
    assert all(p['propulsion']['converged'] and p['propulsion']['phase_check']['checked']
               and p['propulsion']['sample_frames']>0 for p in p38_low['points'] if p['valid'])

    ki_mid = column('ki_61_1a_otsu_china', 631.2476, speed_min_kmh=150.,
                    speed_max_kmh=1300., load_samples=7, sep_tolerance_mps=1.)
    assert ki_mid['boundary_status'] == 'verified limit', ki_mid['boundary_status']
    assert not ki_mid['numerical_gap_brackets'], ki_mid['numerical_gap_brackets']
    assert not ki_mid['interior_failures'], ki_mid['interior_failures']
    ki_high = column('ki_61_1a_otsu_china', 718.7471, speed_min_kmh=150.,
                     speed_max_kmh=1300., load_samples=7, sep_tolerance_mps=1.)
    assert ki_high['boundary_status'] == 'verified limit', ki_high['boundary_status']
    assert not ki_high['numerical_gap_brackets'], ki_high['numerical_gap_brackets']
    assert not ki_high['interior_failures'], ki_high['interior_failures']
    ki_branch = column('ki_61_1a_otsu_china', 368.7489, speed_min_kmh=150.,
                       speed_max_kmh=1300., load_samples=7, sep_tolerance_mps=1.)
    assert ki_branch['boundary_status'] == 'verified limit', ki_branch['boundary_status']

    for speed in [839.05902233963, 849.9964671386718]:
        ki_tail = column('ki_61_1a_otsu_china', speed, speed_min_kmh=150.,
                         speed_max_kmh=1300., load_samples=7, sep_tolerance_mps=1.)
        assert ki_tail['boundary_status'] == 'verified limit', ki_tail['boundary_status']
        assert not ki_tail['numerical_gap_brackets'], ki_tail['numerical_gap_brackets']
        level = next(p for p in ki_tail['points'] if p['load_g'] == 1.)
        assert level['valid'] and level['propulsion']['rpm_threshold_exceeded_engines']
        assert ki_tail['boundary']['load_g'] > 10.

    draken = column('saab_j35d', 150.)
    assert draken['boundary_status'] == 'verified limit', draken['boundary_status']
    assert draken['boundary_reason'] == 'trim fold', draken['boundary_reason']
    fold = draken['boundary']
    assert all(fold['load_g']-n>.0003 for n in fold['envelope_limit']['bracket_loads_g'])
    assert fold['force_error_g']<5e-6 and fold['angular_error_rad_s2']<5e-6

    yak = column('yak-3_france', 500., speed_min_kmh=150., speed_max_kmh=700.)
    assert yak['boundary_status'] == 'verified limit', yak['boundary_status']
    assert yak['boundary_reason'] == 'stall', yak['boundary_reason']
    assert 9.4 < yak['boundary']['load_g'] < 9.6, yak['boundary']['load_g']
    yak_high = column('yak-3_france', 624., speed_min_kmh=150., speed_max_kmh=700.)
    assert yak_high['boundary_status'] == 'verified limit', yak_high['boundary_status']
    assert yak_high['boundary_reason'] == 'wing force', yak_high['boundary_reason']
    assert 13.1 < yak_high['boundary']['load_g'] < 13.3, yak_high['boundary']['load_g']

    print('EM global sampling regression checks passed')


if __name__ == '__main__':
    main()
