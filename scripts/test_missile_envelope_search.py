"""Regression checks for disconnected hit intervals in envelope range searches."""
import unittest
import sys
from pathlib import Path

from missile_envelope import PassTracker, Planner, Ray, terminal_energy, gaps, structure, unreachable


def search(intervals, seed=None, blocked=False, time=lambda r: 1., hit=None, closest=lambda r: 0., fuze_m=0., turn_back=lambda r: False, energy=lambda r: True):
    ray = Ray((0, 0), 1000., 45., 150., 152000., .0075, 15., fuze_m=fuze_m)
    hit = hit or (lambda r: any(a <= r <= b for a, b in intervals))
    ray.blocked = blocked
    work = ray.search(seed)
    count = 0
    try:
        batch = next(work)
        while True:
            count += len(batch)
            outcomes = [dict(hit=hit(r), turn_back=turn_back(r), terminal_energy_ok=energy(r), outcome='fixture', time_s=time(r), closest_m=closest(r)) for r in batch]
            batch = work.send(outcomes)
    except StopIteration:
        pass
    return ray, count


def seeded(extra=()):
    return dict(low=150., high=26000., at_limit=True, spread=.047,
                spread_low=.003375, extra=list(extra), source='between')


class DisconnectedIntervals(unittest.TestCase):
    def test_seeded_near_and_far_hits_do_not_bridge_a_large_miss_gap(self):
        ray, _ = search([(150., 637.), (2473., 26013.)], seeded())
        self.assertEqual(len(ray.result['intervals']), 2)
        self.assertFalse(any(i['start_m'] <= 1500. <= i['end_m']
                             for i in ray.result['intervals']))

    def test_all_detected_boundaries_use_local_range_tolerance(self):
        ray, _ = search([(150., 637.), (2473., 26013.)], seeded([1500.]))
        ranges = sorted(ray.samples)
        transitions = [(a, b) for a, b in zip(ranges, ranges[1:])
                       if ray.samples[a]['hit'] != ray.samples[b]['hit']]
        self.assertEqual(len(transitions), 3)
        for a, b in transitions:
            self.assertLessEqual(b - a, ray.width((a + b) / 2), (a, b))

    def test_empty_neighbour_does_not_skip_the_normal_range_grid(self):
        ray, _ = search([(1200., 1400.)], dict(source='empty'))
        self.assertEqual(len(ray.result['intervals']), 1)
        self.assertTrue(any(o['hit'] for o in ray.samples.values()))

    def test_blocked_ray_still_does_no_flight_work(self):
        ray, count = search([(150., 152000.)], seeded(), blocked=True)
        self.assertEqual(count, 0)
        self.assertTrue(ray.result['blocked'])
        self.assertEqual(ray.result['intervals'], [])


class CloseRangeMissBands(unittest.TestCase):
    def test_flight_time_drop_between_hits_finds_the_misses_between_them(self):
        # Late turn-back hits up to 1.3 km, direct hits from 1.5 km: the misses
        # between them fall between the coarse grid's 1,229 m and 1,966 m.
        ray, _ = search([(150., 1300.), (1500., 26000.)], time=lambda r: 12. if r < 1400. else 1. + r / 1000.)
        self.assertEqual(len(ray.result['intervals']), 2)
        self.assertFalse(any(i['start_m'] <= 1400. <= i['end_m'] for i in ray.result['intervals']))

    def test_miss_band_beyond_the_minimum_range_is_found_without_a_time_change(self):
        ray, _ = search([(2000., 2250.), (2400., 14000.)], time=lambda r: 6.)
        self.assertEqual(len(ray.result['intervals']), 2)
        self.assertFalse(any(i['start_m'] <= 2325. <= i['end_m'] for i in ray.result['intervals']))

    def test_close_range_band_survives_at_its_own_range_scale(self):
        ray, _ = search([(150., 900.), (1100., 70000.)], seeded([1000.]))
        self.assertEqual(len(structure(ray.result, 5 * .0075 * ray.result['max_range_m'])), 1)
        self.assertEqual(len(gaps(ray.result, .0075, 15.)), 1)


class RaggedFarEdge(unittest.TestCase):
    # Every launch hits to 90 km; to 96 km hits thin out and the misses pass
    # 20 m from the target (12 m fuze); beyond, the miss distance grows.
    @staticmethod
    def band_hit(r):
        if r <= 90000.: return True
        if r >= 96000.: return False
        return (int(r) * 2654435761 % 1000) / 10. < 100. * (96000. - r) / 6000.

    @staticmethod
    def band_closest(r):
        return 20. if r < 96000. else 200. + (r - 96000.) * .5

    def test_envelope_ends_where_hits_stop_being_certain(self):
        ray, _ = search([], hit=self.band_hit, closest=self.band_closest, fuze_m=12.)
        self.assertTrue(ray.result['ragged_edge'])
        self.assertLessEqual(ray.result['max_range_m'], 93000.)
        self.assertGreaterEqual(ray.result['max_range_m'], 89000.)
        self.assertEqual(len(ray.result['intervals']), 1)
        self.assertTrue(any(o['hit'] for r, o in ray.samples.items() if r > 93000.))   # scattered hits beyond

    def test_sharp_far_edge_is_not_ragged(self):
        ray, _ = search([(150., 90000.)], closest=lambda r: 0. if r <= 90000. else (r - 90000.) * .9, fuze_m=12.)
        self.assertFalse(ray.result['ragged_edge'])
        self.assertAlmostEqual(ray.result['max_range_m'], 90000., delta=ray.width(90000.))


class ConservativeReachability(unittest.TestCase):
    def test_faster_receding_target_is_unreachable_after_burnout(self):
        self.assertTrue(unreachable([0., 0., 0.], 100., [1000., 0., 0.],
                                    [200., 0., 0.], 60., 40., [0., 0., 0.], 0.))

    def test_target_heading_back_toward_missile_is_not_cut(self):
        self.assertFalse(unreachable([0., 0., 0.], 100., [1000., 0., 0.],
                                     [-200., 0., 0.], 60., 40., [0., 0., 0.], 0.))

    def test_available_gravitational_energy_prevents_premature_cut(self):
        self.assertFalse(unreachable([0., 5000., 0.], 100., [1000., 5000., 0.],
                                     [200., 0., 0.], 60., 40., [0., 5000., 0.], 0.))

    def test_target_staying_outside_flight_distance_limit_is_unreachable(self):
        self.assertTrue(unreachable([0., 1000., 0.], 1000., [1500., 1000., 0.],
                                    [100., 0., 0.], 60., 40., [0., 1000., 0.], 1000.))


class MissedPassClassification(unittest.TestCase):
    @staticmethod
    def track(points, fuze=12.):
        tracker = PassTracker(points[0][0], points[0][1], fuze)
        for t, r in points[1:]: tracker.advance(t, r)
        return tracker

    def test_initial_opening_hobs_turn_is_not_a_missed_pass(self):
        self.assertFalse(self.track([(0., 150.), (1., 500.), (2., 1200.), (3., 500.), (4., 10.)]).confirmed)

    def test_ordinary_interception_is_not_a_return(self):
        self.assertFalse(self.track([(0., 5000.), (2., 3000.), (4., 1000.), (5., 10.)]).confirmed)

    def test_early_unarmed_pass_is_retained_as_a_return(self):
        p = self.track([(0., 150.), (.125, 110.), (.25, 108.), (.5, 220.), (.875, 400.), (5., 2200.), (10., 10.)])
        self.assertTrue(p.confirmed)
        self.assertEqual(p.pass_range, 108.)

    def test_small_or_brief_departures_remain_included(self):
        self.assertFalse(self.track([(0., 500.), (.2, 300.), (.4, 200.), (1., 250.), (2., 10.)]).confirmed)
        self.assertFalse(self.track([(0., 500.), (.2, 300.), (.4, 200.), (.6, 400.), (.8, 10.)]).confirmed)

    def test_tiny_initial_closure_does_not_establish_a_pass(self):
        self.assertFalse(self.track([(0., 150.), (.125, 149.), (.25, 150.), (1., 1000.), (3., 10.)]).confirmed)

    def test_far_range_shallow_reopening_stays_included(self):
        self.assertFalse(self.track([(0., 6000.), (1., 5000.), (2., 5100.), (3., 5300.), (10., 10.)]).confirmed)

    def test_departure_scales_with_fuze_radius(self):
        points = [(0., 500.), (.2, 300.), (.4, 200.), (1., 350.)]
        self.assertTrue(self.track(points).confirmed)
        self.assertFalse(self.track(points, 50.).confirmed)

    def test_deadband_noise_and_duplicate_timestamps(self):
        points = [(0., 500.), (.2, 300.), (.2, 300.), (.4, 200.), (.6, 200.1), (.8, 199.9), (1., 10.)]
        self.assertFalse(self.track(points).confirmed)

    def test_sampling_at_24_48_96_hz_classifies_the_same_pass(self):
        for hz in (24, 48, 96):
            p = self.track([(i / hz, 150. - 200. * i / hz if i / hz <= .2 else 110. + 300. * (i / hz - .2))
                            for i in range(2 * hz)])
            self.assertTrue(p.confirmed)


class FilteredIntervals(unittest.TestCase):
    def test_all_four_filter_combinations_have_independent_boundaries(self):
        ray, _ = search([(150., 26000.)], turn_back=lambda r: r < 2317., energy=lambda r: r < 10000.)
        self.assertEqual(ray.result['min_range_m'], 150.)
        self.assertAlmostEqual(ray.result['with_energy']['max_range_m'], 10000., delta=ray.width(10000.))
        both = ray.result['first_pass_with_energy']
        self.assertAlmostEqual(both['min_range_m'], 2317., delta=ray.width(2317.))
        self.assertAlmostEqual(both['max_range_m'], 10000., delta=ray.width(10000.))

    def test_unknown_energy_does_not_qualify(self):
        ray, _ = search([(150., 26000.)], energy=lambda r: None)
        self.assertTrue(ray.result['intervals'])
        self.assertEqual(ray.result['with_energy']['intervals'], [])

    def test_return_to_direct_transition_without_a_miss_is_refined(self):
        ray, _ = search([(150., 26000.)], turn_back=lambda r: r < 2317.)
        self.assertEqual(len(ray.result['intervals']), 1)
        self.assertEqual(ray.result['min_range_m'], 150.)
        first = ray.result['first_pass']
        self.assertAlmostEqual(first['min_range_m'], 2317., delta=ray.width(2317.))
        self.assertEqual(first['max_range_m'], ray.result['max_range_m'])
        a, b = first['min_bracket_m']
        self.assertLessEqual(b - a, ray.width((a + b) / 2))

    def test_all_return_hits_give_an_empty_filtered_envelope(self):
        ray, _ = search([(150., 26000.)], turn_back=lambda r: True)
        self.assertTrue(ray.result['intervals'])
        self.assertEqual(ray.result['first_pass']['intervals'], [])
        self.assertIsNone(ray.result['first_pass']['max_time_s'])

    def test_both_disconnected_direct_intervals_are_retained(self):
        ray, _ = search([(150., 500.), (1500., 2000.), (5000., 26000.)], turn_back=lambda r: r < 1000.)
        self.assertEqual(len(ray.result['first_pass']['intervals']), 2)
        self.assertTrue(all(s['hit'] for s in ray.result['samples'] if s['turn_back'] and s['range_m'] < 500.))

    def test_angular_refinement_sees_a_filtered_edge_inside_all_hit_rays(self):
        left, _ = search([(150., 26000.)], turn_back=lambda r: True)
        right, _ = search([(150., 26000.)])
        planner = Planner.__new__(Planner)
        planner.config = dict(tolerance=.0075, tolerance_m=15., detail_step_deg=1.25)
        self.assertTrue(planner.refine(left.result, right.result, 10., True))

    def test_angular_refinement_also_sees_the_energy_edge(self):
        left, _ = search([(150., 26000.)], energy=lambda r: False)
        right, _ = search([(150., 26000.)])
        planner = Planner.__new__(Planner)
        planner.config = dict(tolerance=.0075, tolerance_m=15., detail_step_deg=1.25)
        self.assertTrue(planner.refine(left.result, right.result, 10., True))


class TerminalEnergy(unittest.TestCase):
    def sample(self, speed, target=100.):
        return terminal_energy([speed, 0., 0.], [speed, 0., 0.], 0., 1., 1., [target, 0., 0.])

    def test_threshold_is_inclusive(self):
        self.assertTrue(self.sample(120.)['passed'])
        self.assertFalse(self.sample(119.999)['passed'])
        self.assertTrue(self.sample(120.001)['passed'])

    def test_specific_energy_ratio_is_speed_ratio_squared(self):
        result = self.sample(120.)
        self.assertEqual(result['specific_kinetic_energy_j_kg'], 7200.)
        self.assertEqual(result['minimum_specific_kinetic_energy_j_kg'], 7200.)
        self.assertEqual(result['speed_ratio'], 1.2)

    def test_no_mach_or_absolute_speed_floor(self):
        self.assertTrue(self.sample(12., 10.)['passed'])
        self.assertFalse(self.sample(500., 500.)['passed'])

    def test_stationary_target_has_zero_energy_threshold(self):
        result = self.sample(1., 0.)
        self.assertTrue(result['passed'])
        self.assertEqual(result['minimum_specific_kinetic_energy_j_kg'], 0.)
        self.assertIsNone(result['speed_ratio'])

    def test_speed_magnitudes_not_closure_or_direction(self):
        self.assertFalse(self.sample(100., -100.)['passed'])
        result = terminal_energy([0., 72., 96.], [0., 72., 96.], 0., 1., 1., [60., 0., 80.])
        self.assertTrue(result['passed'])
        self.assertEqual(result['speed_mps'], 120.)

    def test_event_uses_interpolated_velocity_not_step_end(self):
        result = terminal_energy([100., 0., 0.], [200., 0., 0.], 10., 10.5, 10.05, [100., 0., 0.])
        self.assertFalse(result['passed'])
        self.assertAlmostEqual(result['speed_mps'], 110.)

    def test_endpoint_fraction_and_zero_duration_are_bounded(self):
        self.assertEqual(terminal_energy([100., 0., 0.], [200., 0., 0.], 0., 1., 2., [100., 0., 0.])['speed_mps'], 200.)
        self.assertEqual(terminal_energy([100., 0., 0.], [200., 0., 0.], 1., 1., 1., [100., 0., 0.])['speed_mps'], 200.)

    def test_invalid_data_is_unknown(self):
        self.assertIsNone(self.sample(float('nan'))['passed'])
        self.assertIsNone(self.sample(120., float('inf'))['passed'])


if __name__ == '__main__':
    unittest.main()
