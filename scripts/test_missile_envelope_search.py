"""Regression checks for disconnected hit intervals in envelope range searches."""
import unittest

from missile_envelope import Ray, gaps, structure, unreachable


def search(intervals, seed=None, blocked=False, time=lambda r: 1., hit=None, closest=lambda r: 0., fuze_m=0.):
    ray = Ray((0, 0), 1000., 45., 150., 152000., .0075, 15., fuze_m=fuze_m)
    hit = hit or (lambda r: any(a <= r <= b for a, b in intervals))
    ray.blocked = blocked
    work = ray.search(seed)
    count = 0
    try:
        batch = next(work)
        while True:
            count += len(batch)
            outcomes = [dict(hit=hit(r), outcome='fixture', time_s=time(r), closest_m=closest(r)) for r in batch]
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

    def test_band_beyond_the_solid_edge_is_reported_as_a_fringe(self):
        ray, _ = search([], hit=self.band_hit, closest=self.band_closest, fuze_m=12.)
        fringe = ray.result['fringe']
        self.assertIsNotNone(fringe)
        self.assertLessEqual(ray.result['max_range_m'], 93000.)
        self.assertGreaterEqual(ray.result['max_range_m'], 89000.)
        self.assertEqual(fringe['start_m'], ray.result['max_range_m'])
        self.assertGreater(fringe['end_m'], 93000.)
        self.assertTrue(all(self.band_hit(s) for s in ray.samples if s < 90000.))

    def test_sharp_far_edge_has_no_fringe(self):
        ray, _ = search([(150., 90000.)], closest=lambda r: 0. if r <= 90000. else (r - 90000.) * .9, fuze_m=12.)
        self.assertIsNone(ray.result['fringe'])
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


if __name__ == '__main__':
    unittest.main()
