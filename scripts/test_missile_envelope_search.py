"""Regression checks for disconnected hit intervals in envelope range searches."""
import unittest

from missile_envelope import Ray, unreachable


def search(intervals, seed=None, blocked=False):
    ray = Ray((0, 0), 1000., 45., 150., 152000., .0075, 15.)
    ray.blocked = blocked
    work = ray.search(seed)
    count = 0
    try:
        batch = next(work)
        while True:
            count += len(batch)
            outcomes = [dict(hit=any(a <= r <= b for a, b in intervals),
                             outcome='fixture', time_s=1., closest_m=0.) for r in batch]
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
