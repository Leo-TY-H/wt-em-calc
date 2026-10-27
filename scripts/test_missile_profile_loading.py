"""Profile generation regressions against retained independent native outputs.

Run with: python -B -m unittest discover -s scripts -p test_missile_profile_loading.py
No game, native emulator, research tree, network or simulation sweep is needed.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import unittest

from missile_profile_loading import (
    MODEL, axis_quaternion, digest, flat_curve, projectile_properties, regenerate_profile)
from regenerate_missile_profiles import plan

ROOT = Path(__file__).resolve().parents[1]
EXPECTATIONS = json.loads((ROOT / 'references/missile-profile-native-259087.json').read_bytes())
GLOBALS = json.loads((ROOT / 'references/body-gameparams.blkx').read_bytes())['shellBallisticsParams']


def profile(asset):
    return json.loads((MODEL / 'launch-profiles' / (Path(asset).stem + '.json')).read_bytes())


class ProfileLoadingTests(unittest.TestCase):
    def test_catalog_matches_independent_current_native_properties(self):
        """All current layouts, including explicit/multiple motors, use native caps."""
        index = json.loads((MODEL / 'launch-profiles/index.json').read_bytes())
        self.assertEqual({row['asset'] for row in index['profiles']}, set(EXPECTATIONS['profiles']))
        for asset, expected in EXPECTATIONS['profiles'].items():
            with self.subTest(asset=asset):
                saved = profile(asset)
                rocket = saved['properties']['rocket']
                self.assertEqual(digest(rocket), expected['rocket_sha256'])
                body, motors, precision = projectile_properties(rocket, GLOBALS)
                self.assertEqual(body['fin_pressure_limit'], expected['fin_pressure_limit_pa'])
                self.assertEqual(body['mass'], expected['mass_kg'])
                self.assertEqual(body['cy_table'], expected['cy_table'])
                self.assertEqual(precision, expected['precision'])
                self.assertEqual(saved['properties']['body']['fin_pressure_limit'], expected['fin_pressure_limit_pa'])
                self.assertEqual(saved['properties']['precision'], expected['precision'])
                self.assertEqual(saved['properties']['body']['cy_table'], expected['cy_table'])
                self.assertEqual(saved['properties']['guidance']['aero']['cy_table'], expected['cy_table'])
                self.assertEqual(saved['properties']['guidance']['motors'], motors)

    def test_regeneration_is_idempotent_and_preserves_authored_state(self):
        for asset in ('cn_pl12a.blkx', 'us_aim_120d.blkx', 'us_aim7f_sparrow.blkx', 'uk_sraam.blkx'):
            with self.subTest(asset=asset):
                saved = profile(asset)
                source = deepcopy(saved)
                regenerated = regenerate_profile(source, GLOBALS)
                self.assertEqual(source, saved, 'Generator mutated its input')
                self.assertEqual(regenerate_profile(regenerated, GLOBALS), regenerated)
                for key in ('state', 'release_inputs', 'launch_metadata'):
                    self.assertEqual(regenerated[key], saved[key])
                self.assertEqual(regenerated['properties']['rocket'], saved['properties']['rocket'])
                self.assertEqual(regenerated['properties']['body']['mass'], saved['properties']['body']['mass'])
                self.assertEqual(regenerated['properties']['body']['inertia'], saved['properties']['body']['inertia'])
                self.assertEqual(regenerated['properties_sha256'], digest(regenerated['properties']))

    def test_flat_and_nested_curve_forms_match_native_positive_controls(self):
        for row in EXPECTATIONS['flat_controls']:
            with self.subTest(config=row['config']):
                self.assertEqual(flat_curve(row['config']), row['expected'])
        for asset in EXPECTATIONS['nested_curve_assets']:
            with self.subTest(asset=asset):
                rocket = profile(asset)['properties']['rocket']
                self.assertEqual(flat_curve(rocket), [])
                self.assertEqual(flat_curve(rocket['CyMachMult']), EXPECTATIONS['nested_positive_control'])
        row = EXPECTATIONS['axis_control']
        self.assertEqual(axis_quaternion(row['angles']), row['expected'])

    def test_precision_is_authored_not_a_fixture_choice(self):
        rocket = deepcopy(profile('us_aim_120d.blkx')['properties']['rocket'])
        self.assertNotIn('precision', rocket)
        self.assertEqual(projectile_properties(rocket, GLOBALS)[2], 1)
        rocket['precision'] = 3
        self.assertEqual(projectile_properties(rocket, GLOBALS)[2], 3)
        for invalid in (True, 0, -1, 1.5, 2**31):
            rocket['precision'] = invalid
            with self.subTest(precision=invalid), self.assertRaises(ValueError):
                projectile_properties(rocket, GLOBALS)

    def test_checkpoint_and_runtime_motor_cache_keep_regenerated_properties(self):
        # Regression for mutating only live properties: checkpoint reconstruction
        # previously restored the old serialized cap. Generated data must agree.
        from flight_session import FlightSession
        for asset in ('cn_pl12a.blkx', 'us_aim7f_sparrow.blkx'):
            with self.subTest(asset=asset):
                saved = profile(asset)
                flight = FlightSession(saved)
                resumed = FlightSession(flight.checkpoint())
                self.assertEqual(resumed.properties['body'], saved['properties']['body'])
                self.assertEqual(resumed.serialized_properties['body'], saved['properties']['body'])
                self.assertEqual(resumed.properties['precision'], 1)
                self.assertEqual(resumed.properties['guidance']['aero']['cy_table'], [])
                # FastInteractionSession takes the already validated frozen motor
                # layout from FlightSession; assert the underlying saved layout
                # survives the same serialization boundary used by that cache.
                self.assertEqual(resumed.properties['guidance']['motors'], flight.properties['guidance']['motors'])

    def test_index_manifest_and_generator_are_current(self):
        changed, records = plan(MODEL, GLOBALS)
        self.assertEqual(changed, {}, 'Run regenerate_missile_profiles.py --write')
        self.assertEqual(len(records), 227)
        manifest = json.loads((MODEL / 'manifest.json').read_bytes())
        for name, expected in manifest['sources'].items():
            if name.startswith('launch-profiles/'):
                with self.subTest(path=name):
                    self.assertEqual(hashlib.sha256((MODEL / name).read_bytes()).hexdigest(), expected)


if __name__ == '__main__':
    unittest.main()
