"""Launch checks, source coverage, API boundary, and unchanged flight inputs."""
from copy import deepcopy
import math
import unittest
from unittest.mock import patch

import missile_launch_aircraft as aircraft


def shot(missile='cn_pl12a', carrier='j_10c', azimuth=0, elevation=0):
    a, e = math.radians(azimuth), math.radians(elevation)
    return dict(missile=missile, launcher=dict(aircraft_id=carrier,
        position=[0, 5000, 0], velocity=[300, 0, 0], angles=[0, 0, 0]),
        target=dict(position=[4000*math.cos(e)*math.cos(a), 5000+4000*math.sin(e),
                              4000*math.cos(e)*math.sin(a)], velocity=[200, 0, 0], angles=[0, 0, 0]))


class LaunchChecks(unittest.TestCase):
    def test_reviewed_aim26b_mapping_and_provenance(self):
        self.assertEqual(set(aircraft.compatible('us_aim_26b')), {'f_102a_late', 'f_102a_greece'})
        for carrier in aircraft.compatible('us_aim_26b'):
            clean = aircraft.trajectory(shot('us_aim_26b', carrier, azimuth=60, elevation=30))
            launch = clean['launcher']['aircraft']
            # The release's catalog cites its review; a per-build catalog comes straight from that commit's data.
            update = aircraft.catalog().get('compatibility_updates', {}).get('us_aim_26b', {}).get(carrier)
            self.assertEqual(launch['source_commit'], update['source_commit'] if update else aircraft.catalog()['source_commit'])
            if update:
                self.assertTrue(launch['compatibility_source']['mounts'])
            self.assertEqual(launch['radars'][0]['azimuth_deg'], [-60, 60])
            self.assertEqual(launch['radars'][0]['elevation_deg'], [-30, 30])
            with self.assertRaisesRegex(ValueError, 'outside'):
                aircraft.trajectory(shot('us_aim_26b', carrier, azimuth=60.001))
            with self.assertRaisesRegex(ValueError, 'outside'):
                aircraft.trajectory(shot('us_aim_26b', carrier, elevation=30.001))

    def test_audited_unused_and_ground_profiles_stay_unavailable(self):
        findings = aircraft.catalog()['availability']
        self.assertEqual(len(findings), 12)
        self.assertEqual(sum(f['status'] == 'active' for f in findings.values()), 1)
        self.assertEqual(sum(f['status'] == 'ground_variant_only' for f in findings.values()), 2)
        self.assertEqual(sum(f['status'] == 'unused_air_profile' for f in findings.values()), 9)
        for ident, finding in findings.items():
            if finding['status'] != 'active':
                with self.subTest(missile=ident): self.assertEqual(aircraft.compatible(ident), [])

    def test_reviewed_update_expires_when_base_source_changes(self):
        import build_missile_launcher_catalog as builder
        from pathlib import Path
        import json
        review = json.loads((Path(__file__).resolve().parents[1]/'references/missile-launcher-audit.json').read_bytes())
        update = review['aircraft_updates']['f_102a_late']
        self.assertEqual(builder.reviewed_assets('f_102a_late', update['base_sha'], review, lambda ident: ident), {'us_aim_26b'})
        with self.assertRaisesRegex(ValueError, 'stale'):
            builder.reviewed_assets('f_102a_late', 'a different vehicle revision', review, lambda ident: ident)
        with self.assertRaisesRegex(ValueError, 'Missing missile profile'):
            builder.reviewed_assets('f_102a_late', update['base_sha'], review, lambda ident: None)

    def test_known_loadouts_and_container_missile(self):
        self.assertEqual(set(aircraft.compatible('cn_pl12a')), {'j_10c', 'j_15t', 'j_16'})
        self.assertIn('f_15c_golden_eagle', aircraft.compatible('us_aim_120d'))
        self.assertIn('hunter_f6', aircraft.compatible('uk_sraam'))
        self.assertNotIn('hunter_f6', aircraft.compatible('cn_pl12a'))
        self.assertIn('z_10', aircraft.compatible('cn_ty_90'))
        self.assertIn('tiger_had_france', aircraft.compatible('atam_mistral'))
        self.assertIn('ah_64d', aircraft.compatible('us_aim92_stinger'))
        self.assertFalse(any('_killstreak' in k for k in aircraft.catalog()['aircraft']))

    def test_all_radar_carriers_have_valid_symmetric_geometry(self):
        data = aircraft.catalog()
        for missile, carriers in data['carriers'].items():
            if data['families'][missile] != 'radar': continue
            for ident in carriers:
                with self.subTest(missile=missile, carrier=ident):
                    self.assertTrue(data['aircraft'][ident]['radars'])
                    for r in data['aircraft'][ident]['radars']:
                        self.assertAlmostEqual(sum(r['azimuth_deg']), 0)
                        self.assertLessEqual(r['elevation_deg'][0], 0)
                        self.assertGreaterEqual(r['elevation_deg'][1], 0)

    def test_selection_required_and_cannot_spoof_metadata(self):
        for ident in (None, '', 'hunter_f6', [], {}):
            s = shot(carrier=ident)
            s['launcher']['aircraft'] = dict(id='j_10c', radar_limited=False)
            with self.subTest(ident=ident), self.assertRaises(ValueError): aircraft.trajectory(s)
        s = shot(); original = deepcopy(s); clean = aircraft.trajectory(s)
        self.assertEqual(s, original)
        for key in ('position', 'velocity', 'angles'): self.assertEqual(clean['launcher'][key], s['launcher'][key])
        self.assertEqual(clean['target'], s['target'])
        self.assertTrue(clean['launcher']['aircraft']['radar_limited'])

    def test_rectangular_azimuth_elevation_boundaries(self):
        # J-10C KLJ-10A is +/-70 azimuth, -50..+90 elevation.
        for az, el in ((70, 0), (-70, 0), (0, -50), (0, 89), (69, 70)):
            with self.subTest(az=az, el=el): aircraft.trajectory(shot(azimuth=az, elevation=el))
        for az, el in ((70.001, 0), (-70.001, 0), (0, -50.001), (180, 0)):
            with self.subTest(az=az, el=el), self.assertRaisesRegex(ValueError, 'outside'):
                aircraft.trajectory(shot(azimuth=az, elevation=el))

    def test_attitude_and_translation(self):
        s = shot(azimuth=90); s['launcher']['angles'] = [90, 0, 0]
        self.assertAlmostEqual(aircraft.bearing(s['launcher'], s['target'])[0], 0)
        aircraft.trajectory(s)
        s = shot(elevation=60); s['launcher']['angles'] = [0, 60, 0]
        self.assertAlmostEqual(aircraft.bearing(s['launcher'], s['target'])[1], 0)
        # Roll a target above the aircraft to the radar's right, then translate.
        s = shot(elevation=30); s['launcher']['angles'] = [0, 0, -90]
        for role in ('launcher', 'target'):
            s[role]['position'] = [v+d for v,d in zip(s[role]['position'], [900, 800, -700])]
        az, el = aircraft.bearing(s['launcher'], s['target'])
        self.assertAlmostEqual(az, 30); self.assertAlmostEqual(el, 0)

    def test_infrared_needs_no_aircraft_and_ignores_saved_selection(self):
        for ident in (None, 'hunter_f6', 'j_10c', 'unknown'):
            request = shot('uk_sraam', ident, azimuth=35)
            if ident is None: request['launcher'].pop('aircraft_id')
            else: request['launcher']['aircraft'] = dict(id=ident, radar_limited=True)
            original = deepcopy(request)
            clean = aircraft.trajectory(request)
            self.assertEqual(request, original)
            self.assertNotIn('aircraft', clean['launcher'])
            self.assertNotIn('aircraft_id', clean['launcher'])
            for key in ('position', 'velocity', 'angles'):
                self.assertEqual(clean['launcher'][key], request['launcher'][key])
            self.assertEqual(clean['target'], request['target'])

    def test_ir_lock_angle_limits_launch_not_wider_tracking_angle(self):
        self.assertEqual(aircraft.infrared_launch_angles('us_aim9l_sidewinder'), (40, 40))
        self.assertEqual(aircraft.infrared_launch_angles('su_r_73'), (45, 45))
        for az, el in ((40, 0), (-40, 0), (0, 40), (0, -40), (40, 40)):
            with self.subTest(az=az, el=el):
                aircraft.trajectory(shot('us_aim9l_sidewinder', None, az, el))
        for az, el in ((40.001, 0), (-40.001, 0), (0, 40.001), (0, -40.001), (42, 0), (180, 0)):
            request = shot('us_aim9l_sidewinder', None, az, el)
            request['launcher'].update(gimbal_deg=180, aircraft=dict(radar_limited=False))
            with self.subTest(az=az, el=el), self.assertRaisesRegex(ValueError, 'IR lock/launch limits'):
                aircraft.trajectory(request)
        # World bearings must be transformed into the launcher's body axes.
        request = shot('us_aim9l_sidewinder', None, azimuth=130)
        request['launcher']['angles'] = [90, 0, 0]
        aircraft.trajectory(request)
        request = shot('us_aim9l_sidewinder', None, elevation=50)
        request['launcher']['angles'] = [0, 15, 90]
        aircraft.trajectory(request)

    def test_ir_gate_uses_managed_profiles_and_authored_launch_limit(self):
        from pathlib import Path
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); (root/'launch-profiles').mkdir()
            profile = dict(family='optical', properties=dict(rocket=dict(guidance=dict(
                opticalSeeker=dict(lockAngleMax=40, angleMax=75), launchAngleMax=[20, 30]))))
            (root/'launch-profiles/us_aim9l_sidewinder.json').write_text(json.dumps(profile))
            aircraft.infrared_launch_angles.cache_clear()
            try:
                with patch.object(aircraft, 'input_directory', return_value=root):
                    aircraft.trajectory(shot('us_aim9l_sidewinder', None, 20, 30))
                    for az, el in ((20.001, 0), (0, 30.001)):
                        with self.assertRaisesRegex(ValueError, 'IR lock/launch limits'):
                            aircraft.trajectory(shot('us_aim9l_sidewinder', None, az, el))
            finally: aircraft.infrared_launch_angles.cache_clear()

    def test_envelope_limits_and_ir_exemption(self):
        base = dict(missile='cn_pl12a', frame='launcher', launcher=dict(platform='air', speed_mps=300, aircraft_id='j_10c', gimbal_deg=180),
                    target=dict(speed_mps=200), layers_m=[5000], quality='quick')
        clean, selected = aircraft.envelope(base)
        self.assertEqual(clean['launcher']['gimbal_deg'], 70)
        self.assertEqual(base['launcher']['gimbal_deg'], 180)
        base['frame'] = 'target'
        self.assertIsNone(aircraft.envelope(base)[0]['launcher']['gimbal_deg'])
        base.update(missile='uk_sraam', frame='launcher'); base['launcher']['aircraft_id'] = 'hunter_f6'
        base['launcher']['aircraft'] = dict(id='hunter_f6')
        for frame in ('target', 'launcher'):
            base['frame'] = frame
            clean, selected = aircraft.envelope(base)
            self.assertIsNone(selected)
            self.assertIsNone(clean['launcher']['gimbal_deg'])
            self.assertNotIn('aircraft_id', clean['launcher'])
            self.assertNotIn('aircraft', clean['launcher'])
        base['launcher'].pop('aircraft_id'); base['launcher'].pop('aircraft')
        self.assertIsNone(aircraft.envelope(base)[1])
        base.update(frame='target'); base['launcher']['platform'] = 'ground'
        clean, selected = aircraft.envelope(base)
        self.assertIsNone(selected); self.assertNotIn('aircraft_id', clean['launcher'])

    def test_http_services_enforce_and_preserve_choices(self):
        import missile_service as flights
        import missile_envelope_service as envelopes
        with patch.object(flights.WORKER, 'submit') as submit:
            key = flights.start(shot())
            try:
                config = submit.call_args.args[2]
                self.assertEqual(config['launcher']['aircraft_id'], 'j_10c')
                with self.assertRaisesRegex(ValueError, 'outside'): flights.start(shot(azimuth=71))
                with self.assertRaisesRegex(ValueError, 'IR lock/launch limits'):
                    flights.start(shot('us_aim9l_sidewinder', None, azimuth=42))
                self.assertEqual(submit.call_count, 1)
            finally: flights.JOBS.pop(key)
        request = dict(missile='cn_pl12a', frame='launcher', launcher=dict(platform='air', speed_mps=300, aircraft_id='j_10c'),
                       target=dict(speed_mps=200), layers_m=[1000], quality='quick')
        # Actual native setup: the aircraft limit reaches the search config.
        with patch.object(envelopes.WORKER, 'submit') as submit:
            key = envelopes.start(request)
            try:
                config = envelopes.get(key)['config']
                self.assertEqual(config['launcher']['gimbal_deg'], 70)
                self.assertEqual(config['launcher']['aircraft_id'], 'j_10c')
                self.assertEqual(config['launcher']['aircraft']['id'], 'j_10c')
            finally: envelopes.JOBS.pop(key)


if __name__ == '__main__': unittest.main(verbosity=2)
