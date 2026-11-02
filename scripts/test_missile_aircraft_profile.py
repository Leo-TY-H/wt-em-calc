"""Regression checks for the reduced model's installed-nozzle projection."""
import math
import unittest
from unittest.mock import patch

import jet_catalog
import jet_model
import missile_aircraft_profile as aircraft


class NozzleProfiles(unittest.TestCase):
    def tearDown(self):
        aircraft.profile.cache_clear()

    def test_forward_reference_profiles_unchanged(self):
        def legacy(nozzles, thrust, cg, **options):
            forces=[jet_model.selected_nozzle(n,thrust,cg,**options)['force'] for n in nozzles]
            return dict(force=[sum(v[i] for v in forces) for i in range(3)])
        for name in ('f_16a_block_15_adf','saab_jas39c'):
            for power in (1.,1.1):
                with self.subTest(aircraft=name,power=power):
                    actual=aircraft.profile(name,power)
                    aircraft.profile.cache_clear()
                    with patch.object(aircraft,'prepare_nozzle',jet_model.prepare_nozzle),patch.object(aircraft,'evaluate_nozzles',legacy):
                        expected=aircraft.profile(name,power)
                    aircraft.profile.cache_clear()
                    self.assertEqual(actual,expected)

    def test_canted_nozzles_keep_forward_projection(self):
        for name in ('j_15t','j_16'):
            fm=jet_catalog.load(name)
            for power in (1.,1.1):
                result=aircraft.profile(name,power)
                for height,speed in ((0.,0.),(5000.,300.),(10000.,600.)):
                    expected=0.
                    for _,engine in jet_catalog.engines(fm):
                        nozzle=engine['Nozzle0'];yaw,pitch=map(math.radians,nozzle['Direction'])
                        thrust=jet_model.steady(jet_model.prepare(engine['Main']),height,speed,power)['thrust']
                        expected+=min(thrust*nozzle['ThrustRatio'],nozzle['ThrustMax'])*math.cos(yaw)*math.cos(pitch)
                    actual=result['thrust'][int(height/1000)][int(speed/50)]
                    with self.subTest(aircraft=name,power=power,height=height,speed=speed):
                        self.assertGreater(actual,0.)
                        self.assertAlmostEqual(actual,expected,delta=max(.01,expected*2e-7))

    def test_pl12a_preparation_for_all_three_carriers(self):
        from missile_service import checked_setup
        for name in ('j_10c','j_15t','j_16'):
            with self.subTest(aircraft=name):
                request=dict(missile='cn_pl12a',duration=1.,
                    launcher=dict(aircraft_id=name,position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                    target=dict(position=[12000,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                    target_control=dict(load_g=9.,aircraft_id=name,throttle=1.1))
                result=checked_setup(request)
                self.assertEqual(result['launcher']['aircraft_id'],name)
                self.assertEqual(result['target_control']['aircraft_profile']['aircraft_id'],name)
                self.assertGreater(result['target_control']['aircraft_profile']['thrust'][5][6],0.)


if __name__=='__main__':unittest.main()
