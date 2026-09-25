"""Requested deep contours must extend the independently checked SEP band."""
import unittest
from unittest.mock import patch
import numpy as np

from em_accuracy import visible_error, visible_range, crossing_coordinates, contour_check_points
from em_solver import contour_levels, settings


class ContourLevelTests(unittest.TestCase):
    def test_deep_requested_contour_is_visible_to_refinement(self):
        config={'sep_contour_levels_mps':[100.,0.,-100.,-200.,-400.]}
        self.assertEqual(visible_range(config),(-400.,300.))
        self.assertTrue(bool(visible_error(-350.,-370.,config)))
        self.assertFalse(bool(visible_error(-350.,-370.)))

    def test_contour_level_validation(self):
        self.assertEqual(contour_levels([100,0,-400]),[100.,0.,-400.])
        for invalid in ([0,0],[float('nan')],[2001],list(range(17))):
            with self.subTest(levels=invalid),self.assertRaises(ValueError):
                contour_levels(invalid)

    def test_contour_mode_retains_crossings_but_skips_unplotted_bands(self):
        config=dict(heatmap=False,sep_tolerance_mps=.5,sep_contour_levels_mps=[0.,-100.,-400.])
        self.assertFalse(bool(visible_error(-220.,-230.,config)))
        self.assertTrue(bool(visible_error(-80.,-120.,config)))
        self.assertTrue(bool(visible_error(-400.,-430.,config)))
        self.assertFalse(bool(visible_error(-350.,-360.,config)))
        self.assertTrue(bool(visible_error(-350.,-360.,dict(config,heatmap=True))))

    def test_crossings_keep_multiple_branches_and_do_not_bridge_masks(self):
        self.assertEqual(crossing_coordinates([0,1,2],[1,-1,1],[0]),[.5,1.5])
        self.assertEqual(crossing_coordinates([0,1,2],[1,np.nan,-1],[0]),[])

    def test_heatmap_defaults_off_and_requires_boolean(self):
        self.assertFalse(settings(dict(aircraft=['f_16xl']))['heatmap'])
        with self.assertRaises(ValueError):settings(dict(aircraft=['f_16xl'],heatmap='false'))

    def test_quarter_checks_use_speed_bounds_not_sep_bounds(self):
        config=dict(heatmap=True,sep_tolerance_mps=.5,sep_contour_levels_mps=[0.],
                    speed_min_kmh=400.,speed_max_kmh=600.,surface_resolution=601)
        columns=[dict(speed_kmh=400.),dict(speed_kmh=600.)]
        with patch('em_accuracy.surface_band_values',return_value=np.zeros((1,9))), \
             patch('em_surface.envelope_limits',return_value=np.array([[1.,3.]])), \
             patch('em_sampling.speed_interpolate',side_effect=lambda cols,speeds,loads:np.array(loads)[:,None]-2.):
            points=contour_check_points(columns,400.,600.,500.,np.array([2.]),np.array([1.]),np.array([0.]),config)
        self.assertEqual([v for v,n in points],[450.,550.])

    def test_contour_only_result_cannot_claim_unchecked_levels(self):
        from em_server import validate_contour_coverage
        data=dict(aircraft=[dict(interpolation=dict(scope='contours',checked_sep_levels_mps=[0.,-400.],
                                                    checked_sep_range_mps=[-400.,300.]))])
        validate_contour_coverage(data,[0.,-400.])
        with self.assertRaises(ValueError):validate_contour_coverage(data,[-200.])
        data['aircraft'][0]['interpolation']['scope']='surface'
        validate_contour_coverage(data,[-200.])


if __name__=='__main__':
    unittest.main()
