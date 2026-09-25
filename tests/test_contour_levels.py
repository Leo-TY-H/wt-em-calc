"""Requested deep contours must extend the independently checked SEP band."""
import unittest

from em_accuracy import visible_error, visible_range
from em_solver import contour_levels


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


if __name__=='__main__':
    unittest.main()
