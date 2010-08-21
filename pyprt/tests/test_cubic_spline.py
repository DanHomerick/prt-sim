from __future__ import division

import unittest

from scipy import poly1d, inf

from pyprt.shared.cubic_spline import Knot
from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import SplineError
from pyprt.shared.cubic_spline import OutOfBoundsError
from pyprt.shared.utility import pairwise

class  TestCubicSpline(unittest.TestCase):
    #def setUp(self):
    #    self.foo = TestCubicSpline()
    #

    #def tearDown(self):
    #    self.foo.dispose()
    #    self.foo = None

    PLACES = 6

    def make_test_spline_I(self):
        return CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200], # pos
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0], # vel
            [0, 5, 5, -5, -5, 0], # accel
            [2.5, 0, -2.5, 0, 2.5], # jerk
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697]) #  time

    def test_init(self):
        spline = CubicSpline([0,1/6,1], [0,1/2,1], [0,1,0], [1,-1], [0,1,2])
        self.assertEqual(spline.q, [0,1/6,1])
        self.assertEqual(spline.v, [0,0.5,1])
        self.assertEqual(spline.a, [0,1,0])
        self.assertEqual(spline.j, [1,-1])
        self.assertEqual(spline.t, [0,1,2])
        self.assertEqual(spline.h, [1,1])
        self.polys_coeffs_check(spline)

    def test_append(self):
        spline = CubicSpline([0,1/6,1], [0,1/2,1], [0,1,0], [1,-1], [0,1,2])
        knot = Knot(11/3,2,1,4) # calculated with jerk = 1/2
        spline.append(knot, 1/2)
        self.assertEqual(spline.q, [0,1/6,1,11/3])
        self.assertEqual(spline.v, [0,1/2,1,2])
        self.assertEqual(spline.a, [0,1,0,1])
        self.assertEqual(spline.j, [1,-1,1/2])
        self.assertEqual(spline.t, [0,1,2,4])
        self.assertEqual(spline.h, [1,1,2])
        self.polys_coeffs_check(spline)

        bad_knot = Knot(0,0,0,1) # The time is too early
        self.assertRaises(SplineError, spline.append, bad_knot, None)

    def test_concat(self):
        spline1 = CubicSpline([0,1/6,1], [0,1/2,1], [0,1,0], [1,-1], [0,1,2])
        spline2 = CubicSpline([1,11/3], [1,2], [0,1], [1/2], [2,4])
        spline = spline1.concat(spline2)
        self.assertEqual(spline.q, [0,1/6,1,11/3])
        self.assertEqual(spline.v, [0,1/2,1,2])
        self.assertEqual(spline.a, [0,1,0,1])
        self.assertEqual(spline.j, [1,-1,1/2])
        self.assertEqual(spline.t, [0,1,2,4])
        self.assertEqual(spline.h, [1,1,2])
        self.polys_coeffs_check(spline)

        self.assertRaises(SplineError, spline2.concat, spline1)

    def test_get_extrema_velocities(self):
        spline = self.make_test_spline_I()
        vels, times = spline.get_extrema_velocities()

        # Check that maxima/minima are among the values
        self.assertTrue(27.015621187164243 in vels)
        self.assertTrue(7.4031242374328485 in times)
        self.assertTrue(0 in vels)
        self.assertTrue(0 in times)
        self.assertTrue(14.806248474865697 in times)

    def test__get_idx_from_time(self):
        spline = self.make_test_spline_I()
        self.assertEquals(spline._get_idx_from_time(1.0), 0)
        self.assertEquals(spline._get_idx_from_time(2.5), 1)
        self.assertEquals(spline._get_idx_from_time(13.0), 4)
        self.assertEquals(spline._get_idx_from_time(9.0), 2)

        self.assertEquals(spline._get_idx_from_time(0), 0)
        self.assertEquals(spline._get_idx_from_time(14.806248474865697), 4)
        self.assertEquals(spline._get_idx_from_time(2.0), 1)

        self.assertRaises(OutOfBoundsError, spline._get_idx_from_time, 15)
        self.assertRaises(OutOfBoundsError, spline._get_idx_from_time, -1)

    def test_copy_left(self):
        orig_spline = self.make_test_spline_I()
        spline = orig_spline.copy_left(5.5)
##        self.plot_it(spline, 'test_copy_left')

        self.assertEquals(len(spline.q), 4)
        self.assertEquals(len(spline.v), 4)
        self.assertEquals(spline.a, [0, 5, 5, 4.7578105935821213])
        self.assertEquals(spline.j, [2.5, 0, -2.5])
        self.assertEquals(spline.t, [0, 2.0, 5.4031242374328485, 5.5])
        self.assertEquals(spline.h, [2.0, 3.4031242374328485, 0.096875762567151469])
        self.assertEquals(len(spline.coeffs), 3)
        self.polys_coeffs_check(spline)
        initial = orig_spline.evaluate(orig_spline.t[0])
        final = orig_spline.evaluate(5.5)
        self.validate_endpoints(spline, initial, final)

    def test_copy_right(self):
        orig_spline = self.make_test_spline_I()
        initial = orig_spline.evaluate(5.5)
        final = orig_spline.evaluate(orig_spline.t[-1])
        spline = orig_spline.copy_right(5.5)
##        self.plot_it(spline, 'test_copy_right')
        self.assertEqual(len(spline.t), 4)
        self.validate_endpoints(spline, initial, final)

    def test_slice_I(self):
        """Test that a slice with equal start and stop times returns a one element spline."""
        orig_spline = self.make_test_spline_I()

        knot = orig_spline.evaluate(5.5)
        spline = orig_spline.slice(5.5, 5.5)
        self.assertEqual(spline.q, [knot.pos])
        self.assertEqual(spline.v, [knot.vel])
        self.assertEqual(spline.a, [knot.accel])
        self.assertEqual(spline.j, [])
        self.assertEqual(spline.t, [knot.time])
        self.assertEqual(spline.coeffs, [])

    def test_slice_II(self):
        """Test for OutOfBoundsErrors"""
        orig_spline = self.make_test_spline_I()

        # stop_time earlier than start_time
        self.assertRaises(OutOfBoundsError, orig_spline.slice, 5.5, 5.4)

        # time beyond the spline
        self.assertRaises(OutOfBoundsError, orig_spline.slice, -1, 5.5)
        self.assertRaises(OutOfBoundsError, orig_spline.slice, 0, 15)

    def test_slice_III(self):
        """Test that a slice holds correct data."""
        orig_spline = self.make_test_spline_I()
        spline = orig_spline.slice(1.5, 5.5)
        self.assertEqual(spline.q, [1.40625, 3.3333333333333335, 49.30209095900485, 51.457954512233492])
        self.assertEqual(spline.v, [2.8125, 5.0, 22.015621187164243, 22.488268858283792])
        self.assertEqual(spline.a, [3.75, 5, 5, 4.7578105935821213])
        self.assertEqual(spline.j, [2.5, 0, -2.5])
        self.assertEqual(spline.t, [1.5, 2.0, 5.4031242374328485, 5.5])

    def tests_splice_IV(self):
        """Test that the first|last elements aren't duplicated if they happen to
        fall on a knot point."""
        orig_spline = self.make_test_spline_I()
        spline = orig_spline.slice(0, 2.0)
        self.assertEqual(spline.q, [0, 3.3333333333333335])
        self.assertEqual(spline.v, [0, 5.0])
        self.assertEqual(spline.a, [0, 5])
        self.assertEqual(spline.j, [2.5])
        self.assertEqual(spline.t, [0, 2.0])

    def test_time_shift(self):
        spline = self.make_test_spline_I()
        time = 10.5
        shifted = spline.time_shift(time)
        for t_orig, t_shift in zip(spline.t, shifted.t):
            self.assertEqual(t_orig + time, t_shift)

    def test_position_shift(self):
        spline = self.make_test_spline_I()
        position = 12.5
        shifted = spline.position_shift(position)
        for q_orig, q_shift in zip(spline.q, shifted.q):
            self.assertEqual(q_orig + position, q_shift)

    def test_get_time_from_dist(self):
        spline = self.make_test_spline_I()
        t = spline.get_time_from_dist(195, 0)
        self.assertAlmostEqual(spline.evaluate(t).pos, 195, self.PLACES)

        t = spline.get_time_from_dist(5, 2) # vehicle at 3.3333@2sec, so targeting 8.3333
        self.assertAlmostEqual(spline.evaluate(t).pos, 8.3333333333333335, self.PLACES)

        self.assertRaises(OutOfBoundsError, spline.get_time_from_dist, 210, 0) # out of bounds...

    def test_get_maximum_velocity(self):
        spline = self.make_test_spline_I()
        self.assertAlmostEqual(spline.get_max_velocity(), spline.evaluate((spline.t[2]+spline.t[3])/2).vel, self.PLACES)

    def test_coeffs(self):
        spline = self.make_test_spline_I()
        self.polys_coeffs_check(spline)

    def test_coeffs_II(self):
        """Test with non-zero start time."""
        spline = CubicSpline(
            [200, 196.66666666666666, 150.69790904099514, 49.302090959004843, 3.3333333333333428, 0], # pos
            [0, -5.0, -22.015621187164243, -22.015621187164243, -5.0, 0], # vel
            [0, -5, -5, 5, 5, 0], # accel
            [-2.5, 0, 2.5, 0, -2.5], # jerk
            [10, 12.0, 15.403124237432849, 19.403124237432849, 22.806248474865697, 24.806248474865697]) # time
        self.polys_coeffs_check(spline)

    def test_evaluate(self):
        spline = CubicSpline(
            [0, 1/6, 1], # pos
            [0, 1/2, 1], # vel
            [0, 1, 0], # accel
            [1, -1], # jerk
            [0, 1, 2]) # time
        knot = spline.evaluate(0.5)
        self.assertEquals(knot.pos, 1/48)
        self.assertEquals(knot.vel, 0.125)
        self.assertEquals(knot.accel, 0.5)
        self.assertEquals(knot.time, 0.5)

    def test_find_intersection(self):
        # spline2 starts behind spline1, but ends in the same place.
        spline1 = CubicSpline( # goes to position: 100, then stays forever
            [10, 13.333333333333334, 24.744343897926601, 85.255656102073402, 96.666666666666671, 100, 100],
            [0, 5.0, 11.794494717703369, 11.794494717703369, 5.0, 0, 0],
            [0, 5, 5, -5, -5, 0, 0],
            [2.5, 0, -2.5, 0, 2.5, 0],
            [0, 2.0, 3.358898943540674, 7.358898943540674, 8.717797887081348, 10.717797887081348, inf])
        spline2 = CubicSpline(
            [0, 3.3333333333333335, 17.507576383774929, 82.492423616225054, 96.666666666666643, 100],
            [0, 5.0, 12.912878474779198, 12.912878474779198, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [2.5, 0, -2.5, 0, 2.5],
            [0, 2.0, 3.5825756949558398, 7.5825756949558398, 9.1651513899116797, 11.16515138991168])

        dist = (spline1.q[0] - 6) - spline2.q[0] # assume spline1's vehicle is 6 meters long
        t = spline2.find_intersection(spline1, 0, spline2.t[-1], dist)
        self.assertAlmostEqual(spline1.evaluate(t).pos - 6, spline2.evaluate(t).pos)

        # Test that nothing is returned if past the point of intersection
        dist = (spline1.evaluate(t+0.1).pos - 6) - spline2.evaluate(t+0.1).pos
        t2 = spline2.find_intersection(spline1, t+0.1, spline2.t[-1], dist)
        self.assertEqual(t2, None)

    def validate_endpoints(self, spline, initial, final, pos=True, vel=True, accel=True, time=True):
        """Checks that the splines endpoints match initial in all fields, and
        match final in the selected fields."""
        self.assertAlmostEqual(initial.pos, spline.q[0], self.PLACES)
        self.assertAlmostEqual(initial.vel, spline.v[0], self.PLACES)
        self.assertAlmostEqual(initial.accel, spline.a[0], self.PLACES)
        self.assertAlmostEqual(initial.time, spline.t[0], self.PLACES)

        if pos:   self.assertAlmostEqual(final.pos, spline.q[-1], self.PLACES)
        if vel:   self.assertAlmostEqual(final.vel, spline.v[-1], self.PLACES)
        if accel: self.assertAlmostEqual(final.accel, spline.a[-1], self.PLACES)
        if time:  self.assertAlmostEqual(final.time, spline.t[-1], self.PLACES)

    def polys_coeffs_check(self, spline):
        polys = [poly1d(coeffs) for coeffs in spline.coeffs]
        for poly, (qi, qf), (vi, vf), (ai, af), (ti, tf) in \
            zip(polys, pairwise(spline.q), pairwise(spline.v), pairwise(spline.a), pairwise(spline.t)):
            self.assertAlmostEqual(poly(ti), qi, self.PLACES)
            self.assertAlmostEqual(poly.deriv(1)(ti), vi, self.PLACES)
            self.assertAlmostEqual(poly.deriv(2)(ti), ai, self.PLACES)

            self.assertAlmostEqual(poly(tf), qf, self.PLACES)
            self.assertAlmostEqual(poly.deriv(1)(tf), vf, self.PLACES)
            self.assertAlmostEqual(poly.deriv(2)(tf), af, self.PLACES)

        # Check that poly's are continuous at boundaries.
        for (poly_a, poly_b), t in zip(pairwise(polys), spline.t[1:-1]):
            self.assertAlmostEqual(poly_a(t), poly_b(t), self.PLACES) # pos
            self.assertAlmostEqual(poly_a.deriv(1)(t), poly_b.deriv(1)(t), self.PLACES) # vel
            self.assertAlmostEqual(poly_a.deriv(2)(t), poly_b.deriv(2)(t), self.PLACES) # accel

    def plot_it(self, spline, title=""):
        """Convience function for debugging"""
        from pyprt.shared.cspline_plotter import CSplinePlotter
        plotter = CSplinePlotter(spline, title=title)
        plotter.display_plot()

if __name__ == '__main__':
    unittest.main()

