import unittest

from scipy import poly1d, inf

from pyprt.shared.cubic_spline import Knot
from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import OutOfBoundsError
from pyprt.shared.utility import pairwise

class  TestCubicSpline(unittest.TestCase):
    #def setUp(self):
    #    self.foo = TestCubicSpline()
    #

    #def tearDown(self):
    #    self.foo.dispose()
    #    self.foo = None

    PLACES = 7

    def test__get_idx_from_time(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        self.assertEquals(spline._get_idx_from_time(1.0), 0)
        self.assertEquals(spline._get_idx_from_time(2.5), 1)
        self.assertEquals(spline._get_idx_from_time(13.0), 4)
        self.assertEquals(spline._get_idx_from_time(9.0), 2)

        self.assertEquals(spline._get_idx_from_time(0), 0)
        self.assertEquals(spline._get_idx_from_time(14.806248474865697), 4)
        self.assertEquals(spline._get_idx_from_time(2.0), 1)

        self.assertRaises(OutOfBoundsError, spline._get_idx_from_time, 15)
        self.assertRaises(OutOfBoundsError, spline._get_idx_from_time, -1)

    def test_clear(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        spline.clear(5.5)
##        self.plot_it(spline, 'test_clear')

        self.assertEquals(len(spline.q), 4)
        self.assertEquals(len(spline.v), 4)
        self.assertEquals(len(spline.a), 4)
        self.assertEquals(len(spline.t), 4)
        self.assertEquals(len(spline.h), 3)
        self.assertEquals(len(spline.coeffs), 3)
        self.assertEquals(spline.t[-1], 5.5)
        self.assertAlmostEquals(spline.t[-2], 5.4031242374328485, self.PLACES)
        self.polys_coeffs_check(spline)

    def test_get_time_from_dist(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        t = spline.get_time_from_dist(195, 0)
        self.assertAlmostEqual(spline.evaluate(t).pos, 195, self.PLACES)

        t = spline.get_time_from_dist(5, 2) # vehicle at 3.3333@2sec, so targeting 8.3333
        self.assertAlmostEqual(spline.evaluate(t).pos, 8.3333333333333335, self.PLACES)

        self.assertRaises(OutOfBoundsError, spline.get_time_from_dist, 210, 0) # out of bounds...


##    def test_evaluate(self):
##        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
##            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
##            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
##            [0, 5, 5, -5, -5, 0],
##            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
##


    def test_get_maximum_velocity(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        self.assertAlmostEqual(spline.get_max_velocity(), spline.evaluate((spline.t[2]+spline.t[3])/2).vel, self.PLACES)

    def test_coeffs(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        self.polys_coeffs_check(spline)

    def test_coeffs_II(self):
        """Test with non-zero start time."""
        spline = CubicSpline(
            [200, 196.66666666666666, 150.69790904099514, 49.302090959004843, 3.3333333333333428, 0], # pos
            [0, -5.0, -22.015621187164243, -22.015621187164243, -5.0, 0], # vel
            [0, -5, -5, 5, 5, 0], # accel
            [10, 12.0, 15.403124237432849, 19.403124237432849, 22.806248474865697, 24.806248474865697]) # time
        self.polys_coeffs_check(spline)

    def test_coeffs_III(self):
        """Test from problematic 'real world' data"""
        spline = CubicSpline(
            [22577.502046707479, 22587.582886707482], # pos
            [16.000000000000764, 13.822000000000493], # vel
            [0.0, -6.6000000000000005], # accel
            [1990.5708351089697, 1991.2308351089698]) # time
        self.polys_coeffs_check(spline)

    def test_find_intersection(self):
        # spline2 starts behind spline1, but ends in the same place.
        spline1 = CubicSpline( # goes to position: 100, then stays forever
            [10, 13.333333333333334, 24.744343897926601, 85.255656102073402, 96.666666666666671, 100, 100],
            [0, 5.0, 11.794494717703369, 11.794494717703369, 5.0, 0, 0],
            [0, 5, 5, -5, -5, 0, 0],
            [0, 2.0, 3.358898943540674, 7.358898943540674, 8.717797887081348, 10.717797887081348, inf])
        spline2 = CubicSpline(
            [0, 3.3333333333333335, 17.507576383774929, 82.492423616225054, 96.666666666666643, 100],
            [0, 5.0, 12.912878474779198, 12.912878474779198, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 3.5825756949558398, 7.5825756949558398, 9.1651513899116797, 11.16515138991168])

        dist = (spline1.q[0] - 6) - spline2.q[0] # assume spline1's vehicle is 6 meters long
        t = spline2.find_intersection(spline1, 0, spline2.t[-1], dist)
        self.assertAlmostEqual(spline1.evaluate(t).pos - 6, spline2.evaluate(t).pos)

        # Test that nothing is returned if past the point of intersection
        dist = (spline1.evaluate(t+0.1).pos - 6) - spline2.evaluate(t+0.1).pos
        t2 = spline2.find_intersection(spline1, t+0.1, spline2.t[-1], dist)
        self.assertEqual(t2, None)

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

    def splinesAlmostEqual(self, a, b):
        self.assertAlmostEqual(a.pos, b.pos, TestCubicSpline.PLACES)
        self.assertAlmostEqual(a.vel, b.vel, TestCubicSpline.PLACES)
        self.assertAlmostEqual(a.accel, b.accel, TestCubicSpline.PLACES)
        self.assertAlmostEqual(a.time, b.time, TestCubicSpline.PLACES)

    def plot_it(self, spline, title=""):
        """Convience function for debugging"""
        from pyprt.shared.cspline_plotter import CSplinePlotter
        plotter = CSplinePlotter(spline, title=title)
        plotter.display_plot()

if __name__ == '__main__':
    unittest.main()

