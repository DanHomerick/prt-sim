import unittest

from scipy import poly1d

from pyprt.shared.cubic_spline import Knot
from pyprt.shared.cubic_spline import CubicSpline

class  TestCubicSpline(unittest.TestCase):
    #def setUp(self):
    #    self.foo = TestCubicSpline()
    #

    #def tearDown(self):
    #    self.foo.dispose()
    #    self.foo = None
    
    PLACES = 7

    def test_evaluate(self):
        knots = [Knot(0,0,0,0), Knot(100,10,0,25), Knot(350,10,0,50), Knot(500,0,0,75)]
        spline = CubicSpline([0,100,350,500], # position
                             [0, 10,  10, 0], # velocity
                             [0,  0,   0, 0], # acceleration,
                             [0,  2,   4, 6]) # times

        self.splinesAlmostEqual(knots[0], spline.evaluate(0))
        

    def test_get_maximum_velocity(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        self.assertAlmostEqual(spline.get_max_velocity(), spline.evaluate((spline.t[2]+spline.t[3])/2).vel, self.PLACES)

    def test_get_coeffs(self):
        spline = CubicSpline( # velocity peaks midway between the 3rd and 4th knot
            [0, 3.3333333333333335, 49.30209095900485, 150.69790904099514, 196.66666666666663, 200],
            [0, 5.0, 22.015621187164243, 22.015621187164243, 5.0, 0],
            [0, 5, 5, -5, -5, 0],
            [0, 2.0, 5.4031242374328485, 9.4031242374328485, 12.806248474865697, 14.806248474865697])
        self.polys_coeffs_check(spline)

    def test_get_coeffs_II(self):
        """Test with non-zero start time."""
        spline = CubicSpline(
            [200, 196.66666666666666, 150.69790904099514, 49.302090959004843, 3.3333333333333428, 0], # pos
            [0, -5.0, -22.015621187164243, -22.015621187164243, -5.0, 0], # vel
            [0, -5, -5, 5, 5, 0], # accel
            [10, 12.0, 15.403124237432849, 19.403124237432849, 22.806248474865697, 24.806248474865697]) # time
        self.polys_coeffs_check(spline)

    def polys_coeffs_check(self, spline):
        polys = [poly1d(coeffs) for coeffs in spline.get_coeffs()]
        for poly, h, ti, tf in zip(polys, spline.h, spline.t[:-1], spline.t[1:]):
            ti_knot = spline.evaluate(ti)
            tf_knot = spline.evaluate(tf)
            self.assertAlmostEqual(poly(0), ti_knot.pos, self.PLACES)
            self.assertAlmostEqual(poly.deriv(1)(0), ti_knot.vel, self.PLACES)
            self.assertAlmostEqual(poly.deriv(2)(0), ti_knot.accel, self.PLACES)

            self.assertAlmostEqual(poly(h), tf_knot.pos, self.PLACES)
            self.assertAlmostEqual(poly.deriv(1)(h), tf_knot.vel, self.PLACES)
            self.assertAlmostEqual(poly.deriv(2)(h), tf_knot.accel, self.PLACES)

        # Check that poly's are continuous at boundaries.
        for poly_a, poly_b, h in zip(polys[:-1], polys[1:], spline.h):
            self.assertAlmostEqual(poly_a(h), poly_b(0), self.PLACES) # pos
            self.assertAlmostEqual(poly_a.deriv(1)(h), poly_b.deriv(1)(0), self.PLACES) # vel
            self.assertAlmostEqual(poly_a.deriv(2)(h), poly_b.deriv(2)(0), self.PLACES) # accel

    def splinesAlmostEqual(self, a, b):
        self.assertAlmostEqual(a.pos, b.pos, TestCubicSpline.PLACES)
        self.assertAlmostEqual(a.vel, b.vel, TestCubicSpline.PLACES)
        self.assertAlmostEqual(a.accel, b.accel, TestCubicSpline.PLACES)
        self.assertAlmostEqual(a.time, b.time, TestCubicSpline.PLACES)

if __name__ == '__main__':
    unittest.main()

