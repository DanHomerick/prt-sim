import unittest
import pyprt.ctrl.trajectory_solver as trajectory_solver
from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import Knot

class  TestTrajectorySolver(unittest.TestCase):
    def setUp(self):
        self.PLACES = 7

    #def tearDown(self):
    #    self.foo.dispose()
    #    self.foo = None

    def test_init(self):
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5)
        self.assertEqual(solver.v_min, 0)
        self.assertEqual(solver.a_min, -5)
        self.assertEqual(solver.j_min, -2.5)

    def test_target_position_vmax_I(self):
        """Hits max and min accelerations."""
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5)

        ### Only position is non-zero at endpoints
        spline = solver.target_position_vmax(Knot(0,0,0,0), Knot(200, 0, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 200, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 16, self.PLACES)

        ### Position and velocity are non-zero at endpoints
        spline = solver.target_position_vmax(Knot(100, 2, 0, 0), Knot(300, -2, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 16.04, self.PLACES)

        ### Position, velocity and accel are non-zero at endpoints
        spline = solver.target_position_vmax(Knot(100, 2, -2, 0), Knot(300, -2, 2, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 18.08906666, self.PLACES)

    def test_target_position_vmax_II(self):
        """May not hit max or min accelerations"""

        # Doesn't hit either max or min accel. zero endpoints
        solver = trajectory_solver.TrajectorySolver(20, 10, 2.5)
        spline = solver.target_position_vmax(Knot(0,0,0,0), Knot(200,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 200, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 15.656854249, self.PLACES)

        # Doesn't hit either max or min accel, with non-zero endopints
        spline = solver.target_position_vmax(Knot(0,1,-1,0), Knot(200,2,-2,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 200, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.879489503, self.PLACES)

        # hits a_max, a_min, and v_max exactly
        solver = trajectory_solver.TrajectorySolver(10, 5, 2.5)
        spline = solver.target_position_vmax(Knot(0,0,0,0), Knot(40,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 40, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 8, self.PLACES)

    def test_target_position_vmax_III(self):
        """Asymetrical jerk limits."""
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5, 0, -5, -5)
        spline = solver.target_position_vmax(Knot(100, 0, 0, 0), Knot(300, 0, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 15.9375, self.PLACES)

        # non-zero endpoints
        spline = solver.target_position_vmax(Knot(100, -2, 2, 0), Knot(300, 2, -4, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -4, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.4719, self.PLACES)

    def test_target_position_vmax_IV(self):
        """Asymetrical accel limits."""
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5, 0, -10, -2.5)
        # zero endpoints
        spline = solver.target_position_vmax(Knot(100, 0, 0, 0), Knot(250, 0, 0, 0))
#        self.plot_it(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 250, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 13.32842712, self.PLACES)

        # non-zeo endpoints
        spline = solver.target_position_vmax(Knot(100, 6, -2, 0), Knot(300, 5, 2, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 5, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 15.0238958709, self.PLACES)

    def test_target_position_vmax_V(self):
        """Endpoints are at limits"""
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5)

        # endpoint at v_max
        spline = solver.target_position_vmax(Knot(0, 0, 0, 0), Knot(250, 20, 0, 0))
#        self.plot_it(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 250, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 20, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 15.5, self.PLACES)

        # endpoint at a_max
        spline = solver.target_position_vmax(Knot(0, 0, 0, 0), Knot(250, 5, 5, 0))
#        self.plot_it(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 250, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 5, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 5, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 20.33333333333, self.PLACES)

    def test_target_position_vmax_VI(self):
        """Running in reverse."""
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5, -20, -5, -2.5)

        # zero endpoints
        spline = solver.target_position_vmax(Knot(250, 0, 0, 0), Knot(0, 0, 0, 0))
#        self.plot_it(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 18.5, self.PLACES)

        # Doesn't hit either max or min accel, with non-zero endopints
        solver = trajectory_solver.TrajectorySolver(20, 10, 2.5)
        spline = solver.target_position_vmax(Knot(200,2,-2,0), Knot(0,-4,-1,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], -4, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -1, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.9259812097, self.PLACES)

        # non-zeo endpoints, asymetrical limits
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5, -20, -10, -2.5)
        spline = solver.target_position_vmax(Knot(300, 5, 2, 0), Knot(100, 6, -2, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 100, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 6, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 21.0882157523, self.PLACES)

    def test_target_position_amax_I(self):
        """Doesn't reach v_max, but does reach a_max"""
        solver = trajectory_solver.TrajectorySolver(60, 5, 2.5)

        # zero endpoints
        spline = solver.target_position_amax(Knot(0,0,0,0), Knot(200,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 200, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.8062484748, self.PLACES)

        # non-zero endpoints
        spline = solver.target_position_amax(Knot(0,1,-1,0), Knot(200,2,1,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 200, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 1, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.98258766288, self.PLACES)

    def test_target_position_amax_II(self):
        """Travelling in reverse"""
        solver = trajectory_solver.TrajectorySolver(60, 5, 2.5)

        # zero endpoints
        spline = solver.target_position_amax(Knot(200,0,0,0), Knot(0,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.8062484748, self.PLACES)

        solver = trajectory_solver.TrajectorySolver(60, 5, 2.5)

        # non-zero endpoints
        spline = solver.target_position_amax(Knot(200,2,1,0), Knot(0,1,-1,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 1, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -1, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 16.451805322, self.PLACES)

    def test_target_position_none_I(self):
        """Zero endpoints"""
        solver = trajectory_solver.TrajectorySolver(30, 5, 2.5)
        spline = solver.target_position_none(Knot(0,0,0,0), Knot(10,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 10, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertAlmostEqual(spline.t[-1], 5.03968417, 5)

    def test_target_position_none_II(self):
        """Non-zero endpoints"""
        solver = trajectory_solver.TrajectorySolver(30, 5, 2.5)
        spline = solver.target_position_none(Knot(0,5,0,0), Knot(10,0,0,0))
#        print "SPLINE:", spline
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 10, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertAlmostEqual(spline.t[-1], 3.38223337, 5)

    
    def test_target_position_none_III(self):
        """Travelling in reverse."""
        solver = trajectory_solver.TrajectorySolver(30, 5, 2.5, -30, -5, -2.5)
        spline = solver.target_position_none(Knot(0,0,0,0), Knot(-10,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], -10, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertAlmostEqual(spline.t[-1], 5.03968417, 5)


    def test_target_position(self):
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5)

        # No change from initial to final
        self.assertRaises(trajectory_solver.FatalTrajectoryError,
                          solver.target_position, Knot(0,0,0,0), Knot(0,0,0,0) )

    def test_target_position_II(self):
        """Can't backup due to v_min=0, but final position is behind start position."""
        solver = trajectory_solver.TrajectorySolver(20, 5, 2.5, 0, -5, -2.5)
        self.assertRaises(trajectory_solver.FatalTrajectoryError,
                          solver.target_position, Knot(100, 0, 0, 0), Knot(90, 0, 0, 0) )

    def test_target_velocity_I(self):
        """Accelerating. Reaches a_max"""
        solver = trajectory_solver.TrajectorySolver(40, 5, 2.5)

        ### zero at endpoints
        spline = solver.target_velocity(Knot(0,0,0,0), Knot(None, 35, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 9, self.PLACES)

        # Non-zero endpoints
        spline = solver.target_velocity(Knot(10,2,1,0), Knot(None, 35, 3, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 3, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 7.399999999, self.PLACES)

        ### Unchanging accel
        spline = solver.target_velocity(Knot(0,0,5,0), Knot(None, 35, 5, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 5, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 7, self.PLACES)


    def test_target_velocity_II(self):
        """Accelerating. Doesn't reach a_max"""
        solver = trajectory_solver.TrajectorySolver(40, 40, 2.5)

        ### zero at endpoints
        spline = solver.target_velocity(Knot(0,0,0,0), Knot(None, 35, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 7.4833147735, self.PLACES)

        # Non-zero endpoints
        spline = solver.target_velocity(Knot(10,2,1,0), Knot(None, 35, 3, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 3, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 5.883314773, self.PLACES)

    def test_target_velocity_III(self):
        """Decelerating. Reaches a_max"""
        solver = trajectory_solver.TrajectorySolver(40, 5, 2.5)

        ### zero at endpoints
        spline = solver.target_velocity(Knot(0,35,0,0), Knot(None, 0, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 9, self.PLACES)

        # Non-zero endpoints
        spline = solver.target_velocity(Knot(10,35,1,0), Knot(None, 2, -1, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -1, self.PLACES)
        self.assertAlmostEqual(spline.t[-1],  8.6799999999, self.PLACES)


    def test_target_velocity_IV(self):
        """Asymmetrical limits."""
        solver = trajectory_solver.TrajectorySolver(40, 5, 2.5, 0, -20, -1.25)

        ### Decelerating, zero at endpoints
        spline = solver.target_velocity(Knot(0,35,0,0), Knot(None, 0, 0, 0))
#        self.plot_it(spline, solver)
#        self.validate_spline(spline, solver) # incompatible with a 'flipped' trajectory that has asymmetrical limits
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 10, self.PLACES)

    def validate_spline(self, spline, solver):
        # all the timespans are non-negative
        for h in spline.h:
            self.assertTrue(h >= 0)

        # all the t points agree with the timespans
        for h, ti, tf in zip(spline.h, spline.t[:-1], spline.t[1:]):
            self.assertAlmostEqual(h, tf-ti, self.PLACES)
            
        # all the knots are valid
        for i in range(len(spline.t)-1):
            # check that jerk is nearly one of: j_min, 0, or j_max
            if spline.h[i] != 0:
                jerk = (spline.a[i+1] - spline.a[i])/ spline.h[i]
            else:
                jerk = 0
            jerk_diffs = [abs(solver.j_min - jerk), abs(solver.j_max - jerk), abs(0-jerk)]
            threshold = 10**(-self.PLACES)
            self.assertTrue(jerk_diffs[0] < threshold or jerk_diffs[1] < threshold or jerk_diffs[2] < threshold)

            self.assertAlmostEqual(jerk*spline.h[i] + spline.a[i], spline.a[i+1])
            self.assertAlmostEqual(jerk*spline.h[i]**2/2 + spline.a[i]*spline.h[i] + spline.v[i], spline.v[i+1])
            self.assertAlmostEqual(jerk*spline.h[i]**3/6 + spline.a[i]*spline.h[i]**2/2 + spline.v[i]*spline.h[i] + spline.q[i], spline.q[i+1])

    def splinesAlmostEqual(self, a, b):
        self.assertAlmostEqual(a.pos, b.pos, self.PLACES)
        self.assertAlmostEqual(a.vel, b.vel, self.PLACES)
        self.assertAlmostEqual(a.accel, b.accel, self.PLACES)
        self.assertAlmostEqual(a.time, b.time, self.PLACES)

    def plot_it(self, spline, solver, title=""):
        """Convience function for debugging"""
        from pyprt.shared.cspline_plotter import CSplinePlotter
        plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min, title)
        plotter.display_plot()

if __name__ == '__main__':
    unittest.main()



