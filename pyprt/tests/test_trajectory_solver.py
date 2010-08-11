import unittest
from pyprt.ctrl.trajectory_solver import TrajectorySolver, TrajectoryError, FatalTrajectoryError, OptimizationError
from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import Knot

class  TestTrajectorySolver(unittest.TestCase):
    def setUp(self):
        self.PLACES = 7
        self.Q_PLACES = self.toPlaces(TrajectorySolver.q_threshold)
        self.V_PLACES = self.toPlaces(TrajectorySolver.v_threshold)
        self.A_PLACES = self.toPlaces(TrajectorySolver.a_threshold)
        self.T_PLACES = self.toPlaces(TrajectorySolver.t_threshold)

    def toPlaces(self, x):
        """Returns the number of places after the decimal for the first digit of x"""
        # Converting to scientific notation, parsing the string to grab the exponent.
        # Yes, this is hacky. No, I'm not proud (okay, maybe a little).
        return -int(("%e" % x).split('e')[-1])

    #def tearDown(self):
    #    self.foo.dispose()
    #    self.foo = None

    def test_init(self):
        solver = TrajectorySolver(20, 5, 2.5)
        self.assertEqual(solver.v_min, 0)
        self.assertEqual(solver.a_min, -5)
        self.assertEqual(solver.j_min, -2.5)

    def test_target_position_vmax_I(self):
        """Hits max and min accelerations."""
        solver = TrajectorySolver(20, 5, 2.5)

        ### Only position is non-zero at endpoints
        spline = solver.target_position_vmax(Knot(0,0,0,0), Knot(200, 0, 0, 0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 200, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 16, self.PLACES)

        ### Position and velocity are non-zero at endpoints
        spline = solver.target_position_vmax(Knot(100, 2, 0, 0), Knot(300, -2, 0, 0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 16.04, self.PLACES)

        ### Position, velocity and accel are non-zero at endpoints
        spline = solver.target_position_vmax(Knot(100, 2, -2, 0), Knot(300, -2, 2, 0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 300, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 2, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 18.08906666, self.PLACES)

    def test_target_position_vmax_II(self):
        """May not hit max or min accelerations"""

        # Doesn't hit either max or min accel. zero endpoints
        solver = TrajectorySolver(20, 10, 2.5)
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
        solver = TrajectorySolver(10, 5, 2.5)
        spline = solver.target_position_vmax(Knot(0,0,0,0), Knot(40,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 40, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 8, self.PLACES)

    def test_target_position_vmax_III(self):
        """Asymetrical jerk limits."""
        solver = TrajectorySolver(20, 5, 2.5, 0, -5, -5)
        spline = solver.target_position_vmax(Knot(100, 0, 0, 0), Knot(300, 0, 0, 0))
##        self.plot_it(spline, solver)
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
        solver = TrajectorySolver(20, 5, 2.5, 0, -10, -2.5)
        # zero endpoints
        spline = solver.target_position_vmax(Knot(100, 0, 0, 0), Knot(250, 0, 0, 0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
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
        solver = TrajectorySolver(20, 5, 2.5)

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
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 250, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 5, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 5, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 20.33333333333, self.PLACES)

    def test_target_position_vmax_VI(self):
        """Running in reverse."""
        solver = TrajectorySolver(20, 5, 2.5, -20, -5, -2.5)

        # zero endpoints
        spline = solver.target_position_vmax(Knot(250, 0, 0, 0), Knot(0, 0, 0, 0))
##        self.plot_it(spline, solver, "test_target_position_vmax_VI_i")
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 18.5, self.PLACES)

        # Doesn't hit either max or min accel, with non-zero endopints
        solver = TrajectorySolver(5, 10, 2.5, -5, -10, -2.5)
        spline = solver.target_position_vmax(Knot(200,2,-2,0), Knot(0,-4,-1,0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], -4, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -1, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 41.807119601154, self.PLACES)

        # non-zeo endpoints, asymetrical limits
        # FIXME: Exceeding accel limit?
        solver = TrajectorySolver(20, 5, 2.5, -20, -10, -2.5)
        spline = solver.target_position_vmax(Knot(300, 5, 2, 0), Knot(100, 6, -2, 0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 100, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 6, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], -2, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 21.15796029495, self.PLACES)

    def test_target_position_vmax_VII(self):
        """Running in reverse with asymetrical limits"""
        solver = TrajectorySolver(30, 7.5, 5, -10, -2.5, -1)

        # Zero endpoints, negative final position
        spline = solver.target_position_vmax(Knot(0, 0, 0, 0), Knot(-250, 0, 0, 0))
##        self.plot_it(spline, solver, "test_target_position_vmax_VII")
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], -250, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 31.181320796734997, self.PLACES)

    def test_target_position_amax_I(self):
        """Doesn't reach v_max, but does reach a_max"""
        solver = TrajectorySolver(60, 5, 2.5)

        # zero endpoints
        spline = solver.target_position_amax(Knot(0,0,0,0), Knot(200,0,0,0))
##        self.plot_it(spline, solver)
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
        solver = TrajectorySolver(60, 5, 2.5, -60, -5, -2.5)

        # zero endpoints
        spline = solver.target_position_amax(Knot(200,0,0,0), Knot(0,0,0,0))
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 14.8062484748, self.PLACES)

        solver = TrajectorySolver(60, 5, 2.5)

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
        solver = TrajectorySolver(30, 5, 2.5)
        spline = solver.target_position_none(Knot(0,0,0,0), Knot(10,0,0,0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 10, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertTrue(spline.t[-1] < 6)

    def test_target_position_none_II(self):
        """Non-zero endpoints"""
        solver = TrajectorySolver(30, 5, 2.5)
        spline = solver.target_position_none(Knot(0,5,0,0), Knot(10,0,0,0))
#        print "SPLINE:", spline
##        self.plot_it(spline, solver, 'test_target_position_none_II')
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 10, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertTrue(spline.t[-1] < 5)


    def test_target_position_none_III(self):
        """Travelling in reverse."""
        solver = TrajectorySolver(30, 5, 2.5, -30, -5, -2.5)
        spline = solver.target_position_none(Knot(0,0,0,0), Knot(-10,0,0,0))
##        self.plot_it(spline, solver, "test_target_position_none_III")
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], -10, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertTrue(spline.t[-1] < 6)


    def test_target_position(self):
        solver = TrajectorySolver(20, 5, 2.5)

        # No change from initial to final
        self.assertRaises(FatalTrajectoryError,
                          solver.target_position, Knot(0,0,0,0), Knot(0,0,0,0) )

    def test_target_position_II(self):
        """Can't backup due to v_min=0, but final position is behind start position."""
        solver = TrajectorySolver(20, 5, 2.5, 0, -5, -2.5)
        self.assertRaises(FatalTrajectoryError,
                          solver.target_position, Knot(100, 0, 0, 0), Knot(90, 0, 0, 0) )

    def test_target_position_III(self):
        """Checks to make sure that it's using the right solver."""
        solver = TrajectorySolver(32, 1.0, 2.5, 0, -5.0, -2.5)
        spline = solver.target_position(Knot(0.0, 10.680000305175781, -4.8299999237060547, 27.727),
                                        Knot(57.145143987525579, 0, 0, None))
##        self.plot_it(spline, solver, "test_target_position_III")
        self.validate_spline(spline, solver)
        self.assertAlmostEqual(spline.q[-1], 57.145143987525579, 5)
        self.assertAlmostEqual(spline.v[-1], 0, 5)
        self.assertAlmostEqual(spline.a[-1], 0, 5)
        self.assertTrue(spline.t[-1] - spline.t[0] <25)

    def test_target_position_IV(self):
        """Test from a real-world solver failure."""
        solver = TrajectorySolver(32, 1.0, 2.5, -5.0, -5.0, -2.5)
        try:
            spline = solver.target_position(Knot(0, 10.573277473449707, -5.0, 303.644),
                                            Knot(68.925505241605777, 0, 0, None))
##            self.plot_it(spline, solver, "test_target_position_IV")
            self.validate_spline(spline, solver)
            self.assertAlmostEqual(spline.q[-1], 68.925505241605777, 5)
            self.assertAlmostEqual(spline.v[-1], 0, 5)
            self.assertAlmostEqual(spline.a[-1], 0, 5)
            self.assertTrue(spline.t[-1] - spline.t[0] < 30)

        except OptimizationError:
            spline = solver._soln_spline
##            self.plot_it(spline, solver, "test_target_position_IV")
            self.assertTrue(False)

    def test_target_position_V(self):
        """Yet another real-world problem case."""
        solver = TrajectorySolver(32, 1., 2.5, -5.0, -5.0, -2.5)
        try:
            spline = solver.target_position(Knot(0.0, 12.135705947875977, -5.0, 220.824),
                                            Knot(76.021737702490753, 0, 0, None))
##            self.plot_it(spline, solver, "test_target_position_V")
            self.validate_spline(spline, solver)
            self.assertAlmostEqual(spline.q[-1], 76.021737702490753, 5)
            self.assertAlmostEqual(spline.v[-1], 0, 5)
            self.assertAlmostEqual(spline.a[-1], 0, 5)
            self.assertTrue(spline.t[-1] - spline.t[0] < 30)
        except OptimizationError:
            spline = solver._soln_spline
##            self.plot_it(spline, solver, "test_target_position_V")
            self.assertTrue(False)

    def test_target_position_VI(self):
        """Yet another real-world problem case."""
        solver = TrajectorySolver(32, 1., 2.5, -5.0, -5.0, -2.5)
        try:
            spline = solver.target_position(Knot(0.0, 10.160758018493652, -5.0, 28.024999999999999),
                                            Knot(67.214868918397073, 0, 0, None))
##            self.plot_it(spline, solver, "test_target_position_VI")
##            self.validate_spline(spline, solver)
            self.assertAlmostEqual(spline.q[-1], 67.214868918397073, 5)
            self.assertAlmostEqual(spline.v[-1], 0, 5)
            self.assertAlmostEqual(spline.a[-1], 0, 5)
            self.assertTrue(spline.t[-1] - spline.t[0] < 60)
        except OptimizationError, err:
            import random
            while True:
                try:
                    new_durations = [random.random() for h in err.durations]
                    print "initial hs:", new_durations
                    solver.target_position_none(Knot(0.0, 10.160758018493652, -5.0, 28.024999999999999),
                                                Knot(67.214868918397073, 0, 0, None),
                                                new_durations)
                except OptimizationError as err:
                    print "Soln hs:", solver._soln_spline.h
                    self.plot_it(solver._soln_spline, solver, "test_remix...")
                else:
                    print "Soln hs:", solver._soln_spline.h
                    self.plot_it(solver._soln_spline, solver, "test_remix...")
                    break

    def test_target_position_VII(self):
        """Can this be done without resorting to optimize routine?"""
        solver = TrajectorySolver(60, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0, 15.5, -1.5, 0)
        final = Knot(42.5, 2.4, 0, None)
        spline = solver.target_position(initial, final, max_speed=initial.vel)
##        self.plot_it(spline, solver, "test_target_position_VII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, time=False)

    def test_target_position_VIII(self):
        """Starts with 0 velocity, ends with max velocity."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,0,0,0)
        final = Knot(500, 40, 0, None)
        spline = solver.target_position(initial, final)
##        self.plot_it(spline, solver, "test_target_position_VIII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, time=False)

##    def test_target_position_IX(self):
##        """Testing with a start point that is outside of the constraints"""
##        solver = TrajectorySolver(5, 5, 2.5, 0, -5, -2.5)
##        spline = solver.target_position(Knot(0, 10, -1, 0), Knot(100, 0, 0, None))
##        self.plot_it(spline, solver, "test_target_position_IX")
##        self.validate_spline(spline, solver)
##        self.assertAlmostEqual(spline.q[-1], 100, self.PLACES)
##        self.assertAlmostEqual(spline.v[-1], 0, self.PLACES)
##        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)

    def test_target_velocity_I(self):
        """Accelerating. Reaches a_max"""
        solver = TrajectorySolver(40, 5, 2.5)

        ### zero at endpoints
        spline = solver.target_velocity(Knot(0,0,0,0), Knot(None, 35, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 9, self.PLACES)

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
        solver = TrajectorySolver(40, 40, 2.5)

        ### zero at endpoints
        spline = solver.target_velocity(Knot(0,0,0,0), Knot(None, 35, 0, 0))
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.v[-1], 35, self.PLACES)
        self.assertAlmostEqual(spline.a[-1], 0, self.PLACES)
        self.assertAlmostEqual(spline.t[-1], 7.4833147735, self.PLACES)

        # Non-zero endpoints
        initial = Knot(10,2,1,0)
        final = Knot(None, 35, 3, 5.883314773)
        spline = solver.target_velocity(initial, final)
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False)

    def test_target_velocity_III(self):
        """Decelerating. Reaches a_max"""
        solver = TrajectorySolver(40, 5, 2.5)

        ### zero at endpoints
        initial = Knot(0,35,0,0)
        final = Knot(None, 0, 0, 0)
        spline = solver.target_velocity(initial, final)
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)

        # Non-zero endpoints
        initial = Knot(10,35,1,0)
        final = Knot(None, 2, -1, 0)
        spline = solver.target_velocity(initial, final)
#        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)
        self.assertAlmostEqual(spline.t[-1],  8.6799999999, self.PLACES)

    def test_target_velocity_IV(self):
        """Asymmetrical limits."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -20, -1.25)

        ### Decelerating, zero at endpoints
        initial = Knot(0,35,0,0)
        final = Knot(None, 0, 0, 0)
        spline = solver.target_velocity(initial, final)
##        self.plot_it(spline, solver)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)

    def test_target_velocity_V(self):
        solver = TrajectorySolver(15, 10, 5.0, 0, -10, -5.0)
        initial = Knot(0, 11.51496564563041, 5.6694744752383066, 0)
        final = Knot(None, 14.732396990426981, 0, None)
        spline = solver.target_velocity(initial, final)
        self.plot_it(spline, solver, "test_target_velocity_V")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)

    def test_target_velocity_nonzero_endpoints_I(self):
        """Initial accel in same direction as initial velocity. Asymmetrical limits"""
        solver = TrajectorySolver(40, 5, 2.5, 0, -20, -1.25)
        initial = Knot(10,2,1,0)
        final = Knot(None,35,3,None)
        spline = solver.target_velocity(initial, final)
##        self.plot_it(spline, solver, "test_target_velocity_nonzero_endpoints_I")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)
        self.assertTrue(spline.q[-1] > 0)
        self.assertAlmostEqual(spline.t[-1], 7.56, self.PLACES)

    def test_target_velocity_nonzero_endpoints_II(self):
        """Velocity is exactly target, after initial acceleration is dissipated."""
        solver = TrajectorySolver(40, 5, 5)
        initial = Knot(0,0,5,0)
        final = Knot(None,2.5,0,None)
        spline = solver.target_velocity(initial, final)
##        self.plot_it(spline, solver, "test_target_velocity_nonzero_endpoints_II")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)

    def test_target_velocity_no_delta_v_I(self):
        """Simple case. No acceleration involved."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -20, -1.25)
        initial = Knot(0, 20, 0, 0)
        final = Knot(None, 20, 0, None)
        spline = solver.target_velocity(initial, final)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)

    def test_target_velocity_no_delta_v_II(self):
        """Velocity is already at target, but there's an initial acceleration.
        Asymmetric jerk limits."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -20, -1.25)
        initial = Knot(0, 20, 5, 0)
        final = Knot(None, 20, 0, None)
        spline = solver.target_velocity(initial, final)
##        self.plot_it(spline, solver, "test_target_velocity_no_delta_v_II")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, pos=False, time=False)

    def test_target_time_I(self):
        """Uses zero for accel and velocity endpoints."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,0,0,0)
        final = Knot(1000, 0, 0, 54)
        spline = solver.target_time(initial, final) # approx 20 m/s cruise, ignoring jerk limits
##        self.plot_it(spline, solver, "test_target_time_I")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_II(self):
        """Non-zero endpoints"""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(100, 10 , 3, 50)
        final = Knot(1000, 2, -1, 100)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_II")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_III(self):
        """Insufficient time to achieve target due to low vehicle v_max"""
        solver = TrajectorySolver(10, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,0,0,0)
        final = Knot(1000, 0, 0, 54)
        self.assertRaises(FatalTrajectoryError,
                          solver.target_time, initial, final) # needs approx 20 m/s cruise, ignoring jerk limits

    def test_target_time_IV(self):
        """Vehicle needs to slow down to achieve target time"""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,40,0,0)
        final = Knot(1000,40,0,54)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_IV")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_V(self):
        """Very short distance, but plenty of time. Does not have a constant acceleration segment."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,0,0,0)
        final = Knot(5,0,0,10)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_V")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_VI(self):
        """Vehicle needs to exactly maintain current speed to achieve target time"""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,40,0,0)
        final = Knot(1000,40,0,25)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_VI")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_VII(self):
        """A problem that cropped up during testing."""
        solver = TrajectorySolver(60, 5, 2.5, 0, -5, -2.5)
        initial = Knot(93.318734132069295, 16.0, 0.0, 5.2)
        final = Knot(307.3406064472577, 16.0, 0, 18.95297859206239)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_VII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_VIII(self):
        """Another problem that occured during testing"""
        solver = TrajectorySolver(60, 5, 2.5, 0, -5, -2.5)
        initial = Knot(42.629766162611496, 7.8664068437709354, 0, 47.65161563986365)
        final = Knot(351.8905858935297, 15.0, 0, 73.876510498199906)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_VIII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_IX(self):
        """Problem from testing. Initial velocity is close to the cruise velocity."""
        solver = TrajectorySolver(60, 5, 2.5, 0, -5, -2.5)
        initial = Knot(47.529766162611494, 7.8664068437709354, 0, 43.972657903425244)
        final = Knot(309.26081973091823, 15.0, 0, 75.376510498199906)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_IX")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_X(self):
        """Problem from testing. Initial velocity is very close to ave velocity."""
        solver = TrajectorySolver(15, 5, 2.5, 0, -5, -2.5)
        initial = Knot(47.529766162611494, 11.907670085181412, 0, 101.08822071657993)
        final = Knot(309.26081973091823, 15.0, 0, 122.74317716486676)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_X")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XI(self):
        """Problem from testing."""
        solver = TrajectorySolver(15, 5, 2.5, 0, -5, -2.5)
        initial = Knot(4.8999999999999986, 0.0, 0.0, 81.599999999999994)
        final = Knot(309.26081973091823, 15.0, 0, 104.90984383153337)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_XI")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XII(self):
        """Problem from testing. Unachievable."""
        solver = TrajectorySolver(15, 5, 2.5, 0, -5, -2.5)
        initial = Knot(2.0, 15.0, 0.0, 5.4795365136060168)
        final = Knot(70.901034075983588, 15.0, 0, 11.706272118671588) # ave speed of 11.065 m/s
        solver.target_time(initial, final)
        self.assertRaises(FatalTrajectoryError,
                          solver.target_time, initial, final )

    def test_target_time_XIII(self):
        """Cruise velocity is very near to initial and final velocities."""
        solver = TrajectorySolver(15, 5, 2.5, 0, -5, -2.5)
        initial = Knot(52.5, 14.99716768771842, 0, 1530.8197971206223)
        final = Knot(267.03399999999999, 15.0, 0, 1545.1247333079007)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_XIII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XIV(self):
        """Problem from testing."""
        solver = TrajectorySolver(15, 5, 2.5, 0, -5, -2.5)
        initial = Knot(9.9000001907348629, 0.0, 0.0, 1978.7999999999861)
        final = Knot(267.03399999999999, 15.0, 0, 2002.9280666602958)
        spline = solver.target_time(initial, final)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XV(self):
        """Problem from testing."""
        solver = TrajectorySolver(15, 5, 2.5, 0, -5, -2.5)
        initial = Knot(52.500999999999998, 14.917710067720561, 0, 2924.9500320272209)
        final = Knot(643.9559999999999, 15.0, 0, 2964.5960666793744)
        spline = solver.target_time(initial, final)
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XVI(self):
        """Problem from testing."""
        solver = TrajectorySolver(60, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0.0, 0.0, 0, 0.0)
        final = Knot(57.926000000000002, 15.0, 0, 7.1870000000000003)
        spline = solver.target_time(initial, final)
        self.plot_it(spline, solver, "test_target_time_XVI")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XVII(self):
        """Problem from testing. Positive starting acceleration and velocity.
        Leads to negative 0->1 duration."""
        solver = TrajectorySolver(15, 10, 5.0, 0, -10, -5.0)
        initial = Knot(52.503999999999998, 11.51496564563041, 5.6694744752383066, 55.354144791110656)
        final = Knot(643.97199999999998, 15.0, 0, 95.582584140902242)
        spline = solver.target_time(initial, final, max_speed=15.0)
##        self.plot_it(spline, solver, "test_target_time_XVII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_XVIII(self):
        solver = TrajectorySolver(15, 10, 5.0, 0, -10, -5.0)
        initial = Knot(23.973000000002685, 15.0, 0.0, 1154.2775841472599)
        final = Knot(277.55799999999999, 15.0, 0, 1172.0256197255005)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_XVIII")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_target_time_nearly_stops(self):
        """Initial knot has a positive velocity, and the vehicle needs to
        almost come to a halt in order to arrive on time. Check that
        the vehicle does not reverse in order to meet the schedule.
        """
        solver = TrajectorySolver(20, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,20,0,0)
        final = Knot(500,20,0,1000)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_nearly_stops")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)
        self.assertTrue(spline.get_min_velocity() >= 0)

    def test_target_time_increase_vel(self):
        """Vehicle starts with zero velocity and ends with a velocity that is
        higher than the 'cruising velocity' required to hit the target time."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(0,0,0,0)
        final = Knot(1000, 40, 0, 54)
        spline = solver.target_time(initial, final)
##        self.plot_it(spline, solver, "test_target_time_increase_vel")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final)

    def test_time_TODO(self):
        """
         - More tests where the vehicle makes a SMALL slip forward or back.
         - Negative position changes.
        """

    def test__no_acceleration_predict_I(self):
        """Has initial accel and velocity"""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(10,2,1,0)
        final = Knot(80,0,0,None)
        spline = solver._no_acceleration_predict(initial, final)
##        self.plot_it(spline, solver, "_no_acceleration_predict_I")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, time=False)

    def test__no_acceleration_predict_II(self):
        """Starts with 0 vel, 0 accel. No accel prediction is mostly meaningless."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(10,0,0,0)
        final = Knot(50,5,-1,None)
        spline = solver._no_acceleration_predict(initial, final)
        self.assertEqual(spline.t[-1], float('inf'))

    def test__no_acceleration_predict_III(self):
        """Has initial vel, no initial accel. Accelerates."""
        solver = TrajectorySolver(40, 5, 2.5, 0, -5, -2.5)
        initial = Knot(10,2,0,0)
        final = Knot(50,10,0,None)
        spline = solver._no_acceleration_predict(initial, final)
##        self.plot_it(spline, solver, "_no_acceleration_predict_III")
        self.validate_spline(spline, solver)
        self.validate_endpoints(spline, initial, final, time=False)

    def test_slip(self):
        original_solver = TrajectorySolver(20, 5, 2.5, 0, -5, -2.5)
        original_spline = original_solver.target_position(Knot(0,0,0,0), Knot(3000,0,0,0))
##        self.plot_it(original_spline, original_solver, "original_test_slip")

        slip_dists = [-1000, -500, -30, -1, -0.25, 0.25, 1, 30, 500]
        slip_solver = TrajectorySolver(30, 5, 2.5, 0, -5, -2.5) # higher max speed allows vehicle to slip ahead
        ti = original_spline.t[3] # beginning of the constant velocity segment
        for slip_dist in slip_dists:
            slip_spline = slip_solver.slip(original_spline, ti+1, slip_dist)
##            self.plot_it(slip_spline, slip_solver, "test_slip")
            self.validate_spline(slip_spline, slip_solver)

            self.assertAlmostEqual(original_spline.q[-4], slip_spline.q[-4], self.PLACES) # position at end of constant velocity segment unchanged
            self.assertAlmostEqual(original_spline.t[-4] - slip_dist/20.0, slip_spline.t[-4], self.PLACES) # but arrival time is shifted by the correct amount

    def validate_spline(self, spline, solver):
        # all the timespans are non-negative
        for h in spline.h:
            self.assertTrue(h >= 0)

        # all the t points agree with the timespans
        for h, ti, tf in zip(spline.h, spline.t[:-1], spline.t[1:]):
            self.assertAlmostEqual(h, tf-ti, self.PLACES)

        # check that jerk is one of: j_min, 0, or j_max
        for j in spline.j:
            self.assertTrue(j in (solver.j_max, 0, solver.j_min))

        # all the knots are valid
        for i in range(len(spline.t)-1):
            jerk = spline.j[i]
            self.assertAlmostEqual(jerk*spline.h[i] + spline.a[i], spline.a[i+1], self.A_PLACES)
            self.assertAlmostEqual(jerk*spline.h[i]**2/2 + spline.a[i]*spline.h[i] + spline.v[i], spline.v[i+1], self.V_PLACES)
            self.assertAlmostEqual(jerk*spline.h[i]**3/6 + spline.a[i]*spline.h[i]**2/2 + spline.v[i]*spline.h[i] + spline.q[i], spline.q[i+1], self.Q_PLACES)

    def validate_endpoints(self, spline, initial, final, pos=True, vel=True, accel=True, time=True):
        """Checks that the splines endpoints match initial in all fields, and
        match final in the selected fields."""
        self.assertAlmostEqual(initial.pos, spline.q[0], self.Q_PLACES)
        self.assertAlmostEqual(initial.vel, spline.v[0], self.V_PLACES)
        self.assertAlmostEqual(initial.accel, spline.a[0], self.A_PLACES)
        self.assertAlmostEqual(initial.time, spline.t[0], self.T_PLACES)

        if pos:   self.assertAlmostEqual(final.pos, spline.q[-1], self.Q_PLACES)
        if vel:   self.assertAlmostEqual(final.vel, spline.v[-1], self.V_PLACES)
        if accel: self.assertAlmostEqual(final.accel, spline.a[-1], self.A_PLACES)
        if time:  self.assertAlmostEqual(final.time, spline.t[-1], self.T_PLACES)

    def splinesAlmostEqual(self, a, b):
        self.assertAlmostEqual(a.pos, b.pos, self.Q_PLACES)
        self.assertAlmostEqual(a.vel, b.vel, self.V_PLACES)
        self.assertAlmostEqual(a.accel, b.accel, self.A_PLACES)
        self.assertAlmostEqual(a.time, b.time, self.T_PLACES)

    def plot_it(self, spline, solver, title=""):
        """Convience function for debugging"""
        from pyprt.shared.cspline_plotter import CSplinePlotter
        plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min, title)
        plotter.display_plot()

if __name__ == '__main__':
    unittest.main()



