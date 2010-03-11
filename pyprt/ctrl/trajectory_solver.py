from __future__ import division # use floating point division unless explicitly otherwise

import random
import math

import scipy.optimize
from scipy import inf

from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import Knot
from pyprt.shared.utility import pairwise

class TrajectoryError(Exception):
    """Raised when a generated trajectory is known to be flawed."""

class FatalTrajectoryError(Exception):
    """Raised when the specified trajectory cannot be met, such as when an
    initial or final value is outside of the specified acceleration or velocity
    limits."""

class OptimizationError(Exception):
    """Raised when a optimizer is used, and fails to converge to correct solution."""
    def __init__(self, residual_error=None, durations=None):
        self.residual_error = residual_error
        self.durations = durations
    def __str__(self):
        return "residual_error: %f, spline durations: %f" % (self.residual_error,
                                                             self.durations)

class TrajectorySolver(object):
    """Generates C2 cubic splines that are a time-optimal 1D trajectory.

    Note that if you are going to generate trajectories that travel in reverse
    (i.e. the final displacement is less than the initial displacement), then
    the acceleration profiles used are vertically mirrored. This is achieved by
    flipping the signs of the velocity, acceleration, and jerk limits.

    The end result is that when travelling in a reverse direction*, v_max is still
    your maximum speed (in the reverse direction), a_max is still how quickly you
    pick up speed, and a_min is how quickly you'll brake. Note that the passengers
    will still experience jerk_max during acceleration, but they will be shoved
    towards the front of the vehicle, and not towards the rear (assuming vehicle
    was not turned around). If you have asymmetrical limits, consider using a
    separate TrajectorySolver instance for generating reverse trajectories.

    * What is considered to be 'forward' and what is 'reverse' are determined
        by the track coordinates, not the vehicle's pose. So long as it's heading
        towards the track segment's origin, then the vehicle is traveling in
        reverse for trajectory generation purposes.
    """
    v_threshold = 0.0001   # == 0.0002 miles per hour
    a_threshold = 0.0001

    def __init__(self, velocity_max, acceleration_max, jerk_max, velocity_min=None, acceleration_min=None, jerk_min=None):
        self.v_max = velocity_max
        self.a_max = acceleration_max
        self.j_max = jerk_max

        self.v_min = 0 if velocity_min is None else velocity_min
        self.a_min = -self.a_max if acceleration_min is None else acceleration_min
        self.j_min = -self.j_max if jerk_min is None else jerk_min

        self._soln_spline = None

    def create_knot_after(self, knot, h, accel, no_pos=False):
        """Creates a knot.
         knot: The knot from which to extend the new knot.
         h: The time interval between knot and the new knot.
         accel: The acceleration at the knot being created.
        """
        if h == 0:
            return knot.copy()

        j = (accel - knot.accel)/h

        # final velocity
        hh = h*h
        vf = j*hh/2 + knot.accel*h + knot.vel

        # final displacement
        if no_pos:
            qf = None
        else:
            hhh = hh*h
            qf = j*hhh/6 + knot.accel*hh/2 + knot.vel*h + knot.pos

        return Knot(qf, vf, accel, knot.time + h)

    def create_knot_before(self, knot, h, accel, no_pos=False):
        """Creates a knot.
         knot: The knot from which to extend the new knot.
         h: The time interval between knot and the new knot.
         accel: The acceleration at the knot being created.
        """
        if h == 0:
            return knot.copy()

        j = (knot.accel - accel)/h

        # velocity
        ti = -h
        ti_square = ti*ti
        vi = j*ti_square/2 + knot.accel*ti + knot.vel

        # displacement
        if no_pos:
            qi = None
        else:
            ti_cube = ti_square*ti
            qi = j*ti_cube/6 + knot.accel*ti_square/2 + knot.vel*ti + knot.pos


        return Knot(qi, vi, accel, knot.time - h)

    def target_position(self, knot_initial, knot_final, fnc_info=False, max_attempts=10):
        """Targets the position and velocity values in knot_final. The time
        supplied by knot_final is ignored, and may be set to None.

        Raises FatalTrajectoryError if it is unable to generate a valid spline.
        Raises OptimizationError if a optimization routine was used and
           the residual error is greater than 'error_threshold'. Exception
           contains the residual error value and the final h array.

        Returns a cubic_spline.CSpline. If fnc_info is True, then returns the
        function used to find the solution in addition to the spline: (spline, fnc)
        """
        knot_final = knot_final.copy() # don't alter the original
        knot_final.time = inf

        # Basic error checking. Bail if the initial or final conditions lie
        # outside of the velocity and/or acceleration constaints.
        if knot_final.pos > knot_initial.pos:
            flip = True
            if __debug__:
                if not (self.v_min - self.v_threshold <= knot_initial.vel <= self.v_max + self.v_threshold and
                        self.v_min - self.v_threshold <= knot_final.vel <= self.v_max + self.v_threshold and
                        self.a_min - self.a_threshold <= knot_initial.accel <= self.a_max + self.a_threshold and
                        self.a_min - self.a_threshold <= knot_final.accel <= self.a_max + self.a_threshold):
                    raise FatalTrajectoryError("Endpoint outside of solver's limit")
        elif knot_final.pos < knot_initial.pos: # travelling in reverse
            flip = False
            if __debug__:
                if not (self.v_min - self.v_threshold <= -knot_initial.vel <= self.v_max + self.v_threshold and
                        self.v_min - self.v_threshold <= -knot_final.vel <= self.v_max + self.v_threshold and
                        self.a_min - self.a_threshold <= -knot_initial.accel <= self.a_max + self.a_threshold and
                        self.a_min - self.a_threshold <= -knot_final.accel <= self.a_max + self.a_threshold):
                    raise FatalTrajectoryError("Endpoint outside of solver's limit")
        else: # knot_final.pos == knot_initial.pos
            # Could return a spline, but in what form? 0 knots, 1 knot, 2 knots?
            raise FatalTrajectoryError("You didn't bloody well want to move then, did ya?")

        # Generate the spline
        try:
            fnc = self.target_position_vmax
            spline = self.target_position_vmax(knot_initial, knot_final)
        except TrajectoryError:
            try:
                fnc = self.target_position_amax
                spline = self.target_position_amax(knot_initial, knot_final)
            except TrajectoryError:
                attempts=1
                durations = None
                while True:
                    try:
                        fnc = self.target_position_none
                        spline = self.target_position_none(knot_initial, knot_final, durations)
                    except OptimizationError as err:
                        if attempts > max_attempts:
                            raise
                        else:
                            durations = [random.random() for h in err.durations]
                            attempts += 1
                    else:
                        break

        # Additional error checking on the generated trajectory, ensuring that
        # the constraints were actually respected.
        if __debug__:
            if min(spline.h) < 0:
                raise FatalTrajectoryError("Generated a trajectory with a negative duration poly.")

            if not flip:
                if spline.get_max_velocity() > self.v_max + self.v_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with max velocity limit.")
                if spline.get_min_velocity() < self.v_min - self.v_threshold:
                    ##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with min velocity limit.")
                if spline.get_max_acceleration() > self.a_max + self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with max acceleration limit.")
                if spline.get_min_acceleration() < self.a_min - self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with min acceleration limit.")

            else:
                if -spline.get_max_velocity() > self.v_max + self.v_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with max velocity limit.")
                if -spline.get_min_velocity() < self.v_min - self.v_threshold:
                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with min velocity limit.")
                if -spline.get_max_acceleration() > self.a_max + self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with max acceleration limit.")
                if -spline.get_min_acceleration() < self.a_min - self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError("Unable to comply with min acceleration limit.")
        if fnc_info:
            return spline, fnc
        else:
            return spline

    def target_position_vmax(self, initial, final):
        """Assumes that max velocity is reached. So long as v_max is reached,
        can handle a_max being reached or not. If a_max and/or a_min are not reached,
        then h1 and/or h5 will equal 0.

        Assumed that the spline should have an acceleration profile of:
          h0  h1  h2  h3   h4  h5  h6
             ____
            /    \
           /      \
          /        \______
                          \          /
                           \        /
                            \______/

         t0 t1  t2 t3    t4 t5    t6 t7

        where h's are the timespans of each polynomial, and t's are the knot times.

        h0: roll on acceleration until a_top is reached
        h1: hold a_top acceleration until v_top is nearly reached
        h2: roll off acceleration, bringing us to constant velocity at v_top
        h3: cruise at constant velocity, v_top (accel = 0)
        h4: roll on the brakes until a_bot is reached
        h5: hold max deceleration until v_final is nearly reached
        h6: roll off of the brakes, leaving us at v_final

        Returns a new spline.
        Raises TrajectoryError if unable to find a valid solution.

        Notes:
        Accelerations and velocities at t0 and t7 are not necessarily 0.
        If the final position is behind the initial position, then the profile
          will be mirrored vertically.
        """
        final.time = inf

        qi = initial.pos
        vi = initial.vel
        ai = initial.accel
        ti = initial.time

        qf = final.pos
        vf = final.vel
        af = final.accel

        if qf > qi: # moving in a forward direction
            flip = False
            jx = self.j_max
            jn = self.j_min
            ax = self.a_max
            an = self.a_min
            vx = self.v_max
            vn = self.v_min

        else: # moving in a reverse direction
            flip = True
            jx = -self.j_max
            jn = -self.j_min
            ax = -self.a_max
            an = -self.a_min
            vx = -self.v_max
            vn = -self.v_min

        # Assume that a_max will not be reached, in which case the beginning
        # of the acceleration profile will look like:
        #
        #   /\
        #  /  \
        # /    \___ ...
        # 0 1  2
        # We can solve for a_top, the maximum acceleration that is reached.
        #
        # Note: in the eqns below *x = max; *n = min; at = a_top; ab = a_bot
        #       and v01 indicates the delta_v from knot 0 to knot 1
        #
        # Setting up, we find that:
        # at = jx*h0 + a0  or equivalently  h0 = (at - a0)/jx
        # h1 = (0-at)/ jn
        # v01 = jx*h0**2/2 + a0*h0
        # v12 = jn*h1**2/2 + at*h1
        #
        # Using the constraint vx - v0 = v01 + v12, and putting the above
        # formula all in terms of a_top, we use the quadratic formula to find
        # the solution for a_top.
        #                                      2
        #                2 /   1      1  \   a0
        #    vx - v0 + at *|  ---- - ----| + ----  =  0
        #                  \  2*jn   2*jx/   2*jx

        _a = 1/(2*jn) - 1/(2*jx)
        _b = 0
        _c = vx - vi + ai**2/(2*jx)

        # For the remainder of the calculations use the given a_max constraint
        # or the 'natural' acceleration maxima, whichever is less.
        if not flip:
            a_top = max(self.nonnegative_roots(_a, _b, _c))
            a_top = min(ax, a_top)
        else:
            a_top = min(self.nonpositive_roots(_a, _b, _c))
            a_top = max(ax, a_top)


        # Similarly for a_bot, the minimum acceleration that is reached. (i.e. a_bottom)
        #       0 1  2
        # ... __
        #       \    /
        #        \  /
        #         \/
        #
        # ab = jn*h0 + 0  or equivalently   h0 = ab/jn
        # h1 = (a2-ab)/jx
        # v01 = jn*h0**2/2 + 0*h0
        # v12 = jx*h1**2/2 + ab*h1
        #
        # Using the constraint v2 - vx = v01 + v12 solve for ab:
        #
        #                                 2
        #             2 / 1      1  \   a2
        # v2 - vx + ab *|---- - ----| - ----
        #               \2*jx   2*jn/   2*jx

        _a = 1/(2*jx) - 1/(2*jn)
        _b = 0
        _c = vf - vx - af**2/(2*jx)
        if not flip:
            a_bot = min(self.nonpositive_roots(_a, _b, _c))
            a_bot = max(an, a_bot)
        else:
            a_bot = max(self.nonnegative_roots(_a, _b, _c))
            a_bot = min(an, a_bot)

        ###  create the first three knots
        h0 = (a_top - ai)/jx
        k1 = self.create_knot_after(initial, h0, a_top)

        h2 = -a_top/jn
        v23 = jn*h2**2/2 + a_top*h2 # delta v from knot 2 -> 3

        v12 = vx - k1.vel - v23 # from v_max - v1 = v12 + v23
        h1 = 0 if (a_top < ax) else v12/a_top

        k2 = self.create_knot_after(k1, h1, a_top)
        k3 = self.create_knot_after(k2, h2, 0)

        ###  create the last three knots
        h6 = (af - a_bot)/jx
        k6 = self.create_knot_before(final, h6, a_bot)

        h4 = a_bot/jn
        v45 = jn*h4**2/2 # delta v from knot 4 -> 5. Similar to v23, but note that a4 == 0.

        v56 = k6.vel - vx - v45 # from v6 - v_4 = v45 + v56. Note that v_4 == v_max
        h5 = 0 if (a_bot > an) else v56/a_bot

        k5 = self.create_knot_before(k6, h5, a_bot)
        k4 = self.create_knot_before(k5, h4, 0)

        ###  calculate the time to cruise at a constant v_max
        h3 = (k4.pos - k3.pos)/vx

        if h3 < 0:
            raise TrajectoryError('Vmax is not reached')

        # the times for k4 through k6 are all inf, so fix them now
        k4.time = k3.time + h3
        k5.time = k4.time + h4
        k6.time = k5.time + h5
        final.time = k6.time + h6

        # Create a spline, while checking that durations are positive
        q, v, a, t = [qi], [vi], [ai], [ti]
        for k1, k2 in pairwise([initial, k1, k2, k3, k4, k5, k6, final]):
            if k2.time > k1.time: # Normal case
                q.append(k2.pos)
                v.append(k2.vel)
                a.append(k2.accel)
                t.append(k2.time)
            elif (k2.time - k1.time) >= -0.0001: # Negative or zero duration, but only by rounding errors
                pass # Skip the knot
            else: # large negative duration
                raise TrajectoryError("Large negative duration: %f seconds" % (k2.t - k1.t))
        spline = CubicSpline(q, v, a, t)

        return spline

    def target_position_amax(self, initial, final):
        """Trajectory does not reach v_max, but does reach a_max.
        Assumed that the spline should have an acceleration profile of:

          h0  h1    h2  h3   h4
             ____
            /    \
           /      \
          /        \
                    \          /
                     \        /
                      \______/

         t0 t1  t2    t3    t4 t5

        where h's are the timespans of each polynomial, and t's are the knot times.
        """
        q0 = initial.pos
        v0 = initial.vel
        a0 = initial.accel

        if final.pos > initial.pos:
            flip = False
            an = self.a_min
            ax = self.a_max
            jn = self.j_min
            jx = self.j_max
        else:
            flip = True
            an = -self.a_min
            ax = -self.a_max
            jn = -self.j_min
            jx = -self.j_max

        h0 = (ax - a0)/jx
        k1 = self.create_knot_after(initial, h0, ax)

        h2 = (an-ax)/jn

        h4 = (final.accel-an)/jx
        k4 = self.create_knot_before(final, h4, an)

        # From v12 = v4 - v1 - v23 - v34 and v12 = ax*h1 we get:
        # h1 = alpha - an/ax*h3
        alpha = (k4.vel-k1.vel)/ax + ax/(2*jn) - an**2/(2*ax*jn)
#        v2 = k1.vel + v12
#        q23 = jn*h2**3/6 + ax*h2**2/2 + v2*h2

        # From trajectory_calcs.py, Problem 1:
        #              /                              2      2                 \       /       2 \                      2                                         2               2            3
        #              |                            an     ax     an*v1   an*ax|     2 |an   an  |              alpha*ax    ax*v1   an*v1   alpha*an*ax   ax*alpha    ax*(an - ax)    (an - ax)
        # q4 - q1 + h3*|-v1 + alpha*an - alpha*ax + ---- + ---- + ----- - -----| + h3 *|-- - ----| - alpha*v1 + --------- + ----- - ----- - ----------- - --------- - ------------- - ----------
        #              \                            2*jn   2*jn     ax      jn /       \2    2*ax/                  jn        jn      jn         jn           2               2             2
        #                                                                                                                                                                 2*jn          6*jn


        # Now, solve for h3 using quadratic formula
        A = an/2 - an**2/(2*ax)
        B = -k1.vel + alpha*an - alpha*ax + an+ax**2/(2*jn) + an*k1.vel/ax - an*ax/jn
        C = k4.pos - k1.pos - alpha*k1.vel + (alpha*ax**2 + ax*k1.vel - an*k1.vel - alpha*an*ax)/jn - ax*alpha**2/2 - ax*(an - ax)**2/(2*jn**2) - (an - ax)**3/(6*jn**2)

        try:
            h3 = min(self.nonnegative_roots(A, B, C))
        except ValueError: # No nonnegative, real solutions. a_min is not reached.
            assert len(self.nonnegative_roots(A, B, C)) == 0
            raise TrajectoryError('No nonnegative, real solutions')

        h1 = alpha - an/ax*h3

        k2 = self.create_knot_after(k1, h1, ax)
        k3 = self.create_knot_after(k2, h2, an)
        k4 = self.create_knot_after(k3, h3, an)
        final.time = k4.time + h4

        # Create a spline, while checking that durations are positive
        q, v, a, t = [q0], [v0], [a0], [initial.time]
        for k1, k2 in pairwise([initial, k1, k2, k3, k4, final]):
            if k2.time > k1.time: # Normal case
                q.append(k2.pos)
                v.append(k2.vel)
                a.append(k2.accel)
                t.append(k2.time)
            elif (k2.time - k1.time) >= -0.0001: # Negative or zero duration, but only by rounding errors
                pass # Skip the knot
            else: # large negative duration
                raise TrajectoryError("Large negative duration: %f seconds" % (k2.time - k1.time))
        spline = CubicSpline(q, v, a, t)

        return spline

    def target_position_none(self, knot_initial, knot_final, initial_hs=None, error_threshold=1E-6):
        """Trajectory does not reach v_max or a_max. Solve using a library's
        optimization routine. Note that a *guarantee* of time-optimality does
        not apply with this function, but is typically satisfied regardless due
        to the nature of the initial guess.

        Assumed that the spline should have an acceleration profile of:

          h0   h1    h2  h3  h4
             _____
            /     \
           /       \
          /         \
                     \         /
                      \       /
                       \_____/

         t0 t1   t2    t3   t4 t5

        where h's are the timespans of each polynomial, and t's are the knot times.
        Note that while the shape is the same as for target_position_amax, in
        this case, h1 and/or h3 may be zero. If neither h1 or h3 is zero, then
        the target_position_amax is sufficient and faster.

        If the residual error is greater than 'error_threshold' then a
        OptimizationError is raised, containing the error and the final h array.

        See: http://docs.scipy.org/doc/scipy-0.7.x/reference/optimize.html
             for more info on the optimization routines available
             (fmin_powell currently used).
        """
        if knot_final.pos > knot_initial.pos:
            jx = self.j_max
            jn = self.j_min
        elif knot_final.pos < knot_initial.pos:
            jx = -self.j_max
            jn = -self.j_min
        else:
            raise FatalTrajectoryError("No change in position.")

        if initial_hs is None:
            initial_spline = self._initial_estimate(knot_initial, knot_final, jx, jn)
            # If all the assumptions were met, then the initial_spline is correct.
            if knot_initial.vel == 0 and knot_final.vel == 0 and \
                    knot_initial.accel == 0 and knot_final.accel == 0 and \
                    jx == -jn:
                return initial_spline
        else: # use the provided durations
            initial_spline = self._update_estimate(initial_hs, knot_initial, jx, jn)

        def update_and_error(hs):
            """Uses closures for knot_initial, jx, jn."""
            self._soln_spline = self._update_estimate(hs, knot_initial, jx, jn)
            error = self._target_position_error_fnc(self._soln_spline, knot_final)
            return error

        xopt, fopt, direc, iter_, funcalls, warnflag = scipy.optimize.fmin_powell(update_and_error,
                                    initial_spline.h,
                                    maxiter=1E6,
                                    maxfun=1E6,
                                    xtol=1E-6,
                                    ftol=1E-6,
                                    full_output=True) # returns xopt, fopt, direc, iter_, funcalls, warnflag if True else xopt

##        hs = scipy.optimize.fmin_cobyla(func=update_and_error,
##                                        x0=[10,10,10,10,10],
##                                        cons=(self._nonneg_h_constraint_fnc,
##                                              self._amax_constraint_fnc,
##                                              self._amin_constraint_fnc),
##                                        rhobeg = 0.01,
##                                        iprint = 3,
##                                        maxfun = 1E7
##                                        )

        if fopt > error_threshold:
##            from pyprt.shared.cspline_plotter import CSplinePlotter
##            plotter = CSplinePlotter(self._soln_spline, self.v_max, self.a_max, self.j_max, self.v_min, self.a_min, self.j_min)
##            plotter.display_plot()
            raise OptimizationError(fopt, self._soln_spline.h)

        # Remove any (tiny) negative duration polys.
        if -0.0001 <= min(self._soln_spline.h) < 0:
            q, v, a, t = [], [], [], []
            for idx, h in enumerate(self._soln_spline.h):
                if h > 0.00001:
                    q.append(self._soln_spline.q[idx])
                    v.append(self._soln_spline.v[idx])
                    a.append(self._soln_spline.a[idx])
                    t.append(self._soln_spline.t[idx])
            q.append(self._soln_spline.q[-1])
            v.append(self._soln_spline.v[-1])
            a.append(self._soln_spline.a[-1])
            t.append(self._soln_spline.t[-1])

            self._soln_spline = CubicSpline(q, v, a, t)

        return self._soln_spline


    def _initial_estimate(self, knot_initial, knot_final, jx, jn):
        """Make the following simplifying assumptions, regardless of how inaccurate...
         jmax == -jmin
         a_initial == a_final == 0
         v_initial == v_final == 0
         h1 and h3 are 0.
         """

        ### Reasoning:
        # Break the accel profile into 4 segments of equal time. h0, h2/2, h2/2, h4
        # q01 = jx*h**3/6
        # a1 = jx*h
        # v1 = jx*h**2/2
        # q12 = jn*h**3/6 + a1*h**2/2 + v1*h
        # q_total = 2*q01 + 2*q12   due to assumed symmetry
        # q_total = 2*jx*h**3       simplified
        h = math.pow((knot_final.pos - knot_initial.pos)/(2*jx), 1/3)
        return self._update_estimate([h, 0, 2*h, 0, h], knot_initial, jx, jn)


    def _update_estimate(self, hs, knot_initial, jx, jn):
        """Create a new spline using the 3 timespans"""
        k1 = self.create_knot_after(knot_initial, hs[0], jx*hs[0] + knot_initial.accel)
        k2 = self.create_knot_after(k1, hs[1], k1.accel)
        k3 = self.create_knot_after(k2, hs[2], jn*hs[2] + k2.accel)
        k4 = self.create_knot_after(k3, hs[3], k3.accel)
        k5 = self.create_knot_after(k4, hs[4], jx*hs[4] + k4.accel)
        spline = CubicSpline([], [], [], [])
        for k in [knot_initial, k1, k2, k3, k4, k5]:
            spline.append(k)
        return spline

##    def _nonneg_h_constraint_fnc(self, h_array):
##        """Constraint function for cobyla. Must return >= 0 for solution to be accepted."""
##        print "_nonneg_h_constraint_fnc:", min(h_array)
##        return min(h_array)
##
##    def _amax_constraint_fnc(self, h_array):
##        """Constraint function for cobyla. Must return >= 0 for solution to be accepted."""
##        result = min(self.a_max - accel for accel in self._soln_spline.a) # neg if accel > self.a_max
##        print "_amax_constraint_fnc:", result
##        return result
##
##    def _amin_constraint_fnc(self, h_array):
##        """Constraint function for cobyla. Must return >= 0 for solution to be accepted."""
##        return min(accel - self.a_min for accel in self._soln_spline.a) # neg if accel < self.a_min

    def _target_position_error_fnc(self, spline, knot_final):
        neg_h_penalty = sum(abs(h) if h < 0 else 0 for h in spline.h)
        assert neg_h_penalty >= 0

        # enforce a_max and a_min constraints
        a_max_penalty = sum(max(0, accel - self.a_max) for accel in spline.a)
        a_min_penalty = sum(max(0, self.a_min - accel) for accel in spline.a)
        assert a_max_penalty >= 0
        assert a_min_penalty >= 0

        # squared error
        error = (knot_final.pos - spline.q[-1])**2 + \
                (knot_final.vel - spline.v[-1])**2 + \
                (knot_final.accel - spline.a[-1])**2 + \
                neg_h_penalty + a_max_penalty + a_min_penalty

        return error

    @staticmethod
    def nonnegative_roots(A, B, C):
        """Finds the nonnegative roots of a 2nd degree polynomial, using quadratic formula.
        Raises a TrajectoryError if answer would be imaginary."""
        try:
            tmp = (B**2 - 4*A*C)**(0.5)
        except ValueError: # negative number cannot be raised to a fractional power
            # Allow that rounding errors may have pushed a zero to be a slight negative.
            if (B**2 - 4*A*C) > -0.0001:
                tmp = 0
            else:
                raise TrajectoryError('No real roots')

        roots = [(-B + tmp)/(2*A), (-B - tmp)/(2*A)]
        def is_positive(x):
            return x >= 0
        return filter(is_positive, roots)

    @staticmethod
    def nonpositive_roots(A, B, C):
        """Finds the nonpositive roots of a 2nd degree polynomial, using quadratic formula.
        Raises a TrajectoryError if answer would be imaginary."""
        try:
            tmp = (B**2 - 4*A*C)**(0.5)
        except ValueError: # negative number cannot be raised to a fractional power
             # Allow that rounding errors may have pushed a zero to be a slight negative.
            if (B**2 - 4*A*C) > -0.0001:
                tmp = 0
            else:
                raise TrajectoryError('No real roots')

        roots = [(-B + tmp)/(2*A), (-B - tmp)/(2*A)]
        def is_negative(x):
            return x <= 0
        return filter(is_negative, roots)

    def target_velocity(self, initial, final):
        """Similar to target_position, but the final position is ignored in
        addition to the final time. 'inital' and 'final' are Knot instances."""
        final = final.copy() # don't alter the original
        final.time = inf

        if __debug__:
            if not (self.v_min - self.v_threshold <= initial.vel <= self.v_max + self.v_threshold):
                raise FatalTrajectoryError("Endpoint velocity outside of solver's limit")
            if not (self.a_min - self.a_threshold <= initial.accel <= self.a_max + self.a_threshold):
                raise FatalTrajectoryError("Endpoint acceleration outside of solver's limit")

        if final.vel > initial.vel:
            flip = False
            jx = self.j_max
            jn = self.j_min
            ax = self.a_max
            an = self.a_min
        elif initial.vel > final.vel:
            flip = True
            jx = -self.j_max
            jn = -self.j_min
            ax = -self.a_max
            an = -self.a_min
        else:
            raise FatalTrajectoryError("No change in velocity requested.")

        # See prob3 in trajectory_calcs_II.py

        #                                 2      2
        #             2 / 1      1  \   a0     a2
        # v2 - v0 + at *|---- - ----| + ---- - ----
        #               \2*jn   2*jx/   2*jx   2*jn

        _a = 1/(2*jn) - 1/(2*jx)
        _b = 0
        _c = final.vel - initial.vel  + initial.accel**2/(2*jx) - final.accel**2/(2*jn)

        if not flip:
            a_top = max(self.nonnegative_roots(_a, _b, _c))
            a_top = min(ax, a_top)
        else:
            a_top = min(self.nonpositive_roots(_a, _b, _c))
            a_top = max(ax, a_top)

        knots = [initial, None, None, None]

        h0 = (a_top - initial.accel)/jx
        knots[1] = self.create_knot_after(initial, h0, a_top)

        h2 = (final.accel - a_top)/jn
        knots[2] = self.create_knot_before(final, h2, a_top, no_pos=True)

        # skip the h1 segment if the peak acceleration didn't get flattened.
        h1 = 0 if a_top < ax else (knots[2].vel - knots[1].vel)/a_top

        knots[2] = self.create_knot_after(knots[1], h1, a_top) # recreate k2 with the correct position and time
        knots[3] = self.create_knot_after(knots[2], h2, final.accel) # recreate final with the correct position and time


        # Create a spline, while checking that durations are positive
        q, v, a, t = [initial.pos], [initial.vel], [initial.accel], [initial.time]
        for k1, k2 in pairwise(knots):
            if k2.time > k1.time: # Normal case
                q.append(k2.pos)
                v.append(k2.vel)
                a.append(k2.accel)
                t.append(k2.time)
            elif (k2.time - k1.time) >= -0.0001: # Negative or zero duration, but only by rounding errors
                pass # Skip the knot
            else: # large negative duration
                raise TrajectoryError("Large negative duration: %f seconds" % (k2.t - k1.t))
        spline = CubicSpline(q, v, a, t)

        return spline

    def _plot(self, spline, v_max=None, a_max=None, j_max=None, v_min=None, a_min=None, j_min=None, title=""):
        """For debugging"""
        from pyprt.shared.cspline_plotter import CSplinePlotter
        vx = v_max if v_max != None else self.v_max
        ax = a_max if a_max != None else self.a_max
        jx = j_max if j_max != None else self.j_max
        vn = v_min if v_min != None else self.v_min
        an = a_min if a_min != None else self.a_min
        jn = j_min if j_min != None else self.j_min
        plotter = CSplinePlotter(spline, vx, ax, jx, vn, an, jn, title)
        plotter.display_plot()


if __name__ == '__main__':
    initial = Knot(0,0,0,0)
    final = Knot(1000,16,0,None)
    solver = TrajectorySolver(25, 5, 2.5)  # v_max, a_max, j_max

    spline = solver.target_velocity(initial, final)

    from pyprt.shared.cspline_plotter import CSplinePlotter
    plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min)
    plotter.display_plot()

    print spline