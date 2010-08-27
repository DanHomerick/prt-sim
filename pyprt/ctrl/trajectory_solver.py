from __future__ import division # use floating point division unless explicitly otherwise

import random
import math

import scipy.optimize
from scipy import inf

import pyprt.shared.utility as utility
from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import Knot
from pyprt.shared.utility import pairwise, same_sign

class TrajectoryError(Exception):
    """Raised when a generated trajectory is known to be flawed."""
    def __init__(self, initial=None, final=None, msg=""):
        self.initial = initial
        self.final = final
        self.msg = msg

    def __str__(self):
        return self.msg + "\ninitial: %s\nfinal: %s" % (str(self.initial), str(self.final))

class ConstraintsError(TrajectoryError):
    """Acceleration constraints are too low or too high to have a viable solution.
    too_loose: A sequence of spline knot indexes for which the accel constraint is too loose.
    too_tight: A sequence of spline knot indexes for which the accel constraint is too tight.
    """
    HUMP_ONE = 0
    HUMP_TWO = 1

    def __init__(self, too_loose=None, too_tight=None):
        super(ConstraintsError, self).__init__(self)
        self.too_loose = too_loose if too_loose is not None else []
        self.too_tight = too_tight if too_tight is not None else []

    def __str__(self):
        return "too_loose: %s\ntoo_tight: %s" % (str(self.too_loose), str(self.too_tight))

class FatalTrajectoryError(TrajectoryError):
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
    q_threshold = 0.0005
    v_threshold = 0.000005   # == 0.00001 miles per hour
    a_threshold = 0.000005
    t_threshold = 0.0005

    def __init__(self, velocity_max, acceleration_max, jerk_max,
                 velocity_min=None, acceleration_min=None, jerk_min=None):
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

    def target_position(self, knot_initial, knot_final,
                        max_speed=None, fnc_info=False, max_attempts=10):
        """Targets the position and velocity values in knot_final. The time
        supplied by knot_final is ignored, and should be set to None.

        Raises FatalTrajectoryError if it is unable to generate a valid spline.
        Raises OptimizationError if a optimization routine was used and
           the residual error is greater than 'error_threshold'. Exception
           contains the residual error value and the final h array.

        Returns a cubic_spline.CSpline. If fnc_info is True, then returns the
        function used to find the solution in addition to the spline: (spline, fnc)
        """
        knot_final = knot_final.copy() # don't alter the original
        knot_final.time = inf

        if max_speed is None:
            max_speed = self.v_max

        max_speed = min(max_speed, self.v_max)

        # Basic error checking. Bail if the initial or final conditions lie
        # outside of the velocity and/or acceleration constaints.
        if knot_final.pos > knot_initial.pos:
            flip = False
            if __debug__:
                if not (self.v_min - self.v_threshold <= knot_initial.vel <= max_speed + self.v_threshold and
                        self.v_min - self.v_threshold <= knot_final.vel <= max_speed + self.v_threshold and
                        self.a_min - self.a_threshold <= knot_initial.accel <= self.a_max + self.a_threshold and
                        self.a_min - self.a_threshold <= knot_final.accel <= self.a_max + self.a_threshold):
                    raise FatalTrajectoryError(knot_initial, knot_final, "Endpoint outside of solver's limit", )
        elif knot_final.pos < knot_initial.pos: # travelling in reverse
            flip = True
            if __debug__:
                if not (self.v_min - self.v_threshold <= -knot_initial.vel <= max_speed + self.v_threshold and
                        self.v_min - self.v_threshold <= -knot_final.vel <= max_speed + self.v_threshold and
                        self.a_min - self.a_threshold <= -knot_initial.accel <= self.a_max + self.a_threshold and
                        self.a_min - self.a_threshold <= -knot_final.accel <= self.a_max + self.a_threshold):
                    raise FatalTrajectoryError(knot_initial, knot_final, "Endpoint outside of solver's limit")
        else: # knot_final.pos == knot_initial.pos
            # Could return a spline, but in what form? 0 knots, 1 knot, 2 knots?
            raise FatalTrajectoryError(knot_initial, knot_final, "You didn't bloody well want to move then, did ya?")

        # Generate the spline
        try:
            fnc = self.target_position_vmax
            spline = self.target_position_vmax(knot_initial, knot_final, max_speed)
        except FatalTrajectoryError:
            raise
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
                            raise FatalTrajectoryError(knot_initial, knot_final, "Unable to find a viable trajectory.")
                        else:
                            durations = [random.random() for h in err.durations]
                            attempts += 1
                    else:
                        break

        # Additional error checking on the generated trajectory, ensuring that
        # the constraints were actually respected.
        if __debug__:
            if min(spline.h) < 0:
                raise FatalTrajectoryError(knot_initial, knot_final, "Generated a trajectory with a negative duration poly.")

            if not flip:
                if spline.get_max_velocity() > max_speed + self.v_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with max velocity limit.")
                if spline.get_min_velocity() < self.v_min - self.v_threshold:
                    ##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with min velocity limit.")
                if spline.get_max_acceleration() > self.a_max + self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with max acceleration limit.")
                if spline.get_min_acceleration() < self.a_min - self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with min acceleration limit.")

            else:
                if -spline.get_max_velocity() > max_speed + self.v_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with max velocity limit.")
                if -spline.get_min_velocity() < self.v_min - self.v_threshold:
                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with min velocity limit.")
                if -spline.get_max_acceleration() > self.a_max + self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with max acceleration limit.")
                if -spline.get_min_acceleration() < self.a_min - self.a_threshold:
##                    self._plot(spline)
                    raise FatalTrajectoryError(knot_initial, knot_final, "Unable to comply with min acceleration limit.")
        if fnc_info:
            return spline, fnc
        else:
            return spline

    def target_position_vmax(self, initial, final, max_speed=None, flip=None):
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

        if max_speed is None:
            max_speed = self.v_max

        max_speed = min(self.v_max, max_speed)

        qi = initial.pos
        vi = initial.vel
        ai = initial.accel
        ti = initial.time

        qf = final.pos
        vf = final.vel
        af = final.accel

        if flip is None:
            if qf > qi:
                flip = False
            else:
                flip = True

        if not flip: # moving in a forward direction
            jx = self.j_max
            jn = self.j_min
            ax = self.a_max
            an = self.a_min
            vx = max_speed
            vn = self.v_min
        else: # moving in a reverse direction
            jx = self.j_min
            jn = self.j_max
            ax = self.a_min
            an = self.a_max
            vx = self.v_min
            vn = max_speed

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
        # or the 'natural' acceleration maxima, whichever is more constraining.
        try:
            if not flip:
                a_top = max(x for x in utility.quadratic_roots(_a, _b, _c, self.a_threshold/2.0) if x >= 0)
                a_top = min(ax, a_top)
            else:
                a_top = min(x for x in utility.quadratic_roots(_a, _b, _c, self.a_threshold/2.0) if x <= 0)
                a_top = max(ax, a_top)
        except ValueError:
            raise TrajectoryError(initial, final)

        # Similarly for a_bot, the minimum acceleration that is reached. (i.e. a_bottom)
        #       5  6  7
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
        #             2 / 1      1  \   af
        # vf - vx + ab *|---- - ----| + ----
        #               \2*jx   2*jn/   2*jn

        _a = 1/(2*jx) - 1/(2*jn)
        _b = 0
        _c = vf - vx + af**2/(2*jn)

        try:
            if not flip:
                a_bot = min(x for x in utility.quadratic_roots(_a, _b, _c, self.a_threshold/2.0) if x <= 0)
                a_bot = max(an, a_bot)
            else:
                a_bot = max(x for x in utility.quadratic_roots(_a, _b, _c, self.a_threshold/2.0) if x >= 0)
                a_bot = min(an, a_bot)
        except ValueError:
            raise TrajectoryError(initial, final)

        ###  create the first three knots
        h0 = (a_top - ai)/jx
        k1 = self.create_knot_after(initial, h0, a_top)

        h2 = -a_top/jn
        v23 = jn*h2**2/2 + a_top*h2 # delta v from knot 2 -> 3

        v12 = vx - k1.vel - v23 # from v_max - v1 = v12 + v23
        if not flip:
            h1 = 0 if (a_top < ax) else v12/a_top
        else:
            h1 = 0 if (ax < a_top) else v12/a_top

        k2 = self.create_knot_after(k1, h1, a_top)
        k3 = self.create_knot_after(k2, h2, 0)

        ###  create the last three knots
        h6 = (af - a_bot)/jx
        k6 = self.create_knot_before(final, h6, a_bot)

        h4 = a_bot/jn
        v45 = jn*h4**2/2 # delta v from knot 4 -> 5. Similar to v23, but note that a4 == 0.

        v56 = k6.vel - vx - v45 # from v6 - v_4 = v45 + v56. Note that v_4 == v_max
        if not flip:
            h5 = 0 if (a_bot > an) else v56/a_bot
        else:
            h5 = 0 if (an > a_bot) else v56/a_bot

        k5 = self.create_knot_before(k6, h5, a_bot)
        k4 = self.create_knot_before(k5, h4, 0)

        ###  calculate the time to cruise at a constant v_max
        try:
            h3 = (k4.pos - k3.pos)/vx
        except ZeroDivisionError:
            if k4.pos == k3.pos:
                h3 = 0
            else:
                raise FatalTrajectoryError(initial, final, 'Vmax is zero, but some distance must be traveled.')

        if h3 < 0:
            raise TrajectoryError(initial, final, 'Vmax is not reached')

        # the times for k4 through k6 are all inf, so fix them now
        k4.time = k3.time + h3
        k5.time = k4.time + h4
        k6.time = k5.time + h5
        final.time = k6.time + h6

        # Create a spline, while checking that durations are positive
        q, v, a, j, t = [qi], [vi], [ai], [], [ti]
        for k1, k2 in pairwise([initial, k1, k2, k3, k4, k5, k6, final]):
            if k2.time > k1.time: # Normal case
                q.append(k2.pos)
                v.append(k2.vel)
                a.append(k2.accel)
                t.append(k2.time)
                if k2.accel > k1.accel:
                    j.append(self.j_max)
                elif k2.accel < k1.accel:
                    j.append(self.j_min)
                else:
                    j.append(0)
            elif (k2.time - k1.time) >= -self.t_threshold: # Negative or zero duration, but only by rounding errors
                pass # Skip the knot
            else: # large negative duration
                raise TrajectoryError(initial, final, "Large negative duration: %f seconds" \
                                      % (k2.time - k1.time))
        spline = CubicSpline(q, v, a, j, t)
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
            an = self.a_max
            ax = self.a_min
            jn = self.j_max
            jx = self.j_min

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
            h3 = min(x for x in utility.quadratic_roots(A, B, C, self.t_threshold/2.0) if x >= 0)
        except ValueError: # No nonnegative, real solutions. a_min is not reached.
            raise TrajectoryError(initial, final, 'No nonnegative, real solutions')

        h1 = alpha - an/ax*h3

        k2 = self.create_knot_after(k1, h1, ax)
        k3 = self.create_knot_after(k2, h2, an)
        k4 = self.create_knot_after(k3, h3, an)
        final.time = k4.time + h4

        # Create a spline, while checking that durations are positive
        j_template = [jx, 0, jn, 0, jx]
        knots = (initial, k1, k2, k3, k4, final)
        q, v, a, j, t = [q0], [v0], [a0], [], [initial.time]
        for i in range(len(knots)-1):
            k2 = knots[i+1]
            k1 = knots[i]
            if k2.time > k1.time: # Normal case
                q.append(k2.pos)
                v.append(k2.vel)
                a.append(k2.accel)
                j.append(j_template[i])
                t.append(k2.time)
            elif (k2.time - k1.time) >= -0.0001: # Negative or zero duration, but only by rounding errors
                pass # Skip the knot
            else: # large negative duration
                raise TrajectoryError(initial, final, "Large negative duration: %f seconds" % (k2.time - k1.time))
        spline = CubicSpline(q, v, a, j, t)

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
            raise FatalTrajectoryError(knot_initial, knot_final, "No change in position.")

        if initial_hs is None:
            initial_spline = self._initial_estimate(knot_initial, knot_final, jx, jn)
            # If all the assumptions were met, then the initial_spline is correct.
            if knot_initial.vel == 0 and knot_final.vel == 0 and \
               knot_initial.accel == 0 and knot_final.accel == 0 and \
               jx == -jn:
                return initial_spline
        else: # use the provided durations
            initial_spline = self._update_estimate(initial_hs, knot_initial, jx, jn)

        # Define update_and_error within this function to take advantage of closures.
        # TODO: Consider using some private class variables instead. Probably faster & clearer?
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

    def target_time(self, knot_initial, knot_final, max_speed=None):
        """knot_initial: Starting pose
        knot_final: Desired final pose
        max_speed: Overrides the objects default v_max.
        """

        if max_speed is None:
            max_speed = self.v_max
        else:
            max_speed = min(self.v_max, max_speed)

        no_time_constraint_spline = self.target_position(knot_initial, knot_final, max_speed)

        # Rare case, but use the no_time_constraint_spline if the final time happens to be (almost) perfect.
        if abs(no_time_constraint_spline.t[-1] - knot_final.time) < self.t_threshold:
            return no_time_constraint_spline

        # The final pose cannot be reached in the allotted time, even when
        # cruising at the max_speed.
        elif no_time_constraint_spline.t[-1] > knot_final.time:
            raise FatalTrajectoryError(knot_initial, knot_final, "Target cannot be reached under current constraints.")


        q = [knot_initial.pos,         None,       None,   None,   None,       None,       None, knot_final.pos]
        v = [knot_initial.vel,         None,       None,   None,   None,       None,       None, knot_final.vel]
        t = [knot_initial.time,        None,       None,   None,   None,       None,       None, knot_final.time]
        spline = None

        # Check what would happen if we don't alter (or only minimally alter)
        # the vehicle's initial pose. From this, we can make assumptions about
        # what acceleration profile we should use.
        # TODO: Incorperate a dead-zone around the time?
        no_accel_spline = self._no_acceleration_predict(knot_initial, knot_final)

        # Rare case, but use the no_accel_spline if the final time happens to be (almost) perfect
        if abs(no_accel_spline.t[-1] - knot_final.time) < self.t_threshold:
            return no_accel_spline

        # Using the average speed as an approximation for the 3-4 segment velocity.
        # This is NOT precise, but is right most of the time. Use a try/except
        # to catch the cases in which the approximation was poor.
        ave_speed = (knot_final.pos - knot_initial.pos)/(knot_final.time - knot_initial.time)
        if no_accel_spline.t[-1] > knot_final.time: # Need to speed up to arrive on time
            try:
                if knot_final.vel <= ave_speed:
                    #             ____
                    #            /    \
                    #           /      \
                    #          /        \______
                    #                          \          /
                    #                           \        /
                    #                            \______/
                    #
                    #         t0 t1  t2 t3    t4 t5    t6 t7
                    j = [self.j_max, 0, self.j_min, 0, self.j_min, 0, self.j_max]
                    a = [knot_initial.accel, self.a_max, self.a_max, 0, 0, self.a_min, self.a_min, knot_final.accel]
                    spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)
                else:
                    #            ____            ______
                    #           /    \          /      \
                    #          /      \        /        \
                    #         /        \______/          \
                    #
                    #        t0 t1  t2 t3    t4 t5    t6 t7
                    j = [self.j_max, 0, self.j_min, 0, self.j_max, 0, self.j_min]
                    a = [knot_initial.accel, self.a_max, self.a_max, 0, 0, self.a_max, self.a_max, knot_final.accel]
                    spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)

            # Average velocity wasn't a good estimate of 3-4 segment's velocity.
            # Reverse the decision to slow|speed at the end.
            except TrajectoryError:
                try:
                    if knot_final.vel <= ave_speed:
                        j = [self.j_max, 0, self.j_min, 0, self.j_max, 0, self.j_min]
                        a = [knot_initial.accel, self.a_max, self.a_max, 0, 0, self.a_max, self.a_max, knot_final.accel]
                        spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)
                    else:
                        j = [self.j_max, 0, self.j_min, 0, self.j_min, 0, self.j_max]
                        a = [knot_initial.accel, self.a_max, self.a_max, 0, 0, self.a_min, self.a_min, knot_final.accel]
                        spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)
                except TrajectoryError:
                    raise FatalTrajectoryError(knot_initial, knot_final)

        elif no_accel_spline.t[-1] < knot_final.time: # Need to slow down to arrive on time
            try:
                if knot_final.vel >= ave_speed:
                    #                              _____
                    #                             /     \
                    #                            /       \
                    #                    _______/         \
                    #        \          /
                    #         \        /
                    #          \______/
                    #
                    #       t0 t1    t2 t3     t4 t5   t6 t7
                    j = [self.j_min, 0, self.j_max, 0, self.j_max, 0, self.j_min]
                    a = [knot_initial.accel, self.a_min, self.a_min, 0, 0, self.a_max, self.a_max, knot_final.accel]
                    spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)
                else:
                    #                    _______
                    #        \          /       \         /
                    #         \        /         \       /
                    #          \______/           \_____/
                    #
                    #       t0 t1    t2 t3     t4 t5   t6 t7
                    j = [self.j_min, 0, self.j_max, 0, self.j_min, 0, self.j_max]
                    a = [knot_initial.accel, self.a_min, self.a_min, 0, 0, self.a_min, self.a_min, knot_final.accel]
                    spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)

            # Average velocity wasn't a good estimate of 3-4 segment's velocity.
            # Reverse the decision to slow|speed at the end.
            except TrajectoryError:
                try:
                    if knot_final.vel >= ave_speed:
                        j = [self.j_min, 0, self.j_max, 0, self.j_min, 0, self.j_max]
                        a = [knot_initial.accel, self.a_min, self.a_min, 0, 0, self.a_min, self.a_min, knot_final.accel]
                        spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)
                    else:
                        j = [self.j_min, 0, self.j_max, 0, self.j_max, 0, self.j_min]
                        a = [knot_initial.accel, self.a_min, self.a_min, 0, 0, self.a_max, self.a_max, knot_final.accel]
                        spline = self._impose_limits_target_amax(q, v, a, j, t, ave_speed, max_speed)
                except TrajectoryError:
                    raise FatalTrajectoryError(knot_initial, knot_final)

        else: # no_accel_knot.time == knot_final.time
            # Rare case, but this leaves us already having a spline that does exactly what we want.
            return no_accel_spline

        assert spline is not None

        if spline.get_max_velocity() > max_speed + 0.0001:
            raise FatalTrajectoryError(knot_initial, knot_final)

        return spline

    def _target_time_no_limits(self, q, v, a, t, flip=False):
        """
        Assumes that the vehicle makes a small change in velocity, such that
        the acceleration limits are not hit.

        If flip is False, the spline has an acceleration profile of:


            /\
           /  \
          /    \______
                      \    /
                       \  /
                        \/

         t0 t1 t2    t3 t4 t5

        If flip is True, the spline has an acceleration profile of:

                        /\
                       /  \
               _______/    \
         \    /
          \  /
           \/

        t0 t1 t2     t3 t4 t5
        """
        raise NotImplementedError


    def _impose_limits_target_amax(self, q, v, a, j, t, ave_vel, max_speed, trials=30):
        """Performs a search over the acceleration limits, stopping
        when it finds viable limits that will be reached. In this way, we
        may satisfy the preconditions for the self._target_time_amax function.
        Warning: inputs are modified.
        """
        original_a = a[:]
        ceilings = [a[1], a[-2]]
        floors = [a[0], a[-1]]
        deltas = [(floors[0] - ceilings[0])/2.0,
                  (floors[1] - ceilings[1])/2.0]

        switchover = trials // 2
        use_alt_soln = False
        spline = None
        while trials:
            if trials == switchover:
                a = original_a[:]
                deltas = [(floors[0] - ceilings[0])/2.0,
                          (floors[1] - ceilings[1])/2.0]
                use_alt_soln = True

            try:
                spline = self._target_time_amax(q,v,a,j,t,ave_vel,max_speed,use_alt_soln)
                break
            except ConstraintsError as err:
                trials -= 1
                for hump in err.too_loose: # tighten the limit
                    if not same_sign(deltas[hump], floors[hump] - ceilings[hump]):
                        # Last time we loosened. Switch direction and cut the magnitude in half.
                        deltas[hump] = -deltas[hump]
                        deltas[hump] /= 2

                    if hump == ConstraintsError.HUMP_ONE:
                        if deltas[hump] < 0:
                            a[1] = a[2] = max(a[1] + deltas[hump], floors[hump]) # don't go below floors
                        else: # deltas[hump] > 0
                            a[1] = a[2] = min(a[1] + deltas[hump], floors[hump]) # don't go above floors
                    elif hump == ConstraintsError.HUMP_TWO:
                        if deltas[hump] < 0:
                            a[5] = a[6] = max(a[5] + deltas[hump], floors[hump])
                        else: # deltas[hump] > 0
                            a[5] = a[6] = min(a[5] + deltas[hump], floors[hump])
                    else:
                        raise Exception("ConstraintsError contains invalid value.")

                for hump in err.too_tight: # loosen the limit
                    if not same_sign(deltas[hump], ceilings[hump] - floors[hump]):
                        # Last time we tightened. Switch direction and cut the magnitude in half.
                        deltas[hump] = -deltas[hump]
                        deltas[hump] /= 2

                    if hump == ConstraintsError.HUMP_ONE:
                        if deltas[hump] > 0:
                            a[1] = a[2] = min(a[1] + deltas[hump], ceilings[hump]) # don't go over ceiling
                        else: # deltas[hump] < 0
                            a[1] = a[2] = max(a[1] + deltas[hump], ceilings[hump]) # don't go over ceiling
                    elif hump == ConstraintsError.HUMP_TWO:
                        if deltas[hump] > 0:
                            a[5] = a[6] = min(a[5] + deltas[hump], ceilings[hump])
                        else: # deltas[hump] > 0
                            a[5] = a[6] = max(a[5] + deltas[hump], ceilings[hump])
                    else:
                        raise Exception("ConstraintsError contains invalid value.")

        if spline is None:
            raise TrajectoryError()
        else:
            return spline


    def _target_time_amax(self, q, v, a, j, t, ave_vel, max_speed, use_alt_soln):
        """
        Assumes that the vehicle makes a large enough velocity change that both
        acceleration limits are hit.
        """
        HUMP_ONE = ConstraintsError.HUMP_ONE
        HUMP_TWO = ConstraintsError.HUMP_TWO

        j_sq = [x*x for x in j]
        a_sq = [x*x for x in a]

        # See trajectory_notes.txt for the Mathematica work used to generate
        # the following equations.
        if a[1] != a[5]:
            numerator_a = 3*(a[0]-a[1])**2*a[5]*j[0]*j_sq[2]*j_sq[4]*j_sq[6]+3*j_sq[0]*j_sq[2]*j_sq[4]*j[6]*(-2*a[5]*j[6]*v[0]+a[1]*(a_sq[6]-a_sq[7]+2*a[5]*(-a[6]+a[7]+j[6]*t[0]-j[6]*t[7])+2*j[6]*v[7]))
            try:
                numerator_b = math.sqrt(3)*math.sqrt(j_sq[0]*j_sq[2]*j_sq[4]*j_sq[6]*(3*j_sq[2]*j_sq[4]*(a_sq[1]*a[5]*j[6]+a[5]*j[6]*(a_sq[0]-2*j[0]*v[0])+a[1]*(-2*a[5]*(a[0]*j[6]+j[0]*(a[6]-a[7]-j[6]*t[0]+j[6]*t[7]))+j[0]*(a_sq[6]-a_sq[7]+2*j[6]*v[7])))**2-(a[1]-a[5])*(a_sq[1]*a_sq[1]*a[5]*j_sq[2]*j_sq[4]*j_sq[6]-6*a_sq[1]*a[5]*j_sq[2]*j_sq[4]*j_sq[6]*(a_sq[0]-2*j[0]*v[0])+3*a[5]*j_sq[4]*j_sq[6]*(a_sq[2]*a_sq[2]*j_sq[0]-j_sq[2]*(a_sq[0]-2*j[0]*v[0])**2)+a[1]*(-6*a_sq[4]*a_sq[5]*j_sq[0]*j_sq[2]*j_sq[6]+a_sq[5]*a_sq[5]*j_sq[0]*j_sq[2]*j_sq[6]-4*a[5]*(a_sq[6]*a[6]*j_sq[0]*j_sq[2]*j_sq[4]+2*a_sq[7]*a[7]*j_sq[0]*j_sq[2]*j_sq[4]+j_sq[6]*(-2*a_sq[4]*a[4]*j_sq[0]*j_sq[2]+j_sq[4]*(a_sq[2]*a[2]*j_sq[0]-2*j_sq[2]*(a_sq[0]*a[0]+3*j_sq[0]*(q[0]-q[7])-3*a[0]*j[0]*v[0])))-6*a[7]*j_sq[0]*j_sq[2]*j_sq[4]*j[6]*v[7]-3*a[6]*j_sq[0]*j_sq[2]*j_sq[4]*(a_sq[7]-2*j[6]*v[7]))+3*j_sq[0]*j_sq[2]*(-a_sq[4]*j[6]+j[4]*(a_sq[6]-a_sq[7]+2*j[6]*v[7]))*(a_sq[4]*j[6]+j[4]*(a_sq[6]-a_sq[7]+2*j[6]*v[7]))))))
            except ValueError:
                raise ConstraintsError(too_tight=(HUMP_ONE, HUMP_TWO)) # FIXME: Choose correct places to loosen?
            denominator = (6*(a[1]-a[5])*j_sq[0]*j_sq[2]*j_sq[4]*j_sq[6])
            potential_solutions = [(numerator_a + numerator_b) / denominator,
                                   (numerator_a - numerator_b) / denominator]
            if abs(potential_solutions[0] - ave_vel) < abs(potential_solutions[1] - ave_vel):
                if not use_alt_soln:
                    cruise_vel = potential_solutions[0]
                else:
                    cruise_vel = potential_solutions[1]
            else:
                if not use_alt_soln:
                    cruise_vel = potential_solutions[1]
                else:
                    cruise_vel = potential_solutions[0]

        # Must use an alternative equation, since the denominator is zero in
        # this case.
        elif a[1] == a[5] and use_alt_soln is False:
            assert a[1] == a[2] == a[5] == a[6]
##            assert j[0] == j[4]
##            assert j[2] == j[6]
            try:
                cruise_vel = (-3*a_sq[0]*a_sq[0]*j_sq[6]+8*a_sq[0]*a[0]*a[1]*j_sq[6]+2*a_sq[1]*a_sq[1]*(j[6]-j[4])*(j[6]+j[4])-24*a[0]*a[1]*j_sq[6]*j[4]*v[0]-6*a_sq[0]*j_sq[6]*(a_sq[1]-2*j[4]*v[0])-8*a[1]*j_sq[4]*(a[7]*a_sq[7]+3*j_sq[6]*(-q[0]+q[7])-3*a[7]*j[6]*v[7])+6*a_sq[1]*j[4]*(a_sq[7]*j[4]+2*j_sq[6]*v[0]-2*j[6]*j[4]*v[7])+3*j_sq[4]*(a_sq[7]+2*j[6]*v[0]-2*j[6]*v[7])*(a_sq[7]-2*j[6]*(v[0]+v[7])))/(12*j[6]*j[4]*(a_sq[0]*j[6]-2*a[0]*a[1]*j[6]+a_sq[1]*(j[6]-j[4])+2*a[1]*j[4]*(a[7]+j[6]*t[0]-j[6]*t[7])-j[4]*(a_sq[7]+2*j[6]*v[0]-2*j[6]*v[7])))
                if math.isnan(cruise_vel):
                    cruise_vel = v[0]
            except ZeroDivisionError:
                assert a[1] == a[2] == a[5] == a[6] == 0
                cruise_vel = v[0]

        else: # use_alt_soln is True
            assert a[1] == a[2] == a[5] == a[6]
##            assert j[0] == j[4]
##            assert j[2] == j[6]
            raise FatalTrajectoryError()

        # choose the minimum, positive velocity option where the solution doesn't involve t12 or t56 being negative
        if cruise_vel > max_speed:
            raise ConstraintsError(too_tight=(HUMP_ONE, HUMP_TWO))

        if cruise_vel <= 0: # TODO: Can't work in a negative direction?
            raise ConstraintsError(too_tight=(HUMP_ONE, HUMP_TWO))

        # Decide if either of the two "humps" in the accel profile will not
        # hit their limits. Tighten the limits if this is the case.
        too_loose = []
        too_tight = []
        first_hump = self.target_velocity(Knot(0,v[0],a[0],0), Knot(None,cruise_vel,0,None))
        if len(first_hump.q) > 1: # possible that no "first hump" is required, thus this may get bypassed
            if a[0] == a[1]:
                pass # Tightened all the way to the initial accel. Don't try to tighten past this.
            elif j[0] > 0:
                if a[1] > first_hump.a[1]: # Force a[1] to be flattened
                    too_loose.append(HUMP_ONE)
            else: # j[0] < 0
                if a[1] < first_hump.a[1]:
                    too_loose.append(HUMP_ONE)

        second_hump = self.target_velocity(Knot(0,cruise_vel,0,0), Knot(None, v[-1],0,None))
        if len(second_hump.q) > 1: # possible that no "second hump" is required
            if a[6] == a[7]:
                pass
            elif j[4] > 0:
                if a[5] > second_hump.a[1]:
                    too_loose.append(HUMP_TWO)
            else: # j[4] < 0
                if a[5] < second_hump.a[1]:
                    too_loose.append(HUMP_TWO)

        if too_loose or too_tight:
            raise ConstraintsError(too_loose=too_loose, too_tight=too_tight)

        v[3] = cruise_vel
        v[4] = cruise_vel

        # Work forward for the first accel|decel phase (polys 01, 12, 23)
        t01 = (a[1] - a[0])/j[0]
        t01_2 = t01*t01
        t[1] = t[0] + t01
        v[1] = j[0]*t01_2/2 + a[0]*t01 + v[0]
        q[1] = j[0]*t01*t01_2/6 + a[0]*t01_2/2 + v[0]*t01 + q[0]

        t23 = -a[2]/j[2]
        t23_2 = t23*t23
        v[2] = v[3] - (j[2]*t23_2/2 + a[2]*t23) # v3 - v23

        t12 = (v[2] - v[1])/a[1] if a[1] != 0 else 0
        t12_2 = t12*t12
        t[2] = t[1] + t12
        q[2] = q[1] + a[1]*t12_2/2 + v[1]*t12 # q1 + q12

        t[3] = t[2] + t23
        q[3] = q[2] + j[2]*t23*t23_2/6 + a[2]*t23_2/2 + v[2]*t23 # q2 + q23

        # Work backward for the last accel|decel phase (polys 45, 56, 67)
        t67 = (a[7] - a[6])/j[6]
        t67_2 = t67*t67
        t[6] = t[-1] - t67
        v[6] = v[-1] - (j[6]*t67_2/2 + a[6]*t67) # v7 - v67
        q[6] = q[-1] - (j[6]*t67*t67_2/6 + a[6]*t67_2/2 + v[6]*t67) # q7 - q67

        t45 = (a[5] - a[4])/j[4]
        t45_2 = t45*t45
        v[5] = j[4]*t45_2/2 + a[4]*t45 + v[4] # v4 + v45

        t56 = (v[6] - v[5])/a[5] if a[5] != 0 else 0
        t56_2 = t56*t56
        t[5] = t[6] - t56
        q[5] = q[6] - (a[5]*t56_2/2 + v[5]*t56) # q6 - q56

        t[4] = t[5] - t45
        q[4] = q[5] - (j[4]*t45*t45_2/6 + a[4]*t45_2/2 + v[4]*t45) # q5 - q45

        t34 = t[4] - t[3]

        # Need to loosen a constraint in order to have enough control authority
        # to hit target time. Checks that:
        #  1. No negative durations
        #  2. Distance traveled on const vel segment is reasonable
        #  3. cruise segment should have a constant velocity
        if [True for ti, tf in pairwise(t) if (tf-ti) < -TrajectorySolver.t_threshold] \
           or abs((q[4] - q[3]) - (v[3]*t34)) > TrajectorySolver.q_threshold \
           or abs(v[4] - v[3]) > TrajectorySolver.v_threshold:
            if len(first_hump.q) <= 1:
                too_tight.append(HUMP_ONE)
            elif a[0] == a[1]:
                too_tight.append(HUMP_ONE)
            elif j[0] > 0:
                if a[1] < first_hump.a[1]:
                    too_tight.append(HUMP_ONE)
            else: # j[0] < 0
                if a[1] > first_hump.a[1]:
                    too_tight.append(HUMP_ONE)

            if len(second_hump.q) <= 1:
                too_tight.append(HUMP_TWO)
            elif a[6] == a[7]:
                too_tight.append(HUMP_TWO)
            elif j[4] > 0:
                if a[5] < second_hump.a[1]:
                    too_tight.append(HUMP_TWO)
            else: # j[4] < 0
                if a[5] > second_hump.a[1]:
                    too_tight.append(HUMP_TWO)

            raise ConstraintsError(too_tight=too_tight)

        return CubicSpline(q, v, a, j, t)

    def _no_acceleration_predict(self, initial, final):
        """Creates a spline that attempts to find a "middle ground" of acceleration.
        If the target needs to be reached sooner, a profile that begins with
        an acceleration should be use. If the target needs to be reached later,
        a profile that begins with a deceleration should be used.
        initial: Knot giving initial conditions. Must be complete.
        final: Knot giving final conditions. May exclude time.
        """
        if initial.accel != 0:
            if initial.accel > 0:
                j = self.j_min
            else: # initial.accel < 0
                j = self.j_max
            t01 = -initial.accel/j
            t01_2 = t01*t01
            t01_3 = t01_2*t01
            v1 = j*t01_2/2 + initial.accel*t01 + initial.vel
            q1 = j*t01_3/6 + initial.accel*t01_2/2 + initial.vel*t01 + initial.pos
            t1 = initial.time + t01
            start_spline = CubicSpline([initial.pos, q1],
                                       [initial.vel, v1],
                                       [initial.accel, 0],
                                       [j],
                                       [initial.time, t1])
            start_knot = Knot(q1, v1, 0, t1)

        else: # initial.accel == 0
            start_spline = CubicSpline([initial.pos], [initial.vel], [initial.accel], [], [initial.time])
            start_knot = initial

        # handle special case
        if initial.vel == 0 and initial.accel == 0:
            spline = CubicSpline([initial.pos, initial.pos],
                                 [0, 0], [0, 0], [0], [initial.time, inf])

        else:
            # Change velocity to hit final at the last possible moment.
            # Until then, maintain the initial velocity.
            end_spline = self.target_velocity(start_knot, final)
            delta_q = final.pos - end_spline.q[-1]
            delta_t = delta_q/start_knot.vel
            spline = CubicSpline(start_spline.q + [q + delta_q for q in end_spline.q],
                                 start_spline.v + end_spline.v,
                                 start_spline.a + end_spline.a,
                                 start_spline.j + [0] + end_spline.j,
                                 start_spline.t + [t + delta_t for t in end_spline.t])

        return spline

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
        """Create a new spline using the 5 timespans"""
        spline = CubicSpline([knot_initial.pos], [knot_initial.vel], [knot_initial.accel], [], [knot_initial.time])
        prev_knot = knot_initial
        for h, j in zip(hs, (jx, 0, jn, 0, jx)):
            h = max(0, h) # don't allow h to go negative
            knot = self.create_knot_after(prev_knot, h, j*h + prev_knot.accel)
            spline.append(knot, j)
            prev_knot = knot

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

    def target_velocity(self, initial, final):
        """Similar to target_position, but the final position is ignored in
        addition to the final time. 'inital' and 'final' are Knot instances.
        """
        final = final.copy() # don't alter the original
        final.time = inf

        delta_v = final.vel - initial.vel

        if abs(delta_v) < self.v_threshold:
            if initial.accel == final.accel:
                return CubicSpline([initial.pos], [initial.vel], [initial.accel], [], [initial.time])
            elif initial.accel <= 0: # initial decel will push vel below target.
                flip = False
            else: # initial accel will push vel above target.
                flip = True

        elif delta_v > 0: # increasing velocity
            if initial.accel <= 0: # initial accel is zero or neg, so clearly need to increase accel
                flip = False
            else: # whether to start with increasing accel or decreasing accel depends on magnitude of delta_v and initial accel
                t01 = -initial.accel/self.j_min
                v1 = self.j_min*t01*t01/2 + initial.accel*t01 + initial.vel
                if v1 <= final.vel:
                    flip = False
                else:
                    flip = True
        else: # delta_v < 0: # decreasing velocity
            if initial.accel >= 0: # initial accel is zero or positive, so clearly need to decrease accel
                flip = True
            else: # whether to start with increasing accel or decreasing accel depends on magnitude of delta_v and initial accel
                t01 = -initial.accel/self.j_max
                v1 = self.j_max*t01*t01/2 + initial.accel*t01 + initial.vel
                if v1 <= final.vel:
                    flip = False
                else:
                    flip = True

        if not flip:
            jx = self.j_max
            jn = self.j_min
            ax = self.a_max
            an = self.a_min
        else:
            jx = self.j_min
            jn = self.j_max
            ax = self.a_min
            an = self.a_max

        # See prob3 in trajectory_calcs_II.py

        #                                 2      2
        #             2 / 1      1  \   a0     a2
        # v2 - v0 + at *|---- - ----| + ---- - ----
        #               \2*jn   2*jx/   2*jx   2*jn

        _a = 1/(2*jn) - 1/(2*jx)
        _b = 0
        _c = final.vel - initial.vel  + initial.accel**2/(2*jx) - final.accel**2/(2*jn)

        try:
            if not flip:
                a_top = max(x for x in utility.quadratic_roots(_a, _b, _c, self.a_threshold/2.0) if x >= 0)
                a_top = min(ax, a_top)
            else:
                a_top = min(x for x in utility.quadratic_roots(_a, _b, _c, self.a_threshold/2.0) if x <= 0)
                a_top = max(ax, a_top)
        except ValueError:
            raise TrajectoryError(initial, final)

        knots = [initial, None, None, None]

        h0 = (a_top - initial.accel)/jx
        knots[1] = self.create_knot_after(initial, h0, a_top)

        h2 = (final.accel - a_top)/jn
        knots[2] = self.create_knot_before(final, h2, a_top, no_pos=True)

        try:
            h1 = (knots[2].vel - knots[1].vel)/a_top
        except ZeroDivisionError:
            h1 = 0

        knots[2] = self.create_knot_after(knots[1], h1, a_top) # recreate k2 with the correct position and time
        knots[3] = self.create_knot_after(knots[2], h2, final.accel) # recreate final with the correct position and time


        # Create a spline, while checking that durations are positive
        j_template = [jx, 0, jn]
        q, v, a, j, t = [initial.pos], [initial.vel], [initial.accel], [], [initial.time]
        for i in range(len(knots)-1):
            k1 = knots[i]
            k2 = knots[i+1]
            if abs(k2.time - k1.time) < self.t_threshold/1000: # Within rounding errors of zero length
                pass # Skip the knot
            elif k2.time > k1.time: # Normal case
                q.append(k2.pos)
                v.append(k2.vel)
                a.append(k2.accel)
                j.append(j_template[i])
                t.append(k2.time)
            else: # large negative duration
                raise TrajectoryError(initial, final, "Large negative duration: %f seconds" % (k2.t - k1.t))
        spline = CubicSpline(q, v, a, j, t)

        # Check that the velocities stayed within the allowed range.
        extrema_velocities, extrema_times = spline.get_extrema_velocities()
        max_vel = max(extrema_velocities)
        min_vel = min(extrema_velocities)
        if max_vel > self.v_max + self.v_threshold:
            raise FatalTrajectoryError(initial, final, "Maximum velocity: %.4f exceeded the allowed value: %.4f"
                                       % (max_vel, self.v_max))
        if min_vel < self.v_min - self.v_threshold:
            raise FatalTrajectoryError(initial, final, "Minimum velocity: %.4f exceeded the allowed value: %.4f"
                                       % (min_vel, self.v_min))
        return spline


    def slip(self, spline, ti, dist):
        """DEPRECATED. See target_time instead.
        Starting with an existing spline which is at constant velocity at
        time t, returns a copy of the spline that is changed so that the vehicle
        slips forward or back dist meters relative to the position that would
        have been achieved if the spline were left unchanged.

        The amount of track that a vehicle requires in order to slip forward or
        back x meters is not a simple function. It depends on the line speed and
        the vehicle's initial velocity, acceleration, and jerk, and on the
        vehicle's constraints. It is a piecewise function.

        There is a constant region where x is a large negative. This corresponds
        to the vehicle coming to a complete stop. Once stopped, the vehicle
        can slip back by any additional distance without "using up" any more track.

        If x is smaller magnitude, yet still negative, then the vehicle does not
        come to a complete stop during the slip maneuver. In this interval of the
        function, the vehicle may require more track than in the "complete stop"
        region, even though less distance is being slipped. The function rapidly
        approaches zero as x goes to zero. The function's max in this region is
        not at either endpoint.

        If x is a small, positive value, the function rapidly approaches zero
        as x goes to zero. This interval monotonically increases, thus the
        largest value for this interval is at the transition to the next region.

        When x is a large positive value, the function enters a linear interval
        with a positive slope of v_max/line_speed.

        The function is continuous over all intervals, but the derivative is
        disjoint at every interval boundary.
        """
        assert isinstance(spline, CubicSpline)

        # Reuse existing code to find the durations for the slip maneuver
        # Treat vi as a 'virtual ground' -- that is, treat it as though it were zero.
        initial_knot = spline.evaluate(ti)
        final_knot = spline.evaluate(spline.t[-1])
        assert abs(initial_knot.accel) < self.a_threshold

        slip_solver = TrajectorySolver(self.v_max-initial_knot.vel, self.a_max, self.j_max,
                                       self.v_min-initial_knot.vel, self.a_min, self.j_min)
        slip_spline = slip_solver.target_position(Knot(0, 0, 0, ti),
                                                  Knot(dist, 0, 0, None))

        result = spline.copy_left(ti)

        prev_knot = result.evaluate(ti)
        for h, a, j in zip(slip_spline.h, slip_spline.a[1:], slip_spline.j):
            knot = self.create_knot_after(prev_knot, h, a)
            result.append(knot, j)
            prev_knot = knot

        tail_solver = TrajectorySolver(spline.get_max_velocity(), self.a_max, self.j_max,
                                       0, self.a_min, self.j_min)
        tail_spline = tail_solver.target_position(knot,
                                                  final_knot)
        for t, j in zip(tail_spline.t, tail_spline.j):
            result.append(tail_spline.evaluate(t), j)

        return result

    def _plot(self, spline, v_max=None, a_max=None, j_max=None, v_min=None, a_min=None, j_min=None, title="", start_idx=0, end_idx=-1):
        """For debugging"""
        from pyprt.shared.cspline_plotter import CSplinePlotter
        vx = v_max if v_max != None else self.v_max
        ax = a_max if a_max != None else self.a_max
        jx = j_max if j_max != None else self.j_max
        vn = v_min if v_min != None else self.v_min
        an = a_min if a_min != None else self.a_min
        jn = j_min if j_min != None else self.j_min
        plotter = CSplinePlotter(spline, vx, ax, jx, vn, an, jn, title, start_idx, end_idx)
        plotter.display_plot()

##    def explore(self):
##        from numpy import linspace
##        points = linspace(-5, 5, 1000)
##        out = open('explore.csv', 'w')
##        out.write("%f,%f,%f,%f,%f,%f\n" % (self.v_max, self.a_max, self.j_max, self.v_min, self.a_min, self.j_min))
##        for point in points:
##            spline = self.slip(0, 10, 0, point)
##            out.write("%f,%f\n" % (point, spline.q[-1]))
##        out.close()

if __name__ == '__main__':
    # Limited program for targeting velocities.
    # TODO: Make a little Traits.UI based program for playing with splines without writing any code?
    import sys
    if len(sys.argv) != 9:
        print "Usage: python %s vel_init vel_final accel_init accel_final max_vel max_accel max_decel max_jerk" % sys.argv[0]
        print "\n  max_decel should be provided as a negative number."
        sys.exit()

    prog_name, vel_init, vel_final, accel_init, accel_final, max_vel, max_accel, max_decel, max_jerk = sys.argv
    solver = TrajectorySolver(float(max_vel), float(max_accel), float(max_jerk), 0.0, float(max_decel), -float(max_jerk))

    initial_knot = Knot(0,float(vel_init),float(accel_init),0)
    final_knot = Knot(None,float(vel_final),float(accel_final),None)

    spline_ = solver.target_velocity(initial_knot, final_knot)
    print spline_
    print "\nFinal: " + str(spline_.evaluate(spline_.t[-1]))

    try:
        from pyprt.shared.cspline_plotter import CSplinePlotter
        plotter = CSplinePlotter(spline_, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min)
        plotter.display_plot()
    except ImportError:
        pass
