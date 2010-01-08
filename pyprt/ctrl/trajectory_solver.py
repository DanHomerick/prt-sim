from __future__ import division # use floating point division unless explicitly otherwise

import scipy.optimize
import math
from scipy import inf
from pyprt.shared.cubic_spline import CubicSpline
from pyprt.shared.cubic_spline import Knot

class TrajectoryError(Exception):
    """Raised when a generated trajectory is known to be flawed."""

class FatalTrajectoryError(Exception):
    """Raised when the specified trajectory cannot be met, such as when an
    initial or final value is outside of the specified acceleration or velocity
    limits."""

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

    def target_position(self, knot_initial, knot_final):
        """Targets the position and velocity values in knot_final.

        Raises FatalTrajectoryError if it is unable to generate a valid spline.

        Returns a cubic_spline.CSpline.
        """
        knot_final = knot_final.copy() # don't alter the original
        knot_final.time = inf

        if knot_final.pos > knot_initial.pos:
            flip = True
            if __debug__:
                if not (self.v_min <= knot_initial.vel <= self.v_max and
                        self.v_min <= knot_final.vel <= self.v_max and
                        self.a_min <= knot_initial.accel <= self.a_max and
                        self.a_min <= knot_final.accel <= self.a_max):
                    raise FatalTrajectoryError("Endpoint outside of solver's limit")
        elif knot_final.pos < knot_initial.pos: # travelling in reverse
            flip = False
            if __debug__:
                if not (self.v_min <= -knot_initial.vel <= self.v_max and
                        self.v_min <= -knot_final.vel <= self.v_max and
                        self.a_min <= -knot_initial.accel <= self.a_max and
                        self.a_min <= -knot_final.accel <= self.a_max):
                    raise FatalTrajectoryError("Endpoint outside of solver's limit")
        else:
            # Could return a spline, but in what form? 0 knots, 1 knot, 2 knots?
            raise FatalTrajectoryError("You didn't bloody well want to move then, did ya?")

        try:
            spline = self.target_position_vmax(knot_initial, knot_final)
        except TrajectoryError:
            try:
                spline = self.target_position_amax(knot_initial, knot_final)
            except TrajectoryError:
                spline = self.target_position_none(knot_initial, knot_final)

        if __debug__:
            if not flip:
                if spline.get_max_velocity() > self.v_max:
                    raise FatalTrajectoryError("Unable to comply with max velocity limit.")
                if spline.get_min_velocity() < self.v_min:
                    raise FatalTrajectoryError("Unable to comply with min velocity limit.")
            else:
                if -spline.get_max_velocity() > self.v_max:
                    raise FatalTrajectoryError("Unable to comply with max velocity limit.")
                if -spline.get_min_velocity() < self.v_min:
                    raise FatalTrajectoryError("Unable to comply with min velocity limit.")

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

        spline = CubicSpline([qi], [vi], [ai], [ti])
        for k in [k1, k2, k3, k4, k5, k6, final]:
            spline.append(k)

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
        except ValueError: # No nonnegative, real solutions. Could be caused by
                            # targetting behind the vehicle when v_min=0, for example.
            assert len(self.nonnegative_roots(A, B, C)) == 0
            raise TrajectoryError('No nonnegative, real solutions') # TODO: Should this be Fatal?


        h1 = alpha - an/ax*h3

        k2 = self.create_knot_after(k1, h1, ax)
        k3 = self.create_knot_after(k2, h2, an)
        k4 = self.create_knot_after(k3, h3, an)
        final.time = k4.time + h4

        spline = CubicSpline([q0], [v0], [a0], [initial.time])
        for k in [k1, k2, k3, k4, final]:
            spline.append(k)

        return spline

    def target_position_none(self, knot_initial, knot_final):
        """Trajectory does not reach v_max or a_max.
        Assumed that the spline should have an acceleration profile of:

          h0   h2  h3

            /\
           /  \
          /    \
                \    /
                 \  /
                  \/

         t0 t1    t2 t3

        where h's are the timespans of each polynomial, and t's are the knot times.
        """
        if knot_final.pos > knot_initial.pos:
            jx = self.j_max
            jn = self.j_min
        elif knot_final.pos < knot_initial.pos:
            jx = -self.j_max
            jn = -self.j_min
        else:
            raise FatalTrajectoryError("No change in position.")

        initial_spline = self._initial_estimate(knot_initial, knot_final, jx, jn)

        # If all the assumptions were met, then the initial_spline is correct.
        if knot_initial.vel == 0 and knot_final.vel == 0 and \
                knot_initial.accel == 0 and knot_final.accel == 0 and \
                jx == -jn:
            return initial_spline

        # Otherwise do non-linear optimization to get the correct solution.
        def update_and_error(hs):
            """Uses closures to keep track of the current state."""
            self._soln_spline = self._update_estimate(hs, knot_initial, jx, jn)
            error = self.target_position_error_fnc(self._soln_spline, knot_final)
            return error

        stats = scipy.optimize.fmin(update_and_error,
                                    initial_spline.h,
                                    maxiter=1E6,
                                    maxfun=1E6,
                                    xtol=1E-6,
                                    ftol=1E-6,
                                    full_output=True)

        if min(self._soln_spline.h) < -0.00001:
            raise FatalTrajectoryError("Trajectory failed.")

        return self._soln_spline


    def _initial_estimate(self, knot_initial, knot_final, jx, jn):
        """Make the following simplifying assumptions, regardless of how inaccurate...
         jmax == -jmin
         a_initial == a_final == 0
         v_initial == v_final == 0
         """     

        ### Reasoning:
        # Break the accel profile into 4 segments of equal time. h = h0 = h1 = h2 = h3
        # q01 = jx*h**3/6
        # a1 = jx*h
        # v1 = jx*h**2/2
        # q12 = jn*h**3/6 + a1*h**2/2 + v1*h
        # q_total = 2*q01 + 2*q12   due to assumed symmetry
        # q_total = 2*jx*h**3       simplified
        h = math.pow((knot_final.pos - knot_initial.pos)/(2*jx), 1/3)
        return self._update_estimate([h, 2*h, h], knot_initial, jx, jn)


    def _update_estimate(self, hs, knot_initial, jx, jn):
        """Create a new spline using the 3 timespans"""
        k1 = self.create_knot_after(knot_initial, hs[0], jx*hs[0] + knot_initial.accel)
        k2 = self.create_knot_after(k1, hs[1], jn*hs[1] + k1.accel)
        k3 = self.create_knot_after(k2, hs[2], jx*hs[2] + k2.accel)
        spline = CubicSpline([], [], [], [])
        for k in [knot_initial, k1, k2, k3]:
            spline.append(k)
        return spline

    def target_position_error_fnc(self, spline, knot_final):
        neg_h_penalty = sum(abs(h) if h < 0 else 0 for h in spline.h)

       # squared error
        error = (knot_final.pos - spline.q[-1])**2 + \
                (knot_final.vel - spline.v[-1])**2 + \
                (knot_final.accel - spline.a[-1])**2 + \
                neg_h_penalty

        return error

    def nonnegative_roots(self, A, B, C):
        """Finds the nonnegative roots of a 2nd degree polynomial, using quadratic formula.
        Raises a TrajectoryError if answer would be imaginary."""
        try:
            tmp = (B**2 - 4*A*C)**(0.5)
        except ValueError: # negative number cannot be raised to a fractional power
            raise TrajectoryError('No real roots')

        roots = [(-B + tmp)/(2*A), (-B - tmp)/(2*A)]
        def is_positive(x):
            return x >= 0
        return filter(is_positive, roots)

    def nonpositive_roots(self, A, B, C):
        """Finds the nonpositive roots of a 2nd degree polynomial, using quadratic formula.
        Raises a TrajectoryError if answer would be imaginary."""
        try:
            tmp = (B**2 - 4*A*C)**(0.5)
        except ValueError: # negative number cannot be raised to a fractional power
            raise TrajectoryError('No real roots')

        roots = [(-B + tmp)/(2*A), (-B - tmp)/(2*A)]
        def is_negative(x):
            return x <= 0
        return filter(is_negative, roots)

    def target_velocity(self, initial, final):
        """Similar to target_position, but the final position is ignored."""
        final = final.copy() # don't alter the original
        final.time = inf

        if __debug__:
            if not (self.v_min <= initial.vel <= self.v_max):
                raise FatalTrajectoryError("Endpoint velocity outside of solver's limit")
            if not (self.a_min <= initial.accel <= self.a_max):
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

        spline = CubicSpline([], [], [], [])
        for knot in knots:
            spline.append(knot)

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
    knot_initial = Knot(0,0,0,0)
    knot_final = Knot(250,0,0,None)
    solver = TrajectorySolver(25, 5, 2.5)  # v_max, a_max, j_max

    spline = solver.target_position(knot_initial, knot_final)
    
    from pyprt.shared.cspline_plotter import CSplinePlotter
    plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min)
    plotter.display_plot()