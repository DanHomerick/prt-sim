from __future__ import division # use floating point division unless explicitly otherwise
from math import isnan, isinf   # requires Python 2.6

from numpy import roots, polysub

from utility import pairwise



# Optimization notes:
# Store jerk in addition to other values, since it's recalculated several places.
#   Associated with this, change trajectory_solver.create_knot_* to accept jerk
#   instead of acceleration.

class Knot(object):
    def __init__(self, position, velocity, acceleration, time):
        self.pos = position
        self.vel = velocity
        self.accel = acceleration
        self.time = time

    def __str__(self):
        return str(self.to_tuple())

    def copy(self):
        return Knot(self.pos, self.vel, self.accel, self.time)

    def to_tuple(self):
        return (self.pos, self.vel, self.accel, self.time)

class CubicSpline(object):
    """Knots is a list of Knot instances. acceleration_min should be negative."""
    TIME_EPSILON = 0.0001
    DIST_EPSILON = 0.001

    def __init__(self, positions, velocities, accelerations, times):
        self.q = positions
        self.v = velocities
        self.a = accelerations
        self.t = times
        self.h = self.make_intervals()

        self.coeffs = []
        for idx in range(len(self.h)):
            self.coeffs.append(self.make_coeffs(idx))

    def __str__(self):
        coeffs_str = ''.join('\n\t\t' + str(c) for c in self.coeffs)
        return "CubicSpline:\n\tq: %s\n\tv: %s\n\ta: %s\n\tt: %s\n\th: %s\n\tCoeffs:%s" \
               % (self.q, self.v, self.a, self.t, self.h, coeffs_str)

    def __len__(self):
        return len(self.t)

    def __getitem__(self, key):
        return Knot(self.q[key], self.v[key], self.a[key], self.t[key])

    def insert(self, index, knot):
        self.q.insert(index, knot.pos)
        self.v.insert(index, knot.vel)
        self.a.insert(index, knot.accel)
        self.t.insert(index, knot.time)
        self.h = self.make_intervals()

        del self.coeffs[index]
        self.coeffs.insert(index, self.make_coeffs(index))
        self.coeffs.insert(index+1, self.make_coeffs(index))

    def append(self, knot):
        self.q.append(knot.pos)
        self.v.append(knot.vel)
        self.a.append(knot.accel)
        self.t.append(knot.time)
        if len(self.t) >= 2:
            self.h.append(knot.time - self.t[-2])
            self.coeffs.append(self.make_coeffs(len(self.h)-1))

    def make_intervals(self):
        return [(tf - ti) for ti, tf in pairwise(self.t)]

    def get_max_acceleration(self):
        return max(self.a)

    def get_min_acceleration(self):
        return min(self.a)

    def get_extrema_velocity(self):
        """Gets points that have the potential to be a global max/min velocity.
        Each point returned is not necessarily a local max or min, since all
        knots are included."""
        # This isn't terribly efficient
        extreama = []
        extreama.extend(self.v) # append all knots

        for i in range(len(self.q)-1):
            # check if there's a zero crossing in acceleration
            if self.h[i]:
                jerk = (self.a[i+1]-self.a[i])/self.h[i]
                if jerk:
                    t = -self.a[i]/jerk + self.t[i]
                    if self.t[i] < t < self.t[i+1]:
                        extreama.append(self.evaluate(t).vel)
        extreama.append(self.v[-1])
        return extreama

    def get_max_velocity(self):
        """Returns the value of the maximum velocity over the whole spline."""
        return max(self.get_extrema_velocity())

    def get_min_velocity(self):
        """Returns the value of the minimum velocity over the whole spline."""
        return min(self.get_extrema_velocity())

    def find_jerks(self):
        """Returns the jerks that occur in each part of the whole spline"""
        jerks = []

        # interval, knot_initial, knot_final
        for hi, (ai, af) in zip(self.h, pairwise(self.a)):
            jerks.append((af - ai)/hi)
        return jerks

    def evaluate(self, time):
        """Returns a Knot instance. Raises an OutOfBoundsError if time is not
        within the spline's valid range."""

        # Optimization note: This is about 5 times faster than using a call to
        # numpy.polyval, even though the poly coefficients are precalculated.
        t = self.t
        if time < t[0] or time > t[-1]:
            raise OutOfBoundsError(t[0], t[-1], time)

        i = self._get_idx_from_time(time)
        a = self.a[i]
        v = self.v[i]
        q = self.q[i]

        delta_t = time - t[i]
        delta_t__2 = delta_t * delta_t
        delta_t__3 = delta_t__2 * delta_t
        jerk = (self.a[i+1]-a)/self.h[i] if self.h[i] else 0 # guard against zero time segments

        at = jerk*delta_t + a
        vt = jerk*delta_t__2/2 + a*delta_t + v
        qt = jerk*delta_t__3/6 + a*delta_t__2/2 + v*delta_t + q

        return Knot(qt, vt, at, time)

    def get_time_from_dist(self, dist, now):
        """Returns the time at which the vehicle will have travelled forward by
        dist meters. Now is the current time, in seconds."""
        idx = self._get_idx_from_time(now)
        target_pos = self.evaluate(now).pos + dist
        end_pos = self.q[idx+1]
        # Iterate over knots until we reach the polynomial containing target_pos
        try:
            while True:
                # Short circuit the root finding if target_pos happens to be a knot position.
                if abs(target_pos - end_pos) < self.DIST_EPSILON:
                    return self.t[idx+1]
                elif target_pos > end_pos: # keep looking
                    idx += 1
                    end_pos = self.q[idx+1]
                else: # found the correct poly
                    break
        except IndexError:
            raise OutOfBoundsError(self.q[0], self.q[-1], target_pos)

        # idx is now the left_index for the poly containing target_pos
        coeffs = list(self.coeffs[idx])
        coeffs[-1] -= target_pos # subtract the target_pos from the position coefficient.
        r = roots(coeffs) # Roots of poly are intersections with target_pos

        # get only the real roots, within the valid range for the poly
        valid_start_time = self.t[idx]
        valid_end_time = self.t[idx+1]

        # weed out imaginary roots, and those outside of the poly's valid range
        pts = [t.real for t in r if
               abs(t.imag - 0.0000) < self.TIME_EPSILON and \
               (t.real >= valid_start_time or abs(t.real - valid_start_time) < self.TIME_EPSILON) and \
               (t.real <= valid_end_time or abs(t.real - valid_end_time) < self.TIME_EPSILON)]
        pts.sort() # want the earliest intersection with target_pos

##        assert abs(polyval(self.coeffs[idx], pts[0]) - target_pos) < self.DIST_EPSILON
        return pts[0]

    def make_coeffs(self, left_idx):
        """Converts from the knot based representation to polynomial coefficients.
        The ordering of the coefficients matches that used by scipy.poly1d, the
        highest degree polynomial first. Returns a 4-tuple.
        """
        # Find poly coefficients for the poly between each pair of knots.
        right_idx = left_idx+1
        t1 = self.t[left_idx]
        t2 = self.t[right_idx]
        q1 = self.q[left_idx]
        q2 = self.q[right_idx]
        v1 = self.v[left_idx]
        v2 = self.v[right_idx]
        a1 = self.a[left_idx]
        a2 = self.a[right_idx]
        j = (a2-a1)/self.h[left_idx] if self.h[left_idx] != 0 else 0

        # Only calc the exponents once
        t1_2 = t1*t1
        t1_3 = t1_2*t1
        t2_2 = t2*t2
        t2_3 = t2_2*t2

        # These equations find the unique 3rd order polynomial that passes
        # through the specified initial and final positions, velocities, and
        # accelerations and jerk. They are derived by sucessively integrating
        # the jerk, and solving the integration constants with the desired
        # constraints. Used sympy to help simplify the resulting equations.
        try:
            ca = (t2*a1 - t1*a2)/(t2 - t1)
            if isnan(ca) or isinf(ca):
                raise ZeroDivisionError
        except ZeroDivisionError:
            ca = a1 - j*t1
        try:
            cv = t1_2/(t1_2 - t2_2)*(v2 - ca*t2) - (t2_2/(t1_2 - t2_2))*(v1 - ca*t1)
            if isnan(cv) or isinf(cv):
                raise ZeroDivisionError
        except ZeroDivisionError:
            cv = v1 - j*t1_2/2 - ca*t1
        try:
            cq = t1_3/(t1_3 - t2_3)*(q2 - ca*t2_2/2 - cv*t2) - t2_3/(t1_3 - t2_3)*(q1 - ca*t1_2/2 - cv*t1)
            if isnan(cq) or isinf(cq):
                raise ZeroDivisionError
        except ZeroDivisionError:
            cq = q1 - j*t1_3/6 - ca*t1_2/2 - cv*t1
        coeffs = (j/6, ca/2, cv, cq)

##        # Check for inconsistancies between the polynomial and the knots. (SLOW!)
##        if __debug__:
##            from scipy import poly1d
##            epsilon = 0.1
##            poly = poly1d(coeffs)
##            assert abs(poly(self.t[left_idx]) - self.q[left_idx]) < epsilon
##            assert abs(poly(self.t[right_idx]) - self.q[right_idx]) < epsilon
##            assert abs(poly.deriv(1)(self.t[left_idx]) - self.v[left_idx]) < epsilon
##            assert abs(poly.deriv(1)(self.t[right_idx]) - self.v[right_idx]) < epsilon
##            assert abs(poly.deriv(2)(self.t[left_idx]) - self.a[left_idx]) < epsilon
##            assert abs(poly.deriv(2)(self.t[right_idx]) - self.a[right_idx]) < epsilon

        return coeffs

    def clear(self, time=0):
        """The spline is cleared from time onwards, and a new knot is created at time.
        """
        if not time:
            idx = 1 # don't delete the initial spline
        else:
            knot = self.evaluate(time)
            idx = self._get_idx_from_time(time)
            idx += 1

        del self.t[idx:] # deletes the element at idx and beyond
        del self.q[idx:]
        del self.v[idx:]
        del self.a[idx:]
        if idx > 0:
            del self.h[idx-1:]
            del self.coeffs[idx-1:]
        else:
            del self.h[0:]
            del self.coeffs[0:]

        if time:
            self.append(knot)

##    def extend(self, spline):
##        """Merges a CubicSpline instance with this. If there is overlap in the
##        time-domain, then 'spline' takes priority."""
##        assert isinstance(spline, CubicSpline)
##        self.clear(spline.t[0])
##        self.q.extend(spline.q)
##        self.v.extend(spline.v)
##        self.a.extend(spline.a)
##        self.t.extend(spline.t)
##        self.h = self.make_intervals()

    def fill_spline_msg(self, spline_msg):
        """Fills a api.Spline msg with data from a cubic spline.
        spline_msg: An api.Spline message instance.
        """
        for c in self.coeffs:
            poly_msg = spline_msg.polys.add()
            poly_msg.coeffs.extend( c )

        spline_msg.times.extend(self.t)

    def _get_idx_from_time(self, time):
        """Returns the left index corresponding to time."""
        if time < self.t[0] or time > self.t[-1]:
            raise OutOfBoundsError(self.t[0], self.t[-1], time)

        for idx in xrange(len(self.t)-1, -1, -1): # iterate in reverse
            if self.t[idx] <= time:
                if idx == len(self.t)-1:
                    return max(idx-1, 0)
                else:
                    return idx

    def find_intersection(self, other, start_time, end_time,
                           dist):
        """Finds position intersections that occur between self and other (spline), in
        the time range from start_time to end_time. Dist is the relative distance
        between between self and other at start_time. Returns the time of the
        earliest intersection, or None if no intersections occur.
        """
        assert isinstance(other, CubicSpline)
        try:
            self_idx_start = self._get_idx_from_time(start_time)
            self_idx_end = self._get_idx_from_time(end_time)
            other_idx_start = other._get_idx_from_time(start_time)
            other_idx_end = other._get_idx_from_time(end_time)
        except OutOfBoundsError:
            return None

        s_iter = iter(zip(self.coeffs[self_idx_start:self_idx_end],
                                     pairwise(self.t[self_idx_start:self_idx_end+1])))

        o_iter = iter(zip(other.coeffs[other_idx_start:other_idx_end],
                          pairwise(other.t[other_idx_start:other_idx_end+1])))
        try:
            s_c, (s_ti, s_tf) = s_iter.next()
            o_c, (o_ti, o_tf) = o_iter.next()
        except StopIteration:
            return None

        offset = dist - other.evaluate(start_time).pos + self.evaluate(start_time).pos

        while True:
            test_coeffs = polysub(o_c, s_c)
            test_coeffs[-1] = test_coeffs[-1] + offset
            test_ti = max(s_ti, o_ti)
            test_tf = min(s_tf, o_tf)
            r = roots(test_coeffs)

            # weed out imaginary roots, and those outside of the poly's valid range
            pts = [t.real for t in r if
                   abs(t.imag - 0.0000) < self.TIME_EPSILON and \
                   (t.real >= test_ti or abs(t.real - test_ti) < self.TIME_EPSILON) and \
                   (t.real <= test_tf or abs(t.real - test_tf) < self.TIME_EPSILON) and \
                   t.real >= start_time and t.real <= end_time]
            pts.sort() # want the earliest intersection with target_pos
            if len(pts) > 0:
                return pts[0] # <------ normal exit point

            # Advance polys such that all overlapping ranges are checked.
            try:
                if s_tf < o_tf:
                    s_c, (s_ti, s_tf) = s_iter.next()
                elif o_tf < s_tf:
                    o_c, (o_ti, o_tf) = o_iter.next()
                else:
                    s_c, (s_ti, s_tf) = s_iter.next()
                    o_c, (o_ti, o_tf) = o_iter.next()
            except StopIteration:
                return None # <------- failure exit point

    def test_3(self):
        for i1, i2 in pairwise(range(len(self.t))):
            t1 = self.t[i1]
            t2 = self.t[i2]
            q1 = self.q[i1]
            q2 = self.q[i2]
            v1 = self.v[i1]
            v2 = self.v[i2]
            a1 = self.a[i1]
            a2 = self.a[i2]
            j = (self.a[i2]-self.a[i1])/self.h[i1]
        t1_2 = t1*t1
        t1_3 = t1_2*t1
        t2_2 = t2*t2
        t2_3 = t2_2*t2
        ca = (t2*a1 - t1*a2)/(t2 - t1)
        cv = t1_2/(t1_2 - t2_2)*(v2 - ca*t2) - (t2_2/(t1_2 - t2_2))*(v1 - ca*t1)
        cq = t1_3/(t1_3 - t2_3)*(q2 - ca*t2_2/2 - cv*t2) - t2_3/(t1_3 - t2_3)*(q1 - ca*t1_2/2 - cv*t1)
        A = j/6
        B = ca/2
        C = cv
        D = cq
        p = poly1d([A, B, C, D])
        print 'pos:', p(self.t[0]), p(self.t[1])
        print 'vel:', p.deriv(1)(self.t[0]), p.deriv(1)(self.t[1])
        print 'accel:', p.deriv(2)(self.t[0]), p.deriv(2)(self.t[1])
        print 'jerk:', p.deriv(3)(self.t[0]), p.deriv(3)(self.t[1])
# Timed at 8.0 sec for 1E6 iterations (ommitting print statements and poly1d creation)
# Timed at 7.2 sec when reusing the time squared and cubed values

class OutOfBoundsError(Exception):
    def __init__(self, start, end, value):
        self.start = start
        self.end = end
        self.value = value

    def __str__(self):
        return 'start:%s, end:%s, value:%s' % (self.start, self.end, self.value)

if __name__ == '__main__':
    import timeit
##    timer = timeit.Timer('spline.get_coeffs()', setup='from pyprt.shared.cubic_spline import CubicSpline; spline = CubicSpline([0, 104.16666666666669], [0, 62.5], [0, 25], [1, 6])')
##    print timer.timeit()

#  get_position(20) takes 25.5 sec for 500,000 iterations.
#  evaluate(20).pos takes  4.4 sec for 500,000 iterations.
    timer = timeit.Timer('spline.evaluate(20).pos', setup="""from pyprt.shared.cubic_spline import CubicSpline;
spline = CubicSpline(
        [200, 196.66666666666666, 150.69790904099514, 49.302090959004843, 3.3333333333333428, 0], # pos
        [0, -5.0, -22.015621187164243, -22.015621187164243, -5.0, 0], # vel
        [0, -5, -5, 5, 5, 0], # accel
        [10, 12.0, 15.403124237432849, 19.403124237432849, 22.806248474865697, 24.806248474865697]) # time""")
    print timer.timeit(500000)

