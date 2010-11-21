from __future__ import division # use floating point division unless explicitly otherwise
from math import isnan, isinf   # requires Python 2.6

from numpy import polysub

from utility import pairwise
import utility

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

    def __eq__(self, other):
        """Exact equality of floating point values. In other words, they were
        created with the same inputs. Not just calculated to be essentially the
        same. THE SAME. Mostly useless."""
        return self.pos == other.pos and self.vel == other.vel \
               and self.accel == other.accel and self.time == other.time

class SplineError(Exception):
    pass

class OutOfBoundsError(SplineError):
    def __init__(self, start, end, value):
        self.start = start
        self.end = end
        self.value = value

    def __str__(self):
        return 'start:%s, end:%s, value:%s' % (self.start, self.end, self.value)

class CubicSpline(object):
    """Knots is a list of Knot instances. acceleration_min should be negative.

    The spline is valid on the range [start_time, end_time].
    Each spline polynomial is valid on [start_knot_time, end_knot_time).
    Except for the last polynomial which is valid on [start_knot_time, end_knot_time].
    Where '[' is inclusive and '(' is exclusive.
    """
    TIME_EPSILON = 0.0001
    DIST_EPSILON = 0.001

    def __init__(self, positions, velocities, accelerations, jerks, times):
        assert len(positions) == len(velocities) == len(accelerations) == len(times)
        assert len(jerks) == 0 if len(positions) == 0 else len(jerks) == len(positions)-1
        self.q = positions
        self.v = velocities
        self.a = accelerations
        self.j = jerks
        self.t = times
        self.h = self.make_intervals()

        self._coeffs = None

    def __str__(self):
        coeffs_str = ''.join('\n\t\t' + str(c) for c in self.coeffs)
        return "CubicSpline:\n\tq: %s\n\tv: %s\n\ta: %s\n\tt: %s\n\tj: %s\n\th: %s\n\tCoeffs:%s" \
               % (self.q, self.v, self.a, self.t, self.j, self.h, coeffs_str)

    def __len__(self):
        return len(self.t)

    def __getitem__(self, key):
        return Knot(self.q[key], self.v[key], self.a[key], self.t[key])

    @property
    def coeffs(self):
        if self._coeffs is None:
            self._coeffs = []
            for idx in range(len(self.h)):
                self._coeffs.append(self.make_coeffs(idx))

        return self._coeffs

    def copy(self):
        # OPTIMIZATION: Test if efficiency gained from copying the intervals
        # would be worth the extra effort. Would need to make intervals be calculated
        # lazily, and explicitly set them before returning the new spline.
        spline = CubicSpline(self.q[:], self.v[:], self.a[:], self.j[:], self.t[:])
        if self._coeffs is not None:
            spline._coeffs = self._coeffs[:]
        return spline

    def append(self, knot, jerk):
        """Append a new knot to the end of the spline. A SplineError is raised
        if knot.time < the spline's last time. If the spline is currently
        empty, then jerk is ignored, and may be None."""
        if self.t and knot.time < self.t[-1]:
            raise SplineError

        self.q.append(knot.pos)
        self.v.append(knot.vel)
        self.a.append(knot.accel)
        self.t.append(knot.time)
        if len(self.t) >= 2:
            self.j.append(jerk)
            self.h.append(self.t[-1] - self.t[-2])

        if self._coeffs is not None and self.h:
            self._coeffs.append(self.make_coeffs(len(self.h)-1))

    def make_intervals(self):
        return [(tf - ti) for ti, tf in pairwise(self.t)]

    def concat(self, other):
        """Returns a new spline, which is a concatenation of this spline and the
        spline 'other'. The last time, pos, etc of self must exactly coincide
        with other's first time, pos, etc, otherwise a SplineError will be raised.
        Neither self nor other is altered by this function."""
        if abs(self.t[-1] - other.t[0]) > 0.001 or abs(self.q[-1] - other.q[0]) > 0.001 \
           or abs(self.v[-1] - other.v[0]) > 0.001 or abs(self.a[-1] - other.a[0]) > 0.001:
            raise SplineError

        try:
            spline = CubicSpline(self.q + other.q[1:], # + operator returns a new list.
                                 self.v + other.v[1:], # contents of lists are primatives
                                 self.a + other.a[1:], # so copies are made.
                                 self.j + other.j,
                                 self.t + other.t[1:])
            if self._coeffs is not None and other._coeffs is not None:
                spline._coeffs = self._coeffs + other._coeffs
        except IndexError:
            # other has < 2 elements
            spline = self.copy()

        return spline

    def time_shift(self, sec):
        """Returns a new spline which has been shifted in time by 'sec'
        seconds. If sec is positive then the resulting spline will start
        later. If sec is negative then the resulting spline will start earlier.
        """
        return CubicSpline(self.q[:], self.v[:], self.a[:], self.j[:],
                           [time + sec for time in self.t])

    def position_shift(self, dist):
        """Returns a new spline which has been shifted in position by 'dist'
        meters. Dist is added to existing positions. A negative dist may be
        supplied."""
        return CubicSpline([pos + dist for pos in self.q],
                           self.v[:], self.a[:], self.j[:], self.t[:])

    def get_max_jerk(self):
        return max(self.j)

    def get_min_jerk(self):
        return min(self.j)

    def get_max_acceleration(self):
        return max(self.a)

    def get_min_acceleration(self):
        return min(self.a)

    def get_extrema_velocities(self):
        """Find points that have the potential to be a global max/min velocity.
        Each point returned is not necessarily a local max or min, since all
        knots are included.

        return: A pair of lists. The lists contain the velocities and times, respectively.
                The lists are ordered with respect to each other, but are not
                otherwise sorted.
        """
        # all knots
        vels = self.v[:]
        times = self.t[:]

        # check if there's a zero crossing in acceleration between knots
        for i in range(len(self.t)-1):
            jerk = self.j[i]
            if jerk != 0:
                crossing_t = -self.a[i]/jerk + self.t[i] # accel crosses zero
                if self.t[i] < crossing_t < self.t[i+1]:
                    vels.append(self.evaluate(crossing_t).vel)
                    times.append(crossing_t)

        return vels, times

    def get_max_velocity(self):
        """Returns the value of the maximum velocity over the whole spline."""
        return max(self.get_extrema_velocities()[0])

    def get_min_velocity(self):
        """Returns the value of the minimum velocity over the whole spline."""
        return min(self.get_extrema_velocities()[0])

    def evaluate(self, time):
        """Returns a Knot instance. Raises an OutOfBoundsError if time is not
        within the spline's valid range."""

        # Optimization note: This is about 5 times faster than using a call to
        # numpy.polyval, even when the poly coefficients are precalculated.
        t = self.t
        if time < t[0] or time > t[-1]:
            raise OutOfBoundsError(t[0], t[-1], time)

        i = self._get_index_from_time(time)

        # Time is almost exactly on a knot. Also captures case where spline has only one time.
        if abs(self.t[i] - time) < 1E-6:
            return Knot(self.q[i], self.v[i], self.a[i], time)

        j = self.j[i]
        a = self.a[i]
        v = self.v[i]
        q = self.q[i]

        delta_t = time - t[i]
        delta_t__2 = delta_t * delta_t
        delta_t__3 = delta_t__2 * delta_t

        at = j*delta_t + a
        vt = j*delta_t__2/2 + a*delta_t + v
        qt = delta_t__3*(j/6) + delta_t__2*(a/2) + v*delta_t + q

        return Knot(qt, vt, at, time)

    def evaluate_sequence(self, times):
        """Similar to evaluate, but more efficient when evaluating multiple times.

        Parameters:
          times -- A sequence of times to evaluate the spline at. The times must
            be in ascending order.

        Returns:
          A sequence of Knots.
        """
        if not len(times): # empty list
            return []

        indexes = self._get_indexes_from_times(times)
        knots = []
        for time, i in zip(times, indexes):
            t = self.t[i]

            # Time is almost exactly on a knot. Also captures case where spline has only one time.
            if (time - t) < 1E-6:
                knots.append(Knot(self.q[i], self.v[i], self.a[i], time))
                continue

            j = self.j[i]
            a = self.a[i]
            v = self.v[i]
            q = self.q[i]

            delta_t = time - t
            delta_t__2 = delta_t * delta_t
            delta_t__3 = delta_t__2 * delta_t

            at = j*delta_t + a
            vt = j*delta_t__2/2 + a*delta_t + v
            qt = delta_t__3*(j/6) + delta_t__2*(a/2) + v*delta_t + q
            knots.append(Knot(qt, vt, at, time))

        return knots

    def get_time_from_dist(self, dist, now):
        """Returns the time at which the vehicle will have travelled forward by
        dist meters. Now is the current time, in seconds."""
        idx = self._get_index_from_time(now)
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
        if self._coeffs is None:
            coeffs = list(self.make_coeffs(idx)) # Just make the coeffs for the one knot interval that we care about.
        else:
            coeffs = list(self._coeffs[idx])
        coeffs[-1] -= target_pos # subtract the target_pos from the position coefficient.
        r = utility.real_roots(coeffs[0], coeffs[1], coeffs[2], coeffs[3], threshold=1E-6) # Roots of poly are intersections with target_pos.

        # get only the real roots, within the valid range for the poly
        valid_start_time = self.t[idx]
        valid_end_time = self.t[idx+1]

        # weed out roots outside of the poly's valid range
        pts = [t for t in r if
               (t >= valid_start_time or abs(t - valid_start_time) < self.TIME_EPSILON) and \
               (t <= valid_end_time or abs(t - valid_end_time) < self.TIME_EPSILON)]
        pts.sort() # want the earliest intersection with target_pos

##        # Accept that there's some inexactness, and accept a root that has
##        # an imaginary component.
##        if len(pts) == 0:
##            pts = [t.real for t in r if
##               (t.real >= valid_start_time or abs(t.real - valid_start_time) < self.TIME_EPSILON) and \
##               (t.real <= valid_end_time or abs(t.real - valid_end_time) < self.TIME_EPSILON)]
##            pts.sort()
##            assert abs(self.evaluate(pts[0]).pos - target_pos) < 0.5, (self.evaluate(pts[0]), target_pos) # loose sanity check

        # TODO: use the bisect algo to find roots instead. I don't see a way
        # around the precision loss that occurs in finding the poly's coefficents
        # when time is very large. If it worked out to 86400 seconds (24 hours),
        # it would be okay, but it doesn't.

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
        j = self.j[left_idx]

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
##            cv = (t2_2*(v1-ca*t1) - t1_2*(v2-ca*t2)) / (t2_2 - t1_2) # original form
            cv = ((t2_2*v1 - t1_2*v2) + ca*t1*t2*(t1-t2)) / ((t2+t1)*(t2-t1)) # crafted to reduce rounding error
            if isnan(cv) or isinf(cv):
                raise ZeroDivisionError
        except ZeroDivisionError:
            cv = v1 - j*t1_2/2 - ca*t1
        try:
##            cq = (t2_3*(q1 - ca*t1_2/2 - cv*t1) - t1_3*(q2 - ca*t2_2/2 - cv*t2)) / (t2_3 - t1_3) # original form
            cq = (t2_3*q1 - t1_3*q2 + t1_2*t2_2*ca/2*(t1-t2) + t1*t2*cv*((t1+t2)*(t1-t2))) / ((t2-t1)*(t2*t1 + t2_2 + t1_2)) # slower, but better numerical precision
            if isnan(cq) or isinf(cq):
                raise ZeroDivisionError
        except ZeroDivisionError:
            cq = q1 - j*t1_3/6 - ca*t1_2/2 - cv*t1
        coeffs = (j/6, ca/2, cv, cq)

##        # Check for inconsistancies between the polynomial and the knots. (SLOW!)
##        if __debug__ and self.h[left_idx] > 0.0001: # ignore tiny h's.
##            from scipy import poly1d
##            epsilon = 0.01
##            poly = poly1d(coeffs)
##            assert abs(poly(self.t[left_idx]) - self.q[left_idx]) < epsilon
##            assert abs(poly(self.t[right_idx]) - self.q[right_idx]) < epsilon
##            assert abs(poly.deriv(1)(self.t[left_idx]) - self.v[left_idx]) < epsilon
##            assert abs(poly.deriv(1)(self.t[right_idx]) - self.v[right_idx]) < epsilon
##            assert abs(poly.deriv(2)(self.t[left_idx]) - self.a[left_idx]) < epsilon
##            assert abs(poly.deriv(2)(self.t[right_idx]) - self.a[right_idx]) < epsilon

        return coeffs

    def copy_left(self, time):
        """Returns a new spline that only contains the data to the "left"
        (i.e. earlier) than 'time'.
        """
        try:
            start_time = self.t[0]
            return self.slice(start_time, time)
        except IndexError: # empty spline
            raise OutOfBoundsError(float('nan'), float('nan'), time)

    def copy_right(self, time):
        """Returns a new spline that only contains the data to the "right"
        (i.e. later) than 'time'."""
        try:
            end_time = self.t[-1]
            return self.slice(time, end_time)
        except IndexError: # empty spline
            raise OutOfBoundsError(float('nan'), float('nan'), time)

    def slice(self, start_time, end_time):
        """Returns a slice of the spline between the two times. The slice
        is a copy and changes to it do not affect the original.

        raises: OutOfBoundsError if start_time or end_time are outside the
                time range of the spline.

        return: A CubicSpline
        """
        if start_time > end_time:
            raise OutOfBoundsError(start_time, end_time, start_time)

        if start_time == end_time:
            knot = self.evaluate(start_time)
            return CubicSpline([knot.pos], [knot.vel], [knot.accel], [], [knot.time])

        start_idx = self._get_index_from_time(start_time) + 1 # right idx
        end_idx = self._get_index_from_time(end_time)
        if self.t[end_idx] != end_time:
            end_idx += 1 # one past, for array splice

        start_knot, end_knot = self.evaluate_sequence( (start_time, end_time) )

        spline = CubicSpline([start_knot.pos] + self.q[start_idx:end_idx] + [end_knot.pos],
                             [start_knot.vel] + self.v[start_idx:end_idx] + [end_knot.vel],
                             [start_knot.accel] + self.a[start_idx:end_idx] + [end_knot.accel],
                             self.j[start_idx-1:end_idx],
                             [start_knot.time] + self.t[start_idx:end_idx] + [end_knot.time])

        # Reuse the already calculated coeffs. At the end segments, the
        # coefficients are unchanged, they're just valid for shorter time spans.
        if self._coeffs is not None:
            coeffs = self._coeffs[start_idx-1:end_idx]

        return spline

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
        spline_msg: An api.Spline message instance. Ommits extreamly short
        duration coefficients, because they add no useful information and add
        large amounts of error.
        """
        spline_msg.times.append(self.t[0])
        for c, (ti, tf) in zip(self.coeffs, pairwise(self.t)):
            poly_msg = spline_msg.polys.add()
            poly_msg.coeffs.extend( c )
            spline_msg.times.append(tf)

    def _get_index_from_time(self, time):
        """Returns the left index corresponding to time."""
        if time < self.t[0] or time > self.t[-1]:
            raise OutOfBoundsError(self.t[0], self.t[-1], time)

        for idx in xrange(len(self.t)-1, -1, -1): # iterate in reverse
            if self.t[idx] <= time:
                if idx == len(self.t)-1:
                    return max(idx-1, 0)
                else:
                    return idx

    def _get_indexes_from_times(self, times):
        """Returns a sequence containing the left indexes corresponding to each
        time in times. More efficient than repeatedly calling _get_index_from_time.
        Times must be in ascending order.
        """
        if not len(times):
            return []

        if times[0] < self.t[0]:
            raise OutOfBoundsError(self.t[0], self.t[-1], times[0])
        if times[-1] > self.t[-1]:
            raise OutOfBoundsError(self.t[0], self.t[-1], times[-1])

        indexes = [ self._get_index_from_time(times[0]) ]
        index = indexes[-1]
        for t in times[1:]:
            while t > self.t[index]: # increment index until it's a right index for t
                index += 1

            if t == self.t[index] and index != len(self.t)-1:
                indexes.append(index)

            else:
                indexes.append(max(index-1,0))
        return indexes

    def find_intersection(self, other, start_time, end_time,
                           dist):
        """Finds position intersections that occur between self and other (spline), in
        the time range from start_time to end_time. Dist is the relative distance
        between between self and other at start_time. Returns the time of the
        earliest intersection, or None if no intersections occur.
        """
        assert isinstance(other, CubicSpline)
        try:
            self_idx_start = self._get_index_from_time(start_time)
            self_idx_end = self._get_index_from_time(end_time)+1 # returns left index, want right
            other_idx_start = other._get_index_from_time(start_time)
            other_idx_end = other._get_index_from_time(end_time)+1 # returns left index, want right
        except OutOfBoundsError:
            return None

        # Create a pair of iterators for self and other. Iterate over the splines
        # supplying (coeffs, (start_valid_time, end_valid_time)) tuples
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
            r = utility.real_roots(test_coeffs[0], test_coeffs[1], test_coeffs[2], test_coeffs[3], threshold=1E-6)

            # weed out roots outside of the poly's valid range
            pts = [t for t in r if
                   (t >= test_ti or abs(t - test_ti) < self.TIME_EPSILON) and \
                   (t <= test_tf or abs(t - test_tf) < self.TIME_EPSILON) and \
                   t >= start_time and t <= end_time]
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
        from numpy import poly1d
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

