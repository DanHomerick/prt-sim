from __future__ import division # use floating point division unless explicitly otherwise

# Optimization notes:
# Consider storing the polynomial coefficients instead of knot points. This is
#   a low priority option though.
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
#    def __init__(self, polys=[], durations=[], times=[]):
#        self.

    def __init__(self, positions, velocities, accelerations, times):
        self.q = positions
        self.v = velocities
        self.a = accelerations
        self.t = times
        self.h = self.make_intervals()

    def __str__(self):
        return "CubicSpline:\n\tq: %s\n\tv: %s\n\ta: %s\n\tt: %s\n\th: %s" % (self.q, self.v, self.a, self.t, self.h)

    def insert(self, index, knot):
        self.q.insert(index, knot.pos)
        self.v.insert(index, knot.vel)
        self.a.insert(index, knot.accel)
        self.t.insert(index, knot.time)
        self.h = self.make_intervals()

    def append(self, knot):
        if len(self.t) > 0:
            self.h.append(knot.time - self.t[-1])
        self.q.append(knot.pos)
        self.v.append(knot.vel)
        self.a.append(knot.accel)
        self.t.append(knot.time)

    def make_intervals(self):
        return [(tf - ti) for ti, tf in zip(self.t[:-1], self.t[1:])]

    def get_max_acceleration(self):
        return max(self.a)

    def get_min_acceleration(self):
        return min(self.a)

    def _get_extrema_velocity(self):
        """Gets points that have the potential to be an extreama."""
        extreama = []
        for i in range(len(self.q)-1):
            extreama.append(self.v[i]) # append all knots

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
        """Returns the value of the maximum velocity over the whole spline.
        Note that this is not necessarily the maximum speed!"""
        return max(self._get_extrema_velocity())

    def get_min_velocity(self):
        """Returns the value of the minimum velocity over the whole spline.
        Note that this is not necessarily the minimum speed!"""
        return min(self._get_extrema_velocity())


    def find_jerks(self):
        """Returns the jerks that occur in each part of the whole spline"""
        jerks = []

        # interval, knot_initial, knot_final
        for hi, ai, af in zip(self.h, self.a[:-1], self.a[1:]):
            jerks.append((af - ai)/hi)
        return jerks

    def evaluate(self, time):
        t = self.t
        if time < t[0] or time > t[-1]:
            return None

        q = self.q
        v = self.v
        a = self.a

        for i in range(len(t)-1):
            # find the right interval.
            if t[i] <= time <= t[i+1]:
                delta_t = time - t[i]
                jerk = (a[i+1]-a[i])/self.h[i] if self.h[i] else 0 # guard against zero time segments

                at = jerk*delta_t + a[i]
                vt = jerk*delta_t**2/2 + a[i]*delta_t + v[i]
                qt = jerk*delta_t**3/6 + a[i]*delta_t**2/2 + v[i]*delta_t + q[i]

                return Knot(qt, vt, at, time)

        raise Exception('Unexpected case.')

    def get_coeffs(self):
        """Converts from the knot based representation to polynomial coefficients.
        The ordering of the coefficients matches that used by scipy.poly1d, the
        highest degree polynomial first.

        Note that the coeffs are not time-shifted. Each equation is assumed
        to start with t=0. This matches the existing convention for the sim.

        Returns a list of 4-tuples.
        """
        coeffs = []
        for i in range(len(self.h)):
            if self.h[i] != 0:
                coeffs.append((
                    (self.a[i+1]-self.a[i])/(self.h[i]*6), # jerk
                    self.a[i]/2, # accel
                    self.v[i], # vel
                    self.q[i])) # pos
        return coeffs

    def fill_spline_msg(self, spline_msg):
        """Fills a api.Spline msg with data from a cubic spline.
        spline_msg: An api.Spline message instance.
        """
        dead_zone = 0.0001
        for i in range(len(self.h)):
            if self.h[i] > dead_zone: # ommit extreamly short phases
                poly_msg = spline_msg.polys.add()
                c = [0,0,0,0]
                c[0] = (self.a[i+1]-self.a[i])/(self.h[i]*6) # jerk
                c[1] = self.a[i]/2                           # accel
                c[2] = self.v[i]                             # vel
                c[3] = self.q[i]                             # pos

                # remove drift from accumulated rounding errors
                for idx, x in enumerate(c):
                    if abs(x) < dead_zone:
                        c[idx] = 0

                poly_msg.coeffs.extend(c)
                spline_msg.times.append(self.t[i]) # time
        spline_msg.times.append(self.t[-1])