from __future__ import division # integer division may result in floats

#import matplotlib          # temp
#matplotlib.use('WXAgg')    # temp

from scipy import poly1d
from math import sqrt
from numpy import inf
#from matplotlib import pyplot

JERK = 0
ACCEL = 1
VEL = 2
POS = 3
TIME = 4


class SpeedProfiler(object):
    """Input is the initial and final velocities, and the acceleration and jerk
    limits to use. Jerk limit is assumed to be the same for positive and negative,
    and should always be supplied as a positive number.
    """

    def __init__(self, v_init, v_final, max_accel, max_decel, max_jerk=2.5, mass=450,
                 a_init=0, a_final=0, p_init=0, step=0.1):
        assert max_jerk > 0
        self.v_init = v_init
        self.v_final = v_final
        self.v_delta = v_final - v_init
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_jerk = max_jerk
        self.a_init = a_init
        self.a_final = a_final
        self.p_init = p_init
        self.mass = mass

        self.step = step # step size for discretizing results

        # A list of pair tuples. Each pair consists of:
        #     (position poly1D , segment duration)
        self.segments = list()

        self.generate_spd_profile()

    def generate_spd_profile(self):
        delta_v = self.v_delta
        a_init = self.a_init

        if delta_v == 0 and a_init == 0: # Continue at current velocity
            pos = poly1d([0,0,self.v_init, self.p_init])
            self.segments.append( (pos, inf) )
            return

        if delta_v >= 0 and a_init <= 0:
            j = self.max_jerk
            a_max = self.max_accel
        elif delta_v <= 0 and a_init >= 0:
            j = -self.max_jerk
            a_max = self.max_decel

        # delta_v and a_init have the same sign. Decide whether to accel or decel.
        elif (delta_v > 0 and a_init > 0) or (delta_v < 0 and a_init < 0):
            # vel change just from bringing initial accel back to zero:
            j = (-self.max_jerk if a_init > 0 else self.max_jerk)
            v_change = -0.5 / j * (a_init**2)

            if abs(v_change - delta_v) < 0.0005: # reducing initial accel will appox hit target vel.
                # one segment soln
                pos0 = poly1d([j/6, a_init/2, self.v_init, self.p_init])
                dur0 = -a_init / j
                assert abs(pos0.deriv()(dur0) - self.v_final) < 0.0005
                pos1 = poly1d([0, 0, self.v_final, pos0(dur0)])
                self.segments.append( (pos0, dur0) )
                self.segments.append( (pos1, inf) )
                return

            elif v_change > delta_v: # Will overshoot target velocity.
                a_max = self.max_decel
            elif v_change < delta_v: # Won't overshoot, increase accel
                j = self.max_jerk
                a_max = self.max_accel
            else:
                raise Exception, "Whoops. Bug."
        else:
            raise Exception, "Whoops. Bug."

        # calc t2 as though it will be a 3 segment soln
        t2 = -1/a_max*(-delta_v + a_max**2/j - 0.5*a_init**2/j)

        # 3 segments
        if t2 > 0:
            t1 = -1 / j * (a_init - a_max)
            t3 = a_max / j

            accel0 = poly1d([j, a_init])
            vel0 = accel0.integ(k=self.v_init)
            pos0 = vel0.integ(k=self.p_init)

            assert abs(accel0(t1) - a_max) < 0.001
            accel1 = poly1d([0, a_max])
            vel1 = accel1.integ(k=vel0(t1))
            pos1 = vel1.integ(k=pos0(t1))

            accel2 = poly1d([-j, a_max])
            vel2 = accel2.integ(k=vel1(t2))
            pos2 = vel2.integ(k=pos1(t2))

            pos3 = poly1d([0, 0, self.v_final, pos2(t3)])

            self.segments.append( (pos0, t1) )
            self.segments.append( (pos1, t2) )
            self.segments.append( (pos2, t3) )
            self.segments.append( (pos3, inf) )

        # 2 segments
        else:
            t1 = 0.5*(2*a_init**2*j**(-2) + 4*delta_v/j)**(0.5) - a_init/j
            if t1 < 0:
                print "t1 = %s, using the other one.", t1
                t1 = -0.5*(2*a_init**2*j**(-2) + 4*delta_v/j)**(0.5) - a_init/j
                assert t1 >= 0

            t2 = t1 + a_init/j

            accel0 = poly1d([j, a_init])
            vel0 = accel0.integ(k=self.v_init)
            pos0 = vel0.integ(k=self.p_init)

            accel1 = poly1d([-j, accel0(t1)])
            vel1 = accel1.integ(k=vel0(t1))
            pos1 = vel1.integ(k=pos0(t1))

            pos2 = poly1d([0,0, self.v_final, pos1(t2)])

            self.segments.append( (pos0, t1) )
            self.segments.append( (pos1, t2) )
            self.segments.append( (pos2, inf) )

        # weed out any segments with duration 0
        self.segments = [s for s in self.segments if s[1] > 0.0]

#        print "time\tpos\tvel\taccel\tjerk"
#
#
#        for eqn, time in self.segments[:-1]:
#            t = 0.0
#            print eqn
#            while t <= time:
#                print "%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f" % \
#                      (t, eqn(t), eqn.deriv(1)(t), eqn.deriv(2)(t), eqn.deriv(3)(t))
#                t += self.step
#
#            print "--------------"

##    def plot_profile(self):
##        time = list()
##        pos = list()
##        vel = list()
##        accel = list()
##        jerk = list()
##        ke = list()
##        pwr = list()
##        total_time = 0
##        for eqn, dur in self.segments[:-1]:
##            step = dur/100.0
##            pos_eqn   = eqn
##            vel_eqn   = eqn.deriv(1)
##            accel_eqn = eqn.deriv(2)
##            jerk_eqn  = eqn.deriv(3)
##            for i in xrange(101):
##                t = i*step
##                time.append(t + total_time)
##                pos.append(pos_eqn(t))
##                vel.append(vel_eqn(t))
##                accel.append(accel_eqn(t))
##                jerk.append(jerk_eqn(t))
##                # Kinetic energy, in Joules
##                ke.append( self.mass * vel_eqn(t)**2 )
##
##                try:
##                    # power, in kilowatts
##                    pwr.append(abs(ke[-1] - ke[-2]) / (step*1000))
##                except IndexError:
##                    pass
##            total_time += dur
##
##        # Position
##        pyplot.subplot(6,1,1)
##        pyplot.plot(time, pos)
##        pyplot.axhline()
##
##        # Velocity
##        pyplot.subplot(6,1,2)
##        pyplot.plot(time, vel)
##        pyplot.axhline()
##
##        # Accel
##        pyplot.subplot(6,1,3)
##        pyplot.plot(time, accel)
##        pyplot.axhline()
##        pyplot.axhline(self.max_accel, color='r')
##        pyplot.axhline(self.max_decel, color='r')
##
##        # Jerk
##        pyplot.subplot(6,1,4)
##        pyplot.plot(time, jerk)
##        pyplot.axhline()
##        pyplot.axhline(self.max_jerk, color='r')
##        pyplot.axhline(-self.max_jerk, color='r')
##
##        pyplot.subplot(6,1,5)
##        pyplot.plot(time, ke)
##
##        pyplot.subplot(6,1,6)
##        pyplot.plot(time[1:], pwr)
##        pyplot.show()

    def print_eqns(self):
        print "a_init", self.a_init
        print "v_init", self.v_init
        print "v_final", self.v_final
        print "max_accel", self.max_accel
        print "max_decel", self.max_decel
        print "max_jerk", self.max_jerk
        print "Coefficients:"
        for num, seg in enumerate(self.segments):
            print num, seg[0].c
        print "Seg\t\tTime\t\tJerk\t\tAccel\t\tVel\t\tPos"
        for num, seg in enumerate(self.segments):
            eqn, dur = seg
            jerk = eqn.deriv(3)
            accel = eqn.deriv(2)
            vel = eqn.deriv(1)
            pos = eqn
            for t in (0, dur):
                print "%s\t\t%s\t\t%s\t\t%s\t\t%s\t\t%s" \
                    % (num, t, jerk(t), accel(t), vel(t), pos(t))

    def get_profile(self):
        """Returns a list containing 2-tuples. Each tuple contains the position
        eqn and duration for each phase in the profile. The position eqns are
        given in the form of a poly1d object from the scipy package.

        [(pos, duration), (pos, duration), ... ]
        """
        return self.segments

    def print_profile(self):
        raise NotImplementedError, "Broken code."

        # TODO: Fixup, if needed.
        print "a_init", self.a_init
        print "v_init", self.v_init
        print "v_final", self.v_final
        print "max_accel", self.max_accel
        print "max_jerk", self.max_jerk
        print "mass", self.mass
        print "time\tpos\tvel\tacc\tjerk\tenergy(J)\tpower(kW)"
        t_total = 0.0
        keprev = 0.0
        while t_total < self.t_phase_one:
            ke = round(0.5 * self.mass * self.phase_one_eqns[VEL](t_total) * self.phase_one_eqns[VEL](t_total),1)
            pwr = round((ke - keprev) / self.step/1000,1)
            print "%s\t%s\t%s\t%s\t%s\t%s\t%s" % \
                  (t_total, \
                  round(self.phase_one_eqns[POS](t_total), 2), \
                  round(self.phase_one_eqns[VEL](t_total), 2), \
                  round(self.phase_one_eqns[ACCEL](t_total), 2), \
                  self.phase_one_eqns[JERK](t_total) ,ke, pwr)
            t_total += self.step
            keprev = ke

        while t_total >= self.t_phase_one and \
              t_total < self.t_phase_one + self.t_phase_two:
            t_region = t_total - self.t_phase_one
            ke = round(0.5 * self.mass * self.phase_two_eqns[VEL](t_region)
                       * self.phase_two_eqns[VEL](t_region),1)
            pwr = round((ke - keprev) / self.step/1000,1)
            print "%s\t%s\t%s\t%s\t%s\t%s\t%s" % \
                  (t_total, \
                  round(self.phase_two_eqns[POS](t_region), 2), \
                  round(self.phase_two_eqns[VEL](t_region), 2), \
                  round(self.phase_two_eqns[ACCEL](t_region), 2), \
                  self.phase_two_eqns[JERK](t_region) ,ke, pwr)
            t_total += self.step
            keprev = ke

        while t_total >= self.t_phase_one + self.t_phase_two and \
              t_total  < self.t_final:
            t_region = t_total - self.t_phase_one - self.t_phase_two
            ke = round(0.5 * self.mass * self.phase_three_eqns[VEL](t_region) *
                       self.phase_three_eqns[VEL](t_region),1)
            pwr = round((ke - keprev) / self.step/1000,1)
            print "%s\t%s\t%s\t%s\t%s\t%s\t%s" % \
                  (t_total, \
                  round(self.phase_three_eqns[POS](t_region), 2), \
                  round(self.phase_three_eqns[VEL](t_region), 2), \
                  round(self.phase_three_eqns[ACCEL](t_region), 2), \
                  self.phase_three_eqns[JERK](t_region) ,ke, pwr)
            t_total += self.step
            keprev = ke

# test run
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 8:
        print "Usage: python %s v_init v_final max_accel max_decel max_jerk a_init mass " % sys.argv[0]
        print "  max_decel should be provided as a negative number."
        sys.exit()

    prof = SpeedProfiler(v_init=float(sys.argv[1]), v_final=float(sys.argv[2]),
                         max_accel=float(sys.argv[3]), max_decel=float(sys.argv[4]),
                         max_jerk=float(sys.argv[5]),
                         a_init=float(sys.argv[6]), mass=float(sys.argv[7]))

    prof.print_eqns()
    prof.plot_profile()
