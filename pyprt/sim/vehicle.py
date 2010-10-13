# '/' is true division, '//' is truncating division
from __future__ import division

import logging
import warnings
import heapq
import itertools

from numpy import inf
import numpy
import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.table_column as ui_tc
from enthought.traits.ui.tabular_adapter import TabularAdapter
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import pyprt.shared.utility as utility
from pyprt.shared.utility import pairwise
import common
import pyprt.shared.cubic_spline as cspline
from visual import NoWritebackOnCloseHandler

def create_vehicle_class(
        model_name, length, vehicle_mass, max_pax_capacity,
        frontal_area, drag_coefficient, rolling_coefficient,
        powertrain_efficiency, regenerative_braking_efficiency,
        jerk_max_norm, jerk_min_norm, jerk_max_emerg, jerk_min_emerg,
        accel_max_norm, accel_min_norm, accel_max_emerg, accel_min_emerg,
        vel_max_norm, vel_min_norm, vel_max_emerg, vel_min_emerg):
    """Creates a new vehicle class, which inherits from BaseVehicle, and which
    has the function parameters as class level attributes.
    """

    # Originally implemented as Traits, but that made the attributes only
    # accessible from an instance, not from the class.
    class_dict = {
                 'model_name':model_name,
                 'length':length,
                 'vehicle_mass':vehicle_mass,
                 'max_pax_capacity':max_pax_capacity,
                 'frontal_area':frontal_area,
                 'drag_coefficient':drag_coefficient,
                 'rolling_coefficient':rolling_coefficient,
                 'powertrain_efficiency':powertrain_efficiency,
                 'regenerative_braking_efficiency':regenerative_braking_efficiency,
                 'jerk_max_norm':jerk_max_norm,
                 'jerk_min_norm':jerk_min_norm,
                 'jerk_max_emerg':jerk_max_emerg,
                 'jerk_min_emerg':jerk_min_emerg,
                 'accel_max_norm':accel_max_norm,
                 'accel_min_norm':accel_min_norm,
                 'accel_max_emerg':accel_max_emerg,
                 'accel_min_emerg':accel_min_emerg,
                 'vel_max_norm':vel_max_norm,
                 'vel_min_norm':vel_min_norm,
                 'vel_max_emerg':vel_max_emerg,
                 'vel_min_emerg':vel_min_emerg
                }

    return type(model_name, (BaseVehicle,), class_dict)

class BaseVehicle(Sim.Process, traits.HasTraits):
    """SimPy has several important features/limitations which have affected the
    structure of this class:

    1. An instance of a 'Sim.Process' has only ONE function that it runs. This
    function is essentially a main loop. For the Vehicle class, my main loop
    function is called 'ctrl_loop'. To be more precise, 'ctrl_loop' is not a
    function, it's a generator. Every time a yield expression is reached
    control is handed back to SimPy along with some information. Only the
    'main loop' generator communicates with SimPy, so only it may use 'yield'
    to pass info to SimPy.

##    2. To allow asychronous events, I create additional Sim.Process's whose
##    sole purpose is to wake up at some point in the future and interrupt the
##    vehicle. SimPy interrupts should be thought of as, "Notifying the process
##    that it should interrupt itself." That is, it is vehicle object that
##    must acknowledge and handle the interrupt -- not the interrupting object.

    """
    # The CType indicates that the Type can be coerced from a string.
    ID              = traits.CInt     # Unique numeric ID.
    label           = traits.Str
    passenger_mass  = traits.CInt   # total mass of passengers and luggage, in kg
    max_pax_capacity = traits.CInt
    _passengers       = traits.List(traits.Instance('events.Passenger'))

    # Action consts
    BOUNDARY = 1
    TAIL_RELEASE = 2
    COLLISION = 3
    ENTER_BERTH = 4
    EXIT_BERTH = 5
    NOTIFY_POSITION = 6
    STOPPED = 7
    SPEEDING = 8

    # A default view for vehicles.
    trait_view =  ui.View('ID', 'length',
                          ui.Item(name='loc', label='Location'),
                          ui.Item(name='pos', label='Position'),
                          ui.Item(name='vel'),
                          ui.Item(name='passenger_mass'),
                          ui.Item(name='total_mass'),
                          ui.Item(name='max_pax_capacity', label='Max. Passenger Capacity'),
                          ui.Item('_passengers@',
                                  editor = ui.ListEditor( use_notebook = True,
                                                          deletable    = False,
                                                          export       = 'DockShellWindow',
                                                          page_name    = '.label' )),
                          title='Vehicle Stats',
                          kind='nonmodal',
                          style='readonly',
                          resizable=True,
                          handler=NoWritebackOnCloseHandler()
                          )

    table_editor = ui.TableEditor(
        columns = [ui_tc.ObjectColumn(name='ID', label='ID', tooltip='Vehicle ID'),
                   ui_tc.ObjectColumn(name='loc', label='Location', tooltip='Current Location'),
                   ui_tc.ObjectColumn(name='pos', label='Position', format='%.3f',
                                      tooltip='Current Position (meters)'),
                   ui_tc.ObjectColumn(name='vel', label='Velocity', format='%.3f',
                                      tooltip='Current Velocity (m/s)'),
                   ui_tc.ObjectColumn(name='accel', label='Accel', format='%.3f',
                                      tooltip='Current Acceleration (m/s^2)'),
                   ui_tc.ExpressionColumn(label='# Pax',
                                          expression='len(object._passengers)',
                                          tooltip='Current number of passengers'),
                   ui_tc.ExpressionColumn(label='Max Pax',
                                          expression='object.get_max_pax()',
                                          tooltip='Most passengers in vehicle'),
                   ui_tc.ObjectColumn(name='max_pax_capacity', label='Capacity',
                                      tooltip='Passenger capacity'),
                   ui_tc.ObjectColumn(name='total_pax', label='Total Pax',
                                      tooltip='Total number of passengers carried'),
                   ui_tc.ObjectColumn(name='dist_travelled', label='Dist Travelled',
                                      format='%d', tooltip='Distance travelled (meters)'),
                   ui_tc.ExpressionColumn(label='Empty Dist', format='%d',
                                          expression='object.get_empty_dist()',
                                          tooltip='Distance travelled with no passengers (meters)'),
                   ui_tc.ExpressionColumn(label='Passenger Meters', format='%d',
                                          expression='object.get_pax_dist()',
                                          tooltip='Distance travelled * passengers carried'),
                   ui_tc.ExpressionColumn(label='Time-Weighted Ave Pax', format='%.2f',
                                          expression='object.get_time_ave_pax()',
                                          tooltip='Average number of passengers, weighted by time'),
                   ui_tc.ExpressionColumn(label='Dist-Weighted Ave Pax', format='%.2f',
                                          expression='object.get_dist_ave_pax()',
                                          tooltip='Average number of passengers, weighted by distance')
                   ],
        other_columns = [ui_tc.ObjectColumn(name='vehicle_mass', label='Vehicle Mass',
                                            tooltip='Excludes passenger or cargo weight (kg)'),
                         ui_tc.ObjectColumn(name='passenger_mass', label='Passenger Mass',
                                            tooltip="Includes passengers' luggage (kg)"),
                         ui_tc.ObjectColumn(name='total_mass', label='Total Mass',
                                            tooltip='(kg)')
                         ],
        deletable = False,
        editable=False,
        sortable = True,
        sort_model = False,
        auto_size = True,
        orientation = 'vertical',
        show_toolbar = True,
        reorderable = False,
        rows = 15,
        row_factory = traits.This)



    def __init__(self, ID, loc, position, vel, label="", **tr):
        assert not isinstance(position, basestring)
        assert not isinstance(vel, basestring)
        assert position <= loc.length
        Sim.Process.__init__(self, name='vehicle'+str(ID))
        traits.HasTraits.__init__(self, **tr)

        self.ID = ID
        self.label = (label if label else str(ID)+'_'+self.__class__.__name__)
        self.door = 0 # 0=closed, 1=open

        self._path = [loc]
        self._path_idx_nose = 0   # index of current location on the path
        self._path_idx_tail = 0

        offset = 0
        if position < self.length: # tail is on a previous location
            # work back from nose's position, extending path, until tail loc is found
            dist_remaining = self.length - position
            while dist_remaining > 0:
                prev_loc = common.digraph.predecessors(self._path[-1])[0] # arbitrarily choose prev loc if downstream of a merge
                self._path.append(prev_loc)
                offset += prev_loc.length
                dist_remaining -= prev_loc.length
            self._path.reverse() # path built up in reverse, since there's no appendleft. Reverse to correct this
            self._path_idx_nose = len(self._path)-1

        self._pos_offset_nose = 0 # sum of all prior location lengths
        self._pos_offset_tail = self.length - offset

        # Place myself in locations' vehicle lists. Ensure that position relative
        # to other vehicles is correct.
        if len(self._path) > 1:
            for l in self._path[:-1]:
                l.vehicles.insert(0, self) # vehicle continues onto next track, therefore it must be at the head of the line
            self._path[-1].vehicles.append(self) # vehicle must be at the end of the line on the nose's TrackSegment.
        else: # compare my position to other's positions, and insert for correct ordering
            inserted = False
            for idx, other_v in enumerate(loc.vehicles): # iterate, starting with frontmost vehicle
                if position > other_v.pos: # I'm ahead, so cut in front
                    loc.vehicles.insert(idx, self)
                    inserted = True
                    break
            if not inserted:
                loc.vehicles.append(self)

        sim_end_time = common.config_manager.get_sim_end_time()
        self._spline = cspline.CubicSpline([position, position+vel*sim_end_time],
                                           [vel, vel],
                                           [0,0],
                                           [0],
                                           [Sim.now(), sim_end_time])

        self._actions_queue = [] # contains 3-tuples: (time, action, data)

        self.total_pax = 0
        self._pax_times = [(Sim.now(),0)] # elements are (time, num_pax)
        self._operational_times = [(Sim.now(),True)] # Elements are (time, not_in_storage)
        self._total_masses = [ (Sim.now(), self.vehicle_mass) ]

    def __str__(self):
        return 'vehicle' + str(self.ID)

    def __cmp__(self,other):
        if isinstance(other, BaseVehicle): # includes subclasses of Vehicle
            return cmp(self.ID, other.ID)
        else:
            return cmp(id(self.ID), id(other))

    def __hash__(self):
        return self.ID.__hash__()

    def get_passengers(self):
        return self._passengers[:]
    passengers = property(fget=get_passengers)

    def embark(self, pax):
        assert pax not in self._passengers
        self._passengers.append(pax)
        self.passenger_mass += pax.mass
        self.total_mass += pax.mass
        self.total_pax += 1
        self._pax_times.append( (Sim.now(), len(self._passengers)) )

    def disembark(self, pax):
        self._passengers.remove(pax)
        self.passenger_mass -= pax.mass
        self.total_mass -= pax.mass
        self._pax_times.append( (Sim.now(), len(self._passengers)) )

    def get_pax_count(self):
        """Return the current number of the passengers"""
        return len(self._passengers)
    pax_count = property(get_pax_count)

    def get_time_ave_pax(self):
        """Returns the time-weighted average number of passengers onboard the
        vehicle from the start until now."""
        op_time = self.get_operational_time()
        if op_time != 0: # guard against zero division
            return len(self._passengers)

        ave = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            ave += (t_f - t_i) * cnt_i

        t_final, cnt_final = self._pax_times[-1]
        ave += (Sim.now() - t_final) * cnt_final
        return ave / op_time
    time_ave_pax = property(get_time_ave_pax)

    def get_dist_ave_pax(self):
        """Returns the dist-weighted average number of passengers onboard the
        vehicle from start until now."""
        ave = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            ave += (self._spline.evaluate(t_f).pos - self._spline.evaluate(t_i).pos) * cnt_i

        t_final, cnt_final = self._pax_times[-1]
        ave += (self._spline.evaluate(Sim.now()).pos - self._spline.evaluate(t_final).pos) * cnt_final

        try:
            result = ave/self.get_dist_travelled()
        except ZeroDivisionError:
            result = len(self._passengers)
        return result
    dist_ave_pax = property(get_dist_ave_pax)

    def get_max_pax(self):
        """Returns the max number of simultaneous passengers. Not the max
        passenger capacity!"""
        return max(pax_count for (time, pax_count) in self._pax_times)
    max_pax = property(get_max_pax)

    def get_pos(self, time=None):
        """The vehicle's nose position, in meters, where the start of the current TrackSegment is 0."""
        if time is None:
            time = Sim.now()
        pos = self._spline.evaluate(time).pos - self._pos_offset_nose
        assert pos > -1.5 # position shouldn't be more than a little bit negative
        assert pos <= self.loc.length + 1 # very loose sanity check
        return pos
    pos = property(fget = get_pos)

    def get_loc(self):
        """The current TrackSegment of the vehicle's nose."""
        return self._path[self._path_idx_nose]
    loc = property(fget = get_loc)

    def get_positions(self):
        """Returns the nose and the tail positions as a pair. More efficient than
        finding each separately. Note that the nose and tail may be on different
        TrackSegments, and thus have different coordinate frames."""
        spline_pos = self._spline.evaluate(Sim.now()).pos
        pos = spline_pos - self._pos_offset_nose
        tail_pos = spline_pos - self._pos_offset_tail
        assert pos > -1.5 # position shouldn't be more than a little bit negative
        assert tail_pos > -1.5
        return (pos, tail_pos)

    def get_tail_pos(self):
        """The vehicle's tail position, in meters, where the start of the current TrackSegment is 0."""
        tail_pos = self._spline.evaluate(Sim.now()).pos - self._pos_offset_tail
        if __debug__ and tail_pos < -0.001:
            logging.debug("t:%.4f v:%d, Neg tail pos: %.3f, spline pos:%.3f, self._pos_offset_tail: %.3f, _spline: %s",
                          Sim.now(), self.ID, tail_pos, self._spline.evaluate(Sim.now()).pos, self._pos_offset_tail, str(self._spline))
        assert tail_pos > -1.5, tail_pos # position shouldn't be more than a little bit negative
        assert tail_pos <= self.tail_loc.length + 1 # very loose sanity check
        return tail_pos
    tail_pos = property(fget=get_tail_pos)

    def get_tail_loc(self):
        return self._path[self._path_idx_tail]
    tail_loc = property(fget=get_tail_loc)

    def get_vel(self, time=None):
        if time is None:
            time = Sim.now()
        return self._spline.evaluate(time).vel

    vel = property(fget=get_vel)

    def get_accel(self, time=None):
        if time is None:
            time = Sim.now()
        return self._spline.evaluate(time).accel

    accel = property(fget=get_accel)

    def get_total_masses(self, times):
        """Returns the vehicle's total mass (incl. passengers) at times."""
        masses = []
        old_mass = 0
        sample_times = iter(times)
        for time, mass in self._total_masses + [ (Sim.now(), self._total_masses[-1][1]) ]:
            try:
                while True:
                    sample_time = sample_times.next()
                    if sample_time < time:
                        masses.append(old_mass)
                    else:
                        break

                masses.append(mass)
                old_mass = mass
            except StopIteration:
                break
        assert len(masses) == len(times)
        return masses

    def get_total_mass(self):
        """Returns the current total mass, calculated as
          vehicle mass + passenger(s) mass
        """
        return self._total_masses[-1][1]
    def set_total_mass(self, mass):
        assert mass >= self.vehicle_mass
        self._total_masses.append( (Sim.now(), mass) )
    total_mass = property(fget=get_total_mass, fset=set_total_mass)

    def get_dist_travelled(self):
        return self._spline.evaluate(Sim.now()).pos - self._spline.evaluate(self._spline.t[0]).pos
    dist_travelled = property(fget=get_dist_travelled)

    def get_empty_dist(self):
        """Returns the deadhead distance, in m."""
        dist = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            if cnt_i == 0:
                dist += self._spline.evaluate(t_f).pos - self._spline.evaluate(t_i).pos

        t_final, cnt_final = self._pax_times[-1]
        if cnt_final == 0:
            dist += self._spline.evaluate(Sim.now()).pos - \
                 self._spline.evaluate(t_final).pos
        return dist
    empty_dist = property(get_empty_dist)

    def get_nonempty_dist(self):
        """Returns the distance travelled with one or more passengers on board,
        measured in meters."""
        return self.get_dist_travelled() - self.get_empty_dist()
    nonempty_dist = property(get_nonempty_dist)

    def get_pax_dist(self):
        """Returns the number of passenger-meters travelled."""
        dist = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            if cnt_i > 0:
                dist += (self._spline.evaluate(t_f).pos - self._spline.evaluate(t_i).pos) * cnt_i

        t_final, cnt_final = self._pax_times[-1]
        if cnt_final > 0:
            dist += self._spline.evaluate(Sim.now()).pos - \
                 self._spline.evaluate(t_final).pos * cnt_final
        return dist
    pax_dist = property(get_pax_dist)

    def get_operational_time(self):
        """Returns the total time spent in operation during the sim. That is,
        time in which the vehicle was not in storage."""
        time = 0
        for (t_i, op), (t_f, op) in pairwise(self._operational_times):
            if op:
                time += t_f - t_i

        t_final, op_final = self._operational_times[-1]
        if op_final:
            time += Sim.now() - t_final
        return time
    operational_time = property(get_operational_time)

    def get_empty_time(self):
        """Returns the operational time spent with no passengers on board."""
        time = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            if cnt_i == 0:
                time += (t_f - t_i)
        t_final, cnt_final = self._pax_times[-1]
        if cnt_final == 0:
            time += Sim.now() - t_final
        return time
    empty_time = property(get_empty_time)

    def get_nonempty_time(self):
        """Returns the operational time spent with one or more passengers on board"""
        return self.get_operational_time() - self.get_empty_time()
    nonempty_time = property(get_nonempty_time)

    def get_passenger_time(self):
        """Returns the number of passenger-seconds spent. That is, 10 seconds
        with 2 passengers on board is 20 passenger-seconds."""
        p_time = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            if cnt_i > 0:
                p_time += (t_f - t_i) * cnt_i
        t_final, cnt_final = self._pax_times[-1]
        if cnt_final > 0:
            p_time += (Sim.now() - t_final) * cnt_final
        return p_time
    passenger_time = property(get_passenger_time)

    def clear_path(self):
        """Clears the planned itinerary."""
        del self._path[self._path_idx_nose+1:]

    def extend_path(self, locs):
        """Adds new locations to the planned itinerary."""
        assert locs[0] in common.digraph.neighbors(self._path[-1])
        self._path.extend(locs)

    def _move_to(self, pos, loc):
        """Abruptly moves the vehicle to a new position and location.
        The vehicle's spline is unaffected; velocity and acceleration are
        not changed. The vehicle's future path is cleared.
        'pos' must be >= the vehicle's length.
        """
        assert pos >= self.length
        follower = self.find_following_vehicle()

        self.loc.vehicles.remove(self)

        # insert the vehicle in loc.vehicles
        for i in range(len(loc.vehicles)-1, -1, -1): # indexes in reverse order
            i_pos = loc.vehicles[i].pos
            if pos <= i_pos:
                loc.vehicles.insert(i+1, self)
                break
        else:
            loc.vehicles.insert(0, self)

        # Change the position offset to alter the position.
        self._pos_offset_nose -= (pos - self.pos)
        self._pos_offset_tail = self._pos_offset_nose + self.length

        # Add the new location to the path, and adjust the path index.
        self._path = self._path[:self._path_idx_nose+1] + [loc]
        self._path_idx_nose = len(self._path)-1
        self._path_idx_tail = len(self._path)-1
        assert self.loc is loc
        assert abs(self.pos - pos) < 0.01 # loose sanity check
        assert abs(self.tail_pos - (pos - self.length)) < 0.01 # loose sanity check

        # Notify the vehicle behind me to clear any upcoming collisions
        if follower is not None:
            self.interrupt(follower)

        # Interrupt this vehicle to allow its current hold time to be changed.
        # Leads to the action queue being cleared and repopulated.
        self.interrupt(self)

    def process_spline_msg(self, spline_msg):
        """Unpacks the contents of an api.Spline message,
        and alters the vehicle's trajectory accordingly. Only uses the jerk
        to alter the trajectory, since it's the only coefficient that
        doesn't require knowledge of the vehicle's true state. The other
        coefficients are checked against "reality" and the discrepencies are
        logged for debugging purposes."""
        spline_start_time = max(spline_msg.times[0], Sim.now())
        last_knot = self._spline.evaluate(spline_start_time)
        spline = cspline.CubicSpline([last_knot.pos],
                                     [last_knot.vel],
                                     [last_knot.accel],
                                     [],
                                     [last_knot.time])
        for poly_msg, (t_initial, t_final) in zip(spline_msg.polys, pairwise(spline_msg.times)):
            if t_final <= Sim.now(): # skip any polys that end before the current time
                continue
            if t_initial < Sim.now():
                t_initial = Sim.now()
            # calculate forward using only the duration and jerk
            if len(poly_msg.coeffs) == 4:
                jerk = poly_msg.coeffs[0]*6
            else:
                jerk = 0
            delta_t = t_final - t_initial
            delta_t__2 = delta_t * delta_t
            delta_t__3 = delta_t__2 * delta_t
            a = jerk*delta_t + last_knot.accel
            v = jerk*delta_t__2/2 + last_knot.accel*delta_t + last_knot.vel
            q = jerk*delta_t__3/6 + last_knot.accel*delta_t__2/2 + last_knot.vel*delta_t + last_knot.pos
            new_knot = cspline.Knot(q, v, a, t_final)

            # It's one thing to not trust the controller to dictate the vehicle's
            # state. But controlling the vehicle by jerk alone eventually results
            # in rounding errors. Use points in the trajectory where the vehicle
            # has clearly come to a complete stop to zero out any accumulated
            # rounding errors in the velocity and acceleration.
            # Note that this only accounts for small errors at knots. A vehicle
            # slamming to an ungraceful halt, or doing a 'stop and go' between
            # knots will be caught by self._validate_spline(), but will not
            # be corrected by this code.
            if (abs(new_knot.vel) < 1E-5 and abs(new_knot.accel) < 1E-5) or \
                                 (new_knot.vel < 0 and new_knot.accel < 0):
                logging.debug("Clipped vel and accel to 0. Vehicle %d, original knot: %s", self.ID, new_knot)
                new_knot.vel = 0
                new_knot.accel = 0

            spline.append(new_knot, jerk)
            last_knot = cspline.Knot(spline.q[-1], spline.v[-1], spline.a[-1], spline.t[-1])

            # log discrepencies between controller's estimate of pos, vel, accel
            # and the actual values at each knot.
            if __debug__:
                errors = [received-true for (received, true) in zip(reversed(poly_msg.coeffs), reversed(spline.coeffs[-1]))]
                if len(errors) == 4:
                    # Assume that the traj msg is using the vehicle's current
                    # track segment as the coordinate frame
                    errors[0] += self._pos_offset_nose
                error_str = ", ".join(msg + "%.6f"%error for (msg, error) in zip(['Pos: ', 'Vel: ', 'Accel: '], errors))
                logging.debug("Vehicle %d, spline check::Times: %s to %s. Errors: %s",
                              self.ID, t_initial, t_final, error_str)

        self._validate_spline(spline)

        # In most cases, the controller should provide a spline that is valid until
        # the end of the sim. When it doesn't, extend the spline with the current
        # velocity and warn. For the extension, the acceleration is set to zero,
        # which may be wholly inaccurate!!
        sim_end_time = common.config_manager.get_sim_end_time()
        if spline.t[-1] < sim_end_time:
            warnings.warn("Spline extended to sim_end_time by simulator.")
            vel = spline.v[-1]
            pos = spline.q[-1] + vel*(sim_end_time-spline.t[-1])
            spline.append(cspline.Knot(pos, vel, 0, sim_end_time), 0)

##            ## DEBUG
##            from pyprt.shared.cspline_plotter import CSplinePlotter
##            plotter = CSplinePlotter(spline, self.vel_max_norm, self.accel_max_norm, self.jerk_max_norm,
##                                     self.vel_min_norm, self.accel_min_norm, self.jerk_min_norm)
##            plotter.display_plot()

        ### SIDE EFFECTS ###
        old_spline = self._spline.copy_left(max(Sim.now(), spline_start_time)) # truncated copy of the old spline
        self._spline = old_spline.concat(spline)

        # Notify vehicle behind me that I changed speeds.
        follower = self.find_following_vehicle()
        if follower is not None:
            self.interrupt(follower)

        # Interrupt this vehicle to allow its current hold time to be changed.
        # Leads to the action queue being cleared and repopulated.
        self.interrupt(self)

    def _validate_spline(self, spline):
        """Raises a subclass of common.MsgRangeError if the spline exceeds
        the emergency [max|min] values. Logs a warning if the spline exceeds the
        normal [max|min] values.

        spline: A cubic_spline.CubicSpline object.
        """
        assert isinstance(spline, cspline.CubicSpline)

        # TODO: Standardize on tolerance values throughout the sim!!
        vel_tol = 1E-6
        accel_tol = 1E-6
        jerk_tol = 1E-6

        # check jerks
        max_jerk = spline.get_max_jerk()
        if max_jerk > self.jerk_max_norm + jerk_tol:
            logging.warn("Jerk exceeds normal max value. vID:%d, jerk:%f, normal:%f, spline:%s",
                         self.ID, max_jerk, self.jerk_max_norm, str(spline))
        if max_jerk > self.jerk_max_emerg + jerk_tol:
            raise common.InvalidJerk(max_jerk, (self.jerk_min_emerg, self.jerk_max_emerg))

        min_jerk = spline.get_min_jerk()
        if min_jerk < self.jerk_min_norm - jerk_tol:
            logging.warn("Jerk exceeds normal min value. vID:%d, jerk:%f, normal:%f, spline:%s",
                         self.ID, min_jerk, self.jerk_min_norm, str(spline))
        if min_jerk < self.jerk_min_emerg - jerk_tol:
            raise common.InvalidJerk(min_jerk, (self.jerk_min_emerg, self.jerk_min_emerg))

        # check accels
        max_accel = spline.get_max_acceleration()
        if max_accel > self.accel_max_norm + accel_tol:
            logging.warn("accel exceeds normal max value. vID:%d, accel:%f, normal:%f, spline:%s",
                         self.ID, max_accel, self.accel_max_norm, str(spline))
        if max_accel > self.accel_max_emerg + accel_tol:
            raise common.InvalidAccel(max_accel, (self.accel_min_emerg, self.accel_max_emerg))

        min_accel = spline.get_min_acceleration()
        if min_accel < self.accel_min_norm - accel_tol:
            logging.warn("accel exceeds normal min value. vID:%d, accel:%f, normal:%f, spline:%s",
                         self.ID, min_accel, self.accel_min_norm, str(spline))
        if min_accel < self.accel_min_emerg - accel_tol:
            raise common.InvalidAccel(min_accel, (self.accel_min_emerg, self.accel_min_emerg))

        # check velocities. Only checks against vehicle velocity limits, not
        # track speed limits.
        extrema_velocities, extrema_times = spline.get_extrema_velocities()
        max_vel = max(extrema_velocities)
        if max_vel > self.vel_max_norm + vel_tol:
            logging.warn("vel exceeds normal max value. vID:%d, vel:%f, normal:%f, spline:%s",
                         self.ID, max_vel, self.vel_max_norm, str(spline))
        if max_vel > self.vel_max_emerg + vel_tol:
            raise common.InvalidVel(max_vel, (self.vel_min_emerg, self.vel_max_emerg))

        min_vel = min(extrema_velocities)
        if min_vel < self.vel_min_norm - vel_tol:
            logging.warn("vel exceeds normal min value. vID:%d, vel:%f, normal:%f, spline:%s",
                         self.ID, min_vel, self.vel_min_norm, str(spline))
        if min_vel < self.vel_min_emerg - vel_tol:
            raise common.InvalidVel(min_vel, (self.vel_min_emerg, self.vel_min_emerg))

    def is_parked_between(self, min_tail_pos, max_nose_pos, track_seg):
        """"Check that the vehicle is stopped, and is located on track_seg with
        the vehicle's nose somewhere before max_nose_pos and the tail somewhere
        beyond min_tail_pos."""
        if abs(self.vel) > 1E-6:
            return False
        elif self.loc is not track_seg:
            return False
        elif self.pos > max_nose_pos or self.tail_pos < min_tail_pos:
            return False
        else:
            return True

    def notify_position(self, ctrl_setnotify_msg, msg_id):
        t = self._spline.get_time_from_dist(ctrl_setnotify_msg.pos - self.pos, Sim.now())
        assert t is not None and t >= Sim.now()
        heapq.heappush(self._actions_queue, (t, self.NOTIFY_POSITION, (ctrl_setnotify_msg, msg_id)))

    def ctrl_loop(self):
        self._traverse()
        self._collision_check()
        self._add_tail_release()

        # main loop
        while True:
            delay = self._actions_queue[0][0] - Sim.now()
            assert delay >= 0, delay
            yield Sim.hold, self, delay

            # Check for interruption(s) and handle
            while self.interrupted():
                # Note: If more than one interruption occurs, the older interruption
                #       never gets handled. So, this needs to handle the interruption
                #       as though every possible interruption could have occurred.
                #       Assumes trajectory was changed.
                old_actions_queue = self._actions_queue
                self._actions_queue = []
                self._traverse()
                self._collision_check()
                self._add_tail_release()

                for time, action, data in old_actions_queue:
                    if action == self.NOTIFY_POSITION:
                        self.notify_position(data[0], data[1])
                    elif action == self.STOPPED:
                        # If the vehicle is currently at the stop position, the
                        # STOPPED event is skipped. This is to prevent a stop
                        # from being ommitted due to an interrupt.
                        if abs(self._spline.evaluate(time).pos - self.pos) <= 1E-4:
                            self._actions_queue.append( (time, action, data) )
                            self._actions_queue.sort()

                delay = self._actions_queue[0][0] - Sim.now()
                assert delay >= 0, delay
                self.interruptReset()
                yield Sim.hold, self, delay

            # Handle the next scheduled action
            time, action, data = heapq.heappop(self._actions_queue)
            assert utility.time_eql(time, Sim.now())
            if action == self.BOUNDARY:
                self._boundary_handler()

            elif action == self.TAIL_RELEASE:
                self._tail_release_handler()

            elif action == self.COLLISION:
                self._collision_handler(data)

            elif action == self.NOTIFY_POSITION:
                self._notify_position_handler(*data) # data is a 2-tuple

            elif action == self.STOPPED:
                self._stopped_handler()

            elif action == self.SPEEDING:
                self._speeding_handler()

            else:
                raise Exception("Unknown action type: %d for vehicle %d" % (action, self.ID))

    def crash(self, other_vehicle):
        raise NotImplementedError

    def _traverse(self):
        # Queue up next TrackSegment boundary
        traverse_dist = self.loc.length - self.pos
        assert traverse_dist >= -1E-3, (self.ID, traverse_dist, self.loc.length, self.loc.ID)

        try:
            traverse_time = self._spline.get_time_from_dist(traverse_dist, Sim.now())
        except cspline.OutOfBoundsError: # Current trajectory spline never has the vehicle leaving this TrackSegment
            traverse_time = inf
        heapq.heappush(self._actions_queue, (traverse_time, self.BOUNDARY, None))
        assert traverse_time >= Sim.now() - 1E-5, (self.ID, traverse_time, self.loc.length, self.loc.ID)

        if traverse_time != inf:
            segment_spline = self._spline.slice(Sim.now(), traverse_time)
        else:
            segment_spline = self._spline.slice(Sim.now(), common.config_manager.get_sim_end_time())
        extrema_velocties, extrema_times = segment_spline.get_extrema_velocities()
        for vel, time in itertools.izip(extrema_velocties, extrema_times):
            # Add stopped notification (if not currently stopped at the same position)
            if vel == 0 and time > Sim.now() and abs(segment_spline.evaluate(time).pos - self.pos) > 1E-4:
                heapq.heappush(self._actions_queue, (time, self.STOPPED, None))
                assert abs(segment_spline.evaluate(time).accel) < 1E-5, (self.ID, vel, time, segment_spline)

        # Detect if there's an upcoming speed limit violation
        speed_limit = min(self.loc.max_speed, self.vel_max_emerg) + 1E-4
        if max(extrema_velocties) > speed_limit:
            # Calculate times when the vehicle begins to exceed the speed limit.
            # That is, want to find the times where the 1st derivative of a poly
            # has a positive crossing of speed_limit.
            speeding_times = []
            speed_limit_poly = numpy.poly1d([speed_limit])
            if segment_spline.v[0] >= speed_limit:
                speeding_times.append(segment_spline.t[0])

            for coeffs, (ti, tf) in zip(segment_spline.coeffs, pairwise(segment_spline.t)):
                vel_poly = numpy.poly1d(coeffs).deriv()
                coeffs = numpy.polysub(vel_poly, speed_limit_poly).coeffs
                if len(coeffs) < 4:
                    coeffs = [0]*(4-len(coeffs)) + list(coeffs) # pad to 4 elements
                    assert len(coeffs) == 4
                roots = utility.real_roots(*coeffs)
                # Remove roots that are outside of the poly's valid times
                roots = [r for r in roots if r >= ti and r <= tf]
                for r in roots:
                    knot = segment_spline.evaluate(r)
                    if knot.accel > 1E-6:
                        speeding_times.append(r)

            for time in speeding_times:
                heapq.heappush(self._actions_queue, (time, self.SPEEDING, None))


##        # Do some extra work to collect statistics about berth usages if the loc is a station platform
##        platform = common.platforms.get(self.loc.ID)
##        if platform is not None:
##            my_pos = self.pos
##            for berth in platform.berths:
##                berth_enter_dist = berth.start_pos - my_pos
##                berth_exit_dist = berth.end_pos - my_pos
##                berth_enter_time = self._spline.get_time_from_dist(berth_enter_dist, Sim.now())
##                berth_exit_time = self._spline.get_time_from_dist(berth_exit_dist, Sim.now())
##                heapq.heappush(self._actions_queue, (berth_enter_time, self.ENTER_BERTH, berth))
##                heapq.heappush(self._actions_queue, (berth_exit_time, self.EXIT_BERTH, berth))

    def _boundary_handler(self):
        """Responsible for moving a vehicle to the next location when it reaches
        a TrackSegment boundary. Initiates a collision check and tail release.
        Alters the self._actions_queue.
        """
        old_loc = self.loc
        assert abs(self.pos - old_loc.length) < 1, (self.pos, old_loc.length) # loose sanity checking

        # What location will be next
        new_loc = self.get_next_loc(old_loc, self._path_idx_nose)
        if new_loc is not None:
            if len(self._path)-1 == self._path_idx_nose: # nothing planned ahead
                self._path.append(new_loc)
            # else we're following the pre-estabished itinerary.
            self._pos_offset_nose += old_loc.length
            self._path_idx_nose += 1
            new_loc.vehicles.append(self)

        else: # new_loc is None ... Vehicle just hit a dead-end, or a track switch in mid-throw.
            heapq.heappush(self._actions_queue, (Sim.now(), self.COLLISION, None)) # single-vehicle crash
            return

        # Queue up other actions ( tail release is added by _tail_release_handler )
        self._traverse()
        self._collision_check()

        # Send Arrival Msg
        notify_msg = api.SimNotifyVehicleArrive()
        notify_msg.trackID = new_loc.ID
        self.fill_VehicleStatus(notify_msg.v_status)
        notify_msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_VEHICLE_ARRIVE, notify_msg)

    def _tail_release_handler(self):
        """Responsible for moving the tail of the vehicle from one TrackSegment
        to the next, and for adding the next tail release to the _actions_queue.
        """
        assert abs(self.tail_pos - self.tail_loc.length) < 1, (self.tail_pos, self.tail_loc.length) # loose sanity checking
        old_loc = self.tail_loc
        old_loc.vehicles.remove(self) # remove myself from old TrackSegment's vehicle list
        self._pos_offset_tail += old_loc.length
        self._path_idx_tail += 1

        # Add next tail release to queue.
        self._add_tail_release()

        # Send Exit Msg (with the new tail position)
        msg = api.SimNotifyVehicleExit()
        msg.trackID = old_loc.ID
        self.fill_VehicleStatus(msg.v_status)
        msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_VEHICLE_EXIT, msg)

    def _collision_handler(self, other_vehicle):
        """Responsible for logging and handling a collision once it occurs."""
        # Single-vehicle crash.
        if other_vehicle is None:
            # log and notify controller
            logging.info("T=%4.3f %s crashed in a single vehicle accident.", Sim.now(), self.ID)
            msg = api.SimNotifyVehicleCrash()
            self.fill_VehicleStatus(msg.v_status)
            common.interface.send(api.SIM_NOTIFY_VEHICLE_CRASH, msg)

        else:
            assert isinstance(other_vehicle, BaseVehicle)
            logging.warn("T=%4.3f Vehicle %s collided with vehicle %s.", Sim.now(), self.ID, other_vehicle.ID)
            msg = api.SimNotifyVehicleCollision()
            other_vehicle.fill_VehicleStatus(msg.v1_status)
            self.fill_VehicleStatus(msg.v2_status)
            msg.time = Sim.now()
            common.interface.send(api.SIM_NOTIFY_VEHICLE_COLLISION, msg)

#        FIXME: Do SOMETHING with the vehicles afterward. Also, notify GUI of crash.
        warnings.warn("Handling of vehicles after a crash is unimplemented.")

    def _notify_position_handler(self, ctrl_setnotify_msg, msg_id):
        """Sends an api.SimNotifyPosition message."""
        assert isinstance(msg_id, int)
        assert isinstance(ctrl_setnotify_msg, api.CtrlSetnotifyVehiclePosition)
        assert utility.dist_eql(self.pos, ctrl_setnotify_msg.pos)
        sim_msg = api.SimNotifyVehiclePosition()
        sim_msg.msgID = msg_id
        sim_msg.time = Sim.now()
        self.fill_VehicleStatus(sim_msg.v_status)
        common.interface.send(api.SIM_NOTIFY_VEHICLE_POSITION, sim_msg)

    def _stopped_handler(self):
        """Sends an api.SimNotifyVehicleStopped message."""
        notify_msg = api.SimNotifyVehicleStopped()
        self.fill_VehicleStatus(notify_msg.v_status)
        notify_msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_VEHICLE_STOPPED, notify_msg)

    def _speeding_handler(self):
        """Sends an api.SimNotifyVehicleSpeeding message."""
        speed_limit = min(self.loc.max_speed, self.vel_max_emerg)
        notify_msg = api.SimNotifyVehicleSpeeding()
        self.fill_VehicleStatus(notify_msg.v_status)
        notify_msg.time = Sim.now()
        notify_msg.speed_limit = speed_limit
        common.interface.send(api.SIM_NOTIFY_VEHICLE_SPEEDING, notify_msg)
        logging.warn("T=%4.3f Vehicle: %d exceeded speed limit: %f on track seg: %d", Sim.now(), self.ID, speed_limit, self.loc.ID)

    def _collision_check(self):
        lv, dist = self.find_leading_vehicle(current_loc_only=True)
        if dist <= 0: # just crashed.
            heapq.heappush( self._actions_queue, (Sim.now(), self.COLLISION, lv) )

        if lv:
            boundary_time = common.config_manager.get_sim_end_time()
            for time, action, data in self._actions_queue:
                if action == self.BOUNDARY:
                    boundary_time = time
                    break

            t = self._spline.find_intersection(lv._spline, Sim.now(), boundary_time, dist)
            if t is not None:
                heapq.heappush( self._actions_queue, (t, self.COLLISION, lv) )

    def _add_tail_release(self):
        if __debug__:
            # Check that the tail isn't already scheduled to be released.
            for time, action, data in self._actions_queue:
                assert action != self.TAIL_RELEASE, (self.ID, time, action, data)

        # Add next tail release to queue.
        traverse_dist = self.tail_loc.length - self.tail_pos
        try:
            traverse_time = self._spline.get_time_from_dist(traverse_dist, Sim.now())
            heapq.heappush(self._actions_queue, (traverse_time, self.TAIL_RELEASE, None))
        except cspline.OutOfBoundsError: # Current trajectory spline never has the vehicle's tail leaving this TrackSegment
            pass

    def find_leading_vehicle(self, current_loc_only=False, max_dist=500):
        """Returns the the vehicle ahead of me, and the distance between its
        tail and my nose. Only looks on self's loc if current_loc_only is True,
        and only looks as far ahead as max_dist, if specified.

        Warning: If the track-switches are configured such that a loop is
                 formed, then a max_dist of 'inf' may cause the program to hang.
        """
        loc = self.loc
        lv = None
        dist = max_dist
        # Locate myself in the TrackSegment's vehicle list.
        # Used to determine relative positioning of vehicles.
        my_idx = loc.vehicles.index(self)
        if my_idx != 0: # I'm not the leading vehicle on the edge
            lv = self.loc.vehicles[my_idx-1] # lead vehicle
            if lv.tail_loc is loc: # normal case - lv's tail is on self's loc
                dist = lv.tail_pos - self.pos
            else: # bad case - this vehicle is somehow in the middle of lv. If not a bug, then a sideswipe.
                dist = 0
        else:
            lv = None

        if lv is None and not current_loc_only:
            idx = self._path_idx_nose
            dist = loc.length - self.pos
            while not lv and dist <= max_dist:
                loc = self.get_next_loc(loc, idx)
                if loc is None:
                    break
                if loc.vehicles:
                    lv = loc.vehicles[-1]
                    dist += lv.tail_pos
                else:
                    idx += 1
                    dist += loc.length

        # didn't find a vehicle in range, or looked all the way around a circle
        if dist > max_dist or lv is self or lv is None:
            lv = None
            dist = max_dist

        return lv, dist

    def find_following_vehicle(self):
        """Similar to 'find_leading_vehicle' except it only ever looks on the
        tail's current loc, has no max_dist, and only returns the vehicle, not
        their separation distance. In other words, not too similiar at all..."""
        loc_vehicles = self.tail_loc.vehicles
        my_idx = loc_vehicles.index(self)
        if my_idx != len(loc_vehicles)-1: # if I'm not the very last vehicle
            return loc_vehicles[my_idx+1]
        else:
            return None

    def get_next_loc(self, old_loc=None, old_loc_idx=None):
        """Returns the location following old_loc. Only helpful to supply
        old_loc_idx if using vehicle-based switching.
        """
        if old_loc_idx is not None:
            try:
                new_loc = self._path[old_loc_idx+1] # use whatever the vehicle chooses (vehicle-based switching)
            except IndexError:
                new_loc = old_loc.next_loc # do whatever the track is set to do.
        else:
            new_loc = self.loc.next_loc

        return new_loc

    def fill_VehicleStatus(self, vs):
        """Fills an api.VehicleStatus instance with current information."""
        vs.vID = self.ID
        vs.nose_pos = self.pos
        vs.nose_locID = self.loc.ID
        vs.tail_pos = self.tail_pos
        vs.tail_locID = self.tail_loc.ID
        vs.vel = self.vel
        vs.accel = self.accel
        for pax in self.passengers:
            vs.passengerIDs.append(pax.ID)

        lv, dist = self.find_leading_vehicle()
        if lv: # ommit if a lead vehicle wasn't found
            vs.lvID = lv.ID
            vs.lv_distance = dist

    def calc_energy_used(self):
        """Return the amount of energy used, in Joules"""
        pass


class VehicleTabularAdapater(TabularAdapter):
    """An adapter for table-based views of multiple vehicles."""

    columns = [
        ('ID', 'ID'),
        ('Location', 'loc'),
        ('Position', 'pos'),
        ('Velocity', 'vel'),
        ('Accel', 'accel'),
        ('# Pax', 'pax_count'),
        ('Max Pax', 'max_pax'),
        ('Capacity', 'max_pax_capacity'),
        ('Total Pax', 'total_pax'),
        ('Empty Dist', 'empty_dist'),
        ('Pax Dist', 'pax_dist'),
        ('Total Dist', 'dist_travelled'),
        ('Time-Weighted Ave Pax', 'time_ave_pax'),
        ('Dist-Weighted Ave Pax', 'dist_ave_pax'),
        ('Vehicle Mass', 'vehicle_mass')
    ]

    ID_width = traits.Float(40)

    # Formatting
    pos_format = traits.Constant('%.2f')
    vel_format = traits.Constant('%.2f')
    accel_format = traits.Constant('%.2f')
    empty_dist_format = traits.Constant('%.1f')
    pax_dist_format = traits.Constant('%.1f')
    total_dist_format = traits.Constant('%.1f')
    dist_travelled_format = traits.Constant('%.1f')
    time_ave_pax_format = traits.Constant('%.1f')
    dist_ave_pax_format = traits.Constant('%.1f')

    # Tooltips
    ID_tooltip = traits.Constant('Unique vehicle identifier')
    loc_tooltip = traits.Constant('Current location')
    pos_tooltip = traits.Constant('Current distance from start of location, in meters.')
    vel_tooltip = traits.Constant('Current velocity, in m/s')
    accel_tooltip = traits.Constant('Current accel, in m/s')
    pax_count_tooltip = traits.Constant('Current number of passengers on board.')
    max_pax_tooltip = traits.Constant('Greatest number of passengers that have been on board.')
    max_pax_capacity_tooltip = traits.Constant('Passenger capacity')
    total_pax_tooltip = traits.Constant('Total number of passengers carried')
    empty_dist_tooltip = traits.Constant('Distance travelled with no passengers on board, in meters.')
    pax_dist_tooltip = traits.Constant('(Distance travelled) * (number of passengers carried)')
    dist_travelled_tooltip = traits.Constant('Total distance travelled, in meters')
    time_ave_pax_tooltip = traits.Constant('Average number of passengers, weighted by time')
    dist_ave_pax_tooltip = traits.Constant('Average number of passengers, weighted by distance')
    vehicle_mass_tooltip = traits.Constant('Vehicle weight, in kg. Excludes passenger or cargo weight.')

