# '/' is true division, '//' is truncating division
from __future__ import division

import logging
import warnings
import heapq

from numpy import inf
import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.table_column as ui_tc
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import pyprt.shared.utility as utility
from pyprt.shared.utility import pairwise
import common
import pyprt.shared.cubic_spline as cspline
from visual import NoWritebackOnCloseHandler

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
    length          = traits.CFloat # Length of vehicle, in meters.
    v_mass          = traits.CInt   # mass of vehicle, in kg
    passenger_mass  = traits.CInt   # total mass of passengers and luggage, in kg
    total_mass      = traits.CInt   # mass of vehicle + passenger mass, in kg
    max_pax_capacity = traits.CInt
    _passengers       = traits.List(traits.Instance('events.Passenger'))

    # Action consts
    BOUNDARY = 1
    TAIL_RELEASE = 2
    COLLISION = 3
    ENTER_BERTH = 4
    EXIT_BERTH = 5
    NOTIFY_POSITION = 6

    # A default view for vehicles.
    trait_view =  ui.View('ID', 'length',
                           ui.Item(name='pos', label='Position'),
                           ui.Item(name='vel'),
                           ui.Item(name='v_mass'),
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
        other_columns = [ui_tc.ObjectColumn(name='v_mass', label='Vehicle Mass',
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
        self._pax_times = [(0,0)] # elements are (time, num_pax)

##        self.emergency = False
##        self.disabled = False

        # attributes: max_speed, maxG, maxlatG
        # forces: gravity (if z), drag, friction, braking, centripital, thrust

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

    def get_time_ave_pax(self):
        """Returns the time-weighted average number of passengers onboard the
        vehicle from the start until now."""
        if Sim.now() == 0: # guard against ZeroDivision
            return len(self._passengers)

        ave = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            ave += (t_f - t_i) * cnt_i

        t_final, cnt_final = self._pax_times[-1]
        ave += (Sim.now() - t_final) * cnt_final
        return ave / Sim.now()

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

    def get_max_pax(self):
        """Returns the max number of simultaneous passengers. Not the max
        passenger capacity!"""
        return max(pax_count for (time, pax_count) in self._pax_times)

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

    def get_dist_travelled(self):
        return self._spline.evaluate(Sim.now()).pos - self._spline.evaluate(0.0).pos
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

    def get_pax_dist(self):
        """Returns the passenger-meters travelled."""
        dist = 0
        for (t_i, cnt_i), (t_f, cnt_f) in pairwise(self._pax_times):
            if cnt_i > 0:
                dist += (self._spline.evaluate(t_f).pos - self._spline.evaluate(t_i).pos) * cnt_i

        t_final, cnt_final = self._pax_times[-1]
        if cnt_final > 0:
            dist += self._spline.evaluate(Sim.now()).pos - \
                    self._spline.evaluate(t_final).pos * cnt_final
        return dist

    def clear_path(self):
        """Clears the planned itinerary."""
        del self._path[self._path_idx_nose+1:]

    def extend_path(self, locs):
        """Adds new locations to the planned itinerary."""
        assert locs[0] in common.digraph.neighbors(self._path[-1])
        self._path.extend(locs)

    def process_spline_msg(self, spline_msg):
        """Unpacks the contents of an api.Spline message,
        and alters the vehicle's trajectory accordingly. Only uses the jerk
        to alter the trajectory, since it's the only coefficient that
        doesn't require knowledge of the vehicle's true state. The other
        coefficients are checked against "reality" and the discrepencies are
        logged for debugging purposes."""
        t_initial = max(spline_msg.times[0], Sim.now())
        self._spline = self._spline.copy_left(t_initial)
        for poly_msg, (t_initial, t_final) in zip(spline_msg.polys, pairwise(spline_msg.times)):
            if t_final <= Sim.now(): # can occur if the spline_msg has some old info
                continue
            t_initial = max(t_initial, Sim.now())
            last_knot = cspline.Knot(self._spline.q[-1], self._spline.v[-1], self._spline.a[-1], self._spline.t[-1])

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

            # It's one thing to not trust the controller to dictate the vehicle's
            # state. But controlling the vehicle by jerk alone eventually results
            # in rounding errors. Use points in the trajectory where the vehicle
            # has clearly come to a complete stop to zero out any accumulated
            # rounding errors in the velocity and acceleration.
            new_knot = cspline.Knot(q, v, a, t_final)
##            if (abs(new_knot.vel) < 0.001 and abs(new_knot.accel) < 0.001) or \
##                                 (new_knot.vel < 0 and new_knot.accel < 0):
##                logging.debug("Clipped vel and accel to 0. Vehicle %d, original knot: %s", self.ID, new_knot)
##                new_knot.vel = 0
##                new_knot.accel = 0
##                # TODO: Send NOTIFY_STOPPED_MSG

            self._spline.append(new_knot, jerk)

            if __debug__:
                errors = [received-true for (received, true) in zip(reversed(poly_msg.coeffs), reversed(self._spline.coeffs[-1]))]
                if len(errors) == 4:
                    errors[0] += self._pos_offset_nose # Assuming that the traj msg sent in current position coordinate frame
                error_str = ", ".join(msg + str(error) for (msg, error) in zip(['Pos: ', 'Vel: ', 'Accel: '], errors))
                logging.debug("Vehicle %d, spline check::Times: %s to %s. Errors: %s",
                              self.ID, t_initial, t_final, error_str)

        # In most cases, the controller should provide a spline that is valid until
        # the end of the sim. When it doesn't, extend the spline with the current
        # velocity and warn. For the extension, the acceleration is set to zero,
        # which may be wholly inaccurate!!
        sim_end_time = common.config_manager.get_sim_end_time()
        if self._spline.t[-1] < sim_end_time:
            warnings.warn("Spline extended to sim_end_time by simulator.")
            vel = self._spline.v[-1]
            pos = self._spline.q[-1] + vel*(sim_end_time-self._spline.t[-1])
            self._spline.append(cspline.Knot(pos, vel, 0, sim_end_time), 0)

##            ## DEBUG
##            from pyprt.shared.cspline_plotter import CSplinePlotter
##            plotter = CSplinePlotter(self._spline, self.vel_max_norm, self.accel_max_norm, self.jerk_max_norm,
##                                     self.vel_min_norm, self.accel_min_norm, self.jerk_min_norm)
##            plotter.display_plot()

        # Notify vehicle behind me that I changed speeds.
        follower = self.find_following_vehicle()
        if follower is not None:
            self.interrupt(follower)

        # Interrupt this vehicle to allow it's current hold time to be changed.
        # Leads to the action queue being cleared and repopulated.
        self.interrupt(self)

    def is_parked_between(self, min_tail_pos, max_nose_pos, track_seg):
        """"Check that the vehicle is stopped, and is located on track_seg with
        the vehicle's nose somewhere before max_nose_pos and the tail somewhere
        beyond min_tail_pos."""
        if abs(self.vel) > 0.01:
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
                #       in the most generic way. Assume trajectory was changed.
                old_actions_queue = self._actions_queue
                self._actions_queue = []
                self._traverse()
                self._collision_check()
                self._add_tail_release()

                for time, action, data in old_actions_queue:
                    if action == self.NOTIFY_POSITION:
                        self.notify_position(data[0], data[1])

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

            else:
                raise Exception("Unknown action type: %d for vehicle %d" % (action, self.ID))

    def crash(self, other_vehicle):
        raise NotImplementedError

    def _traverse(self):
        # Queue up next TrackSegment boundary
        traverse_dist = self.loc.length - self.pos
        try:
            traverse_time = self._spline.get_time_from_dist(traverse_dist, Sim.now())
        except cspline.OutOfBoundsError: # Current trajectory spline never has the vehicle leaving this TrackSegment
##            traverse_time = common.config_manager.get_sim_end_time()
            traverse_time = inf
        heapq.heappush(self._actions_queue, (traverse_time, self.BOUNDARY, None))
        assert traverse_dist >= -1E-3, (self.id, traverse_dist, self.loc.length, self.loc.ID)
        assert traverse_time >= Sim.now() - 1E-5, (self.id, traverse_time, self.loc.length, self.loc.ID)

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
        sim_msg.msgID = ctrl_setnotify_msg.msg_id
        sim_msg.time = Sim.now()
        self.fill_VehicleStatus(sim_msg.v_status)
        common.interface.send(api.SIM_NOTIFY_VEHICLE_POSITION, sim_msg)

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

