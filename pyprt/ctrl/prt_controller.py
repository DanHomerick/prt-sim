from __future__ import division
import optparse
from collections import defaultdict
from collections import deque
import warnings

import networkx
from numpy import arange, inf

import pyprt.shared.api_pb2 as api
from pyprt.ctrl.base_controller import BaseController
from trajectory_solver import TrajectorySolver, FatalTrajectoryError
from pyprt.shared.cubic_spline import Knot, CubicSpline, OutOfBoundsError
from pyprt.shared.utility import pairwise
##from pyprt.shared.utility import deque # extension of collections.deque which includes 'insert' method

def main():
    options_parser = optparse.OptionParser(usage="usage: %prog [options] FILE\nFILE is a path to to the scenario file.")
    options_parser.add_option("--logfile", dest="logfile", default="./ctrl.log",
                metavar="FILE", help="Log events to FILE.")
    options_parser.add_option("--comm_logfile", dest="comm_logfile", default="./ctrl_comm.log",
                metavar="FILE", help="Log communication messages to FILE.")
    options_parser.add_option("--server", dest="server", default="localhost",
                help="The IP address of the server (simulator). Default is %default")
    options_parser.add_option("-p", "--port", type="int", dest="port", default=64444,
                help="TCP Port to connect to. Default is: %default")
    options_parser.add_option("--line_speed", type="float", dest="line_speed", default=25,
                help="The cruising speed for vehicles on the main line, in m/s. Default is %default")
    options_parser.add_option("--station_speed", type="float", dest="station_speed", default=2.4,
                help="The maximum speed for vehicles on station platforms, in m/s. " + \
                "Setting too high may negatively affect computation performance. Default is %default")
    options_parser.add_option("--headway", type="float", dest="headway", default="2.0",
                help="The minimum following time for vehicles, in seconds. Measured from tip-to-tail.")
    options, args = options_parser.parse_args()

    PrtController.LINE_SPEED = options.line_speed
    PrtController.HEADWAY = options.headway
    Merge.LINE_SPEED = options.line_speed
    Merge.HEADWAY = options.headway
    Station.SPEED_LIMIT = options.station_speed

    if len(args) != 1:
        options_parser.error("Expected one argument. Received: %s" % ' '.join(args))

    ctrl = PrtController(options.logfile, options.comm_logfile, args[0])
    ctrl.connect(options.server, options.port)

class NoPaxAvailableError(Exception):
    """No passengers are available."""

class NoBerthAvailableError(Exception):
    """No station berth availaible."""

class PrtController(BaseController):

    # States (for state-machine)
    # ADVANCING means moving between berths.

    RUNNING = "RUNNING"                     # Travelling from one station to the next.
    SLOWING = "SLOWING"                     # Off the main line, slowing for station entry.
    UNLOAD_ADVANCING = "UNLOAD_ADVANCING"   # Have passengers to unload, on unload platform.
    UNLOAD_WAITING = "UNLOAD_WAITING"       # Capable of immediately disembarking passengers.
    DISEMBARKING = "DISEMBARKING"           # Parked in berth, currently unloading passengers.
    QUEUE_WAITING = "QUEUE_WAITING"         # No passengers to unload. Waiting to reach load platform.
    QUEUE_ADVANCING = "QUEUE_ADVANCING"     # No passengers to unload. May be on either the Unload or Queue platform.
    LOAD_WAITING = "LOAD_WAITING"           # Capable of immediately embarking passengers.
    LOAD_ADVANCING = "LOAD_ADVANCING"       # On the load platform.
    EMBARKING = "EMBARKING"                 # Parked in berth, loading passengers
    EXIT_WAITING = "EXIT_WAITING"           # Waiting to reach launch berth.
    EXIT_ADVANCING = "EXIT_ADVANCING"       # Moving towards launch berth.
    LAUNCH_WAITING = "LAUNCH_WAITING"       # In the launch berth. Ready to lauch at any time.
    LAUNCHING = "LAUNCHING"                 # Accelerating towards the main line.

    LINE_SPEED = None  # in meter/sec. Set in main().
    HEADWAY = None     # in sec. Measured from tip-to-tail. Set in main()
    HEARTBEAT_INTERVAL = 5.1  # in seconds. Choosen arbitrarily for testing

    def __init__(self, log_path, commlog_path, scenario_path):
        super(PrtController, self).__init__(log_path, commlog_path)
        self.scenario_path = scenario_path
        self.t_reminders = dict() # keyed by time (in integer form), values are pairs of lists

        # Manager is instantiated upon receipt of a SIM_GREETING msg.
        self.manager = None   # Manager instance

        # Meh. Not fond of this style, but it's easy. Fix if I want something braindead to work on.
        Station.controller = self
        Merge.controller = self
        Vehicle.controller = self

    def set_v_notification(self, vehicle, time):
        """Request a time notification from sim and store which vehicle the
        notification is relevant to."""
        key = int(time*1000)
        try:
            v_list, s_list, fnc_list = self.t_reminders[key]
            if vehicle not in v_list:
                v_list.append(vehicle)
        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([vehicle], [], [])

    def set_s_notification(self, station, time):
        """Request a time notification from sim and store which station the
        notification is relevant to."""
        assert isinstance(station, Station)
        key = int(time*1000)
        try:
            v_list, s_list, fnc_list = self.t_reminders[key]
            assert station not in s_list
            s_list.append(station)

        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([], [station], [])

    def set_fnc_notification(self, fnc, args, time):
        """Request that function fnc be called with arguments args when time is
        reached."""
        key = int(time*1000)
        try:
            v_list, s_list, fnc_list = self.t_reminders[key]
            fnc_list.append( (fnc, args) )
        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([], [], [ (fnc, args) ])

    def request_v_status(self, vehicle_id):
        msg = api.CtrlRequestVehicleStatus()
        msg.vID = vehicle_id
        self.send(api.CTRL_REQUEST_VEHICLE_STATUS, msg)

    def heartbeat(self, station):
        """Triggers synchronized advancement of vehicles in the station."""
        self.log.debug("t:%5.3f Station %d heartbeat.", self.current_time, station.id)
        assert isinstance(station, Station)
        for plat in (Station.LOAD_PLATFORM, Station.QUEUE_PLATFORM, Station.UNLOAD_PLATFORM):
            for v in reversed(station.reservations[plat]):
                if v is not None:
                    # If vehicle is capable of moving forward, do so
                    self.trigger_advance(v, station) # changes v.state to *_ADVANCING if able to move

                    # For the vehicle's that are stuck in their current position
                    # try to do something useful with the time.
                    if v.state is self.UNLOAD_WAITING:
                        v.state = self.DISEMBARKING
                        v.disembark(v.trip.passengers)

                    elif v.state is self.LOAD_WAITING:
                        if v.platform_id == station.LOAD_PLATFORM:
                            try:
                                v.trip = self.manager.request_trip(v)
                                v.set_path(v.trip.path)
                                v.state = self.EMBARKING
                                v.embark(v.trip.passengers)
                            except NoPaxAvailableError:
                                if station.is_launch_berth(v.berth_id, v.platform_id):
                                    # launch as soon as possible, if there
                                    # is another station with passengers waiting.
                                    v.trip = self.manager.deadhead(v, (station,))
                                    if v.trip is not None:
                                        v.set_path(v.trip.path)
                                        v.state = self.LAUNCH_WAITING
                                        self.log.info("%5.3f Vehicle %d deadheading from station %d. Going to station %d.",
                                                      self.current_time, v.id, station.id, v.trip.dest_station.id)


                    elif v.state is self.EXIT_WAITING:
                        if station.is_launch_berth(v.berth_id, v.platform_id):
                            v.state = self.LAUNCH_WAITING

                    self.request_v_status(v.id)

        # schedule the next heartbeat
        self.set_s_notification(station, self.current_time + self.HEARTBEAT_INTERVAL)

    def trigger_advance(self, vehicle, station):
        """Triggers a vehicle to advance, if able. May be called on a vehicle
        that is not ready to advance, in which case the function is a no-op.
        """
        assert isinstance(vehicle, Vehicle)
        assert isinstance(station, Station)

        # This function only changes the state. The actual movement will be
        # commanded after the sim returns current vehicle status information.

        # Exclude vehicles that have reserved berths, but aren't yet in the
        # relevant states.
        if vehicle.state not in (self.UNLOAD_WAITING, self.LOAD_WAITING, self.EXIT_WAITING, None):
            return

        assert vehicle.station is station

        # if vehicle hasn't unloaded, only try to advance as far as the last unload berth
        try:
            if vehicle.state is self.UNLOAD_WAITING:
                station.request_unload_berth(vehicle)
                vehicle.state = self.UNLOAD_ADVANCING
            elif vehicle.state is self.LOAD_WAITING:
                station.request_load_berth(vehicle)
                vehicle.state = self.LOAD_ADVANCING
            elif vehicle.state is self.EXIT_WAITING:
                station.request_launch_berth(vehicle)
                vehicle.state = self.EXIT_ADVANCING
            else:
                raise Exception("Unexpected case. Vehicle.state is: %s" % vehicle.state)
        except NoBerthAvailableError:
            return

        # ... wait for the v_status update from the sim to do the movement
        vehicle._advance_begun = False

    ### Overriden message handlers ###
    def on_SIM_GREETING(self, msg, msgID, msg_time):
        self.sim_end_time = msg.sim_end_time
        self.log.info("Sim Greeting message received. Sim end at: %f" % msg.sim_end_time)

        self.manager = Manager(self.scenario_path, msg.sim_end_time, self)

    def on_SIM_START(self, msg, msgID, msg_time):
        """This function is responsible for getting all the vehicles moving at
        the start of the simulation. It needs to deal with 6 cases:

        Case 1: Vehicle is on a station's platform, and is fully within a berth.
        Case 2: Vehicle is on a station's platform, but it not fully within a berth.
        Case 3: Vehicle is on a station's OFFRAMP or DECEL segments and must
                be given a berth reservation.
        Case 4: Vehicle is on a station's ONRAMP or ACCEL segments and must
                be merged with main-line traffic.
        Case 5: Vehicle is outside of a Merge's zone of control and must be brought
                up to LINE_SPEED.
        Case 6: Vehicle is within a Merge's zone of control, and must be brought
                up to LINE_SPEED and granted a MergeSlot.

        """
        self.log.info("Sim Start message received.")

        for station in self.manager.stations.itervalues():
            # Work from the 2nd to last berth backward, initializing vehicles in order.
            for v in reversed(station.reservations[Station.LOAD_PLATFORM]):
                if v is not None:
                    v.state = self.LOAD_WAITING

            for v in reversed(station.reservations[Station.QUEUE_PLATFORM]):
                if v is not None:
                    v.state = self.QUEUE_WAITING

            for v in reversed(station.reservations[Station.UNLOAD_PLATFORM]):
                if v is not None:
                    v.state = self.QUEUE_WAITING # Assuming vehicles don't start with passengers on board

        v_list = self.manager.vehicles.values()
        # Sort based on location, secondarily sorted by position -- both in descending order.
        v_list.sort(cmp=lambda x,y: cmp(y.ts_id, x.ts_id) if x.ts_id != y.ts_id else cmp(y.pos, x.pos))

        # Make a first pass through the vehicles, setting up the vehicles which
        # are committed to a particular station.
        for vehicle in v_list:
            assert isinstance(vehicle, Vehicle)
            # skip already handled vehicles
            if vehicle.state is not None:
                continue

            station = self.manager.track2station.get(vehicle.ts_id)

            if station is not None:
                assert isinstance(station, Station)

                # Case 3.
                # Approaching station. Taking advantage of the fact that DECEL will have
                # a higher ts_id than OFF_RAMP_I, thus vehicles on DECEL will be handled
                # first and the berth reservation ordering will be correct.
                if vehicle.ts_id in (station.ts_ids[Station.OFF_RAMP_I],
                                     station.ts_ids[Station.OFF_RAMP_II],
                                     station.ts_ids[Station.DECEL]):
                    vehicle.state = self.SLOWING

                    # Set up a trip for reaching the station's UNLOAD platform.
                    path_length, path = self.manager.get_path(vehicle.ts_id, station.ts_ids[Station.UNLOAD])
                    dist = path_length - vehicle.pos
                    vehicle.trip = Trip(station, path, tuple())

                    # Reserve a berth for the vehicle
                    station.request_load_berth(vehicle) # raises an exception if fails.
                    vehicle.station = station
                    vehicle.send_path(path)
                    vehicle.run(speed_limit=station.SPEED_LIMIT, dist=dist, final_speed=0)

                # Case 4.
                # Outbound from a station.
                # TODO: Setup a merge_slot!
                else:
                    assert vehicle.ts_id in (station.ts_ids[Station.ACCEL],
                                             station.ts_ids[Station.ON_RAMP_I],
                                             station.ts_ids[Station.ON_RAMP_II])
                    vehicle.state = self.LAUNCHING
                    vehicle._launch_begun = True
                    vehicle.station = station
                    vehicle.trip = self.manager.deadhead(vehicle)
                    vehicle.set_path(vehicle.trip.path)
                    vehicle.run(speed_limit=self.LINE_SPEED)
                    self.log.info("%5.3f Vehicle %d deadheading to station %d.",
                                  self.current_time, vehicle.id, vehicle.trip.dest_station.id)

        # Make a second pass through the vehicles, handling the remaining vehicles.
        merge2vehicles = defaultdict(list)
        for vehicle in v_list:
            if vehicle.state is not None: # has already been handled
                continue

            vehicle.state = self.RUNNING

            station = self.manager.split2station.get(vehicle.ts_id)
            if station is None:
                vehicle.trip = self.manager.deadhead(vehicle)
            else: # vehicle is on the station's "split" trackseg
                assert vehicle.ts_id == station.split
                # Don't allow a vehicle that starts on a station's 'split'
                # tracksegment to enter the station
                vehicle.trip = self.manager.deadhead(vehicle, exclude=(station, ))

            vehicle.set_path(vehicle.trip.path)

            try:
                merge = self.manager.track2merge[vehicle.ts_id]
                merge2vehicles[merge].append(vehicle)
            except KeyError:
                # Case 5.
                vehicle.run(speed_limit=self.LINE_SPEED)

        # Case 6.
        # Merge the vehicles so that they don't collide at merge point
        for m, v_list in merge2vehicles.items():
            # Redefine the function each time, using closures to easily access the
            # merge's offset values.
            def sort_by_pos(v1, v2):
                """Sort by position relative to the merge point, closest first.
                Decending order, since all positions are negative."""
                v1_pos = v1.pos + m.offsets[v1.ts_id]
                v2_pos = v2.pos + m.offsets[v2.ts_id]
                return cmp(v2_pos, v1_pos)
            v_list.sort(cmp=sort_by_pos)
            for v in v_list:
                v.run(speed_limit=self.LINE_SPEED)
                v.do_merge(m, 0.0)

        # station's heartbeats are all synchronized for now. Change that in the future?
        for station in self.manager.stations.itervalues():
            self.set_s_notification(station, self.HEARTBEAT_INTERVAL)

    def on_SIM_NOTIFY_TIME(self, msg, msgID, msg_time):
        ms_msg_time = int(msg.time*1000) # millisec integer for easy comparison
        vehicles, stations, functions = self.t_reminders[ms_msg_time]

        for vehicle in vehicles:
            if vehicle.state is self.SLOWING: # just arrived at the initially reserved berth
                if vehicle.pax:
                    vehicle.state = self.UNLOAD_ADVANCING
                else:
                    # Skip disembarking entirely
                    vehicle.state = self.QUEUE_ADVANCING
                    vehicle.trip = None

            # Note that this is a separate if/else chain than above
            if vehicle.state in (self.UNLOAD_ADVANCING, self.QUEUE_ADVANCING,
                                 self.LOAD_ADVANCING, self.EXIT_ADVANCING):
                # vehicle should be done parking or advancing. Get a status update to make sure.
                req_v_status = api.CtrlRequestVehicleStatus()
                req_v_status.vID = vehicle.id
                self.send(api.CTRL_REQUEST_VEHICLE_STATUS, req_v_status)
            elif vehicle.state is self.LAUNCH_WAITING:
                blocked_time = self.manager.is_launch_blocked(vehicle.station, vehicle, self.current_time)
                if not blocked_time:
                    vehicle.state = self.LAUNCHING
                    vehicle.run(speed_limit=self.LINE_SPEED)
                    vehicle._launch_begun = True
                    vehicle.station.release_berth(vehicle)
                    self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                                  self.current_time, vehicle.id, vehicle.station.id)
                    # Note: berth is released when tail of vehicle exits the platform.
                    #
                else:
                    self.set_v_notification(vehicle, blocked_time)
                    self.log.info("t:%5.3f Delaying launch of vehicle %d from station %d until %.3f (%.3f delay)",
                                  self.current_time, vehicle.id, vehicle.station.id, blocked_time, blocked_time-self.current_time)

            else:
                warnings.warn("Huh? What was this reminder for again? vehicle %d, state: %s" % (vehicle.id, vehicle.state))
##                raise Exception("Huh? What was this reminder for again? Current State: %s" % vehicle.state)

        for station in stations:
            self.heartbeat(station)

        for fnc, args in functions:
            fnc(*args)

        del self.t_reminders[ms_msg_time]

    def on_SIM_RESPONSE_VEHICLE_STATUS(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        if vehicle.state in (self.UNLOAD_ADVANCING, self.QUEUE_ADVANCING,
                             self.LOAD_ADVANCING, self.EXIT_ADVANCING):
            # TODO: Move to on_NOTIFY_VEHICLE_STOPPED when message is implemented
            if abs(vehicle.vel) < 0.01 and abs(vehicle.accel) < 0.01: # check that we're stopped

                if vehicle._advance_begun is False:
                    vehicle.advance()
                    vehicle._advance_begun = True

                elif vehicle._advance_begun is True:
                    # Really need a NOTIFY STOPPED message to clean up this flow. For now just check if I've reached my berth and stopped.
                    if abs(vehicle.berth_pos - vehicle.BERTH_GAP - vehicle.pos) < 0.1 and vehicle.ts_id == vehicle.plat_ts and vehicle.vel < 0.001:
                        if vehicle.state is self.UNLOAD_ADVANCING:
                            vehicle.state = self.UNLOAD_WAITING
                        elif vehicle.state is self.QUEUE_ADVANCING:
                            vehicle.state = self.QUEUE_WAITING
                        elif vehicle.state is self.LOAD_ADVANCING:
                            vehicle.state = self.LOAD_WAITING
                        elif vehicle.state is self.EXIT_ADVANCING:
                            vehicle.state = self.EXIT_WAITING
                        else:
                            raise Exception("Unexpected state: %s" % vehicle.state)

                else:
                    raise Exception("Unexpected case: %s" % vehicle._advance_begun)

        elif vehicle.state is self.LAUNCH_WAITING:
            vehicle.state = self.LAUNCHING
            merge = self.manager.track2merge.get(vehicle.station.bypass)
            if merge:
                slot, launch_time = merge.create_station_merge_slot(vehicle.station, vehicle, self.current_time)
                vehicle.set_merge_slot(slot)
                vehicle.set_spline(vehicle.merge_slot.spline)
                vehicle._launch_begun = True
                self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                              self.current_time, vehicle.id, vehicle.station.id)
                self.set_fnc_notification(vehicle.station.release_berth, (vehicle,), launch_time)

            else:
                blocked_time = self.manager.is_launch_blocked(vehicle.station, vehicle, self.current_time)
                if not blocked_time:
                    vehicle.state = self.LAUNCHING
                    vehicle.send_path(vehicle.trip.path)
                    vehicle.run(speed_limit=self.LINE_SPEED)
                    vehicle._launch_begun = True
                    vehicle.station.release_berth(vehicle)
                    self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                                  self.current_time, vehicle.id, vehicle.station.id)
                else:
                    self.set_v_notification(vehicle, blocked_time)
                    self.log.info("t:%5.3f Delaying launch of vehicle %d from station %d until %.3f (%.3f delay)",
                                  self.current_time, vehicle.id, vehicle.station.id, blocked_time, blocked_time-self.current_time)

##        else:
##            warnings.warn("Received vehicle status. Not sure why... vID: %d, v.state: %s" % (vehicle.id, vehicle.state))

    def on_SIM_EVENT_PASSENGER_CREATED(self, msg, msgID, msg_time):
        p_status = msg.p_status
        pax = Passenger(p_status.pID, p_status.src_stationID,
                        p_status.dest_stationID, p_status.creation_time)
        station = self.manager.stations[pax.origin_id]
        station.add_passenger(pax)

    def on_SIM_COMPLETE_PASSENGERS_DISEMBARK(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is self.DISEMBARKING
        self.log.info("t:%5.3f Vehicle %d at station %d, berth %d, plat %d has completed disembark of %s",
                       self.current_time, vehicle.id, vehicle.station.id, vehicle.berth_id, vehicle.platform_id, msg.cmd.passengerIDs)

        vehicle.state = self.LOAD_WAITING
        vehicle.trip = None

    def on_SIM_COMPLETE_PASSENGERS_EMBARK(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is self.EMBARKING
        self.log.info("t:%5.3f Vehicle %d at station %d, berth %d, plat %d has completed embark of %s",
                       self.current_time, vehicle.id, vehicle.station.id, vehicle.berth_id, vehicle.platform_id, msg.cmd.passengerIDs)

        vehicle.state = self.EXIT_WAITING

    def on_SIM_NOTIFY_VEHICLE_ARRIVE(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        # TODO: Try to move the decision process closer to the switch to improve efficiency.
        # Vehicle needs to make choice to enter station or bypass.
        # Note: Some track networks will have one station's ON_RAMP_II lead straight into
        #       another station's "split" track seg, thus LAUNCHING is a viable state.
        if vehicle.state in (self.RUNNING, self.LAUNCHING) and msg.trackID == vehicle.trip.dest_station.split:
            if vehicle.pax: # has passengers
                try:   # Reserve an unload berth
                    vehicle.trip.dest_station.request_unload_berth(vehicle)
                    assert vehicle.berth_id is not None
                except NoBerthAvailableError: # none available, circle around and try again
                    # plan a path to the ts prior from current (that is, loop around)
                    old_dest = vehicle.trip.dest_station
                    vehicle.trip = self.manager.wave_off(vehicle)
                    vehicle.set_path(vehicle.trip.path)
                    self.log.info("%5.3f Vehicle %d waved off from dest_station %d because NoBerthAvailable. Going to station %d instead.",
                                   self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)

            else: # no passengers
                try:
                    vehicle.trip.dest_station.request_load_berth(vehicle)
                    assert vehicle.berth_id is not None
                except NoBerthAvailableError:
                    old_dest = vehicle.trip.dest_station
                    vehicle.trip = self.manager.deadhead(vehicle, exclude=(old_dest,))
                    vehicle.set_path(vehicle.trip.path)
                    self.log.info("%5.3f Empty vehicle %d bypassing dest_station %d due to lack of available berth. Going to station %d instead.",
                              self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)



            # If the vehicle is empty, and the assigned berth is the entrance
            # berth, give it up and waive off so as to not clog up the station
            # with an empty vehicle.
            if vehicle.has_berth_reservation() and vehicle.pax is None \
                    and vehicle.platform_id == Station.UNLOAD_PLATFORM \
                    and vehicle.berth_id == 0:
                old_dest = vehicle.trip.dest_station
                vehicle.trip.dest_station.release_berth(vehicle)
                vehicle.trip = self.manager.deadhead(vehicle, exclude=(old_dest,))
                vehicle.set_path(vehicle.trip.path)
                self.log.info("%5.3f Empty vehicle %d bypassing dest_station %d to avoid clogging entrance berth. Going to station %d instead.",
                              self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)

    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        if vehicle.state is self.RUNNING:
            # Tail just cleared the main line by entering into a station.
            station = self.manager.split2station.get(msg.trackID)
            if station is not None and vehicle.ts_id in station.ts_ids: # don't check against OFF_RAMP_I explicitly, because nose may already be past it.
                assert vehicle.berth_id is not None
                if vehicle.pax:
                    assert vehicle.platform_id == station.UNLOAD_PLATFORM
                    vehicle.state = self.UNLOAD_ADVANCING
                else:
                    vehicle.state = self.LOAD_ADVANCING

                vehicle.station = station
                vehicle.enter_station()

        elif vehicle.state is self.LAUNCHING:
            station = self.manager.track2station.get(msg.trackID)
            # Just joined the main line, entirely exiting the station zone
            if msg.trackID == station.ts_ids[Station.ON_RAMP_II]:
                vehicle.state = self.RUNNING
                vehicle.station = None

        # Check if just entered a merge's zone of control.
        if vehicle.state is self.RUNNING: # may have been LAUNCHING at beginning of function
            # manage the vehicle through a merge
            try:
                obj = self.manager.inlets[vehicle.ts_id]
                if isinstance(obj, Merge):
                    vehicle.do_merge(obj, self.current_time)
            except KeyError:
                pass

            try:
                obj = self.manager.outlets[vehicle.ts_id]
                # Vehicle just left a merge's zone of control
                if isinstance(obj, Merge):
                    # A MergeSlot may go unused if a vehicle enters a station
                    # rather than going to the merge point. Discard reservations
                    # until the correct one is found.
                    while True:
                        merge_slot = obj.reservations.pop(0)
                        assert isinstance(merge_slot, MergeSlot)
                        if merge_slot.vehicle is vehicle:
                            assert vehicle.merge_slot is merge_slot
                            vehicle.set_merge_slot(None)
                            break

            except KeyError:
                pass

class Manager(object): # Similar to VehicleManager in gtf_conroller class
    """Coordinates vehicles to satisfy passenger demand. Handles vehicle pathing."""
    def __init__(self, xml_path, sim_end_time, controller):
        """xml_path: the path (including filename) to the xml scenario file
        created by TrackBuilder.

        The scenario file is expected to have a GoogleTransitFeed section which
        provides vehicle scheduling and trip data, in addition to the typical
        TrackSegment, Station, and Vehicle data."""

        # Not fond of this sort of style. Fix when I want something simple to work on.
        Vehicle.manager = self
        Merge.manager = self

        self.controller = controller
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

        self.graph = self.build_graph(doc.getElementsByTagName('TrackSegments')[0])
        self.stations = self.load_stations(doc.getElementsByTagName('Stations')[0], self.graph)

        # Mapping from station's trackSegments to a station id.
        self.track2station = dict((ts, s) for s in self.stations.values() for ts in s.ts_ids)

        # Mapping from the split ts id to the Station instance
        self.split2station = dict((s.split, s) for s in self.stations.values())

        self.vehicles = self.load_vehicles(doc.getElementsByTagName('Vehicles')[0],
                                           doc.getElementsByTagName('VehicleModels')[0])


        # Create the Merges & Switches
        self.merges = []
        self.switches = []
        self.outlets = {}
        self.inlets = {}
        for node in self.graph.nodes_iter():
            predecessors = self.graph.predecessors(node)
            if len(predecessors) > 1:
                # don't include station merges
                is_station = False
                for p_node in predecessors:
                    if p_node in self.track2station:
                        is_station = True
                        break

                if not is_station:
                    merge = Merge(node,
                                  self.graph,
                                  self.vehicles,
                                  self.track2station)
                    self.merges.append(merge)
                    self.outlets[merge.outlet] = merge
                    for inlet in merge.inlets:
                        self.inlets[inlet] = merge

            successors = self.graph.successors(node)
            if len(successors) > 1:
                # don't include station switches
                is_station = False
                for s_node in successors:
                    if s_node in self.track2station:
                        is_station = True
                        break

                if not is_station:
                    switch = Switch(node,
                                    self.graph,
                                    self.vehicles,
                                    self.track2station)
                    self.switches.append(switch)
                    for outlet in switch.outlets:
                        self.outlets[outlet] = switch
                    self.inlets[switch.inlet] = switch

        self.track2merge = dict((ts, m) for m in self.merges for ts in m.zone_ids[0] + m.zone_ids[1])

        # For vehicles that are currently parked in berths, reserve those berths for them.
        for vehicle in self.vehicles.itervalues():
            station = self.track2station.get(vehicle.ts_id)
            if station is not None:
                if vehicle.ts_id == station.ts_ids[Station.UNLOAD]:
                    platform = Station.UNLOAD_PLATFORM
                elif vehicle.ts_id == station.ts_ids[Station.QUEUE]:
                    platform = Station.QUEUE_PLATFORM
                elif vehicle.ts_id == station.ts_ids[Station.LOAD]:
                    platform = Station.LOAD_PLATFORM
                else:
                    platform = None

                if platform is not None:
                    for idx, berth_pos in enumerate(station.berth_positions[platform]):
                        if abs(berth_pos - vehicle.pos) < 2: # within 2 meters of the target position
                            station.reservations[platform][idx] = vehicle
                            vehicle.station = station
                            vehicle.berth_pos = berth_pos
                            vehicle.berth_id = idx
                            vehicle.platform_id = platform
                            vehicle.plat_ts = vehicle.ts_id
                            break
        doc.unlink() # facilitates garbage collection

        # Check that the merge zones are large enough to accomodate a vehicle stopping and starting
        maneuver_dist = -1
        for v in self.vehicles.itervalues():
            maneuver_dist = max(maneuver_dist, v.get_dist_to_stop() + v.get_dist_to_line_speed())
        for m in self.merges:
            if maneuver_dist < m.zone_lengths[0]:
                raise Exception("Insufficient track distance from ts %d to ts %d for pre-merge manuevering." % (m.inlets[0], m.outlet))
            elif maneuver_dist < m.zone_lengths[1]:
                raise Exception("Insufficient track distance from ts %d to ts %d for pre-merge manuevering." % (m.inlets[1], m.outlet))
            else:
                pass

        # keys are station ids, values are lists of waiting (empty) vehicles at station
        self.idle = defaultdict(list) # idle vehicles

        # Precalculate common trips.
        self._dist_path_dict = self._build_station_tables()

    def build_graph(self, track_segments_xml):
        """Returns a networkx.DiGraph suitable for routing vehicles over."""
        graph = networkx.DiGraph()

        for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
            trackID = track_segment_xml.getAttribute('id')
            intId = int(trackID.split("_")[0]) # get a unique, integer ID
            length = float(track_segment_xml.getAttribute('length'))

            graph.add_node(intId)

            connect_to_xml = track_segment_xml.getElementsByTagName('ConnectsTo')[0]
            for id_xml in connect_to_xml.getElementsByTagName('ID'):
                connect_id = self._to_numeric_id(id_xml)
                graph.add_edge(intId, connect_id, weight=length)

        return graph

    def load_stations(self, stations_xml, graph):
        """Returns a dict of stations, keyed by the integer id."""
        stations = dict()
        for station_xml in stations_xml.getElementsByTagName('Station'):
            station_str_id = station_xml.getAttribute('id')
            station_int_id = int(station_str_id.split('_')[0])

            ts_ids_xml = station_xml.getElementsByTagName('TrackSegmentID')[:9] # fragile assumption :(
            ts_ids = [self._to_numeric_id(id_xml) for id_xml in ts_ids_xml]

            # Get berth position data for the three platforms: Unload, Queue, Load
            platforms_xml = station_xml.getElementsByTagName('Platform')

            unload_xml = platforms_xml[0]
            platform_ts_id = self._to_numeric_id(unload_xml.getElementsByTagName('TrackSegmentID')[0])
            assert platform_ts_id == ts_ids[Station.UNLOAD]
            unload_positions = []
            for berth_xml in unload_xml.getElementsByTagName('Berth'):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                unload_positions.append(end_pos)

            queue_xml = platforms_xml[1]
            platform_ts_id = self._to_numeric_id(queue_xml.getElementsByTagName('TrackSegmentID')[0])
            assert platform_ts_id == ts_ids[Station.QUEUE]
            queue_positions = []
            for berth_xml in queue_xml.getElementsByTagName('Berth'):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                queue_positions.append(end_pos)

            load_xml = platforms_xml[2]
            platform_ts_id = self._to_numeric_id(load_xml.getElementsByTagName('TrackSegmentID')[0])
            assert platform_ts_id == ts_ids[Station.LOAD]
            load_positions = []
            for berth_xml in load_xml.getElementsByTagName('Berth'):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                load_positions.append(end_pos)

            # Discover the split, bypass, and merge TrackSegment ids from the graph
            assert len(graph.successors(ts_ids[Station.ON_RAMP_II])) == 1
            assert len(graph.predecessors(ts_ids[Station.OFF_RAMP_I])) == 1
            merge = graph.successors(ts_ids[Station.ON_RAMP_II])[0]
            split = graph.predecessors(ts_ids[Station.OFF_RAMP_I])[0]
            # find the ts that's downstream from the offramp switch, and also upstream from the onramp merge.
            downstream = graph.successors(split)
            upstream = graph.predecessors(merge)
            for ts in downstream:
                if ts in upstream:
                    bypass = ts
                    break

            # Calculate the total onramp length
            post_merge = graph.successors(ts_ids[Station.ON_RAMP_II])[0]
            onramp_length = networkx.dijkstra_path_length(self.graph,
                                            ts_ids[Station.ACCEL], post_merge)

            stations[station_int_id] = Station(station_int_id, ts_ids, split, bypass, merge, onramp_length,
                                               unload_positions, queue_positions, load_positions)
        return stations

    def load_vehicles(self, vehicles_xml, models_xml):
        """Returns a dict of vehicles, keyed by the integer id."""
        vehicles = dict()

        models = dict()
        for model_xml in models_xml.getElementsByTagName('VehicleModel'):
            models[model_xml.getAttribute('model')] = model_xml

        for vehicle_xml in vehicles_xml.getElementsByTagName('Vehicle'):
            vehicle_str_id = vehicle_xml.getAttribute('id')
            vehicle_int_id = int(vehicle_str_id.split('_')[0])

            ts_str_id = vehicle_xml.getAttribute('location')
            ts_int_id = int(ts_str_id.split('_')[0])

            model_name = vehicle_xml.getAttribute('model')

            model_xml = models[model_name]
            length = float(model_xml.getAttribute('length'))
            capacity = int(model_xml.getAttribute('passenger_capacity'))
            jerk_xml = model_xml.getElementsByTagName('Jerk')[0]
            accel_xml = model_xml.getElementsByTagName('Acceleration')[0]
            vel_xml = model_xml.getElementsByTagName('Velocity')[0]

            v = Vehicle(vehicle_int_id,
                        model_name,
                        length,
                        capacity,
                        ts_int_id,
                        float(vehicle_xml.getAttribute('position')),
                        float(vehicle_xml.getAttribute('velocity')),
                        float(vehicle_xml.getAttribute('acceleration')),
                        float(jerk_xml.getAttribute('normal_max')),
                        float(jerk_xml.getAttribute('normal_min')),
                        float(accel_xml.getAttribute('normal_max')),
                        float(accel_xml.getAttribute('normal_min')),
                        float(vel_xml.getAttribute('normal_max')))

            vehicles[vehicle_int_id] = v

        return vehicles

    def request_trip(self, vehicle):
        """The vehicle must be currently in a station. For best efficiency,
        delay calling this function until the vehicle is ready to load passengers.
        Returns a Trip instance.
        Propagates a NoPaxAvailableError if no passengers are available at vehicle's current station.
        """
        assert isinstance(vehicle, Vehicle)
        orig_station = vehicle.station
        pax = orig_station.pop_passenger()
        dest_station = self.stations[pax.dest_id]
        path_length, path = self.get_path(vehicle.ts_id, dest_station.ts_ids[Station.UNLOAD])
        return Trip(dest_station, path, (pax.id,) ) # just one pax for now

    def deadhead(self, vehicle, exclude=tuple()):
        """Go to a station that has waiting passengers and needs more vehicles.
        The exclude parameter is a tuple of Station instances that the vehicle
        should not pick as a destination.
        Returns a Trip instance.
        """
        assert isinstance(vehicle, Vehicle)
        dists, paths = networkx.single_source_dijkstra(self.graph, vehicle.ts_id)
        max_dist = max(dists.itervalues())
        best_station = None
        best_station_dist = 0
        best_value = 0
        closest_station = None
        closest_dist = float('inf')
        for station in self.stations.itervalues():
            if station in exclude:
                continue
            station_ts = station.ts_ids[Station.UNLOAD]
            dist = dists[station_ts]
            demand = station.get_demand()
            value = max_dist/dist * demand
            if value >= best_value:
                best_station = station
                best_station_dist = dist
                best_value = value
            if dist < closest_dist:
                closest_station = station
                closest_dist = dist

        if best_value > 0:
            return Trip(best_station, paths[best_station.ts_ids[Station.UNLOAD]], tuple())
        else: # no demand anywhere
            if vehicle.station is not None: # in a station -- just stay there
                return None
            else: # on the main line, really need someplace to go
                return Trip(closest_station, paths[closest_station.ts_ids[Station.UNLOAD]], tuple())

    def wave_off(self, vehicle):
        """Sends a vehicle around in a loop so as to come back and try entering the
        station later, or sends it to a different station if empty."""
        # vehicle has passengers, so must go to this particular station
        if vehicle.trip.passengers:
            prev_tses = self.graph.predecessors(vehicle.ts_id) # there may be a merge just upstream of station split ts
            best_loop_length, best_loop_path = float('inf'), None
            for prev_ts in prev_tses:
                loop_length, loop_path = self.get_path(vehicle.ts_id, prev_ts)
                if loop_length < best_loop_length:
                    best_prev_ts = prev_ts
                    best_loop_length = loop_length
                    best_loop_path = loop_path

            # concatenate that path with a path to the station's unload
            entry_length, entry_path = self.get_path(best_prev_ts, vehicle.trip.dest_station.ts_ids[Station.UNLOAD])

            path = best_loop_path + entry_path[1:] # concat lists. Discard prev_ts, so that it's not duplicated.

            return Trip(vehicle.trip.dest_station, path, vehicle.trip.passengers)

        # Vehicle is empty, so any station will do.
        else:
            return self.deadhead(vehicle, exclude=(vehicle.trip.dest_station,))

    def get_path(self, ts_1, ts_2):
        """Returns a two tuple. The first element is the path length (in meters),
        and the second is the path. ts_1 and ts_2 are integer trackSegment ids."""
        try:
            return self._dist_path_dict[ts_1][ts_2]
        except KeyError:
            return networkx.bidirectional_dijkstra(self.graph, ts_1, ts_2)

    def is_launch_blocked(self, station, vehicle, now):
        """Returns false if the vehicle may launch immediately. Otherwise,
        returns the time at which the vehicle(s) currently obstructing the
        launch exit the conflict zone. At that time, a new check will need to be
        made, as there is no guarantee that other vehicles won't have entered
        the conflict zone in the interveaning time.

        ASSUMPTION: All vehicles on the network are the same length as vehicle!
        """
        assert isinstance(station, Station)
        assert isinstance(vehicle, Vehicle)
        # Optimization note: this is inefficent. Expect there to only be a few vehicle and station
        # variations, so caching the results for those few cases may make sense.

        # find the time it takes for the launch vehicle to reach the main line
        line_speed_dist = vehicle.get_dist_to_line_speed()

        # assume that vehicle is on the ts just prior to the station offramp
        merge_dist = (self.graph[vehicle.ts_id][station.ts_ids[Station.ACCEL]]['weight'] - vehicle.pos) + station.onramp_length
        assert line_speed_dist <= merge_dist, "Onramp is too short for vehicle to achieve line speed before merge. Adjust onramp length, or vehicle's max accel and jerk in the scenario's XML file. %.4f, %.4f" % (line_speed_dist, merge_dist)

        merge_delay = vehicle.get_time_to_line_speed() + (merge_dist - line_speed_dist)/PrtController.LINE_SPEED

##        knot_initial = Knot(vehicle.pos, vehicle.vel, vehicle.accel, now)
##        knot_line_speed = Knot(None, PrtController.LINE_SPEED, 0, None)
##
##        spline = vehicle.traj_solver.target_velocity(knot_initial, knot_line_speed)
##
##        knot_final = Knot(vehicle.pos + station.onramp_length, PrtController.LINE_SPEED, 0
##
##        assert spline.q[-1] <= vehicle.pos + station.onramp_length, \
##               "Vehicle's accel and jerk settings are not sufficient to reach line speed before merging with the main line. " + \
##               "Adjust the network or vehicle settings in the scenario's XML file. %.3f, %.3f" \
##               % (spline.q[-1], vehicle.pos + station.onramp_length)
##        merge_delay = spline.t[-1] - spline.t[0]
##        assert merge_delay > PrtController.HEADWAY

        # Making the assumption that all vehicles on the main line are travelling
        # at constant velocity, then a safe launch just requires that a particular
        # section of track upstream of the merge is unoccupied. I'm referring
        # to that section as the confict_zone. So long as the line_speed is low,
        # the conflict zone will be close to the station onramp and is unlikely
        # to be complicated. As line speed increases, the conflict zone will be
        # pushed back and enlarge and will be increasing likely to have a merge
        # or station lie between the conflict zone and the onramp.
        headway_dist = PrtController.HEADWAY * PrtController.LINE_SPEED
        conflict_zone_start_dist = PrtController.LINE_SPEED * merge_delay - headway_dist
        conflict_zone_end_dist = conflict_zone_start_dist + headway_dist + vehicle.length + headway_dist

        # walk back from the merge point finding the tracksegments and positions
        # that are in the conflict zone
        merge_seg = self.graph.successors(station.ts_ids[Station.ON_RAMP_II])[0] # assuming only one
        nodes, starts, ends = self.find_distant_segs_reverse(merge_seg,
                           conflict_zone_start_dist, conflict_zone_end_dist, [], [], [])

        # TODO optimize this?
        times = []
        for v in self.vehicles.itervalues():
            if v.station != None:
                continue

            try:
                idx = nodes.index(v.ts_id)
            except ValueError:
                continue

            # v may be in conflict zone. Estimate current position
            if abs(v.accel) > 0.0001:
                warnings.warn("Vehicle on main line NOT travelling at constant velocity! v: %d, ts: %d, vel: %.3f, accel: %.3f" %\
                          (v.id, v.ts_id, v.vel, v.accel))
##            assert abs(v.accel) < 0.0001, v.accel
            v_pos = v.pos + (now - v.last_update)*v.vel

            if v_pos > starts[idx] and v_pos < ends[idx]:
                # find the distance, and thus time, until v clears the conflict zone
                seg = v.ts_id
                clearing_dist = ends[idx] - v_pos
                path = networkx.shortest_path(self.graph, seg, merge_seg)
                for seg in path[1:]:
                    try:
                        idx = nodes.index(seg)
                        clearing_dist += ends[idx] - starts[idx]
                    except ValueError:
                        break

                times.append(clearing_dist/v.vel)

        if times:
            return max(times) + now
        else:
            return False

    def find_distant_segs_reverse(self, initial_node, start_dist, end_dist,
                                  nodes, starts, ends):
        """Recursively walk the edges of a weighted networkx digraph until start_dist is
        reached, then adds all nodes encountered until end_dist is reached.
        Excludes stations from the walk.
        initial_node: the starting segment
        start_dist: the distance from initial node at which the conflict zone starts
        end_dist: the distance from initial node at which the conflict zone ends
        nodes, starts, ends: lists to which the results are added.
        Returns three parallel lists: nodes, starts, and ends"""
        preds = self.graph.predecessors(initial_node)
        for node in preds:
            # exclude station segments from the walk
            if self.track2station.get(node) is not None:
                continue

            edge_length = self.graph[node][initial_node]['weight']
            if start_dist <= edge_length:
                start = max(edge_length-end_dist, 0)
                end = min(edge_length-start_dist, edge_length)
                try: # don't add duplicates. Use the widest start/end values found
                    idx = nodes.index(node)
                    starts[idx] = min(start, starts[idx])
                    ends[idx] = max(end, ends[idx])
                except ValueError: # regular case, adding a new node
                    nodes.append(node)
                    starts.append(start)
                    ends.append(end)

            if end_dist > edge_length:
                self.find_distant_segs_reverse(node, start_dist-edge_length,
                                          end_dist-edge_length, nodes, starts, ends)
        return nodes, starts, ends

    def coordinate_shift(self, idx, path):
        """Returns a numeric offet that when subtracted from a position in path[0]'s
        coordinate frame translates it to path[idx]'s coordinate frame.
        The path is a list of ts_ids."""
        if idx == 0:
            return 0

        return sum(self.graph[a][b]['weight'] for a, b in pairwise(path[:idx+1]))

    def _build_station_tables(self):
        """Returns a pair of 2D tables containing distances and paths. The tables
        are accessed like table[origin][dest], where each origin is a station's
        LOAD ts and the dest is a station's UNLOAD ts."""
        origins = [s.ts_ids[Station.LOAD] for s in self.stations.values()]
        dests = [s.ts_ids[Station.UNLOAD] for s in self.stations.values()]

        path_dist_dict = defaultdict(dict)

        for o in origins:
            for d in dests:
                path_dist_dict[o][d] = networkx.bidirectional_dijkstra(self.graph, o, d)

        return path_dist_dict

    def _to_numeric_id(self, element):
        """For elements similar to:
            <ID>x_trackSegment_forward</ID>
        where x is an integer. Returns just the integer value."""
        return int(element.childNodes[0].data.split('_')[0])

class Station(object):
    OFF_RAMP_I = 0
    OFF_RAMP_II = 1
    DECEL = 2
    UNLOAD = 3
    QUEUE = 4
    LOAD = 5
    ACCEL = 6
    ON_RAMP_I = 7
    ON_RAMP_II = 8

    UNLOAD_PLATFORM = 0
    QUEUE_PLATFORM = 1
    LOAD_PLATFORM = 2

    SPEED_LIMIT = None # In m/s. Set in main(). An ideal speed will be just *below*
                       # the max speed that a vehicle would hit when advancing one berth.
    controller = None

    def __init__(self, s_id, ts_ids, split_ts_id, bypass_ts_id, merge_ts_id, onramp_length,
                 unload_positions, queue_positions, load_positions):
        """s_id: An integer station id.
        ts_ids: A list containing integer TrackSegment ids. See Station consts.
        split_ts_id: The TrackSegment id upsteam of both the bypass and the
                        offramp.
        bypass_ts_id: The TrackSegment id which bypasses the station. That is,
                      the main track rather than the station's track.
        merge_ts_id: The TrackSegment id downstream of both the bypass and the
                        onramp to the main line.

        onramp_length: Sum of track segment lengths for segments leading to exit.
        unload_positions, queue_positions, load_positions:
        Each of the above are lists of floats, where each float designates
        a position that the vehicle will target in order to park in the berth.
        i.e. to unload in berth 1, the vehicle will go to the position found in
        unload_positions[1].
        """
        self.id = s_id
        self.ts_ids = ts_ids
        self.split = split_ts_id
        self.bypass = bypass_ts_id
        self.merge = merge_ts_id
        self.onramp_length = onramp_length
        self.berth_positions = [unload_positions, queue_positions, load_positions]
        self.reservations = [[None]*len(unload_positions),
                             [None]*len(queue_positions),
                             [None]*len(load_positions)]

        self.passengers = deque()
        self.v_count = 0

    def add_passenger(self, pax):
        self.passengers.append(pax)

    def pop_passenger(self):
        """Pops the oldest passenger and returns it. Raises NoPaxAvailableError
        if none available."""
        try:
            return self.passengers.popleft()
        except IndexError:
            raise NoPaxAvailableError

    def get_next_berth(self, berth_id, platform_id):
        """Returns a 4-tuple: (berth_pos, berth_id, platform_id, ts_id)
        describing the berth following the arguments. Returns None if there
        is no next berth."""

        try: # assume not at last berth on this platform_id
            return (self.berth_positions[platform_id][berth_id+1], berth_id+1, platform_id, self.ts_ids[platform_id+3])
        except IndexError: # whoops. That was the last berth.
            try:
                return (self.berth_positions[platform_id+1][0], 0, self.ts_ids[platform_id+4])
            except IndexError: # whoops. I was at the head of the loading platform_id
                return None

    def _request_berth_on_platform(self, vehicle, platform_id):
        """Finds an accessible, available berth which is as close to the station exit as
        possible, and reserves it for use by vehicle.
        platform_id: one of the station consts: *_PLATFORM

        If a berth is found then the vehicle's old berth is released and the vehicle's
        berth_pos, berth_id, platform_id, and plat_ts are updated.

        Will only choose a berth that is ahead of the vehicle's current berth.

        Returns (berth_position, berth_id, platform_id, ts_id) if sucessful,
        Raises NoBerthAvailableError if no berths are available on the requested platform.
        """
        assert isinstance(vehicle, Vehicle)
        if vehicle.platform_id is not None: # only allow forward flow through station
            assert platform_id >= vehicle.platform_id

        if platform_id == vehicle.platform_id:
            curr_berth_id = vehicle.berth_id
        else:
            curr_berth_id = -1

        choosen_idx = None
        for idx, reservation in enumerate(self.reservations[platform_id]):
            if idx <= curr_berth_id:
                continue
            elif reservation is None:
                choosen_idx = idx
            else:
                break

        if choosen_idx is not None:
            self.release_berth(vehicle)
            self.reservations[platform_id][choosen_idx] = vehicle
            self.v_count += 1
            berth_pos = self.berth_positions[platform_id][choosen_idx]
            plat_ts = self.ts_ids[platform_id+3]  # +3 maps platform_id to ts
            vehicle.berth_pos = berth_pos
            vehicle.berth_id = choosen_idx
            vehicle.platform_id = platform_id
            vehicle.plat_ts = plat_ts
            return (berth_pos, choosen_idx, platform_id, plat_ts)
        else: # no berth available
            raise NoBerthAvailableError()

    def request_unload_berth(self, vehicle):
        """Requsts a berth where passengers may be unloaded. If the vehicle
        already has a berth reservation, it is assumed that the vehicle is
        within it's assigned berth. The old reservation is revoked, and the
        vehicle is updated with new reserveation info. The new
        reservation info is also returned.

        Returns (berth_position, berth_id, platform_id, ts_id) if sucessful.
        Raises NoBerthAvailableError if no unload berths are available.
        """
        assert isinstance(vehicle, Vehicle)
        return self._request_berth_on_platform(vehicle, self.UNLOAD_PLATFORM)

    def request_load_berth(self, vehicle):
        """Requests a berth where passengers may be loaded. May return a berth
        on the unload, queue, or load platform depending on accessibility and
        availability. If the vehicle already has a berth reservation, it is
        assumed that the vehicle is within its assigned berth. The old
        reservation is revoked, and the vehicle is updated with new
        reservation info. The new reservation info is also returned.

        Returns (berth_position, berth_id, platform_id, ts_id) if sucessful.
        Raises NoBerthAvailableError if no queue or loading berths are available.
        """
        assert isinstance(vehicle, Vehicle)
        if vehicle.platform_id is None:
            platform_id = self.UNLOAD_PLATFORM
        else:
            platform_id = vehicle.platform_id

        # Check accessibility of platforms, choosing the furthest reachable
        # platform.
        for p_id in range(platform_id, self.LOAD_PLATFORM+1):
            if p_id == vehicle.platform_id:
                if any(self.reservations[p_id][vehicle.berth_id+1:]):
                    break
            else:
                if any(self.reservations[p_id]):
                    break

        # Check availability on desired platform. If NoBerthAvailable, step back
        # to previous platform unless platform would be behind vehicle's current
        # position. Uses assumption that station has linear layout,
        # and that lower platform ids are closer to the station's entrance.
        while True:
            try:
                return self._request_berth_on_platform(vehicle, p_id)
            except NoBerthAvailableError:
                p_id -= 1
                if p_id < vehicle.platform_id or p_id < self.UNLOAD_PLATFORM:
                    raise

    def request_launch_berth(self, vehicle):
        """Requests a berth from which vehicle may exit the station. May
        return a berth on the unload, queue, or load platform depending on
        accessibility and availability. If the vehicle already has a berth
        reservation, it is assumed that the vehicle is within its assigned
        berth. The old reservation is revoked, and the vehicle is updated with
        new reservation info. The new reservation info is also returned.

        Returns (berth_position, berth_id, platform_id, ts_id) if sucessful.
        Raises NoBerthAvailableError if no queue or loading berths are available.
        """
        # With the current station design, vehicles are launced from the last
        # loading berth.
        return self.request_load_berth(vehicle)


    def release_berth(self, vehicle):
        """Frees the vehicle's current berth for reuse. Alters the vehicle!
        Returns None."""
        assert isinstance(vehicle, Vehicle)
        found = False
        for platform in self.reservations:
            for idx, v in enumerate(platform):
                if v is vehicle:
                    platform[idx] = None
                    found = True
                    break
        if found:
            self.v_count -= 1

        vehicle.berth_pos = None
        vehicle.berth_id = None
        vehicle.platform_id = None
        vehicle.plat_ts = None

    def is_launch_berth(self, berth_id, platform_id):
        """Does berth have direct access to the main line? That is, a
        vehicle can exit the station without crossing other berths or platforms.
        """
        return platform_id == self.LOAD_PLATFORM and \
                     berth_id == len(self.reservations[platform_id])-1

    def is_load_berth(self, berth_id, platform_id):
        return platform_id == self.LOAD_PLATFORM

    def is_unload_berth(self, berth_id, platform_id):
        return platform_id == self.UNLOAD_PLATFORM

    def is_empty(self):
        """Returns True if no berths that have been reserved by vehicles."""
        if any(any(self.reservations[self.UNLOAD_PLATFORM]),
               any(self.reservations[self.QUEUE_PLATFORM]),
               any(self.reservations[self.LOAD_PLATFORM])):
            return False
        else:
            return True

    def get_demand(self):
        """Returns a value indicating vehicle demand. Minimum of 0"""
        if self.v_count == 0: # report at least a little demand if empty of vehicles
            return max(len(self.passengers), 0.2)
        else:
            return max(len(self.passengers), 0) - self.v_count

class Passenger(object):
    def __init__(self, pax_id, origin_id, dest_id, origin_time):
        self.id = pax_id
        self.origin_id = origin_id
        self.dest_id = dest_id
        self.origin_time = origin_time

        # This information is held for the purposes of rescheduling the
        # passenger if a vehicle is too full. See add_to_path.
        self.path = []

    def __str__(self):
        return "id: %d, origin_id: %d, dest_id: %d, origin_time: %f" % \
               (self.id, self.origin_id, self.dest_id, self.origin_time)

class Vehicle(object):
    SPEED_INCREMENT = 2.77777  # 10 km/hr
    BERTH_GAP = 0.1 # 10 cm. A little room left between the vehicle's nose and the edge of the berth

    # To prevent a vehicle's trajectory from being undefined, splines are extended
    # so as to reach to the end of the simulation (and a little further). This
    # constant controls the 'little further'. Needs to be long enough that no algorithm
    # tries to predict a vehicle trajectory beyond this length. The incintive to keep
    # it somewhat small is that rounding errors are excacerbated if it is very large.
    SPLINE_TIME_EXTENSION = 3600     # in seconds

    controller = None # interfaces with the sim
    manager = None # high level planner

    def __init__(self, v_id, model, length, capacity, ts_id, pos, vel, accel, j_max, j_min, a_max, a_min, v_max):
        # Can't handle non-zero initial accelerations at this time
        assert accel == 0

        self.id = v_id
        self.model = model
        self.capacity = capacity
        self.length = length
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.j_max = j_max
        self.j_min = j_min
        self.a_max = a_max
        self.a_min = a_min
        self.v_max = v_max

        self.pax = []

        # The spline's coordinate frame is always the first element of the path.
        self._path = [ts_id]
        self._current_path_index = 0

        end_time = self.controller.sim_end_time + 1
        self._spline = CubicSpline([pos, pos+vel*end_time], [vel, vel], [0, 0], [0], [0.0, end_time])
        self._spline_ts_id = ts_id

        self.traj_solver = TrajectorySolver(self.v_max, self.a_max, self.j_max,
                                            0, self.a_min, self.j_min)
        self.last_update = 0.0
        self.trip = None
        self.state = None
        self.station = None
        self._merge_slot = None

        # Info for the reserved berth, not the current berth!!
        self.platform_id = None
        self.berth_pos = None
        self.berth_id = None
        self.plat_ts = None

        # A private flag for indicating whether a vehicle's "advance" has been
        # commanded yet or not.
        self._advance_begun = True
        self._launch_begun = True

        accel_spline = self.traj_solver.target_velocity(Knot(0,0,0,0),
                             Knot(None,self.controller.LINE_SPEED,0,None))
        self._dist_to_line_speed = accel_spline.q[-1]
        self._time_to_line_speed = accel_spline.t[-1]

        decel_spline = self.traj_solver.target_velocity(Knot(0,self.controller.LINE_SPEED,0,0),
                             Knot(None,0,0,None))
        self._dist_to_stop = decel_spline.q[-1]
        self._time_to_stop = decel_spline.t[-1]

    def get_ts_id(self):
        return self._path[self._current_path_index]
    ts_id = property(get_ts_id)

    def get_spline(self):
        return self._spline
    def set_spline(self, spline, send=True):
        """Side Effect Warning: If the spline does not continue until the end of
                             the sim, then it is extended.
        """
        assert isinstance(spline, CubicSpline)
        assert spline.t[0] - self.controller.current_time < 1E-6 # Spline is valid at current time and beyond.
        self._path = self._path[self._current_path_index:]
        self._current_path_index = 0

        sim_end_time = self.controller.sim_end_time
        if spline.t[-1] < sim_end_time:
            assert abs(spline.a[-1]) < 1E-6
            delta_q = spline.v[-1]*(sim_end_time + self.SPLINE_TIME_EXTENSION - spline.t[-1])
            spline.append(Knot(spline.q[-1]+delta_q, spline.v[-1], 0, sim_end_time+self.SPLINE_TIME_EXTENSION), 0)

        self._spline = spline
        if send:
            self.send_spline()
    spline = property(get_spline, doc="""A vehicle's planned trajectory represented by a cubic_spline.CubicSpline object.""")

    def get_merge_slot(self):
        return self._merge_slot
    def set_merge_slot(self, merge_slot):
        self._merge_slot = merge_slot
    merge_slot = property(get_merge_slot, doc="""A vehicle's MergeSlot instance.""")

    def get_path(self):
        return self._path
    def set_path(self, path, send=True):
        assert path[0] == self._path[self._current_path_index]
        self._path = self._path[:self._current_path_index] + path
        if send:
            self.send_path()
    path = property(get_path, doc="""A vehicle's planned path, as a list of tracksegment id's. The first id must be the vehicle's current location.""")

    def update_vehicle(self, v_status, time):
        """Updates the relevant vehicle data."""
        assert v_status.vID == self.id
        if self.ts_id != v_status.nose_locID:
            self._current_path_index += 1
            assert self.ts_id == int(v_status.nose_locID) # convert to int from a long
        self.pos = v_status.nose_pos
        self.vel = v_status.vel
        self.accel = v_status.accel
        self.pax = v_status.passengerIDs[:] # copy
        self.last_update = time

        if v_status.lv_distance <= self.vel*self.controller.HEADWAY - 0.001:
            self.controller.log.warn("Vehicle %i is following too close to vehicle %i. lv_dist: %f.3 desired separation: %f.3" \
                                 % (self.id, v_status.lvID, v_status.lv_distance, self.vel*self.controller.HEADWAY))

##        if __debug__:
##            # Verify that the data from the sim is still matching with the vehicle's
##            # spline. Note that these asserts should be removed if the vehicle
##            # is sent v_status data with noise added.
##            knot = self.estimate_pose(time)
##            assert abs(knot.pos - self.pos) < 1E-3
##            assert abs(knot.vel - self.vel) < 1E-3
##            assert abs(knot.accel - self.accel) < 1E-3

    def estimate_pose(self, time, idx=None, path=None):
        """Returns a cubic_spline.Knot containing the vehicle's position,
        velocity, and acceleration at time. The position is translated
        from path[0]'s coordinate frame to path[idx]'s coordinate frame."""
        if path is None:
            path = self._path
        if idx is None:
            idx = self._current_path_index

        offset = self.manager.coordinate_shift(idx, path)
        knot = self._spline.evaluate(time)
        knot.pos -= offset
        return knot

    def send_path(self):
        """Sends the path to the sim. Path is a sequence of tracksegment ids,
        where the first element is expected to be the vehicle's current
        tracksegment."""
        if len(self.path) > 1: # only send the itinerary msg if it contains information
            itinerary_msg = api.CtrlCmdVehicleItinerary()
            itinerary_msg.vID = self.id
            itinerary_msg.trackIDs.extend(self._path[self._current_path_index+1:]) # don't include the segment self is currently on
            itinerary_msg.clear = True # Always replace the existing path, rather than appending to it.
            self.controller.send(api.CTRL_CMD_VEHICLE_ITINERARY, itinerary_msg)

    def run(self, speed_limit=None, dist=None, final_speed=0):
        """Commands the vehicle's path and trajectory. Obeys the speed_limit
        constraint, if supplied. Estimates the vehicle's current pose from the
        current spline.

        'speed_limit' gives an upper limit on vehicle speed. Vehicle may use
        other, lower limits if applicable.

        'dist' is given as the number of meters to the target position. If left
        as none, then the vehicle accelerates to the speed limit and continues
        at that velocity until the end of the simulation (or until trajectory
        is changed).

        'final_speed' is to be used in conjunction with 'dist'. Measured in meters/sec.

        If a distance is specified, returns the scheduled arrival time."""
        if speed_limit is None:
            speed_limit = self.v_max
        else:
            speed_limit = min(self.v_max, speed_limit)

        initial_knot = self.estimate_pose(self.controller.current_time)

        if dist is None:
            spline = self.traj_solver.target_velocity(initial_knot,
                                                      Knot(None, speed_limit, 0, None))
        else:
            spline = self.traj_solver.target_position(initial_knot,
                                        Knot(dist + initial_knot.pos, final_speed, 0, None),
                                        max_speed=speed_limit)

        self.set_spline(spline)

        # If a distance was specified, return the arrival time.
        if dist is not None:
            return spline.t[-2]

    def enter_station(self):
        """Slows the vehicle to the station's speed limit and brings the vehicle
        to a halt at it's reserved berth/platform. Changes the trajectory, but
        does not alter the vehicle's path."""
        # If vehicle has a merge_slot, revoke it.
        if self.merge_slot:
            self.merge_slot.merge.cancel_merge_slot(self.merge_slot)
            self.set_merge_slot(None)

        station = self.trip.dest_station
        current_knot = self.estimate_pose(self.controller.current_time)

        # Slow to station speed limit at the start of the UNLOAD segment
        unload_dist, unload_path = self.manager.get_path(self.ts_id, station.ts_ids[Station.UNLOAD])
        unload_knot = Knot(unload_dist, station.SPEED_LIMIT, 0, None)
        to_unload_spline = self.traj_solver.target_position(current_knot, unload_knot, max_speed=current_knot.vel)
        unload_knot.time = to_unload_spline.t[-1]

        # Continue on to stop at the desired berth pos (works even if platform is other than UNLOAD).
        berth_dist, berth_path = self.manager.get_path(station.ts_ids[Station.UNLOAD], self.plat_ts)
        berth_knot = Knot(unload_knot.pos + berth_dist + self.berth_pos - self.BERTH_GAP, 0, 0, None)
        to_berth_spline = self.traj_solver.target_position(unload_knot, berth_knot, max_speed=station.SPEED_LIMIT)

        spline = to_unload_spline.concat(to_berth_spline)
        self.controller.set_v_notification(self, spline.t[-1])

        self.set_spline(spline)
        self.set_path(unload_path + berth_path[1:]) # don't duplicate the UNLOAD ts_id

    def send_spline(self):
        """Sends a the current vehicle spline to the sim."""
        traj_msg = api.CtrlCmdVehicleTrajectory()
        traj_msg.vID = self.id
        self._spline.fill_spline_msg(traj_msg.spline)
        self.controller.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, traj_msg)

    def is_full(self):
        assert len(self.pax) <= self.capacity
        return len(self.pax) == self.capacity

    def is_empty(self):
        return True if len(self.pax) == 0 else False

    def disembark(self, passengers):
        """Disembark passengers"""
        # send disembark command
        disembark_msg = api.CtrlCmdPassengersDisembark()
        disembark_msg.vID = self.id
        disembark_msg.sID = self.station.id
        disembark_msg.platformID = self.platform_id
        disembark_msg.berthID = self.berth_id
        disembark_msg.passengerIDs.extend(passengers)
        self.controller.send(api.CTRL_CMD_PASSENGERS_DISEMBARK, disembark_msg)

        self.controller.log.info("t:%5.3f Start Disembark: Vehicle %d, pos %.2f at station %d, plat %d, berth %d is starting disembark of %s",
                                 self.controller.current_time, self.id, self.pos, self.station.id, self.platform_id, self.berth_id, passengers)

        for pax in passengers:
            self.pax.remove(pax)

    def embark(self, passengers):
        """Embark passengers"""
        fill_line = self.capacity - len(self.pax)

        self.pax.extend(passengers[:fill_line])

        # send embark command
        embark_msg = api.CtrlCmdPassengersEmbark()
        embark_msg.vID = self.id
        embark_msg.sID = self.station.id
        embark_msg.platformID = self.platform_id
        embark_msg.berthID = self.berth_id
        embark_msg.passengerIDs.extend(passengers[:fill_line])
        self.controller.send(api.CTRL_CMD_PASSENGERS_EMBARK, embark_msg)

        self.controller.log.info("t:%5.3f Start Embark: Vehicle %d, pos %.2f at station %d, plat %d, berth %d is starting embark of %s. Overflow: %s",
                                 self.controller.current_time, self.id, self.pos,
                                 self.station.id, self.platform_id, self.berth_id,
                                 passengers[:fill_line], passengers[fill_line:])

        # Return overflow passengers
        return passengers[fill_line:]

    def advance(self, speed_limit=None):
        """Sets the vehicle on a itenarary and trajectory for advancing to the
        vehicle's reserved berth.
        Returns the time at which the vehicle will arrive."""
        if self.plat_ts == self.ts_id: # staying on the same platform
            dist = (self.berth_pos - self.BERTH_GAP) - self.pos # stop a little short of the end

        else: # advancing to another next platform
            path_length, path = self.manager.get_path(self.ts_id, self.plat_ts)
            dist = (path_length - self.pos) + (self.berth_pos - self.BERTH_GAP) # stop a little short of the end

            # Set the path, but don't stomp on the existing path if it's the same but longer
            p = self._path[self._current_path_index:]
            if len(path) > len(p) or path != p[:len(path)]:
                self.set_path(path)

        if speed_limit == None:
            speed_limit = self.station.SPEED_LIMIT

        return self.run(speed_limit=speed_limit, dist=dist, final_speed=0)

    def do_merge(self, merge, now):
        assert isinstance(merge, Merge)
        merge_slot = merge.create_merge_slot(self, now)
        self.set_merge_slot(merge_slot)
        if self.merge_slot is not None: # Will be None if the vehicle is not yet at decision point
            self.set_spline(self.merge_slot.spline)

    def get_dist_to_line_speed(self, initial_knot=None):
        if initial_knot is None:
            return self._dist_to_line_speed
        else:
            final_knot = Knot(None, PrtController.LINE_SPEED, 0, None)
            spline = self.traj_solver.target_velocity(initial_knot, final_knot)
            return spline.q[-1] - spline.q[0]

    def get_time_to_line_speed(self, initial_knot=None):
        if initial_knot is None:
            return self._time_to_line_speed
        else:
            final_knot = Knot(None, PrtController.LINE_SPEED, 0, None)
            spline = self.traj_solver.target_velocity(initial_knot, final_knot)
            return spline.t[-1] - spline.t[0]

    def get_dist_to_stop(self, initial_knot=None):
        if initial_knot is None:
            return self._dist_to_stop
        else:
            final_knot = Knot(None, 0, 0, None)
            spline = self.traj_solver.target_velocity(initial_knot, final_knot)
            return spline.q[-1] - spline.q[0]

    def get_time_to_stop(self, initial_knot=None):
        if initial_knot is None:
            return self._time_to_stop
        else:
            final_knot = Knot(None, 0, 0, None)
            spline = self.traj_solver.target_velocity(initial_knot, final_knot)
            return spline.t[-1] - spline.t[0]

    def has_berth_reservation(self):
        if self.berth_id is not None:
            return True
        else:
            return False

class Trip(object):
    def __init__(self, dest_station, path, passengers):
        """A data holding object for vehicle trip info.
        dest_station: the destination Station instance
        dist: The distance to travel.
        path: first element is the vehicle's location at the intended beginning
              of the trip.
        passengers: a tuple of passenger ids.
        """
        assert isinstance(dest_station, Station)
        self.dest_station = dest_station
        self.path = path
        self.passengers = passengers
        self.dest_ts = path[-1]

    def __str__(self):
        return "dest_station: %d, dest_ts: %d, passengers: %s" % \
               (self.dest_station.id, self.dest_ts, str(self.passengers))

class Merge(object):
    """Responsible for zippering vehicles together at the merge point. The
    zone of control extends up both arms of the merging track. The zone of
    control ends at the next upstream merge or switch that isn't part of a
    station.

    ISSUES:
     - Vehicles that wish to enter a station found within the zone of control
       may not be granted entrance. Thus, they need to have a MergeSlot assigned
       to them that is achievable after being waived off from a station.
     - Vehicles that startup

    """

    # Set in main()
    LINE_SPEED = None
    HEADWAY = None
    controller = None
    manager = None

    _slips = None
    _maneuver_dists = None

    _SAMPLE_INTERVAL = 0.5 # distance between samples. Used to determine how far the vehicle can slip forward.

    def __init__(self, merge_node, graph, vehicles, tracks2station, cutoff=0):
        # TODO: Emergency stop distance prior to merge
        assert isinstance(graph, networkx.classes.DiGraph)

        Merge._calc_max_slips(vehicles.values()) # noop if already calculated

        # FIFO queue containing MergeSlot elements. Note that 'front'
        # includes the vehicle's headway. Append to right end, pop from left end.
        # Not using a deque because deque lacks insert, index, and splice.
        self.reservations = []

        # converts track segments to the merge coordinate frame. All positions
        # upstream of the merge are negative. The merge point is at 0.
        self.offsets = {}

        self.outlet = merge_node

        # maps station id's to distances from the station merge to the main merge
        self.station_dists = {}
        self.stations = []

        # set of ids, mainly for associating vehicles with a merge controller at startup
        self.zone_ids = [[], []]

        # Walk upstream for both zones
        self.zone_lengths = [-1, -1]
        self.inlets = [None, None] # segment ids which are entry points for the Merge's zone of control
        zones = graph.predecessors(merge_node)
        assert len(zones) == 2
        for zone_num, node in enumerate(zones):
            offset = 0
            up_node = node     # upstream
            down_node = merge_node # downstream
            while True:
                length = graph[up_node][down_node]['weight']
                offset -= length
                self.offsets[up_node] = offset
                self.zone_ids[zone_num].append(up_node)

                predecessors = graph.predecessors(up_node)

                # Only one path, move upstream
                if len(predecessors) == 1:
                    down_node = up_node
                    up_node = predecessors[0]

                # More than one path, encountered a merge.
                elif len(predecessors) > 1:
                    # if it's a merge from a station onramp, note the distance
                    # from the station to the main merge point and continue walking back
                    is_station = False
                    if predecessors[0] in tracks2station:
                        down_node = up_node
                        up_node = predecessors[1]
                        station = tracks2station[predecessors[0]]
                        self.station_dists[station.id] = offset
                        self.stations.append(station)
                        is_station = True
                    elif predecessors[1] in tracks2station:
                        down_node = up_node
                        up_node = predecessors[0]
                        station = tracks2station[predecessors[1]]
                        self.stations.append(station)
                        self.station_dists[station.id] = offset
                        is_station = True

                    # if the just encountered merge is not due to a station, stop walking
                    if not is_station:
                        self.zone_lengths[zone_num] = offset
                        self.inlets[zone_num] = up_node
                        break

                else: # len(up_nodes) == 0:
                    raise Exception("Not able to handle dead end track.")

                successors = graph.successors(up_node)

                # Sucessfully moved upstream, but up_node is a switch
                if len(successors) > 1:
                    # if it's not a switch from a station offramp, stop walking
                    if successors[0] not in tracks2station and successors[1] not in tracks2station:
                        self.zone_lengths[zone_num] = offset
                        self.inlets[zone_num] = down_node
                        break # TODO: Notify PRT controller that I'm associated with switch. May need to implement Switches sooner, rather than later

        self.cutoff = cutoff
        self._decision_point = max(self.zone_lengths) # max because numbers are negative

        for path in self.zone_ids:
            path.reverse() # zone_ids are now in the correct order to be a path through the merge.

    def create_merge_slot(self, vehicle, now):
        """Creates a non-conflicting MergeSlot for a vehicle that is entering
        the Merge's zone of control. The slot is added to the Merge's
        reservation queue.

        If this function is called prior to the vehicle reaching the Merge's
        zone of control, then the call is rescheduled using the controller's
        set_fnc_notification function.

        Returns: MergeSlot or None

        It is up to the vehicle to alter its trajectory so as to hit the
        MergeSlot, but the slot is guaranteed to be achievable by using the
        included spline.

        TODO: Rather than having the function reschedule itself, raise an
        exception or return None then have the calling function reschedule.
        """
        assert isinstance(vehicle, Vehicle)
        offset = self.offsets[vehicle.ts_id]
        initial_knot = vehicle.estimate_pose(now)

        v_pos = initial_knot.pos + offset # negative number. 0 is merge point
        v_rear_pos = v_pos - vehicle.length
        dist = -v_pos

        # If the vehicle isn't to the decision point yet, then delay managing it
        if v_rear_pos < self._decision_point - 0.0001:
            decision_time = vehicle.spline.get_time_from_dist(self._decision_point - v_rear_pos, now)
            assert decision_time >= now, (decision_time, now)
            self.controller.set_fnc_notification(vehicle.do_merge, (self, decision_time), decision_time) # call manage again
            return None

        # decide which zone the vehicle is in.
        if vehicle.ts_id in self.zone_ids[0]:
            zone = 0
        elif vehicle.ts_id in self.zone_ids[1]:
            zone = 1
        else:
            raise Exception("Vehicle isn't in Merge's zone of control.")

        # If a vehicle is intending to stop at a station prior to the merge point
        # and it has a reserved berth (ensuring that it can enter the station),
        # then it doesn't need a MergeSlot.
        if vehicle.has_berth_reservation() and vehicle.trip.dest_station in self.stations:
            return None

        # Create a MergeSlot for the vehicle to use.
        lead_slot = None
        try: # Usual case. There is a leading vehicle.
            lead_slot = self.reservations[-1]
            assert isinstance(lead_slot, MergeSlot)

            # First Pass Implementation. Only slip vehicle back to avoid collision, don't push lead vehicle forward
            final_knot = Knot(initial_knot.pos + dist, self.LINE_SPEED, 0, None)
            spline = vehicle.traj_solver.target_position(initial_knot, final_knot, max_speed=self.LINE_SPEED)

            # If the vehicle, travelling at line speed, would arrive at the merge point too early, use target_time
            if spline.t[-1] < lead_slot.end_time + self.HEADWAY:
                final_knot.time = lead_slot.end_time + self.HEADWAY
                spline = vehicle.traj_solver.target_time(initial_knot, final_knot)

            assert spline.t[-1] >= lead_slot.end_time + self.HEADWAY
            start_time = spline.t[-1] - self.HEADWAY
            end_time = start_time + vehicle.length/self.LINE_SPEED + self.HEADWAY

        except IndexError: # There is no leading vehicle. Travel at line speed.
            final_knot = Knot(initial_knot.pos + dist, self.LINE_SPEED, 0, None)
            spline = vehicle.traj_solver.target_position(initial_knot, final_knot, max_speed=self.LINE_SPEED)

            start_time = spline.t[-1] - self.HEADWAY
            # Vehicle may not be up to line speed when passing through merge point
            end_time = spline.t[-1] + vehicle.length/spline.evaluate(start_time).vel

            if lead_slot is not None:
                assert start_time >= lead_slot.end_time

        merge_slot = MergeSlot(start_time, end_time, zone, vehicle, spline, self)
        self.reservations.append(merge_slot)

        if __debug__ and len(self.reservations) >= 2:
            # no more than a little rounding error's worth of overlap
            assert (merge_slot.start_time - self.reservations[-2].end_time) >= -1E-6

        return merge_slot

    def create_station_merge_slot(self, station, vehicle, now):
        """Creates a non-conflicting MergeSlot for a vehicle that is launching
        from a station located within the Merge's zone of control. The slot is
        added to the Merge's reservation queue.

        Returns a 2-tuple: (MergeSlot, launch_time)

        It is up to the vehicle to alter its trajectory so as to hit the
        MergeSlot, but the slot is guaranteed to be achievable by using the
        included spline.
        """

        # Strategy is to find a pair of merge slots such that:
        #   1. There is room enough between the slots to fit a new slot.
        #   2. The vehicle leaving the station will not interfere with other
        #      vehicles while achieving its merge slot.
        # Note that the vehicle leaving the station has the option of remaining
        # stopped at the station indefinitely.
        #
        assert isinstance(vehicle, Vehicle)
        assert isinstance(station, Station)
        assert station in self.stations

        # Find which zone the station is in.
        if station.merge in self.zone_ids[0]:
            station_zone = 0
        else:
            assert station.merge in self.zone_ids[1]
            station_zone = 1

        # Vehicle's state, in it's current coordinate frame.
        initial = vehicle.estimate_pose(now)

        station_merge_offset = self.offsets[station.merge] # reminder: offsets are negative
        min_slot_size = self.HEADWAY + vehicle.length/self.LINE_SPEED # in seconds

        # The launch vehicle cannot pass through another vehicle in the same
        # zone while trying to reach its merge slot. Iterate up to the station-merge
        # point and find the blocking vehicle closest to the station-merge. We
        # can disregard any slots prior to the blocking vehicle's slot.
        blocking_slot_idx = None
        for i, slot in enumerate(self.reservations):
            if slot.vehicle.ts_id == self.outlet:
                continue

            if slot.zone == station_zone:
                try:
                    if self.offsets[slot.vehicle.ts_id] < station_merge_offset:
                        break
                    else:
                        blocking_slot_idx = i
                except KeyError:
                    # Vehicle is just departing from a station, and has not yet
                    # hit the main line.
                    continue


        # Bookend the reserved MergeSlots with a pair of dummy slots.
        if blocking_slot_idx is None:
            slots = [MergeSlot(0,0,None,None,None,self)] + self.reservations + [MergeSlot(inf,inf,None,None,None,self)]
        else:
            slots = self.reservations[blocking_slot_idx:] + [MergeSlot(inf,inf,None,None,None,self)]

        # Try constructing a launch spline and slot which disregards the world
        # state -- launch immediately and travel to main-merge at line speed.
        # Fastest possible arrival at the merge point.
        # TODO: Figure out conditions where this couldn't be a viable solution in order to save some computation?
        merge_dist, merge_path = self.manager.get_path(vehicle.ts_id, self.outlet)
        onramp_dist, onramp_path = self.manager.get_path(vehicle.ts_id, station.merge)
        blind_spline = vehicle.traj_solver.target_position(
                initial,
                Knot(merge_dist, self.LINE_SPEED, 0, None),
                max_speed=self.LINE_SPEED)
        blind_slot = MergeSlot(blind_spline.t[-1] - self.HEADWAY,
                               blind_spline.t[-1] + vehicle.length/self.LINE_SPEED,
                               station_zone,
                               vehicle,
                               blind_spline,
                               self)

        # Time at which the blind launch vehicle's tail is at the station merge point
        blind_station_merge_time = blind_spline.get_time_from_dist(onramp_dist - initial.pos + vehicle.length, now)

        # Iteration starts with slots closest to the merge point and works back.
        for idx in xrange(len(slots)-1):
            front_slot = slots[idx]
            rear_slot = slots[idx+1]
            assert isinstance(front_slot, MergeSlot)
            assert isinstance(rear_slot, MergeSlot)

            # Gap between slots must be large enough to fit a new slot.
            if rear_slot.start_time - front_slot.end_time < min_slot_size:
                continue

            # If blind, the fastest possible approach, wouldn't be clear of the
            # merge point when rear_slot starts then the gap is unusable.
            if blind_slot.end_time > rear_slot.start_time:
                continue

            # Optimistic case: No interference from anyone. Go immediately at LINE_SPEED.
            if blind_slot.start_time >= front_slot.end_time and \
                    blind_slot.end_time <= rear_slot.start_time:
                if self._is_rear_vehicle_conflict(idx+1, slots, station_zone, station.merge, blind_station_merge_time):
                    continue
                else:
                    slot = blind_slot
                    launch_time = now
                    break

            # In all other cases, we want to come onto the main line just
            # behind the front_slot vehicle (or ghost of that vehicle, if it's
            # in the other zone). Once on main line, replicate the front vehicle's
            # trajectory.
            else:
                # front_knot has a .pos that is the negative distance to main-merge
                try:
                    fs_outlet_idx = front_slot.vehicle.path.index(self.outlet)
                    front_knot = front_slot.vehicle.estimate_pose(now, fs_outlet_idx)
                except ValueError: # fs vehicle's path doesn't take it the the merge outlet
                    # make a temporary path that would carry the vehicle to the outlet.
                    tmp_idx = self.zone_ids[front_slot.zone].index(front_slot.vehicle.path[0])
                    tmp_path = self.zone_ids[front_slot.zone][tmp_idx:] + [self.outlet]
                    front_knot = front_slot.vehicle.estimate_pose(now, idx=len(tmp_path)-1, path=tmp_path)

                # Front is past the station-merge. Launch immediately.
                if front_knot.pos > station_merge_offset:
                    try:
                        final = Knot(merge_dist, self.LINE_SPEED, 0, front_slot.end_time + self.HEADWAY)
                        spline = vehicle.traj_solver.target_time(initial, final, max_speed=self.LINE_SPEED)
                        launch_time = now
                        station_merge_time = spline.get_time_from_dist(onramp_dist - initial.pos + vehicle.length, now)

                        if self._is_rear_vehicle_conflict(idx+1, slots, station_zone, station.merge, station_merge_time):
                            continue

                        slot = MergeSlot(spline.t[-1] - self.HEADWAY,
                                         spline.t[-1] + vehicle.length/self.LINE_SPEED,
                                         station_zone,
                                         vehicle,
                                         spline,
                                         self)
                        break
                    except FatalTrajectoryError:
                        slot = None
                        continue

                # Front vehicle is still approaching the station-merge.
                else:
                    # When the backend of front vehicle passes station-merge point
                    front_station_merge_time = front_slot.spline.get_time_from_dist(
                            station_merge_offset - front_knot.pos + front_slot.vehicle.length, now)

                    front_station_merge_knot = front_slot.spline.evaluate(front_station_merge_time)
                    try:
                        onramp_knot = Knot(onramp_dist,
                                           front_station_merge_knot.vel,
                                           0,
                                           front_station_merge_time + self.HEADWAY)
                        onramp_spline = vehicle.traj_solver.target_position(initial, onramp_knot, max_speed=onramp_knot.vel)

                        # Check that the vehicle can make it to the onramp knot
                        # at or before the designated time.
                        if onramp_spline.t[-1] - onramp_knot.time > 1E-6:
                            continue # can't make it on time

                        wait_dur = (onramp_knot.time - now) - (onramp_spline.t[-1] - onramp_spline.t[0])
                        launch_time = now + wait_dur

                        # recreate the onramp_spline, waiting to accelerate until launch_time
                        onramp_spline = CubicSpline([initial.pos] + onramp_spline.q,
                                                    [0] + onramp_spline.v,
                                                    [0] + onramp_spline.a,
                                                    [0] + onramp_spline.j,
                                                    [initial.time] + [time + wait_dur for time in onramp_spline.t])

                        final = Knot(merge_dist, self.LINE_SPEED, 0, front_slot.end_time + self.HEADWAY)
                        merge_spline = vehicle.traj_solver.target_time(onramp_knot, final, max_speed=self.LINE_SPEED)
                        spline = onramp_spline.concat(merge_spline)

                        slot = MergeSlot(spline.t[-1] - self.HEADWAY,
                                         spline.t[-1] + vehicle.length/self.LINE_SPEED,
                                         station_zone,
                                         vehicle,
                                         spline,
                                         self)
                        break
                    except FatalTrajectoryError:
                        slot = None
                        continue

        assert slot is not None

        try:
            assert slot.end_time <= rear_slot.start_time
            insert_idx = self.reservations.index(rear_slot)
            self.reservations.insert(insert_idx, slot)
        except ValueError:
            # Rear slot wasn't in self.reservations; it was a dummy slot.
            if __debug__ and self.reservations:
                assert self.reservations[-1].end_time <= slot.start_time
            self.reservations.append(slot)
        return slot, launch_time

    def _is_rear_vehicle_conflict(self, rear_slot_idx, slots, station_zone, station_merge_ts_id, launch_backend_merge_time):
        """A helper function for the 'create_station_merge_slot' function. Returns
        True if a vehicle coming from the rear will interfere with the launch
        vehicle at the station merge point. Returns False otherwise."""
        # HEADWAY seconds after the rear vehicle's tail reaches the
        # station_merge point, it is okay for the rear vehicle's nose
        # to reach the station merge point. Only care about the first vehicle
        # that is in the same zone as the station.
        while rear_slot_idx < len(slots):
            rear_slot = slots[rear_slot_idx]
            if rear_slot.zone != station_zone:
                rear_slot_idx += 1
            else:
                try:
                    station_merge_path_idx = rear_slot.vehicle.path.index(station_merge_ts_id)
                    rear_knot = rear_slot.vehicle.estimate_pose(launch_backend_merge_time + self.HEADWAY, station_merge_path_idx)
                except ValueError: # fs vehicle's path doesn't take it the the station_merge.
                    # make a temporary path that would carry the vehicle to the outlet.
                    tmp_idx = self.zone_ids[rear_slot.zone].index(rear_slot.vehicle.path[0])
                    tmp_path = self.zone_ids[rear_slot.zone][tmp_idx:] + [self.outlet]
                    rear_knot = rear_slot.vehicle.estimate_pose(launch_backend_merge_time, idx=len(tmp_path)-1, path=tmp_path)

                if rear_knot.pos > 0:
                    return True
                else:
                    return False

        return False

    def cancel_merge_slot(self, slot):
        assert isinstance(slot, MergeSlot)
        self.reservations.remove(slot)

    # DEPRECATED!!!
    def manage_(self, vehicle, now):
        """Creates a non-conflicting MergeSlot for vehicle and adds it to
        self.reservations. Alters the vehicle's trajectory in order for it to
        arrive at the merge point at the reserved time.
        TODO: Move the trajectory altering logic to the Vehicle class. Expand
              the MergeSlot class so that it contains all information that the
              vehicle will need to make the trajectory chanege."""
        # TODO: What to do during startup, when vehicles are still getting up to line_speed???
        assert isinstance(vehicle, Vehicle)

        knot = vehicle.estimate_pose(now)
        offset = self.offsets[vehicle.ts_id]
        v_spline = vehicle.get_spline()
        v_pos = knot.pos + offset # negative number. 0 is merge point
        v_rear_pos = v_pos - vehicle.length
        available_maneuver_dist = self.cutoff - v_pos

        # Vehicle is not intending to stop at a station in the Merge's zone of control.
        if vehicle.trip.dest_station not in self.stations:
            v_nose_time = v_spline.get_time_from_dist(-v_pos, now) # nose reaches merge point
            v_start_time = v_nose_time - self.HEADWAY
            v_end_time = v_spline.get_time_from_dist(-v_pos + vehicle.length, now)

        # Vehicle is intending to stop at a station, and is not expected to go
        # all the way to the merge point.
        else:
            # If the vehicle has a berth already reserved at the station, then
            # we can be assured that the vehicle will not be waived off, and thus
            # the vehicle doesn't need a MergeSlot.
            if vehicle.has_berth_reservation():
                return None

            # Otherwise the vehicle needs a MergeSlot in case it gets waived off
            # from the station. This MergeSlot will go unused if the vehicle
            # sucessfully gains entry to the station. We can't use the
            # vehicle's current trajectory to estimate arrival at the merge
            # point, since that trajectory is for the vehicle entering the
            # station. Create a new spline that goes on to the merge point, then
            # alter that.
            merge_knot = Knot(knot.pos - v_pos, self.LINE_SPEED, 0, None) # minus v_pos because it's a negative distance
            v_spline = vehicle.traj_solver.target_position(knot, merge_knot)

        stop_pos = None

        # decide which zone the vehicle is in. This is needed in
        # order to stop vehicles at an appropriate spot
        if vehicle.ts_id in self.zone_ids[0]:
            zone = 0
        elif vehicle.ts_id in self.zone_ids[1]:
            zone = 1
        else:
            raise Exception("Vehicle isn't in Merge's zone of control.")

        if v_rear_pos >= self._decision_point - 0.0001: # give a little flex for rounding error
            try:
                lv_slot = self.reservations[-1]
                assert isinstance(lv_slot, MergeSlot)
                if lv_slot.vehicle is vehicle: # guard against manage being called twice on the same vehicle
                    return

                v_lv_gap_time = v_start_time - lv_slot.end_time

                # vehicles on course to overlap at merge point
                if v_lv_gap_time < -0.00001:
                    try:
                        lv2_slot = self.reservations[-2]
                    except IndexError:
                        lv2_slot = None
                    slip_time = self._slip_ahead(lv_slot, lv2_slot, -v_lv_gap_time, available_maneuver_dist, now)
                    v_lv_gap_time += slip_time

                # vehicles are still on course to overlap at merge point, either
                # the lv couldn't advance, or couldn't advance enough.
                # vehicle slips back to make room.
                if v_lv_gap_time < -0.00001:
                    desired_slip_dist = v_lv_gap_time * self.LINE_SPEED
                    slip_dist, required_maneuver_dist = self._get_achievable_slip(available_maneuver_dist, desired_slip_dist, vehicle)

                    # if the vehicle comes to a complete stop while slipping back, decide where to park it
                    if slip_dist <= self._slips[vehicle.model][0]:
                        for slot in reversed(self.reservations):
                            if slot.zone == zone:
                                if slot.stop_pos is not None:
                                    stop_pos = slot.stop_pos - slot.vehicle.length - 1.0 # arbitrarily chose 1.0 meter as parked vehicle separation
                                    maneuver_start_pos = stop_pos - vehicle.get_dist_to_stop()
                                    break

                        if stop_pos is None: # Within the zone, the leading vehicle doesn't stop
                            stop_pos = self.cutoff - slot.vehicle.get_dist_to_line_speed()
                            maneuver_start_pos = stop_pos - vehicle.get_dist_to_stop()

                    else: # delay the maneuver until the last possible moment (best choice?)
                        stop_pos = None
                        maneuver_start_pos = self.cutoff - required_maneuver_dist

                    maneuver_start_dist = maneuver_start_pos - v_pos
                    assert maneuver_start_dist >= 0, maneuver_start_dist
                    maneuver_start_time = vehicle.spline.get_time_from_dist(maneuver_start_dist, now)

                    if slip_dist != desired_slip_dist:
                        raise Exception("Unable to slip back by the required amount: %.3f, %.3f" % (desired_slip_dist, slip_dist))
                    slip_spline = vehicle.traj_solver.slip(vehicle.spline, maneuver_start_time, slip_dist)
                    vehicle.set_spline(slip_spline)

                    v_nose_time = slip_spline.get_time_from_dist(-v_pos, now) # nose reaches merge point
                    v_start_time = v_nose_time - self.HEADWAY
                    v_end_time = v_nose_time + vehicle.length/self.LINE_SPEED
                else: # current vehicle does not need to adjust trajectory
                    slip_dist = 0

                merge_slot = MergeSlot(v_start_time, v_end_time, slip_dist, stop_pos, zone, vehicle)
                assert merge_slot.start_time >= lv_slot.end_time - 0.0001, (merge_slot.start_time, lv_slot.end_time)
                self.reservations.append(merge_slot)

            except IndexError: # vehicle is the frontmost in the merge queue
                merge_slot = MergeSlot(v_start_time, v_end_time, 0, None, zone, vehicle)
                self.reservations.append(merge_slot)

        else: # Notify when rear of vehicle due to reach the decison point
            time = vehicle.spline.get_time_from_dist(self._decision_point - v_rear_pos, now)
            assert time >= now, (time, now)
            self.controller.set_fnc_notification(self.manage_, (vehicle, time), time) # call manage again

    def _slip_ahead(self, slot, lead_slot, slip_time, avail_maneuver_dist, now):
        """Slips the vehicle in 'slot' ahead by 'slip_time' seconds. Returns the
        actual time slipped (may be less than requested time). Does not slip if
        the vehicle in slot has already slipped."""
        assert isinstance(slot, MergeSlot)
        assert isinstance(lead_slot, MergeSlot) or lead_slot is None
        assert slip_time >= 0, slip_time
        if slot.slip_dist is not None:
            return 0

        vehicle = slot.vehicle
        spline = slot.spline
        knot = slot.vehicle.estimate_pose(now)
        offset = self.offsets[vehicle.ts_id]
        pos = knot.pos + offset

        if lead_slot is not None:
            slip_time_potential = slot.start_time - lead_slot.end_time
            assert slip_time_potential >= -0.0001, slip_time_potential
        else:
            slip_time_potential = spline.get_time_from_dist(avail_maneuver_dist, now) - now # clearly too high. Rely on _get_achievable_slip to reduce it to a realistic value

        slip_dist_potential = slip_time_potential * self.LINE_SPEED
        slip_dist, req_maneuever_dist = self._get_achievable_slip(avail_maneuver_dist, slip_dist_potential, vehicle)

        # slip the desired amount, or the possible amount, whichever is less
        slip_dist = min(slip_time * self.LINE_SPEED, slip_dist)
        if slip_dist > 0:
            slip_spline = vehicle.traj_solver.slip(spline, now, slip_dist)
            vehicle.set_spline(slip_spline)

            slot.slip_dist = slip_dist
            slip_time = slip_dist / self.LINE_SPEED
            slot.start_time -= slip_time # slipping "ahead" means that it reaches the merge point sooner
            slot.end_time -= slip_time

            return slip_time
        else:
            return 0

    @staticmethod
    def _calc_max_slips(vehicles):
        # TODO: This uses vehicle's max velocity, and doesn't check the track's max vel
        if Merge._slips:
            return # only do calcs once

        slips = {}
        maneuver_dists = {}
        orig_spline = CubicSpline([0, 1000000], [Merge.LINE_SPEED, Merge.LINE_SPEED], [0, 0], [0], [0, 1000000/Merge.LINE_SPEED]) # 1000000 is arbitrarily large
        for vehicle in vehicles:
            assert isinstance(vehicle, Vehicle)
            if slips.get(vehicle.model):
                continue

            # find where the slip ahead function requires a linear amount of maneuver distance when slipping ahead
            tmp_spline = vehicle.traj_solver.target_velocity(Knot(0, Merge.LINE_SPEED, 0, 0),
                                                             Knot(None, vehicle.v_max, 0, 0))
            tmp_final = tmp_spline.evaluate(tmp_spline.t[-1])
            tmp_spline = vehicle.traj_solver.target_velocity(tmp_final,
                                                             Knot(None, Merge.LINE_SPEED, 0, None))
            ahead_boundary = tmp_spline.q[-1] - tmp_spline.t[-1] * Merge.LINE_SPEED

            # find where the slip back function requires a constant amount of maneuver distance when slipping back
            tmp_spline = vehicle.traj_solver.target_velocity(Knot(0, Merge.LINE_SPEED, 0, 0),
                                                             Knot(None, 0, 0, 0))
            tmp_final = tmp_spline.evaluate(tmp_spline.t[-1])
            tmp_spline = vehicle.traj_solver.target_velocity(tmp_final,
                                                             Knot(None, Merge.LINE_SPEED, 0, 0))
            back_boundary = tmp_spline.q[-1] - tmp_spline.t[-1] * Merge.LINE_SPEED

            # sample the function at fixed interval, cache slip dists and maneuver dists in parallel arrays
            m_dists = [] # maneuver
            s_dists = [] # slip
            for s in arange(back_boundary, ahead_boundary, Merge._SAMPLE_INTERVAL):
                s_dists.append(s)
                if s == 0:
                    m_dists.append(0)
                else:
                    slip_spline = vehicle.traj_solver.slip(orig_spline, 0, s)
                    m_dists.append(slip_spline.q[-2])

            # Add the ahead_boundary point as the last element
            slip_spline = vehicle.traj_solver.slip(orig_spline, 0, ahead_boundary)
            s_dists.append(ahead_boundary)
            m_dists.append(slip_spline.q[-2])

            slips[vehicle.model] = s_dists
            maneuver_dists[vehicle.model] = m_dists

        Merge._slips = slips
        Merge._maneuver_dists = maneuver_dists


    def _get_achievable_slip(self, maneuver_dist, desired_slip, vehicle):
        assert isinstance(vehicle, Vehicle)
        assert maneuver_dist >= 0

        slips = self._slips[vehicle.model]
        maneuver_dists = self._maneuver_dists[vehicle.model]

        slip = 0
        maneuver = 0

        if desired_slip < 0:
            # in the constant region, where any additional slip takes the same maneuver dist
            if desired_slip < slips[0] and maneuver_dist > maneuver_dists[0]:
                slip = desired_slip
                maneuver = maneuver_dists[0]
            else:
                for idx in range(len(slips)): # slips and maneuver_dists are the same length
                    # a little complicated because the required manuever dist is not a monotonic function of slip
                    if slips[idx] < desired_slip and slips[idx+1] > desired_slip and \
                       maneuver_dist > maneuver_dists[idx] and maneuver_dist > maneuver_dists[idx+1]:
                        slip = desired_slip
                        maneuver = max(maneuver_dists[idx], maneuver_dists[idx+1])
                        break
                    elif slips[idx] > desired_slip and maneuver_dist > maneuver_dists[idx]:
                        slip = slips[idx]
                        maneuver = maneuver_dists[idx]
                        break
                    else:
                        continue

        elif desired_slip > 0:
            # in the linear region, where the vehicle is advancing
            if desired_slip >= slips[-1] and maneuver_dist >= maneuver_dists[-1]:
                slope = vehicle.v_max/(vehicle.v_max - self.LINE_SPEED)
                slip = slips[-1] + min(desired_slip - slips[-1],
                                         (maneuver_dist - maneuver_dists[-1]) / slope)
                maneuver = maneuver_dists[-1] + min(maneuver_dist - maneuver_dists[-1],
                                                    (desired_slip - slips[-1]) * slope)
            else:
                for idx in range(len(slips)-1, -1, -1): # iterate backwards
                    if maneuver_dist < maneuver_dists[idx]:
                        continue
                    elif desired_slip > slips[idx]: # desired_slip exceeds what can be achieved within maneuver_dist. Settle for less.
                        slip = slips[idx]
                        maneuver = maneuver_dists[idx]
                        break
                    else: # desired_slip <= slips[idx] and maneuver_dist >= maneuver_dists[idx]
                        slip = desired_slip # desired slip is achievable
                        maneuver = maneuver_dists[idx]
                        break

        # else leave slip and maneuver as zeros

        return slip, maneuver

    def _slot_time(self, vehicle):
        assert isinstance(vehicle, Vehicle)
        return vehicle.length/self.LINE_SPEED + self.HEADWAY

    def _slot_length(self, vehicle):
        assert isinstance(vehicle, Vehicle)
        return vehicle.length + self.HEADWAY * self.LINE_SPEED

class MergeSlot(object):
    """Contains information about a vehicle's reservation of a merge slot.
    The rear of the slot is aligned with the tail of the vehicle, and the front
    of the slot includes the buffer distance indicated by HEADWAY and LINE_SPEED.
    """

    def __init__(self, start_time, end_time, zone, vehicle, spline, merge):
        """start_time: When the MergeSlot's reservation of the merge point begins, In seconds, where 0 is the start of the sim.
        end_time: When the reservation ends. The vehicle should be clear of the merge point at this time.
        zone: Which zone of the Merge controller the vehicle is approaching from.
        vehicle: The Vehicle instance for which the MergeSlot has been created.
        spline: The spline generated to check that the reserved start and end times can be viably achieved by the vehicle.
        merge: The Merge instance for which the MergeSlot has been created.
        """
        assert start_time >= 0
        assert end_time >= 0
        assert end_time >= start_time
        assert zone in (0,1,None)
        assert isinstance(vehicle, Vehicle) or vehicle is None
        assert isinstance(spline, CubicSpline) or spline is None
        assert isinstance(merge, Merge)
        self.start_time = start_time # the time at which the vehicle claims the merge point
        self.end_time = end_time # the time at which the vehicle releases the merge point
        self.zone = zone
        self.vehicle = vehicle
        self.spline = spline
        self.merge = merge

    def __str__(self):
        try:
            return "Start: %.3f, End: %.3f, Zone: %d, Vehicle: %d" % \
                   (self.start_time, self.end_time, self.zone, self.vehicle.id)
        except AttributeError:
            return "Start: %.3f, End: %.3f, Zone: %s, Vehicle: %s" % \
                   (self.start_time, self.end_time, str(self.zone), str(self.vehicle))

class Switch(object):
    """May force vehicles to reroute, based on conditions downstream of the
    switch."""

    def __init__(self, switch_node, graph, vehicles, tracks2station):
        assert isinstance(graph, networkx.classes.DiGraph)
        self.inlet = switch_node # the segment which has 2 successors
        self.outlets = [None, None] # the last segments in the zone of control
        self.stations = set()
        self.zone_lengths = [-1, -1]

        # Walk downsteam from the split
        zones = graph.successors(switch_node)
        assert len(zones) == 2
        self.offsets = {}
        for zone_num, node in enumerate(zones):
            offset = 0
            up_node = switch_node # upstream
            down_node = node     # downstream
            while True:
                length = graph[up_node][down_node]['weight']
                self.offsets[up_node] = offset
                offset += length

                # moved onto a non-station merge. End of zone.
                down_predecessors = graph.predecessors(down_node)
                if len(down_predecessors) > 1:
                    is_station = False
                    for p_node in down_predecessors:
                        if p_node in tracks2station:
                            is_station = True
                            break

                    if not is_station:
                        self.zone_lengths[zone_num] = offset
                        self.outlets[zone_num] = up_node
                        break # TODO: Notify PRT controller that I'm associated with switch. May need to implement Switches sooner, rather than later

                # move downstream
                down_nodes = graph.successors(down_node)
                if len(down_nodes) == 1:
                    up_node = down_node
                    down_node = down_nodes[0]

                # encountered a split. If split is from a station, continue walking down main track
                elif len(down_nodes) > 1:
                    assert len(down_nodes) == 2
                    if down_nodes[0] in tracks2station:
                        up_node = down_node
                        down_node = down_nodes[1] # take the other path
                        self.stations.add(tracks2station[down_nodes[0]])
                    elif down_nodes[1] in tracks2station:
                        up_node = down_node
                        down_node = down_nodes[0] # take the other path
                        self.stations.add(tracks2station[down_nodes[1]])
                    else: # Switch is not due to a station. End of zone.
                        self.zone_lengths[zone_num] = offset
                        self.outlets[zone_num] = up_node
                        break

                else: # len(down_nodes) == 0:
                    raise Exception("Not able to handle dead end track.")

if __name__ == '__main__':
    main()
