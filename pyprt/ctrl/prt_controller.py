from __future__ import division
import optparse
from collections import defaultdict
from operator import attrgetter
import warnings

import networkx

import pyprt.shared.api_pb2 as api
from pyprt.ctrl.base_controller import BaseController
from trajectory_solver import TrajectorySolver, FatalTrajectoryError
from pyprt.shared.cubic_spline import Knot
from pyprt.shared.utility import pairwise

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

        Station.controller = self

    def set_v_notification(self, vehicle, time):
        """Request a time notification from sim and store which vehicle the
        notification is relevant to."""
        key = int(time*1000)
        try:
            v_list, s_list = self.t_reminders[key]
            if vehicle not in v_list: # TODO: Could use a set here, though list will generally be small
                v_list.append(vehicle)

        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([vehicle], [])

    def set_s_notification(self, station, time):
        """Request a time notification from sim and store which station the
        notification is relevant to."""
        assert isinstance(station, Station)
        key = int(time*1000)
        try:
            v_list, s_list = self.t_reminders[key]
            assert station not in s_list
            s_list.append(station)

        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([], [station])

    def request_v_status(self, vehicle_id):
        msg = api.CtrlRequestVehicleStatus()
        msg.vID = vehicle_id
        self.send(api.CTRL_REQUEST_VEHICLE_STATUS, msg)

    def heartbeat(self, station):
        """Triggers synchronized advancement of vehicles in the station."""
        self.log.debug("t:%5.3f Station %d heartbeat.", self.current_time, station.id)
        assert isinstance(station, Station)
        for plat in (Station.LOAD_PLATFORM, Station.QUEUE_PLATFORM, Station.UNLOAD_PLATFORM):
            for v in reversed(station.berth_reservations[plat]):
                if v is not None:
                    # If vehicle is capable of moving forward, do so
                    self.trigger_advance(v, station) # changes v.state to *_ADVANCING if able to move

                    # For the vehicle's that are stuck in their current position
                    # try to do something useful with the time.
                    if v.state is self.UNLOAD_WAITING:
                        v.state = self.DISEMBARKING
                        v.disembark(v.trip.passengers)

                    elif v.state is self.LOAD_WAITING:
                        try:
                            v.trip = self.manager.request_trip(v)
                            v.state = self.EMBARKING
                            v.embark(v.trip.passengers)
                        except NoPaxAvailableError:
                            if station.is_launch_berth(v.berth_id, v.platform_id):
                                # launch as soon as possible, if there
                                # is another station with passengers waiting.
                                v.trip = self.manager.deadhead(v, (station,))
                                if v.trip is not None:
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
        if vehicle.state not in (self.UNLOAD_WAITING, self.QUEUE_WAITING, self.LOAD_WAITING, self.EXIT_WAITING, None):
            return

        assert vehicle.station is station

        # if vehicle hasn't unloaded, only try to advance as far as the last unload berth
        if vehicle.state is self.UNLOAD_WAITING:
            try:
                station.request_berth(vehicle, Station.UNLOAD_PLATFORM)
            except NoBerthAvailableError:
                return
        else:
            # All remaining cases should advance as far as the load platform, if able
            for plat in (Station.LOAD_PLATFORM, Station.QUEUE_PLATFORM, Station.UNLOAD_PLATFORM):
                try:
                    station.request_berth(vehicle, plat)
                    break
                except NoBerthAvailableError:
                    if plat > vehicle.platform_id:
                        continue
                    else:
                        return
            else:
                return

        # update vehicle's state
        if vehicle.state is self.UNLOAD_WAITING:
            vehicle.state = self.UNLOAD_ADVANCING
        elif vehicle.state is self.QUEUE_WAITING:
            # Change from QUEUE_* to LOAD_* once vehicle has made it to load platform
            if plat == Station.UNLOAD_PLATFORM or plat == Station.QUEUE_PLATFORM:
                vehicle.state = self.QUEUE_ADVANCING
            elif plat == Station.LOAD_PLATFORM:
                vehicle.state = self.LOAD_ADVANCING
            else:
                raise Exception("Unknown platform_id: %d" % plat)
        elif vehicle.state is self.LOAD_WAITING:
            vehicle.state = self.LOAD_ADVANCING
        elif vehicle.state is self.EXIT_WAITING:
            vehicle.state = self.EXIT_ADVANCING
        else:
            assert False, "Unexpected state: %s" % vehicle.state

        assert vehicle.plat_ts in vehicle.station.ts_ids

        # ... wait for the v_status update from the sim to do the movement
        vehicle._advance_begun = False

    ### Overriden message handlers ###
    def on_SIM_GREETING(self, msg, msgID, msg_time):
        self.sim_end_time = msg.sim_end_time
        self.log.info("Sim Greeting message received. Sim end at: %f" % msg.sim_end_time)

        self.manager = Manager(self.scenario_path, msg.sim_end_time, self)
        Vehicle.manager = self.manager
        Vehicle.controller = self

    def on_SIM_START(self, msg, msgID, msg_time):
        self.log.info("Sim Start message received.")

        for station in self.manager.stations.itervalues():
            # Work from the 2nd to last berth backward, initializing vehicles in order.
            for v in reversed(station.berth_reservations[Station.LOAD_PLATFORM]):
                if v is not None:
                    v.state = self.LOAD_WAITING

            for v in reversed(station.berth_reservations[Station.QUEUE_PLATFORM]):
                if v is not None:
                    v.state = self.QUEUE_WAITING

            for v in reversed(station.berth_reservations[Station.UNLOAD_PLATFORM]):
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

            # Within the boundaries of a station, but not at a berth.
            if station is not None:
                assert isinstance(station, Station)

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
                    vehicle.trip = Trip(station, dist, path, tuple())

                    # Reserve a berth for the vehicle
                    station.request_berth(vehicle, Station.UNLOAD_PLATFORM) # raises an exception if fails.
                    vehicle.station = station
                    vehicle.run_trip()

                # Outbound from a station
                else:
                    assert vehicle.ts_id in (station.ts_ids[Station.ACCEL],
                                             station.ts_ids[Station.ON_RAMP_I],
                                             station.ts_ids[Station.ON_RAMP_II])
                    vehicle.state = self.LAUNCHING
                    vehicle._launch_begun = True
                    vehicle.station = station
                    vehicle.trip = self.manager.deadhead(vehicle)
                    vehicle.run_trip()
                    self.log.info("%5.3f Vehicle %d deadheading to station %d.",
                                  self.current_time, vehicle.id, vehicle.trip.dest_station.id)


        # Make a second pass through the vehicles, handling the remaining vehicles.
        for vehicle in v_list:
            if vehicle.state is not None:
                continue

            station = self.manager.choice2station.get(vehicle.ts_id)

            if station is not None: # vehicle is on the station's "choice" trackseg
                assert vehicle.ts_id == station.choice
                vehicle.state = self.RUNNING

                # Set up a trip for reaching the station's UNLOAD platform.
                path_length, path = self.manager.get_path(vehicle.ts_id, station.ts_ids[Station.UNLOAD])
                dist = path_length - vehicle.pos
                vehicle.trip = Trip(station, dist, path, tuple())

                # Reserve a berth for the vehicle
                try:
                    station.request_berth(vehicle, Station.UNLOAD_PLATFORM) # raises an exception if fails.
                    vehicle.run_trip()
                except NoBerthAvailableError:
                    old_dest = vehicle.trip.dest_station
                    vehicle.trip = self.manager.wave_off(vehicle)
                    vehicle.run_trip()
                    self.log.info("%5.3f Empty vehicle %d waved off from dest_station %d because NoBerthAvailable. Going to station %d instead.",
                                  self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)

            # Not inside a station's boundaries, or outbound from a station
            else:
                vehicle.state = self.RUNNING
                vehicle.trip = self.manager.deadhead(vehicle)
                vehicle.run_trip()

        # station's heartbeats are all synchronized for now. Change that in the future?
        for station in self.manager.stations.itervalues():
            self.set_s_notification(station, self.HEARTBEAT_INTERVAL)

    def on_SIM_NOTIFY_TIME(self, msg, msgID, msg_time):
        ms_msg_time = int(msg.time*1000) # millisec integer for easy comparison
        vehicles, stations = self.t_reminders[ms_msg_time]

        for vehicle in vehicles:
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
                    vehicle.run_trip()
                    vehicle._launch_begun = True
                    vehicle.station.release_berth(vehicle)
                    self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                                  self.current_time, vehicle.id, vehicle.station.id)
                else:
                    self.set_v_notification(vehicle, blocked_time)
                    self.log.info("t:%5.3f Delaying launch of vehicle %d from station %d until %.3f (%.3f delay)",
                                  self.current_time, vehicle.id, vehicle.station.id, blocked_time, blocked_time-self.current_time)

            else:
                warnings.warn("Huh? What was this reminder for again? vehicle %d, state: %s" % (vehicle.id, vehicle.state))
##                raise Exception("Huh? What was this reminder for again? Current State: %s" % vehicle.state)

        for station in stations:
            self.heartbeat(station)

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
            blocked_time = self.manager.is_launch_blocked(vehicle.station, vehicle, self.current_time)
            if not blocked_time:
                vehicle.state = self.LAUNCHING
                vehicle.run_trip()
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

        vehicle.state = self.QUEUE_WAITING
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

        if vehicle.state is self.SLOWING and msg.trackID == vehicle.station.ts_ids[Station.UNLOAD]:
            # Just reached the beginning of the unload platform. Go to the already reserved berth.
            if vehicle.pax:
                vehicle.state = self.UNLOAD_ADVANCING
            else:
                # Skip disembarking entirely
                vehicle.state = self.QUEUE_ADVANCING
                vehicle.trip = None

            t_arrival = vehicle.advance()
            self.set_v_notification(vehicle, t_arrival)


        # TODO: Try to move the decision process closer to the switch to improve efficiency.
        # Vehicle needs to make choice to enter station or bypass.
        # Note: Some track networks will have one station's ON_RAMP_II lead straight into
        #       another station's "choice" track seg, thus LAUNCHING is a viable state.
        elif vehicle.state in (self.RUNNING, self.LAUNCHING) and msg.trackID == vehicle.trip.dest_station.choice:
            try:   # Reserve an unload berth
                vehicle.trip.dest_station.request_berth(vehicle, Station.UNLOAD_PLATFORM)
                assert vehicle.berth_id is not None
            except NoBerthAvailableError: # none available, circle around and try again
                # plan a path to the ts prior from current (that is, loop around)
                old_dest = vehicle.trip.dest_station
                vehicle.trip = self.manager.wave_off(vehicle)
                vehicle.run_trip()
                self.log.info("%5.3f Vehicle %d waved off from dest_station %d because NoBerthAvailable. Going to station %d instead.",
                               self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)

            if not vehicle.pax and vehicle.berth_id: # if empty vehicle, and received a berth
                if vehicle.platform_id == Station.UNLOAD_PLATFORM and \
                                                       vehicle.berth_id == 0:
                    # Don't clog up the station entrance with an empty vehicle
                    vehicle.trip.dest_station.release_berth(vehicle)
                    vehicle.trip = self.manager.deadhead(vehicle, exclude=tuple(vehicle.trip.dest_station))
                    vehicle.run_trip()
                    self.log.info("%5.3f Empty vehicle %d bypassing dest_station %d to avoid clogging entrance berth. Going to station %d instead.",
                                  self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)


    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        if vehicle.state is self.RUNNING:
            station = self.manager.choice2station.get(msg.trackID)
            # Just cleared the main line by entering a station.
            if station is not None and vehicle.ts_id in station.ts_ids:
                assert vehicle.berth_id is not None
                vehicle.state = self.SLOWING
                vehicle.station = station


        elif vehicle.state is self.LAUNCHING:
            station = self.manager.track2station.get(msg.trackID)
            # Just joined the main line, entirely exiting the station zone
            if msg.trackID == station.ts_ids[Station.ON_RAMP_II]:
                vehicle.state = self.RUNNING
                vehicle.station = None

class Manager(object): # Similar to VehicleManager in gtf_conroller class
    """Coordinates vehicles to satisfy passenger demand. Handles vehicle pathing."""
    def __init__(self, xml_path, sim_end_time, controller):
        """xml_path: the path (including filename) to the xml scenario file
        created by TrackBuilder.

        The scenario file is expected to have a GoogleTransitFeed section which
        provides vehicle scheduling and trip data, in addition to the typical
        TrackSegment, Station, and Vehicle data."""
        self.controller = controller
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

        self.graph = self.build_graph(doc.getElementsByTagName('TrackSegments')[0])
        self.stations = self.load_stations(doc.getElementsByTagName('Stations')[0], self.graph)

        self.vehicles = self.load_vehicles(doc.getElementsByTagName('Vehicles')[0],
                                           doc.getElementsByTagName('VehicleModels')[0])

        # Mapping from station's trackSegments to a station id.
        self.track2station = dict((ts, s) for s in self.stations.values() for ts in s.ts_ids)

        # Mapping from the choice ts id to the Station instance
        self.choice2station = dict((s.choice, s) for s in self.stations.values())

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
                            station.berth_reservations[platform][idx] = vehicle
                            vehicle.station = station
                            vehicle.berth_pos = berth_pos
                            vehicle.berth_id = idx
                            vehicle.platform_id = platform
                            vehicle.plat_ts = vehicle.ts_id
                            break
        doc.unlink() # facilitates garbage collection

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

            # Discover the bypass TrackSegment id from the graph
            # find the ts that's downstream from the offramp switch, and also upstream from the onramp merge.
            downstream = graph.successors(graph.predecessors(ts_ids[Station.OFF_RAMP_I])[0])
            upstream = graph.predecessors(graph.successors(ts_ids[Station.ON_RAMP_II])[0])
            for ts in downstream:
                if ts in upstream:
                    bypass = ts
                    break

            # Discover the 'choice' TrackSegment id from the graph.
            assert len(graph.predecessors(ts_ids[Station.OFF_RAMP_I])) == 1
            choice = graph.predecessors(ts_ids[Station.OFF_RAMP_I])[0]

            # Calculate the total onramp length
            post_merge = graph.successors(ts_ids[Station.ON_RAMP_II])[0]
            onramp_length = networkx.dijkstra_path_length(self.graph,
                                            ts_ids[Station.ACCEL], post_merge)

            stations[station_int_id] = Station(station_int_id, ts_ids, bypass, choice, onramp_length,
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

            vehicles[vehicle_int_id] = Vehicle(vehicle_int_id,
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
        return vehicles

    def request_trip(self, vehicle):
        """The vehicle must be currently in a station. For best efficiency,
        delay calling this function until the vehicle is ready to load passengers.
        Returns a Trip instance whose distance presumes that vehicle is in the
        origin station's launch berth at the start.
        Propagates a NoPaxAvailableError if no passengers are available at vehicle's current station.
        """
        assert isinstance(vehicle, Vehicle)
        orig_station = vehicle.station
        pax = orig_station.pop_passenger()
        dest_station = self.stations[pax.dest_id]
        path_length, path = self.get_path(orig_station.ts_ids[Station.LOAD], dest_station.ts_ids[Station.UNLOAD])
        dist = path_length - (orig_station.berth_positions[Station.LOAD_PLATFORM][-1] - vehicle.BERTH_GAP)
        return Trip(dest_station, dist, path, (pax.id,) ) # just one pax for now

    def deadhead(self, vehicle, exclude=tuple()):
        """Go to a that has waiting passengers and needs more vehicles. The exclude
        parameter is a tuple of Station instances that the vehicle should not
        pick as a destination.
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
            return Trip(best_station, best_station_dist - vehicle.pos, paths[best_station.ts_ids[Station.UNLOAD]], tuple())
        else: # no demand anywhere
            if vehicle.station is not None: # in a station -- just stay there
                return None
            else: # on the main line, really need someplace to go
                return Trip(closest_station, closest_dist - vehicle.pos, paths[closest_station.ts_ids[Station.UNLOAD]], tuple())

    def wave_off(self, vehicle):
        """Sends a vehicle around in a loop so as to come back and try entering the
        station later, or sends it to a different station if empty."""
        # vehicle has passengers, so must go to this particular station
        if vehicle.trip.passengers:
            prev_tses = self.graph.predecessors(vehicle.ts_id) # there may be a merge just upstream of choice ts
            best_loop_length, best_loop_path = float('inf'), None
            for prev_ts in prev_tses:
                loop_length, loop_path = self.get_path(vehicle.ts_id, prev_ts)
                if loop_length < best_loop_length:
                    best_prev_ts = prev_ts
                    best_loop_length = loop_length
                    best_loop_path = loop_path

            # concatenate that path with a path to the station's unload
            entry_length, entry_path = self.get_path(best_prev_ts, vehicle.trip.dest_station.ts_ids[Station.UNLOAD])

            dist = best_loop_length + entry_length - vehicle.pos # vehicle.pos should be zero, but this code may get moved...
            path = best_loop_path + entry_path[1:] # concat lists. Discard prev_ts, so that it's not duplicated.

            return Trip(vehicle.trip.dest_station, dist, path, vehicle.trip.passengers)

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
        knot_initial = Knot(vehicle.pos, vehicle.vel, vehicle.accel, now)
        knot_final = Knot(vehicle.pos + station.onramp_length,
                          PrtController.LINE_SPEED, 0, None)

        spline = vehicle.traj_solver.target_velocity(knot_initial, knot_final)
        assert spline.q[-1] <= vehicle.pos + station.onramp_length, \
               "Vehicle's accel and jerk settings are not sufficient to reach line speed before merging with the main line. " + \
               "Adjust the network or vehicle settings in the scenario's XML file. %.3f, %.3f" \
               % (spline.q[-1], vehicle.pos + station.onramp_length)
        merge_delay = spline.t[-1] - spline.t[0]
        assert merge_delay > PrtController.HEADWAY

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
        conflict_zone_end_dist = conflict_zone_start_dist + vehicle.length + headway_dist

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

    def __init__(self, s_id, ts_ids, bypass_ts_id, choice_ts_id, onramp_length,
                 unload_positions, queue_positions, load_positions):
        """s_id: An integer station id.
        ts_ids: A list containing integer TrackSegment ids. See Station consts.
        bypass_ts_id: The TrackSegment id which bypasses the station. That is,
                      the main track rather than the station's track.
        choice_ts_id: The TrackSegment id upsteam of both the bypass and the
                        offramp.
        onramp_length: Sum of track segment lengths for segments leading to exit.
        unload_positions, queue_positions, load_positions:
        Each of the above are lists of floats, where each float designates
        a position that the vehicle will target in order to park in the berth.
        i.e. to unload in berth 1, the vehicle will go to the position found in
        unload_positions[1].
        """
        self.id = s_id
        self.ts_ids = ts_ids
        self.bypass = bypass_ts_id
        self.choice = choice_ts_id
        self.onramp_length = onramp_length
        self.berth_positions = [unload_positions, queue_positions, load_positions]
        self.berth_reservations = [[None]*len(unload_positions),
                                   [None]*len(queue_positions),
                                   [None]*len(load_positions)]

        self.passengers = []
        self.v_count = 0

    def add_passenger(self, pax):
        self.passengers.append(pax)

    def pop_passenger(self):
        """Pops the oldest passenger and returns it. Raises NoPaxAvailableError
        if none available."""
        try:
            return self.passengers.pop(0)
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

    def request_berth(self, vehicle, platform_id):
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
        if vehicle.platform_id is not None:
            assert platform_id >= vehicle.platform_id

        if platform_id == vehicle.platform_id:
            curr_berth_id = vehicle.berth_id
        else:
            curr_berth_id = -1

        choosen_idx = None
        for idx, reservation in enumerate(self.berth_reservations[platform_id]):
            if idx <= curr_berth_id:
                continue
            elif reservation is None:
                choosen_idx = idx
            else:
                break

        if choosen_idx is not None:
            self.release_berth(vehicle)
            self.berth_reservations[platform_id][choosen_idx] = vehicle
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

    def release_berth(self, vehicle):
        """Frees the vehicle's current berth for reuse. Alters the vehicle!
        Returns None."""
        assert isinstance(vehicle, Vehicle)
        found = False
        for platform in self.berth_reservations:
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
        if platform_id == self.LOAD_PLATFORM and \
                     berth_id == len(self.berth_reservations[platform_id])-1:
            return True
        else:
            return False

    def is_last_berth(self, berth_id, platform_id):
        """Is berth the last one on the current platform?"""
        if berth_id == len(self.berth_reservations[platform_id])-1:
            return True
        else:
            return False

    def is_empty(self):
        """Returns True if no berths that have been reserved by vehicles."""
        if any(any(self.berth_reservations[self.UNLOAD_PLATFORM]),
               any(self.berth_reservations[self.QUEUE_PLATFORM]),
               any(self.berth_reservations[self.LOAD_PLATFORM])):
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

    controller = None # interfaces with the sim
    manager = None # high level planner

    def __init__(self, v_id, model, length, capacity, ts_id, pos, vel, accel, j_max, j_min, a_max, a_min, v_max):
        self.id = v_id
        self.model = model
        self.capacity = capacity
        self.length = length
        self.ts_id = ts_id
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.j_max = j_max
        self.j_min = j_min
        self.a_max = a_max
        self.a_min = a_min
        self.v_max = v_max

        self.pax = []
        self.path = []
        self.spline = None
        self.traj_solver = TrajectorySolver(self.v_max, self.a_max, self.j_max,
                                            -5, self.a_min, self.j_min)
        self.last_update = 0.0
        self.trip = None
        self.state = None
        self.station = None

        # Info for the reserved berth, not the current berth!!
        self.platform_id = None
        self.berth_pos = None
        self.berth_id = None
        self.plat_ts = None

        # A private flag for indicating whether a vehicle's "advance" has been
        # commanded yet or not.
        self._advance_begun = True
        self._launch_begun = True

    def update_vehicle(self, v_status, time):
        """Updates the relevant vehicle data."""
        assert v_status.vID == self.id
        self.ts_id = int(v_status.nose_locID) # convert to int from a long
        self.pos = v_status.nose_pos
        self.vel = v_status.vel
        self.accel = v_status.accel
        self.pax = v_status.passengerIDs[:] # copy
        self.last_update = time

    def run_trip(self):
        """Runs the vehicle's stored trip. Returns the scheduled arrival time."""
        assert isinstance(self.trip, Trip)
        t = self.run(self.trip.dist, self.trip.path, self.trip.dest_station.SPEED_LIMIT - 0.1, self.controller.LINE_SPEED) # slightly under the speed limit
        return t

    def run(self, dist, path, final_speed=0, speed_limit=None, clear=True):
        """Commands the vehicle's path and trajectory. Obeys the speed_limit
        constraint, if supplied. Assumes that the vehicle's pose is up to date
        and accurate. Set clear to False if the vehicle's path should be extended
        instead of being changed from what was previously commanded.
        Returns the scheduled arrival time."""
        if path and self.path:
            assert path[0] == self.path[-1]
            self.path.extend(path[1:])

        if path[1:]: # only send the msg if it contains information
            itinerary_msg = api.CtrlCmdVehicleItinerary()
            itinerary_msg.vID = self.id
            itinerary_msg.trackIDs.extend(path[1:]) # don't include the segment self is currently on
            itinerary_msg.clear = clear
            self.controller.send(api.CTRL_CMD_VEHICLE_ITINERARY, itinerary_msg)

        if speed_limit:
##            if self.vel > speed_limit:
##                # Bumping the speed_limit by .001 is a workaround for a bug that manifests when the
##                # initial velocity is just *slightly* above the velocity constraint.
##                speed_limit += 0.001
            solver = TrajectorySolver(min(speed_limit, self.v_max), self.a_max, self.j_max, -5, self.a_min, self.j_min)
        else:
            solver = self.traj_solver

        spline = solver.target_position(Knot(self.pos, self.vel, self.accel, self.controller.current_time),
                                        Knot(dist + self.pos, final_speed, 0, None))
        if speed_limit:
            assert spline.get_max_velocity() < speed_limit + 5, (spline.get_max_velocity(), speed_limit)

        if abs(spline.v[-1] - final_speed) > 0.001 or abs(spline.a[-1]) > 0.001:
            from pyprt.shared.cspline_plotter import CSplinePlotter
            plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min)
            plotter.display_plot()
            raise Exception('Targetted velocity or accel not met.')

##        from pyprt.shared.cspline_plotter import CSplinePlotter
##        plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min)
##        plotter.display_plot()

        final_time = spline.t[-1]
        # Append a last knot which extends the spline until past the end of the simulation.
        if spline.t[-1] <= self.controller.sim_end_time:
            spline.append(Knot(dist+self.pos+final_speed*(self.controller.sim_end_time+1-final_time),
                                final_speed, 0, self.controller.sim_end_time+1))

        traj_msg = api.CtrlCmdVehicleTrajectory()
        traj_msg.vID = self.id
        spline.fill_spline_msg(traj_msg.spline)
        self.controller.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, traj_msg)

        return final_time

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
            path = []
            dist = (self.berth_pos - self.BERTH_GAP) - self.pos # stop a little short of the end

        else: # advancing to another next platform
            path_length, path = self.manager.get_path(self.ts_id, self.plat_ts)
            assert len(path) < 10, path # sanity check
            dist = (path_length - self.pos) + (self.berth_pos - self.BERTH_GAP) # stop a little short of the end

        if speed_limit == None:
            speed_limit = self.station.SPEED_LIMIT

        finish_time = self.run(dist, path, 0, speed_limit)
        return finish_time

class Trip(object):
    def __init__(self, dest_station, dist, path, passengers):
        """A data holding object for vehicle trip info.
        dest_station: the destination Station instance
        dist: The distance to travel.
        path: first element is the vehicle's location at the intended beginning
              of the trip.
        passengers: a tuple of passenger ids.
        """
        assert isinstance(dest_station, Station)
        self.dest_station = dest_station
        self.dist = dist
        self.path = path
        self.passengers = passengers
        self.dest_ts = path[-1]

if __name__ == '__main__':
    main()
