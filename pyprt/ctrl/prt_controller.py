from __future__ import division
import optparse
from collections import defaultdict
from operator import attrgetter
import warnings

import networkx

import pyprt.shared.api_pb2 as api
from pyprt.ctrl.base_controller import BaseController
from trajectory_solver import TrajectorySolver
from pyprt.shared.cubic_spline import Knot

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
    options, args = options_parser.parse_args()

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

    LINE_SPEED = 25  # in meter/sec
    HEARTBEAT_INTERVAL = 5  # in seconds. Choosen arbitrarily for testing

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
            assert vehicle not in v_list
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
                        v.disembark(v.pax[:]) # pass a copy of the list

                    elif v.state is self.LOAD_WAITING:
                        try:
                            v.trip = self.manager.request_trip(v)
                            v.state = self.EMBARKING
                            v.embark(v.trip.passengers)
                        except NoPaxAvailableError:
                            pass # no passengers avail... # TODO: Deadhead to another station

                    elif v.state is self.EXIT_WAITING:
                        if station.is_launch_berth(v.berth_id, v.platform_id):
                            v.state = self.LAUNCH_WAITING
                            # Temp -- just launch as soon as possible
                            v.state = self.LAUNCHING
                            v._launching = False

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
        self.request_v_status(vehicle.id)

        if vehicle.state not in (self.UNLOAD_WAITING, self.QUEUE_WAITING, self.LOAD_WAITING, self.EXIT_WAITING, None):
            return

        # if vehicle hasn't unloaded, only try to advance as far as the last unload berth
        if vehicle.state is self.UNLOAD_WAITING:
            try:
                b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.UNLOAD_PLATFORM, vehicle.berth_id)
                plat_id = Station.UNLOAD_PLATFORM
            except NoBerthAvailableError:
                return

        # All remaining cases should advance as far as the load platform, if able
        if vehicle.platform_id == Station.UNLOAD_PLATFORM:
            try: # see if vehicle can go all the way to loading platform
                b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.LOAD_PLATFORM)
                plat_id = Station.LOAD_PLATFORM
            except NoBerthAvailableError: # Settle for trying to get into the queue
                try:
                    b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.QUEUE_PLATFORM)
                    plat_id = Station.QUEUE_PLATFORM
                except NoBerthAvailableError: # Can I at least inch forward on UNLOAD_PLATFORM?
                    try:
                        b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.UNLOAD_PLATFORM, vehicle.berth_id)
                        plat_id = Station.UNLOAD_PLATFORM
                    except NoBerthAvailableError:
                        return

        elif vehicle.platform_id == Station.QUEUE_PLATFORM:
            try:
                b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.LOAD_PLATFORM)
                plat_id = Station.LOAD_PLATFORM
            except NoBerthAvailableError: # Settle for staying on queue
                try:
                    b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.QUEUE_PLATFORM, vehicle.berth_id)
                    plat_id = Station.QUEUE_PLATFORM
                except NoBerthAvailableError:
                    return

        elif vehicle.platform_id == Station.LOAD_PLATFORM:
            try:
                b_pos, b_id, plat_ts = station.request_berth(vehicle, Station.LOAD_PLATFORM, vehicle.berth_id)
                plat_id = Station.LOAD_PLATFORM
            except NoBerthAvailableError:
                return

        # update vehicle's state
        if vehicle.state is self.UNLOAD_WAITING:
            vehicle.state = self.UNLOAD_ADVANCING
        elif vehicle.state is self.QUEUE_WAITING:
            # Change from QUEUE_* to LOAD_* once vehicle has made it to load platform
            if plat_id == Station.UNLOAD_PLATFORM or plat_id == Station.QUEUE_PLATFORM:
                vehicle.state = self.QUEUE_ADVANCING
            elif plat_id == Station.LOAD_PLATFORM:
                vehicle.state = self.LOAD_ADVANCING
            else:
                raise Exception("Unknown platform_id: %d" % plat_id)
        elif vehicle.state is self.LOAD_WAITING:
            vehicle.state = self.LOAD_ADVANCING
        elif vehicle.state is self.EXIT_WAITING:
            vehicle.state = self.EXIT_ADVANCING
        else:
            assert False, "Unexpected state: %s" % vehicle.state

        # store the reserved berth with the vehicle
        vehicle.station.release_berth(vehicle.berth_id, vehicle.platform_id)
        vehicle.berth_pos = b_pos
        vehicle.berth_id = b_id
        vehicle.platform_id = plat_id
        vehicle.plat_ts = plat_ts

        # ... wait for the v_status update from the sim

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
                    self.trigger_advance(v, station)

            for v in reversed(station.berth_reservations[Station.QUEUE_PLATFORM]):
                if v is not None:
                    v.state = self.QUEUE_WAITING
                    self.trigger_advance(v, station)

            for v in reversed(station.berth_reservations[Station.UNLOAD_PLATFORM]):
                if v is not None:
                    v.state = self.QUEUE_WAITING # Assuming vehicles don't start with passengers on board
                    self.trigger_advance(v, station)

        for vehicle in self.manager.vehicles.itervalues():
            assert isinstance(vehicle, Vehicle)
            # skip already handled vehicles
            if vehicle.state is not None:
                continue

            station_id = self.manager.track2station.get(vehicle.ts_id)

            # Within the boundaries of a station, but not at a berth.
            if station_id is not None:
                station = self.manager.stations[station_id]
                assert isinstance(station, Station)

                # Approaching station
                if vehicle.ts_id in (station.ts_ids[Station.OFF_RAMP_I],
                                     station.ts_ids[Station.OFF_RAMP_II],
                                     station.ts_ids[Station.DECEL]):
                    vehicle.state = self.SLOWING

                    # Reserve a berth for the vehicle
                    berth_pos, berth_id, plat_ts = station.request_berth(vehicle, Station.UNLOAD)
                    vehicle.berth_pos = berth_pos
                    vehicle.berth_id = berth_id
                    vehicle.platform_id = Station.UNLOAD_PLATFORM
                    vehicle.plat_ts = plat_ts
                    vehicle.station = station

                    # Set up a trajectory for reaching the station's UNLOAD platform.
                    path_length, path = self.manager.get_path(vehicle.ts_id, station.ts_ids[Station.UNLOAD])
                    dist = path_length - vehicle.pos
                    t_arrival = vehicle.run(dist, path, station.SPEED_LIMIT - 0.1) # a little under the speed limit

                # Outbound from a station
                else:
                    assert vehicle.ts_id in (station.ts_ids[Station.ACCEL],
                                             station.ts_ids[Station.ON_RAMP_I],
                                             station.ts_ids[Station.ON_RAMP_II])
                    vehicle.state = self.LAUNCHING
                    dest_station_id, dist, path = self.manager.deadhead(vehicle.id)
                    vehicle.run(dist, path, Station.SPEED_LIMIT, self.LINE_SPEED)

            # Not inside a station's boundaries, or outbound from a station
            else:
                dest_station_id, dist, path = self.manager.deadhead(vehicle.id)
                vehicle.state = self.RUNNING
                vehicle.run(dist, path, Station.SPEED_LIMIT, self.LINE_SPEED)

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
            else:
                warnings.warn("Huh? What was this reminder for again? vehicle %d, state: %s" % (vehicle.id, vehicle.state))
##                raise Exception("Huh? What was this reminder for again? Current State: %s" % vehicle.state)

        for station in stations:
            self.heartbeat(station)

        del self.t_reminders[ms_msg_time]

    def on_SIM_RESPONSE_VEHICLE_STATUS(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        if vehicle.state in (self.UNLOAD_ADVANCING, self.QUEUE_ADVANCING,
                             self.LOAD_ADVANCING, self.EXIT_ADVANCING):
            # TODO: Move to on_NOTIFY_VEHICLE_STOPPED when message is implemented
            if abs(vehicle.vel) < 0.01 and abs(vehicle.accel) < 0.01: # check that we're stopped

                if vehicle._advancing is False: # advancing hasn't started yet
                    assert vehicle.ts_id != vehicle.plat_ts or abs(vehicle.berth_pos - vehicle.BERTH_GAP - vehicle.pos) > 1
                    t_arrival = vehicle.advance() # sets ._advancing flag to true
                    self.set_v_notification(vehicle, t_arrival)

                elif vehicle._advancing is True: # flag indicates that advancing has completed
                    # Really need a NOTIFY STOPPED message to clean up this flow
                    if abs(vehicle.berth_pos - vehicle.BERTH_GAP - vehicle.pos) < 0.1 and vehicle.ts_id == vehicle.plat_ts and vehicle.vel < 0.001:
                        vehicle._advancing = False # lower the flag
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
                    raise Exception("Unexpected case: %s" % vehicle._advancing)

        elif vehicle.state is self.LAUNCHING:
            if vehicle._launching is False:
                vehicle.run_trip()
                vehicle.station.release_berth(vehicle.berth_id, vehicle.platform_id)
                vehicle.berth_id = None
                vehicle.berth_pos = None
                vehicle.platform_id = None
                vehicle.plat_ts = None
                vehicle._launching = True
            else:
                pass

##        else:
##            warnings.warn("Received vehicle status. Not sure why... vID: %d, v.state: %s" % (vehicle.id, vehicle.state))

    def on_SIM_EVENT_PASSENGER_CREATED(self, msg, msgID, msg_time):
        p_status = msg.p_status
        pax = Passenger(p_status.pID, p_status.src_stationID,
                        p_status.dest_stationID, p_status.creation_time)
        self.manager.add_passenger(pax)

    def on_SIM_COMPLETE_PASSENGERS_DISEMBARK(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is self.DISEMBARKING
        self.log.info("t:%5.3f Vehicle %d at station %d, berth %d, plat %d has completed disembark of %s",
                       self.current_time, vehicle.id, vehicle.station.id, vehicle.berth_id, vehicle.platform_id, msg.cmd.passengerIDs)

        vehicle.state = self.QUEUE_WAITING
##        self.trigger_advance(vehicle, vehicle.station) # Start advancing to the loading platform

    def on_SIM_COMPLETE_PASSENGERS_EMBARK(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is self.EMBARKING
        self.log.info("t:%5.3f Vehicle %d at station %d, berth %d, plat %d has completed embark of %s",
                       self.current_time, vehicle.id, vehicle.station.id, vehicle.berth_id, vehicle.platform_id, msg.cmd.passengerIDs)

        vehicle.state = self.EXIT_WAITING
##        self.trigger_advance(vehicle, vehicle.station) # Start advancing to the exit berth
##        if vehicle.state is self.READY_TO_DEPART:
##            station = vehicle.station
##            if station.is_launch_berth(vehicle.berth_id, vehicle.platform_id):
##                vehicle.state = self.RUNNING
##                vehicle.run_trip()
##                station.release_berth(vehicle.berth_id, vehicle.platform_id)
##                vehicle.berth_id = None
##                vehicle.berth_pos = None
##                vehicle.platform_id = None

    def on_SIM_NOTIFY_VEHICLE_ARRIVE(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        if vehicle.state is self.SLOWING and vehicle.ts_id == vehicle.station.ts_ids[Station.UNLOAD]:
            # Just reached the beginning of the unload platform. Go to the already reserved berth.
            if vehicle.pax:
                vehicle.state = self.UNLOAD_ADVANCING
            else:
                vehicle.state = self.QUEUE_ADVANCING

            t_arrival = vehicle.advance()
            self.set_v_notification(vehicle, t_arrival)

    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        vehicle = self.manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        station_id = self.manager.track2station.get(msg.trackID)
        if station_id is not None:
            station = self.manager.stations[station_id]

        # Within a station's zone
        if vehicle.state is self.RUNNING and station_id is not None:
            # Just left the main line to enter a station.
            if msg.trackID == station.ts_ids[Station.OFF_RAMP_I]:
                # request a berth TODO: Request a berth BEFORE committing to entering the station! Also, try-except
                berth_pos, berth_id, plat_ts = station.request_berth(vehicle, Station.UNLOAD_PLATFORM)
                vehicle.state = self.SLOWING
                vehicle.berth_pos = berth_pos
                vehicle.berth_id = berth_id
                vehicle.platform_id = station.UNLOAD_PLATFORM
                vehicle.plat_ts = plat_ts
                vehicle.station = station

        # Just joined the main line, entirely exiting the station zone
        elif vehicle.state is self.LAUNCHING and vehicle.ts_id == station.ts_ids[Station.ON_RAMP_II]:
            vehicle._launching = False
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

        # Mapping from station id to the station's trackSegments.
        self.station2tracks = dict((s.id, s.ts_ids) for s in self.stations.values())

        # Mapping from station's trackSegments to a station id.
        self.track2station = dict((ts, s.id) for s in self.stations.values() for ts in s.ts_ids)

        # For vehicles that are currently parked in berths, reserve those berths for them.
        for vehicle in self.vehicles.itervalues():
            station_id = self.track2station.get(vehicle.ts_id)
            if station_id is not None:
                station = self.stations[station_id]
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
                            break
        doc.unlink() # facilitates garbage collection

        # keys are station ids, values are lists of passengers originating there
        self.passengers = defaultdict(list)

        # keys are station ids, values are lists of waiting (empty) vehicles at station
        self.idle = defaultdict(list) # idle vehicles

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

            stations[station_int_id] = Station(station_int_id, ts_ids, bypass,
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
            capacity = int(model_xml.getAttribute('passenger_capacity'))
            jerk_xml = model_xml.getElementsByTagName('Jerk')[0]
            accel_xml = model_xml.getElementsByTagName('Acceleration')[0]
            vel_xml = model_xml.getElementsByTagName('Velocity')[0]

            vehicles[vehicle_int_id] = Vehicle(vehicle_int_id,
                                               model_name,
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
        Returns a Trip instance.
        Raises a NoPaxAvailableError if, surprise!, no passengers are available.
        """
        assert isinstance(vehicle, Vehicle)
        orig_station = vehicle.station
        avail_pax = self.passengers[orig_station.id]
        if avail_pax: # passenger available at vehicle's current station
            avail_pax.sort(key=attrgetter('origin_time'), reverse=True) # put the oldest pax at the end
            pax = avail_pax.pop()
            dest_station = self.stations[pax.dest_id]
            path_length, path = self.get_path(orig_station.ts_ids[Station.LOAD], dest_station.ts_ids[Station.UNLOAD])
            dist = path_length - (orig_station.berth_positions[Station.UNLOAD_PLATFORM][-1] - vehicle.BERTH_GAP)
            return Trip(dest_station, dist, path, (pax.id,) ) # just one pax for now

        else: # vehicle must either wait, or travel to a different station
            raise NoPaxAvailableError

    def deadhead(self, vehicle_id):
        """Go to the nearest station that has waiting passengers. Returns a
        3-tuple: (station_id, dist, path)
        where path leads to the station's unloading platform.
        """
        vehicle = self.vehicles[vehicle_id]
        dists, paths = networkx.single_source_dijkstra(self.graph, vehicle.ts_id)
        closest_dist = float('inf')
        closest_station = None
        has_pax = False
        for station in self.stations.itervalues():
            station_ts = station.ts_ids[Station.UNLOAD]
            dist = dists[station_ts]
            if dist < closest_dist:
                if self.passengers[station.id]: # prefer that the station have waiting pax
                    has_pax = True
                    closest_station = station
                    closest_dist = dist
                elif has_pax is False: # only consider stations without passengers if none found with pax
                    closest_station = station
                    closest_dist = dist

        return closest_station.id, closest_dist - vehicle.pos, paths[closest_station.ts_ids[Station.UNLOAD]]

    def add_passenger(self, pax):
        assert isinstance(pax, Passenger)
        self.passengers[pax.origin_id].append(pax)

    def get_path(self, ts_1, ts_2):
        """Returns a two tuple. The first element is the path length (in meters),
        and the second is the path. ts_1 and ts_2 are integer trackSegment ids."""
        return networkx.bidirectional_dijkstra(self.graph, ts_1, ts_2)

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

    SPEED_LIMIT = 2.5 # m/s (approx 5 mph)
    controller = None

    def __init__(self, s_id, ts_ids, bypass_ts_id, unload_positions,
                 queue_positions, load_positions):
        """s_id: An integer station id.
        ts_ids: A list containing integer TrackSegment ids. See Station consts.
        bypass_ts_id: The TrackSegment id which bypasses the station. That is,
                      the main track rather than the station's track.

        unload_positions, queue_positions, load_positions:
        Each of the above are lists of floats, where each float designates
        a position that the vehicle will target in order to park in the berth.
        i.e. to unload in berth 1, the vehicle will go to the position found in
        unload_positions[1].
        """
        self.id = s_id
        self.ts_ids = ts_ids
        self.bypass = bypass_ts_id
        self.berth_positions = [unload_positions, queue_positions, load_positions]
        self.berth_reservations = [[None]*len(unload_positions),
                                   [None]*len(queue_positions),
                                   [None]*len(load_positions)]

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

    def request_berth(self, vehicle, platform_id, curr_berth_id=-1):
        """Finds an accessible, available berth which is as close to the station exit as
        possible, and reserves it for use by vehicle.:
            (berth_position, berth_id, ts_id)
        platform: one of the station consts: *_PLATFORM
        Raises NoBerthAvailableError if no berths are available on the requested platform.
        """
        choosen_idx = None
        for idx, reservation in enumerate(self.berth_reservations[platform_id]):
            if idx <= curr_berth_id:
                continue
            elif reservation is None:
                choosen_idx = idx
            else:
                break

        if choosen_idx is not None:
            self.berth_reservations[platform_id][choosen_idx] = vehicle
            return (self.berth_positions[platform_id][choosen_idx], choosen_idx, self.ts_ids[platform_id+3]) # +3 maps platform_id to ts const
        else: # no berth available
            raise NoBerthAvailableError()

    def release_berth(self, berth_id, platform_id):
        """Frees the berth for reuse.
        platform: one of the station consts: *_PLATFORM
        Returns None."""
        self.berth_reservations[platform_id][berth_id] = None

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
        if any(any(self.berth_reservations[self.UNLOAD_PLATFORM]),
               any(self.berth_reservations[self.QUEUE_PLATFORM]),
               any(self.berth_reservations[self.LOAD_PLATFORM])):
            return False
        else:
            return True

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

    def __init__(self, v_id, model, capacity, ts_id, pos, vel, accel, j_max, j_min, a_max, a_min, v_max):
        self.id = v_id
        self.model = model
        self.capacity = capacity
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
        self._advancing = False
        self._launching = False

    def update_vehicle(self, v_status):
        """Updates the relevant vehicle data."""
        assert v_status.vID == self.id
        self.ts_id = int(v_status.nose_locID) # convert to int from a long
        self.pos = v_status.nose_pos
        self.vel = v_status.vel
        self.accel = v_status.accel
        self.pax = v_status.passengerIDs[:] # copy

    def run_trip(self):
        """Runs the vehicle's stored trip. Returns the scheduled arrival time."""
        assert isinstance(self.trip, Trip)
        t = self.run(self.trip.dist, self.trip.path, self.trip.dest_station.SPEED_LIMIT - 0.1, self.controller.LINE_SPEED) # slightly under the speed limit
        return t

    def run(self, dist, path, final_speed=0, speed_limit=None):
        """Commands the vehicle's path and trajectory. Obeys the speed_limit
        constraint, if supplied. Assumes that the vehicle's pose is up to date
        and accurate. Returns the scheduled arrival time."""
        if path and self.path:
            assert path[0] == self.path[-1]
            self.path.extend(path[1:])

        if path[1:]: # only send the msg if it contains information
            itinerary_msg = api.CtrlCmdVehicleItinerary()
            itinerary_msg.vID = self.id
            itinerary_msg.trackIDs.extend(path[1:]) # don't include the segment self is currently on
            itinerary_msg.clear = False
            self.controller.send(api.CTRL_CMD_VEHICLE_ITINERARY, itinerary_msg)

        if speed_limit:
            solver = TrajectorySolver(min(speed_limit, self.v_max), self.a_max, self.j_max, -5, self.a_min, self.j_min)
        else:
            solver = self.traj_solver

        spline = solver.target_position(Knot(self.pos, self.vel, self.accel, self.controller.current_time),
                                        Knot(dist + self.pos, final_speed, 0, None))

        if abs(spline.v[-1] - final_speed) > 0.001 or abs(spline.a[-1]) > 0.001:
            from pyprt.shared.cspline_plotter import CSplinePlotter
            plotter = CSplinePlotter(spline, solver.v_max, solver.a_max, solver.j_max, solver.v_min, solver.a_min, solver.j_min)
            plotter.display_plot()

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
        self._advancing = True
        if self.plat_ts == self.ts_id: # staying on the same platform
            path = []
            dist = (self.berth_pos - self.BERTH_GAP) - self.pos # stop a little short of the end

        else: # advancing to another next platform
            path_length, path = self.manager.get_path(self.ts_id, self.plat_ts)
            dist = (path_length - self.pos) + (self.berth_pos - self.BERTH_GAP) # stop a little short of the end

        if speed_limit == None:
            speed_limit = self.station.SPEED_LIMIT

        finish_time = self.run(dist, path, 0, speed_limit)
        return finish_time


class Trip(object):
    def __init__(self, dest_station, dist, path, passengers):
        """A data holding object for vehicle trip info.
        dest_station: the destination Station instance
        dist: from the beginning of the origin station's ACCEL track seg
                   to the beginning of the destination station's unload platform
        path: first element is origin station's ACCEL track seg id,
              last element is destination station's unload platform ts id.
        passengers: a tuple of passenger ids.
        """
        self.dest_station = dest_station
        self.dist = dist
        self.path = path
        self.passengers = passengers
        self.dest_ts = path[-1]

if __name__ == '__main__':
    main()
