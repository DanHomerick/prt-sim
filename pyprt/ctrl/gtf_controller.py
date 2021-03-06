"""A controller which attempts to run vehicles on schedules and routes
described by Google Transit Feed data.

Notes:
    - The simulation begins at the start of the feed.
"""
from __future__ import division
import optparse
from collections import defaultdict
from operator import attrgetter
import warnings

import networkx
from numpy import zeros

import pyprt.shared.api_pb2 as api
from base_controller import BaseController
from trajectory_solver import TrajectorySolver, FatalTrajectoryError
from pyprt.shared.cubic_spline import Knot, CubicSpline
from pyprt.shared.utility import pairwise

def main():
    options_parser = optparse.OptionParser(usage="usage: %prog [options]")
    options_parser.add_option("--logfile", dest="logfile", default="./ctrl.log",
                              metavar="FILE", help="Log events to FILE.")
    options_parser.add_option("--comm_logfile", dest="comm_logfile", default="./ctrl_comm.log",
                              metavar="FILE", help="Log communication messages to FILE.")
    options_parser.add_option("--server", dest="server", default="localhost",
                              help="The IP address of the server (simulator). Default is %default")
    options_parser.add_option("-p", "--port", type="int", dest="port", default=64444,
                              help="TCP Port to connect to. Default is: %default")
    options_parser.add_option("--walk_speed", type="float", dest="walk_speed", default=1.33,
                              help="Speed, in meters/sec, at which passengers will walk to a neighboring station. A value of 0 will disable passenger walking entirely.")
    options, args = options_parser.parse_args()

    if len(args) != 0:
        options_parser.error("Expected zero positional arguments. Received: %s" % ' '.join(args))

    ctrl = GtfController(options.logfile, options.comm_logfile, options.walk_speed)
    ctrl.connect(options.server, options.port)

def to_seconds(time_str):
    assert len(time_str) == 8
    hour, minute, sec = [int(t) for t in time_str.split(':')]
    return sec + minute*60 + hour*3600

class GtfController(BaseController):
    # States (for state-machine)
    RUNNING_LEG = "RUNNING_LEG"             # Travelling from one station to the next.
    PARKING = "PARKING"                     # Station berth is reserved. Moving to the berth. Not necessarily on station tracksegment.
    ARRIVED = "ARRIVED"                     # Parked in berth. May have passengers to unload.
    DISEMBARKING = "DISEMBARKING"           # Parked in berth, unloading passengers
    WAITING = "WAITING"                     # Parked in berth. No passengers, no current activity.
    FIRST_CALL = "FIRST_CALL"               # Parked in berth, loading first round of passengers.
    WAITING_WITH_PAX = "WAITING_WITH_PAX"   # Parked in berth, waiting for departure time, may have passengers, need to make a last call before departing.
    LAST_CALL = "LAST_CALL"                 # Parked in berth, loading last round of passengers.
    READY_TO_DEPART = "READY_TO_DEPART"     # Parked in berth, waiting for departure time, ready to go immediately

    def __init__(self, log_path, commlog_path, walk_speed):
        super(GtfController, self).__init__(log_path, commlog_path)
        self.t_reminders = defaultdict(list) # keyed by time, values are lists of vehicle ids
        self._walk_speed = walk_speed

        # self.v_manager and self.p_manager are instantiated upon
        # receipt of a SIM_GREETING msg.
        self.v_manager = None   # VehicleManager
        self.p_manager = None   # PassengerManager

    def set_notification(self, vehicle_id, time):
        notify = api.CtrlSetnotifyTime()
        notify.time = time
        self.send(api.CTRL_SETNOTIFY_TIME, notify)
        self.t_reminders[int(notify.time*1000)].append(vehicle_id)

    def do_embark(self, vehicle):
        """Loads passengers in vehicle, handles overflow passengers. 'vehicle'
        is a Vehicle instance, not an id."""
        leg = vehicle.leg
        passengers = self.p_manager.fetch_embarking_pax(vehicle.id, leg.origin_station, leg.depart)
        overflow_passengers = vehicle.embark(passengers)
        self.log.info("T=%4.3f from %d embarked %s Overflow passengers %s " % (self.current_time,leg.origin_station, passengers, overflow_passengers))

        origin_time = max(self.current_time, leg.depart+1)
        for pax in overflow_passengers:
            self.p_manager.reschedule_pax(pax, leg.origin_station, origin_time)
        if overflow_passengers:
            self.log.info("t:%5.3f Overflow passengers at station %d recreated at time %5.3f : %s",
                          self.current_time, leg.origin_station, origin_time, overflow_passengers)

    ### Overriden message handlers ###
    def on_SIM_GREETING(self, msg, msgID, msg_time):
        self.sim_end_time = msg.sim_end_time
        self.log.info("Sim Greeting message received. Sim end at: %f" % msg.sim_end_time)

        self.v_manager = VehicleManager(msg.scenario_xml, msg.sim_end_time, self)
        Vehicle.manager = self.v_manager
        Vehicle.controller = self

        self.p_manager = PassengerManager(msg.scenario_xml, msg.sim_end_time, self._walk_speed, self)

    def on_SIM_START(self, msg, msgID, msg_time):
        self.log.info("Sim Start message received.")

        for vehicle in self.v_manager.vehicles.values():
            leg = self.v_manager.pop_leg(vehicle.id)
            vehicle.leg = leg
            assert isinstance(leg, Leg)

            if leg is None:
                vehicle.state = self.WAITING
                continue # vehicle never does nothin'

            # parked at station. Either departing immediately, or later.
            elif leg.depart >= 0.0:
                vehicle.state = self.FIRST_CALL
                self.do_embark(vehicle)

            # nearly at dest station, past the point where we normally reserve a berth and park
            elif leg.depart < 0 and \
                 (vehicle.ts_id == leg.dest_ts or vehicle.ts_id in self.v_manager.graph.predecessors(leg.dest_ts)):
                vehicle.state = self.PARKING
                station = self.v_manager.stations[leg.dest_station]
                berth_pos, berth_id, dest_ts = station.reserve_berth(vehicle.id, Station.UNLOAD_PLATFORM)
                t_arrival = vehicle.park(berth_pos, berth_id, dest_ts)

                # use time-based notification of vehicle stopping in berth
                # TODO: Add a SIM_NOTIFY_VEHICLE_STOPPED event message.
                self.set_notification(vehicle.id, t_arrival+1)

            # not at dest station or it's offramp
            elif leg.depart < 0:
                vehicle.state = self.RUNNING_LEG
                vehicle.run_leg()

            else:
                raise Exception("Unexpected case.")

            self.log.debug("t:%5.3f Vehicle %d starts sim at pos: %7.3f, ts_id: %d in %s state",
                           self.current_time, vehicle.id, vehicle.pos, vehicle.ts_id, vehicle.state)

    def on_SIM_EVENT_PASSENGER_CREATED(self, msg, msgID, msg_time):
        p_status = msg.p_status
        pax = Passenger(p_status.pID, p_status.src_stationID,
                        p_status.dest_stationID, p_status.creation_time)
        self.p_manager.add_passenger(pax)

    def on_SIM_NOTIFY_TIME(self, msg, msgID, msg_time):
        ms_msg_time = int(msg.time*1000) # millisec integer for easy comparison
        v_ids = self.t_reminders[ms_msg_time]

        for v_id in v_ids:
            vehicle = self.v_manager.vehicles[v_id]
            if vehicle.state is self.PARKING:
                # vehicle should be done parking. Get a status update to make sure.
                req_v_status = api.CtrlRequestVehicleStatus()
                req_v_status.vID = v_id
                self.send(api.CTRL_REQUEST_VEHICLE_STATUS, req_v_status)

            elif vehicle.state is self.WAITING_WITH_PAX:
                # Do last call
                vehicle.state = self.LAST_CALL
                self.do_embark(vehicle)

            else:
                raise Exception("Huh? What was this reminder for again? Current State: %s" % vehicle.state)
        self.t_reminders[ms_msg_time] = []

    def on_SIM_RESPONSE_VEHICLE_STATUS(self, msg, msgID, msg_time):
        vehicle = self.v_manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        if vehicle.state is self.PARKING:
            # TODO: Move to on_NOTIFY_VEHICLE_STOPPED when message is implemented
            if abs(vehicle.vel) < 0.001 and vehicle.leg.dest_ts == vehicle.ts_id:
                vehicle.state = self.DISEMBARKING
                pax = self.p_manager.fetch_disembarking_pax(vehicle.id,
                                                            vehicle.leg.dest_station,
                                                            vehicle.leg.arrive)
                vehicle.disembark(pax)
            else:
                raise Exception("TODO: Parking trajectory was incorrect, and vehicle did not come to a full stop. vID: %d, vel: %f" % (vehicle.id, vehicle.vel))
        else:
            warnings.warn("Received vehicle status. Not sure why... vID: %d, v.state: %s" % (vehicle.id, vehicle.state))

    def on_SIM_COMPLETE_PASSENGERS_DISEMBARK(self, msg, msgID, msg_time):
        vehicle = self.v_manager.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is self.DISEMBARKING
        vehicle.state = self.WAITING

        # Disembarking completes the old leg. Get a new leg.
        leg = self.v_manager.pop_leg(msg.cmd.vID)
        vehicle.leg = leg

        if leg:
            if self.current_time < leg.depart:
                vehicle.state = self.FIRST_CALL
            else:
                vehicle.state = self.LAST_CALL

            self.do_embark(vehicle)

    def on_SIM_COMPLETE_PASSENGERS_EMBARK(self, msg, msgID, msg_time):
        vehicle = self.v_manager.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state in (self.FIRST_CALL, self.LAST_CALL)
        leg = vehicle.leg

        # Embarking may occur twice for a given vehicle and leg. The first (optional)
        # phase loads any passengers who are waiting at the station. The second
        # phase (the 'last call') occurs right before the vehicle leaves,
        # and it loads the passengers who have shown up in the intervening time.
        # The first phase may occur before the departure time. The last call
        # phase always occurs >= the departure time.
        if vehicle.state is self.FIRST_CALL:
            vehicle.state = self.WAITING_WITH_PAX

            if self.current_time >= leg.depart: # Do last call
                vehicle.state = self.LAST_CALL
                self.do_embark(vehicle)

            else:
                self.set_notification(vehicle.id, leg.depart)

        elif vehicle.state is self.LAST_CALL:
            vehicle.state = self.RUNNING_LEG
            vehicle.run_leg()

        else:
            raise NotImplementedError

    def on_SIM_NOTIFY_VEHICLE_ARRIVE(self, msg, msgID, msg_time):
        vehicle = self.v_manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        # When the vehicle reaches the offramp for it's destination station,
        # reserve a berth, and update the trajectory.
        if vehicle.state is self.RUNNING_LEG and vehicle.ts_id == vehicle.path[-2]:
            vehicle.state = self.PARKING
            station = self.v_manager.stations[vehicle.leg.dest_station]
            berth_pos, berth_id, dest_ts = station.reserve_berth(vehicle.id, Station.UNLOAD_PLATFORM)
            t_arrival = vehicle.park(berth_pos, berth_id, dest_ts)
            self.set_notification(vehicle.id, t_arrival+1)

    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        vehicle = self.v_manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        if vehicle.state is self.RUNNING_LEG and msg.trackID == vehicle.leg.origin_ts:
            if vehicle.berth_id is not None:
                station_id = self.v_manager.track2station[vehicle.leg.origin_ts]
                station = self.v_manager.stations[station_id]
                station.release_berth(vehicle.berth_id, vehicle.platform_id)
                vehicle.platform_id = None
                vehicle.berth_id = None
                vehicle.berth_pos = None

class VehicleManager(object):
    """Reads schedule information from the gtf information in the scenario.
    Provides pathing information for the vehicles."""

    def __init__(self, scenario_xml, sim_end_time, controller):
        """xml_path: The xml scenario data created by TrackBuilder, in string form.

        The scenario file is expected to have a GoogleTransitFeed section which
        provides vehicle scheduling and trip data, in addition to the typical
        TrackSegment, Station, and Vehicle data."""
        self.controller = controller

        import xml.dom.minidom
        doc = xml.dom.minidom.parseString(scenario_xml)

        self.graph = self.build_graph(doc.getElementsByTagName('TrackSegments')[0])
        self.stations = self.load_stations(doc.getElementsByTagName('Stations')[0])

        self.vehicles = self.load_vehicles(doc.getElementsByTagName('Vehicles')[0],
                                           doc.getElementsByTagName('VehicleModels')[0])

        # Mapping from station id to the platform's trackSegment. Assumes
        # that stations in GTF data have only one platform.
        self.station2track = dict((s.id, s.platform_ts_id) for s in self.stations.values())
        self.track2station = dict((s.platform_ts_id, s.id) for s in self.stations.values())

        self.legs = self.load_legs(doc.getElementsByTagName('GoogleTransitFeed')[0], sim_end_time)
        doc.unlink()

        # For vehicles that are currently parked in berths, reserve those berths for them.
        for vehicle in self.vehicles.itervalues():
            station_id = self.track2station.get(vehicle.ts_id)
            if station_id is not None:
                station = self.stations[station_id]
                for idx, berth_pos in enumerate(station.berth_positions):
                    if abs(berth_pos - vehicle.pos) < 2:
                        station.berth_reservations[idx] = vehicle.id
                        vehicle.berth_id = idx
                        break

        # build a table of distances between stations
        self._station_dists, self._station_paths = self._build_station_tables()


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

    def load_stations(self, stations_xml):
        """Returns a dict of stations, keyed by the integer id.
        Note that in GTF data, each station is expected to have
        only a single platform, and therefore each station corresponds to
        one trackSegment."""
        stations = dict()
        for station_xml in stations_xml.getElementsByTagName('Station'):
            station_str_id = station_xml.getAttribute('id')
            station_int_id = int(station_str_id.split('_')[0])

            ts_ids = set() # using a set because 'TrackSegmentID' includes the duplicate ts from Platform
            for id_xml in station_xml.getElementsByTagName('TrackSegmentID'):
                ts_ids.add(self._to_numeric_id(id_xml))

            platform_xml = station_xml.getElementsByTagName('Platform')[0] # Only one platform for GTF data
            platform_ts_id = self._to_numeric_id(platform_xml.getElementsByTagName('TrackSegmentID')[0])
            berth_positions = []
            for berth_xml in platform_xml.getElementsByTagName('Berth'):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                berth_positions.append(end_pos - 0.1) # stop 10cm before the end of the berth

            stations[station_int_id] = Station(station_int_id, ts_ids, platform_ts_id, berth_positions)
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

    def load_legs(self, transit_xml, sim_end_time):
        """Returns a list of lists. The outer list is indexed by vehicle id,
        the inner list contains Legs instances, sorted by time,
        with the earliest at the end.
        """
        feed_start_time = to_seconds(transit_xml.getAttribute('start_time'))
        legs = [[] for i in range(len(self.vehicles))] # empty list for each vehicle
        for route_xml in transit_xml.getElementsByTagName('Route'):
            for trip_xml in route_xml.getElementsByTagName('Trip'):
                vehicle = int(trip_xml.getAttribute('vehicle_id').split('_')[0])
                stops_xml = trip_xml.getElementsByTagName('Stop')
                for origin_xml, dest_xml in pairwise(stops_xml):
                    depart = to_seconds(origin_xml.getElementsByTagName('Departure')[0].firstChild.data)
                    arrive = to_seconds(dest_xml.getElementsByTagName('Arrival')[0].firstChild.data)

                    # express times as number of seconds from the start of the feed.
                    depart -= feed_start_time
                    arrive -= feed_start_time

                    if depart > sim_end_time:
                        break

                    if arrive <= 0: # a leg that completed before the sim started.
                        continue

                    origin_station = int(origin_xml.getAttribute('station_id').split('_')[0])
                    dest_station = int(dest_xml.getAttribute('station_id').split('_')[0])

                    origin_ts = self.station2track[origin_station]
                    dest_ts = self.station2track[dest_station]

                    legs[vehicle].append(Leg(vehicle, origin_station,
                                             dest_station, origin_ts,
                                             dest_ts, depart, arrive))


        # Ensure that the legs are all connected. In particular, extra legs may need
        # to be inserted to account for a vehicle ending a trip at one station,
        # then beginning the next trip at a nearby station.
        for idx, leg_list in enumerate(legs):
            leg_list.sort()
            new_leg_list = [] # make a copy, to avoid altering the list whilst iterating over it
            for leg_a, leg_b in pairwise(leg_list):
                new_leg_list.append(leg_a)
                if leg_a.dest_station != leg_b.origin_station:
                    new_leg = Leg(leg_a.vehicle, leg_a.dest_station, # make a new leg connecting them
                                  leg_b.origin_station, leg_a.dest_ts,
                                  leg_b.origin_ts, leg_a.arrive, leg_b.depart)
                    new_leg_list.append(new_leg)
            new_leg_list.append(leg_b)

            # Each list of legs is sorted with earliest leg at the end of the list.
            new_leg_list.reverse()
            legs[idx] = new_leg_list # replace the old list with the augmented one
        return legs

    def _build_station_tables(self):
        """Returns a pair of 2D tables, where each table is indexed by
        station id: distances, paths"""
        n = len(self.stations)
        distances = zeros( (n, n) )
        indices = range(n)

        # create a 2D array, containing empty python lists
        # (not using an numpy array, because they can't hold references?)
        paths = [[None for i in indices] for j in indices]

        start_idx = 0
        for i in indices:
            for j in indices[i+1:]:
                length, path = networkx.bidirectional_dijkstra(self.graph, i, j)
                distances[i,j] = length
                paths[i][j] = path
                paths[j][i] = path

        distances += distances.transpose()
        return distances, paths

    def _to_numeric_id(self, element):
        """For elements similar to:
            <ID>x_trackSegment_forward</ID>
        where x is an integer. Returns just the integer value."""
        return int(element.childNodes[0].data.split('_')[0])

    def get_path(self, ts_1, ts_2):
        """Returns a two tuple. The first element is the path length (in meters),
        and the second is the path. ts_1 and ts_2 are integer trackSegment ids."""
        return networkx.bidirectional_dijkstra(self.graph, ts_1, ts_2)

    def get_next_leg(self, vehicle_id):
        """Returns None if no leg is available for the vehicle"""
        try:
            return self.legs[vehicle_id][-1]
        except IndexError:
            return None

    def pop_leg(self, vehicle_id):
        try:
            return self.legs[vehicle_id].pop()
        except IndexError:
            return None

class Station(object):
    UNLOAD_PLATFORM = 0
    QUEUE_PLATFORM = 0
    LOAD_PLATFORM = 0

    def __init__(self, s_id, ts_ids, platform_ts_id, berth_positions):
        """s_id: An integer station id.
        ts_ids: A set containing integer TrackSegment ids.
        platform_ts_id: A TrackSegment id. Will also be found in ts_ids.
        berth_positions: A list of floats, each one designating a position on
                         platform_ts_id that the vehicle will target in order
                         to park in the berth. i.e. to park in berth 1, the
                         vehicle will go to the position found in berth_positions[1].
        """
        self.id = s_id
        self.ts_ids = ts_ids
        self.platform_ts_id = platform_ts_id
        self.berth_positions = berth_positions

        self.berth_reservations = [None]*len(berth_positions)

    def reserve_berth(self, vehicle_id, platform_id):
        """Sets aside a berth for use by vehicle. Returns a triple:
            (berth_position, berth_id, ts_id)
        Returns None if no berths are available.
        Always reserves the berth closest to the station exit.
        """
        for idx in range(len(self.berth_positions)-1, -1, -1): # iterate over whole list in reverse
            if self.berth_reservations[idx] is None:
                self.berth_reservations[idx] = vehicle_id
                return (self.berth_positions[idx], idx, self.platform_ts_id)

    def release_berth(self, berth_id, platform_id):
        """Frees the berth for use. Returns None."""
        self.berth_reservations[berth_id] = None

class Vehicle(object):
    SPEED_INCREMENT = 2.77777  # 10 km/hr

    # To prevent a vehicle's trajectory from being undefined, splines are extended
    # so as to reach to the end of the simulation (and a little further). This
    # constant controls the 'little further'. Needs to be long enough that no algorithm
    # tries to predict a vehicle trajectory beyond this length. The incintive to keep
    # it somewhat small is that rounding errors are excacerbated if it is very large.
    SPLINE_TIME_EXTENSION = 3600     # in seconds

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
        self.leg = None
        self._spline = None
        self.traj_solver = TrajectorySolver(self.v_max, self.a_max, self.j_max,
                                            -5, self.a_min, self.j_min)

        self.platform_id = None
        self.berth_pos = None
        self.berth_id = None
        self.state = None

        self.next_platform_id = None
        self.next_berth_pos = None
        self.next_berth_id = None

    def get_spline(self):
        return self._spline
    def set_spline(self, spline, send=True):
        """Side Effect Warning: If the spline does not continue until the end of
                             the sim, then it is extended.
        """
        assert isinstance(spline, CubicSpline)

        sim_end_time = self.controller.sim_end_time
        if spline.t[-1] < sim_end_time:
            assert abs(spline.a[-1]) < 1E-6
            delta_q = spline.v[-1]*(sim_end_time + self.SPLINE_TIME_EXTENSION - spline.t[-1])
            spline.append(Knot(spline.q[-1]+delta_q, spline.v[-1], 0, sim_end_time+self.SPLINE_TIME_EXTENSION), 0)

        self._spline = spline
        if send:
            self.send_spline()
    spline = property(get_spline, doc="""A vehicle's planned trajectory represented by a cubic_spline.CubicSpline object.""")

    def send_spline(self):
        """Sends a the current vehicle spline to the sim."""
        traj_msg = api.CtrlCmdVehicleTrajectory()
        traj_msg.vID = self.id
        self._spline.fill_spline_msg(traj_msg.spline)
        self.controller.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, traj_msg)

    def send_path(self):
        """Sends the path to the sim. Path is a sequence of tracksegment ids,
        where the first element is expected to be the vehicle's current
        tracksegment."""
        if len(self.path) > 1: # only send the itinerary msg if it contains information
            itinerary_msg = api.CtrlCmdVehicleItinerary()
            itinerary_msg.vID = self.id
            itinerary_msg.trackIDs.extend(self.path[1:]) # don't include the segment self is currently on
            itinerary_msg.clear = False
            self.controller.send(api.CTRL_CMD_VEHICLE_ITINERARY, itinerary_msg)

    def update_vehicle(self, v_status):
        """Updates the relevant vehicle data."""
        assert v_status.vID == self.id
        self.ts_id = v_status.nose_locID
        self.pos = v_status.nose_pos
        self.vel = v_status.vel
        self.accel = v_status.accel
        self.pax = v_status.passengerIDs[:] # copy

        if self.vel != 0:
            assert self.state in (self.controller.RUNNING_LEG, self.controller.PARKING)

    def run_leg(self):
        """Plans an itinerary and trajectory for the vehicle to do the next leg
        of its schedule, then sends the appropriate commands to the simulator.
        The trajectory does not take the vehicle all the way to the berth,
        instead it aims to stop at the beginning of the station platform.
        See also: on_SIM_NOTIFY_VEHICLE_ARRIVE
        """
        leg = self.leg

        # Note that path_length does not take into account the self's position along the path.
        path_length, path = self.manager.get_path(self.ts_id, leg.dest_ts)
        self.path = path
        self.send_path()

        departure_time = max(leg.depart, self.controller.current_time) # in seconds
        trip_dist = path_length - self.pos
        assert trip_dist >= 0

        initial = Knot(self.pos, self.vel, self.accel, departure_time)
        final = Knot(path_length, 0, 0, leg.arrive)
        try:
            spline = self.traj_solver.target_time(initial, final)
        except FatalTrajectoryError:
            spline = self.traj_solver.target_position(initial, final) # Falling behind on the schedule. Go as fast as possible.

        self.set_spline(spline)

    def run(self, path, dist, final_speed=0, speed_limit=None):
        """Commands the vehicle's path and trajectory. Obeys the speed_limit
        constraint, if supplied. Assumes that the vehicle's pose is up to date
        and accurate. Returns the scheduled arrival time."""
        self.path = path
        self.send_path()

        if speed_limit:
            solver = TrajectorySolver(min(speed_limit, self.v_max), self.a_max, self.j_max, -5, self.a_min, self.j_min)
        else:
            sovler = self.traj_solver

        spline = solver.target_position(Knot(self.pos, self.vel, self.accel, self.controller.current_time),
                                        Knot(dist + self.pos, final_speed, 0, None))

        final_time = spline.t[-1]
        self.set_spline(spline)
        return final_time

    def park(self, berth_pos, berth_id, ts_id):
        """Changes the trajectory to bring the vehicle into the specified berth.
        Assumes that the vehicle's status information is up to date.
        Returns the time at which the vehicle will be parked, according to the new trajectory."""
        self.berth_pos = berth_pos
        self.berth_id = berth_id
        self.platform_id = Station.UNLOAD_PLATFORM

        # calculate how far the vehicle needs to travel to end in the berth
        path_length, self.path = self.manager.get_path(self.ts_id, ts_id)
        dist = self.berth_pos + path_length - self.pos
        assert dist >= 0

        initial = Knot(self.pos, self.vel, self.accel, self.controller.current_time)
        final = Knot(path_length + self.berth_pos, 0, 0, None)
        if initial.accel <= 0:
            # Computation is faster if we can limit the max velocity.
            spline = self.traj_solver.target_position(initial, final, max_speed=self.vel)
        else:
            spline = self.traj_solver.target_position(initial, final)
        time = spline.t[-1]
        self.set_spline(spline)
        return time

    def is_full(self):
        assert len(self.pax) <= self.capacity
        return len(self.pax) == self.capacity

    def disembark(self, passengers):
        """Disembark passengers"""
        for pax in passengers:
                        # if asserts are turned off, this will still be caught by the sim, logged, and an error msg sent to the controller.
            assert pax in self.pax, "t:%5.3f Disembark failed: vehicle %d at station %d disembarking %s not in %s" % \
                   (self.controller.current_time, self.id, self.leg.dest_station, pax, self.pax)
            self.pax.remove(pax)

        # send disembark command
        disembark_msg = api.CtrlCmdPassengersDisembark()
        disembark_msg.vID = self.id
        disembark_msg.sID = self.leg.dest_station
        disembark_msg.platformID = 0 # assumed that there is only one platform
        disembark_msg.berthID = self.berth_id
        disembark_msg.passengerIDs.extend(passengers)
        self.controller.send(api.CTRL_CMD_PASSENGERS_DISEMBARK, disembark_msg)

        self.controller.log.info("t:%5.3f Vehicle %d at station %d disembarking %s",
                                 self.controller.current_time, self.id, self.leg.dest_station, passengers)

    def embark(self, passengers):
        """Embark passengers"""
        fill_line = self.capacity - len(self.pax)

        self.pax.extend(passengers[:fill_line])

        # send embark command
        embark_msg = api.CtrlCmdPassengersEmbark()
        embark_msg.vID = self.id
        embark_msg.sID = self.leg.origin_station
        embark_msg.platformID = 0  # assumed that there is only one platform
        embark_msg.berthID = self.berth_id
        embark_msg.passengerIDs.extend(passengers[:fill_line])
        self.controller.send(api.CTRL_CMD_PASSENGERS_EMBARK, embark_msg)

        self.controller.log.info("t:%5.3f Vehicle %d at station %d embarking %s. Overflow: %s",
                                 self.controller.current_time, self.id, self.leg.origin_station,
                                 passengers[:fill_line], passengers[fill_line:])

        # Return overflow passengers
        return passengers[fill_line:]

    def advance_berth(self, berth_position, berth_id, platform_id, ts_id):
        """Sets the vehicle on a itenarary and trajectory for advancing to the
        next berth. Returns the time at which the vehicle will arrive."""
        if platform_id == self.platform_id: # staying on the same platform
            final_pos = berth_position - 0.1 # stop 10 cm short of the end
            spline = self.traj_solver.target_position(Knot(self.pos, self.vel, self.accel, self.manager.current_time),
                                                      Knot(final_pos, 0, 0, None))

        else: # advancing to the next platform
            path_length, path = self.manager.get_path(self.ts_id, ts_id)
            self.path = path
            self.send_path()

            final_pos = self.pos + path_length - 0.1 # stop 10 cm short of the end
            spline = self.traj_solver.target_position(Knot(self.pos, self.vel, self.accel, self.manager.current_time),
                                                      Knot(final_pos, 0, 0, None))

        finish_time = spline.t[-1]
        self.set_spline(spline)

        self.next_berth_pos = berth_position
        self.next_berth_id = berth_id
        self.next_platform_id = platform_id

        return finish_time

class Leg(object):
    """Legs of a trip. Each leg travels from one station to the next."""

    def __init__(self, vehicle, origin_station, dest_station, origin_ts, dest_ts, depart_time, arrive_time):
        """vehicle: an integer vehicle ID
        origin_station: the id of the origin station
        dest_station: the id of the dest station
        origin_ts: the id of the trackSegment that the leg originates at.
        dest_ts: id of the trackSegment that this leg ends at.
        depart_time: scheduled departure time, in seconds, measured from the beginning of the sim.
        arrive_time: scheduled arrival time, in seconds, measured from the beginning of the sim.
        """
        self.vehicle = vehicle
        self.origin_station = origin_station
        self.dest_station = dest_station
        self.origin_ts = origin_ts
        self.dest_ts = dest_ts
        self.depart = depart_time
        self.arrive = arrive_time

    def __cmp__(self, other):
        return cmp(self.depart, other.arrive)

    def __str__(self):
        return "V:%d OS:%d DS:%d OTS:%d DTS:%d D:%f A:%f" % \
               (self.vehicle, self.origin_station, self.dest_station,
                self.origin_ts, self.dest_ts, self.depart, self.arrive)

class PassengerManager(object):
    """Handles passenger movement through the transit network, routing them
    from their origin station to their destination. To accomodate the need
    for passengers to transfer between vehicles, a time-expanded graph is used.

    Each station is reperesented by a set of nodes. Each node in that set
    represents a vehicle arrival or departure time (with one special node that is
    time-independent).

    When a passenger is created, the PassengerManager finds the shortest path
    through the time-expanded graph, and sets up a itenerary for the pax to
    folllow in order to get to its destination.

    The itenerary for the passenger is stored in two places. It's stored on
    the station-time nodes, so that there's one centralized place for a
    vehicle to ask, "Who do I pick up, drop off?" It's also stored in the
    Passenger objects, so that I can more easily erase an old itenerary and
    reschedule the pax, for the case where she can't take her scheduled vehicle
    because it's full.
    """

    WALK_ID = -100

    def __init__(self, scenario_xml, sim_end_time, walk_speed, controller):
        """scenario_xml: The xml scenario data created by TrackBuilder, in string form."""
        self._walk_speed = walk_speed
        self.controller = controller

        import xml.dom.minidom
        doc = xml.dom.minidom.parseString(scenario_xml)

        self.graph, self.node_dict = self._build_graph(doc.getElementsByTagName('GoogleTransitFeed')[0], sim_end_time)
        self.paths = self._make_paths(self.graph, self.node_dict)
        self.passengers = {} # keyed by integer pax_id, values are Passenger instances.
        doc.unlink()
        assert isinstance(self.graph, networkx.classes.DiGraph)

    def add_passenger(self, pax):
        """Adds the new passenger's actions to the PassengerManager.
        Passengers embark as soon as their vehicle is available, and stay on
        the vehicle unless they need to transfer or they have arrived at their
        final destination."""

#        To reduce the time required for the controller to startup,
#        the controller only builds a graph for routing passengers to
#        locations that are reached by vehicles before the simulation
#        ends. For short simulations, this means that there aren't paths
#        to every destination station.
        assert isinstance(pax, Passenger)
        assert not pax.actions # pax.actions is empty
        self.controller.log.info("t:%5.3f Pax %d created. Travelling from %d to %d.",
                                 pax.origin_time, pax.id, pax.origin_id, pax.dest_id)

        # Passenger can be created at any time. Find the earliest Node at the
        # passenger's origin station.
        nodes = self.node_dict[pax.origin_id]
        origin_node = None
        for n in nodes:
            if n.time >= pax.origin_time:
                origin_node = n
                break
        if origin_node is None:
            self.controller.log.warn("t:%5.3f Pax %d created too late. No more vehicles are scheduled to arrive or depart from station %d.",
                                     pax.origin_time, pax.id, pax.origin_id)
            return # Passenger created too late. No more vehicles are scheduled to arrive or depart from this station.

        # Get the passenger's path to dest from the cached paths.
        dest_node = Node(pax.dest_id, None)
        try:
            path = self.paths[origin_node][dest_node]
        except KeyError:
            self.controller.log.warn("t:%5.3f Pax %d cannot get from %d to %d on the remaining trips scheduled in the simulation time.",
                                     self.controller.current_time, pax.id, pax.origin_id, pax.dest_id)
            return  # This passenger is stranded.

        if not path: # guard against case where passenger is created at its destination.
            self.controller.log.warn("t: %5.3f Pax %d was created at it's destination. origin: %d, dest: %d",
                                     self.controller.current_time, pax.id, pax.origin_id, pax.dest_id)
            return

        # Iterate the path, noting when to embark/disembark on the nodes.
        # Consider the following table of cases:
        #        a       b
        # prev_v curr_v,        action
        # ----------------------
        #  None,   None,           pass
        #  Walk,   None,           pass
        #    v1,   None,           disembark v1@a
        #  None,   Walk,           walk a->b
        #  Walk,   Walk,           walk a->b
        #    v1,   Walk,           disembark v1@a, walk a->b
        #  None,     v1,           embark v1@a
        #  Walk,     v1,           embark v1@a
        #    v1,     v1,           pass
        #    v1,     v2,           disembark v1@a, embark v2@a

        prev_v = api.NONE_ID # Prime the pump

        for (a, b) in pairwise(path):
            curr_v = self.graph[a][b]['vehicle']
            if prev_v != self.WALK_ID and prev_v != api.NONE_ID: # prev_v is a vehicle
                if prev_v != curr_v:
                    pax.add_action(PaxAction(a, prev_v, disembark=True))

            if curr_v != self.WALK_ID and curr_v != api.NONE_ID: # curr_v is a vehicle
                if prev_v != curr_v: # and I'm not already on it
                    pax.add_action(PaxAction(a, curr_v, embark=True))

            if curr_v == self.WALK_ID:
                pax.add_action(PaxAction(b, self.WALK_ID, walk=True,
                                         walk_dest=b.station,
                                         walk_dist=self.graph[a][b]['weight']))
            prev_v = curr_v

        # Can ignore the last node, since it's the 'destination' node.

        # Store the pax's actions on the station/time nodes to group all
        # pax actions at a given time & place together.
        for action in pax.actions:
            if action.disembark:
                action.node.disembark_pax[action.vehicle].append(pax.id)
            elif action.embark:
                action.node.embark_pax[action.vehicle].append(pax.id)
            elif action.walk:
                action.node.walk_pax[pax.id] = ( action.walk_dest, action.walk_dist )

        self.passengers[pax.id] = pax
        self.controller.log.debug("t:%5.3f Pax %d actions: %s",
                                  pax.origin_time, pax.id, pax.get_actions_str())

    def remove_pax(self, pax):
        """Removes the passenger from the scheduled commands."""
        for action in pax.actions:
            assert isinstance(action, PaxAction)
            if action.embark:
                try:
                    action.node.embark_pax[action.vehicle].remove(pax.id)
                except ValueError:
                    pass
            elif action.disembark:
                try:
                    action.node.disembark_pax[action.vehicle].remove(pax.id)
                except ValueError:
                    pass
            elif action.walk:
                del action.node.walk_pax[pax.id]
            else:
                raise Exception
        pax.actions = []

    def reschedule_pax(self, pax_id, origin_id, origin_time):
        """Reschedules the passenger. For origin_id and origin_time, provide
        the passenger's current station_id and sim_time."""
        pax = self.passengers[pax_id]
        assert isinstance(pax, Passenger)
        self.remove_pax(pax)

        pax.origin_id = origin_id
        pax.origin_time = origin_time

        self.add_passenger(pax)

    def fetch_disembarking_pax(self, vehicle_id, station_id, time):
        """Returns a list of passenger ids. The passengers are removed from the
        disembarking queue.

        Note that time is the vehicle's "official" arrival or departure time
        not its actual time. Time is measured in seconds, with the
        feed_start_time as 0 (the same convention is used for Leg instances)."""
        nodes = self.node_dict[station_id]
        node = None
        for n in nodes:
            if n.time == time:
                node = n
                break
        assert isinstance(node, Node)
        passengers = node.disembark_pax[vehicle_id] # ownership of the list is handed over to caller
        node.disembark_pax[vehicle_id] = []
        return passengers

    def fetch_embarking_pax(self, vehicle_id, station_id, time):
        """Returns a list of passenger ids. The passengers are removed from the
        embarking queue. If they do not fit on the vehicle, it is the caller's
        responsibility to make sure they are rescheduled.

        Note that time is the vehicle's "official" arrival or departure time
        not its actual time. Time is measured in seconds, with the
        feed_start_time as 0 (the same convention is used for Leg instances)."""
        nodes = self.node_dict[station_id]
        node = None
        for n in nodes:
            if n.time == time:
                node = n
                break
        assert isinstance(node, Node)
        passengers = node.embark_pax[vehicle_id] # ownership of the list is handed over to caller
        node.embark_pax[vehicle_id] = []
        return passengers

    def _build_graph(self, gtf_xml, sim_end_time):
        """Returns a networkx.DiGraph suitable for routing passengers over and
        a dictionary whose keys are station_ids and whose values are lists
        containing all Node instances for that station_id.

        sim_end_time has units of seconds, and walk_speed has units of meters/sec.

        The DiGraph is a time-expanded graph of the transit network.
        Each station has multiple Node instances; one Node for each time at which a
        vehicle arrives or departs, and one edge for each vehicle. Example:

        10:00 ---->.
                   |
        10:05 ---->.----->     The diagram to the left represents a single station.
                   |           Each '.' is a node in the digraph, and the vertical
                   |           lines are edges which represent a passenger waiting
        10:15      .----->     at the station.
                   |
        10:20 ---->.----->

        The graph also contains an additional Node for each station whose time
        is None. This additional node has only inbound edges, one edge for
        each vehicle arrival.

        Usage example:
        graph, node_dict = build_graph(xml, end_time, 1.33)
        """
        graph = networkx.DiGraph()

        ### Create edges that represent travelling between stations on a vehicle ###
        feed_start_time = to_seconds(gtf_xml.getAttribute('start_time'))
        for route_xml in gtf_xml.getElementsByTagName('Route'):
            for trip_xml in route_xml.getElementsByTagName('Trip'):
                vehicle = int(trip_xml.getAttribute('vehicle_id').split('_')[0])
                stops_xml = trip_xml.getElementsByTagName('Stop')
                for origin_xml, dest_xml in pairwise(stops_xml):
                    origin_station = int(origin_xml.getAttribute('station_id').split('_')[0])
                    dest_station = int(dest_xml.getAttribute('station_id').split('_')[0])

                    depart = to_seconds(origin_xml.getElementsByTagName('Departure')[0].firstChild.data)
                    arrive = to_seconds(dest_xml.getElementsByTagName('Arrival')[0].firstChild.data)

                    depart -= feed_start_time
                    arrive -= feed_start_time

                    if depart > sim_end_time:
                        break

                    graph.add_edge(Node(origin_station, depart), Node(dest_station, arrive),
                                   weight=arrive-depart, vehicle=vehicle)

        ### Create edges that represent waiting at a station. ###
        node_dict = defaultdict(list)
        for n in graph.nodes():
            node_dict[n.station].append(n)

        # For each list of nodes pertaining to a station, sort them by their
        # timestamp (ascending order) and add 'wait' edges between them. Edges are
        # weighted by their wait time and contain api.NONE_ID as the vehicle id.
        for node_list in node_dict.itervalues():
            node_list.sort(key=attrgetter('time'))
            for a, b in pairwise(node_list):
                graph.add_edge(a, b, weight=b.time-a.time, vehicle=api.NONE_ID)

        ### Create edges that represent walking between nearby stations. ###
        if self._walk_speed: # disable walking if walk speed is 0 or None
            neighbors_xml = gtf_xml.getElementsByTagName('Neighbors')[0]
            for station_xml in neighbors_xml.getElementsByTagName('Station'):
                s_id = int(station_xml.getAttribute('id').split('_')[0])

                # make a list of neighbors, sorted by distance, where each neighbor
                # is a 2-tuple: (time, neighbor_id)
                neighbor_list = []
                for neighbor_xml in station_xml.getElementsByTagName('Neighbor'):
                    neighbor_id = int(neighbor_xml.getAttribute('station_id').split('_')[0])
                    neighbor_dist = int(neighbor_xml.getAttribute('distance'))
                    neighbor_list.append( (neighbor_dist/self._walk_speed, neighbor_id) )
                neighbor_list.sort() # sorts by first element of tuples, then by second.

                # add new outbound 'walk' edges for each station timepoint that
                # has a vehicle arrival.
                for station_node in node_dict[s_id]:
                    edges = graph.in_edges(station_node, data=True)
                    if any([True for e in edges if e[2]['vehicle'] != api.NONE_ID]):
                        for pair in neighbor_list:
                            neighbor_node = Node(pair[1], station_node.time + pair[0])
                            # Will add neighbor_node to graph if it doesn't already,
                            # exist, otherwise the existing node is used.
                            # Relies on Node's __eq__ only checking station_id and time.
                            graph.add_edge(station_node, neighbor_node,
                                           weight=pair[0], vehicle=self.WALK_ID)

        ### Add a 'destination' node that does not have a time. This gives a target to route
        ### to without requiring the time of arrival to be known. Every Node
        ### instance for a station has a 0-weight edge to the station's 'destination' node.
        for s_id, node_list in node_dict.iteritems():
            dest_node = Node(s_id, None)
            for node in node_list:
                if graph.in_degree(node):
                    graph.add_edge(node, dest_node, weight=0, vehicle=api.NONE_ID)

        return graph, node_dict

    def _make_paths(self, graph, node_dict):
        """Finds shortest-time paths from each station node (station + timepoint)
        to every destination node (time independent, one per station)."""
        paths = {}
        for node_list in node_dict.itervalues():
            for node in node_list:
                if graph.out_degree(node): # excludes the destination nodes, which only have in_edges
                    dist_dict, path_dict = networkx.single_source_dijkstra(graph, node)

                    # I only need paths to destination nodes (with time=None). Using
                    # single_source_dijkstra is the fastest, most scalable algo
                    # even though it finds more results than needed. Delete the
                    # unnecessary paths to save memory.
                    for n in path_dict.keys():
                        if n.time is not None: # sink isn't a 'destination node'
                            del path_dict[n]

                    paths[node] = path_dict # use a dict of dicts structure
        return paths

class Node(object):
    def __init__(self, station_id, time):
        """station_id: an integer station id
        time: the time, in seconds, corresponding to the scheduled arrival/departure
        """
        self.station = station_id
        self.time = time

        # keys are passenger_ids, values are (dest_station_id, travel_time) tuples.
        self.walk_pax = dict()

        # keyed by vehicle_id, values are lists of passenger_ids
        self.disembark_pax = defaultdict(list)
        self.embark_pax = defaultdict(list)

    def __hash__(self):
        if self.time:
            return self.station + self.time
        else:
            return self.station
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.station == other.station and self.time == other.time
        else: return False
    def __str__(self):
        return "Station: %d, Time: %s, Disembark: %s, Embark: %s" % \
               (self.station, self.time, self.disembark_pax, self.embark_pax)

class Passenger(object):
    def __init__(self, pax_id, origin_id, dest_id, origin_time):
        self.id = pax_id
        self.origin_id = origin_id
        self.dest_id = dest_id
        self.origin_time = origin_time

        # This information is held for the purposes of rescheduling the
        # passenger if a vehicle is too full. See add_action.
        self.actions = []

    def __str__(self):
        return "id: %d, origin_id: %d, dest_id: %d, origin_time: %f, actions: %s" % \
               (self.id, self.origin_id, self.dest_id, self.origin_time, self.get_actions_str())

    def add_action(self, action):
        assert isinstance(action, PaxAction)
        if self.actions:
            last = self.actions[-1]
            # If we just got off the vehicle, rather than adding the embark, delete the disembark
            if last.disembark is True and action.embark is True and last.vehicle == action.vehicle:
                self.actions.pop()
            else:
                self.actions.append(action)
        else:
            self.actions.append(action)

    def get_actions_str(self):
        """Returns the passengers actions info in string form."""
        return ' --> '.join(str(action) for action in self.actions)

class PaxAction(object):
    """One, and only one, of embark, disembark, or walk will be True.
    If walk is true, include walk_dest, the station_id of the destination."""
    def __init__(self, node, vehicle, embark=False, disembark=False, walk=False,
                 walk_dest=api.NONE_ID, walk_dist=0):
        assert embark or disembark or walk
        assert isinstance(node, Node)
        assert isinstance(vehicle, int)
        assert isinstance(embark, bool)
        assert isinstance(disembark, bool)
        assert isinstance(walk, bool)
        assert isinstance(walk_dest, int)
        self.node = node
        self.vehicle = vehicle
        self.embark = embark
        self.disembark = disembark
        self.walk = walk
        self.walk_dest = walk_dest
        self.walk_dist = walk_dist

    def __str__(self):
        if self.embark:
            return "Embark %d@station %d@t %d" % (self.vehicle, self.node.station, self.node.time)
        elif self.disembark:
            return "Disemb %d@station %d@t %d" % (self.vehicle, self.node.station, self.node.time)
        elif self.walk:
            return"Walk %.1fmeters from %d to station %d@t %d" % (self.walk_dist, self.node.station, self.walk_dest, self.node.time)
        else:
            raise Exception

if __name__ == '__main__':
    main()
