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

import pyprt.shared.api_pb2 as api
from base_controller import BaseController
from trajectory_solver import TrajectorySolver
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
    options, args = options_parser.parse_args()

    if len(args) != 1:
        options_parser.error("Expected one argument. Received: %s" % ' '.join(args))

    ctrl = GtfController(options.logfile, options.comm_logfile, args[0])
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

    def __init__(self, log_path, commlog_path, scenario_path):
        super(GtfController, self).__init__(log_path, commlog_path)
        self.scenario_path = scenario_path
        self.t_reminders = defaultdict(list) # keyed by time, values are lists of vehicle ids

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

        for pax in overflow_passengers:
            origin_time = max(self.current_time, leg.depart+1)
            self.p_manager.reschedule_pax(pax, leg.origin_station, origin_time)

    ### Overriden message handlers ###
    def on_SIM_GREETING(self, msg, msgID, msg_time):
        self.sim_end_time = msg.sim_end_time
        self.log.info("Sim Greeting message received. Sim end at: %f" % msg.sim_end_time)

        self.v_manager = VehicleManager(self.scenario_path, msg.sim_end_time)
        Vehicle.manager = self.v_manager
        Vehicle.controller = self

        self.p_manager = PassengerManager(self.scenario_path, msg.sim_end_time)

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
                t_arrival = vehicle.park()

                # use time-based notification of vehicle stopping in berth
                # TODO: Add a SIM_NOTIFY_VEHICLE_STOPPED event message.
                self.set_notification(vehicle.id, t_arrival+1)

            # not at dest station or it's offramp
            elif leg.depart < 0:
                vehicle.state = self.RUNNING_LEG
                vehicle.run_leg()

            else:
                raise Exception("Unexpected case.")

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
            t_arrival = vehicle.park()
            self.set_notification(vehicle.id, t_arrival+1)

    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        vehicle = self.v_manager.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status)

        if vehicle.state is self.RUNNING_LEG and msg.trackID == vehicle.leg.origin_ts:
            if vehicle.berth_id != None:
                station_id = self.v_manager.track2station[vehicle.leg.origin_ts]
                station = self.v_manager.stations[station_id]
                station.release_berth(vehicle.berth_id)
                vehicle.berth_id = None
                vehicle.berth_pos = None

class VehicleManager(object):
    """Reads schedule information from the gtf information in the scenario.
    Provides pathing information for the vehicles."""

    def __init__(self, xml_path, sim_end_time):
        """xml_path: the path (including filename) to the xml scenario file
        created by TrackBuilder.

        The scenario file is expected to have a GoogleTransitFeed section which
        provides vehicle scheduling and trip data, in addition to the typical
        TrackSegment, Station, and Vehicle data."""
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

        self.graph = self.build_graph(doc.getElementsByTagName('TrackSegments')[0])
        self.stations = self.load_stations(doc.getElementsByTagName('Stations')[0])

        self.vehicles = self.load_vehicles(doc.getElementsByTagName('Vehicles')[0],
                                           doc.getElementsByTagName('VehicleModels')[0])

        # Mapping from station id to the platform's trackSegment. Assumes
        # that stations in GTF data have only one platform.
        self.station2track = dict((s.id, s.platform_ts_id) for s in self.stations.values())
        self.track2station = dict((s.platform_ts_id, s.id) for s in self.stations.values())

        # For vehicles that are currently parked in berths, reserve those berths for them.
        for vehicle in self.vehicles.itervalues():
            station_id = self.track2station.get(vehicle.ts_id)
            if station_id != None:
                station = self.stations[station_id]
                for idx, berth_pos in enumerate(station.berth_positions):
                    if abs(berth_pos - vehicle.pos) < 2:
                        station.berth_reservations[idx] = vehicle.id
                        vehicle.berth_id = idx
                        break

        self.legs = self.load_legs(doc.getElementsByTagName('GoogleTransitFeed')[0], sim_end_time)
        doc.unlink()

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

    def reserve_berth(self, vehicle_id):
        """Sets aside a berth for use by vehicle. Returns a pair:
            (berth_position, berth_id)
        Returns None if no berths are available.
        Always reserves the berth closest to the station exit.
        """
        for idx in range(len(self.berth_positions)-1, -1, -1): # iterate over whole list in reverse
            if self.berth_reservations[idx] is None:
                self.berth_reservations[idx] = vehicle_id
                return (self.berth_positions[idx], idx)

    def release_berth(self, berth_id):
        """Frees the berth for use. Returns None."""
        self.berth_reservations[berth_id] = None

class Vehicle(object):
    SPEED_INCREMENT = 2.77777  # 10 km/hr

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
        self.spline = None
        self.traj_solver = TrajectorySolver(self.v_max, self.a_max, self.j_max,
                                           -5, self.a_min, self.j_min)
        self.berth_pos = None
        self.berth_id = None
        self.state = None

    def update_vehicle(self, v_status):
        """Updates the relevant vehicle data."""
        self.ts_id = v_status.nose_locID
        self.pos = v_status.nose_pos
        self.vel = v_status.vel
        self.accel = v_status.accel
        self.pax = v_status.passengerIDs[:] # copy

        if self.vel != 0:
            assert self.state in (self.controller.RUNNING_LEG, self.controller.PARKING)

    def make_trajectory(self, dist, start_time, end_time):
        """Returns a cubic_spline.CubicSpline containing a trajectory that moves
        the vehicle dist meters and comes to a stop.
        'start_time' and 'end_time' are both measured in seconds, with the
        beginning of the simulation as 0.
        'end_time' is unusual in that it is a target, not a dictate. The
        vehicle will not exceed its velocity/accel/jerk limits, and so it may
        arrive late. In most cases, the vehicle will actually arrive a little
        early. It will be rare for it to arrive at exactly end_time.
        """
        total_time = end_time - start_time

        # In principle, the vehicle should travel as slow as possible while still
        # arriving before end_time. This keeps us closer to the schedule, and
        # helps make up for the fact that we don't have any knowledge about speed
        # limits. To do this, we choose a v_max which may be lower than the
        # vehicle's v_max, while still being sufficient to arrive on time or
        # early.

        # An initial estimate of v_max assumes infinite jerk, which will make
        # the vehicle arrive slightly too late. The max_speed is rounded up to
        # the nearest 10 km/hr increment, and then increased in 10 km/hr
        # increments until the vehicle arrives on-time or better.
        if total_time <= 0 or dist/total_time > self.v_max:
            v_max = self.v_max # go as quick as we can
        else:
            # SEE: trajectory_calcs.py, prob4. Assuming symmetry of jerk, accel!
            # FIXME: Assumption is generally bad, since more braking power than
            #        accel. Results in too low of a v_max, which is acceptable, but wastes CPU time
            A = (1/(2*self.a_max) - 1/(2*self.a_min))  # (1/(2*an) - 1/(2*ax))
            B = (total_time)                           # (t_total)
            C = -(dist)                         # -(q_total)
            v_max = max(TrajectorySolver.nonnegative_roots(A, B, C))
            v_max = (v_max//self.SPEED_INCREMENT + 1)*self.SPEED_INCREMENT  # Round up

        final_pos = dist + self.pos

        # Keep bumping up the target v_max until we arrive on time, or reach
        # the vehicle's max speed. If the initial guess at v_max is good, the
        # trajectory is only calculated once. (Yes, this is linear search.)
        initial_knot = Knot(self.pos, self.vel, self.accel, start_time)
        final_knot = Knot(final_pos, 0, 0, None)
        while True:
            cspline, fnc = self.traj_solver.target_position(initial_knot,
                                                            final_knot,
                                                            fnc_info=True)

            # target_position_none ignores v_max, so raising v_max doesn't help
            if fnc == self.traj_solver.target_position_none:
                break
            if cspline.t[-1] - cspline.t[0] <= total_time: # staying on or ahead of schedule
                break
            elif v_max >= self.v_max:
                break # vehicle will fall behind schedule, but oh well.
            else:
                # bump up the top speed, but not above vehicle's max
                v_max = min(self.v_max, v_max + self.SPEED_INCREMENT)

        assert abs((cspline.q[-1] - cspline.q[0]) -(final_pos - self.pos)) < 0.01

        # Append a last knot which extends the spline until past the end of the simulation.
        if cspline.t[-1] <= self.controller.sim_end_time:
            cspline.append(Knot(final_pos, 0, 0, self.controller.sim_end_time+1))

        return cspline

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

        itinerary_msg = api.CtrlCmdVehicleItinerary()
        itinerary_msg.vID = self.id
        itinerary_msg.trackIDs.extend(path[1:]) # don't include the segment self is currently on
        itinerary_msg.clear = False
        self.controller.send(api.CTRL_CMD_VEHICLE_ITINERARY, itinerary_msg)

        departure_time = max(leg.depart, self.controller.current_time) # in seconds
        trip_dist = path_length - self.pos
        assert trip_dist >= 0
        spline = self.make_trajectory(trip_dist, departure_time, leg.arrive)

        traj_msg = api.CtrlCmdVehicleTrajectory()
        traj_msg.vID = self.id
        spline.fill_spline_msg(traj_msg.spline)
        self.controller.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, traj_msg)

    def park(self):
        """Reserves a berth for the vehicle, and changes the trajectory to
        bring the vehicle into it. Assumes that the vehicle's status
        information is up to date. Returns the time at which the vehicle will
        be parked, according to the new trajectory."""
        station = self.manager.stations[self.leg.dest_station]

        # reserve station berth
        self.berth_pos, self.berth_id = station.reserve_berth(self.id)

        # calculate how far the vehicle needs to travel to end in the berth
        path_length, self.path = self.manager.get_path(self.ts_id, self.leg.dest_ts)
        dist = self.berth_pos + path_length - self.pos
##        for b, a in pairwise(reversed(vehicle.path)): # b is closer to the end of the path
##            if vehicle.ts_id == b:
##                break
##            dist += self.v_manager.graph[a][b]['weight']
        assert dist >= 0

        # make the trajectory
        self.spline = self.make_trajectory(dist, self.controller.current_time, self.leg.arrive)

        # send the trajectory
        traj_msg = api.CtrlCmdVehicleTrajectory()
        traj_msg.vID = self.id
        self.spline.fill_spline_msg(traj_msg.spline)
        self.controller.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, traj_msg)

        return self.spline.t[-2]

    def is_full(self):
        assert len(self.pax) <= self.capacity
        return len(self.pax) == self.capacity

    def disembark(self, passengers):
        """Disembark passengers"""
        for pax in passengers:
            assert pax in self.pax
            self.pax.remove(pax)

        # send disembark command
        disembark_msg = api.CtrlCmdPassengersDisembark()
        disembark_msg.vID = self.id
        disembark_msg.sID = self.leg.dest_station
        disembark_msg.platformID = 0 # assumed that there is only one platform
        disembark_msg.berthID = self.berth_id
        disembark_msg.passengerIDs.extend(passengers)
        self.controller.send(api.CTRL_CMD_PASSENGERS_DISEMBARK, disembark_msg)

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

        # Return overflow passengers
        return passengers[fill_line:]

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
    """

    WALK_ID = -100

    def __init__(self, xml_path, sim_end_time):
        """xml_path: the path (including filename) to the xml scenario file
        created by TrackBuilder."""
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

        self.graph, self.node_dict = self._build_graph(doc.getElementsByTagName('GoogleTransitFeed')[0], sim_end_time, walk_speed=1.33)
        self.paths = self._make_paths(self.graph, self.node_dict)
        self.passengers = {} # keyed by integer pax_id, values are Passenger instances.
        doc.unlink()
        assert isinstance(self.graph, networkx.classes.DiGraph)

    def add_passenger(self, pax):
        """Adds the new passenger's actions to the PassengerManager.
        Passengers embark as soon as their vehicle is available, and stay on
        the vehicle unless they need to transfer or they have arrived at their
        final destination."""

##        To reduce the time required for the controller to startup,
##        the controller only builds a graph for routing passengers to
##        locations that are reached by vehicles before the simulation
##        ends. For short simulations, this means that there aren't paths
##        to every destination station.
        assert isinstance(pax, Passenger)
        assert not pax.path # pax.path is empty

        # Passenger can be created at any time. Find the earliest Node at the
        # passenger's origin station.
        nodes = self.node_dict[pax.origin_id]
        origin_node = None
        for n in nodes:
            if n.time >= pax.origin_time:
                origin_node = n
                break
        if origin_node is None:
            return # Passenger created too late. No more vehicles are scheduled to arrive or depart from this station.

        # Get the passenger's path to dest from the cached paths.
        dest_node = Node(pax.dest_id, None)
        try:
            path = self.paths[origin_node][dest_node]
        except KeyError:
            return  # This passenger is stranded.

        if not path: # guard against case where passenger is created at its destination.
            return

        ### Annotate the Node instances, providing a timeline for all passenger actions.
        ### Work in reverse, from the destination to the origin.

        # Prime the pump and handle the last leg.
        next_v = self.graph[path[-2]][path[-1]]['vehicle']
        if next_v == self.WALK_ID:
            path[-2].walk_pax[pax.id] = (path[-2].station,
                                         self.graph[path[-2]][path[-1]]['weight'])
            pax.add_to_path(path[-2], next_v, walk=True)

        # Disembark if the pax made it to her final destination.
        elif path[-1].station == pax.dest_id:
            path[-1].disembark_pax[next_v].append(pax.id)
            pax.add_to_path(path[-1], next_v, disembark=True)

        # Iterate the path in reverse, noting when to embark/disembark on the nodes.
        # Consider the following table of cases:
        # case,    v, next_v, action
        # --------------------
        #    1, None,   None,  pass
        #    2, None,   Walk,  pass
        #    3, None,     v1,  embark v1@b
        #    4, Walk,   None,  walk a->b
        #    5, Walk,   Walk,  walk a->b
        #    6, Walk,     v1,  embark v1@b then walk a->b (working in reverse order)
        #    7,   v1,   None,  disembark v1@b
        #    8,   v1,   Walk,  disembark v1@b
        #    9,   v1,     v1,  pass
        #   10,   v1,     v2,  embark v2@b then disembark v1@b (working in reverse order)
        for b, a in pairwise(reversed(path)): # a is closer to beginning of path; b is closer to the end
            v = self.graph[a][b]['vehicle']

            # cases: 3, 6p, 10p (p indicates case partially handled)
            if next_v != api.NONE_ID and next_v != self.WALK_ID:
                if v != next_v:
                    b.embark_pax[next_v].append(pax.id)
                    pax.add_to_path(b, next_v, embark=True)

            # cases: 7, 8, 10p
            if v != api.NONE_ID and v != self.WALK_ID:
                if v != next_v:
                    b.disembark_pax[v].append(pax.id)
                    pax.add_to_path(b, next_v, disembark=True)

            # cases: 4, 5, 6p
            if v == self.WALK_ID:
                a.walk_pax[pax.id] = (b.station, self.graph[a][b]['weight'])
                pax.add_to_path(a, v, walk=True)

            # other cases require no passenger actions.

            next_v = v

            # TO DELETE after testing.
##            if v == next_v: # stay on current vehicle, keep waiting, or keep walking
##                pass
##            elif v == api.NONE_ID or v == self.WALK_ID:
##                # if next_v arrives at b, get on it
##                if next_v in [edge[2]['vehicle'] for edge in self.graph.in_edges(b, data=True)]:
##                    b.embark_pax[next_v].append(pax.id)
##                    embarked = True
##                    pax.add_to_path(b, next_v, embark=True)
##            else: # v != next_v and v != api.NONE_ID and v != api.WALK_ID
##                # get off current vehicle, board new vehicle.
##                b.disembark_pax[v].append(pax.id)
##                pax.add_to_path(b, v, disembark=True)
##
##                b.embark_pax[next_v].append(pax.id)
##                pax.add_to_path(b, next_v, embark=True)
##                embarked = True
##
##            if v == self.WALK_ID: # walking from a to b
##                a.walk_pax[pax.id] = (b.station, self.graph[a][b]['weight'])
##                path.add_to_path(a, v, walk=True)
##
##            next_v = v

        # Handle the case where the pax is created at a station after its
        # vehicle has already arrived (but before the vehicle departed, obviously)
        if v != api.NONE_ID:
            if v == self.WALK_ID:
                a.walk_pax[pax.id] = (b.station, self.graph[a][b]['weight'])
                pax.add_to_path(a, v, walk=True)
            else:
                a.embark_pax[v].append(pax.id)
                pax.add_to_path(a, v, embark=True)

        pax.path.reverse()
        self.passengers[pax.id] = pax

    def remove_pax(self, pax):
        """Removes the passenger from the scheduled embark/disembark commands."""
        for node, vehicle, embark, disembark, walk in pax.path:
            assert isinstance(node, Node)
            if embark:
                try:
                    node.embark_pax[vehicle].remove(pax.id)
                except ValueError:
                    pass
            if disembark:
                try:
                    node.disembark_pax[vehicle].remove(pax.id)
                except ValueError:
                    pass
            if walk:
                del node.walk_pax[pax.id]
        pax.path = []

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

    def _build_graph(self, gtf_xml, sim_end_time, walk_speed):
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
        neighbors_xml = gtf_xml.getElementsByTagName('Neighbors')[0]
        for station_xml in neighbors_xml.getElementsByTagName('Station'):
            s_id = int(station_xml.getAttribute('id').split('_')[0])

            # make a list of neighbors, sorted by distance, where each neighbor
            # is a 2-tuple: (time, neighbor_id)
            neighbor_list = []
            for neighbor_xml in station_xml.getElementsByTagName('Neighbor'):
                neighbor_id = int(neighbor_xml.getAttribute('station_id').split('_')[0])
                neighbor_dist = int(neighbor_xml.getAttribute('distance'))
                neighbor_list.append( (neighbor_dist/walk_speed, neighbor_id) )
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
                    # unnecessary paths to save memory. For the paths we do keep,
                    # remove the destination node from the end of the path.
                    for n in path_dict.keys():
                        if n.time is not None: # sink isn't a 'destination node'
                            del path_dict[n]
                        else:
                            path_dict[n].pop() # remove the last node from path

                    paths[node] = path_dict # use a dict of dicts structure

#        print "LEN(PATHS)", len(paths)
#        print sum([len(path) for path in paths.itervalues()])
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
        # passenger if a vehicle is too full. See add_to_path.
        self.path = []

    def __str__(self):
        return "id: %d, origin_id: %d, dest_id: %d, origin_time: %f, path: %s" % \
               (self.id, self.origin_id, self.dest_id, self.origin_time, ', '.join([str(n) for n in self.path]))

    def add_to_path(self, node, vehicle, embark=False, disembark=False, walk=False):
        assert isinstance(node, Node)
        assert isinstance(vehicle, int)
        assert isinstance(embark, bool)
        assert isinstance(disembark, bool)
        assert isinstance(walk, bool)
        self.path.append((node, vehicle, embark, disembark, walk))

if __name__ == '__main__':
    main()
