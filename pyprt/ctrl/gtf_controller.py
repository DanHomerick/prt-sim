"""A controller which attempts to run vehicles on schedules and routes
described by Google Transit Feed data.
"""
from __future__ import division
import optparse
from itertools import izip
from collections import defaultdict
from operator import attrgetter

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
    def __init__(self, log_path, commlog_path, scenario_path):
        super(GtfController, self).__init__(log_path, commlog_path)
        self.scenario_path = scenario_path
        self.reminders = {}

    ### Overriden message handlers ###
    def on_SIM_GREETING(self, msg_type, msgID, msg_time, msg_str):
        msg = api.SimGreeting()
        msg.MergeFromString(msg_str)
        self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
        self.log.info("Sim Greeting message received.")

        self.v_manager = VehicleManager(self.scenario_path, msg.sim_end_time)
        self.p_manager = PassengerManager(self.scenario_path, msg.sim_end_time)

    def on_SIM_START(self, msg_type, msgID, msg_time, msg_str):
        msg = api.SimStart()
        msg.MergeFromString(msg_str)
        self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
        self.log.info("Sim Start message received.")
        inf = 1E10000

        for vehicle_id in self.v_manager.vehicles.keys():
            leg, dist, path = self.v_manager.get_next_leg(vehicle_id)

            itinerary_msg = api.CtrlCmdVehicleItinerary()
            itinerary_msg.vID = vehicle_id
            itinerary_msg.tsIDs.extend(path[1:]) # don't include the segment vehicle is currently on
            itinerary_msg.clear = False
            self.send(api.CTRL_CMD_VEHICLE_ITINERARY, self.sim_time, itinerary_msg)

            traj_msg = api.CtrlCmdVehicleTrajectory()
            traj_msg.vID = vehicle_id
            spline = self.v_manager.make_trajectory(leg, dist, leg.depart)
            spline.append(Knot(spline.q[-1], 0, 0, inf)) # stay stopped here
            self.fill_spline_msg(spline, traj_msg.spline)
            self.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, self.sim_time, traj_msg)

            setnotify_msg = api.CtrlSetnotifyTime()
            setnotify_msg.time = spline.t[-2] + 1
            self.send(api.CTRL_SETNOTIFY_TIME, self.sim_time, setnotify_msg)

            self.reminders[int(setnotify_msg.time*1000)] = leg # convert to an integer number of milliseconds, so that comparison is trivial

        self.send_resume()

    def on_SIM_NOTIFY_TIME(self, msg_type, msgID, msg_time, msg_str):
        msg = api.SimNotifyTime()
        msg.MergeFromString(msg_str)
        self.log_rcvd_msg( msg_type, msgID, msg_time, msg )

        ms_msg_time = int(msg.time*1000) # millisec integer for easy comparison
        leg = self.reminders[ms_msg_time]
        assert isinstance(leg, Leg)
        del self.reminders[ms_msg_time]

        disembarking, embarking = self.p_manager.get_pax_actions(leg.vehicle, leg.dest_station, leg.arrive)


        self.send_resume()

    def on_SIM_EVENT_PASSENGER_CREATED(self, msg_type, msgID, msg_time, msg_str):
        msg = api.SimEventPassengerCreated()
        msg.MergeFromString(msg_str)
        self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
        p_status = msg.p_status
        self.p_manager.add_passenger(p_status.pID, p_status.src_stationID,
                                     p_status.dest_stationID, p_status.creation_time)
        self.send_resume()

class VehicleManager(object):
    """Reads schedule information from the gtf information in the scenario.
    Provides pathing information for the vehicles."""
    SPEED_INCREMENT = 2.77777  # 10 km/hr

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

        self.legs = self.load_legs(doc.getElementsByTagName('GoogleTransitFeed')[0], sim_end_time)

    def build_graph(self, track_segments_xml):
        """Returns a networkx.DiGraph suitable for routing vehicles over."""
        graph = networkx.DiGraph()

        for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
            tsId = track_segment_xml.getAttribute('id')
            intId = int(tsId.split("_")[0]) # get a unique, integer ID
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

            track_segs_xml = station_xml.getElementsByTagName('TrackSegments')[0]
            ts_ids = [self._to_numeric_id(id_xml) for id_xml in track_segs_xml.getElementsByTagName('ID')]

            platform_xml = station_xml.getElementsByTagName('Platform')[0] # Only one platform for GTF data
            berth_length = berth_length = float(platform_xml.getAttribute('berth_length'))
            berth_count = len(platform_xml.getElementsByTagName('Berth'))
            platform_ts_id = self._to_numeric_id(platform_xml.getElementsByTagName('TrackSegment')[0])

            stations[station_int_id] = Station(station_int_id, berth_length, berth_count, ts_ids, platform_ts_id)
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
                                               float(jerk_xml.getAttribute('normal_max')),
                                               float(jerk_xml.getAttribute('normal_min')),
                                               float(accel_xml.getAttribute('normal_max')),
                                               float(accel_xml.getAttribute('normal_min')),
                                               float(vel_xml.getAttribute('normal_max')))
        return vehicles

    def load_legs(self, transit_xml, sim_end_time):
        """Returns a heapq.heap containing Leg instances (sorted by their
        departure times)."""

        feed_start_time = to_seconds(transit_xml.getAttribute('start_time'))
        legs = [[] for i in range(len(self.vehicles))] # empty list for each vehicle
        for route_xml in transit_xml.getElementsByTagName('Route'):
            for trip_xml in route_xml.getElementsByTagName('Trip'):
                vehicle = int(trip_xml.getAttribute('vehicle_id').split('_')[0])
                stops_xml = trip_xml.getElementsByTagName('Stop')
                for origin_xml, dest_xml in pairwise(stops_xml):
                    origin_station = int(origin_xml.getAttribute('station_id').split('_')[0])
                    dest_station = int(dest_xml.getAttribute('station_id').split('_')[0])

                    origin_ts = self.station2track[origin_station]
                    dest_ts = self.station2track[dest_station]

                    depart = to_seconds(origin_xml.getElementsByTagName('Departure')[0].firstChild.data)
                    arrive = to_seconds(dest_xml.getElementsByTagName('Arrival')[0].firstChild.data)

                    depart -= feed_start_time
                    arrive -= feed_start_time

                    if depart > sim_end_time:
                        break

                    legs[vehicle].append(Leg(vehicle, origin_station,
                                             dest_station, origin_ts,
                                             dest_ts, depart, arrive))

        # Each list of legs is sorted with earliest leg at the end of the list.
        for leg_list in legs:
            leg_list.sort()
            leg_list.reverse()
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
        """Returns a 3-tuple. The first element is the next Leg instance, as
        sorted by departure time. The second element is the path length
        (in meters), and the third is the path.

        The path length (dist) is the length from the start of origin to the
        start of dest.

        vehicle: A vehicle's integer id. If None, then the earliest leg is used,
                 without regard to vehicle.
        """
        if vehicle_id == None:
            earliest_time = 1E3000 # inf
            earliest_v = 0
            for v, leg_list in enumerate(self.legs):
                if leg_list[-1].depart < earliest_time:
                    earliest_time = leg_list[-1].depart
                    earliest_v = v
            vehicle_id = earliest_v

        leg = self.legs[vehicle_id].pop()
        dist, path = self.get_path(leg.origin_ts, leg.dest_ts)
        return (leg, dist, path)

    def make_trajectory(self, leg, dist, start_time):

        # Need to find cruising velocity such that the vehicle will arrive at
        # the destination on-time or slightly early. An initial estimate assumes
        # infinite jerk, which will make the vehicle arrive slightly too late.
        # The max_speed is rounded up to the nearest 10 km/hr increment, and
        # then increased in 10 km/hr increments until the vehicle arrives on-time
        # or better.

        vehicle = self.vehicles[leg.vehicle]
        dest_station = self.stations[leg.dest_station]

        total_time = leg.arrive - leg.depart

        # Find the total distance that needs to be traveled. Assume that the
        # vehicle will stop at the furthest berth at the destination station.
        dest_dist = dest_station.berth_length * dest_station.berth_count - 0.2
        total_dist = dist - vehicle.pos + dest_dist

        # check that the time allowed to get between stations is reasonable. If
        # it's not, then choose a time such that it can be achieved without
        # ridiculous speed.
        if total_time == 0 or total_dist/total_time > vehicle.v_max:
            total_time = total_dist/vehicle.v_max

        # SEE: trajectory_calcs.py, prob4
        A = (1/(2*vehicle.a_max) - 1/(2*vehicle.a_min))  # (1/(2*an) - 1/(2*ax))
        B = (total_time)                                 # (t_total)
        C = -(total_dist)                                # -(q_total)
        v_max = max(TrajectorySolver.nonnegative_roots(A, B, C))
        v_max = (v_max//self.SPEED_INCREMENT + 1)*self.SPEED_INCREMENT  # Round up

        solver = TrajectorySolver(v_max, vehicle.a_max, vehicle.j_max)
        final_pos = total_dist + vehicle.pos
        cspline = solver.target_position(Knot(vehicle.pos, 0, 0, start_time),
                                         Knot(final_pos, 0, 0, None))

        while (cspline.t[-1] - cspline.t[0] > total_time):
            v_max += self.SPEED_INCREMENT # bump up the top speed.
            solver = TrajectorySolver(v_max, vehicle.a_max, vehicle.j_max)
            cspline = solver.target_position(Knot(vehicle.pos, 0, 0, start_time),
                                             Knot(final_pos, 0, 0, None))
        return cspline

class Station(object):
    def __init__(self, id, berth_length, berth_count, ts_ids, platform_ts_id):
        """id: An integer station id.
        berth_length: Length of each berth, in meters.
        berth_count: Number of berths.
        ts_ids: An iterable containing integer TrackSegment ids.
        platform_ts_id: A TrackSegment id. Will also be found in ts_ids.
        """
        self.id = id
        self.berth_length = berth_length
        self.berth_count = berth_count
        self.ts_ids = ts_ids
        self.platform_ts_id = platform_ts_id

class Vehicle(object):
    def __init__(self, id, model, capacity, ts_id, pos, j_max, j_min, a_max, a_min, v_max):
        self.id = id
        self.model = model
        self.capacity = capacity
        self.ts_id = ts_id
        self.pos = pos
        self.j_max = j_max
        self.j_min = j_min
        self.a_max = a_max
        self.a_min = a_min
        self.v_max = v_max

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

class PassengerManager(object):
    """Handles passenger movement through the transit network, routing them
    from their origin station to their destination. To accomodate the need
    for passengers to transfer between vehicles, a time-expanded graph is used.
    """

    def __init__(self, xml_path, sim_end_time):
        """xml_path: the path (including filename) to the xml scenario file
        created by TrackBuilder."""
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

        self.graph, self.node_dict = self._build_graph(doc.getElementsByTagName('GoogleTransitFeed')[0], sim_end_time)
        self.paths = self._make_paths(self.graph, self.node_dict)

        assert isinstance(self.graph, networkx.classes.DiGraph)

    def add_passenger(self, id, origin_id, dest_id, origin_time):
        """Adds the new passenger's actions to the PassengerManager.
        Passengers embark as soon as their vehicle is available, and stay on
        the vehicle unless they need to transfer or they have arrived at their
        final destination."""

##        To reduce the time required for the controller to startup,
##        the controller only builds a graph for routing passengers to
##        locations that are reached by vehicles before the simulation
##        ends. For short simulations, this means that there aren't paths
##        to every destination station.

        nodes = self.node_dict[origin_id]
        origin_node = None
        for n in nodes:
            if n.time >= origin_time:
                origin_node = n
                break
        if origin_node is None:
            return # Passenger created too late. No more vehicles are scheduled to arrive or depart from this station.

        dest_node = Node(dest_id, None)
        try:
            path = self.paths[origin_node][dest_node]
        except KeyError:
            return  # This passenger is stranded.

        next_v = self.graph[path[-2]][path[-1]]['vehicle']
        embarked = False

        # If the pax made it to her final destination before the sim is
        # scheduled to stop, then disembark.
        if path[-1].station == dest_id:
            path[-1].disembark_pax[next_v]

        # Iterate the path in reverse, noting when to embark/disembark on the nodes.
        for b, a in pairwise(reversed(path)):
            v = self.graph[a][b]['vehicle']
            if v == next_v:
                continue
            elif v == api.NONE_ID:
                # if next_v in list of vehicles arriving at b
                if next_v in [edge[2]['vehicle'] for edge in self.graph.in_edges(b, data=True)]:
                    b.embark_pax[next_v].append(id)
                    embarked = True
            else: # v != next_v and v != api.NONE_ID
                b.disembark_pax[v].append(id)
                if not embarked:
                    b.embark_pax[next_v].append(id)
                    embarked = False
                next_v = v

        # Handle the case where the pax is created at a station after its
        # vehicle has already arrived (but before the vehicle departed, obviously)
        if not embarked:
            path[0].embark_pax[next_v].append(id)

    def get_pax_actions(self, vehicle_id, station_id, time):
        """Returns the 2-tuple (disembarking_pax, embarking_pax), where each
        element of the tuple is a list of passenger ids. The first list
        specifies which passengers should disembark from the vehicle, and the
        second specifies which should embark.

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

        return (node.disembark_pax[vehicle_id], node.embark_pax[vehicle_id])

    def _build_graph(self, gtf_xml, sim_end_time):
        """Returns a networkx.DiGraph suitable for routing passengers over and
        a dictionary whose keys are station_ids and whose values are lists
        containing all Node instances for that station_id.

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
        graph, node_dict = build_graph(xml)
        """
        graph = networkx.DiGraph()

        ### Create edges that represent travelling between stations on a vehicle ###
        feed_start_time = to_seconds(gtf_xml.getAttribute('start_time'))
        for route_xml in gtf_xml.getElementsByTagName('Route'):
            for trip_xml in route_xml.getElementsByTagName('Trip'):
                vehicle = int(trip_xml.getAttribute('vehicle_id').split('_')[0])
                stops_xml = trip_xml.getElementsByTagName('Stop')
                for origin_xml, dest_xml in izip(stops_xml[:-1], stops_xml[1:]): # iterate over all stops pairwise
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
        # timestamp (ascending order) and add edges between them. Edges are
        # weighted by their wait time and contain api.NONE_ID as the vehicle id.
        for node_list in node_dict.itervalues():
            node_list.sort(key=attrgetter('time'))
            for a, b in pairwise(node_list):
                graph.add_edge(a, b, weight=b.time-a.time, vehicle=api.NONE_ID)

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
                    to_delete = []
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
        time: the time, in seconds, corresponding to the arrival/departure
        """
        self.station = station_id
        self.time = time

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

if __name__ == '__main__':
    main()
