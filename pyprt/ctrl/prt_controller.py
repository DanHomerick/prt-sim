from __future__ import division

if __name__ == '__main__':
    # Ensure that other modules are imported from the same version of pyprt
    import sys
    import os.path
    abs_file = os.path.abspath(__file__)
    rel_pyprt_path = os.path.dirname(abs_file) + os.path.sep + os.path.pardir + os.path.sep + os.path.pardir
    abs_pyprt_path = os.path.abspath(rel_pyprt_path)
    sys.path.insert(0, abs_pyprt_path)

import optparse
from collections import defaultdict
from collections import deque
import collections
import warnings
import math
import operator

import networkx
from numpy import arange, inf

import pyprt.shared.utility as utility
import pyprt.shared.api_pb2 as api
from pyprt.ctrl.base_controller import BaseController
from pyprt.ctrl.trajectory_solver import TrajectorySolver, FatalTrajectoryError
from pyprt.shared.cubic_spline import Knot, CubicSpline
from pyprt.shared.utility import pairwise, is_string_false, is_string_true, sec_to_hms
##from pyprt.shared.utility import deque # extension of collections.deque which includes 'insert' method

def load_scenario(scenario):
    """Parses the scenario's xml data.
    Initializes the Tracks, Stations, VehicleModels, Vehicles, Merges,
    and Switches singletons.

    Returns: None
    """
    import xml.dom.minidom
    doc = xml.dom.minidom.parseString(scenario)
    Tracks.initialize(doc.getElementsByTagName('TrackSegments')[0])
    tracks = Tracks()

    Stations.initialize(doc.getElementsByTagName('Stations')[0], tracks._graph) # TODO: Use tracks object
    stations = Stations()

    VehicleModels.initialize(doc.getElementsByTagName('VehicleModels')[0])
    vehicle_models = VehicleModels()

    Vehicles.initialize(doc.getElementsByTagName('Vehicles')[0], vehicle_models)
    vehicles = Vehicles()

    Merges.initialize(doc.getElementsByTagName('Merges')[0],
                      stations,
                      tracks._graph) # TODO: Use the tracks object

    Switches.initialize(doc.getElementsByTagName('Switches')[0],
                        stations,
                        tracks._graph)

    VehicleManager.initialize(tracks, stations, vehicle_models, vehicles)
    doc.unlink() # facilitates garbage collection

def validate_scenario(controller):
    """Checks that the scenario does not violate any of the controllers requirements.
    Requirements checked are:
      - Merge's zones of control are at least large enough to stop and restart
        a vehicle.
      - ...  # TODO!

    controller: A subclass of BaseController.

    Requires load_scenario to have already been called.
    """
    # Singletons
    stations = Stations()
    vehicles = Vehicles()
    v_manager = VehicleManager()
    merges = Merges()

    # Check that the merge zones are large enough to accomodate a vehicle stopping and starting
    maneuver_dist = -1
    # Find the max maneuver distance for all vehicles. # TODO: Really just need to check vehicle models
    for v in vehicles.itervalues():
        maneuver_dist = max(maneuver_dist, v.get_dist_to_stop() + v.get_dist_to_line_speed())

    for m in merges.itervalues():
        if maneuver_dist < m.zone_lengths[0]:
            msg = api.CtrlScenarioError()
            msg.mergeID = m.id
            msg.trackID = m.outlet
            msg.error_message = "Merge %d has insufficient track distance from ts %d to ts %d for pre-merge manuevering." % (m.id, m.inlets[0], m.outlet)
            controller.send(api.CTRL_SCENARIO_ERROR, msg)
        elif maneuver_dist < m.zone_lengths[1]:
            msg = api.CtrlScenarioError()
            msg.mergeID = m.id
            msg.trackID = m.outlet
            msg.error_message = "Merge %d has insufficient track distance from ts %d to ts %d for pre-merge manuevering." % (m.id, m.inlets[1], m.outlet)
            controller.send(api.CTRL_SCENARIO_ERROR, msg)
        else:
            pass

    # If every single berth is full, then a vehicle will not have anywhere
    # to go if it's kicked out of a station (to make room for incoming vehicles).
    total_vacancies = sum(station.get_num_storage_vacancies(v_manager.MODEL_NAME) for station in stations.itervalues())
    total_berths = sum(station.NUM_BERTHS for station in stations.itervalues())
    total_vehicles = len(vehicles)
    if total_vehicles >= total_berths + total_vacancies:
        msg = api.CtrlScenarioError()
        msg.error_message = \
            "Total number of vehicles must be less than the total number of " \
            "station berths.\nVehicles: %d\nBerths: %d" % (total_vehicles, total_berths)
        controller.send(api.CTRL_SCENARIO_ERROR, msg)
        controller.log.error(msg.error_message)

    # TODO Check that vehicles' initial positions are not too close to a merge.

def _to_numeric_id(element):
    """For elements similar to:
        <ID>x_trackSegment_forward</ID>
    where x is an integer. Returns just the integer value."""
    return int(element.childNodes[0].data.split('_')[0])

class Tracks(utility.Singleton):
    """Holds a graph representation of the track network and is responsible
    for providing shortest path information.
    """

    LENGTH = 'length'
    MAX_SPEED = 'max_speed'
    TRAVEL_TIME = 'travel_time'

    @classmethod
    def initialize(cls, track_segments_xml):
        """Initializes the singleton object.
        track_segments_xml: An xml object corresponding to the 'TrackSegments' entity.
        """
        tracks = cls()
        tracks._graph = tracks._build_graph(track_segments_xml)

    def _build_graph(self, track_segments_xml):
        """Returns a networkx.DiGraph suitable for routing vehicles over.
        Each node represents one track segment. Each edge represents the
        connection to another track regment. The edges hold data that
        actually belongs to the edge's source node, such as length, and
        max_speed.

        A track segment that doesn't connect to anybody (a dead end) still
        requires a node to connect to in order for there to be an edge
        which can hold the segment's length and speed data. At this time,
        dead ends are not supported.
        """
        graph = networkx.DiGraph()

        for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
            trackID = track_segment_xml.getAttribute('id')
            intId = int(trackID.split("_")[0]) # get a unique, integer ID
            length = float(track_segment_xml.getAttribute(Tracks.LENGTH))
            max_speed = float(track_segment_xml.getAttribute(Tracks.MAX_SPEED))

            graph.add_node(intId)

            connect_to_xml = track_segment_xml.getElementsByTagName('ConnectsTo')[0]
            for id_xml in connect_to_xml.getElementsByTagName('ID'):
                connect_id = _to_numeric_id(id_xml)
                graph.add_edge(intId, connect_id, {Tracks.LENGTH:length,
                                                   Tracks.MAX_SPEED:max_speed,
                                                   Tracks.TRAVEL_TIME:length/max_speed})
        return graph

    def get_path(self, ts_1, ts_2):
        """Returns a two tuple. The first element is the path length (in meters),
        and the second is the path. ts_1 and ts_2 are integer trackSegment ids.
        The path is the shortest path, as weighted by vehicle travel time."""
        weight, path = networkx.bidirectional_dijkstra(self._graph, ts_1, ts_2, weight=Tracks.TRAVEL_TIME)
        length = self.get_path_length(path)
        return (length, path)

    def get_path_length(self, path):
        """Returns the length of a path, in meters."""
        length = 0
        for orig, dest in pairwise(path):
            length += self._graph[orig][dest][Tracks.LENGTH]
        return length

    def get_paths(self, source_ts_id, target_ts_ids):
        """More efficient than calling get_path repeatedly with the same source.

        source_ts_id: Integer id for the source track segment.
        dest_ts_ids: An iterable containing integer ids for destinations.

        Returns best paths, and their distances.
        distance,path : dictionaries
            Returns a tuple of two dictionaries keyed by node.
            The first dictionary stores distance from the source.
            The second stores the path from the source to that node.
        """
        weights, paths = networkx.single_source_dijkstra(self._graph, source_ts_id, weight=Tracks.TRAVEL_TIME)
        paths = dict( (target, paths[target]) for target in target_ts_ids)
        lengths = dict( (target, self.get_path_length(paths[target])) for target in target_ts_ids)

        return lengths, paths

    def coordinate_shift(self, idx, path):
        """Returns a numeric offet that when subtracted from a position in path[0]'s
        coordinate frame translates it to path[idx]'s coordinate frame.
        The path is a list of ts_ids."""
        if idx == 0:
            return 0

        return sum(self._graph[a][b][Tracks.LENGTH] for a, b in pairwise(path[:idx+1]))

    def get_speed_zones(self, path):
        """Returns a list of 2-tuples, wherein each tuple consists of:
            (max_speed, path_fragment)

        Example:
        >>> path = [0,1,2,3,4,5]
        >>> get_speed_zones(path)
        [(10, [0,1,2]), (5, [3]), (10, [4]), (10, [5])]

        In the above example, track segments 0,1,2 have a speed limit of 10.
        Track segment 3 has a speed limit of 5. Track segment 4 has a speed limit
        of 10. Track segment 5 has a speed limit of 10.
        """
        if len(path) == 0:
            return []

        wp = path[:] # working path
        # choose a successor to the last segment on the path arbitrarily.
        # Edge data is the same for all successors.
        try:
            successor = self._graph.successors(wp[-1])[0]
        except IndexError:
            raise ValueError("Path may not contain a dead end segment.")
        wp.append(successor)

        result = []
        last_idx = 0
        last_speed = self._graph[wp[0]][wp[1]][Tracks.MAX_SPEED]
        for i in xrange(len(wp)-1):
            curr_speed = self._graph[wp[i]][wp[i+1]][Tracks.MAX_SPEED]
            if last_speed != curr_speed:
                result.append( (last_speed, wp[last_idx:i]) )
                last_idx = i
                last_speed = curr_speed

        result.append( (curr_speed, wp[last_idx:-1]) )
        return result

    def predecessors(self, ts_id):
        return self._graph.predecessors(ts_id)

    def predecessors_iter(self, ts_id):
        return self._graph.predecessors_iter(ts_id)

    def successors(self, ts_id):
        return self._graph.successors(ts_id)

    def successors_iter(self, ts_id):
        return self._graph.successors_iter(ts_id)

    # Deprecated until such time that profiling proves the worth.
    # Code will need an overhaul to work. Note that the call to bidir_dijkstra
    #    is returning the path weight, not length.
##    def _build_station_tables(self):
##        """Returns a pair of 2D tables containing distances and paths. The tables
##        are accessed like table[origin][dest], where each origin is a station's
##        LOAD ts and the dest is a station's UNLOAD ts."""
##        origins = [s.ts_ids[Station.LOAD] for s in self.stations.values()]
##        dests = [s.ts_ids[Station.UNLOAD] for s in self.stations.values()]
##
##        path_dist_dict = defaultdict(dict)
##
##        for o in origins:
##            for d in dests:
##                path_dist_dict[o][d] = networkx.bidirectional_dijkstra(self.graph, o, d)
##
##        return path_dist_dict

class Stations(collections.Mapping, utility.Singleton):
    """Behaves as a dict wherein the keys are integer station id's, and the
    values are Station instances.
    Offers some convience methods for locating stations that are associated
    with particular track segments.
    """

    BERTH_GAP = 0.1 # Distance from the front of the vehicle to the end of the berth

    @classmethod
    def initialize(cls, stations_xml, graph):
        """Initializes the singleton object.
        stations_xml: An xml object corresponding to the 'Stations' entity.
        graph: A networkx.DiGraph describing the track network.
        """
        stations = cls()

        # A dict of Station instances, keyed by an integer id.
        stations._dict = stations._deserialize(stations_xml, graph)

        # A dict mapping ts_ids to Station instances.
        stations._track2station = dict((ts, s) for s in stations._dict.itervalues() for ts in s.ts_ids)

        # A dict mapping split ts_ids to Station instances.
        stations._split2station = dict((s.split, s) for s in stations._dict.itervalues())

        # A dict mapping merge ts_ids to Station instances.
        stations._merge2station = dict((s.merge, s) for s in stations._dict.itervalues())

        # A list of station load track segment ids, one per station.
        stations._load_ts_ids = [s.ts_ids[Station.LOAD] for s in stations._dict.itervalues()]

    def __len__(self):
        return len(self._dict)

    def __iter__(self):
        return iter(self._dict)

    def __getitem__(self, key):
        return self._dict[key]

    def _deserialize(self, stations_xml, graph):
        """Returns a dict of stations, keyed by the integer id."""
        stations = dict()
        for station_xml in stations_xml.getElementsByTagName('Station'):
            station_str_id = station_xml.getAttribute('id')
            station_int_id = int(station_str_id.split('_')[0])

            ts_ids_xml = station_xml.getElementsByTagName('TrackSegmentID')[:9] # fragile assumption :(
            ts_ids = [_to_numeric_id(id_xml) for id_xml in ts_ids_xml]

            # Get berth position data for the three platforms: Unload, Queue, Load
            platforms_xml = station_xml.getElementsByTagName('Platform')

            # Contains (platform_id, berth_id) pairs.
            storage_entrances = []
            storage_exits = []

            unload_xml = platforms_xml[0]
            platform_ts_id = _to_numeric_id(unload_xml.getElementsByTagName('TrackSegmentID')[0])
            assert platform_ts_id == ts_ids[Station.UNLOAD]
            unload_positions = []
            for berth_id, berth_xml in enumerate(unload_xml.getElementsByTagName('Berth')):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                unload_positions.append(end_pos - self.BERTH_GAP)
                if is_string_true(berth_xml.getAttribute('storage_entrance')):
                    storage_entrances.append( (0, berth_id) )
                if is_string_true(berth_xml.getAttribute('storage_exit')):
                    storage_exits.append( (0, berth_id) )

            queue_xml = platforms_xml[1]
            platform_ts_id = _to_numeric_id(queue_xml.getElementsByTagName('TrackSegmentID')[0])
            assert platform_ts_id == ts_ids[Station.QUEUE]
            queue_positions = []
            for berth_id, berth_xml in enumerate(queue_xml.getElementsByTagName('Berth')):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                queue_positions.append(end_pos - self.BERTH_GAP)
                if is_string_true(berth_xml.getAttribute('storage_entrance')):
                    storage_entrances.append( (1, berth_id) )
                if is_string_true(berth_xml.getAttribute('storage_exit')):
                    storage_exits.append( (1, berth_id) )

            load_xml = platforms_xml[2]
            platform_ts_id = _to_numeric_id(load_xml.getElementsByTagName('TrackSegmentID')[0])
            assert platform_ts_id == ts_ids[Station.LOAD]
            load_positions = []
            for berth_id, berth_xml in enumerate(load_xml.getElementsByTagName('Berth')):
                end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                load_positions.append(end_pos - self.BERTH_GAP)
                if is_string_true(berth_xml.getAttribute('storage_entrance')):
                    storage_entrances.append( (2, berth_id) )
                if is_string_true(berth_xml.getAttribute('storage_exit')):
                    storage_exits.append( (2, berth_id) )

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

            onramp_length = networkx.dijkstra_path_length(graph,
                                            ts_ids[Station.ACCEL], merge)

            storage_dict = {}
            for storage_xml in station_xml.getElementsByTagName('Storage'):
                v_model_name = storage_xml.getAttribute('model_name')
                initial_supply_str = storage_xml.getAttribute('initial_supply').lower()
                if initial_supply_str == 'inf':
                    initial_supply = float('inf')
                else:
                    initial_supply = int(storage_xml.getAttribute('initial_supply'))

                max_capacity_str = storage_xml.getAttribute('max_capacity').lower()
                if max_capacity_str == 'inf':
                    max_capacity = float('inf')
                else:
                    max_capacity = int(storage_xml.getAttribute('max_capacity'))
                storage = Storage(v_model_name,
                                  initial_supply,
                                  max_capacity)

                storage_dict[v_model_name] = storage
                self.MODEL_NAME = v_model_name

            stations[station_int_id] = Station(station_int_id, ts_ids, split, bypass, merge, onramp_length,
                                               unload_positions, queue_positions, load_positions,
                                               storage_entrances, storage_exits, storage_dict)

        return stations

    def get_from_ts(self, ts_id):
        """Returns a Station instance that contains the TrackSegment identified
        by ts_id. Returns None if no Station contains the TrackSegment."""
        return self._track2station.get(ts_id)

    def get_from_split_ts(self, split_ts_id):
        """Returns a Station instance who's "split" TrackSegment is identified
        by split_ts_id. Returns None if no Station matches."""
        return self._split2station.get(split_ts_id)

    def get_from_merge_ts(self, merge_ts_id):
        """Returns a Station instance who's "merge" TrackSegent is identified
        by merge_ts_id. Returns None if no Station matches."""
        return self._merge2station.get(merge_ts_id)

    def get_load_ts_ids(self):
        return self._load_ts_ids

class VehicleModels(collections.Mapping, utility.Singleton):
    """Behaves as a dict of vehicle models, keyed by model name. Each vehicle
    model is just a regular Vehicle object, with no location set.
    The models are to be used as a prototype for other vehicles.
    """

    @classmethod
    def initialize(cls, models_xml):
        """Initializes the singleton object.
        models_xml: An xml object corresponding to the 'VehicleModels' entity.
        """
        models = cls()

        # A dict of Vehicle instances. Keyed by string model names.
        models._dict = models._deserialize(models_xml)

    def __len__(self):
        return len(self._dict)

    def __iter__(self):
        return iter(self._dict)

    def __getitem__(self, key):
        return self._dict[key]

    def _deserialize(self, models_xml):
        models = dict()
        for model_xml in models_xml.getElementsByTagName('VehicleModel'):
            model_name = model_xml.getAttribute('model_name')
            length = float(model_xml.getAttribute('length'))
            capacity = int(model_xml.getAttribute('passenger_capacity'))
            jerk_xml = model_xml.getElementsByTagName('Jerk')[0]
            accel_xml = model_xml.getElementsByTagName('Acceleration')[0]
            vel_xml = model_xml.getElementsByTagName('Velocity')[0]

            model_vehicle = Vehicle(api.NONE_ID,
                              model_name,
                              length,
                              capacity,
                              api.NONE_ID,
                              0,  # pos
                              0,  # vel
                              0,  # accel
                              float(jerk_xml.getAttribute('normal_max')),
                              float(jerk_xml.getAttribute('normal_min')),
                              float(accel_xml.getAttribute('normal_max')),
                              float(accel_xml.getAttribute('normal_min')),
                              float(vel_xml.getAttribute('normal_max')))
            models[model_name] = model_vehicle
        return models

class Vehicles(collections.Mapping, utility.Singleton):
    """Behaves as a dict wherein the keys are integer vehicle id's, and the
    values are Vehicle instances.
    """

    @classmethod
    def initialize(cls, vehicles_xml, vehicle_models):
        """Initializes the singleton object.
        vehicles_xml: An xml object corresponding to the 'Vehicles' entity.
        vehicles_models: A 'VehicleModels' instance.
        """
        vehicles = cls()

        # A dict containing Vehicle instances. Keyed by integer ids.
        vehicles._dict = vehicles._deserialize(vehicles_xml, vehicle_models)

        return vehicles

    def __len__(self):
        return len(self._dict)

    def __iter__(self):
        return iter(self._dict)

    def __getitem__(self, key):
        return self._dict[key]

    def __setitem__(self, key, value):
        self._dict[key] = value

    def _deserialize(self, vehicles_xml, v_models):
        """Parses the supplied xml and returns a dict of Vehicle instances keyed by id.
        vehicles_xml: An xml object corresponding to the 'Vehicles' entity.
        v_models: A VehicleModels instance.
        """
        vehicles = dict()

        for vehicle_xml in vehicles_xml.getElementsByTagName('Vehicle'):
            vehicle_str_id = vehicle_xml.getAttribute('id')
            vehicle_int_id = int(vehicle_str_id.split('_')[0])

            ts_str_id = vehicle_xml.getAttribute('location')
            ts_int_id = int(ts_str_id.split('_')[0])

            model_name = vehicle_xml.getAttribute('model_name')
            model = v_models[model_name]

            v = Vehicle(vehicle_int_id,
                        model_name,
                        model.length,
                        model.capacity,
                        ts_int_id,
                        float(vehicle_xml.getAttribute('position')),
                        float(vehicle_xml.getAttribute('velocity')),
                        float(vehicle_xml.getAttribute('acceleration')),
                        model.j_max,
                        model.j_min,
                        model.a_max,
                        model.a_min,
                        model.v_max)

            vehicles[vehicle_int_id] = v
        return vehicles

class Merges(collections.Mapping, utility.Singleton):
    """Behaves as a dict wherein the keys are integer merge id's,
    and the values are Merge instances.
    """

    @classmethod
    def initialize(cls, merges_xml, stations, graph):
        """Initializes the singleton object.
        merges_xml: An xml object corresponding to the 'Merges' entity.
        stations: The 'Stations' instance.
        graph: A networkx.DiGraph representing the track layout. TODO: Replace with Tracks instance?
        """
        merges = cls()

        # A dict of Merge instances. Keyed by integer id.
        merges._dict = merges._deserialize(merges_xml, stations, graph)

        # Mapping from track segment ids to Merge instance.
        merges._track2merge = dict((ts, m) for m in merges._dict.itervalues() for ts in m.zone_ids[0] + m.zone_ids[1])

        # Mapping from inlets to Merge instance.
        merges._inlet2merge = dict((ts, m) for m in merges._dict.itervalues() for ts in m.inlets)

        # Mapping from outlets to Merge instance
        merges._outlet2merge = dict((m.outlet, m) for m in merges._dict.itervalues())

    def __len__(self):
        return len(self._dict)

    def __iter__(self):
        return iter(self._dict)

    def __getitem__(self, key):
        return self._dict[key]

    def _deserialize(self, merges_xml, stations, graph):
        """Returns a dict of Merge instances keyed by id. Ignores merges that
        are the result of a station on-ramp connecting to the main line.
        merges_xml: An xml object corresponding to the 'Merges' entity.
        stations: The 'Stations' instance.
        graph: A networkx.DiGraph describing the track network.
        """
        merges = {}
        for merge_xml in merges_xml.getElementsByTagName('Merge'):
            # don't include station merges.
            outgoing_xml = merge_xml.getElementsByTagName('Outgoing')[0]
            outgoing_id = _to_numeric_id(outgoing_xml)
            if not stations.get_from_merge_ts(outgoing_id):
                str_merge_id = merge_xml.getAttribute('id')
                merge_id = int(str_merge_id.split('_')[0])
                merges[merge_id] = Merge(merge_id, outgoing_id, graph)
        return merges

    def get_from_ts(self, ts_id):
        """Returns a Merge instance that is associated with the track segment id.
        Returns None if no Merge is found."""
        return self._track2merge.get(ts_id)

    def get_from_inlet_ts(self, ts_id):
        """Returns the Merge instance that has ts_id as an inlet. Returns
        None if no Merge is found."""
        return self._inlet2merge.get(ts_id)

    def get_from_outlet_ts(self, ts_id):
        """Returns the Merge instance that has ts_id an an outlet. Returns
        None if no Merge is found."""
        return self._outlet2merge.get(ts_id)

class Switches(collections.Mapping, utility.Singleton):
    """Behaves as a dict wherein the keys are integer switch id's, and the
    values are Switch instances.
    """

    @classmethod
    def initialize(cls, switches_xml, stations, graph):
        """Initializes the singleton object.
        switches_xml: An xml object corresponding to the 'Switches' entity.
        stations: The 'Stations' instance.
        graph: A networkx.DiGraph representing the track layout. TODO: Replace with Tracks instance?
        """
        switches = cls()

        # A dict of Switch instances. Keyed by integer id.
        switches._dict = switches._deserialize(switches_xml, stations, graph)

    def __len__(self):
        return len(self._dict)

    def __iter__(self):
        return iter(self._dict)

    def __getitem__(self, key):
        return self._dict[key]

    def _deserialize(self, switches_xml, stations, graph):
        """Returns a dict of Switch instances keyed by id. Ignores switch that
        are the result of a station off-ramp departing from the main line.
        switches_xml: An xml object corresponding to the 'Switches' entity.
        stations: The 'Stations' instance.
        graph: A networkx.DiGraph describing the track network.
        """
        switches = {}
        for switch_xml in switches_xml.getElementsByTagName('Switch'):
            # don't include station switches
            incoming_xml = switch_xml.getElementsByTagName('Incoming')[0]
            incoming_id = _to_numeric_id(incoming_xml)
            if not stations.get_from_split_ts(incoming_id):
                str_switch_id = switch_xml.getAttribute('id')
                switch_id = int(str_switch_id.split('_')[0])
                switches[switch_id] = Switch(switch_id, incoming_id, graph)
        return switches


class NoPaxAvailableError(Exception):
    """No passengers are available."""

class NoBerthAvailableError(Exception):
    """No station berth availaible."""

class NoVehicleAvailableError(Exception):
    pass

class NoVacancyAvailableError(Exception):
    pass

class States(object):
    """Vehicles use a state machine (diagram in /docs) with these states.
    'ADVANCING' means that the vehicle is moving between berths.
    """
    NONE = 0                    # No state has been set yet.
    RUNNING = 1                 # Travelling from one station to the next.
    UNLOAD_ADVANCING = 2        # Have passengers to unload, on or approaching unload platform.
    UNLOAD_WAITING = 3          # Capable of immediately disembarking passengers.
    DISEMBARKING = 4            # Parked in unload berth, currently unloading passengers.
    LOAD_WAITING = 5            # Capable of immediately embarking passengers.
    LOAD_ADVANCING = 6          # On or approaching the load platform.
    EMBARKING = 7               # Parked in load berth, loading passengers
    STORAGE_ENTERING = 8        # Parked in a berth, in the process of moving into storage.
    STORAGE = 9                 # In storage
    STORAGE_EXITING = 10        # In the process of moving from storage to a berth
    EXIT_WAITING = 11           # Waiting to reach launch berth.
    EXIT_ADVANCING = 12         # Moving towards launch berth.
    LAUNCH_WAITING = 13         # In the launch berth. Ready to lauch at any time.
    LAUNCHING = 14              # Accelerating towards the main line.

    strings = ["NONE",
               "RUNNING",
               "UNLOAD_ADVANCING",
               "UNLOAD_WAITING",
               "DISEMBARKING",
               "LOAD_WAITING",
               "LOAD_ADVANCING",
               "EMBARKING",
               "STORAGE_ENTERING",
               "STORAGE",
               "STORAGE_EXITING",
               "EXIT_WAITING",
               "EXIT_ADVANCING",
               "LAUNCH_WAITING",
               "LAUNCHING"]

    @staticmethod
    def to_string(state):
        return States.strings[state]

class PrtController(BaseController):
    LINE_SPEED = None  # in meter/sec. Set in main().
    HEADWAY = None     # in sec. Measured from tip-to-tail. Set in main()
    HEARTBEAT_INTERVAL = 5.1  # in seconds. Choosen arbitrarily for testing
    DEMAND_THRESHOLD = 3

    def __init__(self, log_path, commlog_path, stats_path):
        super(PrtController, self).__init__(log_path, commlog_path)
        self.t_reminders = dict() # keyed by time (in integer form), values are pairs of lists
        self.stats_path = stats_path

        # Singletons. They do not contain data until they are initialized, which
        # occurs upon receipt of the SimGreeting message.
        self.v_manager = VehicleManager()
        self.v_models = VehicleModels()
        self.vehicles = Vehicles()
        self.tracks = Tracks()
        self.stations = Stations()
        self.merges = Merges()
        self.switches = Switches()

    def set_v_notification(self, vehicle, time):
        """Request a time notification from sim and store which vehicle the
        notification is relevant to."""
        key = int(time*1000)
        try:
            v_list, fnc_list = self.t_reminders[key]
            if vehicle not in v_list:
                v_list.append(vehicle)
        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([vehicle], [])


    def set_fnc_notification(self, fnc, args, time):
        """Request that function fnc be called with arguments args when time is
        reached."""
        key = int(time*1000)
        try:
            v_list, fnc_list = self.t_reminders[key]
            fnc_list.append( (fnc, args) )
        except KeyError:
            notify = api.CtrlSetnotifyTime()
            notify.time = time
            self.send(api.CTRL_SETNOTIFY_TIME, notify)
            self.t_reminders[key] = ([], [ (fnc, args) ])

    def request_v_status(self, vehicle_id):
        msg = api.CtrlRequestVehicleStatus()
        msg.vID = vehicle_id
        self.send(api.CTRL_REQUEST_VEHICLE_STATUS, msg)

    def trigger_advance(self, vehicle, station):
        """Triggers a vehicle to advance, if able. May be called on a vehicle
        that is not ready to advance, in which case the function is a no-op.
        """
        assert isinstance(vehicle, Vehicle)
        assert isinstance(station, Station)

        # Exclude vehicles that have reserved berths, but aren't yet in the
        # relevant states.
        if vehicle.state not in (States.UNLOAD_WAITING, States.LOAD_WAITING, States.EXIT_WAITING, States.NONE):
            return

        assert vehicle.station is station

        # if vehicle hasn't unloaded, only try to advance as far as the last unload berth
        try:
            if vehicle.state is States.UNLOAD_WAITING:
                station.request_unload_berth(vehicle)
                vehicle.state = States.UNLOAD_ADVANCING
            elif vehicle.state is States.LOAD_WAITING:
                station.request_load_berth(vehicle)
                vehicle.state = States.LOAD_ADVANCING
            elif vehicle.state is States.EXIT_WAITING:
                station.request_launch_berth(vehicle)
                vehicle.state = States.EXIT_ADVANCING
            else:
                raise Exception("Unexpected case. Vehicle.state is: %s" % vehicle.state)
        except NoBerthAvailableError:
            return

        # Will only make it to this point in the code if the vehicle has been
        # given a new berth to travel to.
        vehicle.advance()

    ### Overriden message handlers ###
    def on_SIM_GREETING(self, msg, msgID, msg_time):
        self.sim_end_time = msg.sim_end_time


        self.log.info("Sim Greeting message received. Sim end at: %f" % msg.sim_end_time)

        # Initialize the classes that hold scenario info.
        load_scenario(msg.scenario_xml)
        validate_scenario(self)

    def on_SIM_START(self, msg, msgID, msg_time):
        """This function is responsible for getting all the vehicles moving at
        the start of the simulation. It needs to deal with 5 cases:

        Case 1: Vehicle is on a station's platform and must be given a reservation.
        Case 2: Vehicle is on a station's OFFRAMP or DECEL segments. Must
                be given a berth reservation and travel to the platform.
        Case 3: Vehicle is on a station's ONRAMP or ACCEL segments. Rather than
                trying to merge a vehicle with mainline traffic when it's
                starting from halfway down the acceleration ramp, we're just
                going to take a pass on this case. Not allowed.
        Case 4: Vehicle is outside of a Merge's zone of control and must be brought
                up to LINE_SPEED.
        Case 5: Vehicle is within a Merge's zone of control, and must be brought
                up to LINE_SPEED and granted a MergeSlot.

        """
        self.log.info("Sim Start message received.")

        v_list = self.vehicles.values()
        # Sort based on location, secondarily sorted by position -- both in descending order.
        v_list.sort(cmp=lambda x,y: cmp(y.ts_id, x.ts_id) if x.ts_id != y.ts_id else cmp(y.pos, x.pos))

        merge2vehicles = defaultdict(list)

        # Make a first pass through the vehicles, setting up the vehicles which
        # are committed to a particular station.
        for vehicle in v_list:
            assert isinstance(vehicle, Vehicle)
            assert vehicle.state is States.NONE

            station = self.stations.get_from_ts(vehicle.ts_id)

            if station:
                assert isinstance(station, Station)

                # Take advantage of the fact the ts_ids for a station are in
                # ascending order. Due to the sorting of the vehicles, the
                # berth reservation ordering will be correct.

                # Case 1. Vehicle already on a UNLOAD/QUEUE/LOAD platform.
                if vehicle.ts_id in (station.ts_ids[Station.UNLOAD],
                                     station.ts_ids[Station.QUEUE],
                                     station.ts_ids[Station.LOAD]):
                    vehicle.state = States.LOAD_ADVANCING
                    vehicle.station = station
                    station.request_load_berth(vehicle)
                    berth_dist, berth_path = self.tracks.get_path(vehicle.ts_id, vehicle.plat_ts)
                    initial = vehicle.estimate_pose(self.current_time)
                    berth_knot = Knot(berth_dist + vehicle.berth_pos, 0, 0, None)
                    try:
                        spline = vehicle.traj_solver.target_position(initial, berth_knot, max_speed=station.SPEED_LIMIT)
                        vehicle.set_spline(spline)
                        vehicle.set_path(berth_path)
                    except FatalTrajectoryError: # may occur if vehicles cannot reverse and are too closely spaced.
                        raise Exception("Unable to reserve a reachable berth for vehicle: %s" % str(vehicle))

                # Case 2. Approaching station.
                elif vehicle.ts_id in (station.ts_ids[Station.OFF_RAMP_I],
                                       station.ts_ids[Station.OFF_RAMP_II],
                                       station.ts_ids[Station.DECEL]):
                    vehicle.state = States.LOAD_ADVANCING
                    station.request_load_berth(vehicle)
                    vehicle.enter_station(station)

                # Case 3. Outbound from a station.
                elif vehicle.ts_id in (station.ts_ids[Station.ACCEL],
                                       station.ts_ids[Station.ON_RAMP_I],
                                       station.ts_ids[Station.ON_RAMP_II]):
                    raise Exception("Unable to handle a vehicle starting on the "
                                    "ACCEL or ONRAMP portions of the station. Please "
                                    "relocate vehicle: %s" % str(vehicle))

                else:
                    raise Exception("Unexpected case: vehicle.ts_id: %d" % vehicle.ts_id)

            else: # Not in a station.
                vehicle.state = States.RUNNING

                # Assign trips to vehicles. Don't travel to a station when you
                # are on it's "split" segment, since the vehicle is already
                # past the enter/wave off decision point.
                station = self.stations.get_from_split_ts(vehicle.ts_id)
                if station is None:
                    vehicle.trip = self.v_manager.deadhead(vehicle)
                else: # vehicle is on the station's "split" trackseg
                    assert vehicle.ts_id == station.split
                    vehicle.trip = self.v_manager.deadhead(vehicle, exclude=(station, ))

                vehicle.set_path(vehicle.trip.path)

                # Handle all the vehicles that are in a Merge's zone of control
                # together at a later time. If not in a zone of control, just
                # run at LINE_SPEED.
                merge = self.merges.get_from_ts(vehicle.ts_id)
                if merge is None:
                    vehicle.run(speed_limit=self.LINE_SPEED)
                else:
                    # tail must be on inlet ts, not just nose. If just the nose
                    # is on the inlet ts, let the normal code path assign a
                    # merge slot in a few moments.
                    if vehicle.ts_id in merge.inlets and vehicle.pos < vehicle.length:
                        vehicle.run(speed_limit=self.LINE_SPEED)
                    else:
                        merge2vehicles[merge].append(vehicle)

        # Case 5. Within a Merge's zone of control.
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
                v.run(speed_limit=self.LINE_SPEED) # do_merge expects the vehicle to already have a spline.
                try:
                    v.do_merge(m, 0.0)
                except FatalTrajectoryError:
                    # TODO: Do this check in validate_scenario, if practical
                    msg = api.CtrlScenarioError()
                    msg.mergeID = m.id
                    msg.vehicleID = v.id
                    msg.trackID = v.ts_id
                    msg.error_message = "Vehicle %s on track segment %s is too close to a merge to reach full speed." % \
                                        (v.id, v.ts_id)
                    self.send(api.CTRL_SCENARIO_ERROR, msg)

        # Trigger a heartberat for each station.
        for station in self.stations.itervalues():
            station.heartbeat()

    def on_SIM_NOTIFY_TIME(self, msg, msgID, msg_time):
        ms_msg_time = int(msg.time*1000) # millisec integer for easy comparison
        vehicles, functions = self.t_reminders[ms_msg_time]

        for vehicle in vehicles:
            if vehicle.state is States.LAUNCH_WAITING:
                blocked_time = self.v_manager.is_launch_blocked(vehicle.station, vehicle, self.current_time)
                if not blocked_time:
                    vehicle.state = States.LAUNCHING
                    vehicle.run(speed_limit=self.LINE_SPEED)
                    vehicle._launch_begun = True
                    vehicle.station.release_berth(vehicle)
                    self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                                  self.current_time, vehicle.id, vehicle.station.id)
                    vehicle.station.heartbeat()

                else:
                    self.set_v_notification(vehicle, blocked_time)
                    self.log.info("t:%5.3f Delaying launch of vehicle %d from station %d until %.3f (%.3f delay)",
                                  self.current_time, vehicle.id, vehicle.station.id, blocked_time, blocked_time-self.current_time)

            else:
                warnings.warn("Huh? What was this reminder for again? vehicle %d, state: %s" % (vehicle.id, vehicle.state))
##                raise Exception("Huh? What was this reminder for again? Current State: %s" % vehicle.state)

        for fnc, args in functions:
            fnc(*args)

        del self.t_reminders[ms_msg_time]

    def on_SIM_RESPONSE_VEHICLE_STATUS(self, msg, msgID, msg_time):
        vehicle = self.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        if vehicle.state is States.LAUNCH_WAITING:
            merge = self.merges.get_from_ts(vehicle.station.bypass)
            if merge and vehicle.station in merge.zoc_stations:
                vehicle.state = States.LAUNCHING
                slot, launch_time = merge.create_station_merge_slot(vehicle.station, vehicle, self.current_time)
                vehicle.set_merge_slot(slot)
                vehicle.set_spline(vehicle.merge_slot.spline)
                vehicle._launch_begun = True
                self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                              self.current_time, vehicle.id, vehicle.station.id)

                def release_and_heartbeat():
                    vehicle.station.release_berth(vehicle)
                    vehicle.station.heartbeat()

                self.set_fnc_notification(release_and_heartbeat, tuple(), launch_time)

            else:
                blocked_time = self.v_manager.is_launch_blocked(vehicle.station, vehicle, self.current_time)
                if not blocked_time:
                    vehicle.state = States.LAUNCHING
                    vehicle.send_path()
                    vehicle.run(speed_limit=self.LINE_SPEED)
                    vehicle._launch_begun = True
                    vehicle.station.release_berth(vehicle)
                    self.log.info("t:%5.3f Launched vehicle %d from station %d.",
                                  self.current_time, vehicle.id, vehicle.station.id)
                    vehicle.station.heartbeat()
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
        station = self.stations[pax.origin_id]
        station.add_passenger(pax)

        if len(self.vehicles) == 0 or station.get_demand() >= self.DEMAND_THRESHOLD:
            self.v_manager.retrieve_vehicle_from_storage(station, self.v_manager.MODEL_NAME)

        station.heartbeat()

    def on_SIM_COMPLETE_PASSENGERS_DISEMBARK(self, msg, msgID, msg_time):
        vehicle = self.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is States.DISEMBARKING
        self.log.info("t:%5.3f Vehicle %d at station %d, berth %d, plat %d has completed disembark of %s",
                       self.current_time, vehicle.id, vehicle.station.id, vehicle.berth_id, vehicle.platform_id, msg.cmd.passengerIDs)

        vehicle.state = States.LOAD_WAITING
        vehicle.trip = None
        vehicle.station.heartbeat()

    def on_SIM_COMPLETE_PASSENGERS_EMBARK(self, msg, msgID, msg_time):
        vehicle = self.vehicles[msg.cmd.vID]
        assert isinstance(vehicle, Vehicle)
        assert vehicle.state is States.EMBARKING
        self.log.info("t:%5.3f Vehicle %d at station %d, berth %d, plat %d has completed embark of %s",
                       self.current_time, vehicle.id, vehicle.station.id, vehicle.berth_id, vehicle.platform_id, msg.cmd.passengerIDs)
        vehicle.state = States.EXIT_WAITING
        vehicle.station.heartbeat()

    def on_SIM_NOTIFY_VEHICLE_ARRIVE(self, msg, msgID, msg_time):
        vehicle = self.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        # TODO: Try to move the decision process closer to the switch to improve efficiency.
        # Vehicle needs to make choice to enter station or bypass.
        # Note: Some track networks will have one station's ON_RAMP_II lead straight into
        #       another station's "split" track seg, thus LAUNCHING is a viable state.
        station = self.stations.get_from_split_ts(msg.trackID)
        if vehicle.state in (States.RUNNING, States.LAUNCHING) and station:
            # Has passengers and this is the dest station
            if vehicle.pax and station is vehicle.trip.dest_station:
                try:   # Reserve an unload berth
                    vehicle.trip.dest_station.request_unload_berth(vehicle)
                    assert vehicle.berth_id is not None
                except NoBerthAvailableError: # none available, circle around and try again
                    # plan a path to the ts prior from current (that is, loop around)
                    old_dest = vehicle.trip.dest_station
                    vehicle.trip = self.v_manager.wave_off(vehicle)
                    vehicle.set_path(vehicle.trip.path)
                    vehicle.run(speeed_limit=self.LINE_SPEED)
                    vehicle.num_wave_offs += 1
                    self.log.info("%5.3f Vehicle %d waved off from dest_station %d because NoBerthAvailable. Going to station %d instead.",
                                   self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)

            # Has passengers and not at dest station
            elif vehicle.pax and station is not vehicle.trip.dest_station:
                pass

            # Empty vehicle, any station
            else:
                # Refresh the destination station, since demand for empties has
                #   likely changed while travelling. This presents the possiblity
                #   that empties will get caught in a game of 'keep away', where
                #   their destination station keeps changing and they spend a lot
                #   of time chasing new destinations rather than accomplishing
                #   useful work. This shouldn't be much of a problem so long as
                #   vehicles strongly favor nearby stations, which they currently do.
                #
                # TODO: This refreshes the empty's destination whenever it passes by a
                #   station. The preferred approach would be to refresh whenever it
                #   comes to a switch of any sort. That is, refresh whenever the vehicle
                #   has the opportunity to act on the information.
                vehicle.trip = self.v_manager.deadhead(vehicle)
                vehicle.set_path(vehicle.trip.path)

                if vehicle.trip.dest_station is station:
                    try:
                        vehicle.trip.dest_station.request_load_berth(vehicle)
                        assert vehicle.berth_id is not None
                    except NoBerthAvailableError:
                        old_dest = vehicle.trip.dest_station
                        vehicle.trip = self.v_manager.deadhead(vehicle, exclude=(old_dest,))
                        vehicle.set_path(vehicle.trip.path)
                        self.log.info("%5.3f Empty vehicle %d bypassing dest_station %d due to lack of available berth. Going to station %d instead.",
                                  self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)
                vehicle.run(speed_limit=self.LINE_SPEED)

            # If the vehicle is empty, and the assigned berth is the entrance
            # berth, give it up and waive off so as to not clog up the station
            # with an empty vehicle.
            if vehicle.has_berth_reservation() and not vehicle.pax \
                    and vehicle.platform_id == Station.UNLOAD_PLATFORM \
                    and vehicle.berth_id == 0:
                old_dest = vehicle.trip.dest_station
                old_dest.release_berth(vehicle)
                vehicle.trip = self.v_manager.deadhead(vehicle, exclude=(old_dest,))
                vehicle.set_path(vehicle.trip.path)
                vehicle.run(speed_limit=self.LINE_SPEED)
                self.log.info("%5.3f Empty vehicle %d bypassing dest_station %d to avoid clogging entrance berth. Going to station %d instead.",
                              self.current_time, vehicle.id, old_dest.id, vehicle.trip.dest_station.id)

    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        vehicle = self.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        if vehicle.state is States.RUNNING:
            # Tail just cleared the main line by entering into a station.
            station = self.stations.get_from_split_ts(msg.trackID)
            if station is not None and vehicle.ts_id in station.ts_ids: # don't check against OFF_RAMP_I explicitly, because nose may already be past it.
                assert vehicle.berth_id is not None
                if vehicle.pax:
                    assert vehicle.platform_id == station.UNLOAD_PLATFORM
                    vehicle.state = States.UNLOAD_ADVANCING
                else:
                    vehicle.state = States.LOAD_ADVANCING

                vehicle.enter_station(station)

        elif vehicle.state is States.LAUNCHING:
            station = self.stations.get_from_ts(msg.trackID)
            # Just joined the main line, entirely exiting the station zone
            if msg.trackID == station.ts_ids[Station.ON_RAMP_II]:
                vehicle.state = States.RUNNING
                vehicle.station = None

        # Check if just entered a merge's zone of control.
        if vehicle.state is States.RUNNING: # may have been LAUNCHING at beginning of function
            merge = self.merges.get_from_outlet_ts(msg.v_status.tail_locID)

            # Vehicle just left a merge's zone of control
            if merge:
                while True:
                    merge_slot = merge.reservations.pop(0)
                    if not merge_slot.relinquished:
                        assert vehicle.merge_slot is merge_slot
                        break
                vehicle.set_merge_slot(None)

            merge = self.merges.get_from_inlet_ts(msg.v_status.tail_locID)
            # Vehicle just entered a merge's zone of control
            if merge:
                vehicle.do_merge(merge, self.current_time)

    def on_SIM_NOTIFY_VEHICLE_STOPPED(self, msg, msgID, msg_time):
        vehicle = self.vehicles[msg.v_status.vID]
        assert isinstance(vehicle, Vehicle)
        vehicle.update_vehicle(msg.v_status, msg.time)

        if vehicle.state in (States.UNLOAD_ADVANCING, States.LOAD_ADVANCING, States.EXIT_ADVANCING):
            # Check that I'm stopped and where I expect to be
            assert abs(vehicle.vel) < TrajectorySolver.v_threshold, str(vehicle)
            assert abs(vehicle.accel) < TrajectorySolver.a_threshold, str(vehicle)
            assert vehicle.ts_id == vehicle.plat_ts, str(vehicle)
            assert abs(vehicle.berth_pos - vehicle.pos) < 0.1, (str(vehicle), vehicle.berth_pos)

            if vehicle.state is States.UNLOAD_ADVANCING:
                vehicle.state = States.UNLOAD_WAITING
            elif vehicle.state is States.LOAD_ADVANCING:
                vehicle.state = States.LOAD_WAITING
            elif vehicle.state is States.EXIT_ADVANCING:
                vehicle.state = States.EXIT_WAITING
            else:
                raise Exception("Unexpected state: %s" % vehicle.state)

        else:
            self.log.warn("Vehicle %s stopped for unknown reason.", str(vehicle))

        if vehicle.station:
            vehicle.station.heartbeat()

    def on_SIM_NOTIFY_VEHICLE_SPEEDING(self, msg, msgID, msg_time):
        self.log.warn("Speed limit violation: %s", str(msg))

    def on_SIM_COMPLETE_STORAGE_EXIT(self, msg, msgID, msg_time):
        station = self.stations[msg.cmd.sID]
        try:
            vehicle = self.vehicles[msg.v_status.vID]
        except KeyError:
            # Newly created vehicle
            model = self.v_models[msg.cmd.model_name]
            assert isinstance(model, Vehicle)
            vehicle = Vehicle(msg.v_status.vID,
                              model.model,
                              model.length,
                              model.capacity,
                              api.STORAGE_ID,
                              msg.v_status.nose_pos,
                              msg.v_status.vel,
                              msg.v_status.accel,
                              model.j_max,
                              model.j_min,
                              model.a_max,
                              model.a_min,
                              model.v_max)
            self.vehicles[vehicle.id] = vehicle

        vehicle.set_path([msg.v_status.nose_locID], False)
        vehicle.update_vehicle(msg.v_status, self.current_time)
        vehicle.station = station
        vehicle.plat_ts = vehicle.ts_id
        vehicle.berth_id = msg.cmd.berthID
        vehicle.platform_id = msg.cmd.platformID
        vehicle.state = States.LOAD_WAITING
        storage = station.storage_dict[vehicle.model]

        assert isinstance(station, Station)
        assert isinstance(vehicle, Vehicle)
        assert isinstance(storage, Storage)

        # TODO: Wrap this in a Station method?
        station.reservations[msg.cmd.platformID][msg.cmd.berthID] = vehicle
        storage.end_vehicle_exit()

        station.heartbeat()

    def on_SIM_COMPLETE_STORAGE_ENTER(self, msg, msgID, msg_time):
        station = self.stations[msg.cmd.sID]
        vehicle = self.vehicles[msg.cmd.vID]
        storage = station.storage_dict[vehicle.model]

        station.reservations[vehicle.platform_id][vehicle.berth_id] = None
        vehicle.set_path([api.STORAGE_ID], False)
        vehicle.station = None
        vehicle.plat_ts = None
        vehicle.berth_id = None
        vehicle.platform_id = None
        vehicle.state = States.STORAGE

        storage.end_vehicle_entry()
        station.heartbeat()

    def on_SIM_END(self, msg, msgID, msg_time):
        super(PrtController, self).on_SIM_END(msg, msgID, msg_time)
        write_stats_report(self.vehicles, self.stations, options.stats_file)


class VehicleManager(utility.Singleton):
    """Coordinates vehicles to satisfy passenger demand. Handles vehicle pathing."""

    # TODO: Is there any reasonable way to handle multiple vehicle types, without
    #       just hardcoding a behavior to a particular name?
    MODEL_NAME = None # prt_controller only uses the last model_name that it discovers.

    def __init__(self):
        """scenario_xml: the xml scenario file created by TrackBuilder, as a string."""

    @classmethod
    def initialize(cls, tracks, stations, vehicle_models, vehicles):
        v_manager = cls()

        v_manager.tracks = tracks
        v_manager.stations = stations
        v_manager.vehicle_models = vehicle_models
        v_manager.vehicles = vehicles

        try:
            # Need to choose a model name for when I request a vehicle from storage.
            # Eventually this controller may handle multiple vehicle models in a
            # reasonable way, but for now just choose a vehicle model arbitrarily.
            VehicleManager.MODEL_NAME = v_manager.vehicle_models.keys()[0]
        except IndexError:
            warnings.warn("No VehicleModels found.")

    def retrieve_vehicle_from_storage(self, station, model_name):
        """Gets a vehicle from storage as close as possible to station."""
        # OPTIMIZATION:
        #   Right now, this function is doing a lot of redundant work. Could cache:
        #    - The distances between stations.
        #    - Which stations are capable of bringing vehicles out of storage
        assert isinstance(station, Station)

        dists_stations = []
        for s in self.stations.itervalues():
            dist, path = self.tracks.get_path(s.merge, station.merge)
            dists_stations.append( (dist, s) )
        dists_stations.sort() # sort by distances, ascending

        for dist, station in dists_stations:
            try:
                station.call_from_storage(model_name)
                break
            except NoVehicleAvailableError:
                pass

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
        path_length, path = self.tracks.get_path(vehicle.ts_id, dest_station.ts_ids[Station.UNLOAD])
        return Trip(dest_station, path, (pax.id,) ) # just one pax for now

    def deadhead(self, vehicle, exclude=tuple()):
        """Request a trip for an empty vehicle. Go to a station that has waiting
         passengers and needs more vehicles. If no stations have positive demand
         then it sends the vehicle to the nearest station with capacity to store
         this vehicle's model, if there are any such stations available.

        Parameters:
          vehicle -- The empty vehicle requesting a trip.
          exclude -- A tuple of Station instances that the vehicle should not
                     pick as a destination.

        Side Effects:
          None

        Returns:
          A Trip instance.
        """
        assert isinstance(vehicle, Vehicle)
        load_ts_ids = self.stations.get_load_ts_ids()
        dists, paths = self.tracks.get_paths(vehicle.ts_id, load_ts_ids)
        max_dist = max(dists.itervalues())
        best_station = None
        best_station_dist = -inf
        best_value = -inf
        closest_station = None
        closest_dist = inf
        for station in self.stations.itervalues():
            if station in exclude:
                continue
            station_ts = station.ts_ids[Station.LOAD]
            try:
                dist = dists[station_ts]
            except KeyError: # Not all stations must be reachable
                continue

            demand = station.get_demand()

            # If the vehicle is currently in the station, give it a little inertia
            # to overcome before moving on to another station. Otherwise a vehicle
            # will leave a station as soon as demand drops to 0, resulting in
            # the station increasing it's demand again.
            if vehicle.station is station:
                demand += 0.09 # Note that one passenger at the furthest station gives a demand of 0.1

            # The distance weight is 1 when dist is 0, and decays down to 1/10
            # as the distance increases to max_dist.
            value = max_dist/(9*dist+max_dist) * demand

            if value > best_value:
                best_station = station
                best_station_dist = dist
                best_value = value
            if dist < closest_dist and value >= 0: # closest station that isn't rejecting vehicles
                closest_station = station
                closest_dist = dist

        if best_value > 0:
            return Trip(best_station,
                        paths[best_station.ts_ids[Station.LOAD]],
                        tuple())
        else:
            # Consider sending the vehicle to a station where it can be put into storage.
            vehicle_model = vehicle.model
            closest_storage_station = None
            closest_storage_dist = inf
            for station in self.stations.itervalues():
                if station in exclude:
                    continue

                if station.get_num_storage_vacancies(vehicle_model) > 0:
                    try:
                        dist = dists[station.ts_ids[Station.LOAD]]
                        if dist < closest_storage_dist:
                            closest_storage_dist = dist
                            closest_storage_station = station
                    except KeyError: # Not all stations must be reachable
                        continue

            if closest_storage_station is not None:
                # Found a station that can store the vehicle
                return Trip(closest_storage_station,
                            paths[closest_storage_station.ts_ids[Station.LOAD]],
                            tuple())
            else:
                # No storage available anywhere, send it to the closest station
                return Trip(closest_station,
                            paths[closest_station.ts_ids[Station.LOAD]],
                            tuple())

    def wave_off(self, vehicle):
        """Sends a vehicle around in a loop so as to come back and try entering the
        station later, or sends it to a different station if empty."""
        # vehicle has passengers, so must go to this particular station
        if vehicle.trip.passengers:
            prev_tses = self.tracks.predecessors(vehicle.ts_id) # there may be a merge just upstream of station split ts
            best_loop_length, best_loop_path = inf, None
            for prev_ts in prev_tses:
                loop_length, loop_path = self.tracks.get_path(vehicle.ts_id, prev_ts)
                if loop_length < best_loop_length:
                    best_prev_ts = prev_ts
                    best_loop_length = loop_length
                    best_loop_path = loop_path

            # concatenate that path with a path to the station's unload
            entry_length, entry_path = self.tracks.get_path(best_prev_ts, vehicle.trip.dest_station.ts_ids[Station.UNLOAD])

            path = best_loop_path + entry_path[1:] # concat lists. Discard prev_ts, so that it's not duplicated.

            return Trip(vehicle.trip.dest_station, path, vehicle.trip.passengers)

        # Vehicle is empty, so any station will do.
        else:
            return self.deadhead(vehicle, exclude=(vehicle.trip.dest_station,))

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

        # find the dist it takes for the launch vehicle to reach the main line speed
        line_speed_dist = vehicle.get_dist_to_line_speed()

        merge_dist, merge_path = self.tracks.get_path(vehicle.ts_id, station.merge)
        merge_dist -= vehicle.pos
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
        conflict_zone_start_dist = PrtController.LINE_SPEED * merge_delay - headway_dist - vehicle.length
        conflict_zone_end_dist = conflict_zone_start_dist + headway_dist + vehicle.length + headway_dist

        # walk back from the merge point finding the tracksegments and positions
        # that are in the conflict zone
        nodes, starts, ends = self.find_distant_segs_reverse(station.merge,
                           conflict_zone_start_dist, conflict_zone_end_dist, [], [], [])
        # TODO optimize this?
        times = []
        for v in self.vehicles.itervalues():
            if v.station is not None:
                continue

            try:
                idx = nodes.index(v.ts_id)
            except ValueError:
                continue

            # v may be in conflict zone. Estimate current position
            if abs(v.accel) > 0.0001:
                warnings.warn("Vehicle on main line NOT travelling at constant velocity! v: %d, ts: %d, vel: %.3f, accel: %.3f" %\
                          (v.id, v.ts_id, v.vel, v.accel))
            v_pos = v.pos + (now - v.last_update)*v.vel

            if v_pos > starts[idx] + TrajectorySolver.q_threshold \
               and v_pos < ends[idx] - TrajectorySolver.q_threshold:
                # find the distance, and thus time, until v clears the conflict zone
                seg = v.ts_id
                clearing_dist = ends[idx] - v_pos
                dist, path = self.tracks.get_path(seg, station.merge) # Use vehicle's path instead
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
        preds = self.tracks.predecessors(initial_node)
        for node in preds:
            # exclude station segments from the walk
            if self.stations.get_from_ts(node) is not None:
                continue

            edge_length = self.tracks.get_path_length( (node, initial_node) )
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



class Storage(object):
    """A simple data storing class that keeps track of how many vehicles and
    vacancies are available at one Station and for one vehicle model.
    """

    def __init__(self, model_name, initial_supply, max_capacity):
        self.model_name = model_name
        self.max_capacity = max_capacity

        self._num_vehicles = initial_supply
        self._num_pending_entry = 0
        self._num_pending_exit = 0

    def get_num_vehicles(self):
        n = self._num_vehicles - self._num_pending_exit
        return n

    def get_num_vacancies(self):
        if math.isinf(self.max_capacity):
            # avoid case where inf is may be subtracted from inf, resulting in nan
            return self.max_capacity
        else:
            return self.max_capacity - self._num_vehicles - self._num_pending_entry

    def start_vehicle_entry(self):
        self._num_pending_entry += 1
        assert 0 <= self._num_pending_entry <= self.max_capacity

    def end_vehicle_entry(self):
        self._num_pending_entry -= 1
        self._num_vehicles += 1
        assert 0 <= self._num_pending_entry
        assert 0 <= self._num_vehicles <= self.max_capacity

    def start_vehicle_exit(self):
        self._num_pending_exit += 1
        assert 0 <= self._num_pending_exit <= self.max_capacity

    def end_vehicle_exit(self):
        self._num_pending_exit -= 1
        self._num_vehicles -= 1
        assert 0 <= self._num_pending_exit
        assert 0 <= self._num_vehicles <= self.max_capacity

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

    # Singletons
    v_manager = VehicleManager()

    def __init__(self, s_id, ts_ids, split_ts_id, bypass_ts_id, merge_ts_id, onramp_length,
                 unload_positions, queue_positions, load_positions,
                 storage_entrances, storage_exits, storage_dict):
        """s_id: An integer station id.
        ts_ids: A list containing integer TrackSegment ids. See Station consts.
        split_ts_id: The TrackSegment id upsteam of both the bypass and the
                        offramp.
        bypass_ts_id: The TrackSegment id which bypasses the station. That is,
                      the main track rather than the station's track.
        merge_ts_id: The TrackSegment id downstream of both the bypass and the
                        onramp to the main line.
        onramp_length: The sum of the ACCEL, ON_RAMP_I, and ON_RAMP_II lengths.
        unload_positions, queue_positions, load_positions:
            Each of the above are lists of floats, where each float designates
            a position that the vehicle will target in order to park in the berth.
            i.e. to unload in berth 1, the vehicle will go to the position found in
            unload_positions[1].
        storage_entrances: A sequence of (platform_id, berth_id) pairs.
        storage_exits: A sequences of (platform_id, berth_id) pairs.
        storage_dict: A dict keyed by model name, whose values are Storage objects.
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

        self.storage_entrances = storage_entrances
        self.storage_exits = storage_exits
        self.storage_dict = storage_dict

        self.passengers = deque()
        self.v_count = 0

        self.NUM_UNLOAD_BERTHS = len(unload_positions)
        self.NUM_QUEUE_BERTHS = len(queue_positions)
        self.NUM_LOAD_BERTHS = len(load_positions)
        self.NUM_BERTHS = self.NUM_UNLOAD_BERTHS + self.NUM_QUEUE_BERTHS + self.NUM_LOAD_BERTHS

    def __cmp__(self, other):
        if not isinstance(other, Station):
            raise ValueError
        else:
            return cmp(self.id, other.id)

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
            berth_pos = self.berth_positions[platform_id][choosen_idx]
            plat_ts = self.ts_ids[platform_id+3]  # +3 maps platform_id to ts
            vehicle.berth_pos = berth_pos
            vehicle.berth_id = choosen_idx
            vehicle.platform_id = platform_id
            vehicle.plat_ts = plat_ts

            self.v_count += 1
            assert 0 <= self.v_count <= self.NUM_BERTHS

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
            assert 0 <= self.v_count <= self.NUM_BERTHS

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
        """Returns a value indicating vehicle demand. Higher values indicate
        more demand, and values may be negative."""
        if self.v_count == self.NUM_BERTHS:
            # Being full prevents vehicles from arriving, so we want to actively
            # reduce the number of vehicles, regardless of how many passengers
            # are waiting.
            return -float('inf')

        # Provide demand based on the discrepency between the number of
        # passengers waiting and the number of vehicles in the station
        passenger_demand = len(self.passengers) - self.v_count

        if passenger_demand <= 0:
            # Provide some demand until there are enough vehicles in the
            # station to fill the LOAD and QUEUE berths. When no vehicles are present,
            # advertise slightly less than 1 passenger's worth of demand. When
            # there are more vehicles than can fit in LOAD + QUEUE, demand
            # becomes negative.
            berth_demand = (self.NUM_QUEUE_BERTHS + self.NUM_LOAD_BERTHS - self.v_count) \
                            * 1/(self.NUM_QUEUE_BERTHS + self.NUM_LOAD_BERTHS + 1)
            return berth_demand

        else:
            return passenger_demand

    def get_num_storage_vehicles(self, model_name):
        try:
            storage = self.storage_dict[model_name]
            return storage.get_num_vehicles()
        except KeyError:
            return 0

    def get_num_storage_vacancies(self, model_name):
        try:
            storage = self.storage_dict[model_name]
            return storage.get_num_vacancies()
        except KeyError:
            return 0

    def call_from_storage(self, model_name):
        if not self.storage_exits:
            raise NoVehicleAvailableError # TODO: Use a more accurate error?

        storage = self.storage_dict[model_name]
        assert isinstance(storage, Storage)
        if storage.get_num_vehicles() == 0:
            raise NoVehicleAvailableError

        # Find an open berth that is marked as a storage entrance to move the
        # vehicle into. Start the search at the end closer to the exit.
        for platform_id, berth_id in reversed(self.storage_exits):
            if self.reservations[platform_id][berth_id] is None:
                self.reservations[platform_id][berth_id] = api.NONE_ID
                break
        else:
            # Not worth raising a different error type if the behavior is the same
            raise NoVehicleAvailableError

        storage.start_vehicle_exit()
        self.v_count += 1
        cmd = api.CtrlCmdStorageExit()
        cmd.sID = self.id
        cmd.platformID = platform_id
        cmd.berthID = berth_id
        cmd.position = self.berth_positions[platform_id][berth_id]
        cmd.model_name = model_name
        self.controller.send(api.CTRL_CMD_STORAGE_EXIT, cmd)

        # storage.end_vehicle_exit() is called upon receipt of a
        # SimCompleteStorageExit message.

    def put_into_storage(self, vehicle):
        """Moves a vehicle into Storage.

        PreConditions:
          Vehicle is parked in one of the self.storage_entrances berths
        """
        assert isinstance(vehicle, Vehicle)
        assert (vehicle.platform_id, vehicle.berth_id) in self.storage_entrances
        storage = self.storage_dict[vehicle.model]
        if storage.get_num_vacancies() == 0:
            raise NoVacancyAvailableError

        assert isinstance(storage, Storage)
        storage.start_vehicle_entry()
        self.v_count -=1
        vehicle.state = States.STORAGE_ENTERING
        cmd = api.CtrlCmdStorageEnter()
        cmd.vID = vehicle.id
        cmd.sID = self.id
        cmd.platformID = vehicle.platform_id
        cmd.berthID = vehicle.berth_id
        self.controller.send(api.CTRL_CMD_STORAGE_ENTER, cmd)

        # storage.end_vehicle_enter() is called upon receipt of
        # the SimCompleteStorageEnter message.

    def heartbeat(self):
        """Triggers synchronized advancement of vehicles in the station."""
        stations = Stations() # Get the Singleton
        self.controller.log.debug("t:%5.3f Station %d heartbeat.", self.controller.current_time, self.id)
        max_demand = max(s.get_demand() for s in stations.itervalues())
        for plat in (self.LOAD_PLATFORM, self.QUEUE_PLATFORM, self.UNLOAD_PLATFORM):
            for v in reversed(self.reservations[plat]):
                if isinstance(v, Vehicle) \
                   and v.state not in (States.STORAGE_ENTERING, States.STORAGE_EXITING):

                    assert self.reservations[v.platform_id][v.berth_id] is v

                    # If vehicle is capable of moving forward, do so
                    self.controller.trigger_advance(v, self) # changes v.state to *_ADVANCING if able to move

                    # For the vehicle's that are stuck in their current position
                    # try to do something useful with the time.
                    if v.state is States.UNLOAD_WAITING:
                        v.state = States.DISEMBARKING
                        v.disembark(v.trip.passengers)

                    elif v.state is States.LOAD_WAITING:
                        # If there is no postive demand anywhere, start putting vehicles into storage
                        if max_demand <= 0 and (v.platform_id, v.berth_id) in self.storage_entrances:
                            try:
                                self.put_into_storage(v)
                                max_demand = self.get_demand() # Only put vehicles into storage until this station's demand goes non-negative.
                            except NoVacancyAvailableError:
                                pass

                        elif v.platform_id == self.LOAD_PLATFORM:
                            try:
                                v.trip = self.v_manager.request_trip(v)
                                v.set_path(v.trip.path)
                                v.state = States.EMBARKING
                                v.embark(v.trip.passengers)
                            except NoPaxAvailableError:
                                if self.is_launch_berth(v.berth_id, v.platform_id):
                                    trip = self.v_manager.deadhead(v)
                                    if trip.dest_station is not self:
                                        v.trip = trip
                                        v.set_path(v.trip.path)
                                        v.state = States.LAUNCH_WAITING
                                        self.controller.log.info("%5.3f Vehicle %d deadheading from station %d. Going to station %d.",
                                                      self.controller.current_time, v.id, self.id, v.trip.dest_station.id)

                    elif v.state is States.EXIT_WAITING:
                        if self.is_launch_berth(v.berth_id, v.platform_id):
                            v.state = States.LAUNCH_WAITING

                    self.controller.request_v_status(v.id)


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

    # To prevent a vehicle's trajectory from being undefined, splines are extended
    # so as to reach to the end of the simulation (and a little further). This
    # constant controls the 'little further'. Needs to be long enough that no algorithm
    # tries to predict a vehicle trajectory beyond this length. The incintive to keep
    # it somewhat small is that rounding errors are excacerbated if it is very large.
    SPLINE_TIME_EXTENSION = 3600     # in seconds

    controller = None # interfaces with the sim

    # Singletons.
    manager = VehicleManager() # high level planner
    tracks = Tracks()

    def __init__(self, v_id, model_name, length, capacity, ts_id, pos, vel, accel, j_max, j_min, a_max, a_min, v_max):
        # Can't handle non-zero initial accelerations at this time
        assert accel == 0

        self.id = v_id
        self.model = model_name
        self.length = length
        self.capacity = capacity
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.j_max = j_max
        self.j_min = j_min
        self.a_max = a_max
        self.a_min = a_min
        self.v_max = v_max

        self.pax = []

        end_time = self.controller.sim_end_time + 1

        # The spline's coordinate frame is always the first element of the path.
        self._path = [ts_id]
        self._current_path_index = 0

        self._spline = CubicSpline([pos, pos+vel*end_time], [vel, vel], [0, 0], [0], [0.0, end_time])

        self._merge_slot = None

        self.traj_solver = TrajectorySolver(self.v_max, self.a_max, self.j_max,
                                            0, self.a_min, self.j_min)
        self.last_update = 0.0
        self.trip = None

        self._state_change_time = self.controller.current_time
        self._state = States.NONE
        self.time_spent_in_states = [0.0]*len(States.strings)

        # Station that the vehicle is currently in.
        self.station = None

        # Info for a reserved berth, not the current berth!!
        self.platform_id = None
        self.berth_pos = None
        self.berth_id = None
        self.plat_ts = None

        # A private flag for indicating whether a vehicle's launch from a
        # station has been commanded yet or not.
        self._launch_begun = True

        accel_spline = self.traj_solver.target_velocity(Knot(0,0,0,0),
                             Knot(None,self.controller.LINE_SPEED,0,None))
        self._dist_to_line_speed = accel_spline.q[-1]
        self._time_to_line_speed = accel_spline.t[-1]

        decel_spline = self.traj_solver.target_velocity(Knot(0,self.controller.LINE_SPEED,0,0),
                             Knot(None,0,0,None))
        self._dist_to_stop = decel_spline.q[-1]
        self._time_to_stop = decel_spline.t[-1]

        # Runtime stats
        self.num_wave_offs = 0

    def __str__(self):
        return "id:%d, ts_id:%d, pos:%.3f, vel:%.3f, accel:%.3f, last_update:%.3f, model:%s, length:%.3f" \
               % (self.id, self.ts_id, self.pos, self.vel, self.accel, self.last_update, self.model, self.length)

    def __cmp__(self, other):
        if not isinstance(other, Vehicle):
            raise ValueError
        else:
            return cmp(self.id, other.id)

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
        assert spline.t[0] < self.controller.current_time + 2*TrajectorySolver.t_threshold# Spline is valid at current time and beyond.
        self._path = self._path[self._current_path_index:]
        self._current_path_index = 0

        sim_end_time = self.controller.sim_end_time
        if spline.t[-1] < sim_end_time:
            assert abs(spline.a[-1]) < 2*TrajectorySolver.a_threshold
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
##        assert path[0] == self._path[self._current_path_index]
        self._path = self._path[:self._current_path_index] + path
        if send:
            self.send_path()
    path = property(get_path, doc="""A vehicle's planned path, as a list of tracksegment id's. The first id must be the vehicle's current location.""")

    def get_state(self):
        return self._state
    def set_state(self, state):
        """Changes the state, and keeps a tally of how long the vehicle spends
        in each state.
        """
        now = self.controller.current_time
        self.time_spent_in_states[self._state] += now - self._state_change_time
        self._state_change_time = now
        self._state = state
    state = property(get_state, set_state, None, "Current state in the state machine.")

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

        if v_status.lvID != api.NONE_ID \
           and v_status.lv_distance <= self.vel*self.controller.HEADWAY - 0.001:
            self.controller.log.warn("Vehicle %i is following too close to vehicle %i. lv_dist: %.3f desired separation: %.3f" \
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

        offset = self.tracks.coordinate_shift(idx, path)
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

    def run(self, speed_limit=inf, dist=None, final_speed=0):
        """Commands the vehicle's trajectory. Obeys the speed_limit
        constraint, if supplied. Estimates the vehicle's current pose from the
        current spline.

        'speed_limit' gives an upper limit on vehicle speed. Vehicle may use
        other, lower limits if applicable.

        'dist' is given as the number of meters to the target position. If left
        as none, then the vehicle accelerates to the speed limit and continues
        at that velocity until the end of the simulation (or until trajectory
        is changed).

        'final_speed' is to be used in conjunction with 'dist'. Measured in meters/sec.

        If a distance is specified, returns the scheduled arrival time.
        """
        initial_knot = self.estimate_pose(self.controller.current_time)
        if dist is not None:
            final_knot = Knot(initial_knot.pos + dist, final_speed, 0, None)

        speed_zones = self.tracks.get_speed_zones(self.path[self._current_path_index:])
        # Include other speed restrictions
        # TODO: Is it worthwhile to merge speed zones if they end up with the same limit?
        for spd, zone in speed_zones:
            spd = min(self.v_max, speed_limit, spd)

        speed_knots = [initial_knot]
        spline = CubicSpline([initial_knot.pos],
                             [initial_knot.vel],
                             [initial_knot.accel],
                             [],
                             [initial_knot.time])
        path_length = 0


        # Create a spline that covers the entirety of the vehicle's path or
        # to the end of the whichever speed zone 'dist' lands on, whichever
        # is shorter.
        for (curr_speed, curr_path_frag), (next_speed, next_path_frag) in pairwise(speed_zones):
            path_length += self.tracks.get_path_length(curr_path_frag + [next_path_frag[0]])
            knot = Knot(path_length, min(next_speed, curr_speed), 0, None)
            try:
                spline = spline.concat(self.traj_solver.target_position(
                    speed_knots[-1], knot, max_speed=curr_speed))
                knot.time = spline.t[-1]
                speed_knots.append(knot)
            except FatalTrajectoryError:
                # In some cases, the vehicle just won't be able to reach the
                # desired velocity exactly at the speed limit transition point.
                pass

            if dist is not None and path_length >= final_knot.pos:
                break

        # If 'dist' is not specified, extend the spline to cover the whole path.
        if dist is None:
            path_length += self.tracks.get_path_length(next_path_frag)
            knot = Knot(path_length, next_speed, 0, None)
            spline = spline.concat(self.traj_solver.target_position(
                speed_knots[-1], knot, max_speed=curr_speed))

        # If 'dist' is specified, end the spline with a segment that
        # takes the vehicle to end knot. To do this, we try to
        # connect the last speed knot with the end knot. If the trajectory
        # fails, we iteratively try earlier and earlier speed knots as starting
        # points.
        else: # dist is not None:
            end_spline = None
            for i in range(len(speed_knots)-1, -1, -1): # iterate in reverse
                # Use the lowest speed limit out of the speed zones we're traversing for the end spline
                spd_limit = min(speed_zones[i:])[0] # tuple comparison primarily uses the first element
                try:
                    end_spline = self.traj_solver.target_position(speed_knots[i],
                                                                  final_knot,
                                                                  max_speed=spd_limit)
                    final_knot.time = end_spline.t[-1]
                except FatalTrajectoryError:
                    continue
                else:
                    break

            assert end_spline is not None
            left_spline = spline.copy_left(speed_knots[i].time)
            spline = left_spline.concat(end_spline)

        self.set_spline(spline)

        # If a distance was specified, return the arrival time.
        if dist is not None:
            return final_knot.time
        else:
            return

    def enter_station(self, station):
        """Assumes that vehicle has a reserved berth already.
        Slows the vehicle to the station's speed limit then brings the vehicle
        to a halt at its reserved berth/platform. Will change the vehicle's
        spline and path to accomplish this.

        Returns: The time at which the vehicle will come to a halt within its
                 reserved berth."""
        # If vehicle has a merge_slot, relinquish it.
        if self.merge_slot:
            self.merge_slot.relinquished = True
            self.set_merge_slot(None)

        self.station = station
        current_knot = self.estimate_pose(self.controller.current_time)

        path_length, path = self.tracks.get_path(self.ts_id, self.plat_ts)
        self.set_path(path)
        stop_time = self.run(dist=path_length - current_knot.pos + self.berth_pos, final_speed=0)
        return stop_time

##        # Slow to station speed limit at the start of the UNLOAD segment
##        unload_dist, unload_path = self.tracks.get_path(self.ts_id, station.ts_ids[Station.UNLOAD])
##        unload_knot = Knot(unload_dist, station.SPEED_LIMIT, 0, None)
##        if current_knot.vel > TrajectorySolver.v_threshold: # Non-zero velocity
##            if current_knot.accel > 0:
##                accel_bleed = self.traj_solver.target_acceleration(current_knot, Knot(None, None, 0, None))
##                peak_vel_knot = Knot(accel_bleed.q[-1], accel_bleed.v[-1], accel_bleed.a[-1], accel_bleed.t[-1])
##                to_unload_spline = accel_bleed.concat(self.traj_solver.target_position(peak_vel_knot, unload_knot, max_speed=peak_vel_knot.vel))
##            else:
##                to_unload_spline = self.traj_solver.target_position(current_knot, unload_knot, max_speed=current_knot.vel)
##            unload_knot.time = to_unload_spline.t[-1]
##
##            # Continue on to stop at the desired berth pos (works even if platform is other than UNLOAD).
##            berth_dist, berth_path = self.tracks.get_path(station.ts_ids[Station.UNLOAD], self.plat_ts)
##            berth_knot = Knot(unload_knot.pos + berth_dist + self.berth_pos, 0, 0, None)
##            to_berth_spline = self.traj_solver.target_position(unload_knot, berth_knot, max_speed=station.SPEED_LIMIT)
##
##            spline = to_unload_spline.concat(to_berth_spline)
##            path = unload_path + berth_path[1:] # don't duplicate the UNLOAD ts_id
##        else:
##            # When used during sim startup the vehicle is stationary and
##            # doesn't need a separate spline for the decel to station speed limit.
##            berth_dist, berth_path = self.tracks.get_path(self.ts_id, self.plat_ts)
##            berth_knot = Knot(berth_dist + self.berth_pos, 0, 0, None)
##            spline = self.traj_solver.target_position(current_knot, berth_knot, max_speed=self.controller.LINE_SPEED)
##            path = berth_path
##
##        stop_time = spline.t[-1]
##        self.set_spline(spline)
##        return stop_time

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
            dist = (self.berth_pos) - self.pos # stop a little short of the end

        else: # advancing to another next platform
            path_length, path = self.tracks.get_path(self.ts_id, self.plat_ts)
            dist = (path_length - self.pos) + (self.berth_pos)

            # Set the path, but don't stomp on the existing path if it's the same but longer
            p = self._path[self._current_path_index:]
            if len(path) > len(p) or path != p[:len(path)]:
                self.set_path(path)

        if speed_limit == None:
            speed_limit = self.station.SPEED_LIMIT

        return self.run(speed_limit=speed_limit, dist=dist, final_speed=0)

    def do_merge(self, merge, now):
        """Aquire a MergeSlot from the Merge and use the MergeSlot's spline.
        If now is None, then the controller's current time is used.

        Raises: Does not catch a FatalTrajectory exception that may be emitted
                from Merge.create_merge_slot.
        Returns: A MergeSlot in the normal case. Returns None if a vehicle is bound
                for a station within the Merge's zone of control and is guaranteed
                to enter the station (vehicle has already aquired a berth reservation).
        """
        assert isinstance(merge, Merge)
        if now is None:
            now = self.controller.current_time

        try:
            merge_slot = merge.create_merge_slot(self,
                                                 self.ts_id,
                                                 self.estimate_pose(now),
                                                 self.length,
                                                 self.traj_solver,
                                                 now)
            self.set_merge_slot(merge_slot)
            self.set_spline(self.merge_slot.spline)
        except NotAtDecisionPoint:
            slot_assignment_dist = (merge.get_slot_assignment_position(self.ts_id) - self.estimate_pose(now).pos) + self.length # measured from rear of vehicle
            slot_assignment_time = self.spline.get_time_from_dist(slot_assignment_dist, now)
            self.controller.set_fnc_notification(self.do_merge, (merge, None), slot_assignment_time) # call do_merge again later

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

class MergeError(Exception):
    """Base class for exceptions emenating from the Merge class"""

class NotAtDecisionPoint(MergeError):
    """Vehicle hasn't reached the merge's 'decision point' -- the position
    at which a vehicle is assigned a merge slot."""

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

    def __init__(self, merge_id, merge_ts_id, graph, cutoff=0):
        """
        merge_id: The integer id for the Merge object.
        merge_ts_id: The track segment id for which both legs of the merge are upstream.
        graph: A networkx.DiGraph describing the track network.
        cutoff: Distance prior to the merge at which the vehicles should be
                synched up and ready to merge.

        Requires that the 'Stations' singleton has been initialized!
        """
        # TODO: Emergency stop distance prior to merge
        assert isinstance(merge_id, int), type(merge_id)
        assert isinstance(merge_ts_id, int), type(merge_ts_id)
        assert isinstance(graph, networkx.classes.DiGraph), type(graph)

        # Singletons
        stations = Stations()
        self.tracks = Tracks()

        self.id = merge_id

        # FIFO queue containing MergeSlot elements. Note that 'front'
        # includes the vehicle's headway. Append to right end, pop from left end.
        # Not using a deque because deque lacks insert, index, and splice.
        self.reservations = []

        # converts track segments to the merge coordinate frame. All positions
        # upstream of the merge are negative. The merge point is at 0.
        self.offsets = {}

        self.outlet = merge_ts_id

        # maps station id's to distances from the station merge to the main merge
        self.zoc_stations = []

        # A list of ts_ids for each zone. Each list is ordered from the zone's
        # entry point to the merge point (i.e. in order from the inlet to the
        # outlet).
        self.zone_ids = [[], []]

        # Distance prior to the merge at which the vehicles should be synched
        # up and ready to merge.
        self.cutoff = cutoff

        # Walk upstream for both zones
        self.zone_lengths = [-1, -1]
        self.inlets = [None, None] # segment ids which are entry points for the Merge's zone of control
        zones = graph.predecessors(merge_ts_id)
        assert len(zones) == 2
        for zone_num, node in enumerate(zones):
            offset = 0
            up_node = node     # upstream
            down_node = merge_ts_id # downstream
            while True:
                length = graph[up_node][down_node]['length']
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
                    if stations.get_from_ts(predecessors[0]):
                        down_node = up_node
                        up_node = predecessors[1]
                        station = stations.get_from_ts(predecessors[0])
                        self.zoc_stations.append(station)
                        is_station = True
                    elif stations.get_from_ts(predecessors[1]):
                        down_node = up_node
                        up_node = predecessors[0]
                        station = stations.get_from_ts(predecessors[1])
                        self.zoc_stations.append(station)
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
                    if not stations.get_from_ts(successors[0]) and not stations.get_from_ts(successors[1]):
                        self.zone_lengths[zone_num] = offset
                        self.inlets[zone_num] = down_node
                        break # TODO: Notify PRT controller that I'm associated with switch. May need to implement Switches sooner, rather than later

        # The decision point is the distance which the Merge's zone of control extends,
        # expressed as a negative number. The decision point is normally found
        # as the length of the shortest leg. If the point would land between
        # a station's mergepoint and an "onramp's length" upstream of the
        # station's merge, then the decision point is moved downstream to coincide
        # with the station's merge point, and the station is not considered to
        # be in the merge's zone of control.
        self._decision_point = max(self.zone_lengths) # max because numbers are negative
        i = 0
        while i < len(self.zoc_stations):
            s = self.zoc_stations[i]
            station_merge_offset = self.offsets[s.merge]
            if station_merge_offset - s.onramp_length < self._decision_point < station_merge_offset:
                self._decision_point = station_merge_offset
                # restart the loop, to ensure that the decision_point didn't
                # get moved into a no-mans-land on the other leg of the merge.
                i = 0
                continue
            i += 1

        # Only keep stations that are within the
        # zone of control (as determined by the decision_point).
        self.zoc_stations[:] = [s for s in self.zoc_stations if self._decision_point < self.offsets[s.merge]]

        # Only keep track segs that are within the zone of control.
        for zone_num in range(len(self.zone_ids)):
            path = []
            for ts_id in self.zone_ids[zone_num]:
                if self.offsets[ts_id] > self._decision_point: # ts_id is downstream of decision point
                    path.append(ts_id)
                else:
                    path.append(ts_id)
                    break

            self.inlets[zone_num] = path[-1]
            path.reverse() # ts_ids are now in the correct order to be a path through the merge.
            self.zone_ids[zone_num] = path

    def get_slot_assignment_position(self, ts_id):
        """Returns the position at which a vehicle is assigned a MergeSlot,
        in the coordinate frame of the track segment specified by ts_id."""
        try:
            offset = self.offsets[ts_id]
            return self._decision_point - offset
        except KeyError:
            path_length, path = self.tracks.get_path(ts_id, self.outlet)
            return self._decision_point + path_length

    def create_merge_slot(self, vehicle, start_ts_id, initial_knot, vehicle_length, traj_solver, now):
        """Creates a non-conflicting MergeSlot for a vehicle that is entering
        the Merge's zone of control. It is up to the vehicle to alter its
        trajectory so as to hit the MergeSlot, but the slot is guaranteed to be
        achievable by using the included spline.

        Side Effects:
          The newly created slot is added to the Merge's reservation queue.

        Returns:
          MergeSlot
        """
        # Work in the coordinate frame of the outlet. All segments within
        # the merge's zone of control have negative positions.
        offset = self.offsets[start_ts_id]

        v_pos = initial_knot.pos + offset # negative number. 0 is merge point
        dist = -v_pos
        v_rear_pos = v_pos - vehicle.length

        # Require that vehicles start the simulation far enough back from the
        # merge that they can get up to full speed.
        # TODO: Move this check to be in a function dedicated to validating the scenario as soon as it is received.
##        if now == 0.0 and vehicle.get_dist_to_line_speed() > dist:
##            raise FatalTrajectoryError(initial_knot, final_knot)


        # If the vehicle isn't to the decision point yet, then delay managing it.
        # Don't require that the vehicle not be past the decision point, so that
        # the code can be reused during simulation startup.
        if v_rear_pos < self._decision_point - 1: # If the vehicle is within a meter of the decision point, good 'nuff.
            raise NotAtDecisionPoint()

        # decide which zone the vehicle is in.
        if start_ts_id in self.zone_ids[0]:
            zone = 0
        elif start_ts_id in self.zone_ids[1]:
            zone = 1
        else:
            raise Exception("start_ts_id isn't in Merge's zone of control.")

##        # If a vehicle is intending to stop at a station prior to the merge point
##        # and it has a reserved berth (ensuring that it can enter the station),
##        # then it doesn't need a MergeSlot.
##        if vehicle.has_berth_reservation() and vehicle.trip.dest_station in self.zoc_stations:
##            return None

        # Create a MergeSlot for the vehicle to use.
        lead_slot = None
        try: # Usual case. There is a leading vehicle.
            lead_slot = self.reservations[-1]
            assert isinstance(lead_slot, MergeSlot)

            # First Pass Implementation. Only slip vehicle back to avoid collision, don't push lead vehicle forward
            final_knot = Knot(initial_knot.pos + dist, self.LINE_SPEED, 0, None)
            spline = traj_solver.target_position(initial_knot, final_knot, max_speed=self.LINE_SPEED)

            # If the vehicle, travelling at line speed, would arrive at the merge point too early, use target_time
            if spline.t[-1] < lead_slot.end_time + self.HEADWAY:
                final_knot.time = lead_slot.end_time + self.HEADWAY
                spline = traj_solver.target_time(initial_knot, final_knot, max_speed=self.LINE_SPEED)

            assert spline.t[-1] >= lead_slot.end_time + self.HEADWAY - 2*TrajectorySolver.t_threshold
            start_time = spline.t[-1] - self.HEADWAY
            end_time = start_time + vehicle_length/self.LINE_SPEED + self.HEADWAY

        except IndexError: # There is no leading vehicle. Travel at line speed.
            final_knot = Knot(initial_knot.pos + dist, self.LINE_SPEED, 0, None)
            spline = traj_solver.target_position(initial_knot, final_knot, max_speed=self.LINE_SPEED)

            start_time = spline.t[-1] - self.HEADWAY
            # Vehicle may not be up to line speed when passing through merge point
            end_time = spline.t[-1] + vehicle_length/spline.evaluate(start_time).vel

            if lead_slot is not None:
                # Start time should occur after lead_slot ends, with some tolerance for rounding error.
                assert start_time >= lead_slot.end_time - 2*TrajectorySolver.t_threshold

        merge_slot = MergeSlot(start_time, end_time, zone, vehicle, spline, start_ts_id, self)
        self.reservations.append(merge_slot)

        if __debug__ and len(self.reservations) >= 2:
            # no more than a little rounding error's worth of overlap
            assert merge_slot.start_time >= self.reservations[-2].end_time - 2*TrajectorySolver.t_threshold

        return merge_slot

    def create_station_merge_slot(self, station, vehicle, now):
        """Creates a non-conflicting MergeSlot for a vehicle that is launching
        from a station located within the Merge's zone of control. It is up to
        the vehicle to alter its trajectory so as to hit the MergeSlot, but the
        slot is guaranteed to be achievable by using the slot's spline.

        Side Effects:
          The slot is added to the Merge's reservation queue.

        Returns a 2-tuple: (merge_slot, launch_time)
        """
        initial = vehicle.estimate_pose(now)

        # Find which zone the station is in.
        if station.merge in self.zone_ids[0]:
            station_zone = 0
        else:
            assert station.merge in self.zone_ids[1]
            station_zone = 1

        # Try constructing a launch spline and slot which disregards the world
        # state -- launch immediately and travel to main-merge at line speed.
        # Fastest possible arrival at the merge point.
        onramp_dist, onramp_path = self.tracks.get_path(vehicle.ts_id, station.merge)
        station_merge_knot = Knot(onramp_dist, self.LINE_SPEED, 0, None)
        onramp_spline = vehicle.traj_solver.target_position(initial, station_merge_knot, max_speed=self.LINE_SPEED)
        station_merge_knot.time = onramp_spline.t[-1]

        merge_dist, merge_path = self.tracks.get_path(station.merge, self.outlet)
        merge_knot = Knot(merge_dist + onramp_dist, self.LINE_SPEED, 0, None)
        merge_spline = vehicle.traj_solver.target_position(station_merge_knot, merge_knot, max_speed=self.LINE_SPEED)
        merge_knot.time = merge_spline.t[-1]

        no_wait_spline = onramp_spline.concat(merge_spline)

        min_slot_size = self.HEADWAY + vehicle.length/self.LINE_SPEED # in seconds
        min_launch_duration = onramp_spline.t[-1] - onramp_spline.t[0]
        min_to_merge_duration = merge_spline.t[-1] - merge_spline.t[0]
        front_slot, rear_slot = self._find_usable_gap(now, station, min_slot_size, min_launch_duration,
                         min_to_merge_duration,  self.LINE_SPEED,
                         self.HEADWAY, vehicle.length)

        wait_dur = (front_slot.end_time + self.HEADWAY) - (no_wait_spline.t[-1])

        new_slot = None
        launch_time = None
        skip_insert = False
        if rear_slot.spline is not None: # Not a Dummy slot
            wait_dur = max(0, wait_dur)
            wait_spline = CubicSpline([initial.pos, initial.pos],
                                      [initial.vel, initial.vel],
                                      [initial.accel, initial.accel],
                                      [0], [initial.time, initial.time + wait_dur])
            spline = wait_spline.concat(no_wait_spline.time_shift(wait_dur))

            new_slot = MergeSlot(spline.t[-1] - self.HEADWAY,
                                spline.t[-1] + vehicle.length/self.LINE_SPEED,
                                station_zone,
                                vehicle,
                                spline,
                                vehicle.ts_id,
                                self)
            launch_time = now + wait_dur

        else: # rear_slot.spline is None; A dummy slot -- vehicle will be the last in line

            if wait_dur <= TrajectorySolver.t_threshold:
                # When the wait duration is <= 0, then the front vehicle is travelling
                # fast enough (or has enough of a head start) that the launch vehicle
                # can treat it as though it doesn't exist.
                new_slot = MergeSlot(no_wait_spline.t[-1] - self.HEADWAY,
                                    no_wait_spline.t[-1] + vehicle.length/self.LINE_SPEED,
                                    station_zone,
                                    vehicle,
                                    no_wait_spline,
                                    vehicle.ts_id,
                                    self)
                launch_time = now

            else:
                # If we have to wait, consider the following cases:
                # 1. The front vehicle is still approaching the station-merge point.
                #    It may be approaching fast or slow, but at least some part
                #    of the launch delay is strictly necessary so that the launch
                #    vehicle does not interfere with the front vehicle.
                #
                #    1a. The front vehicle is travelling full speed. The wait_dur
                #        is just the duration until we can use the no_wait_spline
                #        and also travel full speed. Simple.
                if abs(front_slot.spline.evaluate(now).vel - self.LINE_SPEED) < TrajectorySolver.v_threshold:
                    assert wait_dur >= 0
                    wait_spline = CubicSpline([initial.pos, initial.pos],
                                              [initial.vel, initial.vel],
                                              [initial.accel, initial.accel],
                                              [0], [initial.time, initial.time + wait_dur])
                    spline = wait_spline.concat(no_wait_spline.time_shift(wait_dur))

                    new_slot = MergeSlot(spline.t[-1] - self.HEADWAY,
                                        spline.t[-1] + vehicle.length/self.LINE_SPEED,
                                        station_zone,
                                        vehicle,
                                        spline,
                                        vehicle.ts_id,
                                        self)
                    launch_time = now + wait_dur


                #    1b. The front vehicle is travelling at less than full speed.
                #        This faces the same issues, and may use the same solution
                #        as outlined below.
                #
                # 2. The front vehicle is past the station_merge point. The wait
                #    time is necessary only because we choose to travel at full
                #    speed and the front vehicle is travelling slower. Like a driver
                #    with a Porche on a windy road, the launch vehicle waits by the
                #    side of the road to allow a gap to develop, then races forward
                #    at top speed, just catching up with the other vehicles at the
                #    main merge.
                #
                #    The risk of this behaviour is that another vehicle may enter
                #    the Merge's zone of control while the launch vehicle is waiting.
                #    The new vehicle may use a moderate pace, so as to come just
                #    behind the launch vehicle at the main merge. The new vehicle may
                #    pass the waiting launch vehicle, only to be overtaken by it
                #    once the launch vehicle starts. Though they have similar
                #    average speeds, one is a tortoise and the other a hare.
                #
                #    A solution to this problem is to create a spline as though
                #    the launch vehicle were entering the Merge's zone of control
                #    at the normal place -- the decision point, located at the end
                #    opposite of the main-merge point. Rather than having the launch
                #    vehicle wait and then go at full speed, the launch vehicle waits
                #    just long enough for this 'phantom' spline to get to the
                #    station merge point. The launch vehicle times its entry so as
                #    to merge with the 'phantom' spline and then uses it for the
                #    remainder of its trip to the merge.
                else:
                    inlet_tsid = self.inlets[station_zone]
                    new_slot = self.create_merge_slot(None,
                                                      inlet_tsid,
                                                      Knot(self.get_slot_assignment_position(inlet_tsid)+vehicle.length,
                                                           self.LINE_SPEED, 0, now),
                                                      vehicle.length,
                                                      vehicle.traj_solver,
                                                      now)
                    spline, ts_id = self._synch_with_slot(station, vehicle, new_slot, now)
                    new_slot.vehicle = vehicle
                    new_slot.spline = spline
                    if spline.j[0] == 0: # there's a wait segment
                        launch_time = spline.t[1]
                    else: # no wait segment
                        launch_time = spline.t[0]
                    skip_insert = True # using the slot that was inserted by create_merge_slot

        assert new_slot is not None
        assert launch_time is not None

        # Insert the new slot into the resevation queue
        if not skip_insert:
            try:
                assert new_slot.end_time <= rear_slot.start_time + 2*TrajectorySolver.t_threshold
                insert_idx = self.reservations.index(rear_slot)
                self.reservations.insert(insert_idx, new_slot)
            except ValueError:
                # Rear slot wasn't in self.reservations; it was a dummy slot.
                if __debug__ and self.reservations:
                    assert new_slot.start_time > self.reservations[-1].end_time - 2*TrajectorySolver.t_threshold
                self.reservations.append(new_slot)

        return new_slot, launch_time

    def _synch_with_slot(self, station, vehicle, slot, now):
        """Return a new spline that will launch vehicle from station so that
        it synchs up with slot.spline at the beginning of station.merge.

        Parameters:
          station -- The station being launched from.
          vehicle -- The launch vehicle. Assumed to be stationary and in a position
            that is able to launch from the station without interference.
          slot -- The slot that's being merged with. The slot's zone should match
            the station's zone.
          now -- The current time, in seconds.

        Throws:
          A MergeError if spline's position at (now + launch_duration) is
          past the start of the station.merge track segment, where
          'launch_duration' is the number of seconds required for vehicle to
          reach the start of station.merge. Note that launch_duration is longer
          for splines with velocities that are lower than the station was
          designed to handle.

        Side Effects:
          None

        Returns:
          A pair: (spline, spline_coordinate_frame)
          The spline starts at time 'now', and at the vehicle's current position.
          The spline duplicates the slot's spline from the station merge
          point onwards.

        """
        assert isinstance(station, Station)
        assert isinstance(vehicle, Vehicle)
        assert isinstance(slot, MergeSlot)
        assert now >= 0
        assert station.merge in self.zone_ids[slot.zone], (station, slot.zone, self.zone_ids)
        # Within this function, "merge" refers to the station merge, not the main merge.

        # Find where the spline hits station.merge
        merge_path_length, merge_path = self.tracks.get_path(slot.spline_frame, station.merge)
        merge_time = slot.spline.get_time_from_dist(merge_path_length-slot.spline.q[0], slot.spline.t[0])
        merge_knot = slot.spline.evaluate(merge_time)

        # Create a spline that launches the vehicle and matches velocities.
        # The time doesn't match up yet.
        initial = vehicle.estimate_pose(now)
        launch_path_length, launch_path = self.tracks.get_path(vehicle.ts_id, station.merge)
        launch_knot = Knot(launch_path_length, merge_knot.vel, merge_knot.accel, merge_time)
        launch_spline = vehicle.traj_solver.target_position(
            initial, launch_knot, max_speed=merge_knot.vel) # Uses simple accel profile
        launch_duration = launch_spline.t[-1] - launch_spline.t[0]
        launch_time = merge_time - launch_duration

        # Fail if launch time has already passed
        if launch_time < now:
            # TODO: Could try using a more aggresive launch spline by allowing
            # the max speed to climb higher than the merge speed. For now, just fail.
            raise MergeError("Launch time to synch with slot.spline is in the past.")

        wait_duration = launch_time - now
        assert initial.vel == 0 and initial.accel == 0, initial # relys on the sim having zeroed out rounding error for a stopped vehicle. Otherwise use thresholds.
        wait_spline = CubicSpline([initial.pos, initial.pos],
                                  [0,0], [0,0], [0], [now, launch_time])

        launch_spline = launch_spline.time_shift(wait_duration)

        # cleave at the point where it reaches the station merge
        main_merge_spline = slot.spline.copy_right(merge_time)

        # translate the spline to the vehicle's coordinate frame
        main_merge_spline = main_merge_spline.position_shift(launch_knot.pos - merge_knot.pos)

        # Create a spline that carries the vehicle through the launch and on
        # to the main merge.
        vehicle_spline = wait_spline.concat(launch_spline.concat(main_merge_spline))

        return vehicle_spline, vehicle.ts_id

    # TODO: Doesn't utilize relinquished slots yet
    def _find_usable_gap(self, now, station, min_slot_size, min_launch_duration,
                         min_to_merge_duration,  station_merge_line_speed,
                         min_separation, launch_vehicle_length):
        """Looks through the Merge's reservation queue and returns a
        pair of merge_slots. The pair has at least a min_slot_size
        gap between them, and is accessible. The pairs are in order from
        earliest to latest. If the gap is in front of the first slot in the
        reservation queue, then a dummy slot is used as the first element in the
        pair. If the gap is after the last slot, a dummy slot is used
        as the second element in the pair. If no slots are in the queue, then
        pair of dummy slots is returned.

        Note that the vehicle which owns the second element of the pair will
        always be travelling at line speed (or be a dummy).

        The gap is considered accessible if a vehicle launching from station
        would not need to pass through a vehicle on the main line to reach the
        slot, taking into account the min_launch_time.

        Parameters:
          station -- the launch Station object
          min_slot_size -- the minimum time gap. Typically the vehicle's desired
              headway at line speed + the vehicle's length / line speed. Measured
              in seconds.
          min_launch_duration -- the minimum duration required for the TAIL of
              the launch vehicle to reach the station merge point. The launch
              vehicle is expected to be travelling at line speed at this time.
          station_merge_line_speed -- the line speed at the station merge.
          min_separation -- minimum number of seconds of separation between
              vehicles.

        Returns:
          A pair: (front_merge_slot, rear_merge_slot)
        """
        # Find which zone the station is in.
        if station.merge in self.zone_ids[0]:
            station_zone = 0
        else:
            assert station.merge in self.zone_ids[1]
            station_zone = 1

        station_merge_offset = self.offsets[station.merge] # reminder: offsets are negative

        # The launch vehicle cannot pass through another vehicle in the same
        # zone while trying to reach its merge slot. Iterate over existing
        # slots, starting closest to the main-merge and working back,
        # to find the blocking vehicle closest to the station-merge. We
        # can disregard any slots prior to the blocking vehicle's slot.
        # Stop iteration once we reach a vehicle who's nose has not yet
        # reached the station merge track segment.
        blocking_slot_idx = None
        for i, slot in enumerate(self.reservations):
            if slot.relinquished:
                # TODO: Use a relinquished slot
                continue

            if slot.vehicle.ts_id == self.outlet:
                continue

            if slot.zone == station_zone:
                try:
                    # Vehicle's nose is before the station-merge point when the launch vehicle reaches it.
                    # Do a quick check, and if it passes that, do a more accurate, expensive check
                    if self.offsets[slot.vehicle.ts_id] < station_merge_offset \
                       and slot.vehicle.estimate_pose(now + min_launch_duration, self.outlet) < station_merge_offset:  # more accurate check
                        break
                    else: # Vehicle's nose is past the station merge point and is a blocking vehicle.
                        blocking_slot_idx = i
                except KeyError:
                    # Vehicle is just departing from a station, and has not yet
                    # hit the main line.
                    continue

        # Bookend the reserved MergeSlots with a pair of dummy slots.
        if blocking_slot_idx is None:
            slots = [MergeSlot(0,0,None,None,None,-1,self)] + self.reservations + [MergeSlot(inf,inf,None,None,None,-1,self)]
        else:
            slots = self.reservations[blocking_slot_idx:] + [MergeSlot(inf,inf,None,None,None,-1,self)]

        # Now walk through the reachable slots and find an appropriate gap
        tail_clear_merge_duration = min_launch_duration \
                                    + min_to_merge_duration \
                                    + launch_vehicle_length/self.LINE_SPEED
        for front, rear in pairwise(slots):
            assert isinstance(front, MergeSlot)
            assert isinstance(rear, MergeSlot)

            # The tail of the vehicle can't be clear of merge before the rear
            # slot's reservation starts, even though the launch vehicle is
            # travelling as fast as possible.
            if now + tail_clear_merge_duration > rear.start_time:
                continue

            # Skip if gap is too small
            if rear.start_time - front.end_time < min_slot_size:
                continue

            # Having passed the tests, the gap is reachable and usable.
            return (front, rear)

        # Should return directly from the loop above. This point should not be reached.
        assert False, (front, rear, station, min_slot_size, min_launch_duration,
                         min_to_merge_duration,  station_merge_line_speed,
                         min_separation, launch_vehicle_length)

    def cancel_merge_slot(self, slot):
        assert isinstance(slot, MergeSlot)
        self.reservations.remove(slot)

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

    def __init__(self, start_time, end_time, zone, vehicle,
                 spline, spline_coordinate_frame, merge):
        """Holds data relevant to a vehicle's planned trip through the merge
        point of a Merge.

        start_time: When the MergeSlot's reservation of the merge point begins.
            Measured in seconds, where 0 is the start of the sim.
        end_time: When the reservation ends. The vehicle should be clear of the
            merge point at this time.
        zone: Which zone of the Merge controller the vehicle is approaching from.
        vehicle: The Vehicle instance for which the MergeSlot has been created.
        spline: The spline generated to check that the reserved start and end
            times can be viably achieved by the vehicle.
        spline_coordinate_frame: The id of the tracksegment that is the spline's
            coordinate frame.
        merge: The Merge instance for which the MergeSlot has been created.

        relinquished: Indicates that the vehicle that was originally using the
            MergeSlot has entered a station, is no longer in the Merge's
            zone of control, and that another vehicle may safely reuse the
            slot if it is able to synch up with the slot's spline.
        """
        assert start_time >= 0
        assert end_time >= 0
        assert end_time >= start_time
        assert zone in (0,1,None)
        assert isinstance(vehicle, Vehicle) or vehicle is None
        assert isinstance(spline, CubicSpline) or spline is None
        assert isinstance(spline_coordinate_frame, int)
        assert isinstance(merge, Merge)
        self.start_time = start_time # the time at which the vehicle claims the merge point
        self.end_time = end_time # the time at which the vehicle releases the merge point
        self.zone = zone
        self.vehicle = vehicle
        self.spline = spline
        self.spline_frame = spline_coordinate_frame
        self.merge = merge

        self.relinquished = False

    def __str__(self):
        try:
            return "Start: %.3f, End: %.3f, Zone: %d, Vehicle: %d, Relinquished: %s" % \
                   (self.start_time, self.end_time, self.zone, self.vehicle.id, self.relinquished)
        except AttributeError:
            return "Start: %.3f, End: %.3f, Zone: %s, Vehicle: %s, Relinquished: %s" % \
                   (self.start_time, self.end_time, str(self.zone), str(self.vehicle), self.relinquished)

class Switch(object):
    """May force vehicles to reroute, based on conditions downstream of the
    switch.

    Requires that the 'Stations' singleton be initialized!
    """

    def __init__(self, switch_id, switch_node, graph):
        assert isinstance(switch_id, int), type(switch_id)
        assert isinstance(switch_node, int), type(switch_node)
        assert isinstance(graph, networkx.classes.DiGraph), type(graph)

        # Singletons
        stations = Stations()

        self.id = switch_id
        self.inlet = switch_node # the segment which has 2 successors
        self.outlets = [None, None] # the last segments in the zone of control
        self.zoc_stations = set() # stations within the switch's zone of control
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
                length = graph[up_node][down_node]['length']
                self.offsets[up_node] = offset
                offset += length

                # moved onto a non-station merge. End of zone.
                down_predecessors = graph.predecessors(down_node)
                if len(down_predecessors) > 1:
                    is_station = False
                    for p_node in down_predecessors:
                        if stations.get_from_ts(p_node):
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
                    if stations.get_from_ts(down_nodes[0]):
                        up_node = down_node
                        down_node = down_nodes[1] # take the other path
                        self.zoc_stations.add(stations.get_from_ts(down_nodes[0]))
                    elif stations.get_from_ts(down_nodes[1]):
                        up_node = down_node
                        down_node = down_nodes[0] # take the other path
                        self.zoc_stations.add(stations.get_from_ts(down_nodes[1]))
                    else: # Switch is not due to a station. End of zone.
                        self.zone_lengths[zone_num] = offset
                        self.outlets[zone_num] = up_node
                        break

                else: # len(down_nodes) == 0:
                    raise Exception("Not able to handle dead end track.")

def write_stats_report(vehicles, stations, file_path):
    """Writes controller specific statistics to file found at file_path.
    Uses tables in CSV format.

    Parameters:
      vehicles -- A dict containing Vehicle objects, keyed by id.
      stations -- A dict containing Station objects, keyed by id.

    Returns:
      None
    """
    with open(file_path, 'w') as f:
        f.write("prt_controller.py Statistics\n\n")
        f.write("Vehicle Stats\n")

        states_string = ', '.join(States.strings)
        f.write("id, %s, total time, # Wave Offs\n" % states_string)
        v_list = vehicles.values()
        v_list.sort()

        num_states = len(States.strings)
        times = [0]*num_states
        totals = [0]*num_states
        for v in v_list:
            v.state = States.NONE # Causes the last interval to be recorded.
            for i in range(num_states):
                times[i] = v.time_spent_in_states[i]
                totals[i] += v.time_spent_in_states[i]

            times_str = ', '.join('%.3f' % time for time in times)
            f.write("%d, %s, %.3f, %d\n" % (v.id, times_str, sum(times), v.num_wave_offs))

        f.write("Total:, %s" % ', '.join('%.3f' % time for time in totals))

def main(options):
    PrtController.LINE_SPEED = options.line_speed
    PrtController.HEADWAY = options.headway
    Merge.LINE_SPEED = options.line_speed
    Merge.HEADWAY = options.headway
    Station.SPEED_LIMIT = options.station_speed

    ctrl = PrtController(options.logfile, options.comm_logfile, options.stats_file)
    Station.controller = ctrl
    Vehicle.controller = ctrl
    Merge.controller = ctrl

    ctrl.connect(options.server, options.port)

if __name__ == '__main__':
    options_parser = optparse.OptionParser(usage="usage: %prog [options]")
    options_parser.add_option("--logfile", dest="logfile", default="./ctrl.log",
                metavar="FILE", help="Log events to FILE.")
    options_parser.add_option("--comm_logfile", dest="comm_logfile", default="./ctrl_comm.log",
                metavar="FILE", help="Log communication messages to FILE.")
    options_parser.add_option("--stats_file", dest="stats_file", default="./ctrl_stats.csv",
                metavar="FILE", help="Log statistics to FILE.")
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
    options_parser.add_option("--profile", dest="profile_path",
                metavar="FILE", help="Log performance data to FILE. See 'http://docs.python.org/library/profile.html'")
    options, args = options_parser.parse_args()

    if len(args) != 0:
        options_parser.error("Expected zero positional arguments. Received: %s" % ' '.join(args))

    if options.profile_path is not None:
        import cProfile
        if __debug__:
            import warnings
            warnings.warn("Profiling prt_controller.py in debug mode! Specify the -O option for python to disable debug code.")
        else:
            print "Profiling prt_controller.py"
        cProfile.run('main(options)', filename=options.profile_path)
    else:
        main(options)
