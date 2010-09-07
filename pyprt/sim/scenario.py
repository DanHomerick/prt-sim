import logging
import os.path

from station import Berth
import common
import layout
import station
import events
import vehicle
import pyprt.shared.api_pb2 as api

class ScenarioManager(object):
    def __init__(self):
        self.scenario_loaded = False

    def load_scenario(self, xml_path):
        """Loads the scenario, storing the resulting data structures in various
        common variables (TODO: change to functional style?).
        Sets self.scenario_loaded to True upon success."""
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

        # Track Segments
        tracks_xml = doc.getElementsByTagName('TrackSegments')[0]
        common.track_segments = self.load_track_segments(tracks_xml)
        common.digraph = self.build_graph(tracks_xml, common.track_segments)

        # Fill in the next fields for the TrackSegments. Arbitrarily choosen
        # when there is more then one neighbor.
        graph = common.digraph
        for n in graph.nodes_iter():
            neighbors = graph.neighbors(n)
            if neighbors:
                n.next_loc = neighbors[0]
            # else n.next_loc is left as None

        # Vehicles
        common.vehicle_models = self.load_vehicle_models(doc.getElementsByTagName('VehicleModels')[0])
        common.vehicles = self.load_vehicles(doc.getElementsByTagName('Vehicles')[0], common.vehicle_models)

        # Stations
        common.stations = self.load_stations(doc.getElementsByTagName('Stations')[0])
        # mapping from track_segment ids to Platform instances
        common.platforms = dict((p.track_segment.ID, p) for s in common.stations.itervalues() for p in s.platforms)

        # Passengers
        passengers_path = common.config_manager.get_passengers_path()
        if passengers_path is None:
            logging.warning("No passenger file found. Running simulation with no passengers.")
        else:
            default_load_time = common.config_manager.get_pax_load_time()
            default_unload_time = common.config_manager.get_pax_unload_time()
            default_will_share = common.config_manager.get_pax_will_share()
            default_mass = common.config_manager.get_pax_weight()
            pax_list = self.load_passengers(passengers_path, default_load_time, default_unload_time, default_will_share, default_mass)
            common.event_manager.clear_events()
            common.event_manager.add_events(pax_list)

        # Background Image
        self.load_image_meta(doc.getElementsByTagName('Image')[0], xml_path) # only one element

        # TODO: Necessary?
        common.station_list = sorted(s for s in common.stations.itervalues()) # by ID
        common.switch_list.sort()
        common.vehicle_list = sorted(v for v in common.vehicles.itervalues())

        self.scenario_loaded = True
        doc.unlink() # facilitates garbage collection on the XML data

    def load_track_segments(self, track_segments_xml):
        # Create the TrackSegments
        all_tracks = {}
        for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
            trackID = track_segment_xml.getAttribute('id')
            intId = int(trackID.split("_")[0]) # get a unique, integer ID

            start_xml = track_segment_xml.getElementsByTagName('Start')[0]
            start_lat = float(start_xml.getAttribute('lat'))
            start_lng = float(start_xml.getAttribute('lng'))

            end_xml = track_segment_xml.getElementsByTagName('End')[0]
            end_lat = float(end_xml.getAttribute('lat'))
            end_lng = float(end_xml.getAttribute('lng'))

            label = track_segment_xml.getAttribute('label')
            label = label if label != 'null' else ''

            max_speed_str = track_segment_xml.getAttribute('max_speed')
            max_speed = float(max_speed_str) if max_speed_str else 1E1000 # inf if not specified
            ts = layout.TrackSegment(ID=intId,
                               x_start=start_lng,
                               y_start=start_lat,
                               x_end=end_lng,
                               y_end=end_lat,
                               length=float(track_segment_xml.getAttribute('length')),
                               max_speed=max_speed,
                               label=label)
            all_tracks[ts.ID] = ts
        return all_tracks

    def build_graph(self, track_segments_xml, track_segs):
        """Builds and returns a networkx.DiGraph representing the track connectivity.
        Each TrackSegment is represented as a node. Each edge has a weight equal to
        the length of the TrackSegment at the edge's tail (origin).

        track_segments_xml: The 'TrackSegments' element of the XML save file.
        track_segs: a Dictionary with integer ID's as keys, and TrackSegment instances
            as values.
        """
        import networkx
        # Build the graph in two passes. In the first pass, create all nodes using
        # the trackSegment instances. In the second pass, add the edges.
        graph = networkx.DiGraph()
        for ts in track_segs.itervalues():
            graph.add_node(ts)

        for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
            trackID = int(track_segment_xml.getAttribute('id').split("_")[0])
            ts = track_segs[trackID]
            connect_to_xml = track_segment_xml.getElementsByTagName('ConnectsTo')[0]
            for id_xml in connect_to_xml.getElementsByTagName('ID'):
                id = self._to_numeric_id(id_xml)
                connect_ts = track_segs[id]
                graph.add_edge(ts, connect_ts, weight=ts.length)

        return graph

    def load_vehicles(self, vehicles_xml, vehicle_classes):
        all_vehicles = dict()

        for vehicle_xml in vehicles_xml.getElementsByTagName('Vehicle'):
            vId = vehicle_xml.getAttribute('id')
            v_intId = int(vId.split("_")[0]) # get a unique, integer ID
            v_model = vehicle_xml.getAttribute('model_name')
            eId = vehicle_xml.getAttribute('location')
            e_intId = int(eId.split("_")[0])
            loc = common.track_segments[e_intId] # look up edge by id
            position = float(vehicle_xml.getAttribute('position'))

            if position > loc.length:
                raise common.ScenarioError("Vehicle %s starting position: %s "
                                           "is greater than location %s length: %s"
                                           % (v_intId, position, loc.ID, loc.length))
            v = vehicle_classes[v_model](
                                ID=v_intId,
                                loc=loc,
                                position=position,
                                vel=float(vehicle_xml.getAttribute('velocity'))
                            )
            all_vehicles[v.ID] = v
        return all_vehicles


    def load_stations(self, stations_xml):
        all_stations = dict()
        for station_xml in stations_xml.getElementsByTagName('Station'):
            station_label = station_xml.getAttribute('label')
            station_id = int(station_xml.getAttribute('id').split('_')[0])
            try:
                storage_entrance_delay = int(station_xml.getAttribute('storage_entrance_delay'))
            except ValueError:
                logging.warn("station '%s' is missing or has an invalid 'storage_entrance_delay' attribute. Defaulting to 0.", station_label)
                storage_entrance_delay = 0

            try:
                storage_exit_delay = int(station_xml.getAttribute('storage_exit_delay'))
            except ValueError:
                logging.warn("station '%s' is missing or has an invalid 'storage_exit_delay' attribute. Defaulting to 0.", station_label)
                storage_exit_delay = 0

            # Get the TrackSegments
            track_segments = set() # using a set because 'TrackSegmentID' includes the duplicate ts from Platform
            for track_id_xml in station_xml.getElementsByTagName('TrackSegmentID'):
                track_id = self._to_numeric_id(track_id_xml)
                track_segments.add(common.track_segments[track_id])

            # Make the Storage objects and place them in a dict keyed by model_name
            storage_dict = {}
            for storage_xml in station_xml.getElementsByTagName('Storage'):
                model_name = storage_xml.getAttribute('model_name')
                if len(model_name) == 0:
                    raise common.ScenarioError(
                        "Storage for station '%s' is missing or has an invalid or missing 'model_name' attribute" \
                        % (station_label))

                initial_supply_str = storage_xml.getAttribute('initial_supply').lower()
                if initial_supply_str == 'inf':
                    initial_supply = float('inf')
                else:
                    try:
                        initial_supply = int(storage_xml.getAttribute('initial_supply'))
                    except ValueError:
                        raise common.ScenarioError(
                            "Storage for station '%s' is missing or has an invalid 'initial_supply' attribute: %s." \
                            % (station_label, initial_supply_str))

                max_capacity_str = storage_xml.getAttribute('max_capacity').lower()
                if max_capacity_str == 'inf':
                    max_capacity = float('inf')
                else:
                    try:
                        max_capacity = int(storage_xml.getAttribute('max_capacity'))
                    except ValueError:
                        raise common.ScenarioError(
                            "Storage for station '%s' has an invalid or missing 'max_capacity' attribute: %s." \
                            % (station_label, max_capacity_str))

                storage = station.Storage(model_name,
                                          initial_supply,
                                          max_capacity)
                storage_dict[storage.model_name] = storage

            station_ = station.Station(station_id,
                                       station_label,
                                       track_segments,
                                       storage_entrance_delay,
                                       storage_exit_delay,
                                       storage_dict)

            # Make the Platforms
            platforms_xml = station_xml.getElementsByTagName('Platform')
            platforms = [None]*len(platforms_xml)
            for platform_xml in platforms_xml:
                platform_trackseg_id = self._to_numeric_id(platform_xml.getElementsByTagName('TrackSegmentID')[0])
                platform_trackseg = common.track_segments[platform_trackseg_id]
                platform_index = int(platform_xml.getAttribute('index'))
                platform = station.Platform(platform_index, platform_trackseg)

                # Make the Berths
                berths_xml = platform_xml.getElementsByTagName('Berth')
                berths = [None]*len(berths_xml)
                for berth_xml in berths_xml:
                    berth_index = int(berth_xml.getAttribute('index'))
                    start_pos = float(berth_xml.getElementsByTagName('StartPosition')[0].firstChild.data)
                    end_pos = float(berth_xml.getElementsByTagName('EndPosition')[0].firstChild.data)
                    unloading = True if berth_xml.getAttribute('unloading') == 'true' else False
                    loading = True if berth_xml.getAttribute('loading') == 'true' else False
                    storage_entrance = True if berth_xml.getAttribute('storage_entrance') == 'true' else False
                    storage_exit = True if berth_xml.getAttribute('storage_exit') == 'true' else False
                    berths[berth_index] = Berth(berth_index, station_,
                                                platform, start_pos, end_pos,
                                                unloading, loading,
                                                storage_entrance, storage_exit)

                platform.berths = berths
                platforms[platform_index] = platform
            station_.platforms = platforms
            all_stations[station_.ID] = station_
        return all_stations

    def load_passengers(self, filename, default_load_time, default_unload_time, default_will_share, default_mass):
        """Load the list of passenger creation events. See /doc/samplePassenger.tsv
        for format. Station labels may be used in place of station IDs."""
        f = open(filename, 'rU')
        lines = f.readlines()
        paxlist = list()

        # generate dict mapping both ids and labels to stations
        aliases = {}
        for station in common.stations.values():
            aliases[station.label] = station # the label
            aliases[str(station.ID)] = station # the integer id, in string form

        for line_num, line in enumerate(lines):
            if line.isspace():
                continue
            elif line[0] == '#':
                continue
            else:
                data = line.split('\t')
                try:
                    pID = int(data[0])
                    sStatID = data[1].strip() # left in string form
                    dStatID = data[2].strip() # left in string form
                    sTime = float(data[3])
                except IndexError:
                    raise common.ScenarioError(
                        "Line %s of passengerfile %s has too few fields."
                        % (line_num+1, filename))
                except ValueError:
                    raise common.ScenarioError(
                        "Line %s of passengerfile %s has a field of the incorrect type."
                        % (line_num+1, filename))
                try:
                    load_delay = float(data[4])
                except (IndexError, ValueError):
                    load_delay = default_load_time
                try:
                    unload_delay = float(data[5])
                except (IndexError, ValueError):
                    unload_delay = default_unload_time
                try:
                    will_share_str = data[6].strip().lower()
                    if will_share_str in ("1", "yes", "true", "on"):
                        will_share = True
                    elif will_share_str in ("0", "no", "false", "off"):
                        will_share = False
                    elif not will_share_str:
                        raise ValueError
                    else:
                        raise common.ScenarioError(
                            "On line %s of passengerfile %s: '%s' is "
                            "not an acceptable boolean value."
                            % (line_num+1, filename, will_share_str))
                except (IndexError, ValueError):
                    will_share = default_will_share
                try:
                    mass = int(data[7])
                except (IndexError, ValueError):
                    mass = default_mass
                try:
                    sStat = aliases[sStatID]
                    dStat = aliases[dStatID]
                except KeyError:
                    raise common.ScenarioError(
                        "On line %s of passengerfile %s, at least one of the "
                        "following ID's is not a station: %s, %s"
                        % (line_num+1, filename, sStatID, dStatID))
                e = events.Passenger(time=sTime, ID=pID,
                                      src_station=sStat, dest_station=dStat,
                                      load_delay=load_delay,
                                      unload_delay=unload_delay,
                                      will_share=will_share,
                                      mass=mass)
                paxlist.append(e)

        return paxlist

    def load_image_meta(self, image_xml, xml_path):
        """Sets common.img_ybounds and common.img_xbounds to 2-tuples containing the
        min/max lat/lng values. Also stores the image path in common.image_path"""
        nw_xml = image_xml.getElementsByTagName('Northwest')[0]
        se_xml = image_xml.getElementsByTagName('Southeast')[0]
        lngs = [float(x.getAttribute('lng')) for x in (nw_xml, se_xml)]
        lats = [float(x.getAttribute('lat')) for x in (nw_xml, se_xml)]
        common.img_xbounds = (min(lngs), max(lngs))
        common.img_ybounds = (min(lats), max(lats))
        common.img_width = int(image_xml.getAttribute('width'))
        common.img_height = int(image_xml.getAttribute('height'))

        if image_xml.hasAttribute('extension'): # Encoding only the filename extension is the current method.
            common.img_path = os.path.splitext(xml_path)[0] + image_xml.getAttribute('extension')
        else: # Legacy
            common.img_path = os.path.dirname(xml_path) + '/' + image_xml.getAttribute('filename')


    def _to_numeric_id(self, element):
        """For elements similar to:
            <ID>x_trackSegment_forward</ID>
        where x is an integer. Returns just the integer value."""
        return int(element.childNodes[0].data.split('_')[0])

    def load_vehicle_models(self, vehicle_models_xml):
        """Returns a dict of new Python classes. Each value in the dict is a
        subclass of BaseVehicle and has jerk_max_norm, jerk_min_norm,
        accel_max_norm, etc. as class attributes."""
        all_models = {}
        for vehicle_model_xml in vehicle_models_xml.getElementsByTagName('VehicleModel'):
            jerk_xml = vehicle_model_xml.getElementsByTagName('Jerk')[0]
            accel_xml = vehicle_model_xml.getElementsByTagName('Acceleration')[0]
            vel_xml = vehicle_model_xml.getElementsByTagName('Velocity')[0]

            # read xml
            model_name = str(vehicle_model_xml.getAttribute('model_name'))
            length = float(vehicle_model_xml.getAttribute('length'))
            vehicle_mass = int(vehicle_model_xml.getAttribute('mass'))
            max_pax_capacity = int(vehicle_model_xml.getAttribute('passenger_capacity'))
            jerk_max_norm = float(jerk_xml.getAttribute('normal_max'))
            jerk_min_norm = float(jerk_xml.getAttribute('normal_min'))
            jerk_max_emerg = float(jerk_xml.getAttribute('emergency_max'))
            jerk_min_emerg = float(jerk_xml.getAttribute('emergency_min'))
            accel_max_norm = float(accel_xml.getAttribute('normal_max'))
            accel_min_norm = float(accel_xml.getAttribute('normal_min'))
            accel_max_emerg = float(accel_xml.getAttribute('emergency_max'))
            accel_min_emerg = float(accel_xml.getAttribute('emergency_min'))
            vel_max_norm = float(vel_xml.getAttribute('normal_max'))
            vel_min_norm = float(vel_xml.getAttribute('normal_min'))
            vel_max_emerg = float(vel_xml.getAttribute('emergency_max'))
            vel_min_emerg = float(vel_xml.getAttribute('emergency_min'))

            # data validation
            if len(model_name) == 0:
                raise common.ScenarioError("Invalid model name: %s" % model_name)
            if length <= 0:
                raise common.ScenarioError("Vehicle length must be positive: %s" % length)
            if vehicle_mass <= 0:
                raise common.ScenarioError("Invalid vehicle mass: %s" % vehicle_mass)
            if max_pax_capacity < 0:
                raise common.ScenarioError("Negative maximum vehicle passenger capacity: %s" % max_pax_capacity)
            if jerk_max_norm < 0:
                raise common.ScenarioError("Negative jerk_max_norm: %s" % jerk_max_norm)
            if jerk_min_norm > 0:
                raise common.ScenarioError("Positive jerk_min_norm: %s" % jerk_min_norm)
            if jerk_max_emerg < 0:
                raise common.ScenarioError("Negative jerk_max_emerg: %s" % jerk_max_emerg)
            if jerk_min_emerg > 0:
                raise common.ScenarioError("Positive jerk_min_emerg: %s" % jerk_min_emerg)
            if accel_max_norm < 0:
                raise common.ScenarioError("Negative accel_max_norm: %s" % accel_max_norm)
            if accel_min_norm > 0:
                raise common.ScenarioError("Positive accel_min_norm: %s" % accel_min_norm)
            if accel_max_emerg < 0:
                raise common.ScenarioError("Negative accel_max_emerg: %s" % accel_max_emerg)
            if accel_min_emerg > 0:
                raise common.ScenarioError("Positive accel_min_emerg: %s" % accel_min_emerg)
            if vel_max_norm < 0:
                raise common.ScenarioError("Negative vel_max_norm: %s" % vel_max_norm)
            if vel_min_norm != 0:
                raise common.ScenarioError("vel_min_norm may not be non-zero (in this version): %s" % vel_min_norm)
            if vel_max_emerg < 0:
                raise common.ScenarioError("Negative vel_max_emerg: %s" % vel_max_emerg)
            if vel_min_emerg != 0:
                raise common.ScenarioError("vel_min_emerg may not be non-zero (in this version): %s" % vel_min_emerg)
            for type_str, norm, emerg in zip(
                    ('jerk', 'jerk', 'accel', 'accel', 'vel', 'vel'),
                    (jerk_max_norm, jerk_min_norm, accel_max_norm, accel_min_norm, vel_max_norm, vel_min_norm),
                    (jerk_max_emerg, jerk_min_emerg, accel_max_emerg, accel_min_emerg, vel_max_emerg, vel_min_emerg)):
                if abs(emerg) < abs(norm):
                    raise common.ScenarioError(
                        "Emergency " + type_str + " is less than the normal value. "
                        "Emergency values represent the limits of a vehicle's "
                        "capabilities (rather than limits for passenger comfort) "
                        "and cannot be more constrained than the normal values.")
            model = vehicle.create_vehicle_class(
                model_name, length, vehicle_mass, max_pax_capacity,
                jerk_max_norm, jerk_min_norm, jerk_max_emerg, jerk_min_emerg,
                accel_max_norm, accel_min_norm, accel_max_emerg, accel_min_emerg,
                vel_max_norm, vel_min_norm, vel_max_emerg, vel_min_emerg
            )

            all_models[model_name] = model

        return all_models

# a testing stub
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print "Usage: %s FILENAME" % sys.argv[0]
        sys.exit(-1)
    else:

        import ConfigParser
        import os

        conf = ConfigParser.SafeConfigParser()
        conf.read(sys.argv[1])
        conf_dir = os.path.dirname(sys.argv[1]) + os.path.sep

        network_filename = conf_dir + conf.get('Input Files', 'scenario')
        manager = ScenarioManager(network_filename)
        manager.load_scenario()

        print "Switches:"
        print common.switch_list
        print "------"
        print "Stations:"
        print [stat.ID for stat in common.station_list]
        print "------"
        print "TrackSegments:"
        print common.track_segments.values()
        print "------"
        print "Vehicles:"
        print [v.ID for v in common.vehicles.itervalues()]
