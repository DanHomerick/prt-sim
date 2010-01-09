import logging
import enthought.traits.api as traits
import os.path

from station import Berth
import globals
import layout
import station
import events

class ScenarioManager(object):
    def __init__(self):        
        self.scenario_loaded = False

    def load_scenario(self, xml_path):
        """Loads the scenario, storing the resulting data structures in various
        global variables (TODO: change to functional style?).
        Sets self.scenario_loaded to True upon success."""
        import xml.dom.minidom
        doc = xml.dom.minidom.parse(xml_path)

#        src_nodes = dict()
#        sink_nodes = dict()

        # Track Segments
        tracks_xml = doc.getElementsByTagName('Tracks')[0]
        globals.TrackSegments = self.load_track_segments(tracks_xml)
        globals.DiGraph = self.build_graph(tracks_xml, globals.TrackSegments)

        # Fill in the next fields for the TrackSegments. Arbitrarily choosen
        # when there is more then one neighbor.
        graph = globals.DiGraph
        for n in graph.nodes_iter():
            neighbors = graph.neighbors(n)
            if neighbors:
                n.next = neighbors[0]
            # else n.next is left as None

        # Stations
        globals.Stations = self.load_stations(doc.getElementsByTagName('Stations')[0])

        # Vehicles
        globals.vehicle_models = self.make_vehicle_classes(doc.getElementsByTagName('VehicleModels')[0])
        globals.Vehicles = self.load_vehicles(doc.getElementsByTagName('Vehicles')[0], globals.vehicle_models)

#        # Switches
#        for switch_xml in doc.getElementsByTagName('Switch'):
#            intId = int(switch_xml.getAttribute('id').split("_")[0])
#            lat = switch_xml.getElementsByTagName('LatLng')[0].getAttribute('lat')
#            lng = switch_xml.getElementsByTagName('LatLng')[0].getAttribute('lng')
#            switch = layout.Switch(ID=intId,
#                                 length=0,
#                                 max_speed=float(switch_xml.getAttribute('max_speed')),
#                                 x_start=float(lng), y_start=float(lat), x_end=float(lng), y_end=float(lat))
#            globals.Switches[switch.ID] = switch
#            globals.switch_list.append(switch)

#            for incoming in switch_xml.getElementsByTagName('Incoming'):
#                sink_nodes[incoming.firstChild.data] = switch
#            for outgoing in switch_xml.getElementsByTagName('Outgoing'):
#                src_nodes[outgoing.firstChild.data] = switch

        # Passengers        
        passengers_path = globals.config_manager.get_passengers_path()
        if passengers_path == None:
            logging.warning("No passenger file found. Running simulation with no passengers.")
        else:
            default_load_time = globals.config_manager.get_pax_load_time()
            default_unload_time = globals.config_manager.get_pax_unload_time()
            default_will_share = globals.config_manager.get_pax_will_share()
            pax_list = self.load_passengers(passengers_path, default_load_time, default_unload_time, default_will_share)
            globals.EventM.clear_events()
            globals.EventM.add_events(pax_list)

        # Background Image
        self.load_image_meta(doc.getElementsByTagName('Image')[0], xml_path) # only one element

        # TODO: Necessary?
        globals.station_list.sort() # by ID
        globals.switch_list.sort()
        globals.vehicle_list = sorted(v for v in globals.Vehicles.itervalues())

        self.scenario_loaded = True

    def load_track_segments(self, track_segments_xml):
        # Create the TrackSegments
        all_tracks = {}
        for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
            tsId = track_segment_xml.getAttribute('id')
            intId = int(tsId.split("_")[0]) # get a unique, integer ID

            start_xml = track_segment_xml.getElementsByTagName('Start')[0]
            start_lat = float(start_xml.getAttribute('lat'))
            start_lng = float(start_xml.getAttribute('lng'))

            end_xml = track_segment_xml.getElementsByTagName('End')[0]
            end_lat = float(end_xml.getAttribute('lat'))
            end_lng = float(end_xml.getAttribute('lng'))

            label = track_segment_xml.getAttribute('label')
            label = label if label != 'null' else ''

            ts = layout.TrackSegment(ID=intId,
                               x_start=start_lng,
                               y_start=start_lat,
                               x_end=end_lng,
                               y_end=end_lat,
                               length=float(track_segment_xml.getAttribute('length')),
                               max_speed=float(track_segment_xml.getAttribute('max_speed')),
                               label=label)
            all_tracks[ts.ID] = ts
        return all_tracks

    def build_graph(self, track_segments_xml, track_segs):
        """Builds and returns a networkx.DiGraph representing the track connectivity.
        Each TrackSegment is represented as a node. Each edge has a weight equal to
        the length of the TrackSegment at the edge's tail (origin).

        track_segments_xml: The 'Tracks' element of the XML save file.
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
            tsId = int(track_segment_xml.getAttribute('id').split("_")[0])
            ts = track_segs[tsId]
            connect_to_xml = track_segment_xml.getElementsByTagName('ConnectsTo')[0]
            for id_xml in connect_to_xml.getElementsByTagName('ID'):
                id = self._to_numeric_id(id_xml)
                connect_ts = track_segs[id]
                graph.add_edge(ts, connect_ts, weight=ts.length)

        if __debug__:
            # do some light-weight validation
            if not networkx.is_strongly_connected(graph):
                print "Number of strongly connected components:", networkx.number_strongly_connected_components(graph)
                try:
                    import matplotlib.pyplot as plt
                    networkx.draw(graph)
                    plt.show()
                except ImportError:
                    pass
                raise globals.ConfigError("DiGraph is not strongly connected.")

        return graph

    def load_vehicles(self, vehicles_xml, vehicle_classes):
        all_vehicles = dict()

    #    vType = 'Default Vehicle' # TODO: Add vType to the trackBuilder?
    #
    #    # Get and validate the vehicle constants
    #    length = self.config_parser.getfloat(vType, 'length')
    #    max_pax_capacity = self.config_parser.getfloat(vType, 'max_pax_capacity')
    #    norm_max_accel = self.config_parser.getfloat(vType, 'norm_max_accel')
    #    norm_max_decel = self.config_parser.getfloat(vType, 'norm_max_decel')
    #    norm_max_jerk = self.config_parser.getfloat(vType, 'norm_max_jerk')
    #    emerg_max_accel = self.config_parser.getfloat(vType, 'emerg_max_accel')
    #    emerg_max_decel = self.config_parser.getfloat(vType, 'emerg_max_decel')
    #    emerg_max_jerk = self.config_parser.getfloat(vType, 'emerg_max_jerk')
    #    v_mass = self.config_parser.getfloat(vType, 'v_mass')
    #
    #    # data validation
    #    if length < 0:
    #        raise globals.ConfigError, "Negative vehicle length: %s" % length
    #    if max_pax_capacity < 0:
    #        raise globals.ConfigError, "Negative maximum vehicle passenger capacity: %s" % max_pax_capacity
    #    if norm_max_accel <= 0:
    #        raise globals.ConfigError, "Invalid norm_max_accel: %s" % norm_max_accel
    #    if emerg_max_accel <= 0:
    #        raise globals.ConfigError, "Invalid emerg_max_accel: %s" % emerg_max_accel
    #    if norm_max_decel >= 0:
    #        raise globals.ConfigError, "Invalid norm_max_decel (must be neg): %s" % norm_max_decel
    #    if emerg_max_decel >= 0:
    #        raise globals.ConfigError, "Invalid emerg_max_decel (must be neg): %s" % emerg_max_decel
    #    if v_mass <= 0:
    #        raise globals.ConfigError, "Invalid vehicle mass (v_mass): %s" % v_mass

        for vehicle_xml in vehicles_xml.getElementsByTagName('Vehicle'):
            vId = vehicle_xml.getAttribute('id')
            v_intId = int(vId.split("_")[0]) # get a unique, integer ID
            v_model = vehicle_xml.getAttribute('model')
            eId = vehicle_xml.getAttribute('location')
            e_intId = int(eId.split("_")[0])
            loc = globals.TrackSegments[e_intId] # look up edge by id
            position = float(vehicle_xml.getAttribute('position'))

            if position > loc.length:
                raise globals.ConfigError("Vehicle %s starting position: %s is greater than location %s length: %s" % (v_intId, position, loc.ID, loc.length))
            vehicle = vehicle_classes[v_model](ID=v_intId,
                                     loc=loc,
    #                                 length=length,
    #                                 max_pax_capacity=max_pax_capacity,
    #                                 norm_max_accel=norm_max_accel,
    #                                 norm_max_decel=norm_max_decel,
    #                                 norm_max_jerk=norm_max_jerk,
    #                                 emerg_max_accel=emerg_max_accel,
    #                                 emerg_max_decel=emerg_max_decel,
    #                                 emerg_max_jerk=emerg_max_jerk,
    #                                 v_mass=v_mass,
                                     position=position,
                                     speed=float(vehicle_xml.getAttribute('velocity'))
                                     )
            all_vehicles[vehicle.ID] = vehicle
        return all_vehicles


    def load_stations(self, stations_xml):
        all_stations = dict()
        for station_xml in stations_xml.getElementsByTagName('Station'):
            station_label = station_xml.getAttribute('id')
            station_id = int(station_label.split('_')[0])

            # Get the TrackSegments
            track_segments = []
            for track_xml in station_xml.getElementsByTagName('TrackSegments'):
                for track_id_xml in track_xml.getElementsByTagName('ID'):
                    track_id = self._to_numeric_id(track_id_xml)
                    track_segments.append(globals.TrackSegments[track_id])


            # Make the Platforms
            platforms = []
            for platform_xml in station_xml.getElementsByTagName('Platform'):
                platform_trackseg_id = self._to_numeric_id(platform_xml.getElementsByTagName('TrackSegment')[0])
                platform_trackseg = globals.TrackSegments[platform_trackseg_id]
                berth_length = float(platform_xml.getAttribute('berth_length'))
                unloading = True if platform_xml.getAttribute('unloading') == 'true' else False
                loading = True if platform_xml.getAttribute('loading') == 'true' else False

                # Make the Berths
                berths = []
                for idx, berth_xml in enumerate(platform_xml.getElementsByTagName('Berth')):
                    v_id = berth_xml.childNodes[0].data
                    label = 'berth'+str(idx)
                    if v_id == 'empty':
                        berths.append(Berth(label, station_id, None))
                    else:
                        vehicle = globals.Vehicles[v_id]
                        berths.append(Berth(label, station_id, vehicle))

                platforms.append(station.Platform(berths, platform_trackseg, berth_length, unloading, loading))
            station_ = station.Station(station_id, station_label, platforms, track_segments)
            all_stations[station_.ID] = station_
        return all_stations

    def load_passengers(self, filename, default_load_time, default_unload_time, default_will_share):
        """Load the list of passenger creation events. See /doc/samplePassenger.tsv
        for format. Station labels may be used in place of station IDs."""
        f = open(filename, 'rU')
        lines = f.readlines()
        paxlist = list()

        # generate dict mapping both ids and labels to stations
        aliases = {}
        for station in globals.station_list:
            aliases[str(station)] = station # the label
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
                    sStatID = data[1] # left in string form
                    dStatID = data[2] # left in string form
                    sTime = float(data[3])
                except IndexError:
                    raise globals.ConfigError, "Line %s of passengerfile %s has too few fields." %\
                                       (line_num+1, filename)
                except ValueError:
                    raise globals.ConfigError, "Line %s of passengerfile %s has a field of the incorrect type." %\
                                       (line_num+1, filename)
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
                        raise globals.ConfigError, "On line %s of passengerfile %s: '%s' is " \
                                          "not an acceptable boolean value." % \
                                       (line_num+1, filename, will_share_str)
                except (IndexError, ValueError):
                    will_share = default_will_share
                try:
                    weight = int(data[7])
                except (IndexError, ValueError):
                    weight = self.parser.getint('Passenger', 'weight')
                try:
                    sStat = aliases[sStatID]
                    dStat = aliases[dStatID]
                except KeyError:
                    raise globals.ConfigError, "On line %s of passengerfile %s, at least one of the following ID's is not a station: %s, %s" %\
                                   (line_num+1, filename, sStatID, dStatID)
                e = events.Passenger(time=sTime, ID=pID,
                                      src_station=sStat, dest_station=dStat,
                                      load_delay=load_delay,
                                      unload_delay=unload_delay,
                                      will_share=will_share,
                                      weight=weight)
                paxlist.append(e)

        return paxlist

    def load_image_meta(self, image_xml, xml_path):
        """Sets globals.img_ybounds and globals.img_xbounds to 2-tuples containing the
        min/max lat/lng values. Also stores the image path in globals.image_path"""
        nw_xml = image_xml.getElementsByTagName('Northwest')[0]
        se_xml = image_xml.getElementsByTagName('Southeast')[0]
        lngs = [float(x.getAttribute('lng')) for x in (nw_xml, se_xml)]
        lats = [float(x.getAttribute('lat')) for x in (nw_xml, se_xml)]
        globals.img_xbounds = (min(lngs), max(lngs))
        globals.img_ybounds = (min(lats), max(lats))
        globals.img_width = int(image_xml.getAttribute('width'))
        globals.img_height = int(image_xml.getAttribute('height'))
        globals.img_path = os.path.dirname(xml_path) + '/' + image_xml.getAttribute('filename')

    def _to_numeric_id(self, element):
        """For elements similar to:
            <ID>x_trackSegment_forward</ID>
        where x is an integer. Returns just the integer value."""
        return int(element.childNodes[0].data.split('_')[0])

    def make_vehicle_classes(self, vehicle_models_xml):
        """Returns a dict of new Python classes. Each value in the dict is a
        subclass of BaseVehicle and has jerk_max_norm, jerk_min_norm,
        accel_max_norm, etc. as class attributes."""
        all_models = {}
        max_pax = 0
        for vehicle_model_xml in vehicle_models_xml.getElementsByTagName('VehicleModel'):
            model_name = vehicle_model_xml.getAttribute('model')
            length = float(vehicle_model_xml.getAttribute('length'))
            v_mass = int(vehicle_model_xml.getAttribute('mass'))
            max_pax_capacity = int(vehicle_model_xml.getAttribute('passenger_capacity'))

            jerk_xml = vehicle_model_xml.getElementsByTagName('Jerk')[0]
            accel_xml = vehicle_model_xml.getElementsByTagName('Acceleration')[0]
            vel_xml = vehicle_model_xml.getElementsByTagName('Velocity')[0]

            type_dict = {'name':traits.String(model_name),
                         'length':traits.Float(length),
                         'v_mass':traits.Int(v_mass),
                         'max_pax_capacity':traits.Int(max_pax_capacity),
                         'jerk_max_norm':traits.Float(float(jerk_xml.getAttribute('normal_max'))),
                         'jerk_min_norm':traits.Float(float(jerk_xml.getAttribute('normal_min'))),
                         'jerk_max_emerg':traits.Float(float(jerk_xml.getAttribute('emergency_max'))),
                         'jerk_min_emerg':traits.Float(float(jerk_xml.getAttribute('emergency_min'))),
                         'accel_max_norm':traits.Float(float(accel_xml.getAttribute('normal_max'))),
                         'accel_min_norm':traits.Float(float(accel_xml.getAttribute('normal_min'))),
                         'accel_max_emerg':traits.Float(float(accel_xml.getAttribute('emergency_max'))),
                         'accel_min_emerg':traits.Float(float(accel_xml.getAttribute('emergency_min'))),
                         'vel_max_norm':traits.Float(float(vel_xml.getAttribute('normal_max'))),
                         'vel_min_norm':traits.Float(float(vel_xml.getAttribute('normal_min'))),
                         'vel_max_emerg':traits.Float(float(vel_xml.getAttribute('emergency_max'))),
                         'vel_min_emerg':traits.Float(float(vel_xml.getAttribute('emergency_min')))
            }

            all_models[model_name] = type(str(model_name), (layout.BaseVehicle,), type_dict)
            max_pax = max(max_pax, max_pax_capacity)

        globals.max_vehicle_pax_capacity = max_pax
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
        print globals.switch_list
        print "------"
        print "Stations:"
        print [stat.ID for stat in globals.station_list]
        print "------"
        print "TrackSegments:"
        print globals.TrackSegments.values()
        print "------"
        print "Vehicles:"
        print [v.ID for v in globals.Vehicles.itervalues()];
