from __future__ import division # use floating point division by default

from sys import stdout
import itertools
import math

import numpy
import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.chaco.api as chaco
import enthought.chaco.tools.api as tools
import enthought.enable.api as enable
from enthought.enable.component_editor import ComponentEditor
import enthought.traits.ui.menu as menu
import SimPy.SimulationRT as Sim


import common
from pyprt.shared.utility import sec_to_hms, pairwise
from events import Passenger, PassengerTabularAdapter
from vehicle import BaseVehicle, VehicleTabularAdapater
from station import Station, StationTabularAdapater
from pyprt.shared.cubic_spline import OutOfBoundsError

class Report(traits.HasTraits):
    """A base class for detailed reports."""
    title = traits.Str
    LINE_DELIMETER = '\n'
    FIELD_DELIMITER = ','

    def __init__(self, title):
        super(Report, self).__init__()
        self.title = title

    def type_str(self, obj):
        return obj.__class__.__name__

class SortHandler(ui.Handler):
    column_clicked = traits.Any
    reverse_sort = traits.Bool(False)

    def _column_clicked_changed(self, event):
        """ Sort the functions based on the clicked column.  Reverse the
            order of the sort each time the column is clicked.
        """

        #### This is the list of the rows in the table.
        values = event.editor.value

        #### Reverse the sort order.
        self.reverse_sort = not self.reverse_sort

        # Sort by the clicked on column's field name and in the correct order.
        event.editor._update_visible = True # Workaround for a bug
        fields = [name for label, name in event.editor.adapter.columns]
        field = fields[event.column]
        values.sort(key=lambda x: getattr(x,field), reverse=self.reverse_sort)

class SummaryReport(Report):
    """Summary statistics for all sections."""

    _lines = traits.List(traits.Str)
    _text = traits.Str

    def __init__(self):
        super(SummaryReport, self).__init__(title="Summary")

    def update(self, pax_report, vehicle_report, station_report, power_report):
        """Returns a list of strings containing summary info."""
        assert isinstance(pax_report, PaxReport)
        assert isinstance(vehicle_report, VehicleReport)
        assert isinstance(station_report, StationReport)
        assert isinstance(power_report, PowerReport)

        KM_TO_MILES = 0.621371192
        lines = []

        lines.append("Minutes Simulated:\t %.1f" % (Sim.now()/60.))
        lines.append("")

        # Passenger summary statistics
        if common.passengers: # some passengers were simulated
            pax_list = common.passengers.values()

            success_rate = sum(1 for p in pax_list if p.trip_success)/len(pax_list) * 100
            lines.append("Passenger Statistics")
            lines.append("Number of Passengers:  %d" % len(pax_list))

            min_t, max_t, sum_t = 0, 0, 0
            for p in pax_list:
                t = p.wait_time
                if t < min_t:
                    min_t = t
                if t > max_t:
                    max_t = t
                sum_t += t
            lines.append("Wait times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min_t),
                            sec_to_hms(sum_t/len(pax_list)),
                            sec_to_hms(max_t)))

            min_t, max_t, sum_t = 0, 0, 0
            for p in pax_list:
                t = p.walk_time
                if t < min_t:
                    min_t = t
                if t > max_t:
                    max_t = t
                sum_t += t
            lines.append("Walk times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min_t),
                            sec_to_hms(sum_t/len(pax_list)),
                            sec_to_hms(max_t)))

            min_t, max_t, sum_t = 0, 0, 0
            for p in pax_list:
                t = p.ride_time
                if t < min_t:
                    min_t = t
                if t > max_t:
                    max_t = t
                sum_t += t
            lines.append("Ride times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min_t),
                            sec_to_hms(sum_t/len(pax_list)),
                            sec_to_hms(max_t)))

            min_t, max_t, sum_t = 0, 0, 0
            for p in pax_list:
                t = p.total_time
                if t < min_t:
                    min_t = t
                if t > max_t:
                    max_t = t
                sum_t += t
            lines.append("Total times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min_t),
                            sec_to_hms(sum_t/len(pax_list)),
                            sec_to_hms(max_t)))
            lines.append("%% Trip success:\t%5d" % success_rate)
            lines.append("")

        else:
            lines.append("No passengers simulated.")
            lines.append("")

        # Vehicle summary statistics
        distances = [v.get_dist_travelled() for v in common.vehicle_list]
        operational_times = [v.get_operational_time() for v in common.vehicle_list]
        nonempty_distances = [v.get_nonempty_dist() for v in common.vehicle_list]
        nonempty_times = [v.get_nonempty_time() for v in common.vehicle_list]
        pax_distances = [v.get_pax_dist() for v in common.vehicle_list]
        pax_counts = [v.total_pax for v in common.vehicle_list]

        total_dist = sum(distances)
        total_km = total_dist/1000.
        total_miles = total_km * KM_TO_MILES
        total_nonempty_dist = sum(nonempty_distances)
        total_nonempty_km = total_nonempty_dist/1000.
        total_nonempty_miles = total_nonempty_km * KM_TO_MILES
        total_pax_dist = sum(pax_distances)
        total_pax_km = total_pax_dist/1000.
        total_pax_miles = total_pax_km * KM_TO_MILES
        total_time = sum(operational_times) # seconds
        total_time_hours = total_time/3600.
        total_nonempty_time = sum(nonempty_times) # seconds
        total_nonempty_time_hours = total_nonempty_time/3600.

        mean_km = total_km/len(common.vehicle_list)
        mean_miles = mean_km * KM_TO_MILES
        max_km = max(distances)/1000.
        max_miles = max_km * KM_TO_MILES
        min_km = min(distances)/1000.
        min_miles = min_km * KM_TO_MILES

        try:
            mean_vel_kph = total_km/total_time_hours
            mean_vel_mph = total_miles/total_time_hours
        except ZeroDivisionError:
            mean_vel_kph = 0
            mean_vel_mph = 0

        try:
            mean_pax_vel_kph = total_nonempty_km/total_nonempty_time_hours
            mean_pax_vel_mph = total_nonempty_miles/total_nonempty_time_hours
        except ZeroDivisionError:
            mean_pax_vel_kph = 0
            mean_pax_vel_mph = 0

        total_pax = sum(pax_counts)
        min_pax = min(pax_counts)
        mean_pax = total_pax/len(common.vehicle_list)
        max_pax = max(pax_counts)

        lines.append("Vehicle statistics")
        lines.append("Number of Vehicles:\t%d" % len(common.vehicle_list))
        lines.append("Total vehicle km travelled:\t%10.3f\t(%.3f miles)" % (total_km, total_miles))
        lines.append("Vehicle km travelled (Min/Mean/Max):\t%9.3f\t%9.3f\t%9.3f" % (min_km, mean_km, max_km))
        lines.append("Vehicle miles travelled (Min/Mean/Max):\t%9.3f\t%9.3f\t%9.3f" % (min_miles, mean_miles, max_miles))
        lines.append("Mean velocity:\t%10d km/hr\t(%d mph)" % (mean_vel_kph, mean_vel_mph))
        lines.append("Mean velocity w/ passengers:\t%10d km/hr\t(%d mph)" % (mean_pax_vel_kph, mean_pax_vel_mph))
        lines.append("Total passengers carried:\t%10d" % total_pax)
        lines.append("Pax carried per vehicle (Min/Mean/Max):\t%9d\t%9.3f\t%9d" % (min_pax, mean_pax, max_pax))
        lines.append("Passenger km travelled: %.1f\t(%.1f miles)" % (total_pax_km, total_pax_miles))
        lines.append("")

        # Station summary statistics
        lines.append("Station statistics")
        lines.append("Number of stations:\t%d" % len(common.station_list))
        lines.append("")

        # Power summary statistics
        try:
            lines.append("Power statistics")
            total_energy = power_report.plot_data.get_data('total_energy').get_data()[-1]
            lines.append("Total Energy:\t%.1f KW-hrs" % total_energy)
            lines.append("Energy/Distance:\t%.1f Watt-hrs/km\t (%.1f Watt-hrs/mile)" \
                         % ((total_energy*1000.)/total_km, (total_energy*1000.)/total_miles))
        except (IndexError, ZeroDivisionError):
            pass

        self._lines = lines
        self._text = self.LINE_DELIMETER.join(self._lines)

    def __str__(self):
        return self._text

class PaxReport(Report):
    """List of details for all passengers in a gridview"""

    passengers = traits.List

    traits_view = ui.View(
                      ui.Group(
                          ui.Item('passengers',
                                  editor=ui.TabularEditor(
                                      adapter=PassengerTabularAdapter(),
                                      operations = [],
                                      images = [],
                                      editable=False,
                                      column_clicked='handler.column_clicked'),
                          show_label=False)
                      ),
                      handler=SortHandler(),
                      kind='live'
                  )

    def __init__(self):
        super(PaxReport, self).__init__(title="Passengers")

        self._header = ["id",
                        "CreationTime",
                        "SrcStatId",
                        "DestStatId",
                        "CurrentLocId",
                        "CurrentLocType",
                        "TimeWaiting",
                        "TimeWalking",
                        "TimeRiding",
                        "TotalTime",
                        "Success",
                        "Mass",
                        "WillShare",
                        "UnloadDelay",
                        "LoadDelay",
                        "SrcStationLabel",
                        "DestStationLabel",
                        "CurrLocLabel"]
        self._lines = []

    def update(self):
        # Check if the locally cached vehicle list has gotten stale.
        if len(self.passengers) != len(common.passengers):
            self.passengers = common.passengers.values()
            self.passengers.sort()

        lines = []
        for pax in self.passengers:
            assert isinstance(pax, Passenger)
            lines.append([str(pax.ID),
                        "%.3f" % pax.time,
                        str(pax.src_station.ID),
                        str(pax.dest_station.ID),
                        str(pax.loc.ID),
                        self.type_str(pax.loc),
                        sec_to_hms(pax.wait_time),
                        sec_to_hms(pax.walk_time),
                        sec_to_hms(pax.ride_time),
                        sec_to_hms(pax.total_time),
                        str(pax.trip_success),
                        str(pax.mass),
                        str(pax.will_share),
                        str(pax.unload_delay),
                        str(pax.load_delay),
                        pax.src_station.label,
                        pax.dest_station.label,
                        pax.loc.label])
        self._lines = lines

    def __str__(self):
        line_strings = [self.title, self.FIELD_DELIMITER.join(self._header)]
        for line in self._lines:
            line_str = self.FIELD_DELIMITER.join(line)
            line_strings.append(line_str)
        return self.LINE_DELIMETER.join(line_strings)

class VehicleReport(Report):

    v_list = traits.List

    traits_view = ui.View(
                      ui.Group(
                          ui.Item('v_list',
                                  editor=ui.TabularEditor(
                                      adapter=VehicleTabularAdapater(),
                                      operations = [],
                                      images = [],
                                      editable=False,
                                      column_clicked='handler.column_clicked'),
                                  show_label=False)
                      ),
                      handler=SortHandler(),
                      kind='live'
                  )

    def __init__(self):
        super(VehicleReport, self).__init__(title='Vehicles')
        self._units_notice = "All values reported in units of meters and seconds."
        self._header = ["id",
                        "Label",
                        "LocId",
                        "Position",
                        "Velocity",
                        "Accel",
                        "TotalPassengers",
                        "MaxPassengers",
                        "TimeWeightedAvePax",
                        "DistWeightedAvePax",
                        "DistTravelled",
                        "EmptyDist",
                        "PassengerMeters",
                        "MaxVelocity",
                        "MinVelocity",
                        "MaxAccel",
                        "MinAccel",
                        "MaxJerk",
                        "MinJerk",
                        ]
        self._lines = []

    def update(self):
        # Check if the locally cached vehicle list has gotten stale.
        if len(self.v_list) != len(common.vehicles):
            self.v_list = common.vehicles.values()
            self.v_list.sort()

        lines = []
        for v in self.v_list:
            assert isinstance(v, BaseVehicle)
            v.update_stats()
            extrema_velocities, extrema_times = v._spline.get_extrema_velocities()
            max_vel = max(extrema_velocities)
            min_vel = min(extrema_velocities)
            max_jerk = max(v._spline.j)
            min_jerk = min(v._spline.j)
            lines.append([str(v.ID),
                          v.label,
                          str(v.loc.ID),
                          "%.3f" % v.pos,
                          "%.3f" % v.vel,
                          "%.3f" % v.accel,
                          str(v.total_pax),
                          str(v.max_pax),
                          "%.2f" % v.time_ave_pax,
                          "%.2f" % v.dist_ave_pax,
                          "%d" % v.dist_travelled,
                          "%d" % v.empty_dist,
                          "%d" % v.pax_dist,
                          "%.3f" % max_vel,
                          "%.3f" % min_vel,
                          "%.3f" % v._spline.get_max_acceleration(),
                          "%.3f" % v._spline.get_min_acceleration(),
                          "%.3f" % max_jerk,
                          "%.3f" % min_jerk
                          ])
        self._lines = lines

    def __str__(self):
        line_strings = [self.title,
                        self._units_notice,
                        self.FIELD_DELIMITER.join(self._header)]
        for line in self._lines:
            line_str = self.FIELD_DELIMITER.join(line)
            line_strings.append(line_str)
        return self.LINE_DELIMETER.join(line_strings)

class StationReport(Report):
    s_list = traits.List

    traits_view = ui.View(
                      ui.Group(
                          ui.Item('s_list',
                                  editor=ui.TabularEditor(
                                      adapter=StationTabularAdapater(),
                                      operations = [],
                                      images = [],
                                      editable=False,
                                      column_clicked='handler.column_clicked'),
                                  show_label=False)
                      ),
                      handler=SortHandler(),
                      kind='live'
                  )


    def __init__(self):
        super(StationReport, self).__init__(title='Stations')

        self._header = ["id",
                       "Label",
                       "Platforms",
                       "Berths",
                       "Unload",
                       "Load",
                       "Load|Unload",
                       "Queue",
                       "Current Pax",
                       "Pax Created",
                       "Pax Arrived",
                       "Pax Departed",
                       "Min Pax Wait",
                       "Mean Pax Wait",
                       "Max Pax Wait",
                       "Vehicles Arrived",
                       "Min Vehicle Dwell",
                       "Mean Vehicle Dwell",
                       "Max Vehicle Dwell"] # TODO: Berth specific stats?
        self._lines = []

    def update(self):
        if len(self.s_list) != len(common.stations):
            self.s_list = common.stations.values()
            self.s_list.sort()

        lines = []
        for s in self.s_list:
            assert isinstance(s, Station)
            berths, unload, load, unload_load, queue = 0, 0, 0, 0, 0
            for platform in s.platforms:
                for berth in platform.berths:
                    berths += 1
                    if berth.unloading and berth.loading:
                        unload_load += 1
                    elif berth.unloading:
                        unload += 1
                    elif berth.loading:
                        load += 1
                    else: # no loading or unloading capability
                        queue += 1
            pax_wait_times = s.all_pax_wait_times()
            if pax_wait_times:
                min_pax_wait = sec_to_hms(min(pax_wait_times))
                mean_pax_wait = sec_to_hms(sum(pax_wait_times)/len(pax_wait_times))
                max_pax_wait = sec_to_hms(max(pax_wait_times))
            else:
                min_pax_wait = "N/A"
                mean_pax_wait = "N/A"
                max_pax_wait = "N/A"
            lines.append(["%d" % s.ID,
                         s.label,
                         "%d" % len(s.platforms),
                         "%d" % berths,
                         "%d" % unload,
                         "%d" % load,
                         "%d" % unload_load,
                         "%d" % queue,
                         "%d" % len(s._passengers),
                         "%d" % sum(1 for pax in s._all_passengers if pax.src_station is s),
                         "%d" % s._pax_arrivals_count,
                         "%d" % s._pax_departures_count,
                         min_pax_wait,
                         mean_pax_wait,
                         max_pax_wait,
                         "inc", # TODO: Vehicle-related stats
                         "inc",
                         "inc",
                         "inc"])
        self._lines = lines

    def __str__(self):
        line_strings = [self.title,
                        self.FIELD_DELIMITER.join(self._header)]
        for line in self._lines:
            line_str = self.FIELD_DELIMITER.join(line)
            line_strings.append(line_str)
        return self.LINE_DELIMETER.join(line_strings)

class PowerReport(enable.Component):

    SAMPLE_INTERVAL = 1 # seconds

    v_list = traits.List
    plot_data = traits.Instance(chaco.ArrayPlotData)
    plot_container = traits.Instance(enable.Component)
    plots = traits.Dict

    traits_view = ui.View(
                     ui.HGroup(
##                        ui.Item(name='v_list', editor=ui.EnumEditor(values=[str(v) for v in self.v_list])),
                        ui.Item(name='plot_container', editor=enable.ComponentEditor(), show_label=False)
                     ),
                     kind='live'
                  )

    def __init__(self):
        super(PowerReport, self).__init__(title='Power')

    def update(self):
        # Check if the locally cached vehicle list has gotten stale.
        if len(self.v_list) != len(common.vehicles):
            self.v_list[:] = common.vehicles.values()
            self.v_list.sort()

        self.plot_data = self.make_plot_data(self.v_list)
        self.plots, self.plot_container = self.make_plots(self.plot_data)

    def make_plot_data(self, v_list):
        """Returns a chaco.ArrayPlotData containing the following:
          v_power -- A 2D array where each row is a vehicle (indexes match
                     self.v_list), and each column is a time point.
          total_power - A 1D row array giving the network-wide power usage
                     at each time point.

        Parameters:
          v_list -- a sequence of Vehicle objects, sorted by ID

        Does not support negative velocities.
        """
        end_time = min(common.Sim.now(), common.config_manager.get_sim_end_time())
        sample_times = numpy.arange(0, end_time+self.SAMPLE_INTERVAL, self.SAMPLE_INTERVAL)
        power_array = numpy.zeros( (len(v_list), len(sample_times)), dtype=numpy.float32)

        air_density = common.air_density
        wind_speed = common.wind_speed
        wind_angle = common.wind_direction # 0 is blowing FROM the East

        g = 9.80665 # m/s^2
        PI_2 = math.pi/2
        PI_3_2 = math.pi * 1.5

        for v_idx, v in enumerate(v_list):
            masses = v.get_total_masses(sample_times)

            # The sample times may be out of the vehicle spline's valid range,
            # since the vehicle may not have been created at the beginning of
            # the simulation.
            v_start_time = v._spline.t[0]
            v_end_time = v._spline.t[-1]
            for idx, t in enumerate(sample_times):
                if t >= v_start_time:
                    v_start_idx = idx # left index
                    break
            for idx in xrange(len(sample_times)-1,-1,-1):
                if sample_times[idx] <= v_end_time:
                    v_end_idx = idx+1 # right index
                    break

            v_sample_times = sample_times[v_start_idx:v_end_idx]
            v_knots = v._spline.evaluate_sequence(v_sample_times)

            knots = [None] * len(sample_times)
            knots[v_start_idx:v_end_idx] = v_knots

            CdA = v.frontal_area * v.drag_coefficient

            path_idx = 0
            path_sum = 0
            loc = v._path[path_idx]

            last_elevation = loc.get_elevation(v_knots[0].pos)

            for sample_idx, (t, mass, knot) in enumerate(itertools.izip(sample_times, masses, knots)):
                if knot is None:
                    power_array[v_idx, sample_idx] = 0
                    continue

                # Track where we are on the vehicle's path
                pos = knot.pos - path_sum
                if pos >= loc.length:
                    path_sum += loc.length
                    path_idx += 1
                    pos = knot.pos - path_sum
                    loc = v._path[path_idx]

                # Power required to overcome rolling resistance. Ignores effect of
                # track slope and assumes that rolling resistance is constant
                # at different velocities.
                if v.rolling_coefficient:
                    rolling_power = v.rolling_coefficient * g * mass * knot.vel # Force * velocity
                else:
                    rolling_power = 0 # Rolling resistance not modelled


                # Power to accelerate / decelerate (change in kinetic energy)
                accel_power = mass * knot.accel * knot.vel

                # Power to overcome aero drag
                if wind_speed and knot.vel != 0: # No power use when stopped
                    travel_angle = loc.get_direction(knot.pos - path_sum) # 0 is travelling TOWARDS the East
                    incidence_angle = wind_angle - travel_angle
                    if PI_2 <= incidence_angle <= PI_3_2: # tail wind
                        vel = knot.vel - math.cos(incidence_angle)*wind_speed
                    else: # head wind
                        vel = knot.vel + math.cos(incidence_angle)*wind_speed
                else:
                    vel = knot.vel
                aero_power = 0.5 * air_density * vel*vel*vel * CdA

                # Power from elevation changes (change in potential energy)
                elevation = loc.get_elevation(pos)
                delta_elevation = elevation - last_elevation
                elevation_power = g * delta_elevation
                last_elevation = elevation

                # Adjust power usages by efficiency
                net_power = accel_power + rolling_power + aero_power + elevation_power
                if net_power > 0:
                    net_power /= v.powertrain_efficiency # low efficiency increases power required
                elif net_power < 0:
                    net_power *= v.regenerative_braking_efficiency # low efficiency decreases power recovered

                power_array[v_idx, sample_idx] = net_power

        power_array = numpy.divide(power_array, 1000.0) # convert from Watts to KW

        positive_power = numpy.clip(power_array, 0, numpy.inf)
        positive_total_power = numpy.sum(positive_power, axis=0)

        negative_power = numpy.clip(power_array, -numpy.inf, 0)
        negative_total_power = numpy.sum(negative_power, axis=0)

        net_total_power = positive_total_power + negative_total_power

        energy_array = numpy.cumsum(power_array, axis=1)
        energy_array = numpy.divide(energy_array, 3600/self.SAMPLE_INTERVAL) # convert to KW-hours
        total_energy_array = numpy.sum(energy_array, axis=0)

        return chaco.ArrayPlotData(
            sample_times=chaco.ArrayDataSource(sample_times, sort_order="ascending"),
            positive_total_power=chaco.ArrayDataSource(positive_total_power),
            negative_total_power=chaco.ArrayDataSource(negative_total_power),
            net_total_power=chaco.ArrayDataSource(net_total_power),
            total_energy=chaco.ArrayDataSource(total_energy_array),
            v_power=power_array,
            positive_power=positive_power,
            negative_power=negative_power,
            energy_array=energy_array
            )

    def make_plots(self, plot_data):
        """Create overlapping power and energy plots from the supplied plot_data.

        Parameters:
          plot_data -- A chaco.ArrayPlotData object. Expected to be created
              by self.make_plot_data.

        Return:
          A 2-tuple containing:
            - A dict containing plots, keyed by the plot name.
            - A chaco.OverlayPlotContainer containing the plots.
        """
        times_mapper = chaco.LinearMapper(range=chaco.DataRange1D(plot_data.get_data('sample_times'), ))

        graph_colors = {'positive_total_power':'black',
                        'negative_total_power':'red',
                        'net_total_power':'purple',
                        'total_energy':'green'}

        plots = {} # Dict of all plots

        # Power graphs
        power_names = ['positive_total_power',
                             'negative_total_power',
                             'net_total_power']
        power_data_range = chaco.DataRange1D(*[plot_data.get_data(name) for name in power_names])
        power_mapper = chaco.LinearMapper(range=power_data_range)

        power_plots = {}
        for plot_name in power_names:
            plot = chaco.LinePlot(index=plot_data.get_data('sample_times'),
                                  value=plot_data.get_data(plot_name),
                                  index_mapper=times_mapper,
                                  value_mapper=power_mapper,
                                  border_visible=False,
                                  bg_color='transparent',
                                  line_style='solid',
                                  color=graph_colors[plot_name],
                                  line_width=2)
            power_plots[plot_name] = plot
            plots[plot_name] = plot

        # Energy graphs -- use a different value scale than power
        energy_plot_names = ['total_energy']
        energy_data_range = chaco.DataRange1D(*[plot_data.get_data(name) for name in energy_plot_names])
        energy_mapper = chaco.LinearMapper(range=energy_data_range)

        energy_plots = {}
        for plot_name in energy_plot_names:
            plot = chaco.LinePlot(index=plot_data.get_data('sample_times'),
                                  value=plot_data.get_data(plot_name),
                                  index_mapper=times_mapper,
                                  value_mapper=energy_mapper,
                                  border_visible=False,
                                  bg_color='transarent',
                                  line_style='solid',
                                  color=graph_colors[plot_name],
                                  line_width=2)
            energy_plots[plot_name] = plot
            plots[plot_name] = plot


        # Blank plot -- Holds the grid and axis, and acts as a placeholder when
        # no other graphs are activated.
        blank_values = chaco.ArrayDataSource(numpy.zeros( plot_data.get_data('sample_times').get_size() ))
        blank_plot = chaco.LinePlot(index=plot_data.get_data('sample_times'),
                                    value=blank_values,
                                    index_mapper=times_mapper,
                                    value_mapper=power_mapper,
                                    border_visible=True,
                                    bg_color='transparent',
                                    line_width=0)
        plots['blank_plot'] = plot

        times_axis = chaco.PlotAxis(orientation='bottom',
                                    title="Time (seconds)",
                                    mapper=times_mapper,
                                    component=blank_plot)
        power_axis = chaco.PlotAxis(orientation='left',
                                    title="Power (KW)",
                                    mapper=power_mapper,
                                    component=blank_plot)
        energy_axis = chaco.PlotAxis(orientation='right',
                                     title="Energy (KW-hrs)",
                                     mapper=energy_mapper,
                                     component=blank_plot)
        blank_plot.underlays.append(times_axis)
        blank_plot.underlays.append(power_axis)
        blank_plot.underlays.append(energy_axis)

        # Add zoom capability
        blank_plot.overlays.append(tools.ZoomTool(blank_plot,
                                   tool_mode='range',
                                   axis='index',
                                   always_on=True,
                                   drag_button='left'))

        plot_container = chaco.OverlayPlotContainer()
        for plot in power_plots.itervalues():
            plot_container.add(plot)
        for plot in energy_plots.itervalues():
            plot_container.add(plot)
        plot_container.add(blank_plot)
        plot_container.padding_left = 60
        plot_container.padding_right = 60
        plot_container.padding_top = 20
        plot_container.padding_bottom = 50

        # Legend
        legend = chaco.Legend(component=plot_container, padding=20, align="ur")
        legend.tools.append(tools.LegendTool(legend, drag_button="right"))
        legend.plots = {}
        legend.plots.update(power_plots)
        legend.plots.update(energy_plots)
        plot_container.overlays.append(legend)

        return plots, plot_container

class ReportsHandler(ui.Handler):
    def refresh(self, info):
        info.object.update()

class Reports(traits.HasTraits):
    """A user interface that displays all the reports in a tabbed notebook."""

    summary_report = traits.Instance(SummaryReport)
    vehicle_report = traits.Instance(VehicleReport)
    pax_report = traits.Instance(PaxReport)
    station_report = traits.Instance(StationReport)
    power_report = traits.Instance(PowerReport)

    refresh = menu.Action(name="Refresh", action="refresh")

    view = ui.View(
                ui.Tabbed(
                    ui.Item('summary_report',
                            label='Summary',
                            editor=ui.TextEditor(),
                            style='readonly'),
                    ui.Item('vehicle_report',
                            label='Vehicles',
                            editor=ui.InstanceEditor(),
                            style='custom'
                            ),
                    ui.Item('pax_report',
                            label='Passengers',
                            editor=ui.InstanceEditor(),
                            style='custom'
                            ),
                    ui.Item('station_report',
                            label='Stations',
                            editor=ui.InstanceEditor(),
                            style='custom'
                            ),
                    ui.Item('power_report',
                            label='Power',
                            editor=ui.InstanceEditor(),
                            style='custom'),
                    show_labels=False,
                 ),
                title = 'Simulation Reports',
                width=1000,
                resizable=True,
                handler=ReportsHandler(),
                buttons= [], #[refresh], #TODO: Disabling the refresh button until I can get it to refresh all reports properly
                kind='live')

    def __init__(self):
        super(Reports, self).__init__()
        self.summary_report = SummaryReport()
        self.pax_report = PaxReport()
        self.vehicle_report = VehicleReport()
        self.station_report = StationReport()
        self.power_report = PowerReport()

        self._last_update_time = None

    def update(self):
        if self._last_update_time == Sim.now():
            return

        self.pax_report.update()
        self.vehicle_report.update()
        self.station_report.update()
        self.power_report.update()
        self.summary_report.update(self.pax_report,
                                   self.vehicle_report,
                                   self.station_report,
                                   self.power_report)

        self._last_update_time = Sim.now()

    def display(self, evt=None):
        self.update()
        self.edit_traits()

    def write(self, report_path, update=True):
        """Writes the report to the filename specified by report_path. Use '-'
        to write to stdout."""
        if update:
            self.update()

        if report_path == '-':
            out = stdout
        else:
            out = open(report_path, 'w')

        out.write(str(self.summary_report))
        out.write('\n\n')
        out.write(str(self.pax_report))
        out.write('\n\n')
        out.write(str(self.vehicle_report))
        out.write('\n\n')
        out.write(str(self.station_report))

##### TESTING/DEBUGGING CODE #####

class MockVehicle(object):
    def __init__(self, id):
        self.id = id

    def __str__(self):
        return str(self.id)

class TestPlotter(traits.HasTraits):
    plot = traits.Instance(enable.Component)
    v_list = traits.List
    vehicle_str = traits.Str
    vehicle = traits.Instance(MockVehicle)

    def __init__(self, v_list):
        super(TestPlotter, self).__init__()
        self.v_list = v_list
        self.plot = self.make_plot(self.make_data())
        self.view = self.make_view()
        self.configure_traits(view=self.view)

    def make_data(self):
        x_data = chaco.ArrayDataSource(numpy.arange(0, 25, 1))
        pow_data_1 = chaco.ArrayDataSource(numpy.arange(0, 25, 2))
        pow_data_2 = chaco.ArrayDataSource(numpy.arange(100, 50, -1))
        pow_data_3 = chaco.ArrayDataSource(numpy.arange(-100, 0, 2))
        energy_data_1 = chaco.ArrayDataSource(numpy.arange(1000, 0, -3))

        data = chaco.ArrayPlotData(
            sample_times=x_data,
            positive_total_power=pow_data_1,
            negative_total_power=pow_data_2,
            net_total_power=pow_data_3,
            total_energy=energy_data_1
        )

        return data

    def make_plot(self, plot_data):
        times_mapper = chaco.LinearMapper(range=chaco.DataRange1D(plot_data.get_data('sample_times'), ))

        graph_colors = {'positive_total_power':'black',
                        'negative_total_power':'red',
                        'net_total_power':'purple',
                        'total_energy':'green'}

        # Power graphs
        power_names = ['positive_total_power',
                             'negative_total_power',
                             'net_total_power']
        power_data_range = chaco.DataRange1D(*[plot_data.get_data(name) for name in power_names])
        power_mapper = chaco.LinearMapper(range=power_data_range)


        power_plots = {}
        for plot_name in power_names:
            plot = chaco.LinePlot(index=plot_data.get_data('sample_times'),
                                  value=plot_data.get_data(plot_name),
                                  index_mapper=times_mapper,
                                  value_mapper=power_mapper,
                                  border_visible=False,
                                  bg_color='transparent',
                                  line_style='solid',
                                  color=graph_colors[plot_name],
                                  line_width=2)
            power_plots[plot_name] = plot

        # Energy graphs -- use a different value scale than power
        energy_plot_names = ['total_energy']
        energy_data_range = chaco.DataRange1D(*[plot_data.get_data(name) for name in energy_plot_names])
        energy_mapper = chaco.LinearMapper(range=energy_data_range)

        energy_plots = {}
        for plot_name in energy_plot_names:
            plot = chaco.LinePlot(index=plot_data.get_data('sample_times'),
                                  value=plot_data.get_data(plot_name),
                                  index_mapper=times_mapper,
                                  value_mapper=energy_mapper,
                                  border_visible=False,
                                  bg_color='transarent',
                                  line_style='solid',
                                  color=graph_colors[plot_name],
                                  line_width=2)
            energy_plots[plot_name] = plot


        # Blank plot -- Holds the grid and axis, and acts as a placeholder when
        # no other graphs are activated.
        blank_values = chaco.ArrayDataSource(numpy.zeros( plot_data.get_data('sample_times').get_size() ))
        blank_plot = chaco.LinePlot(index=plot_data.get_data('sample_times'),
                                    value=blank_values,
                                    index_mapper=times_mapper,
                                    value_mapper=power_mapper,
                                    border_visible=True,
                                    bg_color='transparent',
                                    line_width=0)
        times_axis = chaco.PlotAxis(orientation='bottom',
                                    title="Time (seconds)",
                                    mapper=times_mapper,
                                    component=blank_plot)
        power_axis = chaco.PlotAxis(orientation='left',
                                    title="Power (KW)",
                                    mapper=power_mapper,
                                    component=blank_plot)
        energy_axis = chaco.PlotAxis(orientation='right',
                                     title="Energy (KW-hrs)",
                                     mapper=energy_mapper,
                                     component=blank_plot)
        blank_plot.underlays.append(times_axis)
        blank_plot.underlays.append(power_axis)
        blank_plot.underlays.append(energy_axis)

        # Add zoom capability
        blank_plot.overlays.append(tools.ZoomTool(plot,
                                   tool_mode='range',
                                   axis='index',
                                   always_on=True,
                                   drag_button='left'))

        container = chaco.OverlayPlotContainer()
        for plot in power_plots.itervalues():
            container.add(plot)
        for plot in energy_plots.itervalues():
            container.add(plot)
        container.add(blank_plot)
        container.padding = 50

        # Legend
        legend = chaco.Legend(component=container, padding=20, align="ur")
        legend.tools.append(tools.LegendTool(legend, drag_button="right"))
        legend.plots = {}
        legend.plots.update(power_plots)
        legend.plots.update(energy_plots)
        container.overlays.append(legend)

        return container

    def make_view(self):
        return ui.View(
                     ui.HGroup(
                        ui.Item(name='v_list', editor=ui.EnumEditor(values=[str(v) for v in self.v_list])),
                        ui.Item(name='plot', label="", editor=enable.ComponentEditor(), show_label=False)

                     )
                  )


if __name__ == '__main__':
    vehicles = [MockVehicle(0),
                MockVehicle(1),
                MockVehicle(2),
                MockVehicle(3),
                MockVehicle(4),
                MockVehicle(5),
                MockVehicle(6),
                MockVehicle(7)]
    t = TestPlotter(vehicles)
