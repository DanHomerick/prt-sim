from __future__ import division # use floating point division by default

from sys import stdout
import itertools
import math

import numpy
import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.chaco.api as chaco
import enthought.chaco.tools.api as tools
from enthought.enable.component_editor import ComponentEditor
import SimPy.SimulationRT as Sim


import common
from pyprt.shared.utility import sec_to_hms
from events import Passenger
from vehicle import BaseVehicle
from station import Station
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

class SummaryReport(Report):
    """Summary statistics for all sections."""

    def __init__(self):
        super(SummaryReport, self).__init__(title="Summary")
        self._lines = []

    # TODO: Stop using commmon
    def update(self):
        """Returns a list of strings containing summary info."""
        KM_TO_MILES = 0.621371192
        lines = []
        sim_hours = Sim.now()/3600.

        # Passenger summary statistics
        if common.passengers: # some passengers were simulated
            pax_list = common.passengers.values()
            pax_list.sort()

            success_rate = sum(1 for p in pax_list if p.trip_success)/len(pax_list) * 100
            lines.append("Passenger Statistics")
            lines.append("Number of Passengers:  %d" % len(pax_list))
            lines.append("Wait times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.wait_time for p in pax_list)),
                            sec_to_hms(sum(p.wait_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.wait_time for p in pax_list))))
            lines.append("Walk times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.walk_time for p in pax_list)),
                            sec_to_hms(sum(p.walk_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.walk_time for p in pax_list))))
            lines.append("Ride times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.ride_time for p in pax_list)),
                            sec_to_hms(sum(p.ride_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.ride_time for p in pax_list))))
            lines.append("Total times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.total_time for p in pax_list)),
                            sec_to_hms(sum(p.total_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.total_time for p in pax_list))))
            lines.append("%% Trip success:\t%5d" % success_rate)
            lines.append("")

        else:
            lines.append("No passengers simulated.")
            lines.append("")

        # Vehicle summary statistics
        v_total_dist = sum(v.get_dist_travelled() for v in common.vehicle_list)
        total_km = v_total_dist/1000.
        total_miles = total_km * KM_TO_MILES
        mean_km = total_km/len(common.vehicle_list)
        mean_miles = mean_km * KM_TO_MILES
        max_km = max(v.get_dist_travelled() for v in common.vehicle_list)/1000.
        max_miles = max_km * KM_TO_MILES
        min_km = min(v.get_dist_travelled() for v in common.vehicle_list)/1000.
        min_miles = min_km * KM_TO_MILES
        try:
            mean_vel_kph = total_km/sim_hours/len(common.vehicle_list)
            mean_vel_mph = total_miles/sim_hours/len(common.vehicle_list)
        except ZeroDivisionError:
            mean_vel_kph = 0
            mean_vel_mph = 0
        total_pax = sum(v.total_pax for v in common.vehicle_list)
        min_pax = min(v.total_pax for v in common.vehicle_list)
        mean_pax = total_pax/len(common.vehicle_list)
        max_pax = max(v.total_pax for v in common.vehicle_list)
        lines.append("Vehicle statistics")
        lines.append("Number of Vehicles:\t%d" % len(common.vehicle_list))
        lines.append("Total vehicle km travelled:\t%10.3f\t(%.3f miles)" % (total_km, total_miles))
        lines.append("Vehicle km travelled (Min/Mean/Max):\t%9.3f\t%9.3f\t%9.3f" % (min_km, mean_km, max_km))
        lines.append("Vehicle miles travelled (Min/Mean/Max):\t%9.3f\t%9.3f\t%9.3f" % (min_miles, mean_miles, max_miles))
        lines.append("Mean velocity:\t%10d km/hr\t(%d mph)" % (mean_vel_kph, mean_vel_mph))
        lines.append("Total passengers carried:\t%10d" % total_pax)
        lines.append("Pax carried per vehicle (Min/Mean/Max):\t%9d\t%9.3f\t%9d" % (min_pax, mean_pax, max_pax))
        lines.append("")

        # Station summary statistics
        lines.append("Station statistics")
        lines.append("Number of stations:\t%d" % len(common.station_list))


        self._lines = lines

    def __str__(self):
        return self.LINE_DELIMETER.join(self._lines)

class PaxReport(Report):
    """List of details for all passengers in a gridview"""

    passengers = traits.List # not kept up to date...

##    view = ui.View(ui.Item('passengers@', label='Passengers', editor=Passenger.table_editor))


    def __init__(self, pax_dict):
        super(PaxReport, self).__init__(title="Passengers")
        self.pax_dict = pax_dict
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
        self.passengers = self.pax_dict.values()
        self.passengers.sort()

        lines = []
        for pax in self.pax_dict.itervalues():
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
    def __init__(self, v_list):
        super(VehicleReport, self).__init__(title='Vehicles')
        self._v_list = v_list
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
        if len(self._v_list) != len(common.vehicles):
            self._v_list[:] = common.vehicles.values()
            self._v_list.sort()

        lines = []
        for v in self._v_list:
            assert isinstance(v, BaseVehicle)
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
                          str(v.get_max_pax()),
                          "%.2f" % v.get_time_ave_pax(),
                          "%.2f" % v.get_dist_ave_pax(),
                          "%d" % v.get_dist_travelled(),
                          "%d" % v.get_empty_dist(),
                          "%d" % v.get_pax_dist(),
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
    def __init__(self, station_dict):
        super(StationReport, self).__init__(title='Stations')
        self.station_dict = station_dict
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
        s_list = self.station_dict.values()
        s_list.sort()

        lines = []
        for s in s_list:
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

class PowerReport(Report):
    def __init__(self, v_list):
        super(PowerReport, self).__init__(title='Power')
        self.v_list = v_list
        self.plot_data = None
        self.plot = None

    def make_plot_data(self, v_list):
        """Returns a chaco.ArrayPlotData containing the following:
          v_power -- A 2D array where each row is a vehicle (indexes match
                     self.v_list), and each column is a time point.
          total_power - A 1D row array giving the network-wide power usage
                     at each time point.

        Parameters:
          v_list -- a sequence of Vehicle objects, sorted by ID
        """
        end_time = min(common.Sim.now(), common.config_manager.get_sim_end_time())
        num_samples = min(10000, end_time+1) # 10,000 samples, or once per second, whichever is less
        sample_times = numpy.linspace(0, end_time, num_samples)
        power_array = numpy.zeros( (len(v_list), num_samples), dtype=numpy.float32)

        air_density = common.air_density
        wind_speed = common.wind_speed
        wind_angle = common.wind_direction # 0 is blowing FROM the East

        g = 9.80665 # m/s^2
        PI_2 = math.pi/2
        PI_3_2 = math.pi * 1.5

        for v_idx, v in enumerate(v_list):
            masses = v.get_total_masses(sample_times)
            CdA = v.frontal_area * v.drag_coefficient

            path_idx = 0
            path_sum = 0
            loc = v._path[path_idx]

            last_elevation = loc.get_elevation(v._spline.evaluate(v._spline.t[0]).pos)

            for sample_idx, (t, mass) in enumerate(itertools.izip(sample_times, masses)):
                try:
                    knot = v._spline.evaluate(t)
                    # Power to accelerate / decelerate
                    accel_power = mass * knot.accel * knot.vel

                    # Track where we are on the vehicle's path
                    pos = knot.pos - path_sum
                    if pos >= loc.length:
                        path_sum += loc.length
                        path_idx += 1
                        pos = knot.pos - path_sum
                        loc = v._path[path_idx]

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

                    # Power from elevation changes
                    elevation = loc.get_elevation(pos)
                    delta_elevation = elevation - last_elevation
                    elevation_power = g * delta_elevation
                    last_elevation = elevation

                    # Adjust power usages by efficiency
                    net_power = accel_power + aero_power + elevation_power
                    if net_power > 0:
                        net_power /= v.powertrain_efficiency # low efficiency increases power required
                    elif net_power < 0:
                        net_power *= v.regenerative_braking_efficiency # low efficiency decreases power recovered

                    power_array[v_idx, sample_idx] = net_power

                except OutOfBoundsError:
                    power_array[v_idx, sample_idx] = 0

        power_array = numpy.divide(power_array, 1000.0) # convert from Watts to KW

        positive_power = numpy.clip(power_array, 0, numpy.inf)
        positive_total_power = numpy.sum(positive_power, axis=0)

        negative_power = numpy.clip(power_array, -numpy.inf, 0)
        negative_total_power = numpy.sum(negative_power, axis=0)

        net_total_power = positive_total_power + negative_total_power

        return chaco.ArrayPlotData(sample_times=sample_times,
                                   v_power=power_array,
                                   positive_power=positive_power,
                                   positive_total_power=positive_total_power,
                                   negative_power=negative_power,
                                   negative_total_power=negative_total_power,
                                   net_total_power=net_total_power)

    def make_plot(self, plot_data):
        plot = chaco.Plot(plot_data)
        plot.y_axis.title="Power (KW)"
        plot.x_axis.title="Time (seconds)"
        positive_plot = plot.plot(("sample_times", "positive_total_power"), type="line", color='black', line_width=2)
        negative_plot = plot.plot(("sample_times", "negative_total_power"), type="line", color='red', line_width=2)
        net_plot = plot.plot(("sample_times", "net_total_power"), type="line", color='purple', line_width=2)

        # Zoom tool
        plot.overlays.append(tools.SimpleZoom(plot, tool_mode='box', always_on=True))

        # Legend
        legend = chaco.Legend(component=plot, padding=20, align="ur")
        legend.tools.append(tools.LegendTool(legend, drag_button="left"))
        legend.plots = {'Positive Power':positive_plot, 'Negative Power':negative_plot, 'Net Power':net_plot}
        plot.overlays.append(legend)

        return plot

    def update(self):
        # Check if the locally cached vehicle list has gotten stale.
        if len(self.v_list) != len(common.vehicles):
            self.v_list[:] = common.vehicles.values()
            self.v_list.sort()

        self.plot_data = self.make_plot_data(self.v_list)
        self.plot = self.make_plot(self.plot_data)

class Reports(traits.HasTraits):
    """A user interface that displays all the reports in a tabbed notebook."""

    summary_report = traits.Instance(SummaryReport)
    pax_report = traits.Instance(PaxReport)
    power_report = traits.Instance(PowerReport)
    power_plot = traits.Instance(chaco.Plot)

    @property
    def passengers(self):
        return self.pax_dict.values()

    @property
    def vehicles(self):
        return self.v_list

    @property
    def stations(self):
        return self.station_dict.values()

    view = ui.View(
               ui.Tabbed(
                   ui.Item('summary_report', label='Summary', style='readonly', editor=ui.TextEditor() ),
                   ui.Item('passengers@', style='custom', editor=Passenger.table_editor),
                   ui.Item('vehicles@', style='custom', editor=BaseVehicle.table_editor),
                   ui.Item('stations@', style='custom', editor=Station.table_editor),
                   ui.Item('power_plot', label='Power', editor=ComponentEditor()),
                   show_labels=False,
                ),
               title = 'Simulation Reports',
               kind='modal')

    def __init__(self, pax_dict, vehicle_dict, station_dict):
        super(Reports, self).__init__()
        self.pax_dict = pax_dict
        self.station_dict = station_dict
        self.v_list = vehicle_dict.values()
        self.v_list.sort()

        self.summary_report = SummaryReport()
        self.pax_report = PaxReport(self.pax_dict)
        self.vehicle_report = VehicleReport(self.v_list)
        self.station_report = StationReport(station_dict)
        self.power_report = PowerReport(self.v_list)

        self._last_update_time = None

    def update(self):
        if self._last_update_time == Sim.now():
            return

        self.summary_report.update()
        self.pax_report.update()
        self.vehicle_report.update()
        self.station_report.update()
        self.power_report.update()
        self.power_plot = self.power_report.plot

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




