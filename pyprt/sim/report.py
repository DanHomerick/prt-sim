from __future__ import division # use floating point division by default

from sys import stdout

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import SimPy.SimulationRT as Sim

import common
from pyprt.shared.utility import sec_to_hms
from events import Passenger
from vehicle import BaseVehicle

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
        mean_vel_kph = total_km/sim_hours
        mean_vel_mph = total_miles/sim_hours
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

        # TODO: Station summary statistics

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
    def __init__(self, vehicle_dict):
        super(VehicleReport, self).__init__(title='Vehicles')
        self._vehicle_dict = vehicle_dict
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
        v_list = self._vehicle_dict.values()
        v_list.sort()

        lines = []
        for v in v_list:
            assert isinstance(v, BaseVehicle)
            extrema_velocities = v._spline.get_extrema_velocity()
            max_vel = max(extrema_velocities)
            min_vel = min(extrema_velocities)
            jerks = v._spline.find_jerks()
            max_jerk = max(jerks)
            min_jerk = min(jerks)
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

class Reports(traits.HasTraits):
    """A user interface that displays all the reports in a tabbed notebook."""

    summary_report = traits.Instance(SummaryReport)
    pax_report = traits.Instance(PaxReport)

    @property
    def passengers(self):
        return self.pax_dict.values()

    @property
    def vehicles(self):
        return self.vehicle_dict.values()

    view = ui.View(
               ui.Tabbed(
                   ui.Item('summary_report', label='Summary', style='readonly', editor=ui.TextEditor() ),
                   ui.Item('passengers@', style='custom', editor=Passenger.table_editor),
                   ui.Item('vehicles@', style='custom', editor=BaseVehicle.table_editor),
##                   ui.Item('pax_report', style='custom', editor=ui.InstanceEditor())),
                   show_labels=False,
                ),
               title = 'Simulation Reports',
               kind='modal')

    def __init__(self, pax_dict, vehicle_dict, station_dict):
        super(Reports, self).__init__()
        self.pax_dict = pax_dict
        self.vehicle_dict = vehicle_dict
        self.station_dict = station_dict

        self.summary_report = SummaryReport()
        self.pax_report = PaxReport(self.pax_dict)
        self.vehicle_report = VehicleReport(vehicle_dict)

    def update(self):
        self.summary_report.update()
        self.pax_report.update()
        self.vehicle_report.update()

    def display(self):
        self.summary_report.update()
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




