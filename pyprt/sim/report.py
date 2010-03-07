from __future__ import division # use floating point division by default

from sys import stdout

import enthought.traits.api as traits
import enthought.traits.ui.api as ui

import common
from pyprt.shared.utility import sec_to_hms
from events import Passenger


class Report(traits.HasTraits):
    """A base class for detailed reports."""
    title = traits.Str

    def __init__(self, title):
        super(Report, self).__init__()
        self.title = title

class SummaryReport(Report):
    """Summary statistics for all sections."""
    summary_str = traits.Str

    def __init__(self):
        super(SummaryReport, self).__init__(title="Summary")
        self.summary_str = self.postsim_summary()

    def postsim_summary(self):
        """Returns a formatted string containing summary statistics."""
        KM_TO_MILES = 0.621371192
        summary = []
        end_time = common.config_manager.get_sim_end_time()
        sim_hours = end_time/3600.

        if common.passengers: # some passengers were simulated
            pax_list = common.passengers.values()
            pax_list.sort()

            success_rate = sum(1 for p in pax_list if p.trip_success)/len(pax_list) * 100
            summary.append("\nSummary Statistics")
            summary.append("Number of Passengers:  %d" % len(pax_list))
            summary.append("Wait times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.wait_time for p in pax_list)),
                            sec_to_hms(sum(p.wait_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.wait_time for p in pax_list))))
            summary.append("Walk times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.walk_time for p in pax_list)),
                            sec_to_hms(sum(p.walk_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.walk_time for p in pax_list))))
            summary.append("Ride times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.ride_time for p in pax_list)),
                            sec_to_hms(sum(p.ride_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.ride_time for p in pax_list))))
            summary.append("Total times (Min/Mean/Max):\t%s\t%s\t%s" % \
                           (sec_to_hms(min(p.total_time for p in pax_list)),
                            sec_to_hms(sum(p.total_time for p in pax_list)/len(pax_list)),
                            sec_to_hms(max(p.total_time for p in pax_list))))
            summary.append("%% Trip success:\t%5d" % success_rate)
            summary.append("")

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
            summary.append("Number of Vehicles:\t%d" % len(common.vehicle_list))
            summary.append("Total vehicle km travelled:\t%10.3f\t(%.3f miles)" % (total_km, total_miles))
            summary.append("Vehicle km travelled (Min/Mean/Max):\t%9.3f\t%9.3f\t%9.3f" % (min_km, mean_km, max_km))
            summary.append("Vehicle miles travelled (Min/Mean/Max):\t%9.3f\t%9.3f\t%9.3f" % (min_miles, mean_miles, max_miles))
            summary.append("Mean velocity:\t%10d km/hr\t(%d mph)" % (mean_vel_kph, mean_vel_mph))
            summary.append("\n")

            summary.append("\nPost Sim Passenger Report")
            pax_display_limit = 5 # Limit the report to include a manageble number of records
            summary.append(("%4s" + "%13s"*8) \
                    % ('pID','srcStat','destStat','curLoc','waitT','walkT','rideT','totalT','Success'))
            for p in pax_list[:pax_display_limit]:
                summary.append(("%4d" + "%13s"*8) \
                        % (p.ID, p.src_station, p.dest_station, p.loc,
                           sec_to_hms(p.wait_time), sec_to_hms(p.walk_time),
                           sec_to_hms(p.ride_time), sec_to_hms(p.total_time),
                           p.trip_success))
            if len(pax_list) > pax_display_limit:
                summary.append('An additional %d passenger records not shown...' % (len(pax_list) - pax_display_limit) )
        else:
            summary.append("\nNo passengers simulated.")

        summary.append("\nPost Sim Vehicle Status")
        summary.append("%4s%15s%15s%25s%15s" \
            % ('vID', 'vPos', 'vVel', 'location', 'distTrav'))
        v_display_limit = 5
        for v in common.vehicle_list[:v_display_limit]:
            summary.append("%4d%15.3f%15.3f%25s%15.3f" \
                    % (v.ID, v.pos, v.vel, v.loc, v.get_dist_travelled()))

        return '\n'.join(summary)

class PaxReport(Report):
    """List of details for all passengers in a gridview"""

    def __init__(self, pax_dict):
        super(PaxReport, self).__init__(title="Passengers")


class Reports(traits.HasTraits):
    """A user interface that displays all the reports in a tabbed notebook."""

    summary_report = traits.Instance(SummaryReport)
    passengers = traits.List # not kept up to date...

    view = ui.View(
               ui.Tabbed(
                   ui.Item('summary_report', label='Summary'),
                   ui.Item('passengers@', label='Passengers', editor=Passenger.pax_table_editor)),
               title = 'Simulation Reports',
               kind='modal')

    def __init__(self, pax_dict, vehicle_dict, station_dict):
        super(Reports, self).__init__()
        self.pax_dict = pax_dict
        self.vehicle_dict = vehicle_dict
        self.station_dict = station_dict

        self.summary_report = SummaryReport()
        self.passengers = pax_dict.values()

    def display(self):
        self.passengers = self.pax_dict.values()
        self.edit_traits()

    def write(self, report_path):
        """Writes the report to the filename specified by report_path. Use '-'
        to write to stdout."""
        if report_path == '-':
            out = stdout
        else:
            out = open(report_path, 'w')

        # TODO: Write the full contents of all reports, separated by headers.
        out.write(self.summary_report.postsim_summary())
