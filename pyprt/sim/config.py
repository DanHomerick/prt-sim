import os
import optparse
import ConfigParser
import logging

import common

class ConfigManager(object):
    loglevels = {'DEBUG':logging.DEBUG, 'INFO':logging.INFO,
              'WARNING':logging.WARNING, 'ERROR':logging.ERROR,
              'CRITICAL':logging.CRITICAL}

    def __init__(self):
        # Command line options
        self.options, positional_args = self.read_args()

        if len(positional_args) > 1 or (len(positional_args) == 1 and self.options.config_path != None):
            raise Exception("Unrecognized command line arguments: %s" % positional_args)

        # Config file
        self.config_path = None
        if len(positional_args) == 1:
            self.config_path = positional_args[0]
        elif self.options.config_path != None:
            self.config_path = self.options.config_path
        else: # Fallback to a default.
            from pyprt.sim import __path__ as sim_path
            self.config_path = sim_path[0] + '/default.cfg'

        self.config_parser = ConfigParser.SafeConfigParser()
        self.config_dir = os.path.abspath(os.path.dirname(self.config_path)) + os.path.sep
        # Parse the config file
        if not self.config_parser.read(self.config_path): # returned an empty list
            raise common.ConfigError("Unable to read config file: " + self.config_path)

        self.logfile = None

    def read_args(self):
        """Parse the command line arguments. Returns the 2-tuple (options, args) from optparse.OptionParser"""
        optpar = optparse.OptionParser(usage="usage: %prog [options] [CONFIG]")
        optpar.add_option("--disable_gui", action="store_true", dest="disable_gui",
                  help="Console only. Do not launch a GUI.")
        optpar.add_option("--config", dest="config_path", metavar="FILE",
                  help="Specify a configuration FILE.")
        optpar.add_option("--start_controllers", action="store_true", dest="start_controllers",
                  help="Start external controller specified in config file after startup.")
        optpar.add_option("-s", "--start", action="store_true",
                  help="Start simulation immediately. Implies --controller flag.")

        group = optparse.OptionGroup(optpar, "Configuration Options",
                  "These options override any settings specified in the configuration file.")
        group.add_option("--scenario",  dest="scenario_path",
                  metavar="FILE", help="Specify a scenario FILE.")
        group.add_option("--passengers", dest="passengers_path",
                  metavar="FILE", help="Specify a passengers FILE.")
        group.add_option("--port", type="int", dest="TCP_port",
                  help="TCP port which controllers should connect to.")
        group.add_option("--sim_end_time", type="float", dest="sim_end_time",
                  help="Number of seconds to simulate.")
        group.add_option("--disable_viz", action="store_true", dest="disable_viz",
                  help="Disables the visualization of the sim, but does not affect whether the GUI is used.")
        group.add_option("--fps", type="float", dest="fps", help="Target number of frames per second for the visualization.")
        group.add_option("--loglevel", dest="loglevel",
                  choices=self.loglevels.keys(),
                  help="Minimum level of importance for which events are logged. Choices are: %s" % ', '.join(self.loglevels.keys()))
        group.add_option("--logfile", dest="logfile",
                  metavar="FILE", help="Log details to FILE.")
        group.add_option("--comm_logfile", dest="comm_logfile",
                  metavar="FILE", help="Log communication messages to FILE.")
        group.add_option("--pax_load_time", type="float", dest="pax_load_time",
                  help="Default passenger boarding time, in seconds.")
        group.add_option("--pax_unload_time", type="float", dest="pax_unload_time",
                  help="Default passenger disembark time, in seconds.")
        group.add_option("--pax_will_share", dest="pax_will_share",
                  choices=['yes', 'y', 'no', 'n'],
                  help="Default passenger behavior. 'yes' means the passenger will share a vehicle, when given the opportunity. Choices are: yes, y, no, n")
        group.add_option("--pax_weight", dest="pax_weight", type="int",
                  help="Default passenger weight, including all luggage.")
        group.add_option("--track_switch_time", type="float", dest="track_switch_time",
                  help="Time for track-based switching to switch between lines.")
        optpar.add_option_group(group)

        return optpar.parse_args()

    def get_disable_gui(self):
        if self.options.disable_gui:
            return True
        else:
            try:
                return self.config_parser.getboolean("GUI", "disable_gui")
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                logging.warn('No "disable_gui" setting found. Defaulting to False.')
                return False

    def get_config_path(self):
        return self.config_path

    def get_scenario_path(self):
        if self.options.scenario_path != None:
            return self.options.scenario_path
        else:
            try:
                return self.config_dir + self.config_parser.get("Input Files", "scenario")
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                return None

    def get_loglevel(self):
        """Returns an integer loglevel"""
        if self.options.loglevel != None:
            loglevelstr = self.options.loglevel
        else:
            loglevelstr = self.config_parser.get('Simulation', 'log_level')

        loglevel = self.loglevels[loglevelstr]
        return loglevel

    def get_logfile(self):
        if self.options.logfile != None:
            logfile = self.options.logfile
        else:
            logfile = self.config_dir + self.config_parser.get('Output Files', 'log')
        return logfile

    def get_comm_logfile(self):
        if self.options.comm_logfile != None:
            comm_logfile = self.options.comm_logfile
        else:
            comm_logfile = self.config_dir + self.config_parser.get('Output Files', 'comm_log')
        return comm_logfile

    def get_passengers_path(self):
        if self.options.passengers_path != None:
            return self.options.passengers_path
        else:
            try:
                return self.config_dir + self.config_parser.get('Input Files', 'passengers')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                return None

    def get_pax_load_time(self):
        if self.options.pax_load_time != None:
            return self.options.pax_load_time
        else:
            return self.config_parser.getfloat('Passenger', 'pax_load_time')

    def get_pax_unload_time(self):
        if self.options.pax_unload_time != None:
            return self.options.pax_unload_time
        else:
            return self.config_parser.getfloat('Passenger', 'pax_unload_time')

    def get_pax_will_share(self):
        if self.options.pax_will_share != None:
            if self.options.pax_will_share == 'yes' or 'y':
                return True
            else:
                return False
        else:
            return self.config_parser.getboolean('Passenger', 'will_share')

    def get_pax_weight(self):
        if self.options.pax_weight != None:
            return self.options.pax_weight
        else:
            return self.config_parser.getint('Passenger', 'weight')

    def get_TCP_port(self):
        if self.options.TCP_port != None:
            return self.options.TCP_port
        else:
            return self.config_parser.getint('Simulation', 'TCP_port')

    def get_controllers(self):
        """Returns a list of strings that, when executed on the command line,
        will execute the controller programs."""
        items = self.config_parser.items('Controllers')
        return [value for (name, value) in items]

    def get_start_controllers(self):
        return self.options.start_controllers

    def get_sim_end_time(self):
        if self.options.sim_end_time != None:
            return self.options.sim_end_time
        else:
            return self.config_parser.getfloat('Simulation', 'sim_end_time')

    def get_disable_viz(self):
        if self.options.disable_viz != None:
            return self.options.disable_viz
        else:
            try:
                return self.config_parser.getboolean("Visualization", "disable_viz")
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                logging.warn('No "disable_viz" setting found. Defaulting to False.')
                return False

    def get_fps(self):
        if self.options.fps != None:
            return self.options.fps
        else:
            return self.config_parser.getfloat('Visualization', 'fps')

    def get_track_switch_time(self):
        if self.options.track_switch_time != None:
            return self.options.track_switch_time
        else:
            return self.config_parser.getfloat('Switch', 'track_switch_time')