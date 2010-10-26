import os
import sys
import ConfigParser
import logging

class ConfigManager(object):
    LOGLEVELS = {'DEBUG':logging.DEBUG, 'INFO':logging.INFO,
                 'WARNING':logging.WARNING, 'ERROR':logging.ERROR,
                 'CRITICAL':logging.CRITICAL}

    APP_NAME = "PRT Simulator" # Used in Windows registry

    def __init__(self, options, positional_args):
        """Takes the output of an OptionParser.parse_args() call as inputs."""
        self.options = options

        if len(positional_args) > 1:
            raise ConfigParser.Error("Expected no more than one positional command line argument, but was supplied with: %s" % positional_args)

        self.config_path = None
        self.config_parser = None
        self.config_dir = None

        # Config file
        if len(positional_args) == 1:
            self.set_config_file(positional_args[0])
            self.initialize_logging()
        elif options.config_path != None:
            self.set_config_file(options.config_path)
            self.initialize_logging()

    def initialize_logging(self):
        logfile = self.get_log_file()
        loglevel = self.get_log_level()

        if os.path.basename(logfile) == "stdout":
            from sys import stdout
            logging.basicConfig(format='%(filename)10s %(funcName)20s %(lineno)d' \
                            '%(levelname)8s %(message)s',
                            stream=stdout,
                            level=loglevel)
        else:
            logging.basicConfig(format='%(filename)10s %(funcName)20s %(lineno)d' \
                            '%(levelname)8s %(message)s',
                            filename=logfile,
                            filemode='w',
                            level=loglevel)

    def get_scenarios_path(self):
        """The 'scenarios' path depends on the platform and installation type."""
        if hasattr(sys, 'frozen'): # Frozen executable (installed)
            if sys.platform == 'win32':
                try:
                    import _winreg
                    subkey = _winreg.OpenKey(_winreg.HKEY_CURRENT_USER, "Software\\" + self.APP_NAME)
                    path = _winreg.QueryValueEx(subkey, "ScenariosPath")[0]
                except WindowsError:
                    # Frozen but not installed? FOR DEBUGGING PURPOSES ONLY!
                    # Assume that I'm operating in the prt-sim/dist folder created by bb_freeze_setup.py
                    path = os.path.abspath('../pyprt/scenarios') # WARNING: FRAGILE
                    return path

            elif sys.platform == 'darwin': # Mac OS X
                raise NotImplementedError
            else:
                raise NotImplementedError

        else:
            from pyprt import __path__ as pyprt_path
            pyprt_path = pyprt_path[0]
            path = os.path.join(pyprt_path, 'scenarios')

        return path

    def get_default_config_path(self):
        """Returns the fully qualified path to the default config file."""
        return os.path.join(self.get_scenarios_path(), 'default.cfg')

    def set_config_file(self, filename):
        filename = os.path.abspath(filename)
        self.config_path = filename
        self.config_dir = os.path.dirname(filename) + os.path.sep
        self.config_parser = ConfigParser.SafeConfigParser()

        # Parse the config file
        if not self.config_parser.read(self.config_path): # returned an empty list
            raise ConfigParser.Error("Unable to read config file: " + self.config_path)

    def get_disable_gui(self):
        if self.options.disable_gui:
            return True
        elif self.config_parser:
            try:
                disable_gui = self.config_parser.getboolean("GUI", "disable_gui")
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                disable_gui = None
        else:
            disable_gui = None
        return disable_gui

    def get_config_dir(self):
        return self.config_dir

    def get_profile_path(self):
        return self.options.profile_path

    def get_scenario_path(self):
        """Returns the fully qualified path to the scenario file, if it has been
        specified. Otherwise returns None.
        """
        if self.options.scenario_path != None:
            return self.options.scenario_path
        elif self.config_parser:
            try:
                return os.path.join(self.config_dir, self.config_parser.get("Input Files", "scenario"))
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                return None
        else:
            return None

    def set_scenario_path(self, path):
        self.options.scenario_path = path

    def get_log_level(self):
        """Returns an integer loglevel if it has been set. Otherwise returns
        the value for DEBUG."""
        if self.options.log_level != None:
            loglevelstr = self.options.log_level
        elif self.config_parser:
            try:
                loglevelstr = self.config_parser.get('Simulation', 'log_level')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                loglevelstr = 'DEBUG'
        else:
            loglevelstr = 'DEBUG'

        return self.LOGLEVELS[loglevelstr]

    def get_log_file(self):
        if self.options.log != None:
            logfile = self.options.log
        elif self.config_parser:
            try:
                logfile = os.path.join(self.config_dir, self.config_parser.get('Output Files', 'log'))
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                logfile = None
        else:
            logfile = None
        return logfile

    def get_comm_log_file(self):
        if self.options.comm_log != None:
            comm_logfile = self.options.comm_log
        elif self.config_parser:
            try:
                comm_logfile = os.path.join(self.config_dir, self.config_parser.get('Output Files', 'comm_log'))
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                comm_logfile = None
        else:
            comm_logfile = None
        return comm_logfile

    def get_results_file(self):
        if self.options.results != None:
            resultsfile = self.options.results
        elif self.config_parser:
            try:
                resultsfile = os.path.join(self.config_dir, self.config_parser.get('Output Files', 'results'))
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                resultsfile = None
        else:
            resultsfile = None
        return resultsfile

    def get_passengers_path(self):
        if self.options.passengers_path != None:
            pax_path = self.options.passengers_path
        elif self.config_parser:
            try:
                pax_path = os.path.join(self.config_dir, self.config_parser.get('Input Files', 'passengers'))
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                pax_path = None
        else:
            pax_path = None
        return pax_path

    def get_pax_load_time(self):
        if self.options.pax_load_time != None:
            pax_load = self.options.pax_load_time
        elif self.config_parser:
            try:
                pax_load = self.config_parser.getfloat('Passenger', 'pax_load_time')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                pax_load = None
        else:
            pax_load = None
        return pax_load

    def get_pax_unload_time(self):
        if self.options.pax_unload_time != None:
            pax_unload = self.options.pax_unload_time
        elif self.config_parser:
            try:
                pax_unload = self.config_parser.getfloat('Passenger', 'pax_unload_time')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                pax_unload = None
        else:
            pax_unload = None
        return pax_unload

    def get_pax_will_share(self):
        if self.options.pax_will_share != None:
            if self.options.pax_will_share.lower() == 'yes' or 'y' or '1':
                will_share = True
            else:
                will_share = False
        elif self.config_parser:
            try:
                will_share = self.config_parser.getboolean('Passenger', 'will_share')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                will_share = None
        else:
            will_share = None
        return will_share

    def get_pax_weight(self):
        if self.options.pax_weight != None:
            pax_weight = self.options.pax_weight
        elif self.config_parser:
            try:
                pax_weight = self.config_parser.getint('Passenger', 'weight')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                pax_weight = None
        else:
            pax_weight = None
        return pax_weight

    def get_TCP_port(self):
        if self.options.TCP_port != None:
            port = self.options.TCP_port
        elif self.config_parser:
            try:
                port = self.config_parser.getint('Simulation', 'TCP_port')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                port = None
        else:
            port = None
        return port

    def get_controller_commands(self):
        """Returns a list of strings, each of which may be given to
        subprocess.Popen to start one controller.
        """
        CTRL_TYPES = ('PRT', 'GTF', 'CUSTOM')
        ctrl_exes = self._get_controller_executables()

        items = self.config_parser.items('Controllers')
        cmd_dict = {}  # keyed by num, value is executable string
        args_dict = {} # keyed by num, value is arguments string
        for (name, value) in items:
            try:
                prefix, num = name.split('_')
                num = int(num)
                if prefix == 'ctrl':
                    if value.upper() in CTRL_TYPES:
                        cmd_dict[num] = ctrl_exes[value.upper()]
                    else:
                        raise ConfigParser.Error("Acceptable values for a 'ctrl_N' entry in the [Controllers] section of a configuration file are 'PRT', 'GTF', or 'CUSTOM' but the value '%s' was supplied." % value)
                elif prefix == 'args':
                    args_dict[num] = value
            except ValueError:
                pass

        result = []
        for key, value in cmd_dict.iteritems():
            try:
                result.append(value + ' ' + args_dict[key])
            except IndexError:
                result.append(value)

        return result

    def _get_controller_executables(self):
        """Returns a dict containing the keys 'PRT' and 'GTF'.
        Values are path_to_executable strings (with additional arguments if necessary).
        """

        result = {}
        if hasattr(sys, 'frozen'):
            if sys.platform == 'win32': # Frozen for Windows
                # Assume that the controllers are installed in the same dir as the sim
                sim = sys.executable
                sim_path = os.path.dirname(sim)
                result['PRT'] = os.path.join(sim_path, 'prt_controller.exe')
                result['GTF'] = os.path.join(sim_path, 'gtf_controller.exe')

            elif sys.platform == 'darwin': # Frozen for Mac OS X
                raise NotImplementedError # TODO: Support this
            else:
                raise NotImplementedError

        else:
            # Executed from an egg or SVN checkout
            from pyprt.ctrl import __path__ as tmp_path
            ctrl_path = tmp_path[0]
            python = sys.executable
            result['PRT'] = python + ' ' + os.path.join(ctrl_path, 'prt_controller.py')
            result['GTF'] = python + ' ' + os.path.join(ctrl_path, 'gtf_controller.py')

        if self.config_parser.has_option('Controllers', 'custom'):
            result['CUSTOM'] = self.config_parser.get('Controllers', 'custom')

        return result

    def get_start_controllers(self):
        return self.options.start_controllers

    def get_sim_end_time(self):
        if self.options.sim_end_time != None:
            end_time = self.options.sim_end_time
        elif self.config_parser:
            try:
                end_time = self.config_parser.getfloat('Simulation', 'sim_end_time')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                end_time = None
        else:
            end_time = None
        return end_time

    def get_disable_viz(self):
        if self.options.disable_viz != None:
            disable_viz = self.options.disable_viz
        elif self.config_parser:
            try:
                disable_viz = self.config_parser.getboolean("Visualization", "disable_viz")
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                disable_viz = None
        else:
            disable_viz = None
        return disable_viz

    def get_fps(self):
        if self.options.fps != None:
            fps =  self.options.fps
        elif self.config_parser:
            try:
                fps = self.config_parser.getfloat('Visualization', 'fps')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                fps = None
        else:
            fps = None
        return fps

    def get_track_switch_time(self):
        if self.options.track_switch_time != None:
            switch_time = self.options.track_switch_time
        elif self.config_parser:
            try:
                switch_time = self.config_parser.getfloat('Switch', 'track_switch_time')
            except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
                switch_time = None
        else:
            switch_time = None
        return switch_time
