##if __name__ == '__main__':
##    # Ensure that other modules are imported from the same version of pyprt
##    import sys
##    import os.path
##    abs_file = os.path.abspath(__file__)
##    rel_pyprt_path = os.path.dirname(abs_file) + os.path.sep + os.path.pardir + os.path.sep + os.path.pardir
##    abs_pyprt_path = os.path.abspath(rel_pyprt_path)
##    sys.path.insert(0, abs_pyprt_path)

import Queue
import logging
import optparse

# Use the 'wxPython' backend rather than 'PyQt'
from enthought.etsconfig.api import ETSConfig
ETSConfig.toolkit = 'wx'

# The program entry point must use absolute imports
import pyprt.sim.common as common
import pyprt.shared.api_pb2 as api
import SimPy.SimulationRT as SimPy
import pyprt.sim.config as config

def main():
    options, pos_args = read_args()
    common.config_manager = config.ConfigManager(options, pos_args)

    import pyprt.sim.events as events
    common.event_manager = events.EventManager()

    import pyprt.sim.scenario as scenario
    common.scenario_manager = scenario.ScenarioManager()

    ###    Start GUI, unless otherwise specified    ###
    disable_gui = common.config_manager.get_disable_gui()
    if disable_gui:
        scenario_path = common.config_manager.get_scenario_path()
        if scenario_path is None:
            print "A scenario file must be specified in either the configuration " \
                  "file or as an argument in order to run without a GUI."
            import sys
            sys.exit(1)
        else:
            run_console_only()


    if not disable_gui:
        import pyprt.sim.gui as gui

        # Should switch over to holding the Sim as an object, instead of a
        # treating it as a magic variable at some point. But not yet.
##        common.Sim = SimPy.SimulationRT.SimulationRT()
        gui_app = gui.GUI_App(0)
        common.gui = gui_app
        gui_app.MainLoop()

def run_console_only():
    # TODO: IMPLEMENT and DEBUG

    import pyprt.sim.comm as comm
    common.interface = comm.ControlInterface(log=common.config_manager.get_comm_log_file())
    common.config_manager.initialize_logging()

def run_sim_profiled(end_time, callback, *args):
    import cProfile
    cProfile.runctx('run_sim(end_time, callback, *args)', globals(), locals(), common.config_manager.get_profile_path())

def run_sim(end_time, callback, *args):
    """Activate the SimPy Processes and run the simulation.

    end_time (float) : how many seconds to simulate
    callback (function) : A function called after completion of the simulation
    *args: Arguments for callback.
    """
    # initialize SimPy
    SimPy.initialize()

    # activate the vehicles
    for v in common.vehicles.values():
        SimPy.activate(v, v.ctrl_loop())

    # activate the stations
    for s in common.stations.itervalues():
        s.startup()

    # create queues for communication between the plot and the sim
    common.vehicle_data_queue = Queue.Queue()
    common.station_data_queue = Queue.Queue()

    # If viz is not disabled, setup and start the data collectors
    if not common.config_manager.get_disable_viz():
        try:
            chaco_frame_interval = 1.0 / common.config_manager.get_fps()
        except ZeroDivisionError:
            logging.info("fps is set to 0. Running without visualization.")
        else:
            # Create and activate the SimPy Processes which collect data and
            # push it to the visualization module via queues.
            import visual
            common.vehicle_viz_data_collector = visual.VisDataCollector(data_interval=chaco_frame_interval,
                                                                 queue=common.vehicle_data_queue,
                                                                 chaco_interval=chaco_frame_interval)
            SimPy.activate(common.vehicle_viz_data_collector,
                           common.vehicle_viz_data_collector.collect_vehicle_data())

            common.station_viz_data_collector = visual.VisDataCollector(data_interval=1.0, # don't need station updates < 1 sec
                                                                 queue=common.station_data_queue,
                                                                 chaco_interval=chaco_frame_interval)
            SimPy.activate(common.station_viz_data_collector,
                           common.station_viz_data_collector.collect_station_data())

    # start the communication control
    SimPy.activate(common.interface, common.interface.talk())

    # activate the event manager
    SimPy.activate(common.event_manager, common.event_manager.spawn_events())

    # Tell the controller that the sim is starting.
    start_msg = api.SimStart()
    common.interface.send(api.SIM_START, start_msg)

    # start the sim. This thread blocks until the simulation finishes.
    SimPy.simulate(until=end_time, real_time=True, rel_speed=1)
    common.sim_ended = True

    # notify the controller(s) that the simulation is finished
    end_msg = api.SimEnd()
    end_msg.sim_end_time = end_time
    common.interface.send(api.SIM_END, end_msg)

    # Disconnect from the controller(s)
    print "Disconnecting"
    common.interface.disconnect()

    # Write the results to a file
    common.reports.write(common.config_manager.get_results_file(), update=True)

    if not common.config_manager.get_disable_gui():
        import wx
        import pyprt.sim.gui as gui
        evt = gui.SimulationEndEvent(end_time=SimPy.now())
        wx.PostEvent(common.gui, evt)

    callback(*args)

def stop_sim():
    """Stop the simulation and perform any housekeeping tasks."""
    SimPy.stopSimulation()

def read_args():
    """Parse the command line arguments. Returns the 2-tuple (options, args) from optparse.OptionParser"""
    log_levels_list = config.ConfigManager.LOGLEVELS.keys()

    optpar = optparse.OptionParser(usage="usage: %prog [options] [CONFIG]")
    optpar.add_option("--disable_gui", action="store_true", dest="disable_gui",
                      help="Console only. Do not launch a GUI.")
    optpar.add_option("--config", dest="config_path", metavar="FILE",
                      help="Specify a configuration FILE.")
    optpar.add_option("--start_controllers", action="store_true", dest="start_controllers",
                      help="Start external controller specified in config file after startup.")
    optpar.add_option("-s", "--start", action="store_true",
                      help="Start simulation immediately. Implies --controller flag.")
    optpar.add_option("--profile", dest="profile_path", metavar="FILE",
                      help="Profile the sim's performance and store results in FILE (debug). "
                      "Only profiles the Sim thread, not the viz thread or GUI thread.")

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
    group.add_option("--log_level", dest="log_level",
                     choices=log_levels_list,
                     help="Minimum level of importance for which events are logged. Choices are: %s" % ', '.join(log_levels_list))
    group.add_option("--log", dest="log",
                     metavar="FILE", help="Log details to FILE.")
    group.add_option("--comm_log", dest="comm_log",
                     metavar="FILE", help="Log communication messages to FILE.")
    group.add_option("--results", dest="results",
                     metavar="FILE", help="Write results report to FILE. Use '-' for stdout.")
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

if __name__ == '__main__':
    main()

