import Queue
import logging

import common
import pyprt.shared.api_pb2 as api
import SimPy.SimulationRT as SimPy

def main():
    import config

    common.config_manager = config.ConfigManager()
    scenario_path = common.config_manager.get_scenario_path()

    initialize_logging(common.config_manager)

    import events
    common.event_manager = events.EventManager()

    import comm
    common.interface = comm.ControlInterface(log=common.config_manager.get_comm_log_file())

    import scenario
    common.scenario_manager = scenario.ScenarioManager()

    ###    Start GUI, unless otherwise specified    ###
    disable_gui = common.config_manager.get_disable_gui()
    if disable_gui:
        if scenario_path is None:
            print "A scenario file must be specified in either the configuration " \
                  "file or as an argument in order to run without a GUI."
            import sys
            sys.exit(1)
        else:
            run_console_only()


    if not disable_gui:
        import gui

        # Should switch over to holding the Sim as an object, instead of a
        # treating it as a magic variable at some point. But not yet.
##        common.Sim = SimPy.SimulationRT.SimulationRT()
        gui_app = gui.GUI_App(0)
        common.gui = gui_app
        gui_app.MainLoop()


def initialize_logging(config_manager):
    from sys import stdout
    logfile = config_manager.get_log_file()
    loglevel = config_manager.get_log_level()

    if logfile == "stdout":
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

def run_console_only():
    # TODO: IMPLEMENT
    pass

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
        import gui
        evt = gui.SimulationEndEvent(end_time=SimPy.now())
        wx.PostEvent(common.gui, evt)

    callback(*args)

def stop_sim():
    """Stop the simulation and perform any housekeeping tasks."""
    SimPy.stopSimulation()


if __name__ == '__main__':
    main()

