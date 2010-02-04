import common

def main():
    import config

    common.config_manager = config.ConfigManager()
    scenario_path = common.config_manager.get_scenario_path()

    initialize_logging(common.config_manager)

    import events
    common.event_manager = events.EventManager()

    import comm
    common.interface = comm.ControlInterface(log=common.config_manager.get_comm_logfile())

    import scenario
    common.scenario_manager = scenario.ScenarioManager()

    ###    Start GUI, unless otherwise specified    ###
    disable_gui = common.config_manager.get_disable_gui()
    if disable_gui:
        if scenario_path == None:
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
        gui_app.MainLoop()


def initialize_logging(config_manager):
    import logging
    from sys import stdout
    logfile = config_manager.get_logfile()
    loglevel = config_manager.get_loglevel()

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



if __name__ == '__main__':
    main()

