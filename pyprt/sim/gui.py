from __future__ import division # always use floating point division, unless specified otherwise

import threading
import subprocess
import time
import logging
import os
import sys
from ConfigParser import NoSectionError
from ConfigParser import Error as ConfigError

import wx
import wx.lib.newevent
from numpy import inf, arange
import enthought.chaco.api as chaco

# This Window object allows the plot to look like a generic Panel to WX.
from enthought.enable.wx_backend.api import Window

import common
common.real_time = True
import SimPy.SimulationRT as SimPy
import main
from menubar_manager import MenuBarManager
import visual
import report

# Create a new Event type to signal the end of the simulation.
# The motivation for this is that, apparently, Traits can't display a window
# (via configure_traits() or display_traits()) when called from a thread other
# than main.
# See: http://wiki.wxpython.org/CustomEventClasses
SimulationEndEvent, SIMULATION_END_EVENT = wx.lib.newevent.NewEvent() # Not a command event, so it won't propagate up a display heirarchy.


class GUI_App(wx.App):

    def OnInit(self):
        wx.InitAllImageHandlers()

        main_window = MainWindow(None, -1, "")
        self.SetTopWindow(main_window)

        self.Bind(SIMULATION_END_EVENT, common.reports.display)

        main_window.Show()

        return 1

class MainWindow(wx.Frame):
    def __init__(self, *args, **kwds):
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.visualizer = None # Set to a Visualizer by Open Config...
        self.chaco_window = Window(self, component=None)

        # Menu Bar
        self.menubar_manager = MenuBarManager(self)

        # Status Bar
        self.statusbar = self.CreateStatusBar(3)

        self.__set_properties()
        self.__do_layout()

        # If the scenario file has already been specified, load it.
        scenario_path = common.config_manager.get_scenario_path()
        if scenario_path is not None:
            self.load_scenario(scenario_path)

        if common.config_manager.get_start_controllers() == True:
            self.simmenu_connectcontroller_handler(None)

        common.reports = report.Reports()

    def __set_properties(self):
        self.SetTitle("PRT Simulation")

    def __do_layout(self):
        # We'll create a default sizer and add the Window to it.  Since Window
        # is an Enable object, we need to get its corresponding WX control.
        # This is stored in its ".control" attribute.
        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.sizer.Add(self.chaco_window.control, 1, flag=wx.EXPAND)
        self.SetSizer(self.sizer)
        self.Layout()

    def load_scenario(self, filename):
        manager = common.scenario_manager

        try:
            manager.load_scenario(filename)
        except common.ScenarioError as err:
            msg_dialog = wx.MessageDialog(parent=self,
                                          message="The following error occurred while loading the scenario:\n%s\n\n%s" % (filename, err.msg),
                                          caption='Scenario Error',
                                          style=wx.ICON_ERROR)
            msg_dialog.ShowModal()
            return

        # adjust the window size to match the image.
        self.SetSize(wx.Size(common.img_width+7, common.img_height+76)) # width, height, 76 for menu bar and 7 for border
        my_size = self.GetSize()
        display_size = wx.GetDisplaySize()
##        if (my_size[0] > display_size[0] or my_size[1] > display_size[1]): # if too big
##            self.SetSize(wx.Size(common.img_width/2, common.img_height/2)) # use half-resolution

        # Create the Visualizer
        max_pax_capacity = max(v.max_pax_capacity for v in common.vehicles.itervalues())
        self.visualizer = visual.Visualizer(common.digraph, max_pax_capacity)
        self.chaco_window.component = self.visualizer.plot

        # update what options are enabled in the menu
        self.menubar_manager.config_loaded()

    def filemenu_open_config_handler(self, event):
        """Load configuration info from file."""
        default_path = common.config_manager.get_scenarios_path()
        default_file = os.path.join('config.ini')

        config_path = wx.FileSelector(flags=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST,
                                   wildcard='config files (*.ini)|*.ini|all files (*.*)|*.*',
                                   default_path=default_path,
                                   default_filename=default_file)

        wx.BeginBusyCursor()
        common.config_manager.set_config_file(config_path)
        common.config_manager.initialize_logging()

        # If the config file specifies a scenario, load it.
        scenario_path = common.config_manager.get_scenario_path()
        if scenario_path is not None:
            self.load_scenario(scenario_path)

        wx.EndBusyCursor()

    def filemenu_open_scenario_handler(self, event):
        """Load the scenario from file."""
        filename = wx.FileSelector(flags=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST,
                                   wildcard='xml files (*.xml)|*.xml|all files (*.*)|*.*',
                                   default_path='../scenarios/',
                                   default_filename='scenario.xml')

        if filename: # Empty string if user clicked 'cancel'
            common.config_manager.set_scenario_path(filename)
            self.load_scenario(filename)

    def filemenu_saveconfig_handler(self, event):
        event.Skip()

    def filemenu_savesnapshot_handler(self, event):
        if self.visualizer:
            filename = wx.FileSelector(flags=wx.FD_SAVE |  wx.FD_OVERWRITE_PROMPT,
                                       wildcard='png (*.png)|*.png|gif (*.gif)|*.gif|jpeg (*.jpg)|*.jpg|bitmap (*.bmp)|*.bmp|any (*.*)|*.*')
            if filename:
                try:
                    self.visualizer.save_plot(filename)
                except Exception, e:
                    self.show_message('An error occurred while saving: ' + filename)
        else:
            self.show_message('A configuration file must be loaded first.')

    def filemenu_exit_handler(self, event):
        """Quit. No save capability yet."""
        self.Close()

    def simmenu_start_sim_handler(self, event):
        # Create a new thread for the sim
        end_time = common.config_manager.get_sim_end_time()

        # Bind a timer to the Visualization plot
        viz_timerId = wx.NewId()
        self.viz_timer = wx.Timer(self, viz_timerId)
        self.Bind(wx.EVT_TIMER, self.visualizer.update_plot, id=viz_timerId)

        # Bind another timer to the statusbar update.
        sb_timerId = wx.NewId()
        self.status_bar_timer = wx.Timer(self, sb_timerId)
        self.Bind(wx.EVT_TIMER, self.update_statusbar, id=sb_timerId)

        # Start the timers
        try:
            timer_interval = (1.0 / common.config_manager.get_fps()) * 1000 # in millisec
        except ZeroDivisionError:
            timer_interval = 5000 # 1 update per 5 sec
        self.viz_timer.Start(timer_interval, wx.TIMER_CONTINUOUS)
        self.status_bar_timer.Start(timer_interval, wx.TIMER_CONTINUOUS)

        # Record the start time
        self.start_time = time.time()

        # update menu options
        self.menubar_manager.sim_started()

        # Start the simulation running in a separate thread

        # Run without profiling (normal case)
        if common.config_manager.get_profile_path() is None:
            sim_thread = threading.Thread(name='sim_thread',
                                          target=main.run_sim,
                                          args=[end_time, self.sim_end_callback])
        else: # Run with profiling
            if __debug__:
                import warnings
                warnings.warn("Profiling code that is in debug mode!")
            sim_thread = threading.Thread(name='sim_thread_profiled',
                                          target=main.run_sim_profiled,
                                          args=[end_time, self.sim_end_callback])

        sim_thread.setDaemon(True)
        sim_thread.start()

    def simmenu_stop_sim_handler(self, event):
        main.stop_sim()

    def simmenu_connectcontroller_handler(self, event):
        import pyprt.sim.comm as comm
        common.interface = comm.ControlInterface(log=common.config_manager.get_comm_log_file())
        TCP_port = common.config_manager.get_TCP_port()

        try:
            controllers = common.config_manager.get_controller_commands()
            TCP_port = common.config_manager.get_TCP_port()
            if len(controllers) == 0:
                raise ConfigError
        except ConfigError:
            # Controller info not found in config file. Ask user.
            controller_window = ControllerChoiceDialog(self)
            controller_window.ShowModal()

            # TODO ...

##        dialog = wx.NumberEntryDialog(self,
##                          message='Choose TCP Listen Port',
##                          prompt='Port:',
##                          caption='Connect External Controller...',
##                          value=TCP_port,
##                          min=49152, # Dynamic/Private port range. See: http://www.iana.org/assignments/port-numbers
##                          max=65535)

        common.interface.setup_server_sockets(TCP_port=TCP_port)
        for cmd in controllers:
            cmd = cmd.split()
            if sys.platform == 'win32':
                try:
                    ctrl_proc = subprocess.Popen(cmd,
                                                 cwd=common.config_manager.get_config_dir(),
                                                 bufsize=0 # 0=unbuffered, -1=system default
                                                 )
                except OSError:
                    self.show_message('Failed to open controller using command: %s' % cmd)
                    return

            elif sys.platform == 'darwin': # Mac OS X
                raise NotImplementedError

            else:
                raise NotImplementedError

        wx.BeginBusyCursor()
        common.interface.accept_connections( len(controllers) )
        wx.EndBusyCursor()

        # update what menu options are enabled
        self.menubar_manager.controllers_connected()

    def open_port_handler(self, event):
        import pyprt.sim.comm as comm
        common.interface = comm.ControlInterface(log=common.config_manager.get_comm_log_file())

        dialog = wx.NumberEntryDialog(self,
                                      message="Debugging Tool.\nOpens the specified TCP port.\nThe sim will block until a controller\nconnects at the specified port.\n\nChoose connection port (TCP)",
                                      prompt='Port:',
                                      caption='Connect External Controller...',
                                      value=common.config_manager.get_TCP_port(),
                                      min=49152, # Dynamic/Private port range. See: http://www.iana.org/assignments/port-numbers
                                      max=65535)
        if dialog.ShowModal() == wx.ID_OK:
            port_num = dialog.GetValue()
            common.interface.setup_server_sockets(TCP_port=port_num)
            common.interface.accept_connections( 1 )
            # Sim blocks, then resumes once connection is made.
            self.menubar_manager.controllers_connected()

    def set_sim_speed(self, speed):
        try:
            SimPy.rtset(speed)
        except AttributeError:
            self.show_message('Start simulation first.')

    def viewmenu_zoom_in_handler(self, event):
        self.visualizer.zoom_tool.zoom_in()

    def viewmenu_zoom_out_handler(self, event):
        self.visualizer.zoom_tool.zoom_out()

    def viewmenu_vehicle_handler(self, event):
        labels = [str(v) for v in common.vehicle_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Vehicles', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            common.vehicle_list[i].edit_traits()

    def viewmenu_station_handler(self, event):
        labels = [str(s) for s in common.station_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Stations', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            s = common.station_list[i]
            s.edit_traits()

    def viewmenu_switch_handler(self, event):
        labels = [str(sw) for sw in common.switch_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Stations', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            common.switch_list[i].edit_traits()

    def viewmenu_track_handler(self, event):
        event.Skip()

    def viewmenu_passenger_handler(self, event):
        pax_list = sorted(common.passengers.itervalues())
        labels = [str(p) for p in pax_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Passengers', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            pax_list[i].edit_traits()

    def viewmenu_legend_handler(self, event):
        # use self as the parent, so that the Legend will close when the MainWindow is closed.
        legend = Legend(self, self.visualizer.station_colormap, self.visualizer.vehicle_colormap)

    def viewmenu_reports_handler(self, event):
        # temp!
        import cProfile
        cProfile.run('common.reports.display()', 'reports_profile.dat')
##        common.reports.display()

    def record_start_handler(self, event):
        # setup the timer
        try:
            record_timerId = wx.NewId()
            timer_interval = (1.0 / common.config_manager.get_fps()) * 1000 # in millisec
            self.record_timer = wx.Timer(self, record_timerId)
            self.Bind(wx.EVT_TIMER, self.record_handler, id=record_timerId)
        except ZeroDivisionError:
            self.show_message("Target FPS in the config file is set to 0, so recording is not possible.")
        except NoSectionError: # from ConfigParser
            self.show_message("Config file is missing a 'Simulation' section.")

        # setup the filename
        filename = wx.FileSelector(flags=wx.FD_SAVE |  wx.FD_OVERWRITE_PROMPT,
                                   wildcard='bitmap (*.bmp)|*.bmp|png (*.png)|*.png|gif (*.gif)|*.gif|jpeg (*.jpg)|*.jpg|any (*.*)|*.*')
        try:
            base, ext = filename.split('.')
            self._record_filename_base = base
            self._record_filename_ext = ext
            self._record_count = 0
        except ValueError:
            self.show_message("Unable to use filename. I expect the path to have exactly one '.' in it, and can't handle other cases.")
            return

        # change which menu options are enabled
        self.menubar_manager.record_started()

        # start the timer
        self.record_timer.Start(timer_interval, wx.TIMER_CONTINUOUS)

    def record_stop_handler(self, event):
        self.record_timer.Stop()

        # change which menu options are enabled
        self.menubar_manager.record_stopped()

    def record_handler(self, event):
        if not common.sim_ended:
            try:
                filename = self._record_filename_base + '_%05d' % self._record_count + '.' + self._record_filename_ext
                self.visualizer.save_plot(filename)
                self._record_count += 1
            except Exception:
                # stop the recording
                self.record_stop_handler(event) # passes the wrong event type, but stop doesn't use the event anyways
                self.show_message("Saving the recording snapshot %s failed." % filename)
        else:
            # stop recording
            self.record_stop_handler(event) # passes the wrong event type, but stop doesn't use the event anyways


    def close_handler(self, event):
        # TODO: Check if user wants to save config
        common.sim_ended = True

        if common.interface:
            common.interface.disconnect()

        self.Destroy()

    def update_statusbar(self, evt):
        if not common.sim_ended:
            # calc some passenger stats
            pax_waiting = 0
            total_wait = 0
            max_wait = 0
            for stat in common.station_list: # collected in 'seconds'
                for pax in stat._passengers:
                    pax_waiting += 1
                    pax_wait = pax.wait_time # don't force it to calc the wait time twice
                    total_wait += pax_wait
                    max_wait = max(pax_wait, max_wait)
            if pax_waiting == 0:
                pax_ave_wait = 0
            else:
                pax_ave_wait = total_wait / pax_waiting

            # calc some vehicle stats
            seats_occupied = 0
            vehicles_occupied = 0
            total_vehicles = len(common.vehicles)
            total_seats = 0
            for vehicle in common.vehicle_list:
                total_seats += vehicle.max_pax_capacity
                if vehicle.passengers:
                    vehicles_occupied += 1
                    seats_occupied += len(vehicle.passengers)
            v_avail = total_vehicles - vehicles_occupied
            if total_vehicles > 0:
                v_pct_utilized = vehicles_occupied / total_vehicles * 100
            else:
                v_pct_utilized = 0
            if total_seats > 0:
                seats_pct_utilized = seats_occupied / total_seats * 100
            else:
                seats_pct_utilized = 0

            self.statusbar.SetFields(['SimTime: %.3f RealTime: %.3f' % (SimPy.now(), time.time() - self.start_time), # timers
                                      'PASSENGERS Waiting: %d Ave Wait: %.1f min Max Wait: %.1f min' % (pax_waiting, pax_ave_wait/60, max_wait/60), # report in minutes
                                      'VEHICLES Available: %d %%Utilized: %.1f %%Seats Utilized: %.1f' % (v_avail, v_pct_utilized, seats_pct_utilized)])

    def show_message(self, message):
        """Displays a message in a modal dialog box with an OK button.
        Function returns when once the button is pressed."""
        dialog = wx.MessageDialog(self, message, style=wx.OK)
        dialog.ShowModal()

    def sim_end_callback(self):
        """For now just pops a window announcing that the sim is done."""
        if SimPy.now() == common.config_manager.get_sim_end_time():
            self.show_message("Simulation Complete")
        else:
            self.show_message("Simulation Halted at %.3f" % SimPy.now())

# end of class MainWindow

class ControllerChoiceDialog(wx.Dialog):
    def __init__(self, parent):
        wx.Dialog.__init__(self,
                          parent=parent,
                          title='Connect Controllers',
                          size=(-1,-1) # default sizes
                          )

        port_text = wx.StaticText(self, label="TCP Port:")
        port_spin = wx.SpinCtrl(self,
                                initial=common.config_manager.get_TCP_port(),
                                min=49152,
                                max=65535)

        command_text = wx.StaticText(self, label="Command")
        command_list = common.config_manager.get_controllers()
        command_combobox = wx.ComboBox(self, choices=command_list, style=wx.CB_DROPDOWN)
        self.Bind(wx.EVT_COMBOBOX, self.onComboBox, command_combobox)

        port_sizer = wx.BoxSizer(wx.HORIZONTAL)
        port_sizer.Add(port_text, proportion=1, flag=wx.EXPAND)
        port_sizer.Add(port_spin, proportion=1, flag=wx.EXPAND)

        static_box = wx.StaticBox(self, label="Controller")
        static_box_sizer = wx.StaticBoxSizer(static_box, wx.VERTICAL)
        static_box_sizer.Add(command_text, proportion=1, flag=wx.EXPAND)
        static_box_sizer.Add(command_combobox, proportion=1, flag=wx.EXPAND)

        button_sizer = self.CreateSeparatedButtonSizer(wx.OK | wx.CANCEL)

        v_sizer = wx.BoxSizer(wx.VERTICAL)
        v_sizer.Add(port_sizer)
        v_sizer.Add(static_box_sizer)
        v_sizer.AddSpacer(5)
        v_sizer.Add(button_sizer)

        padding_sizer = wx.BoxSizer(wx.VERTICAL)
        padding_sizer.Add(v_sizer, flag=wx.ALL, border=10)

        self.SetAutoLayout(True)
        self.SetSizerAndFit(padding_sizer)

        # TODOs
        # list control containing known controllers
        # browse button that allows you to choose a process
        # TextCtrl that allows you to specify arguments
        # Okay | Cancel buttons

    def onComboBox(self, event):
        print "Caught event: %s" % str(event)


##class ConfigurationWindow(wx.Frame):
##    def __init__(self, parent):
##        wx.Frame.__init__(self,
##                          parent=parent,
##                          title="Edit Configuration",
##                          style=wx.DEFAULT_FRAME_STYLE
##                          )
##
##        notebook = wx.Notebook(self)
##        notebook.AddPage(
##
##        self.notebook = notebook
##
##        self.Layout()

class Legend(wx.Frame):

    def __init__(self, parent, station_colormap, vehicle_colormap):
        wx.Frame.__init__(self, parent=parent, title='Legend')

        self.container = chaco.VPlotContainer()
        station_colorbar = self.make_colorbar('Number of Passengers at Station', station_colormap)
        vehicle_colorbar = self.make_colorbar('Number of Passengers in Vehicle', vehicle_colormap)
        self.container.add(station_colorbar)
        self.container.add(vehicle_colorbar)

        win = Window(self, component=self.container)
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(win.control, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.SetAutoLayout(True)
        self.Show(True)

    def make_colorbar(self, title, color_map):
        max_range = int(color_map.range.high)
        colorbar = chaco.ColorBar(
            index_mapper=chaco.LinearMapper(range=color_map.range),
            color_mapper=color_map,
            orientation='h',
            direction='normal',
            grid_visible=False,
            fill_padding = True,
            padding=(10, 10, 40, 20)) # left, right, top, bottom
        colorbar.overlays.pop() # remove the old axis

        labels = [str(x) for x in xrange(0, max_range+1)]
        labels[-1] += '+' # indicate that anything past the highest value will be the same color
        colorbar._axis = chaco.LabelAxis(component=colorbar,
                                         orientation='top',
                                         title=title,
                                         title_spacing = 20,
                                         positions = arange(0.5, max_range+1.5), #offset to the middle of each color
                                         labels = labels)
        colorbar.overlays.append(colorbar._axis)
        return colorbar

if __name__ == "__main__":
#    import psyco
#    psyco.full()
#    psyco.log()
#    psyco.profile(.03)

    gui_app = GUI_App(0)
    gui_app.MainLoop()
