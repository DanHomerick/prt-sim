#!/usr/bin/env python
from __future__ import division # always use floating point division, unless specified otherwise

# TODO: Disable some menu options until a config file is loaded.

import wx
import threading
import Queue # msg_box style inter-thread communication
import subprocess
import time
import logging
from ConfigParser import NoSectionError

import numpy
from numpy import array, inf
import math
import enthought.enable.api as enable
import enthought.traits.api as traits
import enthought.chaco.api as chaco
import enthought.traits.ui.api as ui
import enthought.chaco.tools.api as tools
from enthought.kiva import CompiledPath
from enthought.chaco.scatter_markers import CustomMarker
from enthought.chaco.default_colormaps import fix

# This Window object allows the plot to look like a generic Panel to WX.
from enthought.enable.wx_backend.api import Window

import common
common.real_time = True

import SimPy.SimulationRT as SimPy

from menubar_manager import MenuBarManager
import pyprt
import pyprt.shared.api_pb2 as api
import layout
import segment_plot
import station
import report
import pyprt.shared.utility as utility

class GUI_App(wx.App):
    def OnInit(self):
        wx.InitAllImageHandlers()
        main_window = MainWindow(None, -1, "")
        self.SetTopWindow(main_window)
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
        if scenario_path != None:
            self.load_scenario(scenario_path)

        if common.config_manager.get_start_controllers() == True:
            self.simmenu_connectcontroller_handler(None)

        common.reports = report.Reports(common.passengers, common.vehicles, common.stations)

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
        if filename:
            manager = common.scenario_manager
            manager.load_scenario(filename)

            # adjust the window size to match the image.
            self.SetSize(wx.Size(common.img_width, common.img_height)) # width, height
            my_size = self.GetSize()
            display_size = wx.GetDisplaySize()
##            if (my_size[0] > display_size[0] or my_size[1] > display_size[1]): # if too big
##                self.SetSize(wx.Size(common.img_width/2, common.img_height/2)) # use half-resolution

            # Create the Visualizer
            max_pax_capacity = max(v.max_pax_capacity for v in common.vehicles.itervalues())
            self.visualizer = Visualizer(common.digraph, max_pax_capacity)
            self.chaco_window.component = self.visualizer.plot

            # update what options are enabled in the menu
            self.menubar_manager.config_loaded()

        # else they hit 'cancel', do nothing

    def filemenu_open_scenario_handler(self, event): # wxGlade: MainWindow.<event_handler>
        """Load the scenario from file."""
        filename = wx.FileSelector(flags=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST,
              wildcard='xml files (*.xml)|*.xml|all files (*.*)|*.*',
              default_path='../tests/',
              default_filename='scenario.xml')
        self.load_scenario(filename)

    def filemenu_saveconfig_handler(self, event):
        print "Event handler `filemenu_saveconfig_handler' not implemented"
        event.Skip()

    def filemenu_savesnapshot_handler(self, event):
        if self.visualizer:
            filename = wx.FileSelector(flags=wx.FD_SAVE |  wx.FD_OVERWRITE_PROMPT,
                                       wildcard='png (*.png)|*.png|gif (*.gif)|*.gif|jpeg (*.jpg)|*.jpg|bitmap (*.bmp)|*.bmp|any (*.*)|*.*')
            print "Save snapshot filename:", filename
            try:
                self.visualizer.save_plot(filename)
            except Exception, e:
                dialog = wx.MessageDialog(self,
                          message='An error occurred while saving: ' + filename,
                          style=wx.OK)
                dialog.ShowModal()
        else:
            dialog = wx.MessageDialog(self,
                      message='A configuration file must be loaded first.',
                      style=wx.OK)
            dialog.ShowModal()

    def filemenu_exit_handler(self, event):
        """Quit. No save capability yet."""
        self.Close()

    def simmenu_startsim_handler(self, event):
        # Create a new thread for the sim
        end_time = common.config_manager.get_sim_end_time()
        sim_thread = threading.Thread(name='sim_thread',
                                      target=run_sim,
                                      args=[end_time, self.sim_end_callback])
        sim_thread.setDaemon(True)
        sim_thread.start()

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

    def simmenu_connectcontroller_handler(self, event): # wxGlade: MainWindow.<event_handler>
        if common.interface:
            TCP_port = common.config_manager.get_TCP_port()
##            # NumberEntryDialog has a bug that caps the default to a max of 100, always.
##            dialog = wx.NumberEntryDialog(self,
##                              message='Choose connection port (TCP)',
##                              prompt='Port:',
##                              caption='Connect External Controller...',
##                              value=TCP_port,
##                              min=2,
##                              max=65535)
##            if dialog.ShowModal() == wx.ID_OK:
##                TCP_port = dialog.GetValue()
##                # Modal is no good. Need a non-blocking dialog box. Also, style=wx.CANCEL gives an 'OK' button.
###                waiting_dialog = wx.MessageDialog(self,
###                                    message='Waiting for controller to connect on port: %s' % TCP_port,
###                                    style=wx.CANCEL)
###                waiting_dialog.ShowModal()
##                common.interface.connect(TCP_port)
            common.interface.setup_server_sockets(TCP_port=TCP_port)
            controllers = common.config_manager.get_controllers()
            for cmd in controllers:
                cmd = cmd.split()
                try:
                    ctrl_proc = subprocess.Popen(cmd,
                                                cwd=common.config_manager.get_config_dir())
                except OSError:
                    dialog = wx.MessageDialog(self,
                        message='Failed to open controller using command: %s' % cmd,
                        style=wx.OK)
                    dialog.ShowModal()
                    return

            common.interface.accept_connections( len(controllers) )

            # update what menu options are enabled
            self.menubar_manager.controllers_connected()

        else:
            dialog = wx.MessageDialog(self,
                      message='A configuration file must be loaded first.',
                      style=wx.OK)
            dialog.ShowModal()

    def open_port_handler(self, event):
        dialog = wx.NumberEntryDialog(self,
                          message="Debugging Tool.\nOpens the specified TCP port.\nThe sim will block until a controller\nconnects at the specified port.\n\nChoose connection port (TCP)",
                          prompt='Port:',
                          caption='Connect External Controller...',
                          value=common.config_manager.get_TCP_port(),
                          min=2,
                          max=65535)
        if dialog.ShowModal() == wx.ID_OK:
            port_num = dialog.GetValue()
            common.interface.setup_server_sockets(TCP_port=port_num)
            common.interface.accept_connections( 1 )
            # Sim blocks, then resumes once connection is made.
            self.menubar_manager.controllers_connected()

    def simmenu_stopsim_handler(self, event):
        print "Event handler `simmenu_stopsim_handler' not implemented"
        event.Skip()

    def simmenu_pause_handler(self, event):
        self.simmenu_speed(0)

    def simmenu_1x_handler(self, event):
        self.simmenu_speed(1)

    def simmenu_2x_handler(self, event):
        self.simmenu_speed(2)

    def simmenu_4x_handler(self, event):
        self.simmenu_speed(4)

    def simmenu_8x_handler(self, event):
        self.simmenu_speed(8)

    def simmenu_32x_handler(self, event):
        self.simmenu_speed(32)

    def simmenu_fast_handler(self, event):
        self.simmenu_speed(inf)

    def simmenu_speed(self, speed):
        try:
            SimPy.rtset(speed)
            print 'Speed set to', speed, 'X'
        except AttributeError:
            dialog = wx.MessageDialog(self,
                                    message='Start simulation first.',
                                    style=wx.OK)
            dialog.ShowModal()


    def viewmenu_vehicle_handler(self, event): # wxGlade: MaSimPyinWindow.<event_handler>
        labels = [str(v) for v in common.vehicle_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Vehicles', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            common.vehicle_list[i].configure_traits()

    def viewmenu_station_handler(self, event): # wxGlade: MainWindow.<event_handler>
        labels = [str(s) for s in common.station_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Stations', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            s = common.station_list[i]
            s.configure_traits(view=s.view)

    def viewmenu_switch_handler(self, event): # wxGlade: MainWindow.<event_handler>
        labels = [str(sw) for sw in common.switch_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Stations', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            common.switch_list[i].configure_traits()

    def viewmenu_track_handler(self, event): # wxGlade: MainWindow.<event_handler>
        print "Event handler `viewmenu_track_handler' not implemented"
        event.Skip()

    def viewmenu_passenger_handler(self, event):
        pax_list = sorted(common.passengers.itervalues())
        labels = [str(p) for p in pax_list]
        dialog = wx.MultiChoiceDialog(self, '', 'View Passengers', labels)
        dialog.ShowModal()
        selections = dialog.GetSelections() # indexes of the selections
        for i in selections:
            pax_list[i].configure_traits()

    def viewmenu_legend_handler(self, event):
        # use self as the parent, so that the Legend will close when the MainWindow is closed.
        legend = Legend(self, self.visualizer.station_colormap, self.visualizer.vehicle_colormap)

    def viewmenu_reports_handler(self, event):
        common.reports.display()

    def record_start_handler(self, event):
        # setup the timer
        try:
            record_timerId = wx.NewId()
            timer_interval = (1.0 / common.config_manager.get_fps()) * 1000 # in millisec
            self.record_timer = wx.Timer(self, record_timerId)
            self.Bind(wx.EVT_TIMER, self.record_handler, id=record_timerId)
        except ZeroDivisionError:
            dialog = wx.MessageDialog(self,
                      message="Target FPS in the config file is set to 0, so recording is not possible.",
                      style=wx.OK)
            dialog.ShowModal()
        except NoSectionError: # from ConfigParser
            dialog = wx.MessageDialog(self,
                      message="Config file is missing a 'Simulation' section.",
                      style=wx.OK)
            dialog.ShowModal()

        # setup the filename
        filename = wx.FileSelector(flags=wx.FD_SAVE |  wx.FD_OVERWRITE_PROMPT,
                                   wildcard='bitmap (*.bmp)|*.bmp|png (*.png)|*.png|gif (*.gif)|*.gif|jpeg (*.jpg)|*.jpg|any (*.*)|*.*')
        try:
            base, ext = filename.split('.')
            self._record_filename_base = base
            self._record_filename_ext = ext
            self._record_count = 0
        except ValueError:
            dialog = wx.MessageDialog(self,
                      message="Unable to use filename. I expect the path to have exactly one '.' in it, and can't handle other cases.",
                      style=wx.OK)
            dialog.ShowModal()
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
                dialog = wx.MessageDialog(self,
                          message="Saving the recording snapshot %s failed." % filename,
                          style=wx.OK)
                dialog.ShowModal()
        else:
            # stop recording
            self.record_stop_handler(event) # passes the wrong event type, but stop doesn't use the event anyways


    def close_handler(self, event):
        # TODO: Check if user wants to save config
        common.sim_ended = True

        if common.interface:
            common.interface.disconnect()

        for t in threading.enumerate():
            print t.getName()

        self.Destroy()

    def update_statusbar(self, evt):
        if not common.sim_ended:
            # calc some passenger stats
            pax_waiting = 0
            total_wait = 0
            max_wait = 0
            for stat in common.station_list: # collected in 'seconds'
                for pax in stat.passengers:
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
            for vehicle in common.vehicles.itervalues():
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

    def sim_end_callback(self):
        """For now just pops a window announcing that the sim is done."""
##        common.reports.display()

        common.reports.write(common.config_manager.get_results_file(), update=True)
        summary = str(common.reports.summary_report)
        dialog = wx.MessageDialog(self,
                  message='Simulation Complete.\n'+summary,
                  style=wx.OK)
        dialog.ShowModal()

# end of class MainWindow

class Visualizer(object):
    """An object that encapsulates most of the visualization aspects of the GUI.
    Contains a Chaco.plot object."""

    def __init__(self, graph, max_pax):
        self.plot_data = self.make_ArrayPlotData(graph)

        data_range = chaco.DataRange1D()
        data_range.set_bounds(0, 20+1)
        self.station_colormap = chaco.YlOrBr(data_range)
        self.station_colormap.steps = 20+2

        data_range = chaco.DataRange1D()
        data_range.set_bounds(0, max_pax+1)
        self.vehicle_colormap = chaco.YlGnBu(data_range)
        self.vehicle_colormap.steps = max_pax+2

        self.plot = self.make_plot(self.plot_data, self.station_colormap, self.vehicle_colormap)

    def make_ArrayPlotData(self, graph):
        """Returns a chaco.ArrayPlotData object containing data for the track.
        `graph` is a networkx DiGraph describing the track layout.
        """
        # Collect the x and y coordinates for all nodes, grouped by type
        stat_x = []
        stat_y = []
        stat_track_x = []
        stat_track_y = []
        wypt_x = []
        wypt_y = []
        merge_x = []
        merge_y = []
        merge_track_x = []
        merge_track_y = []
        switch_x = []
        switch_y = []
        switch_track_x = []
        switch_track_y = []

        # Determine placement of the Node markers
        # Note that it's important that they are loaded by the list ordering.
        # When mousing over, their ordering is used to recover the true object.
        for s in common.station_list:
            assert isinstance(s, station.Station)
            # find the station midpoint, and use it place a 'station' icon
            mid_platform_idx = len(s.platforms)//2
            mid_platform_ts = s.platforms[mid_platform_idx].track_segment
            stat_x.append( (mid_platform_ts.x_start + mid_platform_ts.x_end)/2.0 )
            stat_y.append( (mid_platform_ts.y_start + mid_platform_ts.y_end)/2.0 )
            for plat in s.platforms:
                stat_track_x.append( plat.track_segment.x_start )
                stat_track_x.append( plat.track_segment.x_end   )
                stat_track_y.append( plat.track_segment.y_start )
                stat_track_y.append( plat.track_segment.y_end   )

        for node in common.switch_list:
            switch_x.append( (node.x_start + node.x_end)/2.0 )
            switch_y.append( (node.y_start + node.y_end)/2.0 )
            switch_track_x.append( node.x_start )
            switch_track_x.append( node.x_end   )
            switch_track_y.append( node.y_start )
            switch_track_y.append( node.y_end   )

        # Collect the x and y coords for all edges
        edge_x = []
        edge_y = []
        for src, dest, edge in graph.edges_iter(data=True):
            edge_x.append(src.x_end)
            edge_x.append(dest.x_start)
            edge_y.append(src.y_end)
            edge_y.append(dest.y_start)

        # Create a mapping from names (str) to data (arrays)
        data = {'stat_x':array(stat_x), 'stat_y':array(stat_y),
                'stat_pax_cnt':array([0 for x in stat_x]), # 0 passengers per station until updated
                'stat_track_x':array(stat_track_x), 'stat_track_y':array(stat_track_y),
                'wypt_x':array(wypt_x), 'wypt_y':array(wypt_y),
                'merge_x':array(merge_x), 'merge_y':array(merge_y),
                'merge_track_x':array(merge_track_x), 'merge_track_y':array(merge_track_y),
                'switch_x':array(switch_x), 'switch_y':array(switch_y),
                'switch_track_x':array(switch_track_x), 'switch_track_y':array(switch_track_y),
                'edges_x':array(edge_x), 'edges_y':array(edge_y),
                'vehicle_x':array([]), 'vehicle_y':array([]),
                'v_pax_cnt':array([]) # number of passengers in vehicles
                }
        return chaco.ArrayPlotData(**data)


    def make_plot(self, plot_data, station_colormap, vehicle_colormap):
        """Generates a plot based on fixed node and edge positions."""

        # Create plots, using the data
        plot = chaco.Plot(plot_data)

        # Add the background img
        img = chaco.ImageData.fromfile(common.img_path)
        # note that I flip the image in the y-axis. Just using the 'origin' style doesn't result in the correct panning behavior.
        plot_data.set_data("imagedata", numpy.flipud(img.get_data()))
        img_plot = plot.img_plot("imagedata",  xbounds=common.img_xbounds, ybounds=common.img_ybounds)[0]

        ##### Causes an error on a some computers ####
        ## Create a custom vehicle marker
        #vehicle_marker = self.make_vehicle_marker(0, size=5)

        # Make the renderers
        stat_plot = plot.plot(('stat_x', 'stat_y', 'stat_pax_cnt'),
                              type='cmap_scatter',
                              color_mapper=station_colormap,
                              fill_alpha=1,
                              name='stat',
                              marker='circle',
                              marker_size=14,
                              bgcolor='transparent')[0]

        wypt_plot = plot.plot(('wypt_x', 'wypt_y'), type='scatter', name='wypt',
                  marker='circle', color='blue', marker_size=1, hide_grids=True,  bgcolor='transparent')[0]

        merge_plot = plot.plot(('merge_x', 'merge_y'), type='scatter', name='merge',
                  marker='inverted_triangle', color='blue', marker_size=5, hide_grids=True,  bgcolor='transparent')[0]

        switch_plot = plot.plot(('switch_x', 'switch_y'), type='scatter', name='switch',
                  marker='triangle', color='blue', marker_size=5, hide_grids=True,  bgcolor='transparent')[0]

        add_xy_plot('stat_track_x', 'stat_track_y', plot, segment_plot.SegmentPlot,
                    color='red', hide_grids=True,  bgcolor='transparent')

        add_xy_plot('merge_track_x', 'merge_track_y', plot, segment_plot.SegmentPlot, color='blue', hide_grids=True,  bgcolor='transparent')

        add_xy_plot('switch_track_x', 'switch_track_y', plot, segment_plot.SegmentPlot, color='blue', hide_grids=True,  bgcolor='transparent')

        add_xy_plot('edges_x', 'edges_y', plot, segment_plot.SegmentPlot, color='blue', hide_grids=True,  bgcolor='transparent')

        # The extra complexity of this one is because of a bug in plot.plot:
        #   The old colormapper is overwritten, rather than allowing multiple color_maps.
        # The following code duplicates, for a single case, the logic in plot.plot.
        imap = chaco.LinearMapper(range=plot.index_range,
                                stretch_data=plot.index_mapper.stretch_data)
        vmap = chaco.LinearMapper(range=plot.value_range,
                                stretch_data=plot.value_mapper.stretch_data)
        self.v_x_datasource = plot._get_or_create_datasource('vehicle_x')
        self.v_y_datasource = plot._get_or_create_datasource('vehicle_y')
        self.v_color_datasource = plot._get_or_create_datasource('v_pax_cnt')
        v_plot = chaco.ColormappedScatterPlot(index=self.v_x_datasource,
                                              index_mapper=imap,
                                              value=self.v_y_datasource,
                                              value_mapper=vmap,
                                              color_data=self.v_color_datasource,
                                              color_mapper=vehicle_colormap,
                                              orientation=plot.orientation,
                                              origin=plot.origin,
                                              fill_alpha=1,
                                              marker_size=5,
                                              hide_grids=True,
                                              bgcolor='transparent')
        plot.add(v_plot)

        plot.bgcolor = 'white' # 'transparent'
        plot.hide_grids=False # True
        plot.padding = 2

        plot.tools.append(tools.PanTool(plot))

        #drag_tool = PointDraggingTool(wypt_plot)
        #drag_tool.modifier_key = 'control'
        #wypt_plot.tools.append(drag_tool)

        plot.overlays.append(tools.SimpleZoom(plot, tool_mode='box', always_on=False))


        # See examples/basic/scatter_inspector.py
        # Add 'clickablility' for vehicles. Clicking brings up traits view.
        v_plot.tools.append(tools.ScatterInspector(v_plot))
        v_plot.tools[-1].selection_mode='single'
        v_plot.tools[-1].threshold = v_plot.marker_size # count a click within x pixels as on the point
        # Add 'clickablility' for stations. Clicking brings up traits view.
        stat_plot.tools.append(tools.ScatterInspector(stat_plot))
        stat_plot.tools[-1].selection_mode='single'
        stat_plot.tools[-1].threshold = stat_plot.marker_size

    ##    v_si_overlay = chaco.ScatterInspectorOverlay(v_plot,
    ##                    hover_color="red",
    ##                    hover_marker_size=6,
    ##                    selection_marker_size=6,
    ##                    selection_color="yellow",
    ##                    selection_outline_color="purple",
    ##                    selection_line_width=3)
    ##    v_plot.overlays.append(v_si_overlay)

        # The ScatterInspector will modify the metadata of the underlying
        # datasource, which will cause a trait event to fire.  So, we'll hook
        # up a listener to that event.
        plot.datasources['vehicle_x'].on_trait_change(self.vehicle_selection_handler,
                                                 name="metadata_changed")
        plot.datasources['stat_x'].on_trait_change(self.station_selection_handler,
                                                 name="metadata_changed")
        return plot

    def save_plot(self, filename):
        """Save an image of the plot. Does not catch any exceptions."""
        # Create a graphics context of the right size
        win_size = self.plot.outer_bounds
        plot_gc = chaco.PlotGraphicsContext(win_size)
        #plot_gc.set_fill_color("transparent")
        # Place the plot component into it
        plot_gc.render_component(self.plot)

        # Save out to the user supplied filename
        plot_gc.save(filename)

    def update_plot(self, evt):
        """Retreives data from the common.vehicle_data_queue and
        common.station_data_queue and updates the plot's datasources.
        """
        try:
            x, y, pax_cnt = common.vehicle_data_queue.get_nowait() # non-blocking
            self.v_x_datasource.set_data(x)
            self.v_y_datasource.set_data(y)
            self.v_color_datasource.set_data(pax_cnt)
##            self.plot.datasources['vehicle_x'].set_data(x)
##            self.plot.datasources['vehicle_y'].set_data(y)
##            self.plot.datasources['v_pax_cnt'].set_data(pax_cnt)
            common.vehicle_data_queue.task_done()
        except (Queue.Empty, AttributeError):
            pass

        try:
            pax_count = common.station_data_queue.get_nowait() # non-blocking
            self.plot.datasources['stat_pax_cnt'].set_data(pax_count)
            common.station_data_queue.task_done()
        except (Queue.Empty, AttributeError):
            pass

    def vehicle_selection_handler(self, object, name, new):
        """This handler method will get called every time the metadata of the
        datasourcse get updated.  We'll just dump out the
        contents of that metadata to screen.  In this case, it is the index
        of the point directly under the mouse, and the index of the last
        point the mouse clicked on (indicated by "selection").
        name is 'metadata changed'
        new is 'True'
        """
        metadata = object.metadata
        selected = []
        if "hover" in metadata:
#            print "Hover:", metadata["hover"][0]
            select = metadata['hover'][0]
            point = (self.v_x_datasource.get_data()[select], self.v_y_datasource.get_data()[select])
##            point = (self.plot_data.get_data('vehicle_x')[select],
##                     self.plot_data.get_data('vehicle_y')[select])
            v = common.vehicle_list[select]
            num_pax = v.passenger_count()
	    vel = v.get_vel()
	    mph = vel * 2.237
            pax_ids = ",".join(str(pax.ID) for pax in v.passengers)
            self._v_label = chaco.DataLabel(
                              component=self.plot,
                              data_point = point,
                              lines = ['ID: %d' % v.ID,
                                       'numPax: %d' % num_pax,
				       'speed: %4.1f m/s %4.0f mph' % (vel, mph),
                                       'paxIDs: ' + pax_ids]
                              )
            self.plot.overlays.append(self._v_label)
        else:
            # see note in station_selection_handler
            to_delete = []
            for idx, overlay in enumerate(self.plot.overlays):
                if isinstance(overlay, chaco.DataLabel):
                    to_delete.append(idx)
            for x in reversed(to_delete):
                del self.plot.overlays[x]

        if "selections" in metadata:
#            print metadata["selections"]
            try:
                for select in metadata["selections"]:
                    v = common.vehicle_list[select]
                    v.edit_traits()
                del metadata["selections"] # clear the selection list
            except IndexError:
                pass

    #    object.configure_traits()
    #    meta_x, meta_y = datasources[0].metadata, datasources[1].metadata
    #    print "Hover (x,y): (%s, %s)" % (meta_x["hover"], meta_y["hover"])
    #    if "selection" in meta_x or "selection" in meta_y:
    #        print "Selected (x,y): (%s, %s)" %  (meta_x["selection"], meta_y["selection"])


    def station_selection_handler(self, object, name, new):
        """This handler method will get called every time the metadata of the
        datasourcse get updated.  We'll just dump out the
        contents of that metadata to screen.  In this case, it is the index
        of the point directly under the mouse, and the index of the last
        point the mouse clicked on (indicated by "selection").
        name is 'metadata changed'
        new is 'True'
        """
        metadata = object.metadata
        selected = []
        if "hover" in metadata: # and not self.data_label_index:
#            print "Hover:", metadata["hover"][0]
            select = metadata['hover'][0]
            point = (self.plot_data.get_data('stat_x')[select],
                     self.plot_data.get_data('stat_y')[select])
#            print "Point:", point
            stat = common.station_list[select]
            stat_name = stat.label
            num_pax = len(stat.passengers)
            now = SimPy.now()
            wait_times = [pax.wait_time for pax in stat.passengers]
            if wait_times:
                max_wait = utility.sec_to_hms(max(wait_times))
                avg_wait = utility.sec_to_hms(sum(wait_times) / len(wait_times))
            else:
                max_wait = utility.sec_to_hms(0)
                avg_wait = utility.sec_to_hms(0)
#            print "stuff", stat_name, num_pax, max_wait, avg_wait
            self._stat_label = chaco.DataLabel(
                              component=self.plot,
                              data_point = point,
                              lines = ['%s' % stat_name,
                                       'ID: %d' % stat.ID,
                                       'PAX Waiting: %d' % num_pax,
                                       'max wait: %s' % max_wait,
                                       'avg wait: %s' % avg_wait]
                              )
            self.plot.overlays.append(self._stat_label)

        # Ahh, the hackery work of a hack. Suck.
        # An explanation: It seems like the metadata is being updated twice,
        # but only on the first time the mouse hovers over any station. Rather
        # than trying to figure out what's up, I'm just deleting all DataLabels
        # whenever the mouse moves. Better long term soln is probably to write
        # my own version of a scatter_inspector that handles the DataLabels
        # internally.
        else:
            to_delete = []
            for idx, overlay in enumerate(self.plot.overlays):
                if isinstance(overlay, chaco.DataLabel):
                    to_delete.append(idx)
            for x in reversed(to_delete):
                del self.plot.overlays[x]

        if "selections" in metadata:
#            print metadata["selections"]
            try:
                for select in metadata["selections"]:
                    stat = common.station_list[select]
                    stat.edit_traits() # old params: view=stat.view
                del metadata["selections"] # clear the selection list
            except IndexError:
                pass

    #### Causes an error on some computers ####
    #def make_vehicle_marker(self, angle, size):
    #    """Create custom markers for the vehicles"""
    #    path = CompiledPath()
    #    path.rotate_ctm(angle)
    #    path.rect(-size, -size, size*2, size*2)
    #    path.move_to(-size, size)
    #    path.line_to(-size/2, size * 2)
    #    path.line_to(size/2, size * 2)
    #    path.line_to(size, size)
    #    return path


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
                                         positions = numpy.arange(0.5, max_range+1.5), #offset to the middle of each color
                                         labels = labels)
        colorbar.overlays.append(colorbar._axis)
        return colorbar

class PointDraggingTool(tools.DragTool):

    component = traits.Instance(enable.Component)

    # The pixel distance from a point that the cursor is still considered
    # to be 'on' the point
    threshold = traits.Int(5)

    # The index of the point being dragged
    _drag_index = traits.Int(-1)

    # The original dataspace values of the index and value datasources
    # corresponding to _drag_index
    _orig_value = traits.Tuple

    def is_draggable(self, x, y):
        # Check to see if (x,y) are over one of the points in self.component
        if self._lookup_point(x, y) is not None:
            return True
        else:
            return False

    def normal_mouse_move(self, event):
        plot = self.component

        ndx = plot.map_index((event.x, event.y), self.threshold)
        if ndx is None:
            if plot.index.metadata.has_key('selections'):
                del plot.index.metadata['selections']
        else:
            plot.index.metadata['selections'] = [ndx]

        plot.invalidate_draw()
        plot.request_redraw()


    def drag_start(self, event):
        plot = self.component
        ndx = plot.map_index((event.x, event.y), self.threshold)
        if ndx is None:
            return
        self._drag_index = ndx
        self._orig_value = (plot.index.get_data()[ndx], plot.value.get_data()[ndx])

    def dragging(self, event):
        plot = self.component

        data_x, data_y = plot.map_data((event.x, event.y))

        plot.index._data[self._drag_index] = data_x
        plot.value._data[self._drag_index] = data_y
        plot.index.data_changed = True
        plot.value.data_changed = True
        plot.request_redraw()

    def drag_cancel(self, event):
        plot = self.component
        plot.index._data[self._drag_index] = self._orig_value[0]
        plot.value._data[self._drag_index] = self._orig_value[1]
        plot.index.data_changed = True
        plot.value.data_changed = True
        plot.request_redraw()

    def drag_end(self, event):
        plot = self.component
        if plot.index.metadata.has_key('selections'):
            del plot.index.metadata['selections']
        plot.invalidate_draw()
        plot.request_redraw()

    def _lookup_point(self, x, y):
        """ Finds the point closest to a screen point if it is within self.threshold

        Parameters
        ==========
        x : float
            screen x-coordinate
        y : float
            screen y-coordinate

        Returns
        =======
        (screen_x, screen_y, distance) of datapoint nearest to the input *(x,y)*.
        If no data points are within *self.threshold* of *(x,y)*, returns None.
        """

        if hasattr(self.component, 'get_closest_point'):
            # This is on BaseXYPlots
            return self.component.get_closest_point((x,y), threshold=self.threshold)

        return None


def run_sim(end_time, callback, *args):
    """Activate the SimPy Processes and run the simulation.

    end_time (float) : how many seconds to simulate
    callback (function) : A function called after completion of the simulation
    *args: Arguments for callback.
    """
    # initialize SimPy
    SimPy.initialize()

##    # Reorder the vehicles so that the queue will have proper FIFO ordering
##    # on the TrackSegment.
##    vehicle_list = common.vehicles.values()  # the Vehicle instances
##    def sort_by_vehicle_pos(a, b):
##        if a.loc is b.loc: # if a and b are both at the same location
##            return cmp(b.pos, a.pos) # sort by position, descending order
##        else:
##            return cmp(a.loc, b.loc) # else sort by place
##    vehicle_list.sort(cmp=sort_by_vehicle_pos)

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
            common.vehicle_viz_data_collector = VisDataCollector(data_interval=chaco_frame_interval,
                                                     queue=common.vehicle_data_queue,
                                                     chaco_interval=chaco_frame_interval)
            SimPy.activate(common.vehicle_viz_data_collector,
                         common.vehicle_viz_data_collector.collect_vehicle_data())

            common.station_viz_data_collector = VisDataCollector(data_interval=1.0, # don't need station updates < 1 sec
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

    # start the sim
    print end_time
    SimPy.simulate(until=end_time, real_time=True, rel_speed=1)
    print 'ended sim'
    common.sim_ended = True

    # notify the controller(s) that the simulation is finished
    end_msg = api.SimEnd()
    end_msg.sim_end_time = end_time
    common.interface.send(api.SIM_END, end_msg)

    # Disconnect from the controller(s)
    print "Disconnecting"
    common.interface.disconnect()

    callback(*args)

class VisDataCollector(SimPy.Process):
    """Draws the track layout and animates the vehicles on it. Since it is
    dependent on the track, station, and vehicle data already being setup,
    it should be instantiated only after they are ready.

    `queue` is a python Queue that connects this object with the Chaco data
       processing.
    `chaco_interval` is the interval at which Chaco repaints the graph (1/fps).
       It's treated as the minimal duration between data collections.
    """
    def __init__(self, data_interval, queue, chaco_interval):
        SimPy.Process.__init__(self, name='Viz')

        # Interval (in sim time) between animation frames
        self.data_interval = data_interval
        self.queue = queue
        self.chaco_interval = chaco_interval

    def collect_vehicle_data(self):
        """Periodically (in sim time) collects vehicle position data
        for a single frame of animation. Places the data in the
        communication queue. This method should be called by
        SimPy.activate. Making this a SimPy.Process ensures that all the
        data is collected simultaneously (in sim_time).
        """
        # Holds arrays for translating from vehicle pos to plot coordinates.
        pos2img = {}
        # precalculate coordinate transform matrixes
        for track_seg in common.track_segments.itervalues():
            try:
                dx = track_seg.x_end - track_seg.x_start
                dy = track_seg.y_end - track_seg.y_start
                angle = math.atan2(dy, dx)
                scale = math.sqrt(dx**2 + dy**2) / track_seg.length # scaling factor
                pos2img[track_seg] = array([[scale*math.cos(angle), track_seg.x_start],
                                       [scale*math.sin(angle), track_seg.y_start]])
            except ZeroDivisionError:
                continue

        chaco_interval = self.chaco_interval
        data_interval = self.data_interval
        empty_q_counts = 0
        while True:
            x_coords = []
            y_coords = []
            pax_cnt = []
            for v in common.vehicle_list:
                position = array([v.pos, 1])
                x, y = numpy.dot(pos2img[v.loc], position) # matrix mult
                x_coords.append(x)
                y_coords.append(y)
                pax_cnt.append(len(v.passengers))
            self.queue.put( (x_coords, y_coords, pax_cnt) )
            yield SimPy.hold, self, data_interval

            # manage the data collection rate to allow chaco to keep up
            qsize = self.queue.qsize()
            if qsize == 0:
                empty_q_counts += 1
            elif qsize == 1:
                empty_q_counts = 0
            elif qsize >= 2: # chaco's falling behind
                empty_q_counts = 0
                data_interval += data_interval / 5.0 # increase interval
#                print "gui_visual 888| Set data interval to:", data_interval

            # Rendering is keeping up with the data flow, and not yet at the
            # desired FPS. Reduce data collection interval by a tenth.
            if empty_q_counts > 10 and data_interval > chaco_interval:
                data_interval = max(chaco_interval, data_interval - data_interval / 10.0)
#                print "gui_visual 894| Set data interval to:", data_interval

    def collect_station_data(self):
        """Periodically (in sim time) collects station passenger data.
        Places the data in the communication queue. This method should be
        called by SimPy.activate.

        Polling is not the most efficient way to update, of course.
        """
        data = array([0 for x in common.station_list])
        while True:
            changed = False
            for idx, stat in enumerate(common.station_list):
                if len(stat.passengers) != data[idx]:
                    data[idx] = len(stat.passengers)
                    changed = True
            if changed:
                self.queue.put( data )
            yield SimPy.hold, self, self.data_interval

def add_xy_plot(index_name, value_name, plot, renderer_factory, name=None,
    origin=None, **kwds):
    """ Add a BaseXYPlot renderer subclass to a Plot.

    Parameters
    ----------
    index_name : str
        The name of the index datasource.
    value_name : str
        The name of the value datasource.
    plot : Plot
        The Plot to add the renderer to.
    renderer_factory : callable
        The callable that creates the plot.
    **kwds :
        Additional keywords to pass to the factory.

    By Robert Kern (provided on the Chaco discussion mailing list)
    """
    if name is None:
        name = plot._make_new_plot_name()
    if origin is None:
        origin = plot.default_origin
    index = plot._get_or_create_datasource(index_name)
    plot.index_range.add(index)
    value = plot._get_or_create_datasource(value_name)
    plot.value_range.add(value)

    if plot.index_scale == "linear":
        imap = chaco.LinearMapper(range=plot.index_range)
    else:
        imap = chaco.LogMapper(range=plot.index_range)
    if plot.value_scale == "linear":
        vmap = chaco.LinearMapper(range=plot.value_range)
    else:
        vmap = chaco.LogMapper(range=plot.value_range)

    renderer = renderer_factory(
        index = index,
        value = value,
        index_mapper = imap,
        value_mapper = vmap,
        orientation = plot.orientation,
        origin = origin,
        **kwds
    )
    plot.add(renderer)
    plot.plots[name] = [renderer]
    plot.invalidate_and_redraw()
    return plot.plots[name]

if __name__ == "__main__":
#    import psyco
#    psyco.full()
#    psyco.log()
#    psyco.profile(.03)

    gui_app = GUI_App(0)
    gui_app.MainLoop()
