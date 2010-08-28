"""2D Visualization of stations and vehicles while the sim is in progress."""

FIXED = 1
SPRING = 2

import math
import Queue

import numpy
from numpy import array
import wx
import SimPy.SimulationRT as SimPy
import enthought.enable.api as enable
import enthought.traits.api as traits
import enthought.chaco.api as chaco
import enthought.chaco.tools.api as tools
import enthought.traits.ui.api as ui
from enthought.kiva import CompiledPath
from enthought.chaco.scatter_markers import CustomMarker
from enthought.enable.events import MouseEvent

import common
import segment_plot
from pyprt.shared import utility

TIMER_ID = wx.NewId()

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

        self.zoom_tool = KeyboardSimpleZoom(plot, tool_mode='box', always_on=False)
        plot.overlays.append(self.zoom_tool)

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
            num_pax = v.get_pax_count()
            vel = v.get_vel()
            mph = vel * 2.237
            lv, dist_to_lv = v.find_leading_vehicle()
            pax_ids = ",".join(str(pax.ID) for pax in v.passengers)

            # Try to find the destination station based on the vehicle's path.
            # Somewhat hackish, may be broken easily.
            dest_station = None
            dest_seg = v._path[-1]
            for s in common.stations.itervalues():
                if dest_seg in s.track_segments:
                    dest_station = s
                    break
            dest_station_str = 'dest station: %d' % dest_station.ID if dest_station is not None else 'dest station: Unkwn'

            self._v_label = chaco.DataLabel(
                component=self.plot,
                data_point = point,
                lines = ['vID: %d' % v.ID,
                         'loc: %d' % v.loc.ID,
                         'speed: %.1f m/s (%.0f mph)' % (vel, mph),
                          dest_station_str,
                          'dist to lv: %.1f m (%.0f ft)' % (dist_to_lv, dist_to_lv*3.28),
                         'numPax: %d' % num_pax,
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

    #    object.edit_traits()
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
            num_pax = len(stat._passengers)
            now = SimPy.now()
            wait_times = [pax.wait_time for pax in stat._passengers]
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
                         'sID: %d' % stat.ID,
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
                    stat.edit_traits()
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
                pax_cnt.append(v.get_pax_count())
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
                if len(stat._passengers) != data[idx]:
                    data[idx] = len(stat._passengers)
                    changed = True
            if changed:
                self.queue.put( data )
            yield SimPy.hold, self, self.data_interval

class KeyboardSimpleZoom(tools.SimpleZoom):
    """Adds the ability to do a zoom without selecting an area or moving the
    cursor."""

    def zoom_in(self):
        """Zoom in, at the center of the plot."""
        # Fakes a MouseEvent with the mouse position in the middle of the plot.
        event = MouseEvent()
        event.mouse_wheel = 6
        c = self.component
        event.x = (c.x2 - c.x)/2.0 + c.x
        event.y = (c.y2 - c.y)/2.0 + c.y
        self.normal_mouse_wheel(event)

    def zoom_out(self):
        """Zoom out, at the center of the plot."""
        # Fakes a MouseEvent with the mouse position in the middle of the plot.
        event = MouseEvent()
        event.mouse_wheel = -6
        c = self.component
        event.x = (c.x2 - c.x)/2.0 + c.x
        event.y = (c.y2 - c.y)/2.0 + c.y
        self.normal_mouse_wheel(event)

class NoWritebackOnCloseHandler(ui.Handler):
    def close(self, info, is_ok):
        """A hack to prevent the (badly designed) default behavior of writing
        the view's data back to the model. They create a cache, allow it to
        potentially go stale, then write it back as though it were the
        authoratitive source!
        """
        info.ui._revert = {}
        return True