"""2D Visualization using matplotlib"""

FIXED = 1
SPRING = 2

import math

import numpy as N
import networkx as NX
import matplotlib.pylab as P
from matplotlib.numerix import asarray
from matplotlib.collections import LineCollection
from matplotlib.colors import colorConverter
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
from matplotlib.backends.backend_wx import NavigationToolbar2Wx
import wx
import SimPy.SimulationRT as Sim

import globals
import config

TIMER_ID = wx.NewId()


class Visualizer(Sim.Process, wx.Frame):
    """Draws the track layout and animates the vehicles on it. Since it is
    dependent on the track, station, and vehicle data already being setup,
    it should be instantiated only after they are ready."""
    def __init__(self, data_interval):
        Sim.Process.__init__(self, name='Viz')
        wx.Frame.__init__(self, None, -1, "Test wxFigure", pos=(0, 0), size=(800, 600))


        # Interval (in sim time) between animation frames
        self.data_interval = data_interval
        self.empty_data = []
        self.full_data = []

        # vehicle drawing settings
        self.v_width = 10
        self.v_alpha = 1.0
        self.v_empty_color = ( colorConverter.to_rgba('g', self.v_alpha))
        self.v_full_color = ( colorConverter.to_rgba('r', self.v_alpha))

        # TODO: Choose best layout algo. Probably spring. Add length effect?
        # Calculate the node and edge positions in image coordinates
        # These should only need to change when the track layout changes
        layout = config.conf.get('Visualization', 'layout')
        if layout.upper() == 'SPRING':
            self.node_pos = NX.spring_layout(globals.DiGraph)
        elif layout.upper() == 'SHELL':
            self.node_pos = NX.shell_layout(globals.DiGraph)
        elif layout.upper() == 'SPECTRAL':
            self.node_pos = NX.spectral_layout(globals.DiGraph)
        elif layout.upper() == 'CIRCULAR':
            self.node_pos = NX.circular_layout(globals.DiGraph)
        elif layout.upper() == 'RANDOM':
            self.node_pos = NX.random_layout(globals.DiGraph)
        elif layout.upper() == 'FIXED':
            self.node_pos = self.fixed_layout(globals.DiGraph)
        else:
            raise Exception, 'Unknown node layout method'
        self.edge_pos = self.calc_edge_coords()

        # animation setup
        self.fig = P.gcf()
        self.ax = P.gca()     # get current axis
        self.canvas = FigureCanvasWxAgg(self, -1, self.fig)
        self.background = None
        self.empty_vehicle_collection = None
        self.full_vehicle_collection = None
        self._frame_num = 0
        wx.EVT_TIMER(self, TIMER_ID, self.update_drawing)

    def collect_data(self):
        """Periodically (in sim time) collects vehicle position data (in
        image coordinates) for a single frame of animation. Appends the
        coordinate data to self.data. This method should be called by
        Sim.activate.
        """
        while True:
            data = self.calc_vehicle_coords()
            self.empty_data.append(data[0])
            self.full_data.append(data[1])
            yield Sim.hold, self, self.data_interval

    def fixed_layout(G, dim=2):
        """ fixed layout based on node x,y"""
        vpos={}
        for v in globals.DiGraph.nodes():
            vpos[v]=N.array([v.x,v.y])
        return vpos

    def run_animation(self):
        """Displays a figure with the track, stations, and vehicles.
        Animates the vehicles, using the image coordinate data stored
        in self.data
        """

        t = wx.Timer(self, TIMER_ID)
        t.Start(200) # What's this do?





        # draw the track once
        NX.draw_networkx(globals.DiGraph, pos=self.node_pos, ax=self.ax)
        self.ax.axis('off')  # Don't draw the axis numbers or lines

        # add the vehicles
        empty_vehicle_coords = self.empty_data[self._frame_num]
        full_vehicle_coords = self.full_data[self._frame_num]
        self._frame_num += 1
        self.empty_vehicle_collection = LineCollection(empty_vehicle_coords,
                                                 colors=self.v_empty_color,
                                                 linewidth=(self.v_width,),
                                                 animated=True)
        self.full_vehicle_collection = LineCollection(full_vehicle_coords,
                                                 colors=self.v_full_color,
                                                 linewidth=(self.v_width,),
                                                 animated=True)

        self.ax.add_collection(self.empty_vehicle_collection)
        self.ax.add_collection(self.full_vehicle_collection)
        self.empty_vehicle_collection.set_zorder(10) # in front of everything else
        self.full_vehicle_collection.set_zorder(10)

        # draw the legend
        P.figlegend((self.empty_vehicle_collection, self.full_vehicle_collection),
                   ('Empty Vehicle', 'Full Vehicle'), # labels
                    'upper right') # fig position
        self.Show()
        wx.EVT_IDLE(wx.GetApp(), self.update_drawing)
        globals.wxApp.MainLoop()


        # TODO: change to timer based to run at some multiple of real-time
#        gobject.idle_add(self.update_drawing)
#        gobject.timeout_add_seconds(0.01)


    def update_drawing(self, evt):
        """Refreshes the drawing."""
        # save the background
        if self.background is None:
            self.background = self.canvas.copy_from_bbox(self.ax.bbox)

        # restore the clean slate background
        self.canvas.restore_region(self.background)

        # update the data
        try:
            empty_vehicle_coords = self.empty_data[self._frame_num]
            full_vehicle_coords = self.full_data[self._frame_num]
            self._frame_num += 1
        except IndexError:
            print "Done!"
            globals.wxApp.ExitMainLoop()
            return True


        self.empty_vehicle_collection.set_verts(empty_vehicle_coords)
        self.full_vehicle_collection.set_verts(full_vehicle_coords)

        # just draw the animated artists
        self.ax.draw_artist(self.empty_vehicle_collection)
        self.ax.draw_artist(self.full_vehicle_collection)

        # just redraw the axes rectangle
        self.canvas.blit(self.ax.bbox)
        wx.WakeUpIdle() # ensure that the idle event keeps firing

        return True

    def calc_edge_coords(self):
        """From the edgelist and node positions, return a list of tuples
        containing the edge objects and beginning and ending image
        coordinates for each edge:
        [edge_object, ( (x_start, y_start), (x_end, y_end) ), ... ]
        """
        edgelist = globals.DiGraph.edges(data=True)

        e_pos = [(data,
                     ((self.node_pos[src][0], self.node_pos[src][1]),
                      (self.node_pos[sink][0], self.node_pos[sink][1])))
                    for src, sink, data in edgelist]

        return e_pos


    def calc_vehicle_coords(self):
        """Vehicles are drawn as line segments. This function returns a list
        of tuples containing the start and end points for each vehicle, in
        image coordinates. This is a format suitable for input into a
        LineCollection using its set_verts() function:
        [((x_start, y_start), (x_end, y_end)), ... ]
        """

        # The vehicles' beginning and ending image coordinates
        empty_v_coords = []
        full_v_coords = []
        for edge, (src_pos, dst_pos) in self.edge_pos:
            x1, y1 = src_pos
            x2, y2 = dst_pos
            dx = x2 - x1 # x offset
            dy = y2 - y1 # y offset
            d = N.sqrt(dx**2 + dy**2) # length of edge
            if d == 0: # source and target at same position
                continue

            scale = d / edge.length

            for a_vehicle in edge.resource.activeQ:
                if dx == 0: # vertical edge
                    x_head = x2
                    y_head = a_vehicle.pos * scale + y1

                    x_tail = x2
                    y_tail = (a_vehicle.pos - a_vehicle.length) * scale + y1

                elif dy == 0: # horizontal edge
                    y_head = y2
                    x_head = a_vehicle.pos * scale * x1

                    y_tail = y2
                    x_tail = (a_vehicle.pos - a_vehicle.length) * scale + x1

                else:
                    theta = math.atan2(dy, dx)
                    x_head = a_vehicle.pos * scale * math.cos(theta) + x1
                    y_head = a_vehicle.pos * scale * math.sin(theta) + y1

                    x_tail = (a_vehicle.pos - a_vehicle.length) * scale * \
                             math.cos(theta) + x1
                    y_tail = (a_vehicle.pos - a_vehicle.length) * scale * \
                             math.sin(theta) + y1

                if a_vehicle.passenger:
                    full_v_coords.append(((x_head, y_head), (x_tail, y_tail)))
                else:
                    empty_v_coords.append(((x_head, y_head), (x_tail, y_tail)))

#        print "v_pos:", v_pos
        return empty_v_coords, full_v_coords
