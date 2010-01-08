import matplotlib
matplotlib.use('WXAgg')
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
from matplotlib.backends.backend_wx import NavigationToolbar2Wx


import sys
#import gtk, gobject
import pylab as p
import matplotlib.numerix as nx
import time
from matplotlib.collections import LineCollection


import wx

TIMER_ID = wx.NewId()

class PlotFigure(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, "Test wxFigure")
        self.fig = p.figure(1)
        self.ax = p.subplot(111)
        self.canvas = FigureCanvasWxAgg(self, -1, self.fig)
#        self.canvas = self.ax.figure.canvas
        self.background = None
        self.cnt = 0
        self.tstart = time.time()
        wx.EVT_TIMER(self, TIMER_ID, self.update_line)

    def init_plot(self):
        # create the initial line
        x = nx.arange(0,2*nx.pi,0.01)
        self.l_pos_a = [((0,0),(1,1)), ((0,1),(1,0))]
        self.l_pos_b = [((0,0),(0,1)), ((1,1),(1,0))]
        self.line_c = LineCollection(self.l_pos_a, animated=True)
        line, = p.plot(x, nx.sin(x), animated=False)
        self.ax.add_collection(self.line_c)

    def update_line(self, evt):
        # save the clean slate background -- everything but the animated line
        # is drawn and saved in the pixel buffer background
        if self.background is None:
            self.background = self.canvas.copy_from_bbox(self.ax.bbox)

        # restore the clean slate background
        self.canvas.restore_region(self.background)
        # update the data
    #    line.set_ydata(nx.sin(x+update_line.cnt/10.0))
        if (self.cnt/10) % 2 == 0:
            self.line_c.set_verts(self.l_pos_b)
        else:
            self.line_c.set_verts(self.l_pos_a)
        # just draw the animated artist
        self.ax.draw_artist(self.line_c)
        # just redraw the axes rectangle
        self.canvas.blit(self.ax.bbox)

        if self.cnt==50:
            # print the timing info and quit
            print 'FPS:' , self.cnt/(time.time()-self.tstart)
            sys.exit()

        self.cnt += 1

#        wx.WakeUpIdle()
        return True

if __name__ == '__main__':
    app = wx.PySimpleApp()
    frame = PlotFigure()
    frame.init_plot()

    t = wx.Timer(frame, TIMER_ID)
    t.Start(200) # What's this do?

    wx.EVT_IDLE(wx.GetApp(), frame.update_line)

    frame.Show()
    app.MainLoop()

#gobject.idle_add(update_line)

p.show()
