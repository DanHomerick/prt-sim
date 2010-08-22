#!/usr/bin/env python
import wx

from numpy import inf

class MenuBarManager(object):
    def __init__(self, parent_window):
        self.parent_window = parent_window # parent window
        self.make_menubar()
        self.new()
        self.parent_window.SetMenuBar(self.menubar)

        self._current_speed_menu = None # Set to one of the speed wx.MenuItem attributes

    def make_menubar(self):
        """Creates menubar object that has been populated and bound to handlers."""
        self.menubar = wx.MenuBar()

        ### File Menu ###
        self.filemenu = wx.Menu()
        self.open_scenario = wx.MenuItem(self.filemenu, wx.ID_OPEN, "Open Scenario...", "", wx.ITEM_NORMAL)
#        self.saveconfig = wx.MenuItem(self.filemenu, wx.FD_SAVE, "Save Config...", "", wx.ITEM_NORMAL)
        self.savesnapshot = wx.MenuItem(self.filemenu, wx.NewId(), "Save Snapshot...", "", wx.ITEM_NORMAL)
        self.exit = wx.MenuItem(self.filemenu, wx.ID_EXIT, "", "", wx.ITEM_NORMAL)

        self.filemenu.AppendItem(self.open_scenario)
#        self.filemenu.AppendItem(self.saveconfig)
        self.filemenu.AppendItem(self.savesnapshot)
        self.filemenu.AppendItem(self.exit)
        self.menubar.Append(self.filemenu, "&File")

        ### Sim Menu ###
        self.simmenu = wx.Menu()
        self.connectcontroller = wx.MenuItem(self.simmenu, wx.NewId(), "Connect External Controller...\tCtrl-C", "", wx.ITEM_NORMAL)
        self.start_sim = wx.MenuItem(self.simmenu, wx.NewId(), "&Start Sim\tCtrl-S", "", wx.ITEM_NORMAL)
        self.stop_sim = wx.MenuItem(self.simmenu, wx.NewId(), "Stop Sim", "", wx.ITEM_NORMAL)
        self.pause_sim = wx.MenuItem(self.simmenu, wx.NewId(), "Pause Sim", "", wx.ITEM_CHECK)
        self.speed_halfx = wx.MenuItem(self.simmenu, wx.NewId(), "1/2x Speed", "", wx.ITEM_CHECK)
        self.speed1x = wx.MenuItem(self.simmenu, wx.NewId(), "1x Speed", "", wx.ITEM_CHECK)
        self.speed2x = wx.MenuItem(self.simmenu, wx.NewId(), "2x Speed", "", wx.ITEM_CHECK)
        self.speed4x = wx.MenuItem(self.simmenu, wx.NewId(), "4x Speed", "", wx.ITEM_CHECK)
        self.speed8x = wx.MenuItem(self.simmenu, wx.NewId(), "8x Speed", "", wx.ITEM_CHECK)
        self.speed32x = wx.MenuItem(self.simmenu, wx.NewId(), "32x Speed", "", wx.ITEM_CHECK)
        self.speed_fast = wx.MenuItem(self.simmenu, wx.NewId(), "Fastest", "", wx.ITEM_CHECK)

        self.simmenu.AppendItem(self.connectcontroller)
        self.simmenu.AppendSeparator()
        self.simmenu.AppendItem(self.start_sim)
        self.simmenu.AppendItem(self.stop_sim)
        self.simmenu.AppendSeparator()
        self.simmenu.AppendItem(self.pause_sim)
        self.simmenu.AppendItem(self.speed_halfx)
        self.simmenu.AppendItem(self.speed1x)
        self.simmenu.AppendItem(self.speed2x)
        self.simmenu.AppendItem(self.speed4x)
        self.simmenu.AppendItem(self.speed8x)
        self.simmenu.AppendItem(self.speed32x)
        self.simmenu.AppendItem(self.speed_fast)
        self.menubar.Append(self.simmenu, "&Simulation")

        ### View Menu ###
        self.viewmenu = wx.Menu()
        self.zoom_in = wx.MenuItem(self.viewmenu, wx.NewId(), "Zoom &In\t+", "", wx.ITEM_NORMAL)
        self.zoom_out = wx.MenuItem(self.viewmenu, wx.NewId(), "Zoom &Out\t-", "", wx.ITEM_NORMAL)
        self.vehicle = wx.MenuItem(self.viewmenu, wx.NewId(), "&Vehicle...", "", wx.ITEM_NORMAL)
        self.station = wx.MenuItem(self.viewmenu, wx.NewId(), "&Station...", "", wx.ITEM_NORMAL)
        self.switch = wx.MenuItem(self.viewmenu, wx.NewId(), "S&witch...", "", wx.ITEM_NORMAL)
        self.track = wx.MenuItem(self.viewmenu, wx.NewId(), "&Track...", "", wx.ITEM_NORMAL)
        self.passenger = wx.MenuItem(self.viewmenu, wx.ID_ANY, "&Passenger...", "", wx.ITEM_NORMAL)
        self.legend = wx.MenuItem(self.viewmenu, wx.ID_ANY, "&Legend")
        self.reports = wx.MenuItem(self.viewmenu, wx.ID_ANY, "&Reports")

        self.viewmenu.AppendItem(self.zoom_in)
        self.viewmenu.AppendItem(self.zoom_out)
        self.viewmenu.AppendSeparator()
        self.viewmenu.AppendItem(self.vehicle)
        self.viewmenu.AppendItem(self.station)
        self.viewmenu.AppendItem(self.switch)
        self.viewmenu.AppendItem(self.track)
        self.viewmenu.AppendItem(self.passenger)
        self.viewmenu.AppendSeparator()
        self.viewmenu.AppendItem(self.legend)
        self.viewmenu.AppendItem(self.reports)
        self.menubar.Append(self.viewmenu, "&View")

        ### Record Menu ###
        self.record = wx.Menu()
        self.record_start = wx.MenuItem(self.record, wx.NewId(), "&Start Recording")
        #self.record_pause = wx.MenuItem(self.record, wx.NewId(), "&Pause Recording")
        self.record_stop = wx.MenuItem(self.record, wx.NewId(), "S&top Recording")

        self.record.AppendItem(self.record_start)
        #self.record.AppendItem(self.record_pause)
        self.record.AppendItem(self.record_stop)
        self.menubar.Append(self.record, "&Record")

        ### Debug Menu ###
        self.debugmenu = wx.Menu()
        #self.debugmenu.AppendCheckItem(wx.NewId(), "Show &Tracks")
        self.open_port = wx.MenuItem(self.debugmenu, wx.NewId(), "&Open Port...", "", wx.ITEM_NORMAL)

        self.debugmenu.AppendItem(self.open_port)
        self.menubar.Append(self.debugmenu, "&Debug")

        ### Binding Handlers ###
        pw = self.parent_window
        pw.Bind(wx.EVT_MENU, pw.filemenu_open_scenario_handler, self.open_scenario)
#        pw.Bind(wx.EVT_MENU, pw.filemenu_saveconfig_handler, self.saveconfig)
        pw.Bind(wx.EVT_MENU, pw.filemenu_savesnapshot_handler, self.savesnapshot)
        pw.Bind(wx.EVT_MENU, pw.filemenu_exit_handler, self.exit)
        pw.Bind(wx.EVT_MENU, pw.simmenu_start_sim_handler, self.start_sim)
        pw.Bind(wx.EVT_MENU, pw.simmenu_stop_sim_handler, self.stop_sim)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.pause_sim)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed_halfx)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed1x)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed2x)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed4x)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed8x)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed32x)
        pw.Bind(wx.EVT_MENU, self.sim_speed_handler, self.speed_fast)
        pw.Bind(wx.EVT_MENU, pw.simmenu_connectcontroller_handler, self.connectcontroller)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_zoom_in_handler, self.zoom_in)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_zoom_out_handler, self.zoom_out)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_vehicle_handler, self.vehicle)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_station_handler, self.station)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_switch_handler, self.switch)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_track_handler, self.track)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_passenger_handler, self.passenger)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_legend_handler, self.legend)
        pw.Bind(wx.EVT_MENU, pw.viewmenu_reports_handler, self.reports)
        pw.Bind(wx.EVT_MENU, pw.record_start_handler, self.record_start)
        #pw.Bind(wx.EVT_MENU, pw.record_pause_handler, self.record_pause)
        pw.Bind(wx.EVT_MENU, pw.record_stop_handler, self.record_stop)
        pw.Bind(wx.EVT_MENU, pw.open_port_handler, self.open_port)
        pw.Bind(wx.EVT_CLOSE, pw.close_handler)


        ## Create Accelerator Table ##
        # Note: Though the wx.WXK_NUMPAD_ADD and wx.WXK_NUMPAD_SUBTRACT seem to
        #   work fine, the wx.WXK_ADD and wx.WXK_SUBTRACT don't. Using ord
        #   works to trigger from the main keys.
        accelerators = [ # Zoom
                        (wx.ACCEL_NORMAL, ord('+'), self.zoom_in.GetId()),
                        (wx.ACCEL_SHIFT, ord('+'), self.zoom_in.GetId()),
                        (wx.ACCEL_NORMAL, wx.WXK_NUMPAD_ADD, self.zoom_in.GetId()),
                        (wx.ACCEL_NORMAL, ord('-'), self.zoom_out.GetId()),
                        (wx.ACCEL_NORMAL, wx.WXK_NUMPAD_SUBTRACT, self.zoom_out.GetId()),

                         # Simulation
                        (wx.ACCEL_CMD, ord('c'), self.start_sim.GetId()),
                        (wx.ACCEL_CTRL, ord('c'), self.start_sim.GetId()),
                        (wx.ACCEL_CMD, ord('s'), self.start_sim.GetId()),
                        (wx.ACCEL_CTRL, ord('s'), self.start_sim.GetId())
                       ]
        accel_table = wx.AcceleratorTable(accelerators)
        self.parent_window.SetAcceleratorTable(accel_table)

    def new(self):
        """Disable all menus except for Open"""
        for menu in [self.savesnapshot, #self.saveconfig,# # file menu
                     self.connectcontroller, self.start_sim, self.stop_sim, self.pause_sim, # sim menu
                     self.speed_halfx, self.speed1x, self.speed2x, self.speed4x, self.speed8x, self.speed32x, self.speed_fast, # sim menu cont.
                     self.vehicle, self.station, self.switch, self.track, self.passenger, self.legend, self.reports, # view menu
                     self.record_start, self.record_stop]: # record menu
            menu.Enable(False)

    def config_loaded(self):
        """Enable connectcontroller and the view menu."""
        for menu in [self.savesnapshot, # file menu
                     self.connectcontroller, # sim menu
                     self.vehicle, self.station, self.switch, self.track, self.passenger, self.legend]: # view menu
            menu.Enable(True)

    def controllers_connected(self):
        """Allow the sim and the recording to start."""
        for menu in [self.start_sim, # sim menu
                     self.record_start]: # record menu
            menu.Enable(True)
        self.connectcontroller.Enable(False)

    def sim_started(self):
        """Allow the speed to be adjusted and the sim to be stopped or paused."""
        for menu in [self.stop_sim, self.pause_sim, # sim menu]
                     self.speed_halfx, self.speed1x, self.speed2x, self.speed4x, self.speed8x, self.speed32x, self.speed_fast,  # sim menu cont.
                     self.reports]: # view menu
            menu.Enable(True)
        self.speed1x.Check(True)
        self._current_speed_menu = self.speed1x
        self.start_sim.Enable(False)

    def record_started(self):
        """Allow the recording to be stopped."""
        self.record_start.Enable(False)
        self.record_stop.Enable(True)

    def record_stopped(self):
        self.record_start.Enable(True)
        self.record_stop.Enable(False)

    def sim_ended(self):
        self.new()

    def sim_speed_handler(self, event):
        """Responsible for updating which menuitem is checked, figuring out
        what numeric speed value to use, and passing the value to the parent
        window to handle.
        """
        self._current_speed_menu.Check(False)
        Id = event.GetId()
        if Id == self.pause_sim.Id:
            self.pause_sim.Check(True)
            self.parent_window.set_sim_speed(0) # FIXME: Doesn't allow sim to resume?
            self._current_speed_menu = self.pause_sim
        elif Id == self.speed_halfx.Id:
            self.speed_halfx.Check(True)
            self.parent_window.set_sim_speed(0.5)
            self._current_speed_menu = self.speed_halfx
        elif Id == self.speed1x.Id:
            self.speed1x.Check(True)
            self.parent_window.set_sim_speed(1)
            self._current_speed_menu = self.speed1x
        elif Id == self.speed2x.Id:
            self.speed2x.Check(True)
            self.parent_window.set_sim_speed(2)
            self._current_speed_menu = self.speed2x
        elif Id == self.speed4x.Id:
            self.speed4x.Check(True)
            self.parent_window.set_sim_speed(4)
            self._current_speed_menu = self.speed4x
        elif Id == self.speed8x.Id:
            self.speed8x.Check(True)
            self.parent_window.set_sim_speed(8)
            self._current_speed_menu = self.speed8x
        elif Id == self.speed32x.Id:
            self.speed32x.Check(True)
            self.parent_window.set_sim_speed(32)
            self._current_speed_menu = self.speed32x
        elif Id == self.speed_fast.Id:
            self.speed_fast.Check(True)
            self.parent_window.set_sim_speed(inf)
            self._current_speed_menu = self.speed_fast
        else:
            raise Exception("Unknown Id")