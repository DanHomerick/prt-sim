#!/usr/bin/env python
import wx

class MenuBarManager(object):
    def __init__(self, parent_window):
        self.parent_window = parent_window # parent window
        self.make_menubar()
        self.new()
        self.parent_window.SetMenuBar(self.menubar)

    def make_menubar(self):
        """Returns an menubar object that has been bound to handlers"""
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
        self.connectcontroller = wx.MenuItem(self.simmenu, wx.NewId(), "Connect External Controller...", "", wx.ITEM_NORMAL)
        self.startsim = wx.MenuItem(self.simmenu, wx.NewId(), "Start Sim", "", wx.ITEM_NORMAL)
        self.stopsim = wx.MenuItem(self.simmenu, wx.NewId(), "Stop Sim", "", wx.ITEM_NORMAL)
        self.pausesim = wx.MenuItem(self.simmenu, wx.NewId(), "Pause Sim", "", wx.ITEM_NORMAL)
        self.speed1x = wx.MenuItem(self.simmenu, wx.NewId(), "1x Speed", "", wx.ITEM_NORMAL)
        self.speed2x = wx.MenuItem(self.simmenu, wx.NewId(), "2x Speed", "", wx.ITEM_NORMAL)
        self.speed4x = wx.MenuItem(self.simmenu, wx.NewId(), "4x Speed", "", wx.ITEM_NORMAL)
        self.speed8x = wx.MenuItem(self.simmenu, wx.NewId(), "8x Speed", "", wx.ITEM_NORMAL)
        self.speed32x = wx.MenuItem(self.simmenu, wx.NewId(), "32x Speed", "", wx.ITEM_NORMAL)
        self.speedfast = wx.MenuItem(self.simmenu, wx.NewId(), "Fastest", "", wx.ITEM_NORMAL)

        self.simmenu.AppendItem(self.connectcontroller)
        self.simmenu.AppendSeparator()
        self.simmenu.AppendItem(self.startsim)
        self.simmenu.AppendItem(self.stopsim)
        self.simmenu.AppendSeparator()
        self.simmenu.AppendItem(self.pausesim)
        self.simmenu.AppendItem(self.speed1x)
        self.simmenu.AppendItem(self.speed2x)
        self.simmenu.AppendItem(self.speed4x)
        self.simmenu.AppendItem(self.speed8x)
        self.simmenu.AppendItem(self.speed32x)
        self.simmenu.AppendItem(self.speedfast)
        self.menubar.Append(self.simmenu, "&Simulation")

        ### View Menu ###
        self.viewmenu = wx.Menu()
        self.vehicle = wx.MenuItem(self.viewmenu, wx.NewId(), "&Vehicle...", "", wx.ITEM_NORMAL)
        self.station = wx.MenuItem(self.viewmenu, wx.NewId(), "&Station...", "", wx.ITEM_NORMAL)
        self.switch = wx.MenuItem(self.viewmenu, wx.NewId(), "S&witch...", "", wx.ITEM_NORMAL)
        self.track = wx.MenuItem(self.viewmenu, wx.NewId(), "&Track...", "", wx.ITEM_NORMAL)
        self.passenger = wx.MenuItem(self.viewmenu, wx.ID_ANY, "&Passenger...", "", wx.ITEM_NORMAL)
        self.legend = wx.MenuItem(self.viewmenu, wx.ID_ANY, "&Legend")
        self.reports = wx.MenuItem(self.viewmenu, wx.ID_ANY, "&Reports")

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
        pw.Bind(wx.EVT_MENU, pw.simmenu_startsim_handler, self.startsim)
        pw.Bind(wx.EVT_MENU, pw.simmenu_stopsim_handler, self.stopsim)
        pw.Bind(wx.EVT_MENU, pw.simmenu_pause_handler, self.pausesim)
        pw.Bind(wx.EVT_MENU, pw.simmenu_1x_handler, self.speed1x)
        pw.Bind(wx.EVT_MENU, pw.simmenu_2x_handler, self.speed2x)
        pw.Bind(wx.EVT_MENU, pw.simmenu_4x_handler, self.speed4x)
        pw.Bind(wx.EVT_MENU, pw.simmenu_8x_handler, self.speed8x)
        pw.Bind(wx.EVT_MENU, pw.simmenu_32x_handler, self.speed32x)
        pw.Bind(wx.EVT_MENU, pw.simmenu_fast_handler, self.speedfast)
        pw.Bind(wx.EVT_MENU, pw.simmenu_connectcontroller_handler, self.connectcontroller)
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

    def new(self):
        """Disable all menus except for Open"""
        for menu in [self.savesnapshot, #self.saveconfig,# # file menu
                     self.connectcontroller, self.startsim, self.stopsim, self.pausesim, # sim menu
                     self.speed1x, self.speed2x, self.speed4x, self.speed8x, self.speed32x, self.speedfast, # sim menu cont.
                     self.vehicle, self.station, self.switch, self.track, self.passenger, self.legend, # view menu
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
        for menu in [self.startsim, # sim menu
                     self.record_start]: # record menu
            menu.Enable(True)
        self.connectcontroller.Enable(False)

    def sim_started(self):
        """Allow the speed to be adjusted and the sim to be stopped or paused."""
        for menu in [self.stopsim, self.pausesim, # sim menu]
                     self.speed1x, self.speed2x, self.speed4x, self.speed8x, self.speed32x, self.speedfast]: # sim menu cont.
            menu.Enable(True)
        self.startsim.Enable(False)

    def record_started(self):
        """Allow the recording to be stopped."""
        self.record_start.Enable(False)
        self.record_stop.Enable(True)

    def record_stopped(self):
        self.record_start.Enable(True)
        self.record_stop.Enable(False)

    def sim_ended(self):
        self.new()