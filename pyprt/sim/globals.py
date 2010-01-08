import threading

scenario_manager = None

DiGraph = None
Interface = None
Switches = dict()
Stations = dict()
TrackSegments = dict()
VehicleModels = dict()
Vehicles = dict()
Passengers = dict()
Delivered_Pax = set()
wxApp = None
EventM = None # Event Manager
SimEnded = False
Viz_VehicleDataCollector = None
Viz_StationDataCollector = None
ManualCtrl = False # No external controller. Only used by gui_visual

vehicle_data_queue = None
station_data_queue = None

# A list of references to the instances, sorted by ID.
vehicle_list = list()
station_list = list()
switch_list = list()

img_path = None
img_width = -1
img_height = -1
img_xbounds = None
img_ybounds = None

# set by scenario.make_vehicle_classes, used by gui.MainWindow.load_scenario
max_vehicle_pax_capacity = None 

trace = False
real_time = False  # A flag indicating that we're using the RT version of SimPy
errors = 0

def reset():
    print "Clearing previous configuration" # temp debug
    globals.DiGraph = None
    globals.Interface = None
    globals.Viz_VehicleDataCollector = None
    globals.Viz_StationDataCollector = None
    globals.TrackSegments = dict()
    globals.Vehicles = dict()
    globals.Passengers = dict()
    globals.Delivered_Pax = set()
    globals.wxApp = None
    globals.EventM = None
    globals.SimEnded = False

class MsgIdWidget(object):
    """A thread-safe msgID counter. Increments counter whenever next_id is called."""
    def __init__(self):
        self._id = 0
        self._lock = threading.Lock()

    def next_id(self):
        self._lock.acquire()
        self._id += 1
        id = self._id
        self._lock.release()
        return id

    def last_id(self):
        return self._id

msg_id_widget = MsgIdWidget()

# If assumed max possible speed is 100 m/s (~ 225 mph), then to have
# positional resolution of 1 cm indicates a temporal resolution of 1 / 10000
# Divide resolution by two for rounding.
DIST_RES = 0.005
DIST_RND = 3
TIME_RES = 0.00005
TIME_RND = 6

def dist_eql(a, b):
    """Include == test to capture float('inf') == float('inf')"""
    return (True if a == b or round(abs(a-b), DIST_RND) < DIST_RES else False)
def dist_ge(a, b):
    return (True if a > b or dist_eql(a,b) else False)
def dist_le(a, b):
    return (True if a < b or dist_eql(a,b) else False)
def dist_gt(a, b):
    return (True if a > b and not dist_eql(a,b) else False)
def dist_lt(a, b):
    return (True if a < b and not dist_eql(a,b) else False)


def time_eql(a, b):
    """Include == test to capture float('inf') == float('inf')"""
    return (True if a == b or round(abs(a-b), TIME_RND) < TIME_RES else False)
def time_ge(a, b):
    return (True if a > b or time_eql(a,b) else False)
def time_le(a, b):
    return (True if a < b or time_eql(a,b) else False)
def time_gt(a, b):
    return (True if a > b and not time_eql(a,b) else False)
def time_lt(a, b):
    return (True if a < b and not time_eql(a,b) else False)

# ------------ Exception classes -------------------
class MsgError(Exception):
    """General message errors"""

class InvalidDestID(MsgError):
    """The destination ID does not correspond to an object"""

class InvalidTimestamp(MsgError):
    """A timestamp that is dated before current time"""

class InvalidMsgType(MsgError):
    """Msg type is not found in the api."""

class InvalidTrackSegID(MsgError):
    pass

class InvalidWaypointID(MsgError):
    pass

class InvalidSwitchID(MsgError):
    pass

class InvalidMergeID(MsgError):
    pass

class InvalidStationID(MsgError):
    pass

class InvalidVehicleID(MsgError):
    pass

class InvalidPassengerID(MsgError):
    pass

class InvalidAccel(MsgError):
    """Magnitude of specified acceleration is too large for conditions.
    Note that setting emergency=True in the CtrlCmdVehicleSpeed command
    may increase the allowed accelerations."""

class InvalidDecel(MsgError):
    """Magnitude of specified deceleration is too large for conditions.
    Note that setting emergency=True in the CtrlCmdVehicleSpeed command
    may increase the allowed decelerations."""

class InvalidJerk(MsgError):
    """Magnitude of specified jerk is too large for conditions.
    Note that setting emergency=True in the CtrlCmdVehicleSpeed command
    may increase the allowed jerk."""

class CollisionError(Exception):
    """A collision occured. Used as a base class for more specific exceptions."""

class StationFullError(CollisionError):
    """A collision occured because the entry berth for a station was occupied."""

class StationOvershootError(CollisionError):
    """A collision occured because a vehicle was travelling too fast to stop
    in the entry berth."""

class ConfigError(Exception):
    """An error occurred while processing configuration files."""