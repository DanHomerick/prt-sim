import threading

scenario_manager = None
config_manager = None

digraph = None
interface = None
stations = dict()
track_segments = dict()
vehicle_models = dict()
vehicles = dict()
passengers = dict()
delivered_pax = set()
wxApp = None
event_manager = None # Event Manager
sim_ended = False
vehicle_viz_data_collector = None
station_viz_data_collector = None

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

#def reset():
#    print "Clearing previous configuration" # temp debug
#    globals.digraph = None
#    globals.interface = None
#    globals.vehicle_viz_data_collector = None
#    globals.station_viz_data_collector = None
#    globals.track_segments = dict()
#    globals.vehicles = dict()
#    globals.passengers = dict()
#    globals.delivered_pax = set()
#    globals.wxApp = None
#    globals.event_manager = None
#    globals.sim_ended = False

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

# Divide resolution by two for rounding.
DIST_RES = 0.01
DIST_RND = 2
TIME_RES = 0.001
TIME_RND = 3

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

class InvalidSwitchID(MsgError):
    pass

class InvalidStationID(MsgError):
    pass

class InvalidPlatformID(MsgError):
    pass

class InvalidBerthID(MsgError):
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