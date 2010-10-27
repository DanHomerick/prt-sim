import threading

import SimPy.SimulationRT as Sim

scenario_manager = None
config_manager = None

digraph = None # set to a networkx.DiGraph object
interface = None # set to a comm.ControlInterface object
stations = dict()
platforms = dict()
track_segments = dict()
vehicle_models = dict()
vehicles = dict()
passengers = dict()
delivered_pax = set()
gui = None # set to a gui.GUI_App object
event_manager = None # Event Manager
sim_ended = False
vehicle_viz_data_collector = None
station_viz_data_collector = None
reports = None

air_density = None
wind_speed = None
wind_direction = None # Angle from East, in radians

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

trace = False
real_time = False  # A flag indicating that we're using the RT version of SimPy
errors = 0

#def reset():
#    common.digraph = None
#    common.interface = None
#    common.vehicle_viz_data_collector = None
#    common.station_viz_data_collector = None
#    common.track_segments = dict()
#    common.vehicles = dict()
#    common.passengers = dict()
#    common.delivered_pax = set()
#    common.gui = None
#    common.event_manager = None
#    common.sim_ended = False

class MsgIdWidget(object):
    """A thread-safe msgID counter. Increments counter whenever next_id is called."""
    def __init__(self):
        self._id = 0
        self._lock = threading.Lock()

    def next_id(self):
        self._lock.acquire()
        self._id += 1
        id_ = self._id
        self._lock.release()
        return id_

    def last_id(self):
        return self._id

msg_id_widget = MsgIdWidget()

class AlarmClock(Sim.Process):
    """A SimPy.Lib.Process that wakes up at time and executes the payload function.

    time: The (simulation) time that the alarm should go off at.
    payload: A function that executes once the alarm clock goes off.
    args: Any number of additional arguments. They are passed to payload.
    kwargs: Keyword args. Pass as **{'var_name':value}
"""
    def __init__(self, time, payload, *args, **kwargs):
        super(AlarmClock, self).__init__(name="AlarmClock")
        self.time = time
        self.payload = payload
        self.args = args
        self.kwargs = kwargs
        Sim.activate(self, self.ring())

    def ring(self):
        yield Sim.hold, self, self.time - Sim.now()
        self.payload(*self.args, **self.kwargs)

    @staticmethod
    def delayed_msg(msg):
        if msg:
            interface.receiveQ.put(msg)
        if interface.passive():
            Sim.reactivate(interface, prior=False)

# ------------ Exception classes -------------------
class MsgError(Exception):
    """General message errors"""

class InvalidDestID(MsgError):
    """The destination ID does not correspond to an object"""

class InvalidTime(MsgError):
    """A time that is dated before current time, or otherwise invalid."""

class InvalidMsgType(MsgError):
    """Msg type is not found in the api."""

class InvalidPosition(MsgError):
    pass

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

class MsgRangeError(MsgError):
    """Base class for message errors in which a value is outside of a valid range.

    Attributes:
        value -- the invalid value
        valid_range -- a 2 element sequence specifying the valid range
    """
    def __init__(self, value, valid_range):
        self.value = value
        self.valid_range = valid_range
    def __str__(self):
        return "value: %s is outside of the valid range: %s, %s" % \
               (self.value, self.valid_range[0], self.valid_range[1])

class InvalidJerk(MsgRangeError):
    pass

class InvalidAccel(MsgRangeError):
    pass

class InvalidVel(MsgRangeError):
    pass

class CollisionError(Exception):
    """A collision occured. Used as a base class for more specific exceptions."""

class StationFullError(CollisionError):
    """A collision occured because the entry berth for a station was occupied."""

class StationOvershootError(CollisionError):
    """A collision occured because a vehicle was travelling too fast to stop
    in the entry berth."""

class ScenarioError(Exception):
    def __init__(self, msg=""):
        self.msg = msg
    def __str__(self):
        return self.msg