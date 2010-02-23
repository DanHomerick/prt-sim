import threading

import SimPy.SimulationRT as Sim

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
#    common.digraph = None
#    common.interface = None
#    common.vehicle_viz_data_collector = None
#    common.station_viz_data_collector = None
#    common.track_segments = dict()
#    common.vehicles = dict()
#    common.passengers = dict()
#    common.delivered_pax = set()
#    common.wxApp = None
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