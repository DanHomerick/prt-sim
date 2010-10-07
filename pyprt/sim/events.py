"""Events, including passenger creation"""

import heapq

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
from enthought.traits.ui.tabular_adapter import TabularAdapter
import enthought.traits.ui.table_column as ui_tc


import common
import pyprt.shared.api_pb2 as api
from pyprt.shared.utility import sec_to_hms
import SimPy.SimulationRT as Sim
from visual import NoWritebackOnCloseHandler

class EventManager(Sim.Process):
    """Primarily envisioned to create passengers as time passes, based on an
    established list of passenger data. Uses the heapq module to keep the
    soonest event at the front of self.list.
    """
    def __init__(self):
        Sim.Process.__init__(self, name="EventManager")
        self.list = list()

    def add_events(self, evts):
        """PrtEvent objects should be contained in a iterable container."""
        self.list.extend(evts)
        heapq.heapify(self.list)
        # If adding events during the sim,
        if self.active() or Sim.now() > 0:
            Sim.reactivate(self, prior = True)


    def add_event(self, evt):
        """Add a single PrtEvent object"""
        heapq.heappush(self.list, evt)
        if self.active() or Sim.now() > 0:
            Sim.reactivate(self, prior = True)

    def spawn_events(self):
        while True:
            try:
                while self.list[0].time <= Sim.now():
                    evt = heapq.heappop(self.list)
                    if isinstance(evt, Passenger):
                        assert not common.passengers.get(evt.ID)
                        common.passengers[evt.ID] = evt # add to common dict

                        evt.loc = evt.src_station # set pax loc
                        evt.loc.add_passenger(evt)
                        msg = api.SimEventPassengerCreated()
                        evt.fill_PassengerStatus(msg.p_status)
                        msg.time = Sim.now()
                        common.interface.send(api.SIM_EVENT_PASSENGER_CREATED, msg)
                    else:
                        raise NotImplementedError

                if self.list[0].time > Sim.now():
                    yield Sim.hold, self, self.list[0].time - Sim.now()

            except IndexError:
                yield Sim.passivate, self # will be reactivated if new items added


    def clear_events(self):
        self.list = list()

class PrtEvent(traits.HasTraits):
    """A baseclass for events. Named PrtEvent to easily distinguish from other
    sorts of events, such as SimPy's SimEvents."""
    time  = traits.Float # Time that event starts or arrives
    ID    = traits.Int
    label = traits.Str

    # default view
    traits_view = ui.View(ui.Item(name='label'))

    def __init__(self, time, ID, label='', **tr):
        traits.HasTraits.__init__(self, **tr)
        self.time = time
        self.ID = ID
        self.label = (label if label else self.__class__.__name__ + str(ID))

    def __eq__(self, other):
        if isinstance(other, PrtEvent):
            return self.ID == other.ID
        else:
            return False

    def __cmp__(self, other):
        """Compare based on time (and ID for equality)"""
        if isinstance(other, PrtEvent):
            if self.ID == other.ID:
                return 0
            else:
                return cmp(self.time, other.time)
        elif isinstance(other, basestring): # compared to a string
            return cmp(self.label, other)

    def __str__(self):
        """Use custom label, or use class's name + ID.
        Will work for subclasses."""
        return self.label

    def __hash__(self):
        return hash(self.ID)

class Passenger(PrtEvent):
    """A passenger."""
    mass = traits.Int()
    _loc = traits.Either(traits.Instance('station.Station'),
                         traits.Instance('vehicle.BaseVehicle'), None)

    traits_view = ui.View(ui.Item(name='label'),
                          ui.Item(name='ID'),
                          ui.Item(name='loc'),
                          ui.Item(name='mass'), # in kg
                          ui.Item(name='trip_success'),
                          ui.Item(name='wait_time', format_func=sec_to_hms),
                          ui.Item(name='walk_time', format_func=sec_to_hms),
                          ui.Item(name='ride_time', format_func=sec_to_hms),
                          ui.Item(name='will_share'),
                          ui.Item(name='src_station'),
                          ui.Item(name='dest_station'),
                          ui.Item(name='load_delay', format_func=sec_to_hms),
                          ui.Item(name='unload_delay', format_func=sec_to_hms),
                          style='readonly',
                          handler=NoWritebackOnCloseHandler()
                          )

##    # Subset of passenger data in table format.
##    table_editor = ui.TableEditor(
##            columns = [ui_tc.ObjectColumn(name='ID', label='ID'),
##                       ui_tc.ObjectColumn(name='src_station', label='Origin'),
##                       ui_tc.ObjectColumn(name='dest_station', label='Destination'),
##                       ui_tc.ExpressionColumn(label='Waiting',
##                                              expression='sec_to_hms(object.wait_time)',
##                                              globals={'sec_to_hms':sec_to_hms},
##                                              tooltip='Time spent waiting'),
##                       ui_tc.ExpressionColumn(label='Riding',
##                                              expression='sec_to_hms(object.ride_time)',
##                                              globals={'sec_to_hms':sec_to_hms},
##                                              tooltip='Time spent riding'),
##                       ui_tc.ExpressionColumn(label='Walking',
##                                              expression='sec_to_hms(object.walk_time)',
##                                              globals={'sec_to_hms':sec_to_hms},
##                                              tooltip='Time spent walking'),
##                       ui_tc.ExpressionColumn(label='Total',
##                                              expression='sec_to_hms(object.total_time)',
##                                              globals={'sec_to_hms':sec_to_hms},
##                                              tooltip='Total time spent on trip'),
##                       ui_tc.ObjectColumn(name='trip_success', label='Success',
##                                          tooltip='Sucessfully reached destination'),
##                       ui_tc.ObjectColumn(name='loc', label='Current Location')
##                      ],
##            other_columns = [ui_tc.ObjectColumn(name='label', label='Label'),
##                       ui_tc.ObjectColumn(name='will_share', label='Will Share',
##                                          tooltip='Willing to share vehicle when destinations match'),
##                       ui_tc.ObjectColumn(name='load_delay', label='Load Delay',
##                                          tooltip='Time that passenger takes to embark'),
##                       ui_tc.ObjectColumn(name='unload_delay', label='Unload Delay',
##                                          tooltip='Time that passenger takes to disembark'),
##                       ui_tc.ObjectColumn(name='mass', label='Mass',
##                                          tooltip='Includes luggage (kg)')
##                       ],
##                       # more...
##            deletable = False,
##            editable=False,
##            sortable = True,
##            sort_model = False,
##            auto_size = True,
##            orientation = 'vertical',
##            show_toolbar = True,
##            reorderable = False,
##            rows = 15,
##            row_factory = traits.This)

    def __init__(self, time, ID, src_station, dest_station,
                 load_delay, unload_delay, will_share, mass):
        super(Passenger, self).__init__(time, ID)
        self.src_station = src_station
        self.dest_station = dest_station
        self.load_delay = load_delay
        self.unload_delay = unload_delay
        self.will_share = will_share  # Willing to share pod (if same dest)
        self.mass = mass
        self.trip_success = False
        self._loc = src_station

        # For the following, where start and end are times in seconds, with 0 being the start of the sim.
        self._wait_times = [[time, None, self._loc]]  # contains triples: [[start, end, loc], ...]
        self._walk_times = [] # containing pairs:  [[start, end], [start, end], ...]
        self._ride_times = [] # contains triples: [[start, end, vehicle], [start, end, vehicle], ...]

        self._start_time = time
        self._end_time = None

    @property
    def wait_time(self): # in seconds
        total = 0
        for start, end, loc in self._wait_times:
            if end is None:
                total += Sim.now() - start
            else:
                total += end - start
        return total

    @property
    def ride_time(self): # in seconds
        total = 0
        for start, end, vehicle in self._ride_times:
            if end is None:
                total += Sim.now() - start
            else:
                total += end - start
        return total

    @property
    def walk_time(self): # in seconds
        total = 0
        for start, end in self._walk_times:
            if end is None:
                total += Sim.now() - start
            else:
                total += end - start
        return total

    @property
    def total_time(self):
        if self._end_time is None:
            return Sim.now() - self._start_time
        else:
            return self._end_time - self._start_time

    def get_loc(self):
        return self._loc
    def set_loc(self, loc):
        """Changes the loc, and keeps track of how much time is spent in each
        mode of transit: waiting, riding, or walking."""
        ### Track time spent in each mode of transit ###
        if self._loc is None: # Was walking
            self._walk_times[-1][1] = Sim.now()
        elif hasattr(self._loc, 'vehicle_mass'): # was in vehicle
            self._ride_times[-1][1] = Sim.now()
        elif hasattr(self._loc, 'platforms'): # was at station
            self._wait_times[-1][1] = Sim.now()
        else:
            raise Exception("Unknown loc type")

        ### Note if trip is completed. ###
        if loc is self.dest_station:
            self._end_time = Sim.now()
            self.trip_success = True

        ### More time tracking ###
        if not self.trip_success:
            if loc is None: self._walk_times.append( [Sim.now(), None] )
            elif hasattr(loc, 'vehicle_mass'): self._ride_times.append( [Sim.now(), None, loc] ) # isinstance(loc, BaseVehicle)
            elif hasattr(loc, 'platforms'): self._wait_times.append( [Sim.now(), None, loc] ) # isinstance(loc, TrackSegment)
            else: raise Exception("Unknown loc type")

        self._loc = loc

    loc = property(get_loc, set_loc, doc="loc is expected to be a Station, a "
                   "Vehicle, or None (which indicates walking from one station "
                   "to another). Setting the loc has side-effects, see set_loc.")

    def walk(self, origin_station, dest_station, travel_time, cmd_msg, cmd_id):
        assert self._loc is origin_station
        assert travel_time >= 0
        assert isinstance(cmd_msg, api.CtrlCmdPassengerWalk)
        assert isinstance(cmd_id, int)
        self.loc = None
        common.AlarmClock(Sim.now() + travel_time,
                          self._post_walk,
                          dest_station, cmd_msg, cmd_id)

    def _post_walk(self, dest_station, cmd_msg, cmd_id):
        """Updates stats, changes location, and sends a SimCompletePassengerWalk
        message. To be called once the walk is complete."""
        assert self._loc is None
        self.loc = dest_station

        msg = api.SimCompletePassengerWalk()
        msg.msgID = cmd_id
        msg.cmd.CopyFrom(cmd_msg)
        msg.time = Sim.now()
        common.interface.send(api.SIM_COMPLETE_PASSENGER_WALK, msg)

    def fill_PassengerStatus(self, ps):
        ps.pID = self.ID
        # I'd much rather use isinstance checks, but circular imports are killing me
        if self._loc is None:
            ps.loc_type = api.WALKING
            ps.locID = api.NONE_ID
        elif hasattr(self._loc, 'vehicle_mass'): # a vehicle
            ps.loc_type = api.VEHICLE
            ps.locID = self._loc.ID
        elif hasattr(self._loc, 'platforms'): # a station
            ps.loc_type = api.STATION
            ps.locID = self._loc.ID
        else:
            raise Exception, "Unknown passenger location type: %s" % self._loc

        ps.src_stationID = self.src_station.ID
        ps.dest_stationID = self.dest_station.ID
        ps.creation_time = self._start_time
        ps.mass = self.mass
        ps.trip_success = self.trip_success

class PassengerTabularAdapter(TabularAdapter):
    """An adapter for table-based views of multiple passengers."""
    columns = [ ('ID', 'ID'),
                ('Origin', 'src_station'),
                ('Destination', 'dest_station'),
                ('Wait Time', 'wait_time'),
                ('Ride Time', 'ride_time'),
                ('Walk Time', 'walk_time'),
                ('Total Time', 'total_time'),
                ('Trip Success', 'trip_success'),
                ('Current Location', 'loc'),
                ('Load Delay', 'load_delay'),
                ('Unload Delay', 'unload_delay'),
                ('Mass', 'mass'),
                ('Will Share', 'will_share')
              ]

    ID_width = traits.Float(40)

    wait_time_text = traits.Property
    ride_time_text = traits.Property
    walk_time_text = traits.Property
    total_time_text = traits.Property
    load_delay_text = traits.Property
    unload_delay_text = traits.Property

    ID_tooltip = traits.Constant('Unique passenger identifier')
    src_station_tooltip = traits.Constant("ID or label for the passenger's origin station.")
    dest_station_tooltip = traits.Constant("ID or label for the passenger's destination station.")
    wait_time_tooltip = traits.Constant('Time spent waiting at a station')
    ride_time_tooltip = traits.Constant('Time spent riding in a vehicle')
    walk_time_tooltip = traits.Constant('Time spent walking to a station')
    total_time_tooltip = traits.Constant('Sum of time spent on trip')
    loc_tooltip = traits.Constant('ID or label for a station or vehicle')
    trip_success_tooltip = traits.Constant('Passenger has sucessfully reached its destination')
    load_delay_tooltip = traits.Constant('Time that passenger requires to enter a vehicle')
    unload_delay_tooltip = traits.Constant('Time that passenger requires to exit a vehicle')
    mass_tooltip = traits.Constant('Total passenger weight, including luggage (kg)')
    will_share_tooltip = traits.Constant('Whether the passenger will share a vehicle with another passenger if they have the same origin and destination stations.')

    def _get_wait_time_text(self):
        return sec_to_hms(self.item.wait_time)

    def _get_ride_time_text(self):
        return sec_to_hms(self.item.ride_time)

    def _get_walk_time_text(self):
        return sec_to_hms(self.item.walk_time)

    def _get_total_time_text(self):
        return sec_to_hms(self.item.total_time)

    def _get_load_delay_text(self):
        return sec_to_hms(self.item.load_delay)

    def _get_unload_delay_text(self):
        return sec_to_hms(self.item.unload_delay)

def to_millisec(sec):
    """Convert from floating point seconds to integer milliseconds, rounding."""
    return int(round(sec * 1000))
