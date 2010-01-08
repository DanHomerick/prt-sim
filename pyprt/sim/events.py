"""Events, including passenger creation"""

import heapq

import globals
import layout
import pyprt.shared.api_pb2 as api
import SimPy.SimulationRT as Sim

import enthought.traits.api as traits
import enthought.traits.ui.api as ui

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
        while self.list:
            assert self.list[0].time >= Sim.now()
            if self.list[0].time > Sim.now():
                yield Sim.hold, self, self.list[0].time - Sim.now()
                continue

            evt = heapq.heappop(self.list)
            if isinstance(evt, Passenger):
                assert not globals.Passengers.get(evt.ID)
                globals.Passengers[evt.ID] = evt # add to global dict

                evt.loc = evt.src_station # set pax loc
                evt.loc.add_passenger(evt)
                msg = api.SimEventPassengerCreated()
                evt.fill_PassengerStatus(msg.p_status)
                globals.Interface.send(api.SIM_EVENT_PASSENGER_CREATED, msg)

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

    def __cmp__(self, other):
        """Compare based on time (and ID for equality)"""
        if isinstance(other, PrtEvent):
            if self.time < other.time: return -1
            elif self.time == other.time and self.ID == other.ID: return 0
            else: return 1
        elif isinstance(other, basestring): # compared to a string
            return cmp(self.label, other)
        else:  # by ID for all other types
            return cmp(self.ID, other)




    def __str__(self):
        """Use custom label, or use class's name + ID.
        Will work for subclasses."""
        return self.label

    def __hash__(self):
        return self.ID.__hash__()

#    def __eq__(self, other):
#        return self.ID.__eq__(other)

class Passenger(PrtEvent):
    """A passenger."""
    weight = traits.Int()

    traits_view = ui.View(ui.Item(name='label'),
                          ui.Item(name='ID'),
                          ui.Item(name='loc'),
                          ui.Item(name='weight'), # in kg
                          ui.Item(name='trip_start'),
                          ui.Item(name='trip_boarded'),
                          ui.Item(name='trip_end'),
                          ui.Item(name='trip_success'),
                          ui.Item(name='will_share'),
                          ui.Item(name='src_station'),
                          ui.Item(name='dest_station'),
                          ui.Item(name='load_delay'),
                          ui.Item(name='unload_delay'),
                          )

    def __init__(self, time, ID, src_station, dest_station,
                 load_delay, unload_delay, will_share, weight):
        super(Passenger, self).__init__(time, ID)
        self.src_station = src_station
        self.dest_station = dest_station
        self.load_delay = load_delay
        self.unload_delay = unload_delay
        self.will_share = will_share  # Willing to share pod (if same dest)
        self.weight = weight
        self.loc = None

        self.trip_start = time
        self.trip_boarded = None
        self.trip_end = None
        self.trip_success = None

    @property
    def wait_time(self): # in seconds
        if self.trip_end or self.trip_boarded:
            return self.trip_boarded - self.trip_start
        else:
            return Sim.now() - self.trip_start

    @property
    def travel_time(self): # in seconds
        if self.trip_end:
            return self.trip_end - self.trip_boarded
        elif self.trip_boarded:
            return Sim.now() - self.trip_boarded
        else:
            return 0

    def fill_PassengerStatus(self, ps):
        ps.pID = self.ID
        if isinstance(self.loc, layout.Edge):
            lt = api.EDGE
        elif isinstance(self.loc, layout.Switch):
            lt = api.SWITCH
        elif isinstance(self.loc, layout.Station):
            lt = api.STATION
        elif isinstance(self.loc, layout.Vehicle):
            lt = api.VEHICLE
        elif self.loc is None:
            lt = api.NONE
        else:
            raise Exception, "Unknown passenger location type: %s" % self.loc
        ps.loc_type = lt
        ps.locID = self.loc.ID
        ps.src_stationID = self.src_station.ID
        ps.dest_stationID = self.dest_station.ID

        ps.wait_time = to_millisec(self.wait_time)
        ps.travel_time = to_millisec(self.travel_time)

        # In python, a bool is a tri-state (True, False, None)
        # In protobuf, need two bools.
        if self.trip_success is None:
            ps.trip_complete = False
        else:
            ps.trip_success = self.trip_success
            ps.trip_complete = True

def to_millisec(sec):
    """Convert from floating point seconds to integer milliseconds, rounding."""
    return int(round(sec * 1000))
