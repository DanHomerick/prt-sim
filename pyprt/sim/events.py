"""Events, including passenger creation"""

import heapq

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.table_column as ui_tc

import globals
import pyprt.shared.api_pb2 as api
import SimPy.SimulationRT as Sim

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
            assert globals.time_ge(self.list[0].time, Sim.now())
            if self.list[0].time > Sim.now():
                yield Sim.hold, self, self.list[0].time - Sim.now()
                continue

            evt = heapq.heappop(self.list)
            if isinstance(evt, Passenger):
                assert not globals.passengers.get(evt.ID)
                globals.passengers[evt.ID] = evt # add to global dict

                evt.loc = evt.src_station # set pax loc
                evt.loc.add_passenger(evt)
                msg = api.SimEventPassengerCreated()
                evt.fill_PassengerStatus(msg.p_status)
                globals.interface.send(api.SIM_EVENT_PASSENGER_CREATED, msg)

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

    # Subset of passenger data in table format.
    pax_table_editor = ui.TableEditor(
                    columns = [ui_tc.ObjectColumn(name='label', label='Name'),
                               ui_tc.ObjectColumn(name='trip_start'),
                               ui_tc.ObjectColumn(name='dest_station', label='Destination'),
                               ui_tc.ObjectColumn(name='wait_time', label='Waiting (sec)', format="%.2f"),
                               ui_tc.ObjectColumn(name='will_share', label='Will Share'),
                               ui_tc.ObjectColumn(name='load_delay', label='Time to Board (sec)')],
                               # more...
                    deletable = False,
##                    sort_model = True,
                    auto_size = True,
                    orientation = 'vertical',
                    show_toolbar = True,
                    reorderable = True, # Does this affect the actual boarding order (think no...)
                    rows = 5,
                    row_factory = traits.This)

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
        # I'd much rather use isinstance checks, but circular imports are killing me
        if hasattr(self.loc, 'v_mass'):
            lt = api.VEHICLE
        elif hasattr(self.loc, 'platforms'):
            lt = api.STATION
        else:
            raise Exception, "Unknown passenger location type: %s" % self.loc

        ps.loc_type = lt
        ps.locID = self.loc.ID
        ps.src_stationID = self.src_station.ID
        ps.dest_stationID = self.dest_station.ID
        ps.creation_time = self.trip_start
        ps.wait_time = self.wait_time
        ps.travel_time = self.travel_time
        ps.weight = self.weight

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
