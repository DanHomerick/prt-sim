from __future__ import division
import logging

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.table_column as ui_tc
from enthought.traits.ui.tabular_adapter import TabularAdapter
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import common
from layout import TrackSegment
from events import Passenger
from visual import NoWritebackOnCloseHandler
from pyprt.shared.utility import sec_to_hms

class BerthError(Exception):
    pass

class VehicleOutOfPositionError(BerthError):
    def __init__(self, vehicle, msg_id):
        self.vehicle = vehicle
        self.msg_id = msg_id

class VehicleFullError(BerthError):
    def __init__(self, pax, vehicle, msg_id):
        self.pax = pax
        self.vehicle = vehicle
        self.msg_id = msg_id

class PassengerNotAvailableError(BerthError):
    def __init__(self, pax, vehicle, msg_id):
        self.pax = pax
        self.vehicle = vehicle
        self.msg_id, = msg_id

class StorageError(Exception):
    pass

class VehicleNotAvailableError(StorageError):
    pass

class StorageFullError(StorageError):
    pass

class Berth(Sim.Process, traits.HasTraits):
    ID = traits.Int
    platform = traits.Instance('Platform')
    station = traits.Instance('Station')
    start_pos = traits.Float # The 'tail' end of the berth
    end_pos = traits.Float   # The 'nose' end of the berth
    unloading = traits.Bool(False)
    loading = traits.Bool(False)
    storage_entrance = traits.Bool(False)
    storage_exit = traits.Bool(False)

    DISEMBARK = "DISEMBARK"
    EMBARK = "EMBARK"
    ENTER_STORAGE = "ENTER_STORAGE"
    EXIT_STORAGE = "EXIT_STORAGE"

    _action = traits.Enum(None, DISEMBARK, EMBARK, ENTER_STORAGE, EXIT_STORAGE)
    _error_continue = traits.Bool(False)

##    traits_view =  ui.View(ui.HGroup(ui.Item(name='vehicle',
##                                             editor = ui.TextEditor()),
##                                     ui.Item('busy')))

    def __init__(self, ID, station, platform, start_pos, end_pos, unloading,
                 loading, storage_entrance, storage_exit):
        Sim.Process.__init__(self, name='berth_' + str(ID))
        traits.HasTraits.__init__(self)
        self.ID = ID
        self.station = station
        self.platform = platform
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.unloading = unloading
        self.loading = loading
        self.storage_entrance = storage_entrance
        self.storage_exit = storage_exit

        # Record keeping for statistics
##        self._occupied_times = [Sim.now(), self._vehicles[:]] # elements are (time, list_of_occupying_vehicle_refs)
        self._busy_times = [] # elements are: (time, busy_state)
        self._all_passengers = [] # record of all passengers, including those who have departed

        # Control flags/settings for the run loop
        self._busy = False # use the self._busy property to enable record gathering

        self._action = None
        self._fnc_args = None
        self._error_continue = False

    def __str__(self):
        return self.name

##    def is_empty(self):
##        """Returns True if the berth is not occupied by a vehicle."""
##        return False if self.vehicle else True

    def disembark(self, vehicle, passengers, cmd_msg, cmd_msg_id):
        """If ordering matters, note that passengers at the end of the list
        are serviced first."""
        assert not self._busy
        self._action = Berth.DISEMBARK
        self._fnc_args = (vehicle, passengers, cmd_msg, cmd_msg_id)
        if self.passive:
            Sim.reactivate(self, prior=True)

    def embark(self, vehicle, passengers, cmd_msg, cmd_msg_id):
        """If ordering matters, note that passengers at the end of the list
        are serviced first."""
        assert not self._busy
        self._action = Berth.EMBARK
        self._fnc_args = (vehicle, passengers, cmd_msg, cmd_msg_id)
        if self.passive:
            Sim.reactivate(self, prior=True)

    def enter_storage(self, vehicle, cmd_msg, cmd_msg_id):
        assert not self._busy
        self._action = Berth.ENTER_STORAGE
        self._fnc_args = (vehicle, cmd_msg, cmd_msg_id)
        if self.passive:
            Sim.reactivate(self, prior=True)

    def exit_storage(self, position, model_name, cmd_msg, cmd_msg_id):
        assert not self._busy
        self._action = Berth.EXIT_STORAGE
        self._fnc_args = (position, model_name, cmd_msg, cmd_msg_id)
        if self.passive:
            Sim.reactivate(self, prior=True)

    def get_busy(self):
        return self.__busy
    def set_busy(self, value):
        self._busy_times.append( (Sim.now(), value) )
        self.__busy = value
    _busy = property(get_busy, set_busy)

    def is_busy(self):
        return self.__busy

    def run(self):
        """ The main loop for the Berth."""
        # A Berth has four different tasks to accomplish but only one active loop.
        while True:
            try:
                if self._action is Berth.DISEMBARK:
                    for disembark_delay in self._do_disembark(*self._fnc_args):
                        yield Sim.hold, self, disembark_delay # Wait while passenger disembarks

                elif self._action is Berth.EMBARK:
                    for embark_delay in self._do_embark(*self._fnc_args):
                        yield Sim.hold, self, embark_delay

                elif self._action is Berth.ENTER_STORAGE:
                    for enter_delay in self._do_enter_storage(*self._fnc_args):
                        yield Sim.hold, self, enter_delay

                elif self._action is Berth.EXIT_STORAGE:
                    for exit_delay in self._do_exit_storage(*self._fnc_args):
                        yield Sim.hold, self, exit_delay

            except VehicleOutOfPositionError as err:
                nose_pos, tail_pos = err.vehicle.get_positions()
                logging.info("T=%4.3f Vehicle not in berth for attempted %s. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, DisembarkCmdId: %s, vNosePos: %s, vNoseLoc %s, vTailPos: %s, vTailLoc: %s, berth.start_pos: %s, berth.end_pos: %s",
                             Sim.now(), self._action, err.vehicle.ID, self.ID, self.platform.ID, self.station.ID, err.msg_id, nose_pos, err.vehicle.loc, tail_pos, err.vehicle.tail_loc, self.start_pos, self.end_pos)
                error_msg = api.SimMsgBodyInvalidId()
                error_msg.id_type = api.VEHICLE
                error_msg.msgID = err.msg_id
                error_msg.ID = err.vehicle.ID
                common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                self._busy = False

            except PassengerNotAvailableError as err:
                logging.info("T=%4.3f Passenger not available for attempted %s. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, DisembarkCmdId: %s, Passenger: %s",
                             Sim.now(), self._action, err.vehicle.ID, self.ID, self.platform.ID, self.station.ID, err.msg_id, err.pax.ID)
                error_msg = api.SimMsgBodyInvalidId()
                error_msg.msgID = err.msg_id
                error_msg.id_type = api.PASSENGER
                error_msg.ID = err.pax.ID
                common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                self._error_continue = True # process other passengers

            except VehicleFullError as err:
                logging.info("T=%4.3f Action %s failed since vehicle is at max passenger capacity. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, Passenger: %s",
                             Sim.now(), self._action, err.vehicle.ID, self.ID, self.platform.ID, self.station.ID, err.msg_id, err.pax.ID)
                error_msg = api.SimMsgBodyInvalidId()
                error_msg.msgID = err.msg_id
                error_msg.id_type = api.PASSENGER
                error_msg.ID = err.pax.ID
                common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                self._error_continue = True # process other passengers

            if not self._error_continue:
                # Reset state
                self._action = None
                self._fnc_args = None
                assert not self._busy
                yield Sim.passivate, self
            else:
                # Go through the loop again
                self._error_continue = False

    def _do_disembark(self, vehicle, passengers, cmd_msg, cmd_msg_id):
        self._busy = True
        while passengers:
            pax = passengers.pop()
            self._do_disembark_pax_start(pax, vehicle, cmd_msg_id)
            yield pax.unload_delay # Wait while passenger disembarks
            self._do_disembark_pax_finish(pax, vehicle, cmd_msg_id)

        self._busy = False

        # Notify controller that all passenger disembarkments are done.
        cmd_complete = api.SimCompletePassengersDisembark()
        cmd_complete.msgID = cmd_msg_id
        cmd_complete.cmd.CopyFrom(cmd_msg)
        cmd_complete.time = Sim.now()
        common.interface.send(api.SIM_COMPLETE_PASSENGERS_DISEMBARK,
                              cmd_complete)

    def _do_disembark_pax_start(self, pax, vehicle, cmd_msg_id):
        # Error if vehicle not parked in berth
        if not vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
            raise VehicleOutOfPositionError(vehicle, cmd_msg_id)

        # Error if pax not in the vehicle
        if pax not in vehicle.passengers:
            raise PassengerNotAvailableError(pax, vehicle, cmd_msg_id)

        # Notify controller that disembark of this passenger is starting
        start_msg = api.SimNotifyPassengerDisembarkStart()
        start_msg.vID = vehicle.ID
        start_msg.sID = self.station.ID
        start_msg.platformID = self.platform.ID
        start_msg.pID = pax.ID
        start_msg.berthID = self.ID
        start_msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_PASSENGER_DISEMBARK_START,
                              start_msg)

    def _do_disembark_pax_finish(self, pax, vehicle, cmd_msg_id):
        # Error if vehicle is not still parked in berth
        if not vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
            raise VehicleOutOfPositionError(vehicle, cmd_msg_id)

        # Move the passenger from the vehicle to the station
        vehicle.disembark(pax)
        pax.loc = self.station

        # Note if the passenger has arrived at final dest (may not be
        # the case with non-PRT systems)
        if self.station.ID == pax.dest_station.ID:
            pax.trip_end = Sim.now()
            pax.trip_success = True
            common.delivered_pax.add(pax)
            self.station._pax_arrivals_count += 1
            self.station._all_passengers.append(pax)
            logging.info("T=%4.3f %s delivered to platform %s in %s by %s (%d out of %d), disembarked in berth %s",
                         Sim.now(), pax, self.platform.ID, self.station.ID, vehicle.ID, vehicle.get_pax_count(), vehicle.max_pax_capacity,  self.ID)
        else:
            self.station.add_passenger(pax)
            self.station._arrivals_count += 1


        # Notify that disembark of this passenger is complete
        end_msg = api.SimNotifyPassengerDisembarkEnd()
        end_msg.vID = vehicle.ID
        end_msg.sID = self.station.ID
        end_msg.platformID = self.platform.ID
        end_msg.pID = pax.ID
        end_msg.berthID = self.ID
        end_msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_PASSENGER_DISEMBARK_END,
                              end_msg)


    def _do_embark(self, vehicle, passengers, cmd_msg, cmd_msg_id):
        self._busy = True
        while passengers:
            pax = passengers.pop()
            self._do_embark_pax_start(pax, vehicle, cmd_msg_id)
            yield pax.load_delay
            self._do_embark_pax_finish(pax, vehicle, cmd_msg_id)

        self._busy = False

        # Notify controller that all passenger embarkments are done.
        cmd_complete = api.SimCompletePassengersEmbark()
        cmd_complete.msgID = cmd_msg_id
        cmd_complete.cmd.CopyFrom(cmd_msg)
        cmd_complete.time = Sim.now()
        common.interface.send(api.SIM_COMPLETE_PASSENGERS_EMBARK,
                              cmd_complete)


    def _do_embark_pax_start(self, pax, vehicle, cmd_msg_id):
        # Error if vehicle not parked in berth
        if not vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
            raise VehicleOutOfPositionError(vehicle, cmd_msg_id)

        # Error if pax not at the station
        if pax not in self.station._passengers:
            raise PassengerNotAvailableError(pax, vehicle, cmd_msg_id)

        # Error if the vehicle is at full capacity
        if vehicle.get_pax_count() >= vehicle.max_pax_capacity:
            raise VehicleFullError(pax, vehicle, cmd_msg_id)

        # Notify controller that embark of this passenger is starting
        start_msg = api.SimNotifyPassengerEmbarkStart()
        start_msg.vID = vehicle.ID
        start_msg.sID = self.station.ID
        start_msg.platformID = self.platform.ID
        start_msg.pID = pax.ID
        start_msg.berthID = self.ID
        start_msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_PASSENGER_EMBARK_START,
                              start_msg)


    def _do_embark_pax_finish(self, pax, vehicle, cmd_msg_id):
        # Error if vehicle is not still parked in berth
        if not vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
            raise VehicleOutOfPositionError(vehicle, cmd_msg_id)

        # Move passenger's location to the vehicle
        vehicle.embark(pax)
        pax.loc = vehicle
        self.station._pax_departures_count += 1
        self.station.remove_passenger(pax)
        pax.trip_boarded = Sim.now()
        logging.info("T=%4.3f %s loaded into %s (%d out of %d) at station %s, platform %s, berth %s ",
                     Sim.now(), pax, vehicle.ID, vehicle.get_pax_count(), vehicle.max_pax_capacity,  self.station.ID, self.platform.ID, self.ID )

        # Notify that embark of this passenger is complete
        end_msg = api.SimNotifyPassengerEmbarkEnd()
        end_msg.vID = vehicle.ID
        end_msg.sID = self.station.ID
        end_msg.platformID = self.platform.ID
        end_msg.pID = pax.ID
        end_msg.berthID = self.ID
        end_msg.time = Sim.now()
        common.interface.send(api.SIM_NOTIFY_PASSENGER_EMBARK_END,
                              end_msg)

    def _do_enter_storage(self, vehicle, cmd_msg, cmd_msg_id):
        if not vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
            raise VehicleOutOfPositionError(vehicle, cmd_msg_id)

        storage = self.station._storage_dict[vehicle.model_name]
        storage._reserve_slot()
        self._busy = True
        yield self.station.storage_entrance_delay
        if not vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
            raise VehicleOutOfPositionError(vehicle, cmd_msg_id)
        storage._store_vehicle(vehicle)

        self._busy = False

        # Notify controller that vehicle entering storage is done.
        cmd_complete = api.SimCompleteStorageEnter()
        cmd_complete.msgID = cmd_msg_id
        cmd_complete.cmd.CopyFrom(cmd_msg)
        cmd_complete.time = Sim.now()
        common.interface.send(api.SIM_COMPLETE_STORAGE_ENTER,
                              cmd_complete)

    def _do_exit_storage(self, position, model_name, cmd_msg, cmd_msg_id):
        storage = self.station._storage_dict[model_name]
        storage._reserve_vehicle()
        self._busy = True
        yield self.station.storage_exit_delay
        vehicle = storage._request_vehicle(position, self.platform.track_segment)
        self._busy = False

        # Notify controller that vehicle exiting storage is done.
        cmd_complete = api.SimCompleteStorageExit()
        cmd_complete.msgID = cmd_msg_id
        cmd_complete.cmd.CopyFrom(cmd_msg)
        cmd_complete.time = Sim.now()
        vehicle.fill_VehicleStatus(cmd_complete.v_status)
        common.interface.send(api.SIM_COMPLETE_STORAGE_EXIT,
                              cmd_complete)


class Platform(traits.HasTraits):
    ID = traits.Int
    berths = traits.List(traits.Instance(Berth))
    track_segment = traits.Instance(TrackSegment)

    def __init__(self, ID, track_segment):
        traits.HasTraits.__init__(self)
        self.ID = ID
        self.berths = []
        self.track_segment = track_segment

##    def is_empty(self):
##        """True if no berths are occupied."""
##        empty = True
##        for berth in self.berths:
##            if not berth.is_empty():
##                empty = False
##                break
##        return empty


class Storage(object):
    """Vehicle storage for one model of vehicle at a station.

    When a vehicle is moved into Storage, it is given a new pos and location
    on a 'private' track segment. This ensures that vehicles always have
    a legitimate position and location, even when in storage.

    Has no 'public' facing methods. Expected to only be used by classes within
    this module.
    """

    def __init__(self, model_name, initial_supply, max_capacity):
        self.model_name = model_name
        self.max_capacity = max_capacity

        self._num_vehicles = initial_supply
        self._num_pending_entry = 0
        self._num_pending_exit = 0

        # Create a 'private' track segment that is used to store vehicles.
        nan, inf = float('nan'), float('inf')
        self._storage_track = TrackSegment(api.STORAGE_ID, nan, nan, nan, nan, inf, 0, [], [], model_name + '_storage')

    def _reserve_vehicle(self):
        """Mark a vehicle as unavailable. To be called at the time that the
        command to move a vehicle out of storage is received.
        """
        if self._num_vehicles - self._num_pending_exit > 0:
            self._num_pending_exit += 1
        else:
            raise VehicleNotAvailableError

    def _request_vehicle(self, pos, loc):
        """Request that a vehicle be moved to pos on loc.
        The effect takes place immediately. To simulate a delay, see reserve_vehicle.
        """
        v = None
        if len(self._storage_track.vehicles):
            v = self._storage_track.vehicles[0]
            v._move_to(pos, loc)
            v._operational_times.append( (Sim.now(), True) )

        else:
            # create a new vehicle
            v_id = max(vehicle.ID for vehicle in common.vehicles.itervalues()) + 1
            v = common.vehicle_models[self.model_name](
                    ID=v_id,
                    loc=loc,
                    position=pos,
                    vel=0)
            common.vehicles[v_id] = v
            common.vehicle_list.append(v)
            Sim.activate(v, v.ctrl_loop())

        self._num_pending_exit -= 1
        self._num_vehicles -= 1

        assert 0 <= self._num_vehicles - self._num_pending_exit <= self.max_capacity
        return v

    def _reserve_slot(self):
        """Mark a slot in the storage area as unavailable. To be called at the
        time the command to move a vehicle into storage is received.
        """
        if self._num_vehicles + self._num_pending_entry + 1 <= self.max_capacity:
            self._num_pending_entry += 1
        else:
            raise StorageFullError

    def _store_vehicle(self, vehicle):
        """Request that vehicle be moved into storage.
        The effect takes place immediately. To simulate a delay, see reserve_slot
        """
        assert vehicle.model_name == self.model_name
        assert abs(vehicle.vel) < 0.1 # loose sanity check, vehicle should be stopped.

        try:
            pos = self._storage_track.vehicles[0].pos + vehicle.length + 1 # arbitrary 1 meter spacing
        except IndexError:
            pos = vehicle.length + 1
        vehicle._move_to(pos, self._storage_track)
        vehicle._operational_times.append( (Sim.now(), False) )
        self._num_pending_entry -= 1
        self._num_vehicles += 1

        assert 0 <= self._num_pending_entry + self._num_vehicles <= self.max_capacity

class Station(traits.HasTraits):
    platforms = traits.List(traits.Instance(Platform))
    track_segments = traits.Set(traits.Instance(TrackSegment))

    # Passengers waiting at the station.
    _passengers = traits.List(traits.Instance(Passenger))

    traits_view = ui.View(ui.VGroup(
                           ui.Group(
                                ui.Label('Waiting Passengers'),
##                                ui.Item(name='_passengers',
##                                        show_label = False,
##                                        editor=Passenger.table_editor
##                                        ),
                                show_border = True)
                           ),
                       title='Station', # was: self.label
#                       scrollable = True,
                       resizable = True,
                       height = 700,
                       width = 470,
                       handler=NoWritebackOnCloseHandler()
                       )

    table_editor = ui.TableEditor(
        columns = [ui_tc.ObjectColumn(name='ID', label='ID', tooltip='Station ID'),
                   ui_tc.ObjectColumn(name='label', label='Label', tooltip='Non-unique identifier'),
                   ui_tc.ExpressionColumn(label='Current Pax', format='%d',
                                          expression='len(object._passengers)',
                                          tooltip='Number of passengers currently at station.')
                   # TODO: The rest...
                   ],
        deletable = False,
        editable=False,
        sortable = True,
        sort_model = False,
        auto_size = True,
        orientation = 'vertical',
        show_toolbar = True,
        reorderable = False,
        rows = 15,
        row_factory = traits.This)


    def __init__(self, ID, label, track_segments,
                 storage_entrance_delay, storage_exit_delay, storage_dict):

        traits.HasTraits.__init__(self)
        self.ID = ID
        self.label = label
        self.platforms = []
        self.track_segments = track_segments
        self.storage_entrance_delay = storage_entrance_delay
        self.storage_exit_delay = storage_exit_delay

        # Keyed by the VehicleModel name (string) with FIFO queues as the values.
        self._storage_dict = storage_dict

        self._pax_arrivals_count = 0
        self._pax_departures_count = 0
        self._pax_times = [(Sim.now(), len(self._passengers))] # elements are (time, num_pax)
        self._all_passengers = []

    def __str__(self):
        if self.label:
            return self.label
        else:
            return str(self.ID)

    def __hash__(self):
        return hash(self.ID)

    def __eq__(self, other):
        if not isinstance(other, Station):
            return False
        else:
            return self.ID == other.ID

    def __ne__(self, other):
        if not isinstance(other, Station):
            return True
        else:
            return self.ID != other.ID

    def __cmp__(self, other):
        return cmp(self.ID, other.ID)

    def startup(self):
        """Activates all the berths"""
        for platform in self.platforms:
            for berth in platform.berths:
                Sim.activate(berth, berth.run())

    def add_passenger(self, pax):
        """Add a passenger to this station."""
        assert pax not in self._passengers
        self._passengers.append(pax)
        self._all_passengers.append(pax)
        self._pax_times.append( (Sim.now(), len(self._passengers)) )

    def remove_passenger(self, pax):
        """Remove a passenger from this station, such as when they load into a
        vehicle, or when they storm off in disgust..."""
        self._passengers.remove(pax)
        self._pax_times.append( (Sim.now(), len(self._passengers)) )

    def get_num_passengers(self):
        return len(self._passengers)
    num_passengers = property(get_num_passengers)

    def all_pax_wait_times(self):
        """Returns a list of wait times for all passengers, not just the current ones."""
        times = []
        for pax in self._all_passengers:
            for start, end, loc in pax._wait_times:
                if loc is self:
                    if end is None:
                        times.append(Sim.now() - start)
                    else:
                        times.append(end - start)
        return times

    def curr_pax_wait_times(self):
        """Returns a list of wait times for passengers currently waiting in the
        station."""
        times = []
        for pax in self._passengers:
            for start, end, loc in pax._wait_times:
                if loc is self:
                    if end is None:
                        times.append(Sim.now() - start)
                    else:
                        times.append(end - start)
        return times

    def get_min_all_pax_wait(self):
        try:
            return min(self.all_pax_wait_times())
        except ValueError: # Empty sequence
            assert len(self.all_pax_wait_times()) == 0
            return 0
    min_all_pax_wait = property(get_min_all_pax_wait)

    def get_mean_all_pax_wait(self):
        try:
            wait_times = self.all_pax_wait_times()
            return sum(wait_times)/len(wait_times)
        except ZeroDivisionError:
            return 0
    mean_all_pax_wait = property(get_mean_all_pax_wait)

    def get_max_all_pax_wait(self):
        try:
            return max(self.all_pax_wait_times())
        except ValueError: # Empty sequence
            return 0
    max_all_pax_wait = property(get_max_all_pax_wait)

    def get_min_curr_pax_wait(self):
        try:
            return min(self.curr_pax_wait_times())
        except ValueError: # Empty sequence
            assert len(self.curr_pax_wait_times()) == 0
            return 0
    min_curr_pax_wait = property(get_min_curr_pax_wait)

    def get_mean_curr_pax_wait(self):
        try:
            wait_times = self.curr_pax_wait_times()
            return sum(wait_times)/len(wait_times)
        except ZeroDivisionError:
            return 0
    mean_curr_pax_wait = property(get_mean_curr_pax_wait)

    def get_max_curr_pax_wait(self):
        try:
            return max(self.curr_pax_wait_times())
        except ValueError: # Empty sequence
            return 0
    max_curr_pax_wait = property(get_max_curr_pax_wait)

class StationTabularAdapater(TabularAdapter):
    columns = [
        ('ID', 'ID'),
        ('Label', 'label'),
        ('Current # Pax', 'num_passengers'),
        ('# Departures', '_pax_departures_count'),
        ('# Arrivals', '_pax_arrivals_count'),
        ('Current Pax Min Wait', 'min_curr_pax_wait'),
        ('Current Pax Mean Wait', 'mean_curr_pax_wait'),
        ('Current Pax Max Wait', 'max_curr_pax_wait'),
        ('All Pax Min Wait', 'min_all_pax_wait'),
        ('All Pax Mean Wait', 'mean_all_pax_wait'),
        ('All Pax Max Wait', 'max_all_pax_wait')
    ]

    # Column widths, in pixels
    ID_width = traits.Float(40)

    # Tooltips
    ID_tooltip = traits.Constant('Unique integer station id')
    label_tooltip = traits.Constant('Optional station name')
    num_passengers_tooltip = traits.Constant('Current number of passengers waiting in station.')
    _pax_departures_count_tooltip = traits.Constant('Total number of passengers to depart from this station.')
    _pax_arrivals_count_tooltip = traits.Constant('Total number of passengers to arrive at this station.')
    min_curr_pax_wait_tooltip = traits.Constant('Minimum wait time for passengers currently in station')
    mean_curr_pax_wait_tooltip = traits.Constant('Mean wait time for passengers currently in station')
    max_curr_pax_wait_tooltip = traits.Constant('Maximum wait time for passengers currently in station')
    min_all_pax_wait_tooltip = traits.Constant('Minimum wait time for passengers in this station, over the course of the whole simulation')
    mean_all_pax_wait_tooltip = traits.Constant('Mean wait time for passengers in this station, over the course of the whole simulation')
    max_all_pax_wait_tooltip = traits.Constant('Maximum wait time for passengers in this station, over the course of the whole simulation')

    # Formatting expressions
    min_curr_pax_wait_text = traits.Property
    mean_curr_pax_wait_text = traits.Property
    max_curr_pax_wait_text = traits.Property
    min_all_pax_wait_text = traits.Property
    mean_all_pax_wait_text = traits.Property
    max_all_pax_wait_text = traits.Property

    def _get_min_curr_pax_wait_text(self):
        return sec_to_hms(self.item.min_curr_pax_wait)

    def _get_mean_curr_pax_wait_text(self):
        return sec_to_hms(self.item.mean_curr_pax_wait)

    def _get_max_curr_pax_wait_text(self):
        return sec_to_hms(self.item.max_curr_pax_wait)

    def _get_min_all_pax_wait_text(self):
        return sec_to_hms(self.item.min_all_pax_wait)

    def _get_mean_all_pax_wait_text(self):
        return sec_to_hms(self.item.mean_all_pax_wait)

    def _get_max_all_pax_wait_text(self):
        return sec_to_hms(self.item.max_all_pax_wait)
