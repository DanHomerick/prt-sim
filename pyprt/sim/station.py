from __future__ import division
import logging

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.table_column as ui_tc
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import common
from events import Passenger

class Berth(Sim.Process, traits.HasTraits):
    ID = traits.Int
    platform = traits.Instance('Platform')
    station = traits.Instance('Station')
    start_pos = traits.Float
    end_pos = traits.Float
    unloading = traits.Bool
    loading = traits.Bool

    _embark_pax = traits.List(traits.Instance('events.Passenger'))
    _embark_vehicle = traits.Instance('vehicle.BaseVehicle')
    _embark_cmd = traits.Instance(api.CtrlCmdPassengersEmbark)
    _embark_cmd_id = traits.Int(api.NONE_ID)

    _disembark_pax = traits.List(traits.Instance('events.Passenger'))
    _disembark_vehicle = traits.Instance('vehicle.BaseVehicle')
    _disembark_cmd = traits.Instance(api.CtrlCmdPassengersDisembark)
    _disembark_cmd_id = traits.Int(api.NONE_ID)

##    traits_view =  ui.View(ui.HGroup(ui.Item(name='vehicle',
##                                             editor = ui.TextEditor()),
##                                     ui.Item('busy')))

    def __init__(self, ID, station, platform, start_pos, end_pos, unloading, loading):
        Sim.Process.__init__(self, name='berth_' + str(ID))
        traits.HasTraits.__init__(self)
        self.ID = ID
        self.station = station
        self.platform = platform
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.unloading = unloading
        self.loading = loading

        self._vehicles = []

        # Record keeping for statistics
        self._occupied_times = [Sim.now(), self._vehicles[:]] # elements are (time, list_of_occupying_vehicle_refs)
        self._busy_times = [] # elements are: (time, busy_state)
        self._all_passengers = [] # record of all passengers, including those who have departed

        # Control flags/settings for the run loop
        self._busy = False # use the self._busy property to enable record gathering

    def __str__(self):
        return self.name

##    def is_empty(self):
##        """Returns True if the berth is not occupied by a vehicle."""
##        return False if self.vehicle else True

    def disembark(self, vehicle, passengers, cmd_msg, cmd_msg_id):
        """If ordering matters, note that passengers at the end of the list
        are serviced first."""
        self._disembark_pax = passengers
        self._disembark_vehicle = vehicle
        self._disembark_cmd = cmd_msg
        self._disembark_cmd_id = cmd_msg_id
        if self.passive:
            Sim.reactivate(self, prior=True)

    def embark(self, vehicle, passengers, cmd_msg, cmd_msg_id):
        """If ordering matters, note that passengers at the end of the list
        are serviced first."""
        self._embark_pax = passengers
        self._embark_vehicle = vehicle
        self._embark_cmd = cmd_msg
        self._embark_cmd_id = cmd_msg_id
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
        while True:
            # disembarking
            while self._disembark_pax:

                # Error if vehicle not parked in berth
                if not self._disembark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    nose_pos, tail_pos = self._disembark_vehicle.get_positions()
                    logging.info("T=%4.3f Vehicle not in berth upon attempted disembark. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, DisembarkCmdId: %s, vNosePos: %s, vNoseLoc %s, vTailPos: %s, vTailLoc: %s, berth.start_pos: %s, berth.end_pos: %s",
                                 Sim.now(), self._disembark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._disembark_cmd_id, nose_pos, self._disembark_vehicle.loc, tail_pos, self._disembark_vehicle.tail_loc, self.start_pos, self.end_pos)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._disembark_vehicle.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    break

                pax = self._disembark_pax.pop()

                # Error if pax not in the vehicle
                if pax not in self._disembark_vehicle.passengers:
                    logging.info("T=%4.3f Disembarking passenger not in vehicle. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, DisembarkCmdId: %s, Passenger: %s",
                                 Sim.now(), self._disembark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._disembark_cmd_id, pax.ID)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.PASSENGER
                    error_msg.ID = pax.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    continue # process other passengers

                self._busy = True

                # Notify controller that disembark of this passenger is starting
                dis_start_msg = api.SimNotifyPassengerDisembarkStart()
                dis_start_msg.vID = self._disembark_vehicle.ID
                dis_start_msg.sID = self.station.ID
                dis_start_msg.platformID = self.platform.ID
                dis_start_msg.pID = pax.ID
                dis_start_msg.berthID = self.ID
                dis_start_msg.time = Sim.now()
                common.interface.send(api.SIM_NOTIFY_PASSENGER_DISEMBARK_START,
                                       dis_start_msg)

                # Wait while passenger disembarks
                yield Sim.hold, self, pax.unload_delay

                # Error if vehicle is not still parked in berth
                if not self._disembark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._disembark_vehicle.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    break

                # Move the passenger from the vehicle to the station
                self._disembark_vehicle.disembark(pax)
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
                                 Sim.now(), pax, self.platform.ID, self.station.ID, self._disembark_vehicle.ID, self._disembark_vehicle.get_pax_count(), self._disembark_vehicle.max_pax_capacity,  self.ID)
                else:
                    self.station.add_passenger(pax)
                    self.station._arrivals_count += 1


                # Notify that disembark of this passenger is complete
                dis_end_msg = api.SimNotifyPassengerDisembarkEnd()
                dis_end_msg.vID = self._disembark_vehicle.ID
                dis_end_msg.sID = self.station.ID
                dis_end_msg.platformID = self.platform.ID
                dis_end_msg.pID = pax.ID
                dis_end_msg.berthID = self.ID
                dis_end_msg.time = Sim.now()
                common.interface.send(api.SIM_NOTIFY_PASSENGER_DISEMBARK_END,
                                       dis_end_msg)

            self._busy = False

            # Notify controller that passenger disembarkment is done.
            if self._disembark_cmd_id != api.NONE_ID:
                dis_cmd_complete = api.SimCompletePassengersDisembark()
                dis_cmd_complete.msgID = self._disembark_cmd_id
                dis_cmd_complete.cmd.CopyFrom(self._disembark_cmd)
                dis_cmd_complete.time = Sim.now()
                common.interface.send(api.SIM_COMPLETE_PASSENGERS_DISEMBARK,
                                       dis_cmd_complete)

                # Reset state
                self._disembark_pax = []
                self._disembark_vehicle = None
                self._disembark_cmd = None
                self._disembark_cmd_id = api.NONE_ID


            ### embarking ###
            while self._embark_pax:

                # Error if vehicle not parked in berth
                if not self._embark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    nose_pos, tail_pos = self._embark_vehicle.get_positions()
                    nose_loc = self._embark_vehicle.loc
                    tail_loc = self._embark_vehicle.tail_loc
                    logging.info("T=%4.3f Vehicle not in berth upon attempted embark. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, vNosePos: %s, vNoseLoc %s, vTailPos: %s, vTailLoc: %s, berth.start_pos: %s, berth.end_pos: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, nose_pos, nose_loc, tail_pos, tail_loc, self.start_pos, self.end_pos)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._embark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._embark_vehicle.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    break

                pax = self._embark_pax.pop()

                # Error if pax not at the station
                if pax not in self.station._passengers:
                    logging.info("T=%4.3f Embarking passenger not at station. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, Passenger: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, pax.ID)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._embark_cmd_id
                    error_msg.id_type = api.PASSENGER
                    error_msg.ID = pax.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    continue # process other passengers

                # Error if the vehicle is at full capacity
                if self._embark_vehicle.get_pax_count() >= self._embark_vehicle.max_pax_capacity:
                    logging.info("T=%4.3f Embarking passenger failed since vehicle is at max capacity. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, Passenger: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, pax.ID)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._embark_cmd_id
                    error_msg.id_type = api.PASSENGER
                    error_msg.ID = pax.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    continue

                self._busy = True

                # Notify controller that embark of this passenger is starting
                em_start_msg = api.SimNotifyPassengerEmbarkStart()
                em_start_msg.vID = self._embark_vehicle.ID
                em_start_msg.sID = self.station.ID
                em_start_msg.platformID = self.platform.ID
                em_start_msg.pID = pax.ID
                em_start_msg.berthID = self.ID
                em_start_msg.time = Sim.now()
                common.interface.send(api.SIM_NOTIFY_PASSENGER_EMBARK_START,
                                       em_start_msg)

                yield Sim.hold, self, pax.load_delay

                # Error if vehicle is not still parked in berth
                if not self._embark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._embark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._embark_vehicle.ID
                    common.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    logging.warning("T=%4.3f vehicle %s not parked in berth cannot embark passenger . Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, Passenger: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, pax.ID)
                    break

                # Move passenger's location to the vehicle
                self._embark_vehicle.embark(pax)
                pax.loc = self._embark_vehicle
                self.station._pax_departures_count += 1
                self.station.remove_passenger(pax)
                pax.trip_boarded = Sim.now()
                logging.info("T=%4.3f %s loaded into %s (%d out of %d) at station %s, platform %s, berth %s ",
                             Sim.now(), pax, self._embark_vehicle.ID, self._embark_vehicle.get_pax_count(), self._embark_vehicle.max_pax_capacity,  self.station.ID, self.platform.ID, self.ID )

                # Notify that embark of this passenger is complete
                em_end_msg = api.SimNotifyPassengerEmbarkEnd()
                em_end_msg.vID = self._embark_vehicle.ID
                em_end_msg.sID = self.station.ID
                em_end_msg.platformID = self.platform.ID
                em_end_msg.pID = pax.ID
                em_end_msg.berthID = self.ID
                em_end_msg.time = Sim.now()
                common.interface.send(api.SIM_NOTIFY_PASSENGER_EMBARK_END,
                                       em_end_msg)

            self._busy = False

            # Notify controller that passenger embarkment is done.
            if self._embark_cmd_id != api.NONE_ID:
                em_cmd_complete = api.SimCompletePassengersEmbark()
                em_cmd_complete.msgID = self._embark_cmd_id
                em_cmd_complete.cmd.CopyFrom(self._embark_cmd)
                em_cmd_complete.time = Sim.now()
                common.interface.send(api.SIM_COMPLETE_PASSENGERS_EMBARK,
                                       em_cmd_complete)

                # Reset state
                self._embark_pax = []
                self._embark_vehicle = None
                self._embark_cmd = None
                self._embark_cmd_id = api.NONE_ID


            assert not self._busy
            yield Sim.passivate, self

class Platform(traits.HasTraits):
    ID = traits.Int
    berths = traits.List(traits.Instance(Berth))
    track_segment = traits.Instance('layout.TrackSegment')

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

class Station(traits.HasTraits):
    platforms = traits.List(traits.Instance(Platform))
    track_segments = traits.Set(traits.Instance('layout.TrackSegment'))

    # Passengers waiting at the station.
    _passengers = traits.List(traits.Instance(Passenger))

    traits_view = ui.View(ui.VGroup(
                           ui.Group(
                                ui.Label('Waiting Passengers'),
                                ui.Item(name='_passengers',
                                        show_label = False,
                                        editor=Passenger.table_editor
                                        ),
                                show_border = True)
                           ),
                       title='Station', # was: self.label
#                       scrollable = True,
                       resizable = True,
                       height = 700,
                       width = 470
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


    def __init__(self, ID, label, track_segments):
        traits.HasTraits.__init__(self)
        self.ID = ID
        self.label = label
        self.platforms = []
        self.track_segments = track_segments

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
