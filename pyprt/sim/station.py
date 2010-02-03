from __future__ import division
import logging

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import globals
from events import Passenger

class Berth(Sim.Process, traits.HasTraits):
    ID = traits.Int
    platform = traits.Instance('Platform')
    station = traits.Instance('Station')
    start_pos = traits.Float
    end_pos = traits.Float
    unloading = traits.Bool
    loading = traits.Bool

    _busy = traits.Bool

    _embark_pax = traits.List(traits.Instance('events.Passenger'))
    _embark_vehicle = traits.Instance('layout.BaseVehicle')
    _embark_cmd = traits.Instance(api.CtrlCmdPassengersEmbark)
    _embark_cmd_id = traits.Int(api.NONE_ID)

    _disembark_pax = traits.List(traits.Instance('events.Passenger'))
    _disembark_vehicle = traits.Instance('layout.BaseVehicle')
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

        # Control flags/settings for the run loop
        self._busy = False

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

    def is_busy(self):
        return self._busy

    def run(self):
        while True:
            # disembarking
            while self._disembark_pax:

                # Error if vehicle not parked in berth
                if not self._disembark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    nose_pos, nose_loc = self._disembark_vehicle.nose
                    tail_pos, tail_loc = self._disembark_vehicle.tail
                    logging.info("T=%4.3f Vehicle not in berth upon attempted disembark. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, DisembarkCmdId: %s, vNosePos: %s, vNoseLoc %s, vTailPos: %s, vTailLoc: %s, berth.start_pos: %s, berth.end_pos: %s",
                                 Sim.now(), self._disembark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._disembark_cmd_id, nose_pos, nose_loc, tail_pos, tail_loc, self.start_pos, self.end_pos)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._disembark_vehicle.ID
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
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
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    continue # process other passengers

                self._busy = True

                # Notify controller that disembark of this passenger is starting
                dis_start_msg = api.SimNotifyPassengerDisembarkStart()
                dis_start_msg.vID = self._disembark_vehicle.ID
                dis_start_msg.sID = self.station.ID
                dis_start_msg.platformID = self.platform.ID
                dis_start_msg.pID = pax.ID
                dis_start_msg.berthID = self.ID
                globals.interface.send(api.SIM_NOTIFY_PASSENGER_DISEMBARK_START,
                                       dis_start_msg)

                # Wait while passenger disembarks
                yield Sim.hold, self, pax.unload_delay

                # Error if vehicle is not still parked in berth
                if not self._disembark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._disembark_vehicle.ID
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    break

                # Move the passenger from the vehicle to the station
                self._disembark_vehicle.passengers.remove(pax)
                pax.loc = self.station

                # Note if the passenger has arrived at final dest (may not be
                # the case with non-PRT systems)
                if self.station.ID == pax.dest_station.ID:
                    pax.trip_end = Sim.now()
                    pax.trip_success = True
                    globals.delivered_pax.add(pax)
                    logging.info("T=%4.3f %s delivered to platform %s in %s by %s. disembarked in berth %s",
                                 Sim.now(), pax, self.platform.ID, self.station.ID, self._disembark_vehicle.ID, self.ID)
                else:
                    self.station.passengers.append(pax)

                # Notify that disembark of this passenger is complete
                dis_end_msg = api.SimNotifyPassengerDisembarkEnd()
                dis_end_msg.vID = self._disembark_vehicle.ID
                dis_end_msg.sID = self.station.ID
                dis_end_msg.platformID = self.platform.ID
                dis_end_msg.pID = pax.ID
                dis_end_msg.berthID = self.ID
                globals.interface.send(api.SIM_NOTIFY_PASSENGER_DISEMBARK_END,
                                       dis_end_msg)

            self._busy = False

            # Notify controller that passenger disembarkment is done.
            if self._disembark_cmd_id != api.NONE_ID:
                dis_cmd_complete = api.SimCompletePassengerDisembark()
                dis_cmd_complete.msgID = self._disembark_cmd_id
                dis_cmd_complete.cmd.CopyFrom(self._disembark_cmd)
                globals.interface.send(api.SIM_COMPLETE_PASSENGER_DISEMBARK,
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
                    nose_pos, nose_loc = self._embark_vehicle.nose
                    tail_pos, tail_loc = self._embark_vehicle.tail
                    logging.info("T=%4.3f Vehicle not in berth upon attempted embark. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, vNosePos: %s, vNoseLoc %s, vTailPos: %s, vTailLoc: %s, berth.start_pos: %s, berth.end_pos: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, nose_pos, nose_loc, tail_pos, tail_loc, self.start_pos, self.end_pos)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._embark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._embark_vehicle.ID
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    break

                pax = self._embark_pax.pop()

                # Error if pax not at the station
                if pax not in self.station.passengers:
                    logging.info("T=%4.3f Embarking passenger not at station. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, Passenger: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, pax.ID)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.PASSENGER
                    error_msg.ID = pax.ID
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    continue # process other passengers

                # Error if the vehicle is at full capacity
                if len(self._embark_vehicle.passengers) >= self._embark_vehicle.max_pax_capacity:
                    logging.info("T=%4.3f Embarking passenger failed since vehicle is at max capacity. Vehicle: %s, Berth: %s, Platform: %s, Station: %s, EmbarkCmdId: %s, Passenger: %s",
                                 Sim.now(), self._embark_vehicle.ID, self.ID, self.platform.ID, self.station.ID, self._embark_cmd_id, pax.ID)
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._disembark_cmd_id
                    error_msg.id_type = api.PASSENGER
                    error_msg.ID = pax.ID
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    continue

                self._busy = True

                # Notify controller that embark of this passenger is starting
                em_start_msg = api.SimNotifyPassengerEmbarkStart()
                em_start_msg.vID = self._embark_vehicle.ID
                em_start_msg.sID = self.station.ID
                em_start_msg.platformID = self.platform.ID
                em_start_msg.pID = pax.ID
                em_start_msg.berthID = self.ID
                globals.interface.send(api.SIM_NOTIFY_PASSENGER_EMBARK_START,
                                       em_start_msg)

                yield Sim.hold, self, pax.load_delay

                # Error if vehicle is not still parked in berth
                if not self._embark_vehicle.is_parked_between(self.start_pos, self.end_pos, self.platform.track_segment):
                    error_msg = api.SimMsgBodyInvalidId()
                    error_msg.msgID = self._embark_cmd_id
                    error_msg.id_type = api.VEHICLE
                    error_msg.ID = self._embark_vehicle.ID
                    globals.interface.send(api.SIM_MSG_BODY_INVALID_ID, error_msg)
                    break

                # Move passenger's location to the vehicle
                self._embark_vehicle.passengers.add(pax)
                pax.loc = self._embark_vehicle
                self.station.passengers.remove(pax)
                pax.trip_boarded = Sim.now()
                logging.info("T=%4.3f %s loaded into %s at station %s, platform %s, berth %s",
                             Sim.now(), pax, self._embark_vehicle.ID, self.station.ID, self.platform.ID, self.ID)

                # Notify that embark of this passenger is complete
                em_end_msg = api.SimNotifyPassengerEmbarkEnd()
                em_end_msg.vID = self._embark_vehicle.ID
                em_end_msg.sID = self.station.ID
                em_end_msg.platformID = self.platform.ID
                em_end_msg.pID = pax.ID
                em_end_msg.berthID = self.ID
                globals.interface.send(api.SIM_NOTIFY_PASSENGER_EMBARK_END,
                                       em_end_msg)

            self._busy = False

            # Notify controller that passenger embarkment is done.
            if self._embark_cmd_id != api.NONE_ID:
                em_cmd_complete = api.SimCompletePassengerEmbark()
                em_cmd_complete.msgID = self._embark_cmd_id
                em_cmd_complete.cmd.CopyFrom(self._embark_cmd)
                globals.interface.send(api.SIM_COMPLETE_PASSENGER_EMBARK,
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
    passengers = traits.List(traits.Instance(Passenger))

    traits_view = ui.View(ui.VGroup(
                           ui.Group(
                                ui.Label('Waiting Passengers'),
                                ui.Item(name='passengers',
                                        show_label = False,
                                        editor=Passenger.pax_table_editor
                                        ),
                                show_border = True)
                           ),
                       title='Station', # was: self.label
#                       scrollable = True,
                       resizable = True,
                       height = 700,
                       width = 470
                       )

    def __init__(self, ID, label, track_segments):
        traits.HasTraits.__init__(self)
        self.ID = ID
        self.label = label
        self.platforms = []
        self.track_segments = track_segments

    def __str__(self):
        return self.label

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
        assert pax not in self.passengers
        self.passengers.append(pax)
