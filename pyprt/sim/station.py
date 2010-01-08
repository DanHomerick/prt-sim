# '/' is true division, '//' is truncating division
from __future__ import division
import logging

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.table_column as ui_tc
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import globals
import events


class Berth(Sim.Process, traits.HasTraits):
    label = traits.Str
    platform_index = traits.Int
    station_id = traits.Int
    vehicle = traits.Instance('Vehicle', None)

    _busy = traits.Bool
    _unload = traits.Bool
    _load = traits.Bool
    _msg_id = traits.Int
    _pax = traits.List(traits.Instance('events.Passenger'))

    traits_view =  ui.View(ui.HGroup(ui.Item(name='vehicle',
                                             editor = ui.TextEditor()),
                                     ui.Item('busy')))

    def __init__(self, label, station_id, vehicle, **tr):
        Sim.Process.__init__(self, name=label)
        traits.HasTraits.__init__(self, **tr)
        self.label = label
        self.station_id = station_id
        self.vehicle = vehicle

        # Control flags/settings for the run loop
        self._busy = False
        self._unload = False
        self._load   = False
        self._msg_id = api.NONE_ID
        self._pax = []

    def __str__(self):
        return str( (self.label, str(self.vehicle), str(self._busy)) )

    def is_empty(self):
        """Returns True if the berth is not occupied by a vehicle."""
        return False if self.vehicle else True

    def unload(self, msg_id, passengers):
        self._unload = True
        self._msg_id = msg_id
        self._pax = passengers
        if self.passive:
            Sim.reactivate(self, prior=True)

    def load(self, msg_id, passengers):
        self._load = True
        self._msg_id = msg_id
        self._pax = passengers
        if self.passive:
            Sim.reactivate(self, prior=True)

    def is_busy(self):
        return self._busy

    def run(self):
        station = globals.stations[self.station_id]
        while True:
            # Unloading
            if self._unload:
                for pax in reversed(self.vehicle.passengers):
                    self._unload = False
                    self._busy = True
                    yield Sim.hold, self, pax.unload_delay
                    del self.vehicle.passengers[-1] # pax that left vehicle
                    pax.loc = self.station
                    pax.trip_end = Sim.now()
                    if self.station_id == pax.dest_station.ID:
                        pax.trip_success = True
                        globals.Delivered_Pax.add(pax)
                        logging.info("T=%4.3f %s delivered to %s by %s. Unloaded in berth %s",
                                      Sim.now(), pax, self.station_id, self.vehicle, self.label)
                        d_msg = api.SimNotifyPassengerDelivered()
                        d_msg.vID = self.vehicle.ID
                        d_msg.sID = self.station_id
                        d_msg.pID = pax.ID
                        globals.Interface.send(api.SIM_NOTIFY_PASSENGER_DELIVERED,
                                              d_msg)
                    else:
                        pax.trip_success = False
                        logging.error("T=%4.3f %s misdelivered to %s by %s. Actual Dest: %s. Unloaded in berth %s",
                                      Sim.now(), pax, self.station_id, self.vehicle,
                                      globals.Stations[pax.dest_station.ID], self.label)

                        md_msg = api.SimNotifyPassengerMisdelivered()
                        md_msg.vID = self.vehicle.ID
                        md_msg.sID = self.station_id
                        md_msg.pID = pax.ID
                        globals.Interface.send(api.SIM_NOTIFY_PASSENGER_MISDELIVERED,
                                              md_msg)
                self._busy = False

                if station.passive():
                    Sim.reactivate(station, prior = True)

            # Loading
            elif self._load:
                for pax in self._pax:
                    self._load = False
                    self._busy = True
                    s_notify = api.SimNotifyPassengerLoadStart()
                    s_notify.vID = self.vehicle.ID
                    s_notify.sID = self.station_id
                    s_notify.pID = pax.ID
                    globals.Interface.send(api.SIM_NOTIFY_PASSENGER_LOAD_START,
                                          s_notify)
                    yield Sim.hold, self, pax.load_delay
                    self.vehicle.passengers.append(pax)
                    pax.trip_boarded = Sim.now()
                    pax.loc = self.vehicle
                    logging.info("T=%4.3f %s loaded into %s at station %s",
                                 Sim.now(), pax, self.vehicle, self.station_id)
                    e_notify = api.SimNotifyPassengerLoadEnd()
                    e_notify.vID = self.vehicle.ID
                    e_notify.sID = self.station_id
                    e_notify.pID = pax.ID
                    globals.Interface.send(api.SIM_NOTIFY_PASSENGER_LOAD_END,
                                          e_notify)
                    # If using the LOBBY policy, notify that passenger load command
                    # has completed.
                    if self._load_msgID:
                        cmd_notify = api.SimCompletePassengerLoadVehicle()
                        cmd_notify.msgID = self._msgID
                        cmd_notify.vID = self.vehicle.ID
                        cmd_notify.sID = self.station_id
                        cmd_notify.pID = pax.ID
                        globals.Interface.send(api.SIM_COMPLETE_PASSENGER_LOAD_VEHICLE,
                                          cmd_notify)
                        self._load_msgID = None

                self._busy = False
                if station.passive():
                    Sim.reactivate(station, prior = True)

            else:
                assert not self._busy
                yield Sim.passivate, self


class Platform(traits.HasTraits):
    berths = traits.List(traits.Instance(Berth))
    track_segment = traits.Instance('layout.TrackSegment')
    berth_length = traits.CFloat
    unloading = traits.CBool
    loading = traits.CBool

    def __init__(self, berths, track_segment, berth_length, unloading, loading):
        traits.HasTraits.__init__(self)
        self.berths = berths
        self.track_segment = track_segment
        self.berth_length = berth_length
        self.unloading = unloading
        self.loading = loading

    def advance(self, prev_platform):
        """Vehicles 'bubble' forward one slot. There is no 'accordian' effect
        -- all vehicles at the platform move synchronously. Does not advance
        sim time. Moves a vehicle from the front berth of prev_platform into the
        rear berth of self if there is room.
        prev_platform may be None.
        """
        if len(self.berths) >= 2:
            for i in xrange(len(self.berths) - 1):
                # Lead berth is empty, the following berth has a non-busy vehicle
                if self.berths[i].is_empty() and \
                      not self.berths[i+1].is_empty() and \
                      not self.berths[i+1].is_busy():
                    # swap the berths' vehicle
                    self.berths[i+1].vehicle.bump_forward(self.berth_length)
                    self.berths[i].vehicle, self.berths[i+1].vehicle = \
                            self.berths[i+1].vehicle, self.berths[i].vehicle

        if self.berths[-1].is_empty() and prev_platform != None:
            prev_front_berth = prev_platform.berths[0]
            if not prev_front_berth.is_empty() and not prev_front_berth.is_busy():
                prev_front_berth.vehicle.bump_forward(self.berth_length)
                self.berths[-1].vehicle, prev_front_berth.vehicle = \
                       prev_front_berth.vehicle, self.berths[-1].vehicle

    def is_empty(self):
        """True if no berths are occupied."""
        empty = True
        for berth in self.berths:
            if not berth.is_empty():
                empty = False
                break
        return empty

class Station(traits.HasTraits):
    """TODO: Check documentation and update if necessary.

    First pass implementation will use a simple interface. Rather than
    explicitly handling passengers queuing for vehicles, the controller may
    tell any passenger to board any vehicle that's at the platform.

    The layout for this Station implementation is serial. Upon entering the
    station, a vehicle enters an unloading platform. Once unloaded, the vehicle
    moves into a empty vehicle queue. Then the vehicle enters a loading
    platform. Finally, the frontmost vehicle of the loading platform may exit
    the station.

    In this version, there is no bypass track -- empty vehicles must pass
    through the loading platform to exit the station.

    The platforms consist of a number of berths. Each berth is a SimPy Process
    object, and the Station process serves to coordinate and direct the actions
    of the berths.

    It is assumed that the vehicle speed is kept low within the station, and
    that vehicles have a fixed time to advance a slot (where a slot is either
    a queue position, or a load/unload berth). That is, advancing five slots
    always takes 5 * v_adv_time. Transfering from the front of the unload
    platform to the rear of the empty queue also takes one v_adv_time. Ditto for
    transferring to the load platform.

    All vehicles in the station advance in synchronicity. A vehicle that is not
    ready to move at the beginning of a cycle is delayed from moving until the
    beginning of the next cycle, where cycle length is the v_adv_time.

    When a vehicle enters or leaves a station, there is a v_adv_time delay. If
    a vehicle is entering, the following timeline is used:
        0: SIM_NOTIFY_VEHICLE_ARRIVE (station) msg sent
        0 - v_adv_time: Vehicle 'fully' occupies rearmost berth of platform
                        Vehicle body moves into station and off of the track
        v_adv_time: Vehicle is completely off of track.
                    SIM_NOTIFY_VEHICLE_EXIT (edge) msg sent

    When a vehicle is launched from the station:
        0: CTRL_CMD_STATION_LAUNCH received
           SIM_NOTIFY_VEHICLE_ARRIVE (edge) msg sent
        0 - v_adv_time: Vehcicle 'fully' occupies launch berth
                        Vehicle body moves onto track
        v_adv_time: Vehicle is completely out of station
                    SIM_COMPLETE_STATION_LAUNCH msg sent

    The entry and launch of a vehicle happens asynchronously.

    TODO: Use berth length and station speed limit to determine v_adv_time?
    TODO: Test / Bugfix StationSummary msgs
    """

    platforms = traits.List(traits.Instance(Platform))
    track_segments = traits.List(traits.Instance('layout.TrackSegment'))

    # Passengers waiting at the station.
    passengers = traits.List(traits.Instance('events.Passenger'))

    def __init__(self, ID, label, platforms, track_segments, **tr):
#        Sim.Process.__init__(self, name=self.label)
        self.ID = ID
        self.label = label
#        self.policy = policy # vehicle loading policy: 'QUEUE' or 'LOBBY'
#        # TODO: Remove policy, and make it a choice of the external station controller

        self.platforms = platforms
        self.track_segments = track_segments

        self.totalDepartures = 0
        self.totalArrivals = 0
        self.totalCrashes = 0

        # Set to True by another process prior to interruption.
        self.launchFlag = False

        # boarding pairs. Key is the Vehicle instance, value is the
        # Passenger instance. Only used under LOBBY policy.
        self._boarding = dict()

        self.rdy_launch_sent = False
        self.unrdy_launch_sent = False
        self.rdy_load_sent = set()

        # Every station will have a slightly different view
        self.view = self.make_view()

    def startup(self):
        # upon station activation, activate berths
        for platform in self.platforms:
            for berth in platform.berths:
                Sim.activate(berth, berth.run())

    def is_empty(self):
        empty = True
        for platform in self.platforms:
            if not platform.is_empty():
                empty = False
                break
        return empty

#    def accept(self, veh):
#        """Accept a vehicle into the station. Notifies the controller of the
#        event and reactivates Station (self) if necessary.
#        """
#        self.totalArrivals += 1
###        rear_berth = self.unload_platform[-1]
###        rb_vehicle = rear_berth.vehicle # save the current rear vehicle, if any
###
###        # take vehicle even if a crash or overshoot occurred. TEMP?
###        rear_berth.vehicle = veh
#        self.unload_platform[-1].vehicle = veh
#        logging.info("T=%4.3f %s accepted at %s. Speed: %s Pos: %s",
#                     Sim.now(), veh, self, veh.speed, veh.pos)
#        if self.passive():
#            Sim.reactivate(self, prior = True)
###        # Can only return one type of crash. Overshoot is considered to be
###        # the more serious one right now.
###        if veh.speed > self.max_speed:
###            self.totalCrashes += 1
###            print "CRASH T=%4.3f %s entered %s at too high of a speed: %f." %\
###                          (Sim.now(), veh, self, veh.speed)
###            logging.error("T=%4.3f %s entered %s at too high of a speed: %f.",
###                          Sim.now(), veh, self, veh.speed)
###            raise globals.StationOvershootError
###        if rb_vehicle:
###            self.totalCrashes += 1
###            print "CRASH T=%4.3f %s entered %s without room to accept. Hit %s" %\
###                          (Sim.now(), veh, self, rb_vehicle)
###            logging.error("T=%4.3f %s entered %s without room to accept. Hit %s",
###                          Sim.now(), veh, self, rb_vehicle)
###            raise globals.StationFullError

#    def ctrl_loop(self):
#        # Startup Code
#        # upon station activation, activate berths
#        for platform in self.platforms:
#            for berth in platform.berths:
#                Sim.activate(berth, berth.run())
#
#        if self.is_empty():
#            yield Sim.passivate, self  # no vehicles, nothing to do.
#        self.next_heartbeat = Sim.now() + self.v_adv_time
#
#        # Main Loop
#        while True:
#            self.send_rdy_load(self.load_platform)
#            self.send_rdy_launch()
#
#            # Allow communication with controller (no time passes)
#            yield Sim.hold, self
#
#            self.unload_passengers(self.unload_platform)
#            self.load_passengers(self.load_platform)
#
#            self.send_rdy_load(self.load_platform)
#            self.send_rdy_launch()
#
#            # Slide everything forward one slot. Don't update positions until
#            # after the time cost is paid.
#            yield Sim.hold, self, self.next_heartbeat - Sim.now()
#
#            # Once the vehicle clears the exit berth, it interrupts the station
#            while self.interrupted():
#                assert self.interruptCause.ID is flb.vehicle.ID # vehicle is interruptCause
#                self.next_heartbeat = Sim.now() + self.interruptLeft
#                flb.vehicle = None
#                # If station is now empty, passivate
#                if self.is_empty():
#                    logging.info("T=%4.3f %s now empty, passivating.", Sim.now(), self)
#                    self.interruptReset()
#                    yield Sim.passivate, self
#                else:
#                    logging.info("T=%4.3f %s still contains a vehicle, staying active",
#                                 Sim.now(), self)
#                    t_left = self.interruptLeft
#                    self.interruptReset()
#                    yield Sim.hold, self, t_left
#
#            # Advance the front-most platform first, then work back.
#            for platform in self.platforms.reverse():
#                platform.advance()
#
#            self.next_heartbeat = Sim.now() + self.v_adv_time

    def add_passenger(self, pax):
        """Add a passenger to this station."""
        assert pax not in self.passengers
        self.passengers.append(pax)

    def unload_passengers(self, platform):
        packed = True
        for b in platform:
            # If there's a vehicle with a passenger in the berth, and it's
            # not already unloading, and it's as far forward as it can go
            # (packed), then unload.
            if not b.vehicle:
                packed = False
            if b.vehicle and b.vehicle.passengers and not b.is_busy() and packed:
                b._unload = True
                Sim.reactivate(b, prior = True)

#    def send_rdy_load(self, platform):
#        packed = True
#        for b in platform:
#            v = b.vehicle
#            if not v:
#                packed = False
#            elif b.is_busy():
#                packed = True
#            elif packed and not v.passengers and not b.is_busy():
#                if v not in self._boarding and \
#                   v not in self.rdy_load_sent:
#                    self.rdy_load_sent.add(v)
#                    rl_msg = api.SimNotifyVehicleReadyLoad()
#                    rl_msg.vID = v.ID
#                    rl_msg.sID = self.ID
#                    globals.Interface.send(api.SIM_NOTIFY_VEHICLE_READY_LOAD,
#                                          rl_msg)

#    def load_passengers(self, platform):
#        """If under a LOBBY boarding policy, uses boarding information
#        previously provided by the board_passenger function to load passengers
#        into the appropriate vehicles, if available.
#
#        WARNING, TODO: LOBBY does not currently support multiple passengers
#        on one vehicle.
#
#        If under a QUEUE boarding policy, passengers are loaded on a first-come/
#        first-served basis. If frontmost passenger in queue is willing to share,
#        other passengers with the same destination (and also willing to share)
#        will board with the frontmost passenger. Time for loading and unloading
#        multiple passengers is the sum of the individual times.
#        """
#        if len(self.passengers) == 0:
#            return
#
#        packed = True
#        for b in platform:
#            # If there's an empty vehicle in the berth, and it's
#            # not already loading, and it's as far forward as it can go
#            # (packed), then check if there is a passenger associated with it.
#            # Load the passenger if there is.
#            v = b.vehicle
#            if not v:
#                packed = False
#            elif b.is_busy():
#                packed = True
#            elif v and not v.passengers and not b.is_busy() and packed:
#                if self.policy == 'LOBBY':
#                    try:
#                        pax, msgID = self._boarding[v]
#                        idx = self.passengers.index(pax)
#                        del self.passengers[idx]
#                        del self._boarding[v]
#                        b._load_msgID = msgID
#                    except KeyError: # No boarding info for that vehicle
#                        continue
#                elif self.policy == 'QUEUE':
#                    if self.passengers:
#                        pax = [self.passengers.pop(0)]
#                        if pax[0].will_share:
#                            share_list = [(idx, p) for idx, p in
#                                        enumerate(self.passengers) if
#                                        p.dest_station is pax[0].dest_station and
#                                        p.will_share]
#                            pax.extend([p for idx, p in share_list[:v.max_pax_capacity-1]])
#                            for idx, p in reversed(share_list[:v.max_pax_capacity-1]):
#                                del self.passengers[idx]
#
#                else:
#                    raise Exception, 'Unknown pax loading policy'
#
#                b.load(msg_id, pax)
#
#                self.rdy_load_sent.discard(v)

    def board_passenger(self, vehicle, passenger, msgID):
        """Tell a passenger which vehicle to board. Command is queued until
        vehicle is ready. MsgID refers to the CtrlCmdPassengerBoardVehicle msg.

        Passenger is required to be at the station already, vehicle is not.

        For now, only one passenger may be told to board a vehicle. If
        board_passenger is called twice for the same vehicle, the old
        passenger will no longer board that vehicle.
        """
        self._boarding[vehicle] = (passenger, msgID)

    def is_launchable(self, vehicle):
        """Returns True if vehicle is launchable, otherwise returns false.
        """
        front_berth = self.load_platform[0]
        if front_berth.vehicle and not front_berth.is_busy():
            return True
        else:
            return False

#    def launch(self, vehicle, target_speed, max_accel, max_decel, max_jerk, msgID):
#        """Intended for use by other objects (e.g. a comm instance).
#        Immediately launches the vehicle if ready. Otherwise raises
#        a globals.InvalidVehicleID exception.
#
#        Note that vehicle is a vehicle instance, not an ID.
#        """
#        if not self.is_launchable(vehicle):
#            raise globals.InvalidVehicleID, vehicle.ID
#        front_berth = self.load_platform[0]
#        vehicle.set_speed(target_speed, max_accel, max_decel, max_jerk, 0)
#        self.totalDepartures += 1
#        self.rdy_launch_sent = False
#        self.unrdy_launch_sent = False
#        self.rdy_load_sent.discard(vehicle)
#        l_msg = api.SimCompleteStationLaunch()
#        l_msg.msgID = msgID
#        l_msg.vID = vehicle.ID
#        l_msg.sID = self.ID
#        globals.Interface.send(api.SIM_COMPLETE_STATION_LAUNCH, l_msg)
#        logging.info("T=%4.3f %s launched from berth %s of %s with pax %s",
#                     Sim.now(), vehicle, front_berth.ID, self,
#                     [pax.ID for pax in vehicle.passengers if pax])
#
#        if vehicle.passive():
#            Sim.reactivate(vehicle, prior = True)
#        else:
#            Sim.activate(vehicle, vehicle.ctrl_loop())

#    def send_rdy_launch(self):
#        flb = self.load_platform[0] # front loading berth
#        if flb.vehicle and flb.vehicle.passengers \
#                and not flb.is_busy() and not self.rdy_launch_sent:
#            self.rdy_launch_sent = True
#            rdy_launch_msg = api.SimNotifyStationReadyLaunch()
#            rdy_launch_msg.vID = flb.vehicle.ID
#            rdy_launch_msg.sID = self.ID
#            for pax in flb.vehicle.passengers:
#                rdy_launch_msg.pID.append(pax.ID)
#            globals.Interface.send(api.SIM_NOTIFY_STATION_READY_LAUNCH,
#                                  rdy_launch_msg)
#        elif flb.vehicle and flb.is_busy() and self.rdy_launch_sent \
#                 and not self.unrdy_launch_sent:
#            self.rdy_launch_sent = False
#            self.unrdy_launch_sent = True
#            urdy_launch_msg = api.SimNotifyStationUnreadyLaunch()
#            urdy_launch_msg.vID = flb.vehicle.ID
#            urdy_launch_msg.sID = self.ID
#            globals.Interface.send(api.SIM_NOTIFY_STATION_UNREADY_LAUNCH,
#                                  urdy_launch_msg)

#    def fill_StationStatus(self, s_status):
#        """Fills an api.StationStatus instance with current information."""
#        s_status.sID = self.ID
#        if self.label:
#            s_status.label = self.label
#
#        # TODO: Only support for one loading/unloading plat right now.
#        ulp_status = s_status.unloading_plat.add()
#        ulp_status.platID = 1
#        for idx, b in enumerate(self.unload_platform):
#            b_status = ulp_status.berths.add()
#            b_status.bID = idx + 1 # first ID is 1
#            b_status.vID = (b.vehicle.ID if b.vehicle else api.NONE_ID)
#            b_status.busy = b.is_busy()
#
#        lp_status = s_status.loading_plat.add()
#        lp_status.platID = 2
#        for idx, b in enumerate(self.load_platform):
#            b_status = lp_status.berths.add()
#            b_status.bID = idx + 1 # first ID is 1
#            b_status.vID = (b.vehicle.ID if b.vehicle else api.NONE_ID)
#            b_status.busy = b.is_busy()
#
#
#        for v in self.queue:
#            s_status.emptyQueue.append(v.ID if v else api.NONE_ID)
#
#        for p in self.passengers:
#            s_status.pID.append(p.ID)
#
#        s_status.v_adv_time = int(round(self.v_adv_time,3)*1000) # in millisec
#        if self.policy == 'QUEUE':
#            s_status.policy = api.QUEUE
#        elif self.policy == 'LOBBY':
#            s_status.policy = api.LOBBY
#        else:
#            raise Exception, "Unknown station policy"
#
#    def fill_StationSummary(self, s_sum):
#        """Fills an api.StationSummary instance with current information."""
#        s_sum.sID = self.ID
#        if self.label:
#            s_sum.label = self.label
#        # TODO: extend to multiple load platforms
#        flb = self.load_platform[0] # front loading berth
#        s_sum.loaded_ready_launch.append(flb.vehicle.ID if flb.vehicle
#                                         and flb.vehicle.passengers
#                                         and not flb.is_busy()
#                                         else 0)
#        # TODO: extend to multiple load platforms, and empty queue bypass
#        s_sum.unloaded_ready_launch.append(flb.vehicle.ID if flb.vehicle
#                                           and not flb.vehicle.passengers
#                                           and not flb.is_busy()
#                                           else 0)
#
#        for p in self.passengers:
#            s_sum.pID.append(p.ID) # passengers waiting at station
#
#        entry_berth = self.unload_platform[-1]
##
##        entering_v = None
##        if self.resource.activeQ[-1].pos <= self.berth_length
#
#        # entry berth is immediately available
#        if not entry_berth.vehicle:
#            s_sum.next_accept_time = int(round(Sim.now()*1000)) # s -> ms
#        # test if entry berth will be available after next heartbeat
#        else:
#            avail = False
#            # look from back to front
#            for b in reversed(self.unload_platform):
#                # if found a busy berth, it's non-deterministic
#                if b.is_busy():
#                    break
#                # if found an empty berth, then vehicles will slide forward.
#                elif not b.vehicle:
#                    avail = True
#                    break
#                else:
#                    continue
#
#            if avail:
#                s_sum.next_accept_time = int(round(self.next_heartbeat*1000)) # s -> ms
#            else:
#                s_sum.next_accept_time = -1 # non-determinstic
#
#        # vehicles_needed
#        v_avail = 0 # empty and unloading vehicles
#        for v in self.queue:
#            if v:
#                v_avail += 1
#        for b in self.unload_platform:
#            if b.vehicle:
#                v_avail += 1
#        v_needed = len(self.passengers)
#        s_sum.vehicles_needed = max(0, v_needed - v_avail)

    def __str__(self):
        return self.label

    def __hash__(self):
        return self.ID.__hash__()

    def make_view(self):
        """Make a traits view (popup window) for this station."""
        pax_table_editor = ui.TableEditor(
                            # Only the passenger data relevant when looking at a station.
                            columns = [ui_tc.ObjectColumn(name='label', label='Name'),
                                       ui_tc.ObjectColumn(name='trip_start'),
                                       ui_tc.ObjectColumn(name='dest_station', label='Destination'),
                                       ui_tc.ObjectColumn(name='wait_time', label='Waiting (sec)', format="%.2f"),
                                       ui_tc.ObjectColumn(name='will_share', label='Will Share'),
                                       ui_tc.ObjectColumn(name='load_delay', label='Time to Board (sec)')],
                                       # more...
                            deletable = True,
#                            sort_model = True,
                            auto_size = True,
                            orientation = 'vertical',
                            show_toolbar = True,
                            reorderable = True, # Does this affect the actual boarding order (think no...)
                            rows = 5,
                            row_factory = events.Passenger)


        groups = ui.VGroup(
                           ui.Group(
                                ui.Label('Waiting Passengers'),
                                ui.Item(name='passengers',
                                        show_label = False,
                                        editor=pax_table_editor
                                        ),
                                show_border = True),
#                           ui.Group(
#                                ui.Label('Load Platform'),
#                                ui.Item(name='load_platform',
#                                        show_label = False,
#                                        editor=ui.ListEditor(style='custom',
#                                                             rows=len(self.load_platform)),
#                                        style='readonly'),
#                                show_border = True
#                                ),
                           ui.Group(
                                ui.Label('Queue'),
                                ui.Item(name='queue',
                                        show_label=False,
                                        editor=ui.ListEditor(editor=ui.TextEditor()),
                                        style='readonly'),
                                show_border = True
#                                ),
#                           ui.Group(
#                                ui.Label('Unload Platform'),
#                                ui.Item(name='unload_platform',
#                                        show_label = False,
#                                        editor=ui.ListEditor(style='custom',
#                                                             rows=len(self.unload_platform)),
#                                        style='readonly',
#                                        ),
#                                show_border = True,
                                ))

        view = ui.View(groups,
                       title=self.label,
#                       scrollable = True,
                       resizable = True,
                       height = 700,
                       width = 470
                       )

        return view



