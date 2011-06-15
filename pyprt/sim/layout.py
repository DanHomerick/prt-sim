# '/' is true division, '//' is truncating division
from __future__ import division
import bisect
import math

from numpy import inf
import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import SimPy.SimulationRT as Sim

import pyprt.shared.api_pb2 as api
import pyprt.shared.utility as utility
import common
import station
import vehicle

##POS = 0
##VEL = 1
##ACCEL = 2
##JERK = 3

class Node(traits.HasTraits):
    """An abstract base class for any location that has some length that can be
    moved along in a continuous manner.

    Parameters:
      ID -- an unique integer ID
      x_start -- x coordinate for the start of the segment, e.g. longitude.
      y_start -- y coordinate for the start of the segment, e.g. latitude.
      x_end -- x coordinate for the end of the segment
      y_end -- y coordinate for the end of the segment.
      length -- length of track segment, including z-dimension, measured in meters.
      max_speed -- a track-imposed speed limit, measured in meters/second.
      elevation_positions -- an ordered sequence of position values which forms
          a parallel array with elevations. Measured in meters.
      elevations -- a sequence of track elevation values at the positions in
          'elevation_positions'. Measured in meters.
      label -- a string identifier which need not be unique.
    """
    ID        = traits.CInt
    length    = traits.CFloat
    max_speed = traits.CFloat
    enabled   = traits.CBool(True)  # For failure scenarios
    label     = traits.String
    x_start   = traits.CFloat
    y_start   = traits.CFloat
    x_end     = traits.CFloat
    y_end     = traits.CFloat
    elevation_positions = traits.List
    elevations= traits.List
    next_loc  = traits.Either(traits.This, None)  # The default next location. May be a Node, or a subclass of Node.
    vehicles  = traits.List(traits.Instance(vehicle.BaseVehicle)) # vehicles[0] is the frontmost vehicle

    # A default view
    traits_view =  ui.View(ui.Item(name='ID'),
                    ui.Item(name='label'),
                    ui.Item(name='length'),
                    ui.Item(name='max_speed'),
                    ui.Item(name='enabled'),
                    ui.Item(name='vehicles')
                   )

    def __init__(self, ID, x_start, y_start, x_end, y_end, length, max_speed,
                 elevation_positions, elevations, label='', **tr):
        traits.HasTraits.__init__(self, **tr)
        self.ID = ID
        self.label = (label if label else str(ID)+'_'+self.__class__.__name__)
        self.x_start = x_start
        self.y_start = y_start
        self.x_end   = x_end
        self.y_end   = y_end
        self.length = length
        self.max_speed = max_speed
        self.elevation_positions = elevation_positions
        self.elevations = elevations

        dx = x_end - x_start
        dy = y_end - y_start
        self._direction = math.atan2(dy, dx)

    def __str__(self):
        return self.label

    def __cmp__(self, other):
        if isinstance(other, Node): # includes subclasses
            return cmp(self.ID, other.ID)
        else:
            return cmp(id(self), id(other))

    def get_direction(self, position):
        """Returns the direction of the track at 'position' as an angle away
        from East, measured in radians. e.g. 0 is due East, pi/2 is North, etc.
        """
        # TODO: position is ignored for now, because track curvature is not
        # currently being used.
        return self._direction

    def get_elevation(self, position):
        """Returns the elevation for the sampled point prior to 'position'. That
        is, no interpolation is done, and the left-hand value is returned.
        """
        idx = bisect.bisect_right(self.elevation_positions, position)
        try:
            return self.elevations[idx-1]
        except IndexError: # No elevation data was available.
            if len(self.elevations) != 0:
                print self, "missing elevations at idx-1", idx, " pos ", position, self.elevations, self.elevation_positions
            assert len(self.elevations) == 0
            return 0

class TrackSegment(Node):
    def __init__(self, ID, x_start, y_start, x_end, y_end, length, max_speed,
                 elevation_positions, elevations, label='', **tr):
        Node.__init__(self, ID, x_start, y_start, x_end, y_end, length, max_speed,
                      elevation_positions, elevations, label)

    def switch(self, new_next, msg_id):
        """Applicable for TrackSegments which have two or more downstream neighbors.
        Switches the default 'next_loc' Tracksegment to new_next after a delay
        specifed in the sim configuration. If a vehicle is on
        the switch (straddling trackID and nextID) or attempts
        to use it during the transition delay, a crash will result."""
        if self.next_loc is not new_next: # if not already set to this track
            # Check if a vehicle is currently straddling the switch point
            if len(self.vehicles) != 0:
                lv = self.vehicles[0]
                neighbors = common.digraph.neighbors(self)
                for neighbor in neighbors:
                    if lv in neighbor.vehicles:
                        lv.crash(other_vehicle=None)

            self.next_loc = None # transitioning - switch is unusable
            switch_delay = common.config_manager.get_track_switch_time()
            common.AlarmClock(Sim.now()+switch_delay, self._delayed_switch, new_next, msg_id)

    def _delayed_switch(self, new_next, msg_id):
        """Changes self.next_loc to point to the new TrackSegment. Sends a
        SimCompleteSwitch message to the controller."""
        self.next_loc = new_next
        complete_msg = api.SimCompleteSwitch()
        complete_msg.msgID = msg_id
        complete_msg.trackID = self.ID
        complete_msg.nextID = self.next_loc.ID
        complete_msg.time = Sim.now()
        common.interface.send(api.SIM_COMPLETE_SWITCH, complete_msg)

    def fill_TrackSegmentStatus(self, ts):
        """Fills an api.TrackSegmentStatus instance with current information."""
        ts.trackID = self.ID
        if self.label:
            ts.label = self.label
        ts.max_speed = int(self.max_speed*100) # m/s -> cm/s
        ts.length = int(self.length*100) # m -> cm
        for v in self.vehicles:
            ts.vID.append(v.ID)
        if ts.next != None:
            ts.next = self.next_loc.id
        else:
            ts.next = api.NONE_ID


##    def traverse_gen(self):
##        """A python generator that yields the hold time until the vehicle
##        reaches the next location.
##        """
##        # According to the path (trajectory), we've aleady reached the next traversal
##        if self.path.get_traversal_from_time() is not \
##                    self.path.get_active_traversal():
##            logging.info("T=%4.3f %s reached end of segment: %s",
##                         Sim.now(), self, self.loc)
##            self._handle_boundry = True
##            return
##
##        # Check for a collision
##        try:
##            next_c = self.collisions[self.collision_idx]
##        except IndexError:
##            next_c = None
##        if next_c:
##            assert utility.time_ge(next_c.time, Sim.now())
##            if utility.time_eql(next_c.time, Sim.now()):
##                # just had a collision
##                msg = api.SimNotifyVehicleCollision()
##                msg.v1ID = next_c.lv.ID
##                msg.v2ID = self.ID
##                loc = next_c.trav.loc
##                msg.loc_type = loc2loctype(loc)
##                msg.locID = loc.ID
##                msg.pos = self.pos
##                msg.delta_v = self.speed - next_c.lv.speed
##                if next_c.rearend:
##                    msg.rearend = True
####                    # think bug may be in get_tail...
####                    if __debug__:
####                        if not utility.dist_eql(self.pos, next_c.lv.tail_pos): # is get tail pos WRONG?
####                            tail_pos, tail_loc = next_c.lv.get_tail()
####                            print "297| Now:%s, self.pos:%s, self.loc:%s, lv.tail_pos:%s, lv.tail_loc:%s" \
####                            % ( Sim.now(), self.pos, self.loc, tail_pos, tail_loc )
####                            print "Collision:", next_c
#####                            print "SELF PATH:"
#####                            print self.path
#####                            print "LV PATH:"
#####                            print next_c.lv.path
#####                            self.path.plot(title='rear vehicle', show=False)
#####                            next_c.lv.path.plot(title='lead vehicle', show=True)
####                            print_activeQs()
####                            assert utility.dist_eql(self.pos, next_c.lv.get_tail()[0])
##                if next_c.sideswipe:
##                    msg.sideswipe = True
##                msg.time = Sim.now()
##                common.interface.send(api.SIM_NOTIFY_VEHICLE_COLLISION, msg)
##                assert next_c.lv is self.find_vehicle_ahead()
##                next_c.occurred = True
##                my_nose_pos, my_tail_pos = self.get_positions()
##                my_nose_loc = self.loc
##                my_tail_loc = self.tail_loc
##                lv_nose_pos = next_c.lv.pos
##                lv_nose_loc = next_c.lv.loc
##                lv_tail_pos = next_c.lv.tail_pos
##                lv_tail_loc = next_c.lv.tail_loc
##                if next_c.sideswipe == True:
##                    collision_type = 'sideswipe'
##                elif next_c.rearend == True:
##                    collision_type = 'rearend'
##                logging.info("CRASH T=%4.3f %s, nose_loc: %s, nose_pos: %.1f, tail_loc: %s, tail_pos: %.1f speed: %.1f collided with:\n\t %s, nose_loc: %s, nose_pos: %.1f, tail_loc: %s tail_pos: %.1f, speed: %.1f\n\tin a %s collision, occurred: %s",
##                             Sim.now(), self, my_nose_loc, my_nose_pos, my_tail_loc, my_tail_pos, self.get_speed()/100,
##                             next_c.lv, lv_nose_loc, lv_nose_pos, lv_tail_loc, lv_tail_pos, next_c.lv.get_speed()/100,
##                             collision_type, next_c.occurred)
##                print "CRASH T=%4.3f %s, nose_loc: %s, nose_pos: %.1f, tail_loc: %s, tail_pos: %.1f speed: %.1f collided with:\n\t %s, nose_loc: %s, nose_pos: %.1f, tail_loc: %s tail_pos: %.1f, speed: %.1f\n\tin a %s collision, occurred: %s" %\
##                             (Sim.now(), self, my_nose_loc, my_nose_pos, my_tail_loc, my_tail_pos, self.get_speed()/100,
##                             next_c.lv, lv_nose_loc, lv_nose_pos, lv_tail_loc, lv_tail_pos, next_c.lv.get_speed()/100,
##                             collision_type, next_c.occurred)
##                self.collision_idx += 1
##
##        if self.tail_releases:
##            # at a tail release point
##            if utility.time_eql(self.tail_releases[0].time, Sim.now()):
##                loc = self.tail_releases[0].trav.loc
##                msg = api.SimNotifyVehicleExit()
##                self.fill_VehicleStatus(msg.v_status)
##                msg.time = Sim.now()
##                common.interface.send(api.SIM_NOTIFY_VEHICLE_EXIT, msg)
##                del self.tail_releases[0]
##                if isinstance(loc, station.Station):
##                    self.interrupt(loc) # notify station that I'm clear
##
##                yield Sim.release, self, loc.resource
##
##        try:
##            next_collision_time = self.collisions[self.collision_idx].time
##        except IndexError:
##            next_collision_time = inf
##        if self.tail_releases:
##            next_tail_release_time = self.tail_releases[0].time
##        else:
##            next_tail_release_time = inf
##        next_traversal_time = self.path.get_eta_next_loc(relative=False)
##
##        delay = min(next_collision_time,
##                    next_tail_release_time,
##                    next_traversal_time) - Sim.now()
##
##        if __debug__:
##            if next_traversal_time <= next_collision_time \
##                    and next_traversal_time <= next_tail_release_time:
##                logging.debug("T=%4.3f %s is traversing %s until next traversal %4.3f",
##                              Sim.now(), self, self.loc, next_traversal_time)
##            elif next_tail_release_time <= next_collision_time \
##                    and next_tail_release_time <= next_traversal_time:
##                logging.debug("T=%4.3f %s is traversing %s until next tail release %4.3f (%s)",
##                              Sim.now(), self, self.loc, next_tail_release_time, self.tail_releases[0].trav.loc)
##            elif next_collision_time <= next_tail_release_time \
##                    and next_collision_time <= next_traversal_time:
##                logging.debug("T=%4.3f %s is traversing %s until next collision time %4.3f",
##                              Sim.now(), self, self.loc, next_collision_time)
##
##        assert utility.time_ge(delay, 0)
##        yield Sim.hold, self, delay



##    def boundry_gen(self):
##        """Handle the transitions from one traversal to the next."""
##        # Check that we're actually at a TrackSegment boundary.
##        assert utility.dist_eql(self._spline.evaluate(Sim.now()).pos,
##                               self._path[self._path_idx_nose].length + self._pos_offset_nose)
##
##        old_loc = self.path.get_active_loc()
##
##        # --- Decide what location will be next ---
##        try:    # Next location has already been decided (vehicle switching)
##            new_loc = self.path.traversals[self.path.active_idx+1].loc
##            assert new_loc in common.digraph.neighbors(old_loc)
##        except IndexError:
##            new_loc = old_loc.next
##            if new_loc is None: # can be caused by a dead-end track or a track switch in mid-throw
##                # TODO: Consistant collision handling through a single function!!!!
##                raise Exception("Vehicle hit a dead end, or a track switch in mid-throw!")
##            else:
##                self.path.append_loc(new_loc)
##
##        # --- Handle that new location ---
##        # Set up helper event to release the tail.
##        tr = TailRelease(self, self.path.get_active_traversal())
##        self.tail_releases.append(tr)
##
##        assert isinstance(new_loc, TrackSegment)
##        yield Sim.request, self, new_loc.resource
##        logging.info("T=%4.3f %s aquired %s", Sim.now(), self, new_loc)
##        self.path.active_idx += 1
##        self._handle_boundry = False

#        # Station is a bit special
#        elif isinstance(new_loc, station.Station):
#            yield Sim.request, self, new_loc.resource
###            try:
###                new_loc.accept(self)             # enter the station
###            except common.StationOvershootError:
###                # TEMP kludge to keep things behaving rationally. Introduce a
###                # discontinuity in the vehicle speed, instantly bringing it
###                # down to the station's max_speed. Longer term soln?
###                seg = Segment(poly1d([0,0,new_loc.max_speed,0]), inf, Sim.now(), self.total_mass)
###                self.path.change_trajectory( [seg] )
###            except common.StationFullError:
###                # FIXME: Undefined what happens to vehicle. Currently
###                # 'overwrites' old vehicle in the station entry berth.
###                pass
#            self.path.active_idx += 1
#
#
#            if __debug__ and self.accel != 0: # TEMP: assumption makes math easier
#                logging.debug("%s came into %s with speed %s and accel %s. For the moment, it needs to come in with 0 accel.", self, self.loc, self.speed, self.accel)
#                raise Exception("%s came into %s with speed %s and accel %s. For the moment, it needs to come in with 0 accel." % (self, self.loc, self.speed, self.accel))
#            # figure how long I should maintain speed before coming to a halt.
#            sp = SpeedProfiler(v_init=self.speed, v_final=0,
#                   max_accel=self.accel_max_norm,
#                   max_decel=self.accel_min_norm,
#                   max_jerk=self.jerk_max_norm,
#                   a_init=0,
#                   p_init=0)
#            slowdown_prof = sp.get_profile()
#            stop_pos = slowdown_prof[-1][0][POS]
#            #print "vID: %s, v_init: %s, a_init: %s, stop_pos: %s" % (self.ID, self.speed, self.accel, stop_pos)
#            if __debug__ and not (stop_pos > 0 and stop_pos <= new_loc.berth_length):
#                logging.debug("%s came into %s too fast and stopped beyond the end of the first berth: %s", self, self.loc, self.pos)
#                raise Exception("%s came into %s too fast and stopped beyond the end of the first berth: %s" % (self, self.loc, self.pos))
#            coasting_dur = (new_loc.berth_length - stop_pos - .01) / self.speed
#            assert coasting_dur >= 0
#            coasting_seg = Segment(poly1d([self.speed, 0]), coasting_dur, Sim.now(), self.total_mass)
#            all_segs = [coasting_seg]
#            for eqn, dur in slowdown_prof: # eqn, dur tuples
#                prevSeg = all_segs[-1]
#                eqn[POS] = prevSeg.pos_eqn(prevSeg.duration) # set pos to match prevSeg
#                seg = Segment(eqn, dur, prevSeg.end_time, self.total_mass)
#                all_segs.append(seg)
#            self.path.change_trajectory(all_segs)
#            self.check_for_collisions()
#
#            # Notify vehicle behind me that I changed speeds.
#            follower = self.find_vehicle_behind()
#            if follower:
#                InterruptCollisionChk(self, follower)
#
#            # Interrupt this vehicle to allow it's current hold time to be changed.
#            InterruptSetSpeed(self)
#
#            StationAccept(self, new_loc, all_segs[-1].start_time)
#            self._handle_boundry = False
#
#        elif isinstance(new_loc, Switch):
#            yield Sim.request, self, new_loc.resource
#            logging.info("T=%4.3f %s aquired %s", Sim.now(), self, new_loc)
#            self.path.active_idx += 1
#            self._handle_boundry = False
#
#        else:
#            raise TypeError, new_loc

#        if __debug__:
#            trav = self.path.get_active_traversal()
#            # if not trav.start_time <= Sim.now() <= trav.end_time:
#            if not (utility.time_le(trav.start_time, Sim.now()) and utility.time_le(Sim.now(), trav.end_time)):
#                raise Exception, (trav.start_time, Sim.now(), trav.end_time)

##        # Send Arrival Msg
##        notify_msg = api.SimNotifyVehicleArrive()
##        self.fill_VehicleStatus(notify_msg.v_status)
##        notify_msg.time = Sim.now()
##        common.interface.send(api.SIM_NOTIFY_VEHICLE_ARRIVE, notify_msg)
##
##        # Now that we're effectively on the new_loc, do some collision checking.
##        self.check_for_collisions()

##    def check_for_collisions(self, lv=None):
##        """Generates a list of times when collisions will occur and stores it
##        as `self.collsion_times`. Determines the lead vehicle (lv) if not provided.
##        Only checks for collisions on self's active Traversal, but checks
##        against lv's active Traversal and part of lv's next Traversal to ensure
##        that lv's tail clears self's location without a collision occuring.
##
##        Discards any pending collisions that have not yet occurred on the
##        principle that no other vehicle can possibly insert itself between
##        self and lv while they are in the midst of traversing a location,
##        therefore (re)checking against just lv is sufficient.
##        """
##        # discard future collisions
##        self.collisions = self.collisions[:self.collision_idx]
##
##        s_trav = self.path.get_active_traversal()
##
##        # find the vehicle ahead of me
##        if lv is None:
##            lv = self.find_vehicle_ahead()
##        if lv:
##            # self is stopped. No possibility with lv unless lv is
##            # travelling in reverse, which is handled at a higher level.
##            if s_trav.get_end_time() == inf:
##                return
##
##            # checks s_trav against lv_trav. Need to check s_trav for it's
##            # entire valid time, even though lv may traverse several segments
##            # during that time.
##            lv_trav = lv.path.get_active_traversal()
##            lv_idx = lv_trav.idx
##            lv_tail_trav = lv.get_tail_traversal()
##
##            if lv_tail_trav.loc is not s_trav.loc \
##                    and utility.time_eql(Sim.now(), s_trav.start_time):
##                # I just started traversing this TrackSegment, but the tail of
##                # lv isn't ahead of me--it's on another TrackSegment entirely.
##                # The only explaination for this is that I am merging, and am
##                # sideswiping lv.
##                assert len(common.digraph.predecessors(s_trav.loc)) >= 2
##                self.collisions.append(Collision(s_trav.start_time, lv,
##                                                 s_trav, sideswipe=True))
##                return
##
##            # Figure out which lv traversals we want to check s_trav against.
##            lv_travs = [lv_trav]
##            offset = lv.length
##            for trav in lv.path[lv_trav.idx+1:]:
##                lv_travs.append(trav)
##                offset -= trav.length
##                if offset <= 0:
##                    break
##
##            lv_tail_idx = lv_tail_trav.idx
##            t = Sim.now()
##
##            # Check s_trav vs. each of lv's relevant traversals
##            trav_offset = 0
##            for trav in lv_travs:
##                self.collisions.extend(Collision(collision_time, lv, s_trav, rearend=True)
##                                       for collision_time in
##                                       s_trav.collision_check(trav, lv.length, trav_offset, t))
##                trav_offset += trav.length
##
##
###            while True:
###                # sum of loc lengths from self to lv. Excludes lv loc.
###                # See Traversal.collision_check
###                trav_offset = sum(trav.length for trav in lv.path.traversals[lv_tail_idx:lv_idx])
###                print "LAYOUT CHECK:", lv, self
###                self.collisions.extend(Collision(collision_time, lv, s_trav, rearend=True)
###                                       for collision_time in
###                                       s_trav.collision_check(lv_trav, lv.length, trav_offset, t))
###
###                # Stop looking at additional traversals if self has hit the end
###                # of it's traversal, or if self is stopped.
###                # TODO: Change this so that we stop checking for collisions
###                # once lv's tail leaves self's current location.
###                if utility.time_le(s_trav.end_time, lv_trav.end_time) or s_trav.end_time == inf:
###                    break
###
###                else:
###                    t = lv_trav.end_time
###                    lv_idx += 1
###                    try:
###                        lv_trav = lv.path.traversals[lv_idx]
###                    except IndexError:
###                        lv_trav = lv.path.future
##
##            for c in self.collisions[self.collision_idx:]:
##                logging.info("T=%4.3f %s Upcoming collision detected: %s", Sim.now(), self, c)
##
##            if __debug__:
##                for c in self.collisions[self.collision_idx:]:
##                    assert utility.time_ge(c.time, Sim.now())





##    def handle_interrupt(self):
##        logging.debug("T=%4.3f %s interrupted by %s",
##                      Sim.now(), self, self.interruptCause)
##
##        # ----- InterruptCollisionChk ------
##        if isinstance(self.interruptCause, InterruptCollisionChk):
##            # The vehicle ahead of me just changed it's trajectory. Recalc
##            # when any upcoming collisions will occur.
##            lead_vehicle = self.interruptCause.cause
##            # filter out pending collisions with lead vehicle
##            self.collisions = [c for c in self.collisions if c.occurred is True or c.lv is not lead_vehicle]
##            self.check_for_collisions(lead_vehicle)
##            assert self.collisions == sorted(self.collisions)
##
##            # reset collision_idx
##            for idx, c in enumerate(self.collisions):
##                if c.occurred is True:
##                    self.collision_idx = idx + 1
##                else:
##                    break
##
##        # ------ InterruptSetSpeed ------
##        elif isinstance(self.interruptCause, InterruptSetSpeed):
##            # Changing the speed requires that I:
##            # * Reconsider hold time until I reach the next traversal
##            #   Accomplished through regular use of ctrl_loop
##            #
##            # * Reconsider tail release times
##            for tr in self.tail_releases:
##                tr.update_release_time()
##
##        else:
##            raise Exception, "Unknown interruption."
##
##        # return to whatever I was doing
##        self.interruptReset()



##    def find_vehicle_behind(self):
##        """Returns the the vehicle behind me. Only looks in the active
##        traversal.
##
##        Returns None if no vehicle found.
##        """
##        trav = self.path.get_active_traversal()
##        fv = None # following vehicle
##        # Locate myself in the edge's activeQ.
##        # Used to determine relative positioning of vehicles.
##        my_idx = trav.loc.resource.activeQ.index(self)
##        if my_idx != len(trav.loc.resource.activeQ)-1: # I'm not the last vehicle on the edge
##            fv = self.loc.resource.activeQ[my_idx+1]
##        return fv

#        for i in reversed(range(active_trav.idx, len(self.path.traversals))):
#            trav = self.path.traversals[i-1]
#            if isinstance(trav.loc, Station):
#                break
#
#            # Locate myself in the edge's activeQ.
#            # Used to determine relative positioning of vehicles.
#            my_idx = trav.loc.resource.activeQ.index(self)
#            if my_idx == len(trav.loc.resource.activeQ): # I'm the last vehicle on the edge
#                continue
#            else:
#                v = self.loc.resource.activeQ[my_idx+1] # vehicle behind
#
#        return v



##class TailRelease(object):
##    """`trav` is the Traversal instance corresponding to the location that the
##    vehicle should release. Appropriate release time is calculated upon creation.
##
##    The vehicle should call `update_release_time` on all pending TailRelease
##    instances whenever the vehicle trajectory is changed.
##    """
##    def __init__(self, my_vehicle, trav):
##        self.v = my_vehicle
##        self.trav = trav # traversal  to release
##        self.time = float()
##        self.update_release_time()
##
##    def update_release_time(self):
##        # walk forward from the end of the release traversal until I've gone v.length
##        l = self.v.length
##        times = None
##        for trav in self.v.path.traversals[self.trav.idx+1:]:
##            if l >= trav.length:
##                l -= trav.length
##            else:
##                times = trav.get_times_from_pos(l)
##                break
##        if times is None:
##            times = self.v.path.future.get_times_from_pos(l)
##
##        if times.points:
##            self.time = times.points[0]
##        elif times.ranges:       # vehicle stops on release point
##            self.time = times.ranges[0][0]
##        else:                    # vehicle stops before reaching release point
##            self.time = inf
##
##class Collision(object):
##    """A struct-like class, holding information about an upcoming collision.
##    Sorts by time. `occurred` is a flag indicating that the collision has
##    already occurred, rather than just being a predicted collision."""
##    def __init__(self, time, lv, trav, rearend=False, sideswipe=False, occurred=False):
##        self.time = time
##        self.lv = lv      # leading vehicle
##        self.trav = trav
##        self.rearend = rearend
##        self.sideswipe = sideswipe
##        self.occurred = occurred
##    def __str__(self):
##        return "time: %f, lv: %s, rearend: %s, sideswipe: %s, occurred: %s speed: %4.3f \n %s"\
##            % (self.time, self.lv, self.rearend, self.sideswipe, self.occurred, self.lv.get_speed(), self.trav)
##    def __cmp__(self, other):
##        return cmp(self.time, other.time)
##
##class InterruptCollisionChk(Sim.Process):
##    """Notify a vehicle that it needs to update it's estimate of upcoming
##    collisions. Generated whenever a vehicle changes it's trajectory, sent
##    to the vehicle behind.
##
##    `cause` is the vehicle that generated this interrupt.
##    `target` is the vehicle that needs to update it's collision checking.
##      That is, `target` is the vehicle behind `cause`.
##
##    Note: This approach is somewhat inefficient. The lead vehicle could directly
##    interrupt the following vehicle, rather than creating a third party to do
##    the interruption. This approach allows more clarity about the reason for
##    the interrupt, however. (Currently, SimPy interruptions have no mechanism
##    for delivering an information payload.)
##    """
##    def __init__(self, cause, target, time=None, relative=False):
##        Sim.Process.__init__(self, name='InterruptCollisionChk_lv%d_fv_%d' \
##                             % (cause.ID, target.ID))
##        self.cause = cause
##        self.target = target
##        self.time = time
##        self.relative = relative
##        Sim.activate(self, self.run(), prior=True)
##    def __str__(self):
##        return self.name
##
##    def run(self):
##        if self.time:
##            if self.relative:
##                yield Sim.hold, self, self.time + Sim.now()
##            else:
##                yield Sim.hold, self, self.time
##        self.interrupt(self.target)
##
##class InterruptSetSpeed(Sim.Process):
##    """Class exists to satisfy SimPy's requirement that a Sim.Process may only
##    be interrupted by another Sim.Process.
##
##    Interrupts my_vehicle at time. Behaves as a signal only. All changes in
##    state should be made by the vehicle after it receives the interrupt.
##    """
##    def __init__(self, my_vehicle, time=None, relative=False):
##        Sim.Process.__init__(self, name='InterruptSetSpeed_v'+str(my_vehicle.ID))
##        self.my_vehicle = my_vehicle
##        self.time = time
##        self.relative = relative
##        Sim.activate(self, self.run(), prior=True)
##    def __str__(self):
##        return self.name
##
##    def run(self):
##        if self.time:
##            if self.relative:
##                yield Sim.hold, self, self.time
##            else:
##                yield Sim.hold, self, self.time - Sim.now()
##        self.interrupt(self.my_vehicle)

##class StationAccept(Sim.Process):
##    """Class is a helper process. Notifies the Station that the vehicle has
##    come to a complete stop in the entry berth. If relative=False, treats `time`
##    as a delay rather than an absolute time."""
##    def __init__(self, my_vehicle, station, time, relative=False):
##        Sim.Process.__init__(self, name='StationAccept_v'+str(my_vehicle.ID))
##        self.my_vehicle = my_vehicle
##        self.station = station
##        self.time = time
##        self.relative = relative
##        Sim.activate(self, self.run())
##    def run(self):
##        if self.relative:
##            yield Sim.hold, self, self.time
##        else:
##            yield Sim.hold, self, self.time - Sim.now()
##        self.station.accept(self.my_vehicle)

##def shift_poly(poly, delta_t, delta_p):
##    """Shift a scipy.poly1d polynomial.
##    A positive delta_t shifts the polynomial towards the right.
##    A negative delta_t shifts towards the left.
##    A positive delta_p shifts the polynomial upward
##    A negative delta_p shifts downward.
##
##    Only handles up to third order polynomials (up to 'jerk').
##
##    Returns the shifted poly.
##    """
##    if delta_p and not delta_t:
##        ret = poly + delta_p
##    elif poly.order == 0:
##        # delta_t is ignored, since a 0th order poly has no time component.
##        ret = poly + delta_p
##    elif poly.order == 1:
##        c, d = poly.coeffs
##        ret = poly1d([c, -c*delta_t + delta_p + d])
##    elif poly.order == 2:
##        b, c, d = poly.coeffs
##        ret = poly1d([b, -2*b*delta_t + c, b*delta_t**2 - c*delta_t + delta_p + d])
##    elif poly.order == 3:
##        a, b, c, d = poly.coeffs
##        ret = poly1d([a,
##                  -3*a*delta_t + b,
##                   3*a*delta_t**2 - 2*b*delta_t + c,
##                  -a*delta_t**3 + b*delta_t**2 - c*delta_t + delta_p + d])
##    else:
##        raise NotImplementedError, "Limited to <= 3rd order polynomials"
##    return ret
##
##def find_positive_roots(poly):
##    """Returns a list containing the real, positive roots of a polynomial.
##    Roots are sorted in ascending order. Returns an empty list if no such roots.
##    """
##    roots = poly.r
##    times = [r for r in roots if r.imag == 0.000 and r.real >= 0]
##    times.sort()
##    return times


##class InvalidTimeError(Exception):
##    """Attempted to use pos_eqn outside of its valid time range."""
##
##class InvalidPosError(Exception):
##    """Attempted to use a pos outside of its valid range."""
##
##class TraversalFullError(Exception):
##    """A segment's finite duration indicates a segment distance greater than
##    the remaining room in the traversal."""
##
##class Times(object):
##    """An object for containing times (seconds).
##      `points` is a list of floats.
##      `ranges` is a list of (start, stop) pairs.
##    """
##    def __init__(self):
##        self.points = []
##        self.ranges = []
##
##    def __str__(self):
##        return 'Times: Points: %s. Ranges: %s' % (self.points, self.ranges)
##
##    def extend(self, t):
##        """Extends the points and ranges lists by adding data from another
##        Times instance."""
##        self.points.extend(t.points)
##        self.ranges.extend(t.ranges)

##def print_activeQs():
##    """For debug puposes. Prints out all vehicles that are holding each edge and node."""
##    print "ActiveQ printout - TrackSegments"
##    for e in common.track_segments.values():
##        print "  " + str(e), [str(v) for v in e.resource.activeQ]


##class FutureLoc(Node):
##    """A mock location indicating a location that has not yet been reached, and
##    where the location is not yet decided upon."""
##
##    length = inf
##    label = 'FutureLoc'
##
##    def __init__(self):
##        pass
##
##class Segment(traits.HasTraits):
##    """A `Segment` consists of a polynomial, the start time at which
##    the poly becomes valid, and the duration for which it remains valid.
##
##    Fundamental 'length' of a Segment is it's duration (seconds).
##    All times are treated by as absolute by default. The position eqn should be
##    formulated such that the expected time inputs are 0 through `duration`.
##
##    mass is the total mass of the vehicle. Used for force and energy
##    calculations.
##
##    Segment is valid from start_time (included) to end_time (excluded).
##    """
##    pos_eqn = traits.Instance(poly1d)
##    duration = traits.Either((traits.Float(), traits.Instance(inf)))
##    start_time = traits.Float()
##    mass = traits.Int()
##
##    def __init__(self, pos_eqn, duration, start_time, mass):
##        traits.HasTraits.__init__(self)
##        self.pos_eqn = pos_eqn
##        self.duration = duration  # seconds, relative. May be infinite. i.e. inf
##        self.start_time = start_time   # seconds, absolute
##        self.mass = mass
##
##    def __str__(self):
##        return "Segment: %s, duration %f, start %f, mass %i" % \
##            (self.pos_eqn.c, self.duration, self.start_time, self.mass)
##
##    def get_end_time(self):
##        return self.start_time + self.duration
##
##    end_time = property(get_end_time)
##
##    def get_start_pos(self):
##        """Returns the position at self.start_time."""
##        return self.pos_eqn(0)
###        return self.get_pos_from_time(self.start_time)
##
##    def get_end_pos(self):
##        """Returns the position at self.end_time."""
##        return self.get_pos_from_time(self.end_time)
##
##    def get_elapsed_time(self):
##        if Sim.now() < self.start_time:
##            return None
##        else:
##            t = Sim.now() - self.start_time
##            return min(t, self.duration)
##
##    def get_elapsed_dist(self):
##        """Sum of the absolute distance's traveled. If vehicle ever reversed,
##        may be greater than delta_position."""
##        raise NotImplementedError
##        # TODO: Find velocity zero-crossings, subsection Segment, find sum.
###        return self.pos_eqn(self.get_elapsed_time()) - self.pos_eqn(0)
##
##    def get_length(self):
##        return self.pos_eqn(self.duration) - self.pos_eqn(0)
##
##    length = property(get_length)
##
##    def is_current(self, time=None):
##        """Segment is valid at time. Range of segment is from start_time
##        (included) to end_time (excluded)."""
##        if time is None:
##            time = Sim.now()
##        if utility.time_ge(time, self.start_time) and \
##                utility.time_lt(time, self.end_time):
##            return True
##        else:
##            return False
##
##    def get_times_from_pos(self, pos, relative=False):
##        """Returns a Times instance containing the absolute time(s) (in seconds)
##        at which position is reached. Will only return times in the valid range
##        for the segment.
##
##        If the vehicle is stopped at pos (i.e. pos_eqn is 0th order and equals
##        pos), the Times.range list will contain a (start, stop) pair.
##
##        Returns an empty list if position is not reached.
##
##        If `relative` is True, will return the time relative to
##        Sim.now() rather than the absolute time.
##        """
##        ret = Times()
##        # stopped at pos
##        if self.pos_eqn.order == 0 and \
##                utility.dist_eql(self.pos_eqn[0], pos):
##            ret.ranges = [ (self.start_time, self.end_time) ]
##
##        else:
##            # Only real roots in the correct time range are valid
##            roots = (self.pos_eqn - pos).r
##            pts = [t.real for t in roots if
##                     utility.time_eql(t.imag, 0.0000) and \
##                     utility.time_ge(t.real, 0.0) and \
##                     utility.time_le(t.real, self.duration)]
##            pts.sort()
##
##            # Remove duplicates that arise from rounding errors
##            for i in reversed(xrange(1, len(pts))): # backward to 1
##                if utility.time_eql(pts[i], pts[i-1]):
##                    avg = (pts[i-1]+pts[i])/2.0
##                    del pts[i]
##                    pts[i-1] = avg
##
##            if relative:
##                offset = Sim.now() - self.start_time
##                ret.points = [t - offset for t in pts]
##
##            else:
##                ret.points = [t + self.start_time for t in pts]
##
##        return ret
##
##    def get_pos_from_time(self, time, rnd=False, relative=False):
##        """A wrapper for get_x_from_time."""
##        return self.get_x_from_time(POS, time, rnd, relative)
##
##    def get_x_from_time(self, x, time, rnd=False, relative=False):
##        """Returns x at time (seconds, absolute), where x is:
##          POS = 0
##          VEL = 1
##          ACCEL = 2
##          JERK = 3
##
##        If seg has an infinite duration, will return +/- inf if the
##        vehicle is moving at a constant speed, or will return the vehicle
##        position if stopped.
##
##        Raises InvalidTimeError if time is outside of segment's valid range.
##
##        If `relative` is True, will treat `time` as relative to
##        Sim.now() rather than as an absolute time.
##        """
##        if relative:
##            time = time + Sim.now()
##
##        # end_time is treated as valid
##        if utility.time_lt(time, self.start_time) or utility.time_gt(time, self.end_time):
##            raise InvalidTimeError, (time, self.start_time, self.end_time)
##
##        ret = None
##        if time != inf:
##            # Normal case. Get correct derivative and evaluate
##            ret = self.pos_eqn.deriv(x)(time - self.start_time)
##
##        else: # time == inf
##            # vehicle shouldn't have anything more complex than constant
##            #   velocity with infinite duration. However, we can safely disregard
##            #   tiny deviations from zero.
##            poly = self.pos_eqn
##            if self.pos_eqn.order > 1: # some accel and/or jerk
##                for coeff in poly.coeffs[:-2]: # raise error if accel or jerk aren't just rounding errors
##                    if abs(coeff) > 0.0001:
##                        raise InvalidTimeError, time
##                # If no error raised, use a clipped version of the poly for calcs
##                poly = poly1d(poly.coeffs[-2:])
##
##            # manually set values, since polynomial eval doesn't work on inf.
##            if x == POS:
##                if poly.order == 0 or abs(poly.coeffs[-2]) < 0.0001:
##                    # stopped, so all times have same pos
##                    ret =  poly(0)
##                else:
##                    # sign of velocity will dominate
##                    ret =  (inf if poly.coeffs[0] > 0 else -inf)
##            elif x == VEL:
##                ret = poly.c[0] # return velocity
##            else: # x == ACCEL or x == JERK
##                ret = 0
##
##        if rnd:
##            ret =  round(ret, common.DIST_RND)
##
##        return ret
##
##
##    def split_at_pos(self, pos):
##        """Splits the segment at pos and returns a pair of new segments. Split
##        is used at location boundries. Since each location starts at position 0,
##        split introduces a discontinuity in position between the two segments,
##        but not in velocity, acceleration, or jerk."""
##        times = self.get_times_from_pos(pos)
##        try:
##            time = times.points[0] # Only concerned with the first time it exceeds a length.
##        except IndexError:
##            raise InvalidPosError
##        dur1 = time - self.start_time
##        s1 = Segment(self.pos_eqn, dur1, self.start_time, self.mass)
##        assert utility.dist_eql(s1.get_end_pos(), pos)
##
##        s2_eqn = shift_poly(self.pos_eqn, -dur1, -pos)
##        dur2 = self.end_time - time
##        s2 = Segment(s2_eqn, dur2, time, self.mass)
##        assert utility.dist_eql(s2.pos_eqn(0), 0)
##
##        print "SPLIT DEBUG!"
##        print "s1.get_end_pos(): %f, pos: %f, error: %f" % (s1.get_end_pos(), pos, s1.get_end_pos() - pos)
##        print "s2.get_end_pos(): %f, self.get_end_pos()-pos: %f, error: %f" % (s2.get_end_pos(), self.get_end_pos()-pos, s2.get_end_pos() - (self.get_end_pos()-pos))
##        print "---"
##        return (s1, s2)
##
##    def split_at_time(self, time):
##        """Splits the segment at time and returns a pair of new segments. See
##        `split_at_pos`."""
##        if time < self.start_time or time > self.end_time:
##            raise InvalidTimeError, time
##        dur1 = time - self.start_time
##        s1 = Segment(self.pos_eqn, dur1, self.start_time, self.mass)
##        s1_end_pos = s1.get_pos_from_time(time, rnd=False)
##
##        s2_eqn = shift_poly(self.pos_eqn, -dur1, -s1_end_pos)
##        dur2 = self.end_time - time
##        s2 = Segment(s2_eqn, dur2, time, self.mass)
##        assert utility.dist_eql(s2.pos_eqn(0), 0)
##        return (s1, s2)
##
##    def collision_check(self, lv_seg, offset, relative=False):
##        """Checks for collisions with a lead vehicle. A collision
##        is detected if there is an intersection of my pos_eqn and the lead
##        vehicle's pos_eqn.
##
##        `lv_seg` is a Segment from the lead vehicle
##        `offset` is correction to the position to account for lead vehicle
##            length and differences between Traversal coordinate frames. See
##            Traversal.collision_check for more information.
##        `relative` controls if the times are returned as relative to Sim.now().
##
##        Returns a list of intersection times from the overlapping
##        valid times. If no collision will occur, returns an empty list.
##
##        Raises InvalidTimeError if lv_seg's valid time range has no
##        overlap with mine.
##        """
##        # align the start times
##        if self.start_time < lv_seg.start_time:
##            shift = self.start_time - lv_seg.start_time # neg, left shift
##            s_eqn = shift_poly(self.pos_eqn, shift, 0)
##            lv_eqn = shift_poly(lv_seg.pos_eqn, 0, offset)
##        elif self.start_time > lv_seg.start_time:
##            shift = lv_seg.start_time - self.start_time # neg, left shift
##            s_eqn = self.pos_eqn
##            lv_eqn = shift_poly(lv_seg.pos_eqn, shift, offset)
##        else:
##            s_eqn = self.pos_eqn
##            lv_eqn = shift_poly(lv_seg.pos_eqn, 0, offset)
##
##        start_time = max(self.start_time, lv_seg.start_time)
##        dur = min(self.end_time, lv_seg.end_time) - start_time
##        if utility.time_lt(dur, 0.0):
##            raise InvalidTimeError, "No overlap: (%s, %s), (%s, %s)" % \
##                (self.start_time, self.end_time, lv_seg.start_time, lv_seg.end_time)
##
##        intersect_eqn = s_eqn - lv_eqn
##        if intersect_eqn.order == 0: # jerk, accel, and vel are identical
##            if utility.dist_eql(intersect_eqn[0], 0): # position is the same
##                times = [0]
##            else:
##                times = []
##        else:                        # normal case
##            roots = intersect_eqn.r
##            times = [r for r in roots if r.imag == 0.000
##                                         and r.real >= 0
##                                         and r.real < dur]
##        if relative:
##            return [t + start_time - Sim.now() for t in times]
##        else:
##            return [t + start_time for t in times]
##
##class Traversal(traits.HasTraits):
##    """A `Traversal` is a sequence of segments that describes the vehicle's
##    trajectory as it traverses a location (that is, a TrackSegment).
##
##    Fundamental 'length' of a Traversal is the location's length."""
##    loc = traits.Instance(Node)
##
##    idx = traits.Either((traits.Int(), None))
##    segments = traits.List(traits.Instance(Segment))
##
##    def __init__(self, loc, idx):
##        traits.HasTraits.__init__(self)
##        self.loc = loc
##        self.idx = idx
##        try:
##            self.length = loc.length
##        except AttributeError:
##            self.length = 0
##        self.segments = []
##
##    def __str__(self):
##        tmp_list = ["Traversal: %s, len:%s, idx:%s\n" % \
##                    (self.loc.label, self.length, self.idx)]
##        tmp_list.extend( ["  %s\n" % seg for seg in self.segments] )
##        return ''.join(tmp_list)
##
##    def __iter__(self):
##        return iter(self.segments)
##
##    def get_start_time(self):
##        if self.segments:
##            return self.segments[0].start_time
##
##    start_time = property(get_start_time)
##
##    def get_end_time(self):
##        if self.segments:
##            return self.segments[-1].end_time
##
##    end_time = property(get_end_time)
##
##    def get_elapsed_time(self):
##        t = Sim.now() - self.start_time
##        assert utility.time_eql( t, sum([seg.get_elapsed_time() for seg in self.segments]) )
##        return t
##
##    def get_elapsed_dist(self):
##        """Sum of the absolute distance's traveled. If vehicle ever reversed,
##        may be greater than the location length."""
##        raise NotImplementedError
##        # TODO: Do Segment version of func.
###        dist = sum( [abs(seg.get_elapsed_dist()) for seg in self.segments] )
###        assert dist <= self.length
###        return dist
##
##    def is_current(self, time=None):
##        """Traversal is valid at time. Valid range is from start_time (included)
##        to end_time (excluded)."""
##        if time is None:
##            time = Sim.now()
##        if utility.time_ge(time, self.start_time) and \
##                utility.time_lt(time, self.end_time):
##            return True
##        else:
##            return False
##
##    def get_segment_from_time(self, time=None):
##        """Returns the segment that will be the current segment at time.
##
##        If time is ommitted, returns the segment at Sim.now()
##
##        Raises InvalidTimeError if no segment in the traversal is current at time.
##        """
##        if time is None:
##            time = Sim.now()
##        if not self.segments:
##            raise InvalidTimeError, time
##        if not self.is_current(time):
##            raise InvalidTimeError, time
##        for seg in reversed(self.segments): # common case is lastest seg
##            if seg.is_current(time):
##                return seg
##
##    def get_times_from_pos(self, pos, relative=False):
##        """Returns a Times instance containing the absolute time(s) (in seconds)
##        at which position is reached.
##
##        If the vehicle stopped at pos, the Times.range list will contain one
##        or more (start, stop) pairs.
##
##        Raises InvalidPosError if pos is beyond the location's length.
##
##        If `relative` is True, will return the time relative to
##        Sim.now() rather than the absolute time.
##        """
##        if utility.dist_lt(pos, 0) or utility.dist_gt(pos, self.length):
##            raise InvalidPosError, pos
##
##        times = Times()
##        for seg in self.segments:
##            times.extend( seg.get_times_from_pos(pos, relative) )
##
##        # Remove duplicates. A pos on a segment boundry will receive identical
##        # times from each segment.
##        for i in reversed(xrange(1, len(times.points))): # backward to 1
##            if utility.time_eql(times.points[i], times.points[i-1]):
##                avg = (times.points[i-1]+times.points[i])/2.0
##                del times.points[i]
##                times.points[i-1] = avg
##
##        return times
##
##    def get_x_from_time(self, x, time, rnd=True, relative=False):
##        """Returns x at time (seconds, absolute), where x is one of:
##          POS = 0
##          VEL = 1
##          ACCEL = 2
##          JERK = 3
##
##        Raises InvalidTimeError if vehicle was not in location at that time.
##
##        `rnd` controls if the position is rounded.
##
##        If `relative` is True, will treat `time` as relative to
##        Sim.now() rather than as an absolute time.
##        """
##        if relative:
##            abs_time = Sim.now() + time
##        else:
##            abs_time = time
##
##        if utility.time_lt(abs_time, self.start_time) or \
##                utility.time_gt(abs_time, self.end_time):
##            raise InvalidTimeError, time
##
##        for seg in self.segments:
##            try:
##                return seg.get_x_from_time(x, time, rnd, relative)
##            except InvalidTimeError:
##                pass
##
##    def get_pos_from_time(self, time, rnd=True, relative=False):
##        """Wraps get_x_from_time."""
##        return self.get_x_from_time(POS, time, rnd, relative)
##
##    def append_segment(self, seg):
##        """Append one segment to a Traversal.
##
##        Raises InvalidTimeError if seg.start_time is not greater
##        than the tail segment's start time.
##
##        Raises TraversalFullError if `seg` has an end position beyond the
##        location's length.
##        """
##        assert utility.time_ge(seg.start_time, Sim.now())
##        assert utility.dist_ge(seg.get_start_pos(), 0)
##
##        # Ensure that segment really starts in loc
##        if utility.dist_ge(seg.get_start_pos(), self.length):
##            raise InvalidPosError, seg.get_start_pos()
##
##        # Ensure that the segment doesn't go beyond the location length.
##        if utility.dist_gt(seg.get_end_pos(), self.length):
##            raise TraversalFullError
##
##        if self.segments:
##            assert utility.dist_eql(self.segments[-1].get_end_pos(), seg.get_start_pos())
##
##        self.segments.append(seg)
##
##    def clear(self, time):
##        """Clears the traversal from time onwards."""
##        assert utility.time_ge(time, Sim.now()) # don't change the past.
##        if not self.segments:
##            return
##
##        if utility.time_le(time, self.start_time): # at start or before. Wipe everything.
##            self.segments = []
##            return
##
##        if utility.time_ge(time, self.end_time): # doesn't apply to me
##            return
##
##        # Find the relevant segment
##        for seg in self.segments:
##            if seg.is_current(time):
##                s = seg
##                break
##
##        # replace segment with a fractional seg, then delete rest.
##        idx = self.segments.index(s)
##        s1, s2 = s.split_at_time(time)
##        self.segments[idx] = s1
##        del self.segments[idx+1:]
##
##    def collision_check(self, lv_trav, lv_length, trav_offset=0,
##                        time=None, relative=False):
##        """Checks a traversal for collisions with a leading vehicle.
##
##        `lv_trav` is a Traversal instance from the lead vehicle. The traversal
##            should be the one occupied by the lead vehicle's nose. If the
##            location of the lv nose is different than the tail, supply a
##            trav_offset which is the sum of the lengths for all locations from
##            the tail loc to the nose loc.
##
##                lv_tail_pos = lv_nose_pos - lv_length + trav_offset
##
##            Example:
##                                                 lv_length: 50
##                                                 lv_nose_pos: 35
##                            lv                   trav_offset: A + B = 25
##                       ***************           lv_tail_pos: 35-50+25 = 10
##
##                |___A____|__B__|____C_____|      traversals: A - 15
##                0       15     10        100                 B - 10
##                                                             C - 100
##
##                collision_check(C, 50, 25)
##
##
##        `lv_length' is the length of the lead vehicle
##        `time` (optional) is the earliest time that will be checked.
##        `relative` controls whether the returned times are relative to Sim.now()
##
##        Returns a list containing times of collisions. If no collisions found,
##        returns an empty list.
##        """
##        times = []
##        lv_iter = iter(lv_trav.segments)
##        self_iter = iter(self.segments)
##        try:
##            lv_seg = lv_iter.next()
##            self_seg = self_iter.next()
##        except StopIteration:
##            raise Exception # one or the other has no segments
##
##        # if time supplied, don't bother comparing segments that end before time
##        if time is not None:
##            try:
##                while lv_seg.end_time < time:
##                    lv_seg = lv_iter.next()
##
##                while self_seg.end_time < time:
##                    self_seg = self_iter.next()
##            except StopIteration:
##                raise Exception # bad mojo
##
##        # combine lv_length and trav_offset into a single offset value
##        offset = trav_offset - lv_length
##
##        # check for rear end collisions on all segments
##        while True:
##            times.extend( self_seg.collision_check(lv_seg, offset, relative) )
##            try:
##                if lv_seg.end_time < self_seg.end_time:
##                    lv_seg = lv_iter.next()
##                elif lv_seg.end_time > self_seg.end_time:
##                    self_seg = self_iter.next()
##                else:
##                    lv_seg = lv_iter.next()
##                    self_seg = self_iter.next()
##            except StopIteration:
##                break
##
##        # if time supplied, prune any times that occured before it.
##        if time:
##            times = [t for t in times if t >= time]
##
##        return times
##
##class Path(traits.HasTraits):
##    """A path is a sequence of Traversals describing the past and future
##    movement of the vehicle. `future` is a Traversal holding Segments describing
##    the planned trajectory of the vehicle, without indicating any planned
##    location.
##
##    vehicle is a reference to the Vehicle object it is the path for.
##    """
##    traversals = traits.List(traits.Instance(Traversal))
##    future = traits.Instance(Traversal)
##    vehicle = traits.Instance(BaseVehicle)
##
##    def __init__(self, loc, seg, vehicle):
##        traits.HasTraits.__init__(self)
##        assert seg.start_time == 0
##        self.traversals.append(Traversal(loc=loc, idx=0))
##        self.future = Traversal(loc=FutureLoc(), idx=None)
##        trav = self.traversals[0]
##        self.vehicle = vehicle
##        try:
##            trav.append_segment(seg)
##        except TraversalFullError:
##            s1, s2 = seg.split_at_pos(trav.length)
##            trav.append_segment(s1)
##            self.future.append_segment(s2)
##        self.active_idx = 0
##
##    def __str__(self):
##        tmp_list = ['Path-Past:\n']
##        tmp_list.extend( [' %s' % trav for trav in self.traversals] )
##        tmp_list.append( 'Path-Future\n' )
##        tmp_list.append( ' %s' % self.future )
##        return ''.join(tmp_list)
##
##    def __iter__(self):
##        return itertools.chain(self.traversals, (self.future,))
##
##    def __len__(self):
##        return len(self.traversals) + 1
##
##    def __getitem__(self, key):
##        tmp = self.traversals + [self.future]
##        return tmp[key]
##
##    def get_active_traversal(self):
##        """Returns the traversal marked as active. `Active` means
##        that the sim 'thinks' it's on this traversal. In contrast, `current`
##        refers to the segment or traversal that the sim should be on, based on
##        the simulation time."""
##        return self.traversals[self.active_idx]
##
##    def get_active_loc(self):
##        """Returns the location of the active traversal."""
##        return self.traversals[self.active_idx].loc
##
##    def append_segment(self, seg, trav):
##        """Appends a single segment to the path. If the segment stradles a
##        traversal boundary, splits the segment and returns the unassigned part.
##        If the segment is beyond the traversal, returns the segment. Otherwise,
##        returns None."""
##        try:
##            trav.append_segment(seg)
##        except TraversalFullError: # Only part of the segment fits
##            s1, s2 = seg.split_at_pos(trav.length)
##            trav.append_segment(s1)
##            return s2
##        except InvalidPosError:
##            return seg
##
##
##    def append_segments(self, segs, trav):
##        """Appends one or more Segments to the path. Assumed that the Segments
##        are contiguous, and that the Traversals are empty.
##
##        segs: A list containing Segment instances.
##        trav: The Traversal instance which corresponds to the first Segment.
##        """
##        if not segs:
##            return
##
##        dist_offset = 0
##        for seg in segs:
##            # Offset the position to correct for each traversal starting with pos=0
##            seg.pos_eqn[0] -= dist_offset
##
##            while True:
##                seg = self.append_segment(seg, trav)
##                if seg:
##                    dist_offset += trav.length
##                    try:
##                        trav = self.traversals[trav.idx+1]
##                    except IndexError:
##                        trav = self.future
##                else:
##                    break
##
##    def append_loc(self, loc):
##        """Appends a new location to the path. Applies any planned segments to
##        it (up to its length).
##        """
##        idx = len(self.traversals)
##        trav = Traversal(loc, idx)
##        self.traversals.append(trav)
##        future_segs = self.future.segments
##
##        # reset self.future
##        self.future = Traversal(loc=FutureLoc(), idx=None)
##
##        # using append_segments ensures that loc is not 'overfilled' with
##        # segments, and stores any excess segments in self.future again.
##        self.append_segments(future_segs, trav)
##
##    def get_pos_loc_from_time(self, time, rnd=True, relative=False):
##        """A wrapper for get_x_loc_from_time"""
##        return self.get_x_loc_from_time(POS, time, rnd, relative)
##
##    def get_x_loc_from_time(self, x, time, rnd=True, relative=False):
##        """Returns a pair containing (x, loc) if found, where x is one of:
##          POS = 0
##          VEL = 1
##          ACCEL = 2
##          JERK = 3
##        Raises InvalidTimeError if not found.
##        """
##        trav = self.get_traversal_from_time(time)
##        value = trav.get_x_from_time(x, time, rnd, relative)
##        loc = trav.loc
##        return (value, loc)
##
##    def get_eta_next_loc(self, relative=False):
##        """Gets the estimated time to arrive at next location.
##
##        If relative=True, time given as difference from now.
##        """
##        trav = self.get_traversal_from_time() # TODO: Use active_traversal instead???
##        t = trav.end_time
##        if relative:
##            t = t - Sim.now()
##        return t
##
##
##    def get_times_from_pos_loc(self, pos, loc=None, relative=False):
##        """Returns all times that vehicle has reached pos in loc.
##
##        If loc is ommitted, then times are provided for the vehicle's current
##        location only. Past visits the location are not included.
##
##        If loc is provided and is the current location, times may be found for
##        positions not yet reached, as well as all previous visits. If loc is
##        provided and is not the current location, times will be for previous
##        visits.
##
##        If loc is a FutureLoc instance, times will only be found from
##        self.path.future.
##
##        In all cases, times are in ascending sorted order, and are returned as
##        a Times object.
##        """
##
##        if isinstance(loc, FutureLoc):
##            times = self.future.get_times_from_pos(pos, relative)
##        elif loc:
##            times = Times()
##            for trav in itertools.ifilter(lambda x: x.loc is loc, self.traversals):
##                times.extend(trav.get_times_from_pos(pos, relative))
##        else:
##            times = self.traversals[-1].get_times_from_pos(pos)
##        return times
##
##    def get_traversal_from_time(self, time=None):
##        """Returns the traversal that should be current at time."""
##        if time is None:
##            time = Sim.now()
##        if self.future.is_current(time):
##            return self.future
##        else:
##            for trav in reversed(self.traversals): # common case is lastest seg
##                if trav.is_current(time):
##                    return trav
##
##        raise InvalidTimeError, time
##
##    def get_loc_from_time(self, time=None):
##        """Returns the location that is 'current' at time."""
##        return self.get_traversal_from_time(time).loc
##
##    def clear_planned_route(self):
##        """Clears planned but unreached locations.
##        Does not change the trajectory. Will leave segments fragmented, but
##        contiguous.
##        """
##        current_trav = self.get_traversal_from_time()
##        assert current_trav is not self.future
##        # No planned, unreached traversals
##        if current_trav is self.traversals[-1]:
##            pass # nothing to do
##        else:
##            idx = self.traversals.index(current_trav)
##
##            # remove position adjustments on planned segments
##            offset = 0
##            new_segs = []
##            mass = self.vehicle.total_mass
##            for trav in self.traversals[idx+1:]:
##                for seg in trav.segments:
##                    new_seg = Segment(shift_poly(seg.pos_eqn, 0, offset),
##                                      seg.dur, seg.start_time, mass)
##                    new_segs.append(new_seg)
##                offset += trav.length
##            for seg in self.future.segments:
##                new_seg = Segment(shift_poly(seg.pos_eqn, 0, offset),
##                                  seg.dur, seg.start_time, mass)
##                new_segs.append(new_seg)
##
##            self.future.segments = new_segs
##            del self.traversals[idx+1:]
##
##    def get_latest_traversal_from_loc(self, loc):
##        """Returns the most recent traversal over location `loc`."""
##        for trav in reversed(self.traversals):
##            if trav.loc is loc:
##                return trav
##
##    def change_trajectory(self, segments):
##        """Invalidates previously planned segments and replaces them.
##        Segements should be an iterable (e.g. tuple or list) containing
##        Segment objects."""
##        t = segments[0].start_time
##        assert utility.time_ge(t, Sim.now()) # don't change the past
##        current_trav = self.get_traversal_from_time(t)
##
##        # clear planned trajectory past time t
##        for trav in self.traversals[current_trav.idx:]:
##            trav.clear(t)
##
##        self.future = Traversal(loc=FutureLoc(), idx=None) # clear out future
##        self.append_segments(segments, current_trav)

#    def collision_check(self, lv_path, lv_length, time=None, relative=False):
#        """UNUSED and not debugged!! Path level collision checking seems
#        to be unnecessary. Remove the 'raise NotImplementedError' to reactivate.
#
#        Checks a traversal for collisions with a leading vehicle.
#
#        `lv_path` is the Path instance from the lead vehicle.
#        `lv_length' is the length of the lead vehicle
#        `time` (optional) is the earliest time that will be checked.
#
#        Returns a list containing times of collisions. If no collisions found,
#        returns an empty list.
#
#        Checks starting at time, continues until it reaches a merge, switch, or
#        station, then stops. It checks the merge or switch, but does not check
#        the station.
#        Merges introduce the possibility of the lead vehicle changing. Switches
#        introduce the possibility of the locations diverging. Stations are waaay
#        special.
#        """
#        raise NotImplementedError
#        # figure out what Traversals to start from
#        if time:
#            s_idx = self.get_traversal_from_time(time).idx
#            lv_idx = lv_path.get_traversal_from_time(time).idx
#        else:
#            s_idx = 0
#            lv_idx = 0
#
#        # setup lists of Traversals to compare
#        if s_idx is None: # idx of None indicates FutureLoc
#            s_travs = []
#        else:
#            s_travs = self.traversals[s_idx:]
#        if lv_idx is None:
#            lv_travs = []
#        else:
#            lv_travs = lv_path.traversals[lv_idx:]
#
#        # find collision times
#        times = []
#        for s_tr, lv_tr in map(None, s_travs, lv_travs):
###            if isinstance(s_tr, Station) or isinstance(lv_tr, Station):
###                break
#
#            if s_tr and lv_tr:
#                times.extend(s_tr.collision_check(lv_tr, lv_length, time, relative))
#            elif s_tr: # just s_tr, no lv_tr
#                times.extend(s_tr.collision_check(lv_path.future, lv_length, time, relative))
#            else:     # just lv_t, no s_t
#                times.extend(self.future.collision_check(lv_tr, lv_length, time, relative))
#
##            if isinstance(s_tr, Switch) or (lv_tr, Switch):
##                break
#        times.extend(self.future.collision_check(lv_path.future, lv_length, time, relative))
#
#        return times

##    def plot(self, title='path.plot()', show=True, new_fig=True):
##        """Plots the whole path using matplotlib. The x-axis is time, the
##        y-axis is velocity. Vertical blue lines indicate location boundries.
##        The vertical red line is the current time."""
##        if new_fig:
##            pyplot.figure()
##        step_size = 0.125 # chosen to be expressed exactly in floating point
##        t = 0
##        times = []
##        positions = []
##        velocities = []
##        loc_boundries = [trav.end_time for trav in self.traversals]
##        for trav in self.traversals:
##            times.append(trav.start_time)
##            positions.append(trav.segments[0].get_x_from_time(POS, trav.start_time))
##            velocities.append(trav.segments[0].get_x_from_time(VEL, trav.start_time))
##            for seg in trav.segments:
##                stop = seg.end_time
##                if stop == inf:
##                    stop = Sim._endtime
##                while t <= stop:
##                    times.append(t)
##                    positions.append(seg.get_x_from_time(POS, t))
##                    velocities.append(seg.get_x_from_time(VEL, t))
##                    t += step_size
##            times.append(stop)
##            positions.append(trav.segments[-1].get_x_from_time(POS, trav.end_time))
##            velocities.append(trav.segments[-1].get_x_from_time(VEL, trav.end_time))
##
##        for seg in self.future.segments:
##            if seg.end_time == inf:
##                stop = seg.start_time
##            else:
##                stop = seg.end_time
##            times.append(seg.start_time)
##            positions.append(seg.get_x_from_time(POS, seg.start_time))
##            velocities.append(seg.get_x_from_time(VEL, seg.start_time))
##            while t <= stop:
##                times.append(t)
##                positions.append(seg.get_x_from_time(POS, t))
##                velocities.append(seg.get_x_from_time(VEL, t))
##                t += step_size
##
##        pos_line = pyplot.plot(times, positions)
##        vel_line = pyplot.plot(times, velocities)
##        now_line = pyplot.axvline(Sim.now(), linewidth=2, color='r')
##        for b in loc_boundries:
##            pyplot.axvline(b)
##        pyplot.title(title)
##        pyplot.figlegend( (pos_line, vel_line, now_line),
##                          ('Position', 'Velocity', 'Now'),
##                          'upper right')
##        axes = pyplot.gca()
##        axes.set_xlim([times[0], times[-1]])
##        if show:
##            pyplot.show()

##    def get_dist_travelled(self, time=None):
##        """Makes the assumption that all travel is in a forward direction. TODO: Implement and use get_elapsed_dist()."""
##        if time is None:
##            time = Sim.now()
##
##        # Break the path into three sections: the first traversal, the whole middle set of traversals, and the active traversal.
##        # The first may not start at 0, and the last may not be completed.
##        start_trav = self.traversals[0]
##        active_trav = self.get_active_traversal()
##
##        if start_trav.idx == active_trav.idx:
##            dist = start_trav.get_pos_from_time(time) - start_trav.get_pos_from_time(0)
##        else:
##            start_dist = start_trav.length - start_trav.segments[0].get_start_pos()
##            middle_dist = sum(trav.length for trav in self.traversals[1:active_trav.idx])
##            end_dist = active_trav.get_pos_from_time(time)
##            dist = start_dist + middle_dist + end_dist
##        return dist



#class Switch(Node):
#    """A track Switch. Length should be considered the amount of
#    downstream track in which two pods cannot be 'shoulder-to-shoulder' due to
#    their width (plus safety margins).
#    """
#
#    routing_table = traits.Dict
#
#    traits_view =  ui.View(ui.Item(name='ID'),
#                    ui.Item(name='label'),
#                    ui.Item(name='length'),
#                    ui.Item(name='max_speed'),
#                    ui.Item(name='enabled'),
#                    ui.Item(name='routing_table')
#                   )
#
#    def __init__(self, ID, length, max_speed, x_start=0, y_start=0,
#                 x_end=0, y_end=0, label='',
#                 src_edge=None, sink_edge1=None, sink_edge2=None,
#                 default=None, **tr):
#        Node.__init__(self, ID, x_start, y_start, x_end, y_end, length, max_speed, SWITCH_CAPACITY, label, **tr)
##        self.src_edge = src_edge
##        self.sink_edge1 = sink_edge1
##        self.sink_edge2 = sink_edge2
##        self.default = default
#        # Routing table consists of vehicle:(edge, msgID) pairs,
#        # where msgID is the CtrlCmdSwitch message.
#        self.errors=0
#
#    def __str__(self):
#        return self.label
#
#    def get_outbound_edges(self):
#        """Returns the outbound edge instances.
#        TEMP: Using graph lookup each time."""
#        return [data for src, sink, data in common.digraph.edges(self, data=True)]
#
#    def add_rt_entry(self, veh, edge, msgID):
#        """Adds a vehicle: (edge, msgID) entry to the routing table. If an
#        entry for the vehicle is already present, it is overwritten. Does no
#        validation.
#        """
#        self.routing_table[veh] = (edge, msgID)
#    def del_rt_entry(self, veh):
#        """Deletes a routing table entry for vehicle."""
#        del self.routing_table[veh]
#    def get_rt_entry(self, veh):
#        """Returns an (edge, msgID) tuple for the vehicle, or """
#        return self.routing_table.get(veh)
#
#    def fill_SwitchStatus(self, sws):
#        """Fills an api.SwitchStatus instance with current information."""
#        sws.swID = self.ID
#        if self.label:
#            sws.label = self.label
#        for key, value in self.routing_table.iteritems():
#            # routing entries
#            re = sws.entry.add()
#            re.vID = key
#            re.eID = value[0].ID
#            re.msgID = value[1]


##def loc2loctype(loc):
##    """Returns the integer representing the location type for loc."""
##    ret = None
##    if isinstance(loc, TrackSegment):
##        ret = api.TRACK_SEGMENT
##    elif isinstance(loc, station.Station):
##        ret = api.STATION
##    elif isinstance(loc, BaseVehicle):
##        ret = api.VEHICLE
##    else:
##        raise TypeError, "Unknown location type."
##    return ret
