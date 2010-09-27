import unittest
import tempfile
import pyprt.ctrl.prt_controller as prt

import networkx

from pyprt.ctrl.prt_controller import Vehicle

class MockManager(object):
    def get_path(self, ts_1, ts_2):
        """Mocks out path info for station generated by the 'make_station_I'
        test function."""
        if ts_1 == 10: # OFF_RAMP_I
            if ts_2 == 10:
                return (0, [10])
            elif ts_2 == 13: # UNLOAD
                # offramp, [OFF_RAMP_I, OFF_RAMP_II, DECEL, UNLOAD]
                return (40, [10,11,12,13])
            elif ts_2 == 14: # QUEUE
                # offramp + unload, [OFF_RAMP_I, OFF_RAMP_II, DECEL, UNLOAD, QUEUE]
                return (40+45, [10,11,12,13,14])
            elif ts_2 == 15: # LOAD
                # offramp + unload + queue, [OFF_RAMP_I, OFF_RAMP_II, DECEL, UNLOAD, QUEUE, LOAD]
                return (40+45+45, [10,11,12,13,14,15])
            else:
                raise NotImplementedError
        elif ts_1 == 13: # UNLOAD
            if ts_2 == 15: # LOAD
                # unload + queue, [UNLOAD, QUEUE, LOAD]
                return (45+45, [13,14,15])
            else:
                raise NotImplementedError
        else:
            raise NotImplementedError

    def coordinate_shift(self, ts_id_source, ts_id_dest, path):
        if ts_id_source == 10: # station_I OFF_RAMP_I
            if ts_id_dest == 10:  # station_I OFF_RAMP_I
                return 0
            else:
                raise NotImplementedError
        else:
            raise NotImplementedError

class MockController(object):
    LINE_SPEED = 16
    current_time = 0.0
    sim_end_time = 3600.0

    def set_v_notification(self, vehicle, time):
        """Does nothing."""

    def send(self, msg_type, msg):
        """Does nothing."""

class TestVehicle(unittest.TestCase):
    def setUp(self):
        prt.Station.controller = MockController()
        prt.Vehicle.controller = MockController()
        prt.Vehicle.manager = MockManager()
        prt.Station.SPEED_LIMIT = 2.5

    def test_enter_station(self):
        """Depends on 'Station.request_load_berth' functioning correctly."""
        s = make_station_I()
        START_POS = 4 # same as length of vehicle. Tail just inside station OFF_RAMP_I.
        v0 = make_vehicle(0, s.ts_ids[s.OFF_RAMP_I], START_POS)
        v0.trip = prt.Trip(s, None, [10,11,12,13,14,15], tuple())
        s.request_load_berth(v0) # gives the last berth of the LOAD platform.
        old_v0_pos = v0.pos
        v0.enter_station()
        self.assertEqual(v0.spline.q[0], START_POS)
        self.assertAlmostEqual(v0.spline.q[-1], s.onramp_length + 45 + 45 + 45 - v0.BERTH_GAP, places=4)

    def test_run(self):
        t = make_Track()
        v = make_vehicle(0, 0, 5) # ts:0, pos:5
        v.set_path([0,1,2,3,4,5,6], send=False)
        v.run()
        spline = v.get_spline()

        # Reached slow segment
        idx = spline.q.index(t.get_path_length([0,1,2,3]))
        time = spline.t[idx]
        knot = spline.evaluate(time)
        self.assertAlmostEqual(knot.vel, 5)
        self.assertAlmostEqual(knot.accel, 0)

        # 2nd slow segment
        idx = spline.q.index(t.get_path_length([0,1,2,3,4,5,6]))
        time = spline.t[idx]
        knot = spline.evaluate(time)
        self.assertAlmostEqual(knot.vel, 5)
        self.assertAlmostEqual(knot.accel, 0)

class TestMerge(unittest.TestCase):
    """Tests for the Merge class"""

class TestStation(unittest.TestCase):
    def setUp(self):
        prt.Station.controller = MockController()
        prt.Vehicle.controller = MockController()
##
##    def tearDown(self):
##        pass

    def test_request_berth_on_platform(self):
        s = make_station_I()

        # Request an unload berth in an empty station. Should get last unload berth.
        ts_id = s.ts_ids[prt.Station.UNLOAD]
        v0 = make_vehicle(0, 0, 0) # actual ts_id, pos don't matter
        req_pos, req_b_id, req_p_id, req_ts_id = s._request_berth_on_platform(v0, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(req_pos, s.berth_positions[prt.Station.UNLOAD_PLATFORM][-1])
        self.assertEqual(req_b_id, 2) # last berth
        self.assertEqual(req_p_id, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(req_ts_id, s.ts_ids[prt.Station.UNLOAD])

        # Add a vehicle reservation for the middle unload berth
        pos = s.berth_positions[prt.Station.UNLOAD_PLATFORM][1] - 0.1
        ts_id = s.ts_ids[prt.Station.UNLOAD]
        v1 = make_vehicle(1, ts_id, pos)
        s.reservations[prt.Station.UNLOAD_PLATFORM][1] = v1

        # Request another berth. Should get the first berth.
        v2 = make_vehicle(2, s.ts_ids[prt.Station.OFF_RAMP_I], 0)
        req_pos, req_b_id, req_p_id, req_ts_id =  s._request_berth_on_platform(v2, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(req_pos, s.berth_positions[prt.Station.UNLOAD_PLATFORM][0])
        self.assertEqual(req_b_id, 0)
        self.assertEqual(req_p_id, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(req_ts_id, s.ts_ids[prt.Station.UNLOAD])

        # Check that re-requesting a berth fails, since the vehicle can't move
        # forward. Check that the vehicle does not lose its reservation.
        self.assertRaises(prt.NoBerthAvailableError, s._request_berth_on_platform, v2, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(v2.berth_id, 0)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM], [v2, v1, v0])

    def test_request_unload_berth(self):
        s = make_station_I()
        v0 = make_vehicle(1, s.ts_ids[prt.Station.DECEL], 0) # Create on DECEL
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_unload_berth(v0)
        self.assertEqual(v0.berth_pos, req_pos)
        self.assertEqual(v0.berth_id, req_b_id)
        self.assertEqual(v0.platform_id, req_p_id)
        self.assertEqual(v0.plat_ts, req_ts_id)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM][2], v0)

        v1 = make_vehicle(2, s.ts_ids[prt.Station.OFF_RAMP_II], 0) # Create on OFF_RAMP_II
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_unload_berth(v1)
        self.assertEqual(v1.berth_pos, req_pos)
        self.assertEqual(v1.berth_id, req_b_id)
        self.assertEqual(v1.platform_id, req_p_id)
        self.assertEqual(v1.plat_ts, req_ts_id)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM][1], v1)

        v2 = make_vehicle(2, s.ts_ids[prt.Station.OFF_RAMP_I], 5) # Create on OFF_RAMP_I
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_unload_berth(v2)
        self.assertEqual(v2.berth_pos, req_pos)
        self.assertEqual(v2.berth_id, req_b_id)
        self.assertEqual(v2.platform_id, req_p_id)
        self.assertEqual(v2.plat_ts, req_ts_id)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM][0], v2)

        # No room left. Raises NoBerthAvailableError
        v3 = make_vehicle(2, s.ts_ids[prt.Station.OFF_RAMP_I], 0) # Create on OFF_RAMP_I
        self.assertRaises(prt.NoBerthAvailableError, s.request_unload_berth, v3)

        # Remove the vehicle from the last berth and let v1 re-request.
        # v1's reservation should be switched to the last berth (berth_id 2)
        s.release_berth(v0)
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_unload_berth(v1)
        self.assertEqual(v1.berth_id, 2)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM][1], None)

        # Even though middle berth is open, it is not accessible to vehicles outside
        # of station and should not be available for reservation.
        v4 = make_vehicle(4, s.ts_ids[prt.Station.OFF_RAMP_I], 0) # pos doesn't matter
        self.assertRaises(prt.NoBerthAvailableError, s.request_unload_berth, v4)

        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM], [v2, None, v1])

    def test_request_load_berth(self):
        """Test depends on Station.request_unload_berth and Station.release_berth
        working correctly.
        """
        s = make_station_I()
        v0 = make_vehicle(0, s.ts_ids[prt.Station.DECEL], 0) # Create on DECEL
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_load_berth(v0)
        self.assertEqual(v0.berth_id, req_b_id)
        self.assertEqual(s.reservations[s.LOAD_PLATFORM][-1], v0)

        # Make a call to request_unload_berth which blocks everything past the UNLOAD plat.
        v1 = make_vehicle(1, s.ts_ids[prt.Station.OFF_RAMP_II], 0)
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_unload_berth(v1)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM][-1], v1)

        # Next request_load_berth should be limited to UNLOAD plat.
        v2 = make_vehicle(2, s.ts_ids[prt.Station.OFF_RAMP_I], 5)
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_load_berth(v2)
        self.assertEqual(v2.berth_id, req_b_id)
        self.assertEqual(v2.platform_id, s.UNLOAD_PLATFORM)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM][-2], v2)

        # add one more vehicle behind
        v3 = make_vehicle(3, s.ts_ids[prt.Station.OFF_RAMP_I], 0) # Create on OFF_RAMP_I
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_load_berth(v3)

        # clear v2 from reservations.
        s.release_berth(v2)
        self.assertEqual(s.reservations[s.UNLOAD_PLATFORM], [v3, None, v1])

        # re-request load berth for v1. Should get the middle LOAD_PLATFORM berth.
        req_pos, req_b_id, req_p_id, req_ts_id = s.request_load_berth(v1)
        self.assertEqual(req_b_id, 1)
        self.assertEqual(req_p_id, s.LOAD_PLATFORM)
        self.assertEqual(s.reservations[s.LOAD_PLATFORM], [None, v1, v0])

class TestTracks(unittest.TestCase):
    def test_get_speed_zones(self):
        t = make_Track()

        self.assertEqual(t.get_speed_zones([0,1,2,3,4,5,6]),
                         [(10, [0,1,2]), (5, [3]), (10, [4,5]), (5, [6])])

        self.assertEqual(t.get_speed_zones([0,1,2,3,4,5]),
                         [(10, [0,1,2]), (5, [3]), (10, [4,5])])

        self.assertEqual(t.get_speed_zones([0,1,2,3,4]),
                         [(10, [0,1,2]), (5, [3]), (10, [4])])

        self.assertEqual(t.get_speed_zones([0,1,2,3]),
                         [(10, [0,1,2]), (5, [3])])

        self.assertEqual(t.get_speed_zones([0,1]),
                         [(10, [0,1])])

        self.assertEqual(t.get_speed_zones([0]),
                         [(10, [0])])

        self.assertEqual(t.get_speed_zones([]), [])

    def test_get_path_length(self):
        t = make_Track()
        self.assertEqual(t.get_path_length([0,1,2,3,4,5]), 100+50+2+20+100)
        self.assertEqual(t.get_path_length([0,1,2,3,4]), 100+50+2+20)
        self.assertEqual(t.get_path_length([0,1,2,3]), 100+50+2)
        self.assertEqual(t.get_path_length([0,1]), 100)
        self.assertEqual(t.get_path_length([0]), 0)
        self.assertEqual(t.get_path_length([]), 0)

def make_Track():
    """Not a thorough implementation. Just creates a graph and tacks it onto the
    Tracks singleton."""
    graph = networkx.DiGraph()
    graph.add_edge(0,1, {prt.Tracks.MAX_SPEED:10, prt.Tracks.LENGTH:100})
    graph.add_edge(1,2, {prt.Tracks.MAX_SPEED:10, prt.Tracks.LENGTH:50})
    graph.add_edge(2,3, {prt.Tracks.MAX_SPEED:10, prt.Tracks.LENGTH:2})
    graph.add_edge(3,4, {prt.Tracks.MAX_SPEED:5,  prt.Tracks.LENGTH:20})
    graph.add_edge(4,5, {prt.Tracks.MAX_SPEED:10, prt.Tracks.LENGTH:100})
    graph.add_edge(5,6, {prt.Tracks.MAX_SPEED:10, prt.Tracks.LENGTH:100})
    graph.add_edge(6,0, {prt.Tracks.MAX_SPEED:5,  prt.Tracks.LENGTH:1})
    t = prt.Tracks()
    t._graph = graph
    return t

def make_station_I():
    """Make a station with 3 unload, 3 queue, and 3 loading berths. Berths
    are at 15 meter spacings."""
    return prt.Station(0, (10,11,12,13,14,15,16,17), 8, 9, 18, 40,
                       (15, 30, 45), (15, 30, 45), (15, 30, 45))

def make_vehicle(v_id, ts_id, pos):
    """Make a vehicle."""
    return Vehicle(v_id, 'PRT_DEFAULT', 4, 4, ts_id, pos, 0, 0, 2.5, -2.5, 5.0, -5.0, 25)

if __name__ == '__main__':
    unittest.main()