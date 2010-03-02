import unittest
import pyprt.ctrl.prt_controller as prt

from pyprt.ctrl.gtf_controller import Vehicle

class TestStation(unittest.TestCase):
##    def setUp(self):
##        pass
##
##    def tearDown(self):
##        pass

    def test_request_berth(self):
        s = make_station_I()

        # Request an unload berth in an empty station. Should get last unload berth.
        ts_id = s.ts_ids[prt.Station.UNLOAD]
        v0 = make_vehicle(0, 0, 0) # actual ts_id, pos don't matter
        req_pos, req_b_id, req_ts_id = s.request_berth(v0, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(req_pos, s.berth_positions[prt.Station.UNLOAD_PLATFORM][-1])
        self.assertEqual(req_b_id, 2) # last berth
        self.assertEqual(req_ts_id, s.ts_ids[prt.Station.UNLOAD])

        # Add a vehicle reservation for the middle unload berth
        pos = s.berth_positions[prt.Station.UNLOAD_PLATFORM][1] - 0.1
        ts_id = s.ts_ids[prt.Station.UNLOAD]
        v1 = make_vehicle(1, ts_id, pos)
        s.berth_reservations[prt.Station.UNLOAD_PLATFORM][1] = v1

        # Request another berth. Should get the first berth.
        v2 = make_vehicle(2, s.ts_ids[prt.Station.OFF_RAMP_I], 0)
        req_pos, req_b_id, req_ts_id =  s.request_berth(v2, prt.Station.UNLOAD_PLATFORM)
        self.assertEqual(req_pos, s.berth_positions[prt.Station.UNLOAD_PLATFORM][0])
        self.assertEqual(req_b_id, 0)
        self.assertEqual(req_ts_id, s.ts_ids[prt.Station.UNLOAD])

def make_station_I():
    """Make a station with 3 unload, 3 queue, and 3 loading berths. Berths
    are at 15 meter spacings."""
    return prt.Station(0, (10,11,12,13,14,15,16,17), 9,
                       (15, 30, 45), (15, 30, 45), (15, 30, 45))

def make_vehicle(v_id, ts_id, pos):
    """Make a vehicle."""
    return Vehicle(v_id, 'DEFAULT', 4, ts_id, pos, 0, 0, 2.5, -2.5, 5.0, -5.0, 25)

if __name__ == '__main__':
    unittest.main()