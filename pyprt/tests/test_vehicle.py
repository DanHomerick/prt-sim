import unittest

import networkx
import SimPy.Simulation as Sim

from pyprt.shared.utility import pairwise
import pyprt.sim.vehicle as vehicle
from pyprt.sim.layout import TrackSegment
from pyprt.shared.cubic_spline import Knot, CubicSpline
import pyprt.shared.api_pb2 as api

class MockConfigManager(object):
    def __init__(self, sim_end_time):
        self.sim_end_time = sim_end_time

    def get_sim_end_time(self):
        return self.sim_end_time

class MockInterface(object):
    def send(self, msg_type, msg):
        pass

class MockCommon(object):
    def __init__(self, tracks, digraph, sim_end_time):
        self.track_segments = tracks
        self.digraph = digraph
        self.config_manager = MockConfigManager(sim_end_time)
        self.interface = MockInterface()

class MockController(Sim.Process):
    """Bypasses communication stuff, calling a vehicle's methods directly."""
    def __init__(self):
        Sim.Process.__init__(self)
        self.cmds = []

    def add_cmd(self, time, fnc, *args):
        self.cmds.append( (time, fnc, args) )
        self.cmds.sort()

    def run(self):
        while self.cmds:
            time, fnc, args = self.cmds.pop(0)
            yield Sim.hold, self, time - Sim.now()
            fnc(*args)

class TestBaseVehicle(unittest.TestCase):
    SIM_END_TIME = 1000
    PLACES = 7

    def setUp(self):
        tracks, digraph = self.make_track()
        vehicle.common = MockCommon(tracks, digraph, self.SIM_END_TIME)

        self.ts0 = tracks[0]
        self.ts1 = tracks[1]
        self.ts2 = tracks[2]
        self.ts3 = tracks[3]

        Sim.initialize()

    def make_track(self):
        """Makes a simple loop shaped track with 4 TrackSegments. Makes the
        corresponding DiGraph."""
        tracks = {0:TrackSegment(0, 0, 0, 0, 0, 100, 50, 'TS0'), # 100 meters long
                  1:TrackSegment(1, 0, 0, 0, 0, 3, 50, 'TS1'),
                  2:TrackSegment(2, 0, 0, 0, 0, 100, 50, 'TS2'),
                  3:TrackSegment(3, 0, 0, 0, 0, 25, 50, 'TS3')}

        digraph = networkx.DiGraph()
        digraph.add_edges_from(pairwise(sorted(tracks.values())))
        digraph.add_edge(tracks[3], tracks[0])

        tracks[0].next_loc = tracks[1]
        tracks[1].next_loc = tracks[2]
        tracks[2].next_loc = tracks[3]
        tracks[3].next_loc = tracks[0]

        return tracks, digraph

    def test_init_vehicle_placement(self):
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 0)
        self.assertAlmostEqual(v0.pos, 50, self.PLACES)
        self.assertTrue(v0.loc is self.ts0)
        self.assertTrue(v0.tail_loc is self.ts0)
        self.assertTrue(self.ts0.vehicles[0] is v0)

        # Place a second vehicle on the same track, ahead of v0
        v1 = vehicle.BaseVehicle(1, self.ts0, 75, 0)
        self.assertAlmostEqual(v1.pos, 75, self.PLACES)
        self.assertTrue(v1.loc is self.ts0)
        self.assertTrue(v1.tail_loc is self.ts0)
        self.assertTrue(self.ts0.vehicles[0] is v1)
        self.assertTrue(self.ts0.vehicles[1] is v0)

        # Place a third vehicle, straddling ts1 and ts0. Should come ahead of
        # v0 and v1 on ts0.
        v_length = v1.length
        v2 = vehicle.BaseVehicle(2, self.ts1, 1, 0)
        self.assertAlmostEqual(v2.pos, 1, self.PLACES)
        self.assertTrue(v2.loc is self.ts1)
        self.assertTrue(v2.tail_loc is self.ts0)
        self.assertAlmostEqual(v2.tail_pos, self.ts0.length - (v_length-1), self.PLACES)
        self.assertTrue(self.ts1.vehicles[0] is v2)
        self.assertTrue(self.ts0.vehicles[0] is v2)
        self.assertTrue(self.ts0.vehicles[1] is v1)
        self.assertTrue(self.ts0.vehicles[2] is v0)

    def test_init_vehicle_placement_II(self):
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 0)
        self.assertAlmostEqual(v0.pos, 50, self.PLACES)
        self.assertAlmostEqual(v0.tail_pos, 50 - v0.length, self.PLACES)

        # test that get_positions is consistant with .pos and .tail_pos
        self.assertEqual(v0.pos, v0.get_positions()[0])
        self.assertEqual(v0.tail_pos, v0.get_positions()[1])

        # test that positions correct when vehicle created straddling several boundaries.
        # Nose should be on ts2, middle is on ts1, and tail is on ts0.
        v_length = v0.length
        self.assertEqual(v_length, 5) # update this test if default length is changed...
        v1 = vehicle.BaseVehicle(1, self.ts2, 1, 0)
        self.assertAlmostEqual(v1.pos, 1, self.PLACES)
        self.assertAlmostEqual(v1.tail_pos, 99, self.PLACES)
        self.assertTrue(v1.loc is self.ts2)
        self.assertTrue(v1.tail_loc is self.ts0)
        self.assertTrue(self.ts2.vehicles[0] is v1)
        self.assertTrue(self.ts1.vehicles[0] is v1)
        self.assertTrue(self.ts0.vehicles[0] is v1)
        self.assertTrue(self.ts0.vehicles[1] is v0)

    def test_one_vehicle_loop_sim(self):
        """A single vehicle, running around the loop at a constant 5 m/s.
        Makes almost two complete loops, stopping 6 meters before it's starting
        position."""
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 5)

        Sim.activate(v0, v0.ctrl_loop())
        Sim.simulate(until=81.4)

        self.assertAlmostEqual(v0.pos, 1, self.PLACES)
        self.assertAlmostEqual(v0.tail_pos, 21, self.PLACES)
        self.assertTrue(v0.loc is self.ts0)
        self.assertTrue(v0.tail_loc is self.ts3)
        self.assertTrue(self.ts0.vehicles[0] is v0)
        self.assertEquals(len(self.ts1.vehicles), 0)
        self.assertEquals(len(self.ts2.vehicles), 0)
        self.assertTrue(self.ts3.vehicles[0] is v0)

    def test_process_spline_I(self):
        """Check that a spline was transferred correctly to the vehicle."""
        spline = CubicSpline(
            [50, 53.333333333333336, 73.333333333333343, 110.0, 190.0, 226.66666666666666, 246.66666666666666, 250, 250], # pos
            [0, 5.0, 15.0, 20.0, 20.0, 15.0, 5.0, 0, 0], # vel
            [0, 5, 5, 0, 0, -5, -5, 0, 0], # accel
            [0, 2.0, 4.0, 6.0, 10.0, 12.0, 14.0, 16.0, 100])

        spline_msg = api.Spline()
        spline.fill_spline_msg(spline_msg)

        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 0)
        v0.process_spline_msg(spline_msg)

        # Test that the vehicle's spline matches the original
        for q_orig, q_vehicle in zip(spline.q, v0._spline.q):
            self.assertAlmostEqual(q_orig, q_vehicle, self.PLACES)
        for v_orig, v_vehicle in zip(spline.v, v0._spline.v):
            self.assertAlmostEqual(v_orig, v_vehicle, self.PLACES)
        for a_orig, a_vehicle in zip(spline.a, v0._spline.a):
            self.assertAlmostEqual(a_orig, a_vehicle, self.PLACES)
        for coeffs_orig, coeffs_vehicle in zip(spline.coeffs, v0._spline.coeffs):
            for x_orig, x_vehicle in zip(coeffs_orig, coeffs_vehicle):
                self.assertAlmostEqual(x_orig, x_vehicle, self.PLACES)

    def test_process_spline_II(self):
        """Check that the vehicle only trusts the jerk coefficients to recreate
        the spline (not trusting controllers knowledge of vehicle state)."""
        spline = CubicSpline(
            [50, 53.333333333333336, 73.333333333333343, 110.0, 190.0, 226.66666666666666, 246.66666666666666, 250, 250], # pos
            [0, 5.0, 15.0, 20.0, 20.0, 15.0, 5.0, 0, 0], # vel
            [0, 5, 5, 0, 0, -5, -5, 0, 0], # accel
            [0, 2.0, 4.0, 6.0, 10.0, 12.0, 14.0, 16.0, 100])

        spline_msg = api.Spline()
        spline.fill_spline_msg(spline_msg)

        # Zero out everything but jerk and times in the message
        for poly in spline_msg.polys:
            for i in range(1,4): # skip jerk
                poly.coeffs[i] = 0

        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 0)
        v0.process_spline_msg(spline_msg)

        # Test that the vehicle's spline matches the original
        for q_orig, q_vehicle in zip(spline.q, v0._spline.q):
            self.assertAlmostEqual(q_orig, q_vehicle, self.PLACES)
        for v_orig, v_vehicle in zip(spline.v, v0._spline.v):
            self.assertAlmostEqual(v_orig, v_vehicle, self.PLACES)
        for a_orig, a_vehicle in zip(spline.a, v0._spline.a):
            self.assertAlmostEqual(a_orig, a_vehicle, self.PLACES)
        for coeffs_orig, coeffs_vehicle in zip(spline.coeffs, v0._spline.coeffs):
            for x_orig, x_vehicle in zip(coeffs_orig, coeffs_vehicle):
                self.assertAlmostEqual(x_orig, x_vehicle, self.PLACES)

    def test_find_leading_vehicle_I(self):
        """Tested when no other vehicle exists on track."""
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 15)

        lv, dist = v0.find_leading_vehicle(current_loc_only=False, max_dist=500)
        self.assertEqual(lv, None)
        self.assertEqual(dist, 500)

    def test_find_leading_vehicle_II(self):
        """Tested with one other vehicle ahead of v0, placed on the same TrackSegment."""
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 0)
        v1 = vehicle.BaseVehicle(1, self.ts0, 75, 0)

        lv, dist = v0.find_leading_vehicle(current_loc_only=True, max_dist=500)
        self.assertTrue(lv is v1)
        self.assertAlmostEqual(dist, 25-v1.length, self.PLACES)

        # Check that max_dist works
        lv, dist = v0.find_leading_vehicle(current_loc_only=True, max_dist=10)
        self.assertTrue(lv is None)
        self.assertAlmostEqual(dist, 10, self.PLACES)

        # Check that v1 can see v0 if allowed to look far enough.
        lv, dist = v1.find_leading_vehicle(current_loc_only=False, max_dist=1000)
        self.assertTrue(lv is v0)
        self.assertAlmostEqual(dist, 203-v0.length, self.PLACES)

        # Check that v1 can't see v0 if current_loc_only is True
        lv, dist = v1.find_leading_vehicle(current_loc_only=True, max_dist=1000)
        self.assertTrue(lv is None)
        self.assertAlmostEqual(dist, 1000, self.PLACES)

    def test_fill_VehicleStatus(self):
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 15)

        status = api.VehicleStatus()
        v0.fill_VehicleStatus(status)

        self.assertEqual(status.vID, 0)
        self.assertEqual(status.nose_locID, 0)
        self.assertAlmostEqual(status.nose_pos, 50, self.PLACES)
        self.assertEqual(status.tail_locID, 0)
        self.assertAlmostEqual(status.tail_pos, 50 - v0.length, self.PLACES)
        self.assertAlmostEqual(status.vel, 15)
        self.assertEqual(status.passengerIDs, [])
        self.assertFalse(status.HasField('lvID'))
        self.assertFalse(status.HasField('lv_distance'))

        # create a new vehicle in front of v1, and check again
        v1 = vehicle.BaseVehicle(1, self.ts0, 75, 0)
        status = api.VehicleStatus()
        v0.fill_VehicleStatus(status)
        self.assertEqual(status.lvID, 1)
        self.assertAlmostEqual(status.lv_distance, 25 - v1.length, self.PLACES)

    def test_updating_trajectory(self):
        """Creates a vehicle, starts the sim with it travelling at a fixed velocity,
        then changes the trajectory 11 seconds after the start. When the trajectory
        is changed, the vehicle is straddling two track segments."""
        v0 = vehicle.BaseVehicle(0, self.ts0, 50, 5) # fixed velocity, 5 m/s

        # stop at 50 meters on ts2
        spline = CubicSpline(
            [2, 6.2324427610773778, 20.693227678868798, 46.258768272424305, 46.67126664464751, 49.999999999978492], # pos
            [5, 5.8065992674127145, 9.581117951456692, 5.3923182554918929, 4.9953989633679301, -9.5825569701446511e-12], # vel
            [0, 2.0082321422244926, 2.0082321422244926, -4.9976989522066617, -4.9976989522066617, -1.8332002582610585e-12], # accel
            [11, 11.803292856889797, 13.682815948210798, 16.48518838598326, 16.564608794439177, 18.56368837532111]) # time

        spline_msg = api.Spline()
        spline.fill_spline_msg(spline_msg)

        controller = MockController()
        controller.add_cmd(11, v0.process_spline_msg, spline_msg)

        Sim.activate(controller, controller.run())
        Sim.activate(v0, v0.ctrl_loop())
        Sim.simulate(until=20)

        self.assertAlmostEqual(v0.pos, 50, self.PLACES)
        self.assertTrue(v0.loc is self.ts2)
        self.assertAlmostEqual(v0.vel, 0, self.PLACES)
        self.assertAlmostEqual(v0.accel, 0, self.PLACES)
        self.assertAlmostEqual(v0.tail_pos, 45, self.PLACES)
        self.assertTrue(v0.tail_loc is self.ts2)
        self.assertTrue(len(self.ts0.vehicles) == 0)
        self.assertTrue(len(self.ts1.vehicles) == 0)
        self.assertTrue(len(self.ts2.vehicles) == 1)


if __name__ == '__main__':
    unittest.main()