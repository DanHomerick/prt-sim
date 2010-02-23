import unittest
import SimPy.Simulation as Sim
import sys

import common
import layout
from scipy import poly1d
from vehicle import Segment
from vehicle import Traversal
from vehicle import Path
from vehicle import InvalidTimeError
from vehicle import InvalidPosError
from vehicle import TraversalFullError
from vehicle import FutureLoc
from vehicle import shift_poly
from vehicle import Vehicle


class MockLoc(object):
    label = 'MockLoc'
    def __init__(self, length, id=1):
        self.length = length
        self.ID = id

class MockInterface(object):
    def __init__(self):
        self.msgs = []
    def send(self, msg_type, msg):
        self.msgs.append( (msg_type, msg) )

POS = 0
VEL = 1
ACCEL = 2
JERK = 3

def seg_continuity_check(utest, seg1, seg2, checks=(POS, VEL, ACCEL)):
    # Tests continuity between segments. Checks pos, vel, accel by default
    # utest is the unit test I'm calling from (i.e. `self`)
    eqns1 = []
    for i in checks:
        eqns1.append(seg1.pos_eqn.deriv(i))
    eqns2 = []
    for i in checks:
        eqns2.append(seg2.pos_eqn.deriv(i))
    t1 = seg1.duration
    t2 = 0
    for eqn1, eqn2 in zip(eqns1, eqns2):
        utest.assertAlmostEqual( eqn1(t1), eqn2(t2), places = 5 )

def trav_continuity_check(utest, trav):
    # check that a traversal is continuous between it's segments and that the
    # first segment starts at pos 0 (unless it's the initial traversal).
    # utest is the unit test I'm calling from (i.e. `self`)
    if trav.idx > 1 and trav.segments:
        utest.assertAlmostEqual( trav.segments[0].pos_eqn(0), 0, places=common.DIST_RND)
    if len(trav.segments) >= 2:
        for seg1, seg2 in zip(trav.segments[0:-1], trav.segments[1:]):
            seg_continuity_check(utest, seg1, seg2)

def path_continuity_check(utest, path):
    # check that each traversal is continuous within itself
    for trav in path.traversals:
        trav_continuity_check(utest, trav)
    # check that traversals are continuous between each other
    if len(path.traversals) >= 2:
        for trav1, trav2 in zip(path.traversals[0:-1], path.traversals[1:]):
            seg_continuity_check(utest, trav1.segments[-1], trav2.segments[0], checks=(VEL, ACCEL))
    # check that last traversal is continuous with path.future
    if len(path.future.segments) >= 1:
        last_trav = path.traversals[-1]
        seg_continuity_check(utest, last_trav.segments[-1], path.future.segments[0], checks=(VEL, ACCEL))
    # check that path.future is continuous within itself
    trav_continuity_check(utest, path.future)
    # check that path.future always ends with an infinite duration segment
    utest.assertEqual(path.future.segments[-1].duration, float('Inf'))

def traversal_index_check(utest, path):
    for idx, trav in enumerate(path.traversals):
        utest.assertTrue(trav.idx == idx)

class SegmentTestBasic(unittest.TestCase):

    def setUp(self):
        """A three segment acceleration from 0 to 25 followed by an infinite
        segment maintaining 25 m/s. Following data from speed_profiler (rev 199)
        t1: 2.0, t2: 0.5, t3: 3.0
        a_init 2.5
        v_init 0.0
        v_final 25.0
        max_accel 7.5
        max_decel -7.5
        max_jerk 2.5
        Coefficients:
        0 [ 0.41666667  1.25        0.          0.        ]
        1 [  3.75        10.           8.33333333]
        2 [ -0.41666667   3.75        13.75        14.27083333]
        3 [ 25.          78.02083333]
        Seg        Time        Jerk        Accel        Vel        Pos
        0        0        2.5        2.5        0.0        0.0
        0        2.0        2.5        7.5        10.0        8.33333333333
        1        0        0.0        7.5        10.0        8.33333333333
        1        0.5        0.0        7.5        13.75        14.2708333333
        2        0        -2.5        7.5        13.75        14.2708333333
        2        3.0        -2.5        0.0        25.0        78.0208333333
        3        0        0.0        0.0        25.0        78.0208333333
        3        inf        nan        nan        nan        nan
        """
        seg0 = Segment(poly1d([0.41666667,  1.25, 0., 0.]), 2.0, 10)
        seg1 = Segment(poly1d([3.75, 10., 8.33333333]), 0.5, 12.0)
        seg2 = Segment(poly1d([-0.41666667, 3.75, 13.75, 14.27083333]), 3.0, 12.5)
        seg3 = Segment(poly1d([ 25., 78.02083333]), float('Inf'), 15.5)
        self.segs = (seg0, seg1, seg2, seg3)

    def test_get_start_pos(self):
        answers = (0.0, 8.33333333333, 14.2708333333, 78.0208333333)
        for s, a in zip(self.segs, answers):
            self.assertAlmostEqual(s.get_start_pos(), a, places = common.DIST_RND)

    def test_get_end_pos(self):
        # Test inf duration case seperate, AssertAlmostEquals doesn't like it.
        answers = (8.33333333333, 14.2708333333, 78.0208333333)
        for s, a in zip(self.segs, answers):
            self.assertAlmostEqual(s.get_end_pos(), a, places = common.DIST_RND)
        self.assertEqual(self.segs[3].get_end_pos(), float('Inf'))

    def test_get_length(self):
        answers = (8.33333333333, 14.2708333333 - 8.33333333333, 78.0208333333 - 14.2708333333)
        for s, a in zip(self.segs, answers):
            self.assertAlmostEqual(s.get_length(), a, places = common.DIST_RND)

    def test_get_times_from_pos(self):
        pos = (0.0, 1.0 + 2.0/3.0, 8.33333333333)
        answers = (10.0, 11.0, 12.0)
        for p, a in zip(pos, answers):
            t = self.segs[0].get_times_from_pos(p)
            self.assertAlmostEqual(t.points[0], a, places = 5)
        # Out of range
        t = self.segs[1].get_times_from_pos(15)
        self.assertTrue( len(t.points) == 0 )
        # Stopped vehicles
        stopped = Segment(poly1d([5]), 10, 10)
        t = stopped.get_times_from_pos(5)
        self.assertEquals( len(t.points), 0 )
        self.assertEquals(t.ranges[0][0], 10) # range start time
        self.assertEquals(t.ranges[0][1], 20) # range stop time

    def test_get_pos_from_time(self):
        times = (10.0, 11.0, 12.0)
        pos = (0.0, 1.0 + 2.0/3.0, 8.33333333333)
        for t, p in zip(times, pos):
            self.assertAlmostEqual(self.segs[0].get_pos_from_time(t), p, places = common.DIST_RND)
        self.assertRaises(InvalidTimeError, self.segs[1].get_pos_from_time, 1.99)
        self.assertRaises(InvalidTimeError, self.segs[1].get_pos_from_time, 2.51)
        self.assertAlmostEqual(self.segs[3].get_pos_from_time(20.0), 190.52099999999999, places = common.DIST_RND)

        # additional test
        seg = Segment(poly1d([0,0,10,0]), 10, 0) # constant speed
        times = (0, 0.0001, 5.00000, 9.9999)
        ans = (0.0, 0.001, 50.000, 99.999, 100.000)
        for t, a in zip(times, ans):
            pos = seg.get_pos_from_time(t)
            self.assertAlmostEqual(pos, a)
        self.assertRaises(InvalidTimeError, seg.get_pos_from_time, -0.1)
        self.assertRaises(InvalidTimeError, seg.get_pos_from_time, 10.1)

    def test_split_at_pos(self):
        s1, s2 = self.segs[1].split_at_pos(10)
        self.assertAlmostEqual(s1.get_length(), 1.0+2.0/3.0, places = common.DIST_RND)
        self.assertAlmostEqual(s1.get_end_pos(), 10, places = common.DIST_RND)
        self.assertAlmostEqual(s2.get_start_pos(), 0, places = common.DIST_RND)
        self.assertAlmostEqual(s2.get_length(), 14.2708333333 - 10, places = common.DIST_RND)
        seg_continuity_check(self, s1, s2, checks=(VEL, ACCEL))

        s3, s4 = self.segs[3].split_at_pos(100.0208333333)
        seg_continuity_check(self, s3, s4, checks=(VEL, ACCEL))
        self.assertAlmostEqual(s3.get_length(), 100.0208333333 - 78.0208333333, places = common.DIST_RND)
        self.assertAlmostEqual(s3.get_end_pos(), 100.0208333333, places = common.DIST_RND)
        self.assertAlmostEqual(s4.get_start_pos(), 0, places = common.DIST_RND)
        self.assertTrue(s4.end_time == float('Inf') )

    def test_collision_check_no_overlap(self):
        # No overlap between the two segments
        self.assertRaises(InvalidTimeError, self.segs[0].collision_check, self.segs[2], 0)
        self.assertRaises(InvalidTimeError, self.segs[2].collision_check, self.segs[0], 0)

        # valid range is a single point. Since the end of segment is not part
        # of it's range, it has no real overlap. Not raising an exception,
        # so long as correct results still returned.
        seg1 = Segment(poly1d([1,0]), 1, 0)
        seg2 = Segment(poly1d([-1,1]), 1, 1)
        self.assertEqual(seg1.collision_check(seg2, 0), [])
        self.assertEqual(seg2.collision_check(seg1, 0), [])

    def test_collision_check_simple(self):
        # Segments fully overlap. Collision at time 0.5
        seg1 = Segment(poly1d([1,0]), 1, 0)
        seg2 = Segment(poly1d([-1,1]), 1, 0)
        self.assertEqual(seg1.collision_check(seg2, 0), [0.5])
        self.assertEqual(seg2.collision_check(seg1, 0), [0.5])

        # Segments fully overlap. Collision outside of range.
        seg1 = Segment(poly1d([1,0]), 1, 0)
        seg2 = Segment(poly1d([-1,5]), 1, 0)
        self.assertEqual(seg1.collision_check(seg2, 0), [])
        self.assertEqual(seg2.collision_check(seg1, 0), [])

    def test_collision_check_offset(self):
        # collision occurs inside valid range
        seg1 = Segment(poly1d([1,0]), 1, 0)
        seg2 = Segment(poly1d([-1,1]), 1, 0.6)
        self.assertEqual(seg1.collision_check(seg2, 0), [0.8])
        self.assertEqual(seg2.collision_check(seg1, 0), [0.8])

        # collision occurs outside of valid range
        seg1 = Segment(poly1d([1,0]), 1, 0)
        seg2 = Segment(poly1d([-1,2]), 1, 0.6)
        self.assertEqual(seg1.collision_check(seg2, 0), [])
        self.assertEqual(seg2.collision_check(seg1, 0), [])

    def test_collision_check_stopped(self):
        # doesn't blow up when two stopped vehicles compared
        seg1 = Segment(poly1d([5]), float('inf'), 0)
        seg2 = Segment(poly1d([5]), float('inf'), 100)
        self.assertEqual(seg1.collision_check(seg2, 0), [100])
        self.assertEqual(seg2.collision_check(seg1, 0), [100])

        # or when two vehicles are moving in sychronicity
        seg1 = Segment(poly1d([5, 10, 15, 20]), 100, 0)
        seg2 = Segment(poly1d([5, 10, 15, 25]), 100, 0)
        self.assertEqual(seg1.collision_check(seg2, 0), [])
        self.assertEqual(seg2.collision_check(seg1, 0), [])

    def test_collision_check_lvlength(self):
        # take into account vehicle length
        seg1 = Segment(poly1d([5, 10, 15, 20]), 100, 0)
        seg2 = Segment(poly1d([5, 10, 15, 25]), 100, 0)
        self.assertEqual(seg1.collision_check(seg2, -5), [0])
        self.assertEqual(seg2.collision_check(seg1, -5), [])

        seg1 = Segment(poly1d([1,0]), 1, 0)
        seg2 = Segment(poly1d([-1,11]), 1, 0.6)
        self.assertEqual(seg1.collision_check(seg2, -10), [0.8])
        self.assertEqual(seg2.collision_check(seg1, -10), [])

class TraversalTestBasic(unittest.TestCase):
    def setUp(self):
        # Same data as for SegmentTestBasic
        seg0 = Segment(poly1d([0.41666667,  1.25, 0., 0.]), 2.0, 10)
        seg1 = Segment(poly1d([3.75, 10., 8.33333333]), 0.5, 12.0)
        seg2 = Segment(poly1d([-0.41666667, 3.75, 13.75, 14.27083333]), 3.0, 12.5)
        seg3 = Segment(poly1d([ 25., 78.02083333]), float('Inf'), 15.5)
        self.segs = (seg0, seg1, seg2, seg3)

    def test_append_segment_large(self):
        loc = MockLoc(100) # holds whole trajectory
        trav = Traversal(loc, 0)
        trav.append_segment(self.segs[0])
        self.assertTrue(len(trav.segments) == 1)
        self.assertTrue(trav.segments[0] is self.segs[0])

        trav.append_segment(self.segs[1])
        self.assertTrue(len(trav.segments) == 2)
        self.assertTrue(trav.segments[1] is self.segs[1])

        trav.append_segment(self.segs[2])
        self.assertAlmostEquals(trav.segments[2].get_length(), 78.0208333333 - 14.2708333333, places = common.DIST_RND)
        self.assertRaises(TraversalFullError, trav.append_segment, self.segs[3])
        trav_continuity_check(self, trav)

    def test_append_segment_small(self):
        loc = MockLoc(50)         # too small to fit trajectory
        trav = Traversal(loc, 0)
        for seg in self.segs[:-2]:
            trav.append_segment(seg)
        trav_continuity_check(self, trav)
        self.assertRaises(TraversalFullError, trav.append_segment, self.segs[2])
        # segment starts beyond location length
        self.assertRaises(InvalidPosError, trav.append_segment, self.segs[3])

    def test_get_segment_from_time(self):
        loc = MockLoc(100)
        trav = Traversal(loc, 0)
        self.assertRaises(InvalidTimeError, trav.get_segment_from_time)
        trav.append_segment(self.segs[0])
        # Segment starts at time 10
        self.assertRaises(InvalidTimeError, trav.get_segment_from_time, 5)
        self.assertTrue(trav.get_segment_from_time(10) is self.segs[0])

        trav2 = Traversal(loc, 0)
        seg = Segment(poly1d([0]), 0, 0)
        trav2.append_segment(seg)
        for s in self.segs[0:-1]:
            trav2.append_segment(s)
        self.assertTrue(trav2.get_segment_from_time() is trav2.segments[0])

    def test_get_times_from_pos(self):
        loc = MockLoc(100.0)
        trav = Traversal(loc, 0)
        for seg in self.segs[0:-1]:
            trav.append_segment(seg)
        pos = (0.0, 1.0 + 2.0/3.0, 8.33333333333, 14.2708333333, 78.0208333333)
        times = (10.0, 11.0, 12.0, 12.5, 15.5)
        for p, t in zip(pos, times):
            get_t = trav.get_times_from_pos(p)
            self.assertEqual(len(get_t.points), 1)
            self.assertEqual(len(get_t.ranges), 0)
            self.assertAlmostEquals(get_t.points[0], t, places = 4)
        # pos outside of location length
        self.assertRaises(InvalidPosError, trav.get_times_from_pos, -5)
        self.assertRaises(InvalidPosError, trav.get_times_from_pos, 100.01)
        # pos inside of location length, but beyond last segment
        self.assertEqual(len(trav.get_times_from_pos(78.03).points), 0)

    def test_get_pos_from_time(self):
        loc = MockLoc(100)
        trav = Traversal(loc, 0)
        seg = Segment(poly1d([0,0,10,0]), 10, 0) # constant speed
        trav.append_segment(seg)
        times = (0, 0.0001, 5.00000, 9.9999)
        ans = (0.0, 0.001, 50.000, 99.999, 100.000)
        for t, a in zip(times, ans):
            pos = trav.get_pos_from_time(t)
            self.assertAlmostEqual(pos, a)
        self.assertRaises(InvalidTimeError, trav.get_pos_from_time, -0.1)
        self.assertRaises(InvalidTimeError, trav.get_pos_from_time, 10.1)

    def test_clear(self):
        loc = MockLoc(100)
        trav = Traversal(loc, 0)
        for seg in self.segs[0:-1]:
            trav.append_segment(seg)
        self.assertEqual(len(trav.segments), 3)
        trav.clear(12.2)
        self.assertEqual(len(trav.segments), 2)
        self.assertEqual(trav.segments[-1].end_time, 12.2)
        trav_continuity_check(self, trav)

    def test_collision_check(self):
        loc = MockLoc(8)
        trav1 = Traversal(loc, 0)
        seg1_1 = Segment(poly1d([1, 0, 0]), 1, 0 )
        seg2_1 = Segment(poly1d([0, 2, 1]), 3, 1 )
        seg3_1 = Segment(poly1d([-1, 2, 7]), 1, 4 )
        for seg in (seg1_1, seg2_1, seg3_1):
            trav1.append_segment(seg)

        trav2 = Traversal(loc, 0)
        seg1_2 = Segment(poly1d([1, 0, 3]), 0.5, 0 )
        seg2_2 = Segment(poly1d([0, 1, 3.25]), 3.75, 0.5 )
        seg3_2 = Segment(poly1d([-1, 2, 7]), 0.5, 4.25 )
        seg4_2 = Segment(poly1d([0, 0, 7.25]), float('inf'), 4.75)
        for seg in (seg1_2, seg2_2, seg3_2, seg4_2):
            trav2.append_segment(seg)

        # lv length 0. Collide at pos 6.25 at time 3.75
        self.assertEqual(trav1.collision_check(trav2, 0), [3.75])

        # lv length 1. Collide at pos 4.5 | 5.5 at time 2.75
        self.assertEqual(trav1.collision_check(trav2, 1), [2.75])

        # no collision
        self.assertEqual(trav1.collision_check(trav1, -5), [] )

class PathTestBasic(unittest.TestCase):
    def setUp(self):
        # Same data as other two test suites
        seg0 = Segment(poly1d([0.41666667,  1.25, 0., 0.]), 2.0, 0)
        seg1 = Segment(poly1d([3.75, 10., 8.33333333]), 0.5, 2.0)
        seg2 = Segment(poly1d([-0.41666667, 3.75, 13.75, 14.27083333]), 3.0, 2.5)
        seg3 = Segment(poly1d([ 25., 78.02083333]), float('Inf'), 5.5)
        self.segs = (seg0, seg1, seg2, seg3)

    def test_append_segment_large(self):
        start_loc = MockLoc(100) # fits whole trajectory in one traversal
        path = Path(start_loc, self.segs[0])
        for s in self.segs[1:]:
            path.append_segment(s)
        self.assertEqual(len(path.traversals[0].segments), 4)
        self.assertEqual(len(path.future.segments), 1)
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_append_segment_small(self):
        # Doesn't fit whole trajectory in one traversal
        start_loc = MockLoc(50)
        path = Path(start_loc, self.segs[0])
        for s in self.segs[1:]:
            path.append_segment(s)
        self.assertEqual(len(path.future.segments), 2)
        self.assertEqual(len(path.traversals), 1)
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_append_segment_stopped(self):
        start_loc = MockLoc(50)
        seg = Segment(poly1d([0,0,0,0]), float('Inf'), 0.0)
        path = Path(start_loc, seg)
        self.assertEqual(len(path.traversals), 1)
        self.assertEqual(len(path.future.segments), 0)
#        path_continuity_check(self, path) # No future until traj changed
        traversal_index_check(self, path)

    def test_append_loc_large(self):
        start_loc = MockLoc(100) # fits trajectory in one traversal
        path = Path(start_loc, self.segs[0])
        for s in self.segs[1:]:
            path.append_segment(s)
        self.assertEqual(len(path.future.segments), 1)
        path.append_loc(MockLoc(10))
        self.assertEqual(len(path.traversals), 2)
        self.assertEqual(len(path.traversals[0].segments), 4)
        self.assertEqual(len(path.traversals[1].segments), 1)
        self.assertEqual(len(path.future.segments), 1)
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_append_loc_small(self):
        # Doesn't fit whole trajectory in one traversal
        start_loc = MockLoc(50)
        path = Path(start_loc, self.segs[0])
        for s in self.segs[1:]:
            path.append_segment(s)
        self.assertEqual(len(path.future.segments), 2)
        path.append_loc(MockLoc(50))
        self.assertEqual(len(path.traversals), 2)
        self.assertEqual(len(path.traversals[0].segments), 3)
        self.assertEqual(len(path.traversals[1].segments), 2)
        self.assertEqual(len(path.future.segments), 1)
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_append_loc_zero_length(self):
        start_loc = MockLoc(50)
        seg = Segment(poly1d([0,0,10,0]), float('Inf'), 0)
        path = Path(start_loc, seg)
        zero_loc = MockLoc(0)
        ten_loc = MockLoc(10)
        path.append_loc(zero_loc)
        path.append_loc(ten_loc)
        self.assertEqual(len(path.traversals), 3)
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_change_trajectory_a(self):
        # Starting at time 1.0, smoothly change to a trajectory that uses:
        # a_init 5.0, v_init 3.75, v_final 1.0, max_accel 5.0, max_decel -5.0,
        # max_jerk 2.5
        start_loc = MockLoc(25)
        path = Path(start_loc, self.segs[0])
        for s in self.segs[1:]:
            path.append_segment(s)
        # New seg splits 1st segment.
        new_seg1 = Segment(poly1d([-0.41666667, 2.5, 3.75,1.0+2.0/3.0]), 3.76068168617, 1.0)
        new_seg2 = Segment(poly1d([0.41666667, -2.20085211,4.875,27.29841758+1.0+2.0/3.0]), 1.76068168617, 4.76068168617)
        new_seg3 = Segment(poly1d([1.,31.33331311+1.0+2.0/3.0]), float('Inf'), 4.76068168617 + 1.76068168617)
        new_segs = (new_seg1, new_seg2, new_seg3)
        path.change_trajectory( new_segs )
        end_vel, end_loc = path.get_x_loc_from_time(VEL, 6.521363372)
        self.assertAlmostEqual(end_vel, 1.0)
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_change_trajectory_b(self):
        # Make the change occur starting on a traversal boundry
        seg = Segment(poly1d([0,0,10,0]), float('inf'), 0)
        path = Path(MockLoc(100), seg)
        path.append_loc(MockLoc(10))
        new0 = Segment(poly1d([2.5/6, 0, 10, 0]), 1.41421356237, 10)
        new1 = Segment(poly1d([ -2.5/6, 1.76776695,12.5,15.32064693]), 1.41421356237, 11.41421356237)
        new2 = Segment(poly1d([ 15, 35.35533906]), float('inf'), 12.828427124739)
        path.change_trajectory( (new0, new1, new2) )
        path_continuity_check(self, path)
        traversal_index_check(self, path)

    def test_change_trajectory_from_stop(self):
        start_loc = MockLoc(25)
        seg = Segment(poly1d([0,0,0,0]), float('Inf'), 0.0)
        path = Path(start_loc, seg)
        self.assertEqual(path.get_x_loc_from_time(VEL, 100)[0], 0)
        # Similar to normal, but starting with 0 init_accel and at time 10
        seg0 = Segment(poly1d([0.41666667, 0, 0., 0.]), 3.0, 10)
        seg1 = Segment(poly1d([3.75, 11.25, 11.25]), 1.0/3.0, 13.0)
        seg2 = Segment(poly1d([-0.41666667, 3.75, 13.75, 15.41666667]), 3.0, 13+1.0/3.0)
        seg3 = Segment(poly1d([ 25., 79.16666667]), float('Inf'), 16+1.0/3.0)
        new_segs = (seg0, seg1, seg2, seg3)
        path.change_trajectory( new_segs )
        path_continuity_check(self, path)
        self.assertEqual(path.get_x_loc_from_time(VEL, 100)[0], 25)
        traversal_index_check(self, path)

    def test_get_pos_loc_from_time(self):
        start_loc = MockLoc(50)
        seg = Segment(poly1d([0,0,10,0]), float('Inf'), 0) # constant speed
        path = Path(start_loc, seg)
        zero_loc = MockLoc(0)
        path.append_loc(zero_loc)
        ten_loc = MockLoc(10)
        path.append_loc(ten_loc)
        times = (0, 4.9999, 5.00000, 5.0001, 6.0)
        ans = ((0.000, start_loc), (49.999, start_loc), (0.000, ten_loc), (0.001, ten_loc), (0.0, FutureLoc()))
        for t, a in zip(times, ans):
            pos, loc = path.get_pos_loc_from_time(t)
            a_pos, a_loc = a
            self.assertAlmostEqual(pos, a_pos)
            # Typical usage will be an isinstance test
            self.assertTrue(loc == a_loc or isinstance(loc, FutureLoc))
        traversal_index_check(self, path)

    def test_get_eta_next_loc_a(self):
        start_loc = MockLoc(25)
        seg = Segment(poly1d([0,0,10,0]), float('Inf'), 0) # constant speed
        path = Path(start_loc, seg)
        path.append_loc(MockLoc(0))
        path.append_loc(MockLoc(30))
        self.assertAlmostEqual(path.get_eta_next_loc(), 2.5)
        traversal_index_check(self, path)

    def test_get_eta_next_loc_b(self):
        seg = Segment(poly1d([0,0,10,0]), float('Inf'), 0) # constant speed
        path = Path(MockLoc(0), seg)
        path.append_loc(MockLoc(0))
        path.append_loc(MockLoc(25))
        path.append_loc(MockLoc(30))
        self.assertAlmostEqual(path.get_eta_next_loc(), 2.5)
        traversal_index_check(self, path)

    def test_get_eta_next_loc_c(self):
        # eta to next loc when stopped
        seg = Segment(poly1d([0,0,0,0]), float('Inf'), 0)
        path = Path(MockLoc(100), seg)
        self.assertEqual(path.get_eta_next_loc(), float('inf'))
        traversal_index_check(self, path)

#    def test_clear_planned_route(self):
#        raise NotImplementedError

#  Collision checking at the path level not performed.
##    def test_collision_check(self):
##        start_loc = MockLoc(10)
##        seg1_1 = Segment(poly1d([1, 0, 0]), 1, 0 )
##        seg2_1 = Segment(poly1d([0, 2, 1]), 30, 1 )
##        seg3_1 = Segment(poly1d([-1, 2, 61]), 1, 31 )
##        seg4_1 = Segment(poly1d([0, 0, 62]), float('inf'), 32 )
##        path1 = Path(start_loc, seg1_1)
##        for seg in (seg2_1, seg3_1, seg4_1):
##            path1.append_segment(seg)
##
##        seg1_2 = Segment(poly1d([1, 0, 3]), 0.5, 0 )
##        seg2_2 = Segment(poly1d([0, 1, 3.25]), 3.75, 0.5 )
##        seg3_2 = Segment(poly1d([-1, 1, 7]), 0.5, 4.25 )
##        seg4_2 = Segment(poly1d([0, 0, 7.25]), float('inf'), 4.75)
##        path2 = Path(start_loc, seg1_2)
##        for seg in (seg2_2, seg3_2, seg4_2):
##            path2.append_segment(seg)
##
##        # collides at pos 6.5, time 3.75
##        self.assertEqual(path1.collision_check(path2, 0, 0, False), [3.75])
##        # doesn't collide again after t=3.75
##        self.assertEqual(path1.collision_check(path2, 0, 3.75, False), [3.75])
##        self.assertEqual(path1.collision_check(path2, 0, 3.76, False), [])
##
##        # try a collision that happens in future. Same trajectories
##        short_loc = MockLoc(5)
##        path3 = Path(short_loc, seg1_1)
##        for seg in (seg2_1, seg3_1, seg4_1):
##            path3.append_segment(seg)
##        path4 = Path(short_loc, seg1_2)
##        for seg in (seg2_2, seg3_2, seg4_2):
##            path4.append_segment(seg)
##        self.assertEqual(path3.collision_check(path4, 0, 0, False), [3.75])
##        self.assertEqual(path3.collision_check(path4, 0, 1, False), [3.75])


    def test_plot_path(self):
        start_loc = MockLoc(25)
        path = Path(start_loc, self.segs[0])
        for s in self.segs[1:]:
            path.append_segment(s)
        path.append_loc(MockLoc(10))
        path.append_loc(MockLoc(30))
        path.append_loc(MockLoc(50))
        path.plot('test_plot_path')

class MiscTest(unittest.TestCase):
    def test_shift_poly_0(self):
        # 0th order
        p = poly1d([5])
        self.assertEqual(shift_poly(p, 10, 0), p) # delta_t, no effect
        self.assertEqual(shift_poly(p, 0, -5), poly1d([0]))

    def test_shift_poly_1(self):
        # 1st order
        p = poly1d([-2, 10])
        self.assertEqual(shift_poly(p, -1, 0), poly1d([-2, 8])) # shift left 1
        self.assertEqual(shift_poly(p, 1, 0), poly1d([-2, 12])) # shift right 1
        self.assertEqual(shift_poly(p, 0, -1), poly1d([-2, 9])) # shift down 1
        self.assertEqual(shift_poly(p, 0, 1), poly1d([-2, 11])) # shift up 1
        self.assertEqual(shift_poly(p, 1, 1), poly1d([-2, 13])) # shift up 1, right 1

    def test_shift_poly_2(self):
        # 2nd order
        p = poly1d([2, 1, 10])
        self.assertEqual(shift_poly(p, -1, 0), poly1d([2, 5, 13])) # shift left 1
        self.assertEqual(shift_poly(p, 1, 0), poly1d([2, -3, 11])) # shift right 1
        self.assertEqual(shift_poly(p, 0, -5), poly1d([2, 1, 5])) # shift down 5
        self.assertEqual(shift_poly(p, 0, 5), poly1d([2, 1, 15])) # shift up 5
        self.assertEqual(shift_poly(p, 1, 5), poly1d([2, -3, 16])) # shift up 5, right 1

    def test_shift_poly_3(self):
        # 3rd order
        p = poly1d([3, 2, 1, 10])
        self.assertEqual(shift_poly(p, -1, 0), poly1d([3, 11, 14, 16])) # shift left 1
        self.assertEqual(shift_poly(p, 1, 0), poly1d([3, -7, 6, 8])) # shift right 1
        self.assertEqual(shift_poly(p, 0, -1), poly1d([3, 2, 1, 9])) # shift down 1
        self.assertEqual(shift_poly(p, 0, 1), poly1d([3, 2, 1, 11])) # shift up 1

class VehicleTestOneLoc(unittest.TestCase):
    def setUp(self):
        # Create a track consisting of a single long length location.
        common.track_segments[0] = layout.Edge(0, sys.maxint, sys.maxint, None, None)
        common.interface = MockInterface()
        Sim.initialize()


    def add_vehicle(self, loc, pos, speed):
        """Create a default vehicle with starting position and speed."""
        id = len(common.vehicles) # 0 based
        v = Vehicle(ID=id, loc=loc, length=100,
                            accel_max_norm=5.0, accel_min_norm=-5.0, jerk_max_norm=2.5,
                            accel_max_emerg=5.0, accel_min_emerg=-5.0, jerk_max_emerg=2.5,
                            v_mass=0, position=pos, speed=speed, payload=0)
        common.vehicles[id] = v
        Sim.activate(v, v.ctrl_loop())
        return v

    def test_simple_traversal(self):
        v = self.add_vehicle(common.track_segments[0], 0, 100)
        Sim.simulate(until=100)
        self.assertEqual(Sim.now(), 100)
        self.assertEqual(v.pos, 10000)

if __name__ == '__main__':
    SegmentSuite = unittest.TestLoader().loadTestsFromTestCase(SegmentTestBasic)
    TraversalSuite = unittest.TestLoader().loadTestsFromTestCase(TraversalTestBasic)
    PathSuite = unittest.TestLoader().loadTestsFromTestCase(PathTestBasic)
    MiscSuite = unittest.TestLoader().loadTestsFromTestCase(MiscTest)
    VehicleSuite = unittest.TestLoader().loadTestsFromTestCase(VehicleTestOneLoc)

    TestVehicleSuite = unittest.TestSuite([SegmentSuite, TraversalSuite, PathSuite, MiscSuite, VehicleSuite])
    unittest.TextTestRunner(verbosity=2).run(TestVehicleSuite)