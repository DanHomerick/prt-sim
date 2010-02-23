import unittest

import SpeedProfiler

from SpeedProfiler import SpeedProfiler as sp
from numpy import arange

# sp init parameters:
#    v_init, v_final, max_accel, max_jerk=2.5, mass=450,
#      a_init=0, a_final=0, p_init=0, step=0.1):

POS = 0
VEL = 1
ACCEL = 2
JERK = 3

class SPTest(unittest.TestCase):
    
    def test_3segment_0initAccel_0initVel(self):
        # Accelerate to 100 from a standstill. 3 Segments
        initial = [0,0,0] # pos, vel, accel
        final  = (None, 100, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 4) # 3 segs, plus end seg
        # accel matches expected profile
        self.assertAlmostEqual(eqns[1][ACCEL](0), a_m)
        self.assertAlmostEqual(eqns[2][ACCEL](0), a_m)
        self.assertAlmostEqual(eqns[3][ACCEL](0), 0)
        self.assertAlmostEqual(eqns[3][VEL](0), final[VEL])
    
    def test_3segment_5initAccel_0initVel(self):
        # Accelerate to 100 with small positive initial accel
        initial = [0,0,5] # pos, vel, accel
        final  = (None, 100, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 4) # 3 segs, plus end seg
        # accel matches expected profile
        self.assertAlmostEqual(eqns[1][ACCEL](0), a_m)
        self.assertAlmostEqual(eqns[1][VEL](0), 15)
        self.assertAlmostEqual(eqns[2][ACCEL](0), a_m)
        self.assertAlmostEqual(eqns[3][ACCEL](0), 0)
        self.assertAlmostEqual(eqns[3][VEL](0), final[VEL])

    def test_3segment_neg5initAccel_0initVel(self):
        # Accelerate to 100 with small neg initial accel
        initial = [0,0,-5] # pos, vel, accel
        final  = (None, 100, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 4) # 3 segs, plus end seg
        # accel matches expected profile
        self.assertAlmostEqual(eqns[0][ACCEL](2), 0) # removed initial accel
        self.assertAlmostEqual(eqns[0][VEL](2), -5) 
        self.assertAlmostEqual(eqns[1][ACCEL](0), a_m)
        self.assertAlmostEqual(eqns[2][ACCEL](0), a_m)
        self.assertAlmostEqual(eqns[3][ACCEL](0), 0)
        self.assertAlmostEqual(eqns[3][VEL](0), final[VEL])


    def test_2segment_5initAccel_0initVel(self):
        # Accelerate to 10 with small pos initial accel. Does not reach max_accel.
        initial = [0,0,5] # pos, vel, accel
        final  = (None, 10, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 3) # 2 segs, plus end seg
        # accel matches expected profile
        self.assertTrue(eqns[1][ACCEL](0) <= a_m)
        self.assertTrue(eqns[1][ACCEL](0) >= d_m)
        self.assertAlmostEqual(eqns[2][ACCEL](0), 0)
        self.assertAlmostEqual(eqns[2][VEL](0), final[VEL])

    def test_2segment_neg5initAccel_0initVel(self):
        # Accelerate to 10 with small neg initial accel.
        initial = [0,0,-5] # pos, vel, accel
        final  = (None, 5, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 3) # 2 segs, plus end seg
        # accel matches expected profile
        self.assertTrue(eqns[1][ACCEL](0) <= a_m)
        self.assertTrue(eqns[1][ACCEL](0) >= d_m)
        self.assertAlmostEqual(eqns[2][ACCEL](0), 0)
        self.assertAlmostEqual(eqns[2][VEL](0), final[VEL])

    def test_2segment_10initAccel_0initVel(self):
        # Small positive velocity change, but starts with large initial accel.
        # Proper solution is to remove pos accel, then brake for a bit to reduce
        # speed back to target.
        initial = [0,0,10] # pos, vel, accel
        final  = (None, 10, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.assertAlmostEqual(durs[0], 6)
        self.assertAlmostEqual(durs[1], 2)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 3) # 2 segs, plus end seg
        # accel matches expected profile
        self.assertAlmostEqual(eqns[0][ACCEL](4), 0)
        self.assertTrue(eqns[1][ACCEL](0) <= a_m)
        self.assertTrue(eqns[1][ACCEL](0) >= d_m)
        self.assertAlmostEqual(eqns[2][ACCEL](0), 0)
        self.assertAlmostEqual(eqns[2][VEL](0), final[VEL])
 
    def test_1segment_5initAccel_ontarget(self):
        # Positive velocity change with initial accel s.t. reducing accel to 0
        # leaves the vehicle at target velocity.
        initial = [0,0,5] # pos, vel, accel
        final  = (None, 5, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.assertAlmostEqual(durs[0], 2)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 2) # 1 seg, plus end seg
        # accel matches expected profile
        self.assertAlmostEqual(eqns[0][ACCEL](2), 0)

    def test_2segment_5initAccel_overshoot(self):
        # Positive velocity change with initial accel s.t. reducing accel to 0
        # will overshoot target velocity.
        initial = [0,0,5] # pos, vel, accel
        final  = (None, 2, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs = self.get_segs(initial, final, a_m, d_m, j_m)
        eqns = self.get_eqns(segs)
        durs = self.get_durs(segs)
        self.continuity_test(eqns, durs, initial)
        self.assertEqual(len(segs), 3) # 2 seg, plus end seg
        # accel matches expected profile
        self.assertAlmostEqual(eqns[0][ACCEL](2), 0)
        self.assertAlmostEqual(eqns[0][VEL](2), 5)        

    def test_20initPos(self):
        # 3 segment test with an initial position
        initial20 = [20,0,0] # pos, vel, accel
        final  = (None, 100, 0) # pos, vel, accel
        a_m = 10   # max accel
        d_m = -10   # max decel
        j_m = 2.5  # max jerk
        segs20 = self.get_segs(initial20, final, a_m, d_m, j_m)
        eqns20 = self.get_eqns(segs20)
        durs20 = self.get_durs(segs20)
        self.continuity_test(eqns20, durs20, initial20)
        
        # compare to a run without initial position
        initial0 = [0,0,0] # pos, vel, accel
        segs0 = self.get_segs(initial0, final, a_m, d_m, j_m)
        eqns0 = self.get_eqns(segs0)
        durs0 = self.get_durs(segs0)

        for seg0, dur0, seg20, dur20 in zip(eqns0, durs0, eqns20, durs20):
            self.assertEqual(dur0, dur20) # must have same duration segments
            for t in arange(0, dur0+0.1, 0.1): # sample at 0.1 sec increments
                # vel and accel should be identical at same times
                for i in (VEL, ACCEL):
                    self.assertAlmostEqual(seg0[i](t), seg20[i](t))
                # pos should be offset by 20
                self.assertAlmostEqual(seg0[POS](t) + 20.0, seg20[POS](t))
                    
    def get_segs(self, initial, final, a_m, d_m, j_m):
        self.profiler = sp(p_init  = initial[POS],
              v_init  = initial[VEL],
              v_final = final[VEL],
              a_init  = initial[ACCEL],
              a_final = final[ACCEL],
              max_accel=a_m,
              max_decel=d_m,
              max_jerk=j_m)
        segs = self.profiler.get_profile()
        return segs

    def get_eqns(self, segs):
        """Returns a 2D matrix of eqns. Contains POS through ACCEL eqns
        for each segment."""
        s = list()
        for i in xrange(len(segs)):
            e = list()
            s.append(e)
            for j in xrange(3):  # POS through ACCEL
                e.append(segs[i][0].deriv(j))
        return s

    def get_durs(self, segs):
        """Returns a list of segment durations. Verifies that all are positive."""
        durs = list()
        for s in segs:
            durs.append(s[1])
        
        for d in durs:
            self.assertTrue(d >= 0 or d is None)
            
        return durs

    def continuity_test(self, eqns, durs, initial):
        """Check that trajectory is continuous."""
        for seg, dur in zip(eqns, durs):
            for i in xrange(3):  # POS through ACCEL
                self.assertAlmostEqual(seg[i](0), initial[i])
                if dur:
                    initial[i] = seg[i](dur)

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(SPTest)
    unittest.TextTestRunner(verbosity=2).run(suite)