import unittest

if __debug__:
    import SimPy.SimulationTrace as Sim
else:
    import SimPy.Simulation as Sim

import pyprt.sim.layout as layout
import pyprt.sim.vehicle as vehicle

Sim.initialize()

class StationTest(unittest.TestCase):

    def setUp(self):
        self.s = layout.Station(ID=1, length=68, max_speed=100, x=0, y=0, label='', v_adv_time=0.5,
                 unload_size=3, queue_size=3, load_size=3, storage_size=5,
                 policy='QUEUE')

    def test_advance_queue_no_vehicles(self):
        # No vehicles, no change
        q = self.s.queue
        self.s.advance_queue(self.s.queue)
        self.assertEqual(q, self.s.queue)

    def test_advance_queue_one_vehicle(self):
        # Assumes queue is a list. Place vehicle at entry slot
        self.s.queue[2] = 'V'
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, [None, 'V', None])
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, ['V', None, None])
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, ['V', None, None])

    def test_advance_queue_two_vehicles(self):
        self.s.queue[2] = 'V2'
        self.s.queue[1] = 'V1'
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, ['V1', 'V2', None])
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, ['V1', 'V2', None])

    def test_advance_queue_two_vehicles2(self):
        self.s.queue[2] = 'V2'
        self.s.queue[0] = 'V1'
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, ['V1', 'V2', None])
        self.s.advance_queue(self.s.queue)
        self.assertEqual(self.s.queue, ['V1', 'V2', None])

    def test_advance_platform_nobusy(self):
        b0 = self.s.load_platform[0]
        b1 = self.s.load_platform[1]
        b2 = self.s.load_platform[2]
        b1.vehicle = 'V1'
        b2.vehicle = 'V2'
        self.s.advance_platform(self.s.load_platform)
        self.assertEqual(b0.vehicle, 'V1')
        self.assertEqual(b1.vehicle, 'V2')
        self.assertEqual(b2.vehicle, None)

    def test_advance_platform_nobusy_end(self):
        b0 = self.s.load_platform[0]
        b1 = self.s.load_platform[1]
        b2 = self.s.load_platform[2]
        b2.vehicle = 'V1'
        self.s.advance_platform(self.s.load_platform)
        self.assertEqual(b0.vehicle, None)
        self.assertEqual(b1.vehicle, 'V1')
        self.assertEqual(b2.vehicle, None)

    def test_advance_platform_busy(self):
        b0 = self.s.load_platform[0]
        b1 = self.s.load_platform[1]
        b2 = self.s.load_platform[2]
        b1.vehicle = 'V1'
        b1.busy = True
        b2.vehicle = 'V2'
        self.s.advance_platform(self.s.load_platform)
        self.assertEqual(b0.vehicle, None)
        self.assertEqual(b1.vehicle, 'V1')
        self.assertEqual(b2.vehicle, 'V2')

if __name__ == '__main__':
    unittest.main()
##    suite = unittest.TestLoader().loadTestsFromTestCase(StationTest)
##    unittest.TextTestRunner(verbosity=2).run(suite)
