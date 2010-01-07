package edu.ucsc.neartree
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import flexunit.framework.TestCase;

	public class NearTreeTest extends TestCase
	{
		protected var tree:NearTree
		
		override public function setUp():void {
			tree = new NearTree();
			
			// 1 Degree apart, very far
			tree.insert(new LatLng(1,1), "1,1");
			tree.insert(new LatLng(1,2), "1,2");
			tree.insert(new LatLng(2,1), "2,1");
			tree.insert(new LatLng(2,2), "2,2");
			
			// ~0.157 meters apart. Will be merged.
			tree.insert(new LatLng(1.3, 1.3), "1.3,1.3");
			tree.insert(new LatLng(1.300001, 1.300001), "1.300001,1.300001");
		}
		
		/** Randomly creates 100 LatLng's in the 0,0 -> 1,1 region and addes them to <code>tree</code>.
		 * @returns Contains the LatLng instances that were added to tree.
		 */ 
		public function setUp2():Vector.<LatLng> {
			tree = new NearTree();
			
			/* create random LatLng values and insert them into the tree */
			var latlngs:Vector.<LatLng> = new Vector.<LatLng>();
			for (var i:int=0; i<100; ++i) {
				latlngs.push(new LatLng(Math.random(), Math.random())); // all between 0 and 1
			}			
			for each (var latlng:LatLng in latlngs) {
				tree.insert(latlng, null); // no payload
			}
			return latlngs;
		}
		
		
		public function test_nearest():void {
			var result:Result = tree.nearest(new LatLng(1.1,1.1));
			assertTrue(result.latlng.equals(new LatLng(1,1)));
			assertEquals("1,1", result.objs[0]);
			
			result = tree.nearest(new LatLng(1.31,1.31)); // nearl 1.3,1.3 cluster
			assertTrue(result.latlng.equals(new LatLng(1.3,1.3)));
			assertEquals(2, result.objs.length);
			assertEquals("1.3,1.3", result.objs[0]);
			assertEquals("1.300001,1.300001", result.objs[1]);
			
			result = tree.nearest(new LatLng(0.9,0.9)); // near 1,1
			assertTrue(result.latlng.equals(new LatLng(1,1)));
			assertEquals(1, result.objs.length);
			assertEquals("1,1", result.objs[0]);
			
			result = tree.nearest(new LatLng(3,3)); // nearest to 2,2
			assertTrue(result.latlng.equals(new LatLng(2,2)));
			assertEquals(1, result.objs.length);
			assertEquals("2,2", result.objs[0]);
		}
		
		/** Looking for the nearest LatLng to a LatLng that is stored in the tree should always give a match. */
		public function test_nearest_II():void {
			var vec:Vector.<LatLng> = setUp2();			
			for each (var latlng:LatLng in vec) {
				var result:Result = tree.nearest(latlng);
				assertStrictlyEquals(latlng, result.latlng);
			}		
		}
		
		/** Probing at random locations gives the correct results. */
		public function test_nearest_III():void {
			var vec:Vector.<LatLng> = setUp2();
			for (var i:int=0; i<50; ++i) {
				var probe:LatLng = new LatLng(Math.random(), Math.random());
				var result:Result = tree.nearest(probe);
				var closest:LatLng = findClosest(probe, vec);
				assertEquals(probe.distanceFrom(closest), probe.distanceFrom(result.latlng));
			}
		}
		
		public function test_withinRadius():void {
			var vec:Vector.<Result> = tree.withinRadius(new LatLng(1.0001,1.0001), 1000) // 1 km
			assertEquals(1, vec.length);
			var result:Result = vec[0];
			assertTrue(result.latlng.equals(new LatLng(1,1)));
			assertEquals(1, result.objs.length);
			assertEquals("1,1", result.objs[0]);
			
			vec = tree.withinRadius(new LatLng(1.5,1.5), 10000000); // 10,000km
			assertEquals(5, vec.length); // not 6, since two latlngs were merged into one result
		}
		
		public function test_withinRadius_II():void {
			var vec:Vector.<LatLng> = setUp2();
			
			for (var i:int=0; i<50; ++i) { // 50 tests
				var probe:LatLng = new LatLng(Math.random(), Math.random());
				var radius:Number = Math.random()*100000; // between 0 meters and 100 kilometers
				var results:Vector.<Result> = tree.withinRadius(probe, radius);
				var truth:Vector.<LatLng> = findWithin(probe, radius, vec);
				
				var testValues:Vector.<LatLng> = new Vector.<LatLng>();
				for each (var result:Result in results) {
					testValues.push(result.latlng);
				}
				
				assertTrue(latlngVecsMatch(truth, testValues));
			}
		}
		
		/** @param vec A vector containing all LatLng instances in the tree. Returned by setUp2. */
		protected function findClosest(probe:LatLng, vec:Vector.<LatLng>):LatLng {
			var dist:Number = Number.MAX_VALUE;
			var closest:LatLng;
			for each (var latlng:LatLng in vec) {
				var tmpDist:Number = probe.distanceFrom(latlng);
				if (tmpDist < dist) {
					dist = tmpDist;
					closest = latlng;
				}
			}
			return closest;
		}
		
		/** @param vec A vector containing all LatLng instances in the tree. Returned by setUp2. */
		protected function findWithin(probe:LatLng, radius:Number, vec:Vector.<LatLng>):Vector.<LatLng> {
			var ret_value:Vector.<LatLng> = new Vector.<LatLng>();
			for each (var latlng:LatLng in vec) {
				if (probe.distanceFrom(latlng) <= radius) {
					ret_value.push(latlng);
				}
			}
			return ret_value;
		}
		
		/** Checks that two LatLng Vectors have contents that equal each other, without regard to position within the vector. */
		protected function latlngVecsMatch(vec1:Vector.<LatLng>, vec2:Vector.<LatLng>):Boolean {
			if (vec1.length != vec2.length) {
				return false;
			}
			
			var v2:Vector.<LatLng> = vec2.concat(); // make copy
			var numRemoved:uint = 0;
			for each (var a:LatLng in vec1) {
				for (var i:int = 0; i < v2.length; ++i) {
					if (a.equals(v2[i])) {
						if (i == v2.length-1) {
							v2.pop();
						} else {
							v2[i] = v2.pop();
						}
						++numRemoved;						
					}
				}
			}
			if (numRemoved == vec1.length && v2.length == 0) {
				return true;
			} else {
				return false;
			}
		}
	}
}