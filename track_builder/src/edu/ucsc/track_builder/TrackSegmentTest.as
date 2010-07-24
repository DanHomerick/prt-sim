package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	
	import flash.geom.Vector3D;
	
	import flexunit.framework.TestCase;

	public class TrackSegmentTest extends TestCase
	{
//		override public function setUp():void {	}
//		override public function tearDown():void { }
	
		/** Factory function */
		public function makeStraightSeg(id:String, start:LatLng, end:LatLng):TrackSegment
		{
			return new TrackSegment(id, "", start.clone(), end.clone(), 20, 3, 3, 0, 0, null, false);
		}
		
		/** Factory function */
		public function makeCurvedSeg(id:String, start:LatLng, end:LatLng, radius:Number, center:LatLng):TrackSegment
		{
			var v1:Vector3D = Utility.calcVectorFromLatLngs(center, start);
			var v2:Vector3D = Utility.calcVectorFromLatLngs(center, end);
			var angle:Number = Utility.angleBetween(v1, v2);
			var z:Vector3D = v1.crossProduct(v2);
			if (z.z < 0) { // angle from v1 to v2 is clockwise. I think I have my convention backwards. =(
				angle = -angle;
			}
			var arcAngle:Number = angle * radius;
			return new TrackSegment(id, "", start.clone(), end.clone(), 20, 3, 3, radius, arcAngle, center.clone(), false);
		}  
			
		/** Disconnects the segment from all other segments. */
		public function disconnect(seg:TrackSegment):void
		{
			seg.next_ids.splice(0, seg.next_ids.length); // delete contents
			seg.prev_ids.splice(0, seg.prev_ids.length);		
		}

		public function isDisconnected(seg:TrackSegment):Boolean
		{
			return !seg.next_ids.length && !seg.prev_ids.length;
		}

		public function areConnected(seg1:TrackSegment, seg2:TrackSegment):Boolean
		{
			var optionA:Boolean = seg1.next_ids.length == 1 && seg2.prev_ids.length == 1
			                      && seg1.next_ids[0] == seg2.id && seg2.prev_ids[0] == seg1.id;
			var optionB:Boolean = seg1.prev_ids.length == 1 && seg2.next_ids.length == 1
			                      && seg1.prev_ids[0] == seg2.id && seg2.next_ids[0] == seg1.id;
			return optionA || optionB;
		}

		public function checkConnections(noConnects:Array, connects:Array):void
		{
			var iterArray:Array;
			var seg1:TrackSegment;
			var seg2:TrackSegment;
			for each (iterArray in noConnects) {
				seg1 = iterArray[0];
				seg2 = iterArray[1]; 
				seg1.connect(seg2);
				assertTrue(isDisconnected(seg1));
				assertTrue(isDisconnected(seg2));
				disconnect(seg1); // Probably pointless? Don't think this code will run if assertTrue fails.
				disconnect(seg2);
			}
			
			for each (iterArray in connects) {
				seg1 = iterArray[0];
				seg2 = iterArray[1];
				seg1.connect(seg2);
				assertTrue(areConnected(seg1, seg2));
				disconnect(seg1);
				disconnect(seg2);
			}
		}

		/** Test 5 main types of connections:
		 * 1. Two straight segments.
		 * 2. One straight, one curved; curved rotates away from the straight.
		 * 3. One straight, one curved; curved rotates towards straight.
		 * 4. Two curved; curved rotates away from each other.
		 * 5. Two curved; curved rotates towards each other.
		 *
		 * In all 5 types, there are 4 cases which need to be tested.
		 * Case 1. Nose to Tail
		 * Case 2. Nose to Nose
		 * Case 3. Tail to Tail
		 * Case 4. Skewed angle (e.g. perpendicular intersections)			 
		 */		
		public function test_connect():void {
			/* Set up a grid of 6 equally spaced LatLng's, like so:
			 *    .  .  .
			 *    .  .  .
			 */
			var spacing:Number = 10; // how many meters apart each latlng is.
			var tl:LatLng = new LatLng(20, 20); // top left. Arbitrary position.
			var tc:LatLng = Utility.calcLatLngFromVector(tl, new Vector3D(spacing, 0, 0)); // top center
			var tr:LatLng = Utility.calcLatLngFromVector(tl, new Vector3D(2*spacing, 0, 0)); // top right
			var bl:LatLng = Utility.calcLatLngFromVector(tl, new Vector3D(0, -spacing, 0)); // bottom left
			var bc:LatLng = Utility.calcLatLngFromVector(bl, new Vector3D(spacing, 0, 0)); // bottom center
			var br:LatLng = Utility.calcLatLngFromVector(bl, new Vector3D(2*spacing, 0, 0)); // bottom center			 
			 
			/* Type 1. Two straight segments. */
			var s_bl_bc:TrackSegment = makeStraightSeg("s_bl_bc", bl, bc); // straight. From bottom left to bottom center
			var s_bc_bl:TrackSegment = makeStraightSeg("s_bc_bl", bc, bl);
			var s_bc_br:TrackSegment = makeStraightSeg("s_bc_br", bc, br);
			var s_br_bc:TrackSegment = makeStraightSeg("s_br_bc", br, bc);
			var s_bc_tr:TrackSegment = makeStraightSeg("s_bc_tr", bc, tr); // on diagonal
			 			
			var noConnects:Array = [[s_bl_bc,s_bc_bl], // doubles back
			   					    [s_bl_bc,s_br_bc], // nose to nose
									[s_bc_bl,s_bc_br], // tail to tail
			 						[s_bl_bc,s_bc_tr]]; // skewed
			var connects:Array = [[s_bl_bc,s_bc_br]];			
			checkConnections(noConnects, connects);
			
			/* Type 2. One straight, one curved; curved rotates away from straight. */
			var c_bc_tr:TrackSegment = makeCurvedSeg("c_bc_tr", bc, tr, spacing, tc);
			var c_tr_bc:TrackSegment = makeCurvedSeg("c_tr_bc", tr, bc, spacing, tc);
			var c_bc_tr_skew:TrackSegment = makeCurvedSeg("c_bc_tr_skew", bc, tr, spacing, br); // perpendicalar to the straight segs
			
			noConnects = [[s_bl_bc,c_tr_bc], // nose to nose
			              [s_bc_bl,c_bc_tr], // tail to tail
			              [s_bl_bc,c_bc_tr_skew]] // perpendicular intersection
			connects = [[s_bl_bc,c_bc_tr], [s_bc_bl,c_tr_bc]];
			checkConnections(noConnects, connects);
			
			/* Type 3. One straight, one curved; curved rotates towards staight. */
			var c_bc_tl:TrackSegment = makeCurvedSeg("c_bc_tl", bc, tl, spacing, tc);
			var c_tl_bc:TrackSegment = makeCurvedSeg("c_tl_bc", tl, bc, spacing, tc);
			var c_tl_bc_skew:TrackSegment = makeCurvedSeg("c_tl_bc_skew", tl, bc, spacing, bl); // perpendicular to the straight segs
			
			noConnects = [[s_bl_bc,c_bc_tl], // nose to tail, but wrong angle
			              [s_bl_bc,c_tl_bc], // nose to nose
			              [s_bc_bl,c_bc_tl], // tail to tail
			              [s_bc_bl,c_tl_bc], // nose to tail, but wrong angle
			              [s_bc_bl,c_tl_bc_skew]]; // perpendicular intersection
			connects = [];
			checkConnections(noConnects, connects);
			
			/* Type 4. Two curved; curves rotates away from each other. */
			noConnects = [[c_tl_bc,c_tr_bc], // nose to nose
			              [c_bc_tl,c_bc_tr], // tail to tail
			              [c_tl_bc,c_bc_tr_skew]]; // perpendicular intersection
			connects = [[c_tl_bc,c_bc_tr]];
			checkConnections(noConnects, connects);
			
			/* Type 5. Two curved; curved rotates towards each other. */
			noConnects = [[c_tl_bc_skew,c_bc_tl]];
			connects = [];
			checkConnections(noConnects, connects);
		}
	}
}