package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	import com.google.maps.interfaces.IPane;
	
	import flash.display.Graphics;
	import flash.geom.Point;
	import flash.geom.Vector3D;
	import flash.utils.getQualifiedClassName;
	
	public class Utility
	{
		private static const wsRegExp:RegExp = new RegExp("^[\\s\\t]+|[\\s\\t]+$", "g"); // remove leading or trailing whitespace
		/** Returns a new string that has had space and tab characters (whitespace) stripped 
		 * from the start and end. */
		public static function stripWS(string:String):String {
			return string.replace(wsRegExp, "");
		}
		
		public static function refreshScreen():void {
			for each (var pane:IPane in [Globals.curvedTrackPane,
										 Globals.straightTrackPane,
										 Globals.vehiclePane,
										 Globals.stationPane,
										 Globals.markerPane]) {
				pane.updatePosition();
			}
		}

		/** Returns a latlng that is dist meters away from src, in line towards dest. Assumes
		 * that the curvature of the earth is insignificant over the distances involved, so
		 * it won't be accurate for very long spans.
		 * 
		 * @DEPRECATED */
		public static function calcLatLngFromDist(dist:uint, src:LatLng, dest:LatLng):LatLng {			
			var actualDist:Number = src.distanceFrom(dest);
			var deltaLat:Number = dest.lat() - src.lat();
			var deltaLng:Number = dest.lng() - src.lng();
						
			// scale the deltas by the ratio of targetRads to actualRads		
			var ratio:Number = dist / actualDist;
			var result:LatLng = new LatLng(src.lat() + deltaLat*ratio, src.lng() + deltaLng*ratio); 
			trace("desired dist: ", dist, "calc'd dist: ", result.distanceFrom(src));
			return result; 
		}    

		public static const EARTH_CIRCUMFERENCE:Number = LatLng.EARTH_RADIUS * 2 * Math.PI;

		/** Returns a vector containing the distance in meters from src to dest */
		public static function calcVectorFromLatLngs(src:LatLng, dest:LatLng):Vector3D {
			var lngRadians:Number = dest.lngRadians() - src.lngRadians();
			var latRadians:Number = dest.latRadians() - src.latRadians();
			
			// yMeters = EARTH_CIRCUMFERENCE * arcDegrees
			// yMeters = LatLng.EARTH_RADIUS*2*Math.PI * latRadians/(2*Math.PI)
			var yMeters:Number = LatLng.EARTH_RADIUS * latRadians;
			
			// xMeters = cos(latitude) * EARTH_CIRCUMFERENCE * arcDegrees
			var xMeters:Number = Math.cos((src.latRadians() + dest.latRadians())/2) * LatLng.EARTH_RADIUS * lngRadians;
			return new Vector3D(xMeters, yMeters);
		}
		
		/** Creates a new LatLng that is located where vec is pointing, assuming that vec originates at src.
		 * 
		 * @param src The reference position.
		 * @param vec Indicates distance and direction away from src for the returned latlng.
		 * @param dist (Optional, default is 0) When non-zero, rather than using the length of vec for distance, uses dist.
		 * @return A new LatLng at the desired point.
		 */
		public static function calcLatLngFromVector(src:LatLng, vec:Vector3D, dist:Number=0):LatLng {
			var latRad:Number;
			var lngRad:Number;
			if (dist) {
				latRad = (vec.y/vec.length * dist) / LatLng.EARTH_RADIUS + src.latRadians();
				lngRad = (vec.x/vec.length * dist) / (Math.cos(src.latRadians())*LatLng.EARTH_RADIUS) + src.lngRadians()
			} else {
				latRad = vec.y/LatLng.EARTH_RADIUS + src.latRadians();
				lngRad = vec.x/(Math.cos(src.latRadians())*LatLng.EARTH_RADIUS) + src.lngRadians();
			}
			return LatLng.fromRadians(latRad, lngRad);		
		}
	
	
//      /** A higher precision version. Appears to be unnecessary. */
//		public static function calcLatLngFromVector(src:LatLng, vec:Vector3D, dist:Number=0):LatLng
//		{
//			/* From: http://williams.best.vwh.net/avform.htm#LL
//			 *
//			 *  lat=asin(sin(lat1)*cos(d)+cos(lat1)*sin(d)*cos(tc))
//     		 *	IF (cos(lat)=0)
//        	 *		lon=lon1      // endpoint a pole
//     		 *	ELSE
//        	 *		lon=mod(lon1-asin(sin(tc)*sin(d)/cos(lat))+pi,2*pi)-pi
//			 *  ENDIF
//			 */
//			 
//			 var distRad:Number; // Distance in radians. Also, it's just rad.
//			 if (!dist) {
//			 	distRad = vec.length / LatLng.EARTH_RADIUS; // meters * (pi / (radius*pi))
//			 } else {
//			 	distRad = dist / LatLng.EARTH_RADIUS;
//			 }			 
//
//			 // "True course is defined as usual, as the angle between the course line and the local meridian measured clockwise."
//			 var tc:Number = Utility.signedAngleBetween(Vector3D.Y_AXIS, vec); 
//			 
//			 var latRad:Number = Math.asin(Math.sin(src.latRadians()) * Math.cos(distRad) + Math.cos(src.latRadians()) * Math.sin(distRad) * Math.cos(tc));
//			 var lngRad:Number;
//			 if (Math.cos(latRad) == 0) {
//			 	lngRad = src.lngRadians();
//			 } else {
//			 	lngRad = ((src.lngRadians() - Math.asin(Math.sin(tc) * Math.sin(distRad) / Math.cos(latRad)) + Math.PI) % (2*Math.PI)) - Math.PI
//			 }
//			 
//			 var result:LatLng = LatLng.fromRadians(latRad, lngRad);
//			 var cmp_result:LatLng = _calcLatLngFromVector(src, vec, dist);
//			 trace("Comparison:", result, cmp_result, result.distanceFrom(cmp_result)); 
//			 return result;
//		}
	
		/** Returns the angle by which vec1 would need to rotate in order to overlap vec2.
		 *  A positive number indicates a CCW rotation. A negative number indicates a CW rotation.
		 *  The number returned will always be in the range [PI .. -PI).
		 */
		public static function signedAngleBetween(vec1:Vector3D, vec2:Vector3D):Number {
			var angle:Number = Math.atan2(vec2.y, vec2.x) - Math.atan2(vec1.y, vec1.x);
			if (angle > Math.PI) {
				angle = -Math.PI + angle%Math.PI;
			} else if (angle <= -Math.PI) {
				angle = Math.PI - Math.abs(angle)%Math.PI
			}
			
			return angle;
		}

		/** A replacement for Vector3D.angleBetween. That function returns NaN when two nearly coincident vectors are
		 * compared. This funcion catches the problem and returns a reasonable result. 
		 */
		public static function angleBetween(v1:Vector3D, v2:Vector3D):Number {
			var n:Number = v1.dotProduct(v2)/(v1.length*v2.length);
			// Due to rounding errors, n may be just outside the valid range for acos: (-1, 1)
			if (n > 1.0) {
				return 0;
			} else if (n < -1.0) {
				return Math.PI;
			}
			return Math.acos(n);
		}

		/** Rotates a 2D vector by 'angle' radians counter clockwise around the z-axis. */
		public static function rotate(vec:Vector3D, angle:Number):void {
			var x:Number = vec.x;
			var y:Number = vec.y;
			vec.x = x * Math.cos(angle) - y * Math.sin(angle);
			vec.y = x * Math.sin(angle) + y * Math.cos(angle);
		}
			

		/** Draws an arc, starting at the current position. A positive value for radians indicates a CCW arc,
		 * a negative value indicates CW. Steps controls how many line segements are used to approximate the circle.
		 */
		public static function arcTo(g:Graphics, start:Point, center:Point, radians:Number, radius:Number, steps:Number=12):void {
			var x:Number;
			var y:Number;
			var angle:Number;
			
			g.moveTo(start.x, start.y);
			var startAngle:Number = Math.atan2( center.y - start.y , start.x - center.x ); // y inverted in screen coordinates
			var angleStep:Number = radians/steps;

		    // Draw a sequence of line segments to approximate the arc
		    for(var i:int = 1; i <= steps; i++) {		        
		        angle = startAngle + i * angleStep; // Increment the angle by angleStep		       
		        x = center.x + Math.cos(angle) * radius;
		        y = center.y - Math.sin(angle) * radius;
		        g.lineTo(x, y);
			}
		}

		/** Strips the package info from the classname */
		public static function getClassName(o:Object):String {
			var fullClassName:String = getQualifiedClassName(o);
			return fullClassName.slice(fullClassName.lastIndexOf("::") + 2);
		}

		public static function truncateTo(x:Number, places:int):Number {
			var power:int = Math.pow(10, places);
			return int(x*power) / power; 
		}
		
		public static function compareElevations(x:Number, y:Number):Number {
			var int_x:int = x*10; // keep precision to 1/10th of a meter
			var int_y:int = y*10;
			return int_x - int_y;
		}

		/** Converts a string id to just the leading integer. */
		public static function toIntegerId(id:String):Number {
			var parts:Array = id.split("_");
			return Number(parts[0]);			
		}
		
		public static function compareIds(a:Object, b:Object):int {
            var aInt:int = Utility.toIntegerId(a.id);
            var bInt:int = Utility.toIntegerId(b.id);
            if (aInt > bInt) return -1;
            else if (aInt == bInt) return 0;
            else return 1; // (aInt < bInt)
  		}

		/**Returns the distance required to do an acceleration change. Assumes that initial and final accelerations are zero.
		 * 
		 * @param v_initial The initial velocity.
		 * @param v_final The final velocity.
		 * @param max_accel The absolute value of the maximum acceleration or deceleration to be used.
		 * @param max_jerk The absolute value of the maximum jerk.
		 * @return Distance required to perform the velocity change. 
		 * */
		public static function distanceFromVelocityChange(v_initial:Number, v_final:Number, max_accel:Number, max_jerk:Number):Number {
			var delta_v:Number = v_final - v_initial;
			var j_initial:Number; // The first jerk value to use
			var j_final:Number;   // The last jerk value to use
			if (delta_v == 0) {
				return 0;
			} else if (delta_v < 0) {
				j_initial = -max_jerk;
				j_final = max_jerk
			} else {
				j_initial = max_jerk;
				j_final = -max_jerk;
			}
			
	        /* See prob3 in trajectory_calcs.py
             *
	         *                                  2      2
	         *             2 / 1      1  \   a0     a2
	         * v2 - v0 + at *|---- - ----| + ---- - ----
	         *               \2*jn   2*jx/   2*jx   2*jn
	         *
	         * In this case, a0 and a2 are both 0.	         
			 */
			var a:Number = 1/(2*j_final) - 1/(2*j_initial);
			var b:Number = 0
			var c:Number = delta_v
			
			/* Quadratic formula simplified by b==0:
			 *   +/- sqrt(-AC)/A
			 */
			var accel_peak:Number = -Math.sqrt(-a*c)/a; // Acceleration peak if there is not limit (or it's not reached).
			var accel_limit:Number;
			if (delta_v > 0) {
				accel_limit = Math.min(max_accel, accel_peak);
			} else {
				accel_limit = Math.max(-max_accel, accel_peak);
			}
						
			var t01:Number = accel_limit/j_initial; // How many seconds it takes to reach accel_limit
			var t23:Number = t01;                   // By symmetry, how many seconds to return to zero acceleration.
			var v1:Number = j_initial*t01*t01/2.0 + v_initial;  // Velocity at start of constant accel segment
			var v2:Number = j_final*t23*t23/2.0 + v_final;    // Velocity at end of constant accel segment
			var v12:Number = v2 - v1;   // How much the velocity needs to change during constant accel.
			var t12:Number;             // How many seconds to hold constant acceleration to get desired velocity change
			if (Math.abs(v12) < 1E-8) {
				v12 = 0.0;
				t12 = 0.0;
			} else {
				t12 = v12/accel_limit;
			}
 						
			var dist:Number = j_initial*t01*t01*t01/6.0 + v_initial*t01  // Increasing accel. Note that initial accel = 0			
			                + accel_limit*t12*t12/2.0 + v1*t12           // Constant accel
			                + j_final*t23*t23*t23/6.0 + accel_limit*t23*t23/2.0 + v2*t23;          // Decreasing accel.
			return dist;		
		}
		 

		// Looks good. Off by at most a couple meters at high latitude over a full degree (7000 m)
		public static function unitTest():void {
			trace("Utility.unitTest!!!!!");
			
			var origin:LatLng = new LatLng(0, 0);
			var northPole:LatLng = new LatLng(90,0);
			var midLat:LatLng = new LatLng(45,0);
			var origin_0_1:LatLng = new LatLng(0,1);
			var origin_1_0:LatLng = new LatLng(1,0);
			var midLat_45_1:LatLng = new LatLng(45,1);
			var midLat_46_0:LatLng = new LatLng(46,0);
			var midLat_46_1:LatLng = new LatLng(46,1);
			var np_89_10:LatLng = new LatLng(89,10);
			var np_89_11:LatLng = new LatLng(89,11);
			
			var dist_1:Vector3D = calcVectorFromLatLngs(origin, northPole);
			trace("#1: ", origin.distanceFrom(northPole), dist_1.x, dist_1.y, dist_1.length);
			
			var dist_2:Vector3D = calcVectorFromLatLngs(origin, origin_0_1);
			trace("#2: ", origin.distanceFrom(origin_0_1), dist_2.x, dist_2.y, dist_2.length);

			var dist_3:Vector3D = calcVectorFromLatLngs(origin, origin_1_0);
			trace("#3: ", origin.distanceFrom(origin_1_0), dist_3.x, dist_3.y, dist_3.length);
		
			var dist_4:Vector3D = calcVectorFromLatLngs(midLat, midLat_45_1);
			trace("#4: ", midLat.distanceFrom(midLat_45_1), dist_4.x, dist_4.y, dist_4.length);

			var dist_5:Vector3D = calcVectorFromLatLngs(midLat, midLat_46_0);
			trace("#5: ", midLat.distanceFrom(midLat_46_0), dist_5.x, dist_5.y, dist_5.length);
			
			var dist_6:Vector3D = calcVectorFromLatLngs(np_89_10, np_89_11);
			trace("#6: ", np_89_10.distanceFrom(np_89_11), dist_6.x, dist_6.y, dist_6.length);
			
			var dist_7:Vector3D = calcVectorFromLatLngs(midLat, midLat_46_1);
			trace("#7: ", midLat.distanceFrom(midLat_46_1), dist_7.x, dist_7.y, dist_7.length);
			
			// do roundtrip conversions
			trace("#1' ", northPole, calcLatLngFromVector(origin, dist_1));			
			trace("#2' ", origin_0_1, calcLatLngFromVector(origin, dist_2));			
			trace("#3' ", origin_1_0, calcLatLngFromVector(origin, dist_3));			
			trace("#4' ", midLat_45_1, calcLatLngFromVector(midLat, dist_4));
			trace("#5' ", midLat_46_0, calcLatLngFromVector(midLat, dist_5));
			trace("#6' ", np_89_11, calcLatLngFromVector(np_89_10, dist_6));
			trace("#7' ", midLat_46_1, calcLatLngFromVector(midLat, dist_7));

		}
	}
}