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