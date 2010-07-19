package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.Color;
	import com.google.maps.LatLng;
	import com.google.maps.MapMouseEvent;
	import com.google.maps.overlays.Marker;
	import com.google.maps.overlays.MarkerOptions;
	
	import flash.display.Shape;
	import flash.events.MouseEvent;
	import flash.geom.Point;

	public class SnappingMarker extends Marker
	{	
		/* Constructor */		
		public function SnappingMarker(latlng:LatLng, icon:Shape, visible:Boolean)
		{
			super(latlng);
			setOptions( new MarkerOptions({
						clickable: false,			
						icon: icon,
						hasShadow: false
				      }));
			Globals.map.addOverlay(this);
			this.visible = visible;
		}

		protected var _overlay:TrackOverlay;
		public function getSnapOverlay():TrackOverlay {return _overlay;}
		public function setSnapOverlay(overlay:TrackOverlay):void {
			this._overlay = overlay;
		}

		public function destroy():void {
			Globals.map.removeOverlay(this);
		}

		public function snapTo(latlng:LatLng=null):void {
			if (_overlay != null) {
				if (latlng == null) {
					setLatLng(_overlay.snapTo(null, getLatLng()));
				} else {
					setLatLng(_overlay.snapTo(null, latlng));
				}
			} else {
				if (latlng == null) {
					; // do nothing
				} else { // set the position, even though there's no _overlay to snap to.
					setLatLng(latlng);
				} 
			}
		}

		/** Returns a vector containing all TrackOverlays that are beneath the centerpoint of the marker.
		 * This function is not constant time! It's O(n) where n is the number of track overlays. */
		public function getTrackOverlays():Vector.<TrackOverlay> {
			var pt:Point = Globals.straightTrackPane.fromLatLngToPaneCoords(getLatLng());
			var results:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();
			for each (var o:TrackOverlay in Globals.tracks.overlays) {				  
				if (o.hitTestPoint(pt.x, pt.y, true)) {
					results.push(o);
				}
			}
			return results;
		}

		public function onMapMouseMove(event:MapMouseEvent):void {
			// snap to the TrackOverlay that the mouse is currently over (if any)
			snapTo(event.latLng);		
		}
		
		/** Checks if the marker is too far away from the _overlay. If it is, cuts it loose. */
		public function onMouseMove(event:MouseEvent):void {			
			var myPos:Point = Globals.map.fromLatLngToViewport(getLatLng());
			var mousePos:Point = new Point(event.stageX, event.stageY);
			var maxDist:Number = TrackOverlay.hitLineThickness;
			if (Point.distance(myPos, mousePos) > maxDist) {
				_overlay = null; // cut it loose
			}
		}

		public function onMapRollOut(event:MapMouseEvent):void {			
			Undo.undo(Undo.PREVIEW); // remove the preview, if it exists
		}
		
		public static function makeCyanCircleIcon():Shape {
			return makeCircleIcon(5, 3, Color.CYAN, 1);
		}
		
		public static function makeBlueCircleIcon():Shape {
			return makeCircleIcon(5, 3, Color.BLUE, 0.7);	
		}
		
		public static function makeCircleIcon(radius:Number, thickness:Number, color:uint, alpha:Number):Shape {
			var icon:Shape = new Shape();
			icon.graphics.lineStyle(thickness, color, alpha);
			icon.graphics.drawCircle(0,0,radius);
			return icon;
		}		
	}
}