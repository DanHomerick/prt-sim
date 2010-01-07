package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	import com.google.maps.MapMouseEvent;
	import com.google.maps.overlays.MarkerOptions;
	
	import flash.display.Shape;
	import flash.events.MouseEvent;
	import flash.geom.Point;

	public final class OriginMarker extends SnappingMarker
	{
		public var segments:Array; // an array of Segment objects that lie under the marker 
		
		public function OriginMarker(latlng:LatLng)
		{
			super(latlng);
			setOptions(new MarkerOptions({
						draggable: true,				
						icon: makeIcon(),
						clickable: false,
						hasShadow: false
				      }));

			Globals.map.addOverlay(this);
			Globals.map.addEventListener(MapMouseEvent.MOUSE_MOVE, onMapMouseMove);
			Globals.map.addEventListener(MouseEvent.MOUSE_MOVE, onMouseMove);
		}
		
		/** A small blue circle */
		private function makeIcon():Shape {
			var icon:Shape = new Shape();
			icon.graphics.lineStyle(3,0x6060FF); // Blue
			icon.graphics.drawCircle(0,0,5);
			return icon;
		}
		
		public function destroy():void {
			Globals.map.removeOverlay(this);
			Globals.map.removeEventListener(MapMouseEvent.MOUSE_MOVE, onMapMouseMove);
			Globals.map.removeEventListener(MouseEvent.MOUSE_MOVE, onMouseMove);
			Globals.originMarker = null;
		}
		
		/** Moves the origin marker if the ctrl key is held down. */
		public function onMapMouseMove(event:MapMouseEvent):void {		
			// when ctrl key down or the user is first starting, move the origin, rather than dest
			if (event.ctrlKey || !Globals.haveClicked) { 
				// snap to the TrackOverlay that the mouse is currently over (if any)
				snapTo(event.latLng);
				
			}
		}

		/** Checks if the marker is too far away from the overlay. If it is, cuts it loose. */
		public function onMouseMove(event:MouseEvent):void {			
			if (event.ctrlKey) {
				var myPos:Point = Globals.map.fromLatLngToViewport(getLatLng());
				var mousePos:Point = new Point(event.stageX, event.stageY);
				var maxDist:Number = TrackOverlay.hitLineThickness;
				if (Point.distance(myPos, mousePos) > maxDist) {
					overlay = null; // cut it loose
				}  
			}			
		}
	}
}