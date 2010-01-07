package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	import com.google.maps.MapMouseEvent;
	import com.google.maps.overlays.MarkerOptions;
	
	import flash.display.Shape;
	import flash.events.MouseEvent;
	import flash.geom.Point;

	public final class DestMarker extends SnappingMarker
	{ 				
		public function DestMarker(latlng:LatLng)
		{			
			super(latlng);			
			setOptions( new MarkerOptions({
						clickable: false,			
						icon: makeIcon(),
						hasShadow: false
				      }));						
			Globals.map.addOverlay(this);			
			Globals.map.addEventListener(MapMouseEvent.MOUSE_MOVE, onMapMouseMove);
			Globals.map.addEventListener(MouseEvent.MOUSE_MOVE, onMouseMove);
			Globals.map.addEventListener(MapMouseEvent.ROLL_OUT, onMapRollOut);
			visible = false;
		}
		
		/** A small, filled, yellow circle */
		private function makeIcon():Shape {
			var icon:Shape = new Shape();
			icon.graphics.lineStyle(3,0xFFFF60);
			icon.graphics.drawCircle(0,0,10);
			return icon;
		}

		public function destroy():void {
			Globals.map.removeOverlay(this);
			Globals.map.removeEventListener(MapMouseEvent.MOUSE_MOVE, onMapMouseMove);
			Globals.map.removeEventListener(MouseEvent.MOUSE_MOVE, onMouseMove);
			Globals.map.removeEventListener(MapMouseEvent.ROLL_OUT, onMapRollOut);
			Globals.destMarker = null;
		}
		
		public function onMapMouseMove(event:MapMouseEvent):void {
			 if (!event.ctrlKey && Globals.haveClicked) {
				// snap to the TrackOverlay that the mouse is currently over (if any)
				snapTo(event.latLng);
			}		
		}

		/** Checks if the marker is too far away from the overlay. If it is, cuts it loose. */
		public function onMouseMove(event:MouseEvent):void {			
			if (!event.ctrlKey) {
				var myPos:Point = Globals.map.fromLatLngToViewport(getLatLng());
				var mousePos:Point = new Point(event.stageX, event.stageY);
				var maxDist:Number = TrackOverlay.hitLineThickness;
				if (Point.distance(myPos, mousePos) > maxDist) {
					overlay = null; // cut it loose
				}
			}			
		}
		
		public function onMapRollOut(event:MapMouseEvent):void {
//			trace("DestMarker.onMapRollOut");
			if (!event.ctrlKey) {
				Undo.undo(Undo.PREVIEW); // remove the preview, if it exists
			}
		}
		
	}
}