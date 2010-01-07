package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.overlays.Marker;
	
	import flash.events.MouseEvent;
	import flash.geom.Point;

	public class SnappingMarker extends Marker
	{
		public var overlay:TrackOverlay;

		/* Constructor */		
		public function SnappingMarker(latlng:LatLng)
		{
			super(latlng);
		}

		public function snapTo(latlng:LatLng=null):void {
			if (overlay != null) {
				if (latlng == null) {
					setLatLng(overlay.snapTo(null, getLatLng()));
				} else {
					setLatLng(overlay.snapTo(null, latlng));
				}
			} else {
				if (latlng == null) {
					; // do nothing
				} else { // set the position, even though there's no overlay to snap to.
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
	}
}