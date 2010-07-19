package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	/** A simple data-passing class. Used to aid construction of larger track structures, such as interchanges. */ 
	public class TrackBundle
	{	
		/** All overlays. */
		public var overlays:Vector.<TrackOverlay>;

		/** The LatLng where this structure should be connected to the new TrackOverlay */
		public var connectNewLatLng:LatLng;		
		/** The overlays that should be connected to the new TrackOverlay. */		
		public var connectNewOverlays:Vector.<TrackOverlay>;
		
		/** An alternate connection point for structures that have more than one. */
		public var connectAltLatLng:LatLng;
		/** The overlays that should be connected at the alternate connection point. */
		public var connectAltOverlays:Vector.<TrackOverlay>;

		/** The latlngs at which TrackOverlays need to be merged with the <code>existing</code> TrackOverlay.
		 * Is a parallel array with mergeExistingOverlays. May contain repeated LatLngs.
		 */		 
		public var connectExistingLatLngs:Vector.<LatLng>;
		/** The TrackOverlays that need to be merged with the <code>existing</code> TrackOverlay.
		 * Is a parallel array with mergeExistingLatLngs.
		 */
		public var connectExistingOverlays:Vector.<TrackOverlay>;

		/* Constructor */
		public function TrackBundle() {
			this.overlays = new Vector.<TrackOverlay>();
			this.connectNewOverlays = new Vector.<TrackOverlay>();
			this.connectAltOverlays = new Vector.<TrackOverlay>();
			this.connectExistingLatLngs = new Vector.<LatLng>();
			this.connectExistingOverlays = new Vector.<TrackOverlay>();
		}
		
		public function addOverlay(overlay:TrackOverlay):void {
			this.overlays.push(overlay);
		}

		public function setConnectNewLatLng(latlng:LatLng):void {
			this.connectNewLatLng = latlng;
		}
		
		public function markAsConnectNew(overlay:TrackOverlay):void {
			this.connectNewOverlays.push(overlay);
		}

		public function setConnectAltLatLng(latlng:LatLng):void {
			this.connectAltLatLng = latlng;
		}

		public function markAsConnectAlt(overlay:TrackOverlay):void {
			this.connectAltOverlays.push(overlay);
		}		

		public function setConnectExisting(latlng:LatLng, overlay:TrackOverlay):void {
			this.connectExistingLatLngs.push(latlng);
			this.connectExistingOverlays.push(overlay);
		}
		
		/** Sorts the parallel <code>connectExistingLatLngs</code> and <code>connectExistingOverlays</code> vectors
		 * by distance from latlng.
		 */
		public function sortConnectExistingArrays(latlng:LatLng):void {
			// This is a little crufty, but I think it's still clearer and easier than writing my own sort.
			var tmp:Vector.<Object> = new Vector.<Object>();
			for (var i:int = 0; i < this.connectExistingLatLngs.length; ++i) {
				// Create a bunch of new objects containing the latlng and overlay.
				tmp.push({latlng:this.connectExistingLatLngs[i], overlay:this.connectExistingOverlays[i]});
			}
			
			var sortByDist:Function = function(x:Object, y:Object):Number {
				return x.latlng.distanceFrom(latlng) - y.latlng.distanceFrom(latlng);
			}
						
			tmp.sort(sortByDist);
			
			this.connectExistingLatLngs.splice(0, this.connectExistingLatLngs.length); // remove all contents from vector
			this.connectExistingOverlays.splice(0, this.connectExistingOverlays.length); 
			
			for each (var obj:Object in tmp) {
				this.connectExistingLatLngs.push(obj.latlng);
				this.connectExistingOverlays.push(obj.overlay);
			}
			
		}
	}
}