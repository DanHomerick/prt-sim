package edu.ucsc.track_builder.gtf_import
{
	import __AS3__.vec.Vector;
	
	public class RouteBundle
	{
		public var route:Route;
		public var service:Service;		
		public var tripBundles:Vector.<TripBundle>;	
		
		public function RouteBundle(route:Route, service:Service)
		{
			this.route = route;
			this.service = service;
			tripBundles = new Vector.<TripBundle>();
		}
		
		/** Removes trips that have completed before the simulation even begins. Does not maintain the order of the tripBundles Vector.
		 * @param time All trips whose last departure is before <code>time</code> are removed.
		 */
		public function removeEarlyTrips(time:Time):void {
			for (var i:int=tripBundles.length-1; i > -1; --i) {
				var tBundle:TripBundle = tripBundles[i];
				if (tBundle.endStopTime.departure.toSeconds() < time.toSeconds()) {
					if (i == tripBundles.length-1) {
						tripBundles.pop();
					} else {
						tripBundles[i] = tripBundles.pop();
					}
				}
			}
		}		
	}
}