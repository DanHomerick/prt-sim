package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	
	import flash.net.URLRequest;
	
	/** A simple data holding class. ElevationRequests are placed in a holding queue until they can be serviced.
	 * This class holds all the relevant information for the request.
	 */
	internal class ElevationRequest
	{
		public var urlRequest:URLRequest;
		public var latlngs:Vector.<LatLng>;
		public var callback:Function;
		
		public function ElevationRequest(urlRequest:URLRequest, latlngs:Vector.<LatLng>, callback:Function)
		{
			this.urlRequest = urlRequest;
			this.latlngs = latlngs;
			this.callback = callback;
		}
		
		public function toString():String {
			return '[' + urlRequest.url + ', ' + latlngs.toString() + ']';
		} 
	}	
}