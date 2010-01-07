package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	
	import flash.net.URLRequest;
	
	internal class ElevationRequest
	{
		public var urlRequest:URLRequest;
		public var latlng:LatLng;
		public var callback:Function;
		
		public function ElevationRequest(urlRequest:URLRequest, latlng:LatLng, callback:Function)
		{
			this.urlRequest = urlRequest;
			this.latlng = latlng;
			this.callback = callback;
		}
		
		public function toString():String {
			return '[' + urlRequest.url + ', ' + latlng.toString() + ']';
		} 
	}	
}