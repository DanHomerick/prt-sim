package edu.ucsc.track_builder
{
	import com.google.maps.services.ClientGeocoder;
	import com.google.maps.services.GeocodingEvent;
	
	public class GoTo
	{
		/** Center the map on an address. Opens a window to get the address from the user,
		 * then uses Google's geocoding service to get a latlng coord. Recenters the map.
		 */
		public static function address(address:String):void {			

	        //PopUpManager.removePopUp(tw);
			var geocoder:ClientGeocoder = new ClientGeocoder();
			geocoder.geocode(address);
			geocoder.addEventListener(GeocodingEvent.GEOCODING_SUCCESS,
			                          function(event:GeocodingEvent):void {
			                          	var placemarks:Array = event.response.placemarks;
			                          	if (placemarks.length > 0) {
			                          		Globals.map.setCenter(event.response.placemarks[0].point);
			                          	} else {
			                          		trace(event.status);
			                          	}
			                          } );
			geocoder.addEventListener(GeocodingEvent.GEOCODING_FAILURE,
			                          function(event:GeocodingEvent):void {
			                            trace("Geocoding failed." + event.status);
			                          });
		}
	}
}

