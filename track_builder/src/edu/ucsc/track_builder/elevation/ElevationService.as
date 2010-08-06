package edu.ucsc.track_builder.elevation
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import edu.ucsc.track_builder.Utility;
	
	import flash.errors.IOError;
	import flash.events.Event;
	import flash.events.IOErrorEvent;
	import flash.events.SecurityErrorEvent;
	import flash.events.TimerEvent;
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	import flash.net.URLLoader;
	import flash.net.URLRequest;
	import flash.utils.Timer;

	public class ElevationService
	{
		public static var enabled:Boolean = true;
		
		protected static var elevation_cache:Object = new Object(); // associative array keyed by latlng strings, with elevation values
		protected static const cacheName:String = 'elevation.dat'; // Filename
				
		 // To increase the likelihood of a cache hit, we treat two coordinates that differ by less than
		 // 0.000001 degree (<0.1 arcseconds) as having the same elevation. The best elevation data has a
		 // resolution of about 0.1 arcseconds, so we shouldn't lose much (if any) accuracy by doing this. 		
		protected static const PRECISION:int = 5;
	
		protected static var loader:URLLoader = new URLLoader();
		loader.addEventListener(Event.COMPLETE, onComplete);
		loader.addEventListener(IOErrorEvent.IO_ERROR, onIOError);
		loader.addEventListener(SecurityErrorEvent.SECURITY_ERROR, onSecurityError);
		protected static var loader_busy:Boolean = false;
		
		protected static var timer:Timer = new Timer(5000, 1); // Five second timer, one shot.
		timer.addEventListener(TimerEvent.TIMER_COMPLETE, onTimerComplete);
		
		/** Associative array that holds latlngs pending retrieval from an online elevation provider.
		 * Keys ars latlng strings (trimmed to PRECISION), values are LatLng objects.  */
		protected static var pendingLatLngs:Object = new Object();

		/** Returns the elevation for the requested latlng. If the elevation is not in the cache, then NaN is returned.
		 */
		public static function getElevation(latlng:LatLng):Number {
			var elevation:Number = elevation_cache[latlng.toUrlValue(PRECISION)]; // null gets converted to NaN on a cache misses
			return elevation;
		}
		
		/** Used to submit elevation requests for latlngs. */
		public static function requestElevations(latlngs:Vector.<LatLng>):void {
			for each (var latlng:LatLng in latlngs) {
				var latlngstr:String = latlng.toUrlValue(PRECISION);
				pendingLatLngs[latlngstr] = latlng;
			}
			
			/* If the loader is ready to be used. */
			if ( !(timer.running || loader_busy) ) {
				fetchElevations() // go get 'em, boy!
			}
		}
		
		/** Triggers the ElevationService to get fetch elevations from the online elevation provider.
		 * If there are no latlngs pending, then no action is taken.
		 */
		public static function fetchElevations():void {
			if (!enabled) {return;}
			
			var latlngs:Vector.<LatLng> = new Vector.<LatLng>();
			var elevation:Number;
			/* Weed out latlngs that are found in the cache, and fill a vector with the rest.
			 * Note that latlngs aren't removed from pending unless the results have been placed
			 * in the cache, so if an error occurs nothing special needs to be done to rerequest
			 * the latlngs that were in flight.
			 */
			for (var latlngstr:String in pendingLatLngs) {
				if (isNaN(elevation_cache[latlngstr])) {
					latlngs.push(pendingLatLngs[latlngstr]);
				} else {
					delete pendingLatLngs[latlngstr];
				}
			}
			
			if (latlngs.length > 0) {
				var maxLatLngs:int = Math.min(latlngs.length, 300); // capping at 300 latlngs is about right for a 1400 char url limit
				// The URL has a maximum character length, but the number of characters used per latlng is variable.
				// Try successively fewer latlngs until it falls into the viable range.
				while (true) {
					try {
						var urlRequest:URLRequest = makeGoogleUrlRequest(latlngs.slice(0, maxLatLngs));
						break;
					} catch (err:URLLengthError) {
						maxLatLngs *= 0.9
					}
				}
				
				loader_busy = true;
				loader.load(urlRequest);
			}
		}
			
		public static function saveCache():void {
			var dir:File = new File();
			dir = File.applicationStorageDirectory;
			var cacheFile:File = dir.resolvePath(cacheName);
			var cacheFS:FileStream = new FileStream();
			cacheFS.open(cacheFile, FileMode.WRITE);
			cacheFS.writeObject(elevation_cache);
			cacheFS.close();
			trace('Elevation cache saved to', cacheFile.nativePath, 'Size:', cacheFile.size);
		}
		
		public static function readCache():void {
			var dir:File = new File();
			dir = File.applicationStorageDirectory;
			var cacheFile:File = dir.resolvePath(cacheName);
			var cacheFS:FileStream = new FileStream();
			try {
				cacheFS.open(cacheFile, FileMode.READ);
				elevation_cache = cacheFS.readObject();
			} catch (err:IOError) {
				trace("IOError upon loading elevation cache:", cacheFile.toString()); // do nothing
			} finally {
				cacheFS.close();
			}
		}

		public static function addToCache(latlng:LatLng, elevation:Number):void {
			elevation = Utility.truncateTo(elevation, 1);
			elevation_cache[latlng.toUrlValue(PRECISION)] = elevation;
		}
		
		public static function clearCache():void {
			elevation_cache = new Object();
		}
		
		/** Returns the current size of the cache, in bytes. */ 
		public static function getCacheSizeOnDisk():Number {
			var dir:File = new File();
			dir = File.applicationStorageDirectory;
			var cacheFile:File = dir.resolvePath(cacheName);
			return cacheFile.size;
		}				
	
		/** Uses srtm3 data for geonames.org
		 * @deprecated
		 */ 
		protected static function makeGeonamesUrlRequest(latlng:LatLng):URLRequest {
			return new URLRequest("http://ws.geonames.org/srtm3?lat="+latlng.lat()+"&lng="+latlng.lng());
		}

		/** Uses USGS Elevation Query Web Service via a GET.
		 * See: http://gisdata.usgs.net/XMLWebServices/TNM_Elevation_Service.php
		 * @deprecated
		 */
		protected static function makeUsgsUrlRequest(latlng:LatLng):URLRequest {
			var baseUrl:String = "http://gisdata.usgs.gov/xmlwebservices2/elevation_service.asmx/getElevation?";
			var params:Array = new Array();
			params.push("X_Value=" + latlng.lng());
			params.push("Y_Value=" + latlng.lat());
			params.push("Elevation_Units=METERS");
			params.push("Source_Layer=-1"); // best available. 
			params.push("Elevation_Only=true");
			
			var url:String = baseUrl + params.join("&");
			return new URLRequest(url);
		}
		
		/** Uses Google's Elevation API. See: http://code.google.com/apis/maps/documentation/elevation/ */
		protected static function makeGoogleUrlRequest(latlngs:Vector.<LatLng>):URLRequest {
			/* Capable of using path= with a set number of samples. Since I need to calculate
			 * individual LatLngs for curved segments however, I'm just using the locations=
			 * query for all cases.
			 */
			var baseUrl:String = "http://maps.google.com/maps/api/elevation/xml?sensor=false&locations=enc:"
			var encodedPolyline:String = PolylineEncoder.fromPoints(latlngs);
			var url:String = baseUrl + encodedPolyline;
			// According to: http://code.google.com/apis/maps/documentation/webservices/index.html#BuildingURLs
			// URL lengths can't exceed 2048 characters. In practice, I was having IOStream errors when trying to
			// work near that limit. Scaling back to 1400 chars as a limit seems to work fine.
			if (url.length >= 1400) { 
				throw new URLLengthError("URL exceeded 1400 chars: " + url.length.toString());
			}				
			return new URLRequest(url);
		}
				
		protected static function onTimerComplete(evt:TimerEvent):void {
			fetchElevations();
		}
		
		protected static function onComplete(evt:Event):void {
			var data:XML = XML(loader.data);
			processGoogleResponse(data);
		
			loader_busy = false;
			timer.reset();
			timer.start();	
		}
		
		/** TODO: What can I do on an IO error? Pop an alert? */
		protected static function onIOError(evt:IOErrorEvent):void {
			trace(evt.text, evt.errorID, evt.type);
			throw new IOError(evt.text);
		}
		
		/** TODO: What can I do on a SecurityError? Pop an alert? */
		protected static function onSecurityError(evt:SecurityErrorEvent):void {
			trace(evt.text, evt.errorID, evt.type);
			throw new SecurityError(evt.text);
		}
						
		protected static function processGoogleResponse(data:XML):void {
			if (data.status == "OK") {
				var latlng:LatLng;
				var elevation:Number;
				for each (var result:XML in data.result) {
					latlng = new LatLng(result.location.lat,result.location.lng);
					elevation = Number(result.elevation);
					addToCache(latlng, elevation);
					/* Note that I go from string -> LatLng -> string. If I can be sure of the precision of the
					 * results, I can get a little speedup by skipping the conversions. Probably not significant, though.
					 */
				}
			}					
			else if (data.status == "OVER_QUERY_LIMIT") {
				timer.delay *= 2; // TODO: Switch to a different elevation provider?
			}
			else if (data.status == "INVALID_REQUEST") {throw new ElevationError("Status of: " + data.status);}
			else if (data.status == "REQUEST_DENIED") {throw new ElevationError("Status of: " + data.status);}
			else if (data.status == "UNKNOWN_ERROR") {throw new ElevationError("Status of: " + data.status);}			
			else {throw new ElevationError("Unexpected case.");}				
		}
	}
}


