package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import flash.errors.IOError;
	import flash.events.Event;
	import flash.events.IOErrorEvent;
	import flash.events.SecurityErrorEvent;
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	import flash.net.URLRequest;

	public class ElevationService
	{
		public static const MAX_SIMULTANEOUS_REQUESTS:uint = 1;
		
		/* Shared between all instances */
		public static var elevation_cache:Object = new Object();
		public static const cacheName:String = 'elevation.dat';
				
		 // To increase the likelihood of a cache hit, we treat two coordinates that differ by less than
		 // 0.000001 degree (<0.1 arcseconds) as having the same elevation. The best elevation data has a
		 // resolution of about 0.1 arcseconds, so we shouldn't lose much (if any) accuracy by doing this. 		
		public static const PRECISION:int = 5;	
	
		public var loaders:Vector.<ElevationLoader>; 
		
		public function ElevationService()
		{
			loaders = new Vector.<ElevationLoader>(MAX_SIMULTANEOUS_REQUESTS);
			for (var i:int=0; i < loaders.length; ++i) {
				loaders[i] = new ElevationLoader();
				loaders[i].addEventListener(Event.COMPLETE, loaders[i].onComplete);
				loaders[i].addEventListener(IOErrorEvent.IO_ERROR, loaders[i].onIOError);
				loaders[i].addEventListener(SecurityErrorEvent.SECURITY_ERROR, loaders[i].onSecurityError);
			}
		}
		
		/** Requests elevations. All elevations are in meters.
		 * @param latlngs The points for which elevations are requested.
		 * @param callback A function has parameters of:
		 *                     latlngs:Vector.<LatLng>
		 *                     elevations:Vector.<Number>
		 *                 where latlngs and elevations are parallel arrays.
		 * The latlngs parameter is passed to the callback function, in addition to a new Vector containing the elevations.
		 * The callback function takes ownership of the elevations vector; it is not reused. 
		 */
		public function requestElevations(latlngs:Vector.<LatLng>, callback:Function):void {
			/* Check the cache first. Use an all-or-nothing approach: if any latlngs miss the cache, request all of them. */
			var latlngsLength:Number = latlngs.length;
			var elevation:Number;
			var elevations:Vector.<Number> = new Vector.<Number>(latlngsLength);
			var foundAll:Boolean = true;
			for (var i:int=0; i < latlngsLength; ++i) {
				elevation = NaN; // necessary? What value is used in a cache miss?
				elevation = elevation_cache[latlngs[i].toUrlValue(PRECISION)];
				if (!isNaN(elevation)) { // cache hit
					elevations[i] = elevation;				
				} else { // cache miss
					foundAll = false;
					break;
				}
			} 
			
			if (foundAll) { // every latlng was found in the elevation cache
				callback(latlngs, elevations); // call the callback function immediately.
				return;
			} else {				
				/* On a cache miss, put it in the work queue */
				var request:ElevationRequest = new ElevationRequest(makeGoogleUrlRequest(latlngs), latlngs, callback);
				/* Give the request to the loader with the shortest work queue */
				var minLength:Number = Number.POSITIVE_INFINITY;
				var minIndex:int;
				for (i=0; i < loaders.length; ++i) {
					if (loaders[i].queue.length < minLength) {
						minLength = loaders[i].queue.length;
						minIndex = i;
					}
				}
				loaders[minIndex].addRequest(request);
//				trace('Added', request.toString(), 'to loader', minIndex, 'with queue length', minLength);
			}
		}		
		
		/** Uses srtm3 data for geonames.org */ 
		public function makeGeonamesUrlRequest(latlng:LatLng):URLRequest {
			return new URLRequest("http://ws.geonames.org/srtm3?lat="+latlng.lat()+"&lng="+latlng.lng());
		}

		/** Uses USGS Elevation Query Web Service via a GET.
		 * See: http://gisdata.usgs.net/XMLWebServices/TNM_Elevation_Service.php */
		public function makeUsgsUrlRequest(latlng:LatLng):URLRequest {
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
		public function makeGoogleUrlRequest(latlngs:Vector.<LatLng>):URLRequest {
			/* Capable of using path= with a set number of samples. Since I need to calculate
			 * individual LatLngs for curved segments however, I'm just using the location=
			 * query for all cases.
			 */
			var baseUrl:String = "http://maps.google.com/maps/api/elevation/xml?sensor=false&locations="
			var params:Array = new Array();
			for each (var latlng:LatLng in latlngs) {
				params.push(latlng.toUrlValue(PRECISION));
			}
			var url:String = baseUrl + params.join("|");
			trace("request url:", url);
			return new URLRequest(url);
		}
		
		public static function addToCache(latlng:LatLng, elevation:Number):void {
			elevation_cache[latlng.toUrlValue(PRECISION)] = elevation;
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
				; // do nothing
			} finally {
				cacheFS.close();
			}
		}
		
		public static function clearCache():void {
			elevation_cache = new Object();
		}
		
		/** Returns the currenst size of the cache, in bytes. */ 
		public static function getCacheSizeOnDisk():Number {
			var dir:File = new File();
			dir = File.applicationStorageDirectory;
			var cacheFile:File = dir.resolvePath(cacheName);
			return cacheFile.size;
		}		
	}	
}


