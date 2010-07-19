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
		/* Shared between all instances */
		public static var elevation_cache:Object = new Object();
		public static const cacheName:String = 'elevation.dat';
				
		 // To increase the likelihood of a cache hit, we treat two coordinates that differ by less than
		 // 0.000001 degree (<0.1 arcseconds) as having the same elevation. The best elevation data has a
		 // resolution of about 0.1 arcseconds, so we shouldn't lose much (if any) accuracy by doing this. 		
		public static const PRECISION:int = 5;	
	
		public var loaders:Vector.<ElevationLoader>; 
		
		public function ElevationService(maxParallelRequests:uint)
		{
			loaders = new Vector.<ElevationLoader>(maxParallelRequests);
			for (var i:int=0; i < loaders.length; ++i) {
				loaders[i] = new ElevationLoader();
				loaders[i].addEventListener(Event.COMPLETE, loaders[i].onComplete);
				loaders[i].addEventListener(IOErrorEvent.IO_ERROR, loaders[i].onIOError);
				loaders[i].addEventListener(SecurityErrorEvent.SECURITY_ERROR, loaders[i].onSecurityError);
			}
		}
		
		/** Requests an elevation. All elevations are in meters.
		 * @param latlng The point for which an elevation is requested.
		 * @param callback A function has parameters of:
		 *                     elevation:Number
		 *                     latlng:LatLng
		 */
		public function requestElevation(latlng:LatLng, callback:Function):void {
			/* Check the cache first */
			var elevation:Number = NaN;
			elevation = elevation_cache[latlng.toUrlValue(PRECISION)];
			if (!isNaN(elevation)) { // cache hit
				callback(elevation, latlng);
			} 
			
			/* On a cache miss, put it in the work queue */
			var request:ElevationRequest = new ElevationRequest(makeUsgsUrlRequest(latlng), latlng, callback);
			/* Give the reques to the loader with the shorter work queue */
			var minLength:Number = Number.POSITIVE_INFINITY;
			var minIndex:int;
			for (var i:int=0; i < loaders.length; ++i) {
				if (loaders[i].queue.length < minLength) {
					minLength = loaders[i].queue.length;
					minIndex = i;
				}
			}
			loaders[minIndex].addRequest(request);
//			trace('Added', request.toString(), 'to loader', minIndex, 'with queue length', minLength);
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
		
		public function makeGoogleElevationUrlRequest(latlngs:Vector.<LatLng>):URLRequest {
			/* Capable of using path= with a set number of samples. Since I need to calculate
			 * individual LatLngs for curved segments however, I'm just using the location=
			 * query for all cases.
			 */
			var baseUrl:String = "http://maps.google.com/maps/api/elevation/xml?"
			var params:Array = new Array();
			for each (var latlng:LatLng in latlngs) {
				params.push(latlng.toString());
			}
			var url:String = baseUrl + params.join("|");
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


