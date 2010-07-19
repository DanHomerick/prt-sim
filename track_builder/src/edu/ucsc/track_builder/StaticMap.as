/** Uses Google's Static Map API to retrieve an image showing the area in which the track network is located.
 * 
 * Note: I never got this to work correctly. This class is currently dead code. */

package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	
	import mx.controls.Alert;
	import mx.graphics.ImageSnapshot;
	
	public class StaticMap
	{
		public static var apiKey:String = "ABQIAAAADtK5P0wYyZFDmjzoanxE5xSDTED1dMqBzm1PMihRGvUygQO1OxTN4Ia2w7lhdfpkzT5dOtexVdW8eQ"
		public static var baseUrl:String = "http://maps.google.com/staticmap?"
		public static var size:String = "640x640" // 640x640 is the max allowed by Google
		
//		/** Fetch a static image of the file and save it alongside the xml file */
//		public static function saveMap(file:File):void {
//			saveFile = file;
////			getImage();
//		
//			
//		}

		/** Messy. Needs refactoring. Assumes saveFile has been set. Saves the file and returns XML */
		public static function toDataXML():XML {
			var saveFile:File = saveImage(true, true); // returns the filename it used                      			
			if (saveFile == null) {
				return new XML("<Image/>");
			}
			
			var zoom:Number = Globals.map.getZoom();
			var bounds:LatLngBounds = Globals.map.getLatLngBounds();
			var center:LatLng = bounds.getCenter();
			var nw:LatLng = bounds.getNorthWest();
			var se:LatLng = bounds.getSouthEast();	

			var width:Number = Globals.map.width;
			var height:Number = Globals.map.height;
			
			var xml:XML = <Image filename={saveFile.name} width={width} height={height}>
			                      <Center lat={center.lat()} lng={center.lng()}/>
			                      <Northwest lat={nw.lat()} lng={nw.lng()}/>
			                      <Southeast lat={se.lat()} lng={se.lng()}/>
			              </Image>;
			return xml;
		}

		public static function saveImage(showTrack:Boolean=false, showStation:Boolean=false):File {
			if (!Globals.dataXMLFile) {
				trace("No save filename available.")
				return null;
			}
			
			var saveFile:File = new File(Globals.dataXMLFile.nativePath.replace(".xml", ".png"));
			
			Globals.straightTrackPane.visible = showTrack;
			Globals.curvedTrackPane.visible = showTrack;
			Globals.stationPane.visible = showStation;		
			Globals.vehiclePane.visible = false;
			Globals.markerPane.visible = false;			
			Globals.zoomControl.visible = false;
			
			var img:ImageSnapshot = ImageSnapshot.captureImage(Globals.map);
			var fs:FileStream = new FileStream();
			try {
				fs.open(saveFile, FileMode.WRITE);
				fs.writeBytes(img.data);			
			} catch (e:Error) {
				Alert.show("Unable to write image file: " + saveFile.nativePath);
			} finally {
				fs.close();
			}
			
			Globals.straightTrackPane.visible = true;
			Globals.curvedTrackPane.visible = true;
			Globals.stationPane.visible = true;
			Globals.vehiclePane.visible = true;
			Globals.markerPane.visible = true;
			Globals.zoomControl.visible = true;
			return saveFile;	
		}

//		private static function saveCurrent():void {
//			var img:ImageSnapshot = ImageSnapshot.captureImage(Globals.map);
//			var fs:FileStream = new FileStream();
//			try {				
//				fs.open(saveFile, FileMode.WRITE);
//				fs.writeBytes(img.data);
//				fs.close();
//			} catch (e:Error) {
//				trace("Unable to write file"); // TODO: Real error handling
//			}
//		}
		
	
//		/** Notes: AIR caches data automatically */ 
//		private static function getImage():void {
//			// Choose the location and zoom to show the entire track.
//			var bounds:LatLngBounds = Globals.tracks.getBounds();
//			var center:LatLng = bounds.getCenter();
//			var zoom:Number = Globals.map.getBoundsZoomLevel(bounds) - 1; // -1 to account for a station's coverage area not being considered
//			
//			// debug
//			center = new LatLng(37.00356438307206, -122.0588842010497888);
//			zoom = 16;
//			
//			var params:Array = new Array();
//			params.push("center=" + center.lat().toFixed(6) + ","+ center.lng().toFixed(6)); // center
//			params.push("zoom=" + zoom.toString()); // zoom
//			params.push("size=" + size.toString()); // size
//			params.push("maptype=" + getMapType());
//			params.push("key=" + apiKey);
//			params.push("sensor=false");
//
//			var url:String = baseUrl + params.join("&"); // &
//			var url2:String = "http://maps.google.com/staticmap?center=37.003564,-122.058884&zoom=16&size=640x640&maptype=roadmap&key=ABQIAAAADtK5P0wYyZFDmjzoanxE5xSDTED1dMqBzm1PMihRGvUygQO1OxTN4Ia2w7lhdfpkzT5dOtexVdW8eQ&sensor=false"
//			var url3:String = "http://static.arstechnica.com/mt-static/plugins/ArsTheme/style/themes/light/images/logo.png";
//			
//			trace("url : ", url);
//			trace("url2: ", url2);
//			trace("url == url2: ", url == url2);
//			var urlRequest:URLRequest = new URLRequest(url2);
//			urlRequest.useCache = false; // FOR DEBUGGING only!
//			var urlLoader:URLLoader = new URLLoader();
//			urlLoader.dataFormat=URLLoaderDataFormat.BINARY;
//			
//            urlLoader.addEventListener(Event.COMPLETE, onLoadComplete);
//            urlLoader.addEventListener(Event.OPEN, onOpen);
//            urlLoader.addEventListener(ProgressEvent.PROGRESS, onProgress);
//            urlLoader.addEventListener(SecurityErrorEvent.SECURITY_ERROR, onSecurityError);
//            urlLoader.addEventListener(HTTPStatusEvent.HTTP_STATUS, onHttpStatus);
//            urlLoader.addEventListener(IOErrorEvent.IO_ERROR, onIoError);					
//			
//			try {
//				urlLoader.load(urlRequest);
//			} catch (error:Error) {
//				trace("Unable to load url: ", url);
//			}
//		}
//		
//		private static function getMapType():String {
//			var mapType:IMapType = Globals.map.getCurrentMapType();
//			var name:String = mapType.getName();		
//			switch (name) {
//				case "Map": return "roadmap";
//				case "--": return "satellite";
//				case "--": return "terrain";
//				case "--": return "hybrid";
//				default: return "roadmap";
//			}
//		}
//		
//		private static function onLoadComplete(event:Event):void {
//			var loader:URLLoader = URLLoader(event.target);
//			
//			var fs:FileStream = new FileStream();
//			try {				
//				fs.open(saveFile, FileMode.WRITE);
//				fs.writeBytes(loader.data);
//			} catch (e:Error) {
//				trace("Unable to write file"); // TODO: Real error handling
//			}
//		}
//		
//		private static function onOpen(event:Event):void {
//			trace("StaticMap.onOpen: ", event);
//		}
//
//		private static function onProgress(event:ProgressEvent):void {
//			trace("StaticMap.onProgress: " + event.bytesLoaded + ", " + event.bytesTotal);			
//		}
//		
//        private static function onSecurityError(event:SecurityErrorEvent):void {
//            trace("onSecurityError: " + event);
//        }
//
//        private static function onHttpStatus(event:HTTPStatusEvent):void {
//            trace("httpStatus: " + event);
//        }
//
//        private static function onIoError(event:IOErrorEvent):void {
//            trace("ioErrorHandler: " + event);
//        }
	}
}