package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import flash.events.Event;
	import flash.events.IOErrorEvent;
	import flash.events.SecurityErrorEvent;
	import flash.events.TimerEvent;
	import flash.net.URLLoader;
	import flash.utils.Timer;
	
	/** The ElevationService employs multiple copies of these ElevationLoader objects. Each ElevationLoader has a queue
	 * of ElevationRequests to perform. Each ElevationLoader blocks while waiting for the webservice to respond to the
	 * request.
	 */
	public class ElevationLoader extends URLLoader
	{			
		/* A FIFO queue. Entry to the queue is at the right end, Exit from queue at left end */
		public var queue:Vector.<ElevationRequest>;
		public var currRequest:ElevationRequest;
	
		public static var REQUEST_INTERVAL:int = 1000; // in milliseconds.
		public var timer:Timer = new Timer(REQUEST_INTERVAL, 1); // fire once per start.
				
		public function ElevationLoader()
		{
			queue = new Vector.<ElevationRequest>();
			timer.addEventListener(TimerEvent.TIMER_COMPLETE, fetch);
		}
		
		/** Parses the XML returned from a Google elevation query, and calls the ElevationRequests callback function
		 * with the latlngs and elevations Vectors as arguments. Elevations are truncated to the 1/10ths place (fixed
		 * point) for easy comparisons. */
		public function onComplete(evt:Event):void {
			var data:XML = XML(this.data);
			trace("Elevation xml:", data)
			
			if (data.status == "OVER_QUERY_LIMIT") {
				/* Take a short break and try again. */
				queue.push(this.currRequest);
				this.currRequest = null;
				REQUEST_INTERVAL *= 1.5
				timer.delay = REQUEST_INTERVAL;
				timer.reset();
				timer.start();
				return;			
			} else if (data.status != "OK") {
				throw new Error("Received bad elevation response:", data); // FIXME Do something about it, don't just bail
			}
			
			var requestLength:int = this.currRequest.latlngs.length;
			var elevations:Vector.<Number> = new Vector.<Number>(requestLength);			
			var result:XML;
			var latlng:LatLng;
			var ele:Number;
			for (var i:int=0; i < requestLength; ++i) { 			
				result = data.result[i];
				latlng = this.currRequest.latlngs[i];
												
				ele = result.elevation;
				ele = Utility.truncateTo(ele, 1);
				elevations[i] = ele;
				
				// cache each result
				ElevationService.addToCache(latlng, ele);
			}
			
			this.currRequest.callback(this.currRequest.latlngs, elevations);
			
			// Grab another work unit (ElevationRequest) to service.
			this.currRequest = null;
			timer.reset();
			timer.start(); // does another fetch.
		}
		
		public function onIOError(evt:IOErrorEvent):void {
			trace(evt.errorID, evt.text);			
			addRequest(currRequest); // put the failed request back in the queue
			currRequest = null;
			fetch();
		}
		
		public function onSecurityError(evt:SecurityErrorEvent):void {
			throw new Error(evt.text); // debug!
		}
		
		/** Triggers the ElevationLoader to process another job from the queue, if any are available. If no jobs are
		 * available, the function does nothing.
		 * @param evt Included only to allow fetch to be triggered by a Timer. Not used. */
		public function fetch(evt:TimerEvent=null):void {
			if (queue.length > 0) {
				currRequest = queue.shift();
				this.load(currRequest.urlRequest);
			}
		}
		
		public function addRequest(request:ElevationRequest):void {
			queue.push(request);
			if (queue.length == 1 && currRequest == null) { // was sitting idle
				fetch(); // restart the processing
			}
		}	
	}
}