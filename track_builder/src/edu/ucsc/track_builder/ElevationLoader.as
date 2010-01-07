package edu.ucsc.track_builder
{
	import flash.events.Event;
	import flash.events.IOErrorEvent;
	import flash.events.SecurityErrorEvent;
	import flash.net.URLLoader;
	
	public class ElevationLoader extends URLLoader
	{
		/* A FIFO queue. Entry to the queue is at the right end, Exit from queue at left end */
		public var queue:Vector.<ElevationRequest>;
		public var currRequest:ElevationRequest;
				
		public function ElevationLoader()
		{
			queue = new Vector.<ElevationRequest>();
		}
		
		public function onComplete(evt:Event):void {
			var dataXml:XML = XML(this.data);
			var elevation:Number = Number(dataXml);
			elevation = Utility.truncateTo(elevation, 1); // standardize accuracy to 1/10th of a meter.
			currRequest.callback(elevation, currRequest.latlng);
						
			/* Cache the results */
			ElevationService.elevation_cache[currRequest.latlng.toUrlValue(ElevationService.PRECISION)] = elevation;
			
			currRequest = null;
			fetch()
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
		
		public function fetch():void {
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