package edu.ucsc.track_builder.gtf_import
{
	import __AS3__.vec.Vector;
	
	
	public class TripBundle
	{
		public var trip:Trip;		
		public  var stopTimes:Vector.<StopTime>;
		
		protected var _beginStopTime:StopTime;
		protected var _endStopTime:StopTime;
		
		public function TripBundle(trip:Trip, stopTimes:Vector.<StopTime>)
		{
			this.trip = trip;
			
			this.stopTimes = stopTimes;
			updateEndpoints();
			stopTimes.sort(StopTime.compareBy('stopSeq'));
		}		
		
		/** Find the endpoints by earliest arrival and latest depature. Assumes that stopTimes is length >= 2 */
		protected function updateEndpoints():void {
			_beginStopTime = stopTimes[0];			
			_endStopTime = stopTimes[0];
			
			var startTimeSec:Number = _beginStopTime.arrival.toSeconds();
			var endTimeSec:Number = _endStopTime.departure.toSeconds();
			
			for (var i:uint=1; i < stopTimes.length; ++i) {
				if (stopTimes[i].arrival.toSeconds() < startTimeSec) {
					_beginStopTime = stopTimes[i];
					startTimeSec = _beginStopTime.arrival.toSeconds();
				}
				
				if (stopTimes[i].departure.toSeconds() > endTimeSec) {
					_endStopTime = stopTimes[i];
					endTimeSec = _endStopTime.departure.toSeconds();
				}
			}			
		}
		
		public function get beginStopTime():StopTime {
			return _beginStopTime;
		}
		
		public function get endStopTime():StopTime {
			return _endStopTime;
		}
		
		/** Returns all stopIds, in stop sequence order. Stop Ids may not be unique within the results. */ 
		public function getStopIds():Vector.<String> {
			var result:Vector.<String> = new Vector.<String>();
			for each (var stopTime:StopTime in stopTimes) {
				result.push(stopTime.stopId);
			}
			return result;
		}
	}
}