package edu.ucsc.track_builder.gtf_import
{
	import __AS3__.vec.Vector;
	
	public class StopTimeCollection
	{
		/** key is a trip id (String), value is a Vector.<StopTime> */
		protected var dict:Object;
		
		public function StopTimeCollection()
		{
			dict = new Object();
		}

		public function add(stopTime:StopTime):void  {			 
			var tripVec:Vector.<StopTime> = dict[stopTime.tripId];
			
			/* First StopTime for a particular trip */
			if (tripVec == null) {
				tripVec = new Vector.<StopTime>();
				tripVec.push(stopTime);
				dict[stopTime.tripId] = tripVec;
			}
			
			/* Array already created */
			else {
				tripVec.push(stopTime);
			}
		}
		
		public function sortAll():void {
			for each (var vec:Vector.<StopTime> in dict) {
				vec.sort(StopTime.compareBy("stopSeq")); // they are probably already in sorted order, but not guaranteed.
			}
		}
		
		/**
		 * @param trip A Trip instance.
		 * @return A reference to a Vector containing StopTimes associated with <code>trip</code>.
		 *          Modifying the Vector will modify the contents of the StopTimeCollection.
		 */ 
		public function getByTrip(trip:Trip):Vector.<StopTime> {
			return dict[trip.id];
		}

		/**
		 * @param tripId A valid tripId.
		 * @return A reference to a Vector containing StopTimes associated with <code>trip</code>.
		 *          Modifying the Vector will modify the contents of the StopTimeCollection.
		 */		
		public function getByTripId(tripId:String):Vector.<StopTime> {
			return dict[tripId];
		}			
		
		/** Get all Stop instances for the supplied Trips. Uses StopTime data to associate
		 * trips with stops. 
		 * @param trips Contains the Trip instances of interest.
		 * @param allStops An Object being used as a dictionary, which holds all Stop instances. Keys are stop_ids, values are Stop instances.
		 * @param stops An Object in the same format as the return value. If not null, stops are added to this object, rather than creating a new one.
		 * @return An Object being used as a dictionary. The keys are unique stop_ids and the values are Stop instances.
		 */  
		public function getStopsFromTrips(trips:Vector.<Trip>, allStops:Object, stops:Object=null):Object {
			if (stops == null) {
				var stops:Object = new Object();
			}
			for each (var trip:Trip in trips) {
				var stopTimes:Vector.<StopTime> = dict[trip.id];
				for each (var stopTime:StopTime in stopTimes) {
					var stop_id:String = stopTime.stopId;
					stops[stop_id] = allStops[stop_id];				
				}
			}
			
			return stops;
		}
		
	}
}