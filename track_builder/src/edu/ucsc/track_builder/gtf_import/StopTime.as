package edu.ucsc.track_builder.gtf_import
{
	import mx.utils.ObjectUtil;
	
	public class StopTime
	{
		public var tripId:String;
		public var arrival:Time;
		public var departure:Time;
		public var stopId:String;
		public var stopSeq:uint;
		public var headsign:String;
		public var pickup:uint;
		public var dropoff:uint;
		public var shapeDist:Number;
		
		protected static const today:Date = new Date();
		
		/* Constructor */
		public function StopTime(trip_id:String,
		                         arrival_time:String,
		                         departure_time:String,
		                         stop_id:String,
		                         stop_sequence:String,
		                         stop_headsign:String,
		                         pickup_type:String,
		                         drop_off_type:String,
		                         shape_dist_traveled:String) {
			tripId = trip_id;
			arrival = Time.fromString(arrival_time);
			departure = Time.fromString(departure_time);
			stopId = stop_id;
			stopSeq = uint(stop_sequence);
			headsign = stop_headsign;
			pickup = uint(pickup_type);
			dropoff = uint(drop_off_type);
			shapeDist = Number(shape_dist_traveled);		
		}
		
		public static function compareBy(prop:String):Function {
			switch (prop) {
				case 'trip':
				case 'stop':
				case 'headsign':
						return function(a:StopTime, b:StopTime):Number {return ObjectUtil.stringCompare(a[prop], b[prop]);};
				
				case 'arrival':
				case 'departure':
						return function(a:StopTime, b:StopTime):Number {return ObjectUtil.numericCompare(a[prop].toSeconds(), b[prop].toSeconds());};
				
				case 'stopSeq':
				case 'pickup':
				case 'dropoff':
				case 'shapeDist':
						return function(a:StopTime, b:StopTime):Number {return ObjectUtil.numericCompare(a[prop].toString(), b[prop].toString());};
			
				default:
						return null;
			}
		}		
	}
}