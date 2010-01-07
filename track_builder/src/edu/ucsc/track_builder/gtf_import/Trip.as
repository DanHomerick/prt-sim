package edu.ucsc.track_builder.gtf_import
{
	import mx.utils.ObjectUtil;
	
	public class Trip
	{
		public var routeId:String;
		public var serviceId:String;
		public var id:String;
		public var headsign:String;
		public var short:String;
		public var dir:uint; // 0 or 1
		public var blockId:String;
		public var shapeId:String;
		
		/* Constructor */
		public function Trip(route_id:String,
							 service_id:String,
							 trip_id:String,
							 trip_headsign:String,
							 trip_short_name:String,
							 direction_id:String,
							 block_id:String,
							 shape_id:String) {
			this.routeId = route_id;
			this.serviceId = service_id;
			this.id = trip_id;
			this.headsign = trip_headsign;
			this.short = trip_short_name;
			this.dir = direction_id != "" ? uint(direction_id) : NaN;
			this.blockId = block_id;
			this.shapeId = shape_id;
		}
		
		public static function compareBy(prop:String):Function {
			return function(a:Trip, b:Trip):Number {return ObjectUtil.stringCompare(a[prop], b[prop]);};
		}
	}
}