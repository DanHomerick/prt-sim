
package edu.ucsc.track_builder.gtf_import
{
	public class Route
	{
		public static const typeStrings:Vector.<String> = Vector.<String>(["Streetlevel Rail",
		                                                                        "Underground Rail",
		                                                                        "Intercity Rail",
		                                                                        "Bus",
		                                                                        "Ferry",
		                                                                        "Cable Car",
		                                                                        "Gondola",
		                                                                        "Funicular"]);
		                                                                        
		public var id:String;
		public var agency:String;
		public var short:String;
		public var long:String;
		public var desc:String;
		public var type:uint;
		public var url:String;
		public var color:uint;
		
		/* Constructor */
		public function Route(route_id:String,
							  agency_id:String,
							  route_short_name:String,
							  route_long_name:String,
							  route_desc:String,
							  route_type:String,
							  route_url:String,
							  route_color:String) {
			id = route_id;
			agency = agency_id;
			short = route_short_name;
			long = route_long_name;
			desc = route_desc;
			type = uint(route_type);
			url = route_url;
			color = uint(route_color);	  	
		}
	
		public function get typeStr():String {
			return typeStrings[type];
		}
	
		public function toString():String {
			return short + " : " + long + " " + desc + " " + typeStrings[type];
		}
		
		/** Returns true if the route is of a transit type in which a vehicle at a stop does not
		 * prevent other vehicles from passing. Bus and ferry have offline stations, whereas rail systems
		 * typically do not.
		 */
		public function hasOfflineStations():Boolean {
			switch (type) {
				case 0: // fall through
				case 1:
				case 2:
				case 5:
				case 6:
				case 7: return false;
				
				case 3:
				case 4: return true;
				
				default: throw new Error("Unknown route type: " + type);
			}
		}
	}
}