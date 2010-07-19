package edu.ucsc.track_builder
{	
	/** Creates unique string-based Id's for various types. Increments by one each time. */
	public class IdGenerator
	{
		private static var trackSeg:uint = 0;
		private static var station:uint = 0;
		private static var vehicle:uint = 0;
		private static var waypointId:uint = 0;
		private static var switchId:uint = 0;
		private static var mergeId:uint = 0;
		
		public static function getTrackSegId(ext:String=""):String {
			var id:String = trackSeg.toString()+"_trackSegment"+ext;
			trackSeg += 1;
			Undo.pushMicro(IdGenerator, undoID, id);
			return id;
		}
		
		public static function getStationId():String {
			var id:String = station.toString()+"_station";
			station += 1;
			Undo.pushMicro(IdGenerator, undoID, id);
			return id;
		}

		public static function getVehicleId():String {
			var id:String = vehicle.toString()+"_vehicle";
			vehicle += 1;
			Undo.pushMicro(IdGenerator, undoID, id);
			return id;
		}
		

		/** WaypointId is special. The number of waypoints aren't tracked during the program, they
		 * are only described in the XML. */
		public static function getWaypointId():String {
			var id:String = waypointId.toString()+"_waypoint";
			waypointId += 1;
			return id;
		}

		/** SwitchId is special. The number of switches aren't tracked during the program, they
		 * are only described in the XML. */
		public static function getSwitchId():String {
			var id:String = switchId.toString()+"_switch";
			switchId += 1;
			return id;
		}
		
		/** MergeId is special. The number of merges aren't tracked during the program, they
		 * are only described in the XML. */
		public static function getMergeId():String {
			var id:String = mergeId.toString()+"_merge";
			mergeId += 1;
			return id;
		}
		
		public static function reinitialize():void {
			trackSeg = 0;
			station = 0;
			vehicle = 0;
		}
		
		/** id is the latest id used, as loaded from a file */ 
		public static function updateTrackSegId(id:String):void {
			var subStrings:Array = id.split("_");
			var num:int = int(subStrings[0]); // the latest number used
			trackSeg = num + 1;
		}

		/** id is the latest id used, as loaded from a file */ 
		public static function updateStationId(id:String):void {
			var subStrings:Array = id.split("_");
			var num:int = int(subStrings[0]); // the latest number used
			station = num + 1;
		}		
		
		/** id is the latest id used, as loaded from a file */ 
		public static function updateVehicleId(id:String):void {
			var subStrings:Array = id.split("_");
			var num:int = int(subStrings[0]); // the latest number used
			vehicle = num + 1;
		}		
		
		/** WaypointId should be reset each time the XML is generated. */
		public static function resetWaypointId(num:Number=0):void {
			waypointId = num;
		}
		
		/** SwitchId should be reset each time the XML is generated. */
		public static function resetSwitchId(num:Number=0):void {
			switchId = num;
		}

		/** MergeId should be reset each time the XML is generated. */
		public static function resetMergeId(num:Number=0):void {
			mergeId = num;
		}

		/** As though the id had never been used. */
		public static function undoID(id:String):void {
			var subStrings:Array = id.split("_");
			var num:int = int(subStrings[0]);
			var type:String = subStrings[1];
			switch (type) {
				case 'trackSegment':
					trackSeg = num;
				    break;
				case 'station':
					station = num;
					break;
				case 'vehicle':
					vehicle = num;
					break;
				default:
					throw new Error("Unknown id type: "+type);
			}
		}		 
	}
}