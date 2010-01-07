package edu.ucsc.track_builder.gtf_import
{
	import com.google.maps.LatLng;
	
	public class Stop
	{
		// location types
		static public const STOP:uint = 0;
		static public const STATION:uint = 1;
		
		public var id:String;
		public var name:String;
		public var latlng:LatLng;
		public var desc:String;
		public var type:uint;
		public var parent:String;
		
		/* Constructor */
		public function Stop(id:String, name:String, latlng:LatLng, desc:String="", type:uint=STOP, parent:String="")
		{
			this.id = id;
			this.name = name;
			this.latlng = latlng;
			this.desc = desc;
			this.type = type;
			this.parent = parent;
		} 
	}
}