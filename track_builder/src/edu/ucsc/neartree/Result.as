package edu.ucsc.neartree
{
	import com.google.maps.LatLng;
	
	/** A class for holding results data. */
	public class Result
	{
		public var latlng:LatLng;
		public var objs:Array;
		 
		public function Result()
		{
			objs = new Array();
		}
	}
}