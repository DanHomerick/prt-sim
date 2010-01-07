package edu.ucsc.graph
{
	import com.google.maps.LatLng;
	
	public class Node
	{
		public var latlng:LatLng;
		public var key:String;
		 
		public function Node(latlng:LatLng, key:String)
		{
			this.latlng = latlng;
			this.key = key;
		}
		
		public function toString():String {
			return key;
		}
		
		public function equals(other:Node):Boolean {
			return this.key == other.key;
		}
		
		public static function fromLatLng(latlng:LatLng):Node {
			var key:String = latlng.toUrlValue(Graph.PRECISION);
			/* Create a new LatLng using the correct precision, so that latlng.equals() will work. */
			var precisionLatLng:LatLng = LatLng.fromUrlValue(key);
			return new Node(precisionLatLng, key);
		}
		
		public static function fromKey(key:String):Node {
			var latlng:LatLng = LatLng.fromUrlValue(key);
			return new Node(latlng, key);
		}
	}
}