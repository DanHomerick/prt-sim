package edu.ucsc.track_builder.gtf_import
{
	import com.google.maps.LatLng;
	
	public class ShapeNode
	{	
		public var id:String;
		public var sequence:uint;
		public var latlng:LatLng;
		
		/* Constructor */
		public function ShapeNode(id:String, latlng:LatLng, sequence:uint)
		{
			this.id = id;
			this.latlng = latlng;
			this.sequence = sequence;
		}
		
		public static function cmp(a:ShapeNode, b:ShapeNode):Number {
			return a.sequence - b.sequence;
		} 
	}
}