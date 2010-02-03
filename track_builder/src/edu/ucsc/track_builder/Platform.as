package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	public class Platform
	{	
		public var trackSegId:String;
		public var index:uint;
		public var berths:Vector.<Berth>;	

		public function Platform(trackSegId:String, index:uint) {
			this.trackSegId = trackSegId;
			this.index = index;
			this.berths = new Vector.<Berth>()
		}
		
		public function toXML():XML {			
			var xml:XML = <Platform index={index}>
								<TrackSegmentID>{trackSegId}</TrackSegmentID>
						  </Platform>;
			
			for each (var berth:Berth in berths) {
				xml.appendChild(berth.toXML());
			}	
			return xml;
		}
		
		public static function fromXML(xml:XML):Platform {
			var platform:Platform = new Platform(xml.TrackSegmentID, xml.@index);
			for each (var berthXml:XML in xml.Berth) {
				platform.berths.push(Berth.fromXML(berthXml));
			}
			return platform;
		}
	}
}