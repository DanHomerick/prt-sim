package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	public class Platform
	{	
		public var trackSegId:String;
		public var berths:Vector.<String>;		
		public var berthLength:Number;
		public var unloading:Boolean;
		public var loading:Boolean;
		
		public static const EMPTY:String = 'empty';

		public function Platform(trackSegId:String, berthCount:uint, berthLength:Number, unloading:Boolean, loading:Boolean) {
			this.trackSegId = trackSegId;
			this.berths = new Vector.<String>(berthCount, true);
			this.berthLength = berthLength;
			this.unloading = unloading;
			this.loading = loading; 
		}
		
		public function toXML():XML {			
			var xml:XML = <Platform loading={loading} unloading={unloading} berth_length={berthLength}>
								<TrackSegment>{trackSegId}</TrackSegment>
						  </Platform>;
			
			for each (var berth:String in berths) {
				var vehicleId:String = berth == null ? EMPTY : berth;
				xml.appendChild(<Berth>{vehicleId}</Berth>);
			}
			return xml;
		}
		
		public static function fromXML(xml:XML):Platform {
			var platform:Platform = new Platform(xml.TrackSegment, 0, 0, xml.@unloading, xml.@loading);
			var berths:Vector.<String> = new Vector.<String>();
			for each (var berth:String in xml.Berth) {
				var vehicleId:String = berth == EMPTY ? null : berth;
				berths.push(vehicleId);
			}
			platform.berths = berths;
			return platform;
		}
	}
}