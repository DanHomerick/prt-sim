package edu.ucsc.track_builder
{
	public class Berth
	{
		public var index:int;
		public var startPos:Number;
		public var endPos:Number;
		public var unloading:Boolean;
		public var loading:Boolean;
		
		public function Berth(index:int, startPos:Number, endPos:Number, unloading:Boolean, loading:Boolean)
		{
			this.index = index;
			this.startPos = startPos;
			this.endPos = endPos;
			this.unloading = unloading;
			this.loading = loading;
		}

		public function toXML():XML {			
			var xml:XML = <Berth index={index} unloading={unloading} loading={loading}>
								<StartPosition>{startPos}</StartPosition>
								<EndPosition>{endPos}</EndPosition>
						  </Berth>;
			return xml;
		}
		
		public static function fromXML(xml:XML):Berth {
			var startPos:Number = xml.StartPosition
			var endPos:Number = xml.EndPosition
			var berth:Berth = new Berth(xml.@index, startPos, endPos, xml.@unloading, xml.@loading)
			return berth;
		}
	}
}