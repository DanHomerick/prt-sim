package edu.ucsc.track_builder
{
	public class Berth
	{
		public var index:int;
		public var startPos:Number;
		public var endPos:Number;
		public var unloading:Boolean;
		public var loading:Boolean;
		public var storageEntrance:Boolean;
		public var storageExit:Boolean;
		
		public function Berth(index:int, startPos:Number, endPos:Number, unloading:Boolean, loading:Boolean,
		                      storageEntrance:Boolean, storageExit:Boolean)
		{
			this.index = index;
			this.startPos = startPos;
			this.endPos = endPos;
			this.unloading = unloading;
			this.loading = loading;
			this.storageEntrance = storageEntrance;
			this.storageExit = storageExit;
		}

		public function toXML():XML {			
			var xml:XML = <Berth
			                    index={index}
			                    unloading={unloading}
			                    loading={loading}
			                    storage_entrance={storageEntrance}
			                    storage_exit={storageExit} >
							 <StartPosition>{startPos}</StartPosition>
							 <EndPosition>{endPos}</EndPosition>
						  </Berth>;
			return xml;
		}
		
		public static function fromXML(xml:XML):Berth {
			var startPos:Number = xml.StartPosition
			var endPos:Number = xml.EndPosition
			var berth:Berth = new Berth(xml.@index,
			                            startPos,
			                            endPos,
			                            xml.@unloading == "true" || xml.@unloading == "1",			                            
			                            xml.@loading == "true" || xml.@loading == "1",
			                            xml.@storage_entrance == "true" || xml.@storage_entrance == "1",
			                            xml.@storage_exit == "true" || xml.@storage_exit == "1")
			return berth;
		}
	}
}