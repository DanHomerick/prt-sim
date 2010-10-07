package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	
	public class Vehicle
	{		
		public var id:String;
		public var velocity:Number;
		public var acceleration:Number;
		public var location:TrackSegment;		
		public var latlng:LatLng;
		public var elevation:Number;
		public var label:String;
		public var modelName:String;
		
		public var overlay:VehicleOverlay;
		
		/** Distance in meters from the start of the track segment. */
		public function get position():Number {
			return this.location.getPosition(this.latlng);	
		}
		
		/** Constructor */
		public function Vehicle(id:String, vel:Number, accel:Number, loc:TrackSegment, latlng:LatLng, elevation:Number, label:String, modelName:String, preview:Boolean):void 
		{
			this.id           = id;
			this.velocity     = vel;
			this.acceleration = accel;
			this.location     = loc;			
			this.latlng       = latlng;
			this.elevation    = elevation;
			this.label        = label;
			this.modelName    = modelName;
			
			if (!preview) {
				// add to global store
				Undo.pushMicro(Globals.vehicles.vehicles, Globals.vehicles.vehicles.pop)
				Globals.vehicles.vehicles.push(this);
			}			
		}

		/** Reverses all of the constructor's side effects (with Undo support) */
		public function remove():void {
			// remove from global store
			function removeMe(item:Vehicle, index:int, vector:Vector.<Vehicle>):Boolean {return item !== this};
			Undo.assign(Globals.vehicles, "vehicles", Globals.vehicles.vehicles);
			Globals.vehicles.vehicles = Globals.vehicles.vehicles.filter(removeMe, this);
		}

		public function toXML():XML {
			var xml:XML = <Vehicle id={id}
			                       location={location.id}
			                       position={position.toFixed(3)}
			                       label={label}
			                       velocity={velocity}
			                       acceleration={acceleration}
			                       model_name={modelName}>
							   <LatLng lat={latlng.lat().toFixed(7)}
			                           lng={latlng.lng().toFixed(7)}
			                       	   elevation={elevation}/>
			              </Vehicle>;												      
			return xml;
		}
		
		public static function fromXML(xml:XML):Vehicle {			
			var loc:TrackSegment = Globals.tracks.getTrackSegment(xml.@location);
			var latlng:LatLng = new LatLng(xml.LatLng.@lat, xml.LatLng.@lng);
			if (!('@model_name' in xml)) {
				throw new Error('Vehicle "' + xml.@id + '" from the XML file has no "model_name" attribute.');
			}
			return new Vehicle(xml.@id,
			                   xml.@velocity,
			                   xml.@acceleration,
			                   loc,
			                   latlng,
			                   xml.LatLng.@elevation,
			                   xml.@label,
			                   xml.@model_name,
			                   false);
		}
		
		public function toString():String {
			if (this.label) {
				return this.label;
			} else {
				return this.id.toString();
			}			
		}
	}
}