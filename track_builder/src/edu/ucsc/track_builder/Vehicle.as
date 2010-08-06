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
		public var model:String;
		
		/** Distance in meters from the start of the track segment. */
		public function get position():Number {
			return this.location.getPosition(this.latlng);	
		}
		
		/** Constructor */
		public function Vehicle(id:String, vel:Number, accel:Number, loc:TrackSegment, latlng:LatLng, elevation:Number, label:String, model:String, preview:Boolean):void 
		{
			this.id           = id;
			this.velocity     = vel;
			this.acceleration = accel;
			this.location     = loc;			
			this.latlng       = latlng;
			this.elevation    = elevation;
			this.label        = label;
			this.model        = model;
			
			if (!preview) {
				Undo.pushMicro(Globals.vehicles.vehicles, Globals.vehicles.vehicles.pop)
				Globals.vehicles.vehicles.push(this);
			}
		}

		public function toXML():XML {
			var xml:XML = <Vehicle id={id}
			                       location={location.id}
			                       position={position.toFixed(3)}
			                       label={label}
			                       velocity={velocity}
			                       acceleration={acceleration}
			                       model={model}>
							   <LatLng lat={latlng.lat().toFixed(7)}
			                           lng={latlng.lng().toFixed(7)}
			                       	   elevation={elevation}/>
			              </Vehicle>;												      
			return xml;
		}
		
		public static function fromXML(xml:XML):Vehicle {			
			var loc:TrackSegment = Globals.tracks.getTrackSegment(xml.@location);
			var latlng:LatLng = new LatLng(xml.LatLng.@lat, xml.LatLng.@lng);
			return new Vehicle(xml.@id,
			                   xml.@velocity,
			                   xml.@acceleration,
			                   loc,
			                   latlng,
			                   xml.LatLng.@elevation,
			                   xml.@label,
			                   xml.@model,
			                   false);        
		}
	}
}