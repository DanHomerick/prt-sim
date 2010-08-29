/** Using a Model-View-Controller design pattern. The Vehicles class plays the role of a Controller. */

package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.MapMouseEvent;
	
	[Bindable]
	public class Vehicles
	{	
		public var model:VehicleModel;
		public var velocity:Number;
		public var acceleration:Number;
		public var label:String = "";
		public var reverseDir:Boolean;			
		public var position:Number = -1; // denotes invalid
		public var location:TrackSegment;
		
		public var targetVehicle:Vehicle; // a dummy vehicle, holding just enough info for targetMarker to work.
		public var targetMarker:VehicleOverlay;
		public var vehicles:Vector.<Vehicle>;
		public var overlays:Vector.<VehicleOverlay>;
		
		/* Constructor */
		public function Vehicles() {
			vehicles = new Vector.<Vehicle>(); 
			overlays = new Vector.<VehicleOverlay>();
		}
		
		public function onMapClick(event:MapMouseEvent):void {
			var marker:SnappingMarker = Globals.getActiveMarker();
			try {
				Undo.startCommand(Undo.USER);
				addVehicle(marker.getLatLng(), marker.getSnapOverlay(), false);
				Undo.endCommand();
			} catch (err:VehicleError) {
				Undo.endCommand();
				Undo.undo(Undo.USER);
//				Alert.show(err.message); // Really don't need to take over the screen whenever a user clicks off of the track.
			}
		}	
	
		public function onMapMouseMove(event:MapMouseEvent):void {
//			trace("Vehicles.onMapMouseMove");
			makePreview();
		}

		public function makePreview():void {
			Undo.undo(Undo.PREVIEW); // get rid of the old 'live preview', if one exists
			var marker:SnappingMarker = Globals.getActiveMarker();
			
			if (marker.getSnapOverlay() == null) { // active marker isn't attached to an overlay.
				return; // do nothing
			}
							
			try {
				Undo.startCommand(Undo.PREVIEW);
				addVehicle(marker.getLatLng(), marker.getSnapOverlay(), true);
				Undo.endCommand();
			} catch (err:VehicleError) {
				// undo any side effects from before the error
				Undo.endCommand();
				Undo.undo(Undo.PREVIEW);
			}				

		}

		public function addVehicle(latlng:LatLng, tOverlay:TrackOverlay, preview:Boolean):void {
			if (tOverlay == null) {
				throw new VehicleError("Cursor must be over a track segment to place a vehicle");
			}
			location = reverseDir ? tOverlay.segments[tOverlay.segments.length-1] : tOverlay.segments[0];
			position = location.getPosition(latlng);  // update to provide user feedback
			var id:String = preview ? 'previewVehicle' : IdGenerator.getVehicleId();
			var vehicle:Vehicle = new Vehicle(id,
											  velocity,
											  acceleration,
											  location,
											  latlng,
											  location.getElevation(position),
											  label,
											  model.modelName,
											  preview);
			var vOverlay:VehicleOverlay = new VehicleOverlay(vehicle, tOverlay, preview);		
		}
		
		public function getVehicleOverlaysFromTrackOverlay(t_overlay:TrackOverlay):Vector.<VehicleOverlay> {
			// Linear search over all VehicleOverlays
			var results:Vector.<VehicleOverlay> = new Vector.<VehicleOverlay>(); 
			for each (var v_overlay:VehicleOverlay in this.overlays) {
				if (v_overlay.getTrackOverlay() == t_overlay) {
					 results.push(v_overlay);
				}
			}
			return results;
		}
		
		public function toDataXML():XML {
			var xml:XML = <Vehicles/>
			for each (var v:Vehicle in vehicles) {
				xml.appendChild(v.toXML());
			}
			return xml;
		}

		public function fromDataXML(xml:XMLList):void {
			var vehicle:Vehicle;
			var vOverlay:VehicleOverlay;
			var trackOverlay:TrackOverlay;
			for each (var vXml:XML in xml) {
				vehicle = Vehicle.fromXML(vXml);
				trackOverlay = Globals.tracks.getTrackOverlay(vehicle.location.id);
				trace(vehicle.latlng.lat(), vehicle.latlng.lng());
				vOverlay = new VehicleOverlay(vehicle, trackOverlay);
			}
		}
		
		/** Generate xml from current preferences */
		public function toPrefsXML():XML {
			var xml:XML = <Vehicles
							model_name={model.modelName}
							velocity={velocity}
			                acceleration={acceleration}
			                reverse_dir={reverseDir}
			              />
			return xml;
		}
		
		/** Generate xml from hard-coded default preferences */
		public function toDefaultPrefsXML():XML {
			var xml:XML = <Vehicles
							model_name="PRT_DEFAULT"
							velocity="0"
			                acceleration="0"
			                reverse_dir="false"
		                  />
			return xml;
		}		
		
		public function fromPrefsXML(xml:XML):void {
			model = Globals.vehicleModels.getModelByName(xml.@model_name);
			velocity = xml.@velocity;
			acceleration = xml.@acceleration;
			reverseDir = xml.@reverse_dir == 'false' || xml.@reverse_dir == '0' ? false : true;
		}
		
		public function getNumberOfVehicles():Number {
			return vehicles.length;			
		}
		
		public function reinitialize():void {
			Globals.vehiclePane.clear();
			targetMarker = null;			
			vehicles = new Vector.<Vehicle>();
			overlays = new Vector.<VehicleOverlay>();
		}
		
		
		/** Removes the vehicle from the global store */
		public function removeVehicle(v:Vehicle):void {
			function removeV (item:Vehicle, index:int, vector:Vector.<Vehicle>):Boolean {return item != v;};
			Undo.assign(this, 'vehicles', vehicles); // note that filter creates a new Vector, which is why I'm able to just store a ref to the old vector
			vehicles = vehicles.filter(removeV); 
		}
		
		/** Removes the associated vehicle and the overlay from the global store */ 
		public function removeVehicleOverlay(vo:VehicleOverlay):void {
			if (vo.vehicle) {
				removeVehicle(vo.vehicle);
			}
			Undo.pushMicro(Globals.vehiclePane, Globals.vehiclePane.addOverlay, vo);
			Globals.vehiclePane.removeOverlay(vo);
			function removeVO (item:VehicleOverlay, index:int, vector:Vector.<VehicleOverlay>):Boolean {return item != vo;};
			Undo.assign(this, 'overlays', overlays);
			overlays = overlays.filter(removeVO);
		}
		
		public function validate():Array {
			return new Array();
//			/* Check that vehicles don't overlap. Doing it brute force, so there's O(n^2) comparisons.
//			Optimization note: segregate vehicles by their location, then compare within locations. Would need to
//			                   handle vehicles that staddle location boundary (or boundaries, for long vehicles) */
//			for (var v:Vehicle in this.vehicles) {
//				for (var w:Vehicle in this.vehicles) {
//					if v.location == w.location:
//						if v.model.length
//			}	
		}
	}
}