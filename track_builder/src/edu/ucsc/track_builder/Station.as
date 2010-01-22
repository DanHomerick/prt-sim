package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import mx.controls.Alert;

	public final class Station
	{	
		public var id:String;		
		public var label:String;
		public var allSegments:Vector.<TrackSegment>;
		public var platforms:Vector.<Platform>;
		public var coverageRadius:uint;
		public var peakHour:uint;
		public var daily:uint;
		
		public static const ENTRANCE:int = 0;
		public static const EXIT:int = 0;
		

		/** Station constructor.
		 * @param id
		 * @param label
   		 * @param allSegments
  		 * @param platforms
		 * @param coverage_radius
		 * @param peakHour
		 * @param daily
		 */
		public function Station(id:String,
								label:String,
								allSegments:Vector.<TrackSegment>,
								platforms:Vector.<Platform>,
								coverage_radius:uint, // the expected walk-in distance for the station. i.e. it's coverage
								peakHour:uint,
								daily:uint) 
		{
			this.id = id;
			this.label = label;
			this.allSegments = allSegments;
			this.platforms = platforms;
			this.coverageRadius = coverage_radius;
			this.peakHour = peakHour;
			this.daily = daily;			
			
			/* Side effect */
			Undo.pushMicro(Globals.stations.stations, Globals.stations.stations.pop);
			Globals.stations.stations.push(this);
		}
		
		/** Creates a new vehicle in an available berth closest to the departure end of the station.
		 * @param side If EXIT (the default) place in an available berth closest to the departure.
		 *                  If ENTRANCE, place the vehicle in an available berth closest to the arrival.
		 * @return The vehicle that was created. If no berths are available, returns null. */  
		public function makeVehicle(label:String, side:int=EXIT):Vehicle {
			var platform:Platform;
			var berthIndex:int;
			var foundAvail:Boolean = false;
			
			if (side == EXIT) { // iterate in reverse
				for (var i:int=platforms.length-1; i > -1; --i) {
					platform = platforms[i];
					var berths:Vector.<String> = platform.berths;
					for (berthIndex = berths.length-1; berthIndex > -1; --berthIndex) {
						if (berths[berthIndex] == null) {
							foundAvail = true;
							break;
						}
					}
					if (foundAvail) break;
				}
			}
			
			else if (side == ENTRANCE) { // iterate forward
				for (i=0; i < platforms.length; ++i) {
					platform = platforms[i];
					berths = platform.berths;
					for (berthIndex=0; berthIndex < berths.length; ++berthIndex) {
						if (berths[berthIndex] == null) {
							foundAvail = true;
							break;
						}
					}
					if (foundAvail) break;
				}
			}
			
			else throw new Error("Unknown side.");
							
			/* Bail if no empty berths found */
			if (!foundAvail) return null;
			
			/* Create the vehicle in the empty berth */
			var vehicleSeg:TrackSegment = Globals.tracks.getTrackSegment(platform.trackSegId);
			var pos:Number = platform.berthLength * (berthIndex+1); // place the nose at the downstream edge of the berth
			var vehicle:Vehicle = new Vehicle(
						              	IdGenerator.getVehicleId(),
						              	pos,
						              	0,
						              	0,
						              	vehicleSeg,
			                            vehicleSeg.getLatLng(pos),
			                            vehicleSeg.getElevation(pos),
			                            label,
			                            'DEFAULT',
			                            false
			                          );
			new VehicleOverlay(vehicle, Globals.tracks.getTrackOverlay(vehicleSeg.id)); // Placed in the global store by side effect.
			berths[berthIndex] = vehicle.id
			return vehicle;
		}
		
		public function toXML():XML {			
			var xml:XML = <Station id={id} label={label} >
								<TrackSegments/>
								<Platforms/>
							    <Coverage radius={coverageRadius}/>
							    <Usage peak_hour={peakHour} daily={daily} />
						  </Station>;
			
			for each (var ts:TrackSegment in allSegments) {
				xml.TrackSegments.appendChild(<ID>{ts.id}</ID>);
			}
			
			for each (var plat:Platform in platforms) {
				xml.Platforms.appendChild(plat.toXML());
			}
			 
			return xml;	
		}
		
		/** Requires that TrackSegments have been created already. */
		public static function fromXML(xml:XML):Station {
			var segments:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			for each (var id:String in xml.TrackSegments.ID) {
				var match:TrackSegment = Globals.tracks.getTrackSegment(id);
				if (!match) {
					Alert.show("Invalid XML file.");
				} else {
					segments.push(match);
				}
			}
			
			var platforms:Vector.<Platform> = new Vector.<Platform>();
			for each (var platXml:XML in xml.Platforms.Platform) {
				platforms.push(Platform.fromXML(platXml));
			}
			
			var station:Station = new Station(xml.@id,
											  xml.@label,
											  segments,
											  platforms,
											  xml.Coverage.@radius,
											  xml.Usage.@peak_hour,
											  xml.Usage.@daily);

			return station;

		}
		
	}
}