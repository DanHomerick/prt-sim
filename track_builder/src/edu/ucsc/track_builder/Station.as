package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;

	public class Station
	{	
		public var id:String;		
		public var label:String;
		public var allSegments:Vector.<TrackSegment>;
		public var platforms:Vector.<Platform>;
		public var coverageRadius:uint;
		public var peakHour:uint;
		public var daily:uint;
		
		public var overlay:StationOverlay;
		
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
			
			/* Side effect - store Station in Stations container */
			Undo.pushMicro(Globals.stations.stations, Globals.stations.stations.pop);
			Globals.stations.stations.push(this);
			
			/* Side effect - store TrackSegment:Station pairing in Station's dictionary */
			for each (var ts:TrackSegment in allSegments) {
				Undo.assign(Globals.stations.trackSegment2Station, ts.id, this);	
				Globals.stations.trackSegment2Station[ts.id] = this;
			}
		}	
		
		/** Reverses all of the constructor's side effects (with Undo support). Does not affect associated tracks or
		 * vehicles.
		 * 
		 * @see #Stations.remove */
		public function remove():void {
			var iterSeg:TrackSegment;		
			
			// Remove TrackSegment:Station pairings from Station's dictionary */			
			for each (iterSeg in this.allSegments) {
				Undo.assign(Globals.stations.trackSegment2Station, iterSeg.id, this);
				delete Globals.stations.trackSegment2Station[iterSeg.id];
			}
			
			// Remove Station from global store			
			function removeStation(item:Station, index:int, vector:Vector.<Station>):Boolean {return item !== this};
			Undo.assign(Globals.stations, "stations", Globals.stations.stations);
			Globals.stations.stations = Globals.stations.stations.filter(removeStation, this);
			
			// Remove all tracks that make up the Station
			for each (iterSeg in this.allSegments) {
				Globals.tracks.remove(iterSeg); // handles Undo
			}
		}
		
		public function toXML(modelNames:Vector.<String>):XML {			
			var xml:XML = <Station id={id} label={label}/>;
			
			for each (var ts:TrackSegment in allSegments) {
				xml.appendChild(<TrackSegmentID>{ts.id}</TrackSegmentID>);
			}
			
			for each (var plat:Platform in platforms) {
				xml.appendChild(plat.toXML());
			}

			xml.appendChild(<Coverage radius={coverageRadius}/>)
			xml.appendChild(<Usage peak_hour={peakHour} daily={daily}/>)
			
			// FIXME: initial_suppy and max_capacity are hardcoded to facilitate testing the design
			for each (var model_name:String in modelNames) {
				xml.appendChild(<Storage
				                    model_name={model_name}
				                    initial_supply="INF"
				                    max_capacity="INF"
				                />);
            }
			
			return xml;	
		}
		
		/** Requires that TrackSegments have been created already. */
		public static function fromXML(xml:XML):Station {
			var segments:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			for each (var id:String in xml.TrackSegmentID) {
				var match:TrackSegment = Globals.tracks.getTrackSegment(id);
				if (match == null) {
					throw new Error("Unknown TrackSegmentID: " + id + " referred to by Station: " + xml.@id)
				} else {
					segments.push(match);
				}
			}
			
			var platforms:Vector.<Platform> = new Vector.<Platform>();
			for each (var platXml:XML in xml.Platform) {
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