package edu.ucsc.track_builder
{	
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.MapMouseEvent;
	
	import flash.events.MouseEvent;
	import flash.geom.Vector3D;
	
	import mx.controls.Alert;
	
	/** Acts as a factory for Stations and StationOverlays.
	 * Keeps references to both of them in parallel arrays.
	 * Contains some handlers for GUI events relating to stations. */	
	public class Stations
	{			
		[Bindable] public var maxSpeed:Number;
		public var label:String = "";
		public var reverse:Boolean = false;
		[Bindable] public var lateralOffset:Number;
		[Bindable] public var accelLength:Number;
		[Bindable] public var decelLength:Number;
		[Bindable] public var slotLength:Number;		
		[Bindable] public var queueSlots:uint;
		[Bindable] public var unloadSlots:uint;
		[Bindable] public var loadSlots:uint;
		public var storageSlots:uint;
		public var coverageRadius:uint;
		public var peakHour:uint;
		public var daily:uint;		
		
		public var stations:Vector.<Station>;
		public var overlays:Vector.<StationOverlay>;		
		
		public var targetSegment:Station;
		public var targetOverlay:StationOverlay;

		public const OFF_RAMP:uint = 0;
		public const ON_RAMP:uint = 1;
		
		/** A mapping from trackSegment ids to Station objects */ 
		public var trackSegment2Station:Object; 
		
		[Bindable] public var startGround:Number = NaN;
		public var startOffset:Number;
		public function get startElev():Number {return startGround + startOffset;}
			
		[Bindable] public var endGround:Number = NaN;
		public var endOffset:Number = 0; 
		public function get endElev():Number {return endGround + endOffset;}

		/* TODO
		 * public function get minBypassLength():Number { }
		 */

		/* Constructor */
		public function Stations() {
			stations = new Vector.<Station>();
			overlays = new Vector.<StationOverlay>();
			trackSegment2Station = new Object();
		}

		public function onMapClick(event:MapMouseEvent):void {
			Undo.undo(Undo.PREVIEW); // undo any preview stuff
			var marker:SnappingMarker = Globals.getActiveMarker();			
			
			if (marker.getSnapOverlay() == null) {
				Alert.show("Cursor must be over a straight track segment to place a station.");
				return;
			}
			try {
				Undo.startCommand(Undo.USER); // somewhat temporary. Splits adding a station into two undo phases, creating 'bypass' track, then adding the station.						
				var stat:Station = makeOfflineStation(marker.getSnapOverlay(), marker.getLatLng(), false); // add the station to it, resizing it to just the right length
				Undo.assign(Globals, 'dirty', Globals.dirty);
				Globals.dirty = true; // mark that changes have occured since last save				
				Undo.endCommand();
			} catch (err:StationLengthError) {
				Undo.endCommand();
				Undo.undo(Undo.USER); // pop it from the USER stack
				Alert.show(err.message);
			} catch (err:TrackError) {
				Undo.endCommand();
				Undo.undo(Undo.USER); // pop it from the USER stack
				Alert.show(err.message);
			}
		}

		public function onMapMouseMove(event:MapMouseEvent):void {
			makePreview();
		}

		public function makePreview():void {
			// make a 'live preview', that is, a temporary station
			Undo.undo(Undo.PREVIEW); // get rid of the old 'live preview', if one exists
			var marker:SnappingMarker = Globals.getActiveMarker();
			if (marker.getSnapOverlay() == null) {
				return; // The active marker isn't attached to an overlay. Do nothing.
			}
			
			try {				
				Undo.startCommand(Undo.PREVIEW);	
				makeOfflineStation(marker.getSnapOverlay(), marker.getLatLng(), true);
				Undo.endCommand();
			} catch (err:StationLengthError) {
				Undo.endCommand();
				Undo.undo(Undo.PREVIEW); // undo any side effects from before the error
			} catch (err:TrackError) {
				// just do minimal cleanup. The user just won't see a preview in this case.
				Undo.endCommand();
				Undo.undo(Undo.PREVIEW); 					
			}
		}

		public function makeOfflineStation(mainLineOverlay:TrackOverlay, stationStart:LatLng, preview:Boolean):Station {
			if (stationStart == null) {
				throw new StationLengthError();
			}

			var stationBundle:TrackBundle = new TrackBundle();
			
			// Shadow Stations.reverse with a local reverse. False when overlay is not bidirectional.
			var reverse:Boolean = this.reverse && mainLineOverlay.bidirectional;
									
			var bypass:TrackSegment;
			if (!reverse) {
				bypass = mainLineOverlay.segments[0];
			} else { // reverse == true
				bypass = mainLineOverlay.segments[1];
			}

			// make the off ramp (squiggle + decel run)
			var bypassVec:Vector3D = Utility.calcVectorFromLatLngs(bypass.getStart(), bypass.getEnd());
			bypassVec.normalize();
			var offRamp:TrackBundle = Globals.tracks.makeRamp(bypassVec, stationStart, this.decelLength, this.lateralOffset,
					Globals.tracks.radius, Globals.tracks.driveSide, Tracks.OFF_RAMP, Globals.tracks.sCurveMaxSpeed, 
					bypass.getStartOffset(), bypass.getEndOffset(), preview);
			
			/* Make the main station line */ 			
			// make a TrackSegment for each Platform, then make the Platform
			var platforms:Vector.<Platform> = new Vector.<Platform>();
			var num_slots:Vector.<Number> = Vector.<Number>([unloadSlots, queueSlots, loadSlots]);
			var platformStart:LatLng = offRamp.connectNewLatLng.clone();
			var platformOverlay:TrackOverlay;
			const UNLOAD:int = 0;
			const QUEUE:int = 1;
			const LOAD:int = 2; 
			for (var i:uint=0; i < num_slots.length; ++i) {
				var platformEnd:LatLng = Utility.calcLatLngFromVector(platformStart, bypassVec, slotLength*num_slots[i]);
				platformOverlay = Globals.tracks.makeStraight( // creates the station main line
						platformStart, platformEnd, false, preview, this.maxSpeed);
				stationBundle.addOverlay(platformOverlay);
				var platform:Platform = new Platform(platformOverlay.segments[0].id, i)
				for (var j:uint=0; j < num_slots[i]; ++j) {
					if (i == UNLOAD) {
						platform.berths.push(new Berth(j, j*slotLength, (j+1)*slotLength, true, false, false, false))
					} else if (i == QUEUE) {
						platform.berths.push(new Berth(j, j*slotLength, (j+1)*slotLength, false, false, true, true)) // TEMP: Hardcoding the QUEUE platform as a storage entrance/exit
					} else if (i == LOAD) {
						platform.berths.push(new Berth(j, j*slotLength, (j+1)*slotLength, false, true, false, false))
					} else {
						throw new Error("Whoops. Bug.")
					}
				}
				platforms.push(platform);				
				platformStart = platformOverlay.getEnd();								
			}

			// make the on ramp (accel run + squiggle)
			var onRamp:TrackBundle = Globals.tracks.makeRamp(bypassVec, platformEnd.clone(), this.accelLength, this.lateralOffset,
					Globals.tracks.radius, Globals.tracks.driveSide, Tracks.ON_RAMP, Globals.tracks.sCurveMaxSpeed, 
					bypass.getEndOffset(), bypass.getEndOffset(), preview); 

			// Check that the resulting station will fit in the desired location.
			var availableTrackLength:Number;
			if (!reverse) {
				availableTrackLength = stationStart.distanceFrom(mainLineOverlay.getEnd());	
			} else { // reverse == true
				availableTrackLength = stationStart.distanceFrom(mainLineOverlay.getStart());
			}
			
			var stationLength:Number = stationStart.distanceFrom(onRamp.connectNewLatLng);
			
			if (stationLength > availableTrackLength) {
				throw new StationLengthError("Station needs a longer straight section.");
			}
			
			// Skip some steps when just generating a preview
			if (!preview) {
				// connect together all the parts of the station
				for (i=0; i < stationBundle.overlays.length - 1; ++i) {
					stationBundle.overlays[i].connect(stationBundle.overlays[i+1]);	
				} 
				offRamp.connectNewOverlays[0].connect(stationBundle.overlays[0]);
				stationBundle.overlays[stationBundle.overlays.length-1].connect(onRamp.connectExistingOverlays[0]);
				
				// split the main line and connect the ramps to it.				
				var bypassOverlay:TrackOverlay;
				var preStationOverlay:TrackOverlay;
				var postStationOverlay:TrackOverlay;
								
				if (!reverse) {				
					if (!stationStart.equals(mainLineOverlay.getStart())) { // Normal case.
						preStationOverlay = mainLineOverlay;
						bypassOverlay = mainLineOverlay.split(stationStart, mainLineOverlay.segments[0].getStartOffset(), preview);						
					} else { // Station is starting on mainLineOverlay's start. Don't split it. 
						if (mainLineOverlay.segments[0].prev_ids.length > 1) {
							throw new StationLengthError("The starting point for a station cannot be directly on a merge point.");	
						} else if (mainLineOverlay.segments[0].prev_ids.length == 1) {
							preStationOverlay = Globals.tracks.getTrackOverlay(mainLineOverlay.segments[0].prev_ids[0]);
						} else {
							preStationOverlay = null;
						}
						bypassOverlay = mainLineOverlay;
					}

					if (!onRamp.connectNewLatLng.equals(bypassOverlay.getEnd())) { // Normal case.
						postStationOverlay = bypassOverlay.split(onRamp.connectNewLatLng, bypassOverlay.segments[0].getStartOffset(), preview);	
					} else { // Station is ending on mainLineOverlay's end.
						if (bypassOverlay.segments[0].next_ids.length > 1) {
							throw new StationLengthError("The ending point for a station cannot be directly on a switch point.");
						} else if (bypassOverlay.segments[0].next_ids.length == 1) {
							postStationOverlay = Globals.tracks.getTrackOverlay(bypassOverlay.segments[0].next_ids[0]);
						} else {
							postStationOverlay = null;
						}
					}
				
				} else { // reverse == true					
					if (!onRamp.connectNewLatLng.equals(mainLineOverlay.getStart())) { // Normal case
						postStationOverlay = mainLineOverlay;
						bypassOverlay = mainLineOverlay.split(onRamp.connectNewLatLng, mainLineOverlay.segments[1].getEndOffset(), preview);
					} else { // Station ends on mainLineOverlay's start.
						bypassOverlay = mainLineOverlay;
						if (bypassOverlay.segments[1].next_ids.length > 1) {
							throw new StationLengthError("The ending point for a station cannot be directly on a switch point.");
						} else if (bypassOverlay.segments[0].next_ids.length == 1) {
							postStationOverlay = Globals.tracks.getTrackOverlay(bypassOverlay.segments[0].next_ids[0]);	
						} else {
							postStationOverlay = null;
						}
					}
													
					if (!stationStart.equals(mainLineOverlay.getEnd())) { // Normal case
						preStationOverlay = bypassOverlay.split(stationStart, mainLineOverlay.segments[1].getStartOffset(), preview);						
					} else { // Station is starting on reversed mainLineOverlay's end. Don't split it.
						if (mainLineOverlay.segments[1].prev_ids.length > 1) {
							throw new StationLengthError("The starting point for a station cannot be directly on a merge point.");
						} else if (bypassOverlay.segments[1].prev_ids.length == 1) { 						
							preStationOverlay = Globals.tracks.getTrackOverlay(bypassOverlay.segments[1].prev_ids[0]);
						} else {
							preStationOverlay = null;
						}					
					}
				}
				
				if (preStationOverlay != null) {
					preStationOverlay.connect(offRamp.connectExistingOverlays[0]);
				}
				if (postStationOverlay != null) {
					postStationOverlay.connect(onRamp.connectNewOverlays[0]);
				}
			}
			
			// Make the Station instance
			var id:String = IdGenerator.getStationId();
			var segs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			for each (var bundle:TrackBundle in [offRamp, stationBundle, onRamp]) {						
				for each (var olay:TrackOverlay in bundle.overlays) {
					segs.push(olay.segments[0]);
				}
			}
			var station:Station = new Station(id, this.label, segs, platforms, this.coverageRadius, this.peakHour, this.daily);			
			var overlay:StationOverlay = new StationOverlay(station);
						
			return station;
		}
		
		public function onMouseMove(event:MouseEvent):void {			
//			var start:LatLng = Globals.originMarker.getLatLng();
//			var end:LatLng = targetOverlay.calcLatLngFromDist(length, start, event); 
			
			//targetSegment = new Segment("", "",  
		}
		
		public function toDataXML(vehicles:Vector.<Vehicle>):XML {
			var scenarioModels:Vector.<VehicleModel> = Globals.vehicleModels.getModelsInUse(vehicles);
			var modelNames:Vector.<String> = new Vector.<String>();
			for each (var m:VehicleModel in scenarioModels) {
				modelNames.push(m.modelName);
			}
			
			var xml:XML = <Stations/>;
			for each (var station:Station in stations) {
				xml.appendChild(station.toXML(modelNames));
			}
			return xml;
		}
		
		public function fromDataXML(xml:XMLList):void {			
			for each (var stationXml:XML in xml) {
				var station:Station = Station.fromXML(stationXml);
				new StationOverlay(station); // placed in the global store by side effect.			
			}
		}

		/** Generate xml from current preferences */
		public function toPrefsXML():XML {
			var xml:XML = <Stations max_speed={maxSpeed}
			                        reverse={reverse}
			                        lateral_offset={lateralOffset}
			                        accel_length={accelLength}
			                        decel_length={decelLength}
			                        slot_length={slotLength}
			                        queue_slots={queueSlots}
			                        unload_slots={unloadSlots}
			                        load_slots={loadSlots}
			                        storage_slots={storageSlots}
			                        coverage_radius={coverageRadius}
			                        peak_hour={peakHour}
			                        daily={daily}
			               />
			return xml;
		}

		/** Generate xml from hard-coded default preferences */
		public function toDefaultPrefsXML():XML {
			var xml:XML = <Stations
							max_speed="3.5"
							reverse="0"
							lateral_offset="2.0"
							accel_length="42.5"
							decel_length="42.5"
							slot_length="5"
							queue_slots="2"
							unload_slots="3"
							load_slots="2"
							storage_slots="0"
							coverage_radius="500"
							peak_hour="0"
							daily="0"
						  />
			return xml;
		}
		
		public function fromPrefsXML(xml:XML):void {
			maxSpeed = xml.@max_speed;
			reverse = xml.@reverse == 'false' || xml.@reverse == '0' ? false : true;
			lateralOffset = xml.@lateral_offset;
			accelLength = xml.@accel_length;
			decelLength = xml.@decel_length;
			slotLength = xml.@slot_length;
			queueSlots = xml.@queue_slots;
			unloadSlots = xml.@unload_slots;
			loadSlots = xml.@load_slots;
			storageSlots = xml.@storage_slots;
			coverageRadius = xml.@coverage_radius;
			peakHour = xml.@peak_hour;
			daily = xml.@daily;
		}
		
		public function getNumberOfStations():int {
			return stations.length;			
		}
		
		/** Returns the approximate station coverage, in square meters. Only approximate, in that
		 * it will double count overlapping station areas.
		 */
		public function getStationCoverage():Number {
			var totalCoverage:Number = 0;
			for each (var stat:Station in stations) {
				totalCoverage += Math.PI * stat.coverageRadius * stat.coverageRadius; // area of circle
			}
			return totalCoverage;
		}
		
		public function reinitialize():void {
			// remove existing overlays from the map
			Globals.stationPane.clear();
			
			stations = new Vector.<Station>();
			overlays = new Vector.<StationOverlay>();
			trackSegment2Station = new Object();
		}
		
		/** Removes the station overlay, the station, and all tracks and track overlays associated with it.
		 * @param station The Station object to delete.
		 */
		public function remove(station:Station):void {						
			for each (var ts:TrackSegment in station.allSegments) {
				Globals.tracks.remove(ts);
			}
			
			station.remove();
			station.overlay.remove();
		}		
	}
}