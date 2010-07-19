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
			
			// Shadow Stations.reverse with a local reverse. Only true when the overlay is bidirectional.
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
			var offRamp:TrackBundle = makeRamp(bypassVec, stationStart, this.accelLength, this.lateralOffset,
					Globals.tracks.radius, Globals.tracks.driveSide, this.OFF_RAMP, Globals.tracks.maxSpeed, 
					bypass.getStartOffset(), bypass.getEndOffset(), preview);
			
			/* Make the main station line */ 			
			// make a TrackSegment for each Platform, then make the Platform
			var platforms:Vector.<Platform> = new Vector.<Platform>();
			var num_slots:Vector.<Number> = Vector.<Number>([unloadSlots, queueSlots, loadSlots]);
			var platformStart:LatLng = offRamp.connectNewLatLng.clone();
			var platformOverlay:TrackOverlay;
			for (var i:uint=0; i < num_slots.length; ++i) {
				var platformEnd:LatLng = Utility.calcLatLngFromVector(platformStart, bypassVec, slotLength*num_slots[i]);
				platformOverlay = Globals.tracks.makeStraight( // creates the station main line
						platformStart, platformEnd, false, preview, this.maxSpeed);
				stationBundle.addOverlay(platformOverlay);
				var platform:Platform = new Platform(platformOverlay.segments[0].id, i)
				for (var j:uint=0; j < num_slots[i]; ++j) {
					if (i == 0) {
						platform.berths.push(new Berth(j, j*slotLength, (j+1)*slotLength, true, false))
					} else if (i == 1) {
						platform.berths.push(new Berth(j, j*slotLength, (j+1)*slotLength, false, false))
					} else if (i == 2) {
						platform.berths.push(new Berth(j, j*slotLength, (j+1)*slotLength, false, true))
					} else {
						throw new Error("Whoops. Bug.")
					}
				}
				platforms.push(platform);				
				platformStart = platformOverlay.getEnd();								
			}

			// make the on ramp (accel run + squiggle)
			var onRamp:TrackBundle = makeRamp(bypassVec, platformEnd.clone(), this.accelLength, this.lateralOffset,
					Globals.tracks.radius, Globals.tracks.driveSide, this.ON_RAMP, Globals.tracks.maxSpeed, 
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

		/** Makes a station ramp consisting of two curved segments (the 'squiggle') and a straight segment.
		 * The squiggle moves the track <code>lateralOffset</code> meters away from vec and leaves it parallel to vec.
		 * The straight segment is <code>length - squiggle_length</code> meters long. If the ramp would exceed
		 * <code>length</code> meters, then a StationLengthError is thrown.
		 * 
		 * @param vec A unit vector that points in the direction of the bypass. Must have length 1.
		 * @param latlng Starting location for the ramp.
		 * @param length Total track length for the entire ramp (including decel segment). Measured in meters.
		 * @param lateralOffset Desired distance from the bypass and the station line. Measured in meters.
		 * @param radius Desired radius for the two curve segments which make up the squiggle. The actual radius used
		 * will be: <code>max(radius, lateralOffset/2.0)</code>. Measured in meters.
		 * @param side One of Tracks.LEFT or Tracks.RIGHT, which may be thought of as counterclockwise and clockwise
		 * respectively.
		 * @param rampType One of OFF_RAMP or ON_RAMP. 
		 * @param maxSpeed Measured in meters/second.
		 * @param startOffset Vertical distance from the ground of the start point. Measured in meters.
		 * @param endOffset Vertical distance from the ground of the end point. Measured in meters.
		 * @param preview Whether or not this function has been called as part of the live preview system.
		 * 
		 * @return A TrackBundle containing the TrackOverlays for the ramp. For an OFF_RAMP,
		 * the bundle's only connectExisting element refers to where the ramp connects to the main line and 
		 * the bundle's connectNew refers to where the ramp connects to the station line. For an ON_RAMP,
		 * connectExisting refers to the where the ramp connects to the station line, and connectNew refers to where
		 * the ramp connects to the main line. 
		 */ 
		public function makeRamp(
				vec:Vector3D, latlng:LatLng, length:Number, lateralOffset:Number, radius:Number, side:String,
				rampType:uint, maxSpeed:Number, startOffset:Number, endOffset:Number, preview:Boolean 
				):TrackBundle
		{
			var bundle:TrackBundle = new TrackBundle();
					
			// If the radius is less than the lateral offset, we would need to introduce a straight segment in the
			// 'squiggle'. Rather than doing this, just increase the radius.
			radius = Math.max(radius, lateralOffset/2.0);
			var angle:Number = Math.acos((radius - lateralOffset/2.0)/radius);
			
			// Assumes both curves in the squiggle are the same length.
			var squiggleLength:Number = 2*angle*radius
			var straightLength:Number = length - squiggleLength;
			if (straightLength < 0.001) {
				throw new StationLengthError("The requested station ramp length is too short, given the requested lateral offset and curve radius.");
			} 
			
			if (rampType == ON_RAMP) {
				var straightEnd:LatLng = Utility.calcLatLngFromVector(latlng, vec, straightLength);
				var straight:TrackOverlay = Globals.tracks.makeStraight(latlng, straightEnd, false, preview, maxSpeed, startOffset, endOffset);
				bundle.addOverlay(straight);
				bundle.setConnectExisting(latlng, straight);				
				latlng = straight.getEnd(); // start the squiggle at latlng, regardless of whether an ON_RAMP or OFF_RAMP
			}
			
			var squiggleIntersectDist:Number = Math.tan(angle/2.0)*radius;
			var curveOneIntersect:LatLng = Utility.calcLatLngFromVector(latlng, vec, squiggleIntersectDist);
			var curveOneV1:Vector3D = vec.clone();
			curveOneV1.negate();
			var curveOneV2:Vector3D = vec.clone();						
			if ((side == Tracks.LEFT && rampType == OFF_RAMP) || (side == Tracks.RIGHT && rampType == ON_RAMP)) { // rotate vector CCW
				Utility.rotate(curveOneV2, angle);
			} else { // (side == Tracks.LEFT && ON_RAMP) || (side == Tracks.RIGHT && OFF_RAMP) // rotate vector CW
				Utility.rotate(curveOneV2, -angle);
			}
						 
			var curveOne:TrackOverlay = Globals.tracks.makeTangentCurve(
					curveOneIntersect, curveOneV1, curveOneV2, maxSpeed, startOffset, endOffset, radius, false, preview);
			bundle.addOverlay(curveOne);
			
			// use a couple vectors while finding the intersect point for curve 2
			var curveTwoIntersectVec:Vector3D = curveOneV2.clone();
			curveTwoIntersectVec.scaleBy(lateralOffset / Math.cos( Math.PI/2.0 - angle ));
			var curveTwoIntersect:LatLng = Utility.calcLatLngFromVector(curveOneIntersect, curveTwoIntersectVec);
			var curveTwoV1:Vector3D = curveOneV2.clone();
			curveTwoV1.negate();
			var curveTwoV2:Vector3D = curveOneV1.clone();
			curveTwoV2.negate();
						
			var curveTwo:TrackOverlay = Globals.tracks.makeTangentCurve(
						curveTwoIntersect, curveTwoV1, curveTwoV2, maxSpeed, startOffset, endOffset, radius, false, preview);
			bundle.addOverlay(curveTwo);
			
			if (rampType == OFF_RAMP) {
				var straightStart:LatLng = curveTwo.getEnd();
				var straightEnd:LatLng = Utility.calcLatLngFromVector(straightStart, vec, straightLength);
				var straight:TrackOverlay = Globals.tracks.makeStraight(straightStart, straightEnd, false, preview);
				bundle.addOverlay(straight);
				bundle.setConnectNewLatLng(straightEnd);
				bundle.markAsConnectNew(straight);
			}
			
			/* Make connections within the bundle and set data for external connections. */ 
			trace("curveOne:", curveOne.getStart().toString(), " : ", curveOne.getEnd().toString());
			trace("curveTwo:", curveTwo.getStart().toString(), " : ", curveTwo.getEnd().toString());
			trace("distances:", curveOne.getStart().distanceFrom(curveTwo.getStart()),
			                    curveOne.getStart().distanceFrom(curveTwo.getEnd()),
			                    curveOne.getEnd().distanceFrom(curveTwo.getStart()),
			                    curveOne.getEnd().distanceFrom(curveTwo.getEnd()));
			curveOne.connect(curveTwo);
			if (rampType == OFF_RAMP) {
				bundle.setConnectExisting(latlng, curveOne);
				trace("curveTwo, straight:", curveTwo.getEnd().toString(), straight.getStart().toString(), curveTwo.getEnd().distanceFrom(straight.getStart()));
				curveTwo.connect(straight);
				trace("straight & vec:", Utility.angleBetween(Utility.calcVectorFromLatLngs(straight.getStart(), straight.getEnd()), vec));
				trace("curveTwo & vec:", Utility.angleBetween(Utility.calcVectorFromLatLngs(curveTwo.getCenter(), curveTwo.getEnd()), vec)); 
			} else { // rampType == ON_RAMP
				bundle.setConnectNewLatLng(curveTwo.getEnd());
				bundle.markAsConnectNew(curveTwo);
				straight.connect(curveOne);
				trace(straight.getEnd(), curveOne.getStart(), straight.getEnd().distanceFrom(curveOne.getStart()));
			}
						
			return bundle;
		}
		
		public function onMouseMove(event:MouseEvent):void {			
//			var start:LatLng = Globals.originMarker.getLatLng();
//			var end:LatLng = targetOverlay.calcLatLngFromDist(length, start, event); 
			
			//targetSegment = new Segment("", "",  
		}
		
		public function toDataXML():XML {
			var xml:XML = <Stations/>;
			for each (var station:Station in stations) {
				xml.appendChild(station.toXML());
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
							lateral_offset="5"
							accel_length="42.5"
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
		
		public function fromPrefsXML(xml:XMLList):void {
			maxSpeed = xml.@max_speed;
			reverse = xml.@reverse == 'false' || xml.@reverse == '0' ? false : true;
			lateralOffset = xml.@lateral_offset;
			accelLength = xml.@accel_length;
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
		}
		
		/** Removes the station overlay, the station, and all tracks and track overlays associated with it */
		public function removeStationOverlay(overlay:StationOverlay):void {
			Undo.pushMicro(Globals.stationPane, Globals.stationPane.addOverlay, overlay);		
			Globals.stationPane.removeOverlay(overlay);
			
			// remove overlay from Stations.
			function removeSOL (item:StationOverlay, index:int, vector:Vector.<StationOverlay>):Boolean {return item != overlay;};
			Undo.assign(this, 'overlays', overlays);
			overlays = overlays.filter(removeSOL); // note that filter returns a new Vector, which is why I'm able to just store a ref to the old vector.
			
			for each (var ts:TrackSegment in overlay.station.allSegments) {
				var tOL:TrackOverlay = Globals.tracks.getTrackOverlay(ts.id);
				Globals.tracks.removeTrackOverlay(tOL);
			}
			
			// remove station from Stations
			function removeS (item:Station, index:int, vector:Vector.<Station>):Boolean {return item.id != overlay.station.id;};
			Undo.assign(this, 'stations', stations);
			stations = stations.filter(removeS);
		}		
	}
}