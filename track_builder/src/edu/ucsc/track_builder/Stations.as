package edu.ucsc.track_builder
{	
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.MapMouseEvent;
	
	import flash.events.MouseEvent;
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;
	
	import mx.controls.Alert;
	
	/** Acts as a factory for Stations and StationOverlays.
	 * Keeps references to both of them in parallel arrays.
	 * Contains some handlers for GUI events relating to stations. */	
	public final class Stations
	{			
		[Bindable] public var maxSpeed:Number;
		public var label:String;
		public var bidirectional:Boolean;
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
			var overlay:TrackOverlay = Globals.destMarker.overlay;
			if (overlay == null) {
				Alert.show("Cursor must be over a straight track segment to place a station.");
				return;
			}
			try {				
				Undo.startCommand(Undo.USER); // somewhat temporary. Splits adding a station into two undo phases, creating 'bypass' track, then adding the station.		
				var latlng:LatLng = overlay.snapTo(null, Globals.destMarker.getLatLng()); // redundant?
				var stat:Station = makeOfflineStation(overlay, latlng, false, false); // add the station to it, resizing it to just the right length
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
			if (Globals.destMarker.overlay) {
				var overlay:TrackOverlay = Globals.destMarker.overlay;
				var latlng:LatLng = Globals.destMarker.getLatLng();			
				
				try {				
					Undo.startCommand(Undo.PREVIEW);	
					makeOfflineStation(overlay, latlng, false, true);
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
		}

		public function makeOfflineStation(bypassOverlay:TrackOverlay, stationStart:LatLng, reverse:Boolean, preview:Boolean):Station {						
			var bypass:TrackSegment;
			bypass = reverse ? bypassOverlay.segments[1] : bypassOverlay.segments[0]; 	

			if (stationStart != null) {
				bypassOverlay = bypassOverlay.split(stationStart, bypass.getStartOffset(), preview);
				bypass = reverse ? bypassOverlay.segments[1] : bypassOverlay.segments[0]; 
			} else {
				trace("Hmm");
			}

			// make the off ramp (squiggle + decel run)
			var segs:Vector.<TrackSegment> = makeRamp(bypass, accelLength, lateralOffset , Globals.tracks.radius, OFF_RAMP, preview);
			
			// make the main station line 
			var stationDir:Vector3D = Utility.calcVectorFromLatLngs(bypass.getStart(), bypass.getEnd()); // parallel to bypass
			
			// make a TrackSegment for each Platform, then make the Platform
			var platforms:Vector.<Platform> = new Vector.<Platform>();
			var slots:Vector.<Number> = Vector.<Number>([unloadSlots, queueSlots, loadSlots]);
			for (var i:uint=0; i < slots.length; ++i) {
				stationDir.scaleBy(slotLength*slots[i] / stationDir.length); // rescale to slotLength*slots
				var platformSegStart:LatLng = segs[segs.length-1].getEnd();
				var platformSegEnd:LatLng = Utility.calcLatLngFromVector(platformSegStart, stationDir);			
				segs.push(new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
				                           label,
				                           platformSegStart,
				                           platformSegEnd,
				                           maxSpeed,
				                           Globals.tracks.offset,
				                           Globals.tracks.offset,
				                           0,
				                           0,
				                           null,
				                           preview)); // creates the station main line
				platforms.push(new Platform(segs[segs.length-1].id,
											i,
				                            slots[i],
				                            slotLength,
				                            i == 0 ? true : false,  // unloading
				                            i == 2 ? true : false)); // loading
			}

			// make the on ramp (accel run + squiggle)			                           
			segs = segs.concat(makeRamp(segs[segs.length-1], accelLength, lateralOffset , Globals.tracks.radius, ON_RAMP, preview)); 

			// adjust the main line to have the right length
			var stationEnd:LatLng = segs[segs.length-1].getEnd();
			var stationLength:Number = stationStart.distanceFrom(stationEnd);			
			if (bypass.getStart().distanceFrom(bypass.getEnd()) >= stationLength) { // bypass was long enough already
				bypassOverlay.split(stationEnd, bypass.getEndOffset(), preview);			
			} else if (bypassOverlay.isEndExposed()) { // bypass too short, but it's end is exposed, so it may be modified
				bypassOverlay.setEnd(stationEnd);
			} else {				
				throw new StationLengthError("Station needs a longer straight section.");	
			}

			// connect the segments
			for (i=0; i < segs.length-1; ++i) {
				segs[i].connect(segs[i+1]);
			}

			// create overlays for the segments
			var segOverlays:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();
			for each (var ts:TrackSegment in segs) {
				var tsVec:Vector.<TrackSegment> = new Vector.<TrackSegment>();
				tsVec.push(ts);
				segOverlays.push(new TrackOverlay(tsVec));
			}
			
			// connect the beginning and ending segments to the main track
			if (!preview) {
				for each (var prevId:String in bypass.prev_ids) {
					var prev:TrackOverlay = Globals.tracks.getTrackOverlay(prevId);
					prev.connect(segOverlays[0]);
				}
				for each (var nextId:String in bypass.next_ids) {
					var next:TrackOverlay = Globals.tracks.getTrackOverlay(nextId);
					next.connect(segOverlays[segOverlays.length-1]);
				}
			}

			var id:String = IdGenerator.getStationId();
			var station:Station = new Station(id, label, segs, platforms, coverageRadius, peakHour, daily);			
			var overlay:StationOverlay = new StationOverlay(station);
			
			Undo.assign(Globals, 'dirty', Globals.dirty);
			Globals.dirty = true; // mark that changes have occured since last save
			
			return station;
		}

		/** seg: the straight track segment that the ramp begins from.
		 *  length: the amount of track required to decelerate to station entry speed.
		 *  offset: the minimum perpendicular seperation between the main line and the station track.
		 *  radius: minimum radius circle, as determined by lateral acceleration limits.
		 * 
		 * Uses two arcsegments of equal length to bring the station line away from the main line by amount 'offset'.
		 * Then uses a straight segment of whatever length is required to finish the deceleration.
		 * 
		 * For SkyTran San Jose Demo: offset=2, length=40
		 */ 		
		public function makeRamp(seg:TrackSegment, length:Number, offset:Number, radius:Number, type:uint, preview:Boolean):Vector.<TrackSegment> { 									
			var trackSegVec:Vector3D = Utility.calcVectorFromLatLngs(seg.getStart(), seg.getEnd()); // points along the main track 
			trackSegVec.normalize(); // make it a unit vector
			
			var newSegs:Vector.<TrackSegment> = new Vector.<TrackSegment>();

			var arcAngle:Number = Math.acos((radius - offset/2) / radius);
			var squiggleLen:Number = 2*radius*arcAngle;
			
			var startLatLng:LatLng;
			if (type == OFF_RAMP) {
				startLatLng = seg.getStart();
			} else { // if (type == ON_RAMP) {
				// make acceleration run
				var accelVec:Vector3D = trackSegVec.clone();
				var accelLen:Number = length - squiggleLen;
				if (accelLen > 0) { // normal case
					accelVec.scaleBy(accelLen);
				} // otherwise we just have a 1 meter accel segment (easier than ommitting entirely).
				var accelEnd:LatLng = Utility.calcLatLngFromVector(seg.getEnd(), accelVec);
				newSegs.push(new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
				                              label,
				                              seg.getEnd(),
				                              accelEnd,
				                              Globals.tracks.maxSpeed,
				                              Globals.tracks.offset,
				                              Globals.tracks.offset,
				                              0,
				                              0,
				                              null,
				                              preview)); 
				startLatLng = accelEnd.clone();
			}
			
			// make a vector that points to the center of the first curve
			var centerVec:Vector3D;
			if (type == OFF_RAMP) {
				centerVec = new Vector3D(trackSegVec.y, -trackSegVec.x); // CW perpindicular
			} else { // if (type == ON_RAMP)
				centerVec = new Vector3D(-trackSegVec.y, trackSegVec.x); // CCW perpindicular
			}
			centerVec.scaleBy(radius);
			var centerLatLng:LatLng = Utility.calcLatLngFromVector(startLatLng, centerVec);
						
			var chordLen:Number = Math.SQRT2*radius*Math.sqrt(1-Math.cos(arcAngle)); // from law of cosines
			var chordAngle:Number = Math.sin( (offset/2) / chordLen); // actually, 90 deg - chordangle
						
			// make a vector that points to the midpoint of the squiggle
			var midpoint:Vector3D = trackSegVec.clone();		
			var matrix:Matrix3D = new Matrix3D();
			if (type == OFF_RAMP) {
				matrix.appendRotation(-chordAngle * (180/Math.PI), Vector3D.Z_AXIS); // rotate CW. Convert to degrees
			} else { // if (type == ON_RAMP) {
				matrix.appendRotation(chordAngle * (180/Math.PI), Vector3D.Z_AXIS); // rotate CCW. Convert to degrees
			}
			midpoint = matrix.transformVector(midpoint);			
			midpoint.scaleBy(chordLen);
			var midpointLatLng:LatLng = Utility.calcLatLngFromVector(startLatLng, midpoint);

			// make control point. Know the arc angle, the control point is the bisect angle, along the seg seg.
			var controlLen:Number = Math.tan(arcAngle/2) * radius;
			var controlVec:Vector3D = trackSegVec.clone();			
			controlVec.scaleBy(controlLen);
			var controlLatLng:LatLng = Utility.calcLatLngFromVector(startLatLng, controlVec);						
			
			// create the endpoint for the second curve in the squiggle.
			var endSquiggle:LatLng = Utility.calcLatLngFromVector(midpointLatLng, midpoint); 

			// create the center point for the second curve
			centerVec.negate();
			var centerTwoLatLng:LatLng = Utility.calcLatLngFromVector(endSquiggle, centerVec);

			// create the control point for the second curve in the squiggle. Same logic as the first.
			controlVec.negate();			
			var controlTwoLatLng:LatLng = Utility.calcLatLngFromVector(endSquiggle, controlVec);

			// create the first curve
			if (type == ON_RAMP) {
				arcAngle *= -1;
			}								
			newSegs.push(new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
			                              label,
			                              startLatLng,
			                              midpointLatLng,
			                              Globals.tracks.maxSpeed,
			                              Globals.tracks.offset,
			                              Globals.tracks.offset,
			                              radius,
			                              -arcAngle,			                              
			                              centerLatLng,
			                              preview));
			                              
			// create the second curve			                              
			newSegs.push(new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
			                              label,
			                              midpointLatLng,
			                              endSquiggle,
			                              Globals.tracks.maxSpeed,
			                              Globals.tracks.offset,
			                              Globals.tracks.offset,
			                              radius,
			                              arcAngle,
			                              centerTwoLatLng,
			                              preview));

			if (type == OFF_RAMP) {						
				// create the straight, deceleration segment. Begins decelereation in the squiggle.
				var decelVec:Vector3D = trackSegVec.clone();
				var decelLen:Number = length-squiggleLen;
				if (decelLen > 0) { // normal case. Requires some straight segment to finish deceleration.
					decelVec.scaleBy(decelLen);	
				} // otherwise, use a 1 meter decel seg. Easier than ommitting entirely.				
				var decelEndLatLng:LatLng = Utility.calcLatLngFromVector(endSquiggle, decelVec);
				newSegs.push(new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
				                              label,
				                              endSquiggle,
				                              decelEndLatLng,
				                              Globals.tracks.maxSpeed,
				                              Globals.tracks.offset,
				                              Globals.tracks.offset,
				                              0, 0, null, preview));
			}					
			
//			trace("ArcAngle: ", arcAngle * 180/Math.PI);
//			trace("Chordlen: ", chordLen);
//			trace("Start: ", startLatLng);
//			trace("Midpoint: ", midpointLatLng);
//			trace("Control1: ", controlLatLng);
//			trace("Control2: ", controlTwoLatLng);
//			trace("Endpoint: ", endSquiggle);
//			trace("len start -> end: ", Utility.calcVectorFromLatLngs(seg.getStart(), endSquiggle).length);
			
			return newSegs;
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
			                        bidirectional={bidirectional}
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
							bidirectional="0"
							lateral_offset="10"
							accel_length="40"
							slot_length="5"
							queue_slots="5"
							unload_slots="5"
							load_slots="5"
							storage_slots="0"
							coverage_radius="500"
							peak_hour="0"
							daily="0"
						  />
			return xml;
		}
		
		public function fromPrefsXML(xml:XMLList):void {
			maxSpeed = xml.@max_speed;
			bidirectional = xml.@bidirectional == 'false' || xml.@bidirectional == '0' ? false : true;
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