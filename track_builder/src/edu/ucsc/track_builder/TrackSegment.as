package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import flash.geom.Vector3D;
	
	public class TrackSegment
	{ 
		/** A unique id, generally created by IdGenerator, having a specific format. */
		public var id:String;
		/** A non-unique label, which may have any format */
		public var label:String;   // not unique
		
		// These should "properly" be private variables, but I'd like Undo to have easy
		// access to them, so I'm using naming convention alone.
		public var _start:LatLng;
		public var _end:LatLng;
		public var _center:LatLng;
		public var _startOffset:Number;
		public var _endOffset:Number;		
		
		/** Ground level elevation at <code>start</code>. */
		public var startGround:Number;
		/** Ground level elevation at <code>end</code>. */
		public var endGround:Number;
		/** Ground level elveation at <code>center</code>. */
		public var centerGround:Number;	

		/** Linear length, taking into account vertical distance and curvature */
		public var length:Number;
		/** Maximum speed for segment. */
		public var maxSpeed:Number;
		/** If a curved segment, the radius of the curvature. 0 if straight. Use NaN to allow straight segments to directly connect to each other. */
		public var radius:Number;
		/** If a curved segment, the arclength of the segment in radians. Positive indicates a CCW direction, negative indicates CW. 0 if straight. */
		public var arcAngle:Number;
		/** Indicates that the object is being created for the 'preview', which shows where the track will be created when the user clicks. */
		public var preview:Boolean;

		/** The id(s) of the trackSegment(s) that this segment connects to. */
		public var next_ids:Vector.<String>;     // of String ids
		/** The id(s) of the trackSegment(s) that connect to this segment. */
		public var prev_ids:Vector.<String>;     // of String ids
		/** The id of anti-parallel tracksegment in a bidirectional layout.*/
		public var parallel_ids:Vector.<String>; 
		
		/** Which overlay, if any, the segment is associated with. */
		public var overlay:TrackOverlay;
		
		/** Fragment of id string that indicates that this segment is 'forward'. */ 
		public static var forwardExt:String = "_forward"
		/** Fragment of id string that indicates that this segment is 'reverse'. */
		public static var reverseExt:String = "_reverse"
		
		/** Maximum error that is allowed in the angle between two TrackSegments while still making a connection.*/
		public static const MAX_CONNECTION_ANGLE:Number = 0.0872664626; // 5 degrees
		
		/** Maximum error that is allowed in the distance between two TrackSegments while still making a connection.*/
		public static const MAX_CONNECTION_DISTANCE:Number = 0.1; // 10 cm

		/**
		 * Constructor for a TrackSegment object. Adds itself to the vector at Globals.tracks.segments,
		 * and begins asynchronously fetching elevation data for the starting and ending points.
		 * Connections to other segments are made after the object is created by adding id strings to
		 * the next_ids and prev_ids vectors. Typically use the <code>connect</code> function for this.
		 * 
		 * @param id A unique string, supplied by IdGenerator class.
		 * @param label A non-unique label which may take any value.
		 * @param start The coordinates of the starting point.
		 * @param end The coordinates of the ending point.
		 * @param maxSpeed A speed limit imposed by the track segment, which may differ from any vehicle imposed speed limits.
		 * @param startOffset Distance, in meters, that the start point is raised/lowered above ground level.
		 * @param endOffset Distance, in meters, that the start point is raised/lowered above ground level.	
		 * @param radius If a curved segment, the radius of the curvature. 0 if straight. Use NaN to allow straight segments to directly connect to each other.
		 * @param arcAngle If a curved segment, the arclength of the segment in radians. Positive indicates a CCW direction, negative indicates CW. 0 if straight.
		 * @param center If a curved segment, the coordinates of the central point of the arc (not midpoint). null if straight. 
		 * @param preview Indicates that the object is being created for the 'preview', which shows where the track will be created when the user clicks. 
		 * 
		 * @see #connect()		   
		 */
		public function TrackSegment(id:String,
								label:String,
								start:LatLng,
								end:LatLng,
								maxSpeed:Number,
								startOffset:Number,
								endOffset:Number,
								radius:Number,
								arcAngle:Number,
								center:LatLng,
								preview:Boolean)
		{
			// denotes that the segment is being created while the mouse is being moved (prior to a click)
			this.preview = preview;

			this.id = id;
			this.label = label;
			this.maxSpeed = maxSpeed;	
			this.startGround = NaN; // ground values are determined when the start/end LatLngs are set.
			this.endGround = NaN;
			this.length = NaN; // is set by fetchElevation once both elevations are ready.
			this._startOffset = startOffset;
			this._endOffset = endOffset;

			this.next_ids = new Vector.<String>();
			this.prev_ids = new Vector.<String>();
			this.parallel_ids = new Vector.<String>();

			// for curved segments
			this.radius = radius;
			this.arcAngle = arcAngle;
			this._center = center ? center : null;
			
			/* Two notes about setStart and setEnd:
			 *  1. We never want _start or _end to be undone back to null, so we bypass the undo system in the constructor.
			 *  2. Must be called after the offsets, the ground values, and length have been initialized.
			 */
			setStart(start, true, preview); 
			setEnd(end, true, preview);
			
			/* Side effects */			
			Undo.pushMicro(Globals.tracks.segments, function():void {delete Globals.tracks.segments[id];});
			Globals.tracks.segments[id] = this; // Add a reference to the global store.
			
		}
		
		/* Fetch elevation whenever start or end is updated. Also, adhere to convention that only copies
		 * of LatLng objects should be shared. */
		public function getStart():LatLng {return this._start.clone();}
		public function setStart(latlng:LatLng, bypassUndo:Boolean=false, preview:Boolean=false):void {
			if (!bypassUndo) {Undo.assign(this, '_start', _start);}
			_start = latlng;
			if (!preview) {
				Globals.elevationService.requestElevations(Vector.<LatLng>([latlng.clone()]),
														   function(latlngs:Vector.<LatLng>, elevation:Number):void {
																// check that the latlng wasn't moved again before a response arrived
			                                                     if (_start.equals(latlngs[0])) { 
			                                                     	startGround = elevation;
			                                                     	updateLength();
			                                                     }
															});
			}
		}
		public function getEnd():LatLng {return this._end.clone();} 
		public function setEnd(latlng:LatLng, bypassUndo:Boolean=false, preview:Boolean=false):void {
			if (!bypassUndo) {Undo.assign(this, '_end', _end);}
			_end = latlng;
			if (!preview) {
				Globals.elevationService.requestElevations(Vector.<LatLng>([latlng.clone()]),
														  function(latlngs:Vector.<LatLng>, elevation:Number):void {
			                                                     // check that the latlng wasn't moved again before a response arrived
			                                                     if (_end.equals(latlngs[0])) {
			                                                     	endGround = elevation;
			                                                     	updateLength();
			                                                     }
															});
			}		
		}
		public function getCenter():LatLng {return _center ? _center.clone() : _center;} // don't try to clone null
		public function setCenter(latlng:LatLng):void {Undo.assign(this, '_center', _center); _center = latlng;} 
		
		public function isCurved():Boolean {return _center != null;}
				
		
		/** Find the LatLng coordinates for locations between start and end.
		 * @param stepSize The number of meters between samples.
		 * @return Vector of new LatLng objects. First element is always start, and the last element is end.
		 */
		public function getLatLngs(stepSize:Number):Vector.<LatLng> {
			var result:Vector.<LatLng> = new Vector.<LatLng>();
			result.push(this.getStart()); // copy of _start
			var samples:int = int(this.h_length/stepSize);
			var vec:Vector3D;
			var i:int;
			
			if (!isCurved()) { // straight segment				
				vec = Utility.calcVectorFromLatLngs(this._start, this._end);
				vec.scaleBy(stepSize/vec.length);
				
				for (i = 0; i < samples; i++) {
					result.push(Utility.calcLatLngFromVector(result[result.length-1], vec));
				}
			} else { // curved segment
				vec = Utility.calcVectorFromLatLngs(this._center, this._start);
				var stepAngle:Number = stepSize/this.radius;
				if (this.arcAngle < 0) { // CW rotation
					stepAngle = -stepAngle;
				}
				var x:Number = vec.x;
				var y:Number = vec.y;
				var cosAngle:Number = Math.cos(stepAngle);
				var sinAngle:Number = Math.sin(stepAngle);
				for (i = 0; i < samples; i++) {
					// Rotate by stepAngle
					vec.x = x*cosAngle - y*sinAngle; 
					vec.y = x*sinAngle + y*cosAngle;
					result.push(Utility.calcLatLngFromVector(this._center, vec)); 
				}
			}			
			result.push(this.getEnd()); // copy of _end
			return result;
		}
		
		public function getStartOffset():Number {return _startOffset};
		public function setStartOffset(offset:Number):void {			
			Undo.assign(this, '_startOffset', _startOffset);
			_startOffset = offset;		
			
			for each (var segId:String in prev_ids) {
				var seg:TrackSegment = Globals.tracks.getTrackSegment(segId);
				if (Utility.compareElevations(offset, seg.getEndOffset()) != 0) {
					seg.setEndOffset(offset);
				}
			}
		}
		public function getOffset(position:Number):Number {
		    var slope:Number = (_endOffset - _startOffset) / h_length;
		    return slope*position + _startOffset; // y = mx + b 
		}
		
		public function get startElev():Number {return startGround + _startOffset;}				
			
		public function getEndOffset():Number {return _endOffset};
		public function setEndOffset(offset:Number):void {
			Undo.assign(this, '_endOffset', _endOffset);
			_endOffset = offset;
			
			for each (var segId:String in next_ids) {
				var seg:TrackSegment = Globals.tracks.getTrackSegment(segId);
				if (Utility.compareElevations(offset, seg.getStartOffset()) != 0) {
					seg.setStartOffset(offset);
				}
			}
		}		 
		public function get endElev():Number {return endGround + _endOffset;}
		
		/**
		 * Raises the lower end to remove any elevation gradiant.
		 * @throws Error If the levelling requires an offset greater than maxOffset.
		 */
		public function makeLevel(maxOffset:Number):void {
			var cmp:Number = Utility.compareElevations(startElev, endElev);
			var offset:Number;
			if (cmp < 0) { // start is lower
				offset = endElev - startGround;
				if (offset > maxOffset) {
					throw new Error("makeLevel on segment " + id + " failed"); 
				}
				setStartOffset(offset);
			} else if (cmp > 0) { // end is lower
				offset = startElev - endGround;
				if (offset > maxOffset) {
					throw new Error("makeLevel on segment " + id + " failed"); 
				}
				setEndOffset(offset);
			} // else it is already level		
		} 
		
		/** Horizontal length. Ignores any vertical component. */
		public function get h_length():Number {
			if (!isCurved()) { // straight seg
				return _start.distanceFrom(_end);
			} else {
				return Math.abs(arcAngle*radius);
			}
		}

		/** Returns a new copy of the trackSegment. The prev_ids and next_ids are left empty.
		 * parallel_ids gets updated appropriately for both the original and the clone.
		 * 
		 * flipDir: when true, the starting and ending point are reversed, the id extension is toggled.
		 * newId: Allows the id of the new segment to be specified.
		 */
		public function clone(flipDir:Boolean, newId:String=null):TrackSegment {
		    var ts:TrackSegment;
		
			var parts:Array = this.id.split("_");
			var ext:String = '_' + parts[parts.length-1];
			
			if (flipDir) { // reverse the direction of the segment
				if (newId == null) {
					// use a new ID, with a toggled direction extension
					if (ext == TrackSegment.forwardExt) {
						newId = IdGenerator.getTrackSegId(TrackSegment.reverseExt);
					} else if (ext == TrackSegment.reverseExt) {
						newId = IdGenerator.getTrackSegId(TrackSegment.forwardExt);
					} else {
						throw new Error("Unknown id format: ", id);
					}
				}
				ts = new TrackSegment (newId, label, getEnd(), getStart(), maxSpeed, _endOffset, _startOffset, radius, -arcAngle, getCenter(), preview);				
			} else { // keep the same direction
				if (newId == null) {
					// use a new ID, with the same direction extension
					if (ext == TrackSegment.forwardExt) {
						newId = IdGenerator.getTrackSegId(TrackSegment.forwardExt);
					} else if (ext == TrackSegment.reverseExt) {
						newId = IdGenerator.getTrackSegId(TrackSegment.reverseExt);
					} else {
						throw new Error("Unknown id format: ", id);
					}
				} 
				ts = new TrackSegment (newId, label, getStart(), getEnd(), maxSpeed, _startOffset, _endOffset, radius, arcAngle, getCenter(), preview);
			}
			
 			/* For now, only two lanes possible. More work on id system required to extend beyond this. */			
//			for each (otherId in this.parallel_ids) {
//				var otherTs:TrackSegment = Globals.tracks.getTrackSegment(otherId);
//				otherTs.parallel_ids.push(ts.id);
//				ts.parallel_ids.push(otherId);	
//			}
			
			this.parallel_ids.push(ts.id);
			ts.parallel_ids.push(this.id);
			Undo.pushMicro(this.parallel_ids, this.parallel_ids.pop);
			Undo.pushMicro(ts.parallel_ids, ts.parallel_ids.pop); // may be ommitted, since ts will be destroyed.
			
			return ts;
		}

		/** Checks for a viable connection between the trackSegments, and if found adds the appropriate
		 * ids to prev_ids and next_ids on each segment. A viable connection is one in which the endpoints
		 * overlap appropriately, and in which a curve smoothly flows into a straight segment (or vice versa).
		 * 
		 * FIXME: Does not check for elevation mismatches.
		 */
		public function connect(other:TrackSegment):void {
			/* Find a shared endpoint, if one exists. Shared is defined as the endpoints being within 
		     * MAX_CONNECTION_DISTANCE of each other and in a nose->tail configuration.
		     */
		    var shared:LatLng;
			if (this._end.distanceFrom(other._start) < MAX_CONNECTION_DISTANCE) {
				shared = this.getEnd();
			} else if (other._end.distanceFrom(this._start) < MAX_CONNECTION_DISTANCE) {
				shared = other.getEnd();
			} else {
				return; // no shared endpoints
			}
			
			// Only check that the angles are compatible if neither radius is set to NaN.
			if (!isNaN(this.radius) && !isNaN(other.radius)) {
				/* Both segments are curved. */
				if (this.isCurved() && other.isCurved()) { 
					var curveAVec:Vector3D = Utility.calcVectorFromLatLngs(this._center, shared);
					var curveBVec:Vector3D = Utility.calcVectorFromLatLngs(other._center, shared);
					
					// Tangent test. Check that the vectors are parallel or antiparallel. 
					var angle:Number = Utility.angleBetween(curveAVec, curveBVec);
					if (Math.abs(angle) < MAX_CONNECTION_ANGLE) { // angle ~ 0. Parallel.
						// Direction of the curves must match.
						if ((this.arcAngle > 0 && other.arcAngle < 0) || (this.arcAngle < 0 && other.arcAngle > 0)) {
							trace("Two curved segments not connected due to incompatible directions.");
							return;
						}
					}
					else if (Math.abs(angle - Math.PI) < MAX_CONNECTION_ANGLE) { // angle ~ pi. Antiparallel.
						// Directions of the curves must be opposite.
						if ((this.arcAngle > 0 && other.arcAngle > 0) || (this.arcAngle < 0 && other.arcAngle < 0)) {
							trace("Two curved segments not connected due to incompatible directions.");
							return;
						}
					}
					else { // Neither parallel or antiparallel.
						trace("Two curved segments not connected due to incompatible angle:", angle);
						return;
					}
				}
				
				/* One segment is curved. */
				else if (this.isCurved() || other.isCurved()) {
					var curve:TrackSegment = this.isCurved() ? this : other;
					var straight:TrackSegment = !this.isCurved() ? this : other;
					var curveVec:Vector3D = Utility.calcVectorFromLatLngs(curve._center, shared);
					var straightVec:Vector3D = Utility.calcVectorFromLatLngs(straight._start, straight._end);
					
					// Angle between a straight and curved piece must be pi/2 (perpendicular).
					angle = Utility.angleBetween(curveVec, straightVec);					
					if (Math.abs(angle - Math.PI/2.0) > MAX_CONNECTION_ANGLE) {
						trace("One curved and one straight segment not connected due to incompatible angle:", angle);
						return;
					}
					
					// Check that the direction of the curve does not form an acute angle.
					var crossProd:Vector3D = straightVec.crossProduct(curveVec);
					if (crossProd.z > 0 && curve.arcAngle > 0) {
						trace("One curved and one straight segment not connected due to incompatible directions.");
						return;
					} else if (crossProd.z < 0 && curve.arcAngle < 0) {
						trace("One curved and one straight segment not connected due to incompatible directions.");
						return;
					}
				}
								
				/* Both segments are straight. */
				else { 
					var straightAVec:Vector3D = Utility.calcVectorFromLatLngs(this._start, this._end);
					var straightBVec:Vector3D = Utility.calcVectorFromLatLngs(other._start, other._end);
				
					// Just need to check that the angle is ~0.
					angle = Utility.angleBetween(straightAVec, straightBVec);
					if (Math.abs(angle) > MAX_CONNECTION_ANGLE) {
						trace("Two straight segments not connected due to incompatible angle:", angle);
						return;
					}
				}
			}

			/* If we've made it past all the gates then add the id, but only if it's not duplicating
			 * an existing connection.
			 */ 			
			if (this._end.distanceFrom(shared) < this._start.distanceFrom(shared)) { // this -> other				
				if (this.next_ids.indexOf(other.id) == -1) {
					Undo.pushMicro(this.next_ids, this.next_ids.pop);
					this.next_ids.push(other.id);					
				}
				if (other.prev_ids.indexOf(this.id) == -1) {
					Undo.pushMicro(other.prev_ids, other.prev_ids.pop);
					other.prev_ids.push(this.id);					
				}
			} else { // other -> this
				if (other.next_ids.indexOf(this.id) == -1) {
					other.next_ids.push(this.id);
					Undo.pushMicro(other.next_ids, other.next_ids.pop);
				}
				if (this.prev_ids.indexOf(other.id) == -1) {
					this.prev_ids.push(other.id);
					Undo.pushMicro(this.prev_ids, this.prev_ids.pop);	
				}
			}
		}				

		/** Makes latlng the new end point for the TrackSegment. Connections are made, so that the connectivity is unchanged.
		 * Curved segments may not be split. If latlng is already at this segment's end, then the track is not changed,
		 * but the next segment is still returned. 
		 * 
		 * @param latlng The latlng that indicates where the TrackSegment should be split. 
		 * @param elevOffset The vertical track offset at latlng. FIXME: Remove this?
		 * @param preview Whether or not this function call was made for the live preview.
		 *
		 * @return A new TrackSegment that begins at latlng.
		 */
		public function split(latlng:LatLng, elevOffset:Number, preview_:Boolean):TrackSegment {
			var nextSeg:TrackSegment;
			var nextId:String;
			
			if (isCurved()) {
				throw new TrackError("Curved trackSegments are unsplittable");
			} else if (latlng.equals(this._end) || latlng.equals(this._start)) {
				throw new TrackError("Attempted to split at a TrackSegement's endpoint.");	
			}			
				
			// make a new id with the same forward or reverse extension as original
			var newId:String;
			newId = id.search(TrackSegment.forwardExt) != -1 ?
			        IdGenerator.getTrackSegId(TrackSegment.forwardExt) :
			        IdGenerator.getTrackSegId(TrackSegment.reverseExt);
			
			// make latlng the starting point for the new segment ...
			var newSeg:TrackSegment = new TrackSegment(newId, label, latlng.clone(), getEnd(), maxSpeed, elevOffset, _endOffset, radius, arcAngle, getCenter(), preview_);
			setEndOffset(elevOffset);
			setEnd(latlng.clone()); // ... and make latlng the endpoint for the original segment
			
			// transfer the old next_ids to the newSeg, and leave this with an empty next_ids						
			Undo.assign(newSeg, 'next_ids', newSeg.next_ids);
			newSeg.next_ids = this.next_ids;
			Undo.assign(this, 'next_ids', this.next_ids);
			this.next_ids = new Vector.<String>();
			
			
			// if 'this' was previously connected to one or more segments, then update the prev_id
			// lists for those segments to use the newSeg id.
			for each (nextId in newSeg.next_ids) {
				nextSeg = Globals.tracks.getTrackSegment(nextId);
				for (var i:int=0; i < nextSeg.prev_ids.length; ++i) {
					if (nextSeg.prev_ids[i] == this.id) {
						Undo.assignElement(nextSeg, 'prev_ids', i, nextSeg.prev_ids[i]);
						nextSeg.prev_ids[i] = newSeg.id;
					}
				}
			}
			
			// connect this to newSeg, and vice versa
			this.connect(newSeg);
			return newSeg;
		}

		/** Returns the slope of the tracks assoctiated with this overlay. Example, a 45 degree grade will return 1. */
		public function getGrade():Number {
			return (endElev - startElev) / h_length;
		}

		/** Represent the TrackData object in XML. */
		public function toXML():XML {			
			var xml:XML = <TrackSegment id={id}
			                            length={length}>
			                <Start lat={_start.lat()}
			                       lng={_start.lng()}
			                       ground_level={startGround.toFixed(1)}
			                       offset={_startOffset}
			                       elevation={startElev.toFixed(1)}/>
			                <End lat={_end.lat()}
			                     lng={_end.lng()}
			                     ground_level={endGround.toFixed(1)}
			                     offset={_endOffset}			                     
			                     elevation={endElev.toFixed(1)}/>
			                <ConnectsFrom/>
							<ConnectsTo/>
							<ParallelTo/>               
						  </TrackSegment>;

            if (label != "")      xml.@label=label
            if (!isNaN(maxSpeed)) xml.@max_speed=maxSpeed
            if (!isNaN(radius) && radius)   xml.@radius=radius
            if (arcAngle != 0)    xml.@arc_angle = arcAngle*180/Math.PI
 
			// Insert the center point if it's a curved segment			
			if (_center) {
				xml.insertChildBefore(xml.End[0], <Center lat={_center.lat()}
				                                          lng={_center.lng()} />);
			}
			
			for each (var f_id:String in prev_ids) {
				if (f_id != "") {
					var from_id:XML = <ID>{f_id}</ID>;					
					xml.ConnectsFrom.appendChild(from_id);
				}			
			}
			for each (var t_id:String in next_ids) {
				if (t_id != "") {
					var to_id:XML = <ID>{t_id}</ID>;
					xml.ConnectsTo.appendChild(to_id);
				}
			}
			
			for each (var p_id:String in parallel_ids) {
				var parallel_id:XML = <ID>{p_id}</ID>;
				xml.ParallelTo.appendChild(parallel_id); 
			}
			return xml;
		}

		/** Creates a new TrackSegment from the XML data. */ 
		public static function fromXML(xml:XML):TrackSegment {
			var otherId:String;
			trace(xml.hasOwnProperty("Control"));
			
			// add the ground elevations to the Cache
			
			ElevationService.addToCache(new LatLng(xml.Start.@lat, xml.Start.@lng),
			                            xml.Start.@ground_level)
			ElevationService.addToCache(new LatLng(xml.End.@lat, xml.End.@lng),
			                            xml.End.@ground_level)			                            
			
			var ts:TrackSegment = new TrackSegment(xml.@id,
												   "@label" in xml ? xml.@label : "",
												   new LatLng(xml.Start.@lat, xml.Start.@lng),
												   new LatLng(xml.End.@lat, xml.End.@lng),
												   "@max_speed" in xml ? xml.@max_speed : NaN,
												   xml.Start.@offset,
												   xml.End.@offset,
												   "@radius" in xml ? xml.@radius : NaN,
												   "@arc_angle" in xml ? xml.@arc_angle*Math.PI/180 : 0, // degrees to radians
												   xml.hasOwnProperty("Center") ? new LatLng(xml.Center.@lat, xml.Center.@lng) : null,
												   false);
			// ConnectsFrom
			for each (otherId in xml.ConnectsFrom.ID) {
				ts.prev_ids.push(otherId);
			}
			
			// ConnectsTo
			for each (otherId in xml.ConnectsTo.ID) {
				ts.next_ids.push(otherId);
			}
			
			// ParallelTo
			for each (otherId in xml.ParallelTo.ID) {
				trace("ParallelTo: ", otherId);
				ts.parallel_ids.push(otherId);
			}
			return ts;
		}

//		public function fetchElevation(latlng:LatLng, endpoint:String):void {
//			if (preview) {
//				return;
//			}
//			Undo.assign(this, 'length', length);
//			length = NaN; // invalidate the length
//			
//			// invalidate the elevation. The undo and invalidation is done now, to avoid complexities of undoing an asynch operation.
//			switch (endpoint) {
//				case ElevationFetcher.START:
//					Undo.assign(this, 'startGround', startGround);
//					startGround = NaN; 
//					break;
//				case ElevationFetcher.END:
//					Undo.assign(this, 'endGround', endGround);
//					endGround = NaN;
//					break;
//				case ElevationFetcher.CENTER:
//					Undo.assign(this, 'center', centerGround);
//					centerGround = NaN;
//					break;
//				default:
//					throw new Error("Unknown enpoint: " + endpoint);
//					break;
//			}			
//			
//			var elev_fetcher:ElevationFetcher = new ElevationFetcher();						
//			elev_fetcher.addEventListener(ElevationEvent.COMPLETE, onElevationComplete, false, 0, true); // weak_ref=true
//			elev_fetcher.addEventListener(ElevationEvent.ERROR, onElevationError, false, 0, true); // weak_ref=true
//			elev_fetcher.fetch(latlng, endpoint);
//		}
//		
//		/** Save the ground elevation. */
//		private function onElevationComplete(event:ElevationEvent):void {
//			switch (event.endpoint) {
//				case ElevationFetcher.START:
//					if (event.latlng.equals(this.getStart())) { // check that the elevation is for the current position
//						startGround = event.elevation;						
//					}
//					break;
//				case ElevationFetcher.END:
//					if (event.latlng.equals(this.getEnd())) {
//						endGround = event.elevation;												
//					}
//					break;
//				case ElevationFetcher.CENTER:
//					if (event.latlng.equals(this.getCenter())) {
//						centerGround = event.elevation;	
//					}					
//					break;
//				default:
//					throw new Error("Unknown case.");			
//			}	
//			
//			if ( !isNaN(startGround) && !isNaN(endGround) ) {
//				updateLength();
//			}
//		}
//		
//		private function onElevationError(event:IOErrorEvent):void {
//			trace("Segment elevation fetch failed", this.id);
//		}

		/** Changes the value of length to incorperate vertical distance.
		 * Does not take curvature of the earth into account.
		 */ 
		private function updateLength():void {
			/* bypass updating length unless both ends of the segment have elevation data */
			if (isNaN(startGround) || isNaN(endGround)) return;
			
			Undo.assign(this, 'length', this.length);
			if (!isCurved()) { // A straight segment
				var h_dist:Number = _start.distanceFrom(_end);
				var v_dist:Number = endElev - startElev;
				length = Math.sqrt(h_dist*h_dist + v_dist*v_dist);
			} else { // is curved
			    /* To account for the elevation change, project the circle onto
			     * a plane, a forming an ellipse. Calculate the perimeter of the
			     * elipse with the formula:
			     *   P = pi * sqrt( 2(a^2+b^2) - (a-b)^2/2 )
			     * where a is the radius of the long axis, and b the radius of the short axis.
			     * http://home.att.net/~numericana/answer/ellipse.htm#elliptic
			     * 
			     * Then approximate length of the arc as though it were a circle,
			     * although this will have significant error in highly eccentric
			     * ellipses.  
   				 */
   				 
   				 // TODO: get the centerLatLng's elevation, so that the slope of the
   				 // plane is known. Also, before implementing the above, check that
   				 // I'm not using length for 2D calculations anywhere.
   				 // In the mean-time, using a formula that completely disregards the
   				 // Z-axis.
   				length = Math.abs(radius * arcAngle);
   			}
   			if (isNaN(length)) {
   				throw new Error(id + " has a NaN length");
   			}
		}
		
		public function getElevation(position:Number):Number {
			var slope:Number = (endElev - startElev) / length;
			return slope * position + startElev;
		}
		
		
		/** Returns the distance from the start point to latlng in meters. Assumes that LatLng falls on the segment.*/
		public function getPosition(latlng:LatLng):Number {
			if (!isCurved()) { // straight segment
				return _start.distanceFrom(latlng);
			} else { // curved segment
				var cs:Vector3D = Utility.calcVectorFromLatLngs(_center, _start);
				var cv:Vector3D = Utility.calcVectorFromLatLngs(_center, latlng);				
				return Utility.angleBetween(cs, cv) * radius;
			} 
		}

		/** Returns the latlng of position, where position is measured as meters along the
		 * segment, disregarding z axis distance */
		public function getLatLng(position:Number):LatLng {
			var vec:Vector3D;
			if (this.isCurved()) {
				var angle:Number = position/this.h_length * this.arcAngle;
				vec = Utility.calcVectorFromLatLngs(_center, _start);
				var cosAngle:Number = Math.cos(angle);
				var sinAngle:Number = Math.sin(angle);
				// Rotate the vector
				vec.x = vec.x*cosAngle - vec.y*sinAngle;
				vec.y = vec.x*sinAngle + vec.y*cosAngle;
			} else { // straight segment
				vec = Utility.calcVectorFromLatLngs(_start, _end);
				vec.scaleBy(position/vec.length);
			}			
			return Utility.calcLatLngFromVector(_start, vec);
		}
		
		public function get midpoint():LatLng {
			var vec:Vector3D = Utility.calcVectorFromLatLngs(_start, _end);
			vec.scaleBy(0.5);
			return Utility.calcLatLngFromVector(_start, vec);
		}
		

	}
}