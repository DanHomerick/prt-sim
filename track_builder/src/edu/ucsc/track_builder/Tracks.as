package edu.ucsc.track_builder
{		
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	import com.google.maps.MapMouseEvent;
	
	import de.polygonal.ds.Heap;
	
	import flash.events.MouseEvent;
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;
	import flash.utils.Dictionary;
	
	import mx.controls.Alert;

	/** Acts as a factory for TrackSegments and TrackOverlays.
	 * Keeps references to both of them.
	 * Contains some handlers for GUI events relating to tracks. */	
	public final class Tracks
	{
		/* Values that are controlled by the GUI */
		public var label:String = "";		
		[Bindable] public var maxSpeed:Number;
		[Bindable] public var offset:Number = 0; 
		public var bidirectional:Boolean;
		[Bindable] public var radius:Number;
		public var minOffset:Number;
		public var maxOffset:Number;			

		private const INCOMING:String = "INCOMING"
		private const OUTGOING:String = "OUTGOING"

		// holds all TrackSegment objects in an associative array. Uses id as key, and a TrackSegment instance as the value. 
		public var segments:Dictionary;

		// holds all TrackOverlay objects.		
		public var overlays:Vector.<TrackOverlay>; // displays the track segments on the map


		/* Constructor */
		public function Tracks() {
			 segments = new Dictionary();
			 overlays = new Vector.<TrackOverlay>();			
		}

		public function onMapClick(event:MapMouseEvent):void {
			Undo.undo(Undo.PREVIEW); // destroy the 'live preview' track segment
			Undo.startCommand(Undo.USER);			
			if (!Globals.haveClicked) {
				Undo.assign(Globals, 'haveClicked', Globals.haveClicked);
				Globals.haveClicked = true;
				Undo.endCommand();
				return;
			}								
			
			/* Force the markers to snap to the line (if attached to one) */
//			trace(Globals.originMarker.overlay);
			Globals.originMarker.snapTo();
			Globals.destMarker.snapTo();
			
			try {				
				var trail:Vector.<TrackOverlay> = makeTrail(Globals.originMarker.getTrackOverlays(),
						  									Globals.destMarker.getTrackOverlays(),
						  									Globals.originMarker.getLatLng(),
						  									Globals.destMarker.getLatLng(),
						  									false); // not a preview
				// Add listeners that enable the snap-to behavior. 
				for each (var overlay:TrackOverlay in trail) {
					if (!overlay.isCurved()) { // for straight segments only
						overlay.addEventListener(MouseEvent.ROLL_OVER, overlay.onRollOver, false, 0, true); // weak_ref=true
					}
				}
				
				// move the originMarker							
				Globals.originMarker.overlay = trail[trail.length-1];
				Globals.originMarker.snapTo(trail[trail.length-1].getEnd());
				
			
				Undo.assign(Globals, 'dirty', Globals.dirty);
				Globals.dirty = true;
				
				Undo.endCommand();
			} catch (err:TrackError) { // error thrown from makeTrail
				Undo.endCommand();
				Undo.undo(Undo.USER); // undo any side effects from before the error.
				Alert.show(err.message); // explain to user what went wrong
			}			
		} 

		public function onMapMouseMove(event:MapMouseEvent):void {
//			trace("Tracks.onMapMouseMove", event.currentTarget);
			if (event.ctrlKey) { // moving the originMarker
				Undo.undo(Undo.PREVIEW); // get rid of the old 'live preview', if one exists
			} else { // normal case
				makePreview();
			}
		} 				

		/** Makes a temporary trail showing where the track will go if the user clicks. */
		public function makePreview():void {			
			Undo.undo(Undo.PREVIEW); // get rid of the old 'live preview', if one exists			
			if (Globals.haveClicked && !Globals.originMarker.getLatLng().equals(Globals.destMarker.getLatLng())) {				
				Undo.startCommand(Undo.PREVIEW);
				try {
					var originOverlays:Vector.<TrackOverlay>;
					var destOverlays:Vector.<TrackOverlay>;
					
					/* Want to avoid the case that a marker is over some Overlay, but is not
					 * affiliated to it. This can be caused by the user placing a marker near
					 * a trackSegment, then zooming out.
					 */			
					if (Globals.originMarker.overlay == null) {
						originOverlays = new Vector.<TrackOverlay>();
					} else {
						originOverlays = Globals.originMarker.getTrackOverlays(); 
					}
					
					if (Globals.destMarker.overlay == null) {
						destOverlays = new Vector.<TrackOverlay>();
					} else {
						destOverlays = Globals.destMarker.getTrackOverlays();
					}
					makeTrail(originOverlays,
							  destOverlays,
							  Globals.originMarker.getLatLng(),
							  Globals.destMarker.getLatLng(),
							  true);
					Undo.endCommand();
				} catch (err:TrackError) {
					// just do minimal cleanup. The user just won't see a preview in this case.
					Undo.endCommand();
					Undo.undo(Undo.PREVIEW); // undo any side effects from before the error					
				}
			}
		}

		/** Makes a trail from originMarker to destMarker, utilizing curved segments as appropriate,
		 * and connecting the segments as appropriate. Moves the markers. Returns the overlays that
		 * it creates, though not in order.
		 */
		public function makeTrail(originOverlays:Vector.<TrackOverlay>,
		                          destOverlays:Vector.<TrackOverlay>,
		                          originLatLng:LatLng,
		                          destLatLng:LatLng,
		                          preview:Boolean):Vector.<TrackOverlay> {		                          	
			
			var curveOverlays:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();
			var overlay:TrackOverlay; // for iterating through Vectors.

			// Create outgoing curved segment
			if (originOverlays.length >= 1) {
				// TEMP: bidirectional should not be blindly passed through
				curveOverlays.push(makeCurve(originOverlays, originLatLng.clone(), destLatLng.clone(), OUTGOING, bidirectional, preview));
				originLatLng = curveOverlays[0].getEnd();				
			}
			
			// reserve any ids needed for the straight section connecting the two curves, so that the ids will be in sequential order
			var straightIds:Vector.<String> = new Vector.<String>();
			straightIds.push(IdGenerator.getTrackSegId(TrackSegment.forwardExt));
			if (bidirectional) {
				straightIds.push(IdGenerator.getTrackSegId(TrackSegment.reverseExt));
			} 
			
			// Create incoming curved segment
			if (destOverlays.length >= 1) {
				// TEMP: bidirectional should not be blindly passed through
				curveOverlays.push(makeCurve(destOverlays, originLatLng.clone(), destLatLng.clone(), INCOMING, bidirectional, preview));
				destLatLng = curveOverlays[curveOverlays.length-1].getStart();
			}

			// If any curved overlays were required, they've been created, and the origin/dest moved appropriately.
			// Now add a straight seg/overlay that connects the current origin/dest.
			var straightSegs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			straightSegs.push(makeStraightSeg(originLatLng.clone(), destLatLng.clone(), preview, straightIds[0]));
			if (bidirectional) {
				straightSegs.push(straightSegs[0].clone(true, straightIds[1]));
			}
			var straightOverlay:TrackOverlay = new TrackOverlay(straightSegs);
			
			// connect the straight seg/overlay
			for each (overlay in curveOverlays) {
				straightOverlay.connect(overlay);
			}				

			// collect all the overlays, and sort them in connected order
			var returnOverlays:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();
			for each (overlay in curveOverlays) {
				returnOverlays.push(overlay);				
			}				
			returnOverlays.push(straightOverlay);
			returnOverlays = returnOverlays.sort(TrackOverlay.compareById); 
			
			// validate that the trail is properly connected within itself (possible todo: fix the causes of any errors)
			if (returnOverlays.length > 1) {
				for (var i:int=0; i < returnOverlays.length; i++) {
					overlay = returnOverlays[i];
					if ( (i > 0 && overlay.isStartExposed()) || (i < returnOverlays.length-1 && overlay.isEndExposed()) ) {
						throw new TrackError("Can not create a correct connection for this placement.");
					}
				}
			}
			return returnOverlays;
        }

		/** Creates a single curve segment and wraps it in an overlay.*/
		public function makeCurve(targetOverlays:Vector.<TrackOverlay>,
		                          srcLatLng:LatLng,
		                          destLatLng:LatLng,
		                          type:String, // OUTGOING or INCOMING. Affects the sense of 'start' and 'end' of an overlay.
		                          bidir:Boolean, // whether the curve should be bidirectional
		                          preview:Boolean
		                         ):TrackOverlay {
		    // input validation
		    if (type != OUTGOING && type != INCOMING) {
		    	throw new Error("Unknown type: " + type);
		    }
		    
		    var anchor1:LatLng;
		    var control:LatLng;
		    var anchor2:LatLng;
		    
		    var curveSegs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			var curveOverlay:TrackOverlay;
			var newOverlay:TrackOverlay; // the result of a split
			var srcVec:Vector3D;			
			
			var cut:Boolean = false;

			if (targetOverlays.length == 1) {				
				var targetOverlay:TrackOverlay = targetOverlays[0];
				
				if (type == OUTGOING) {
					anchor1 = targetOverlay.getStart();
					control = srcLatLng;
					anchor2 = destLatLng;
				} else { // INCOMING
					anchor1 = srcLatLng;
					control = destLatLng;
					anchor2 = targetOverlay.getEnd();
				}
				
				if (canCut(anchor1, control, anchor2, radius)) {
					cut = true;
				}
			}

			// Cut the corner						
			if (cut) {				
				curveSegs.push(makeCurveSeg_Cut(anchor1, control, anchor2, this.radius, targetOverlay.h_length, preview));
				if (bidir) {
					curveSegs.push(curveSegs[0].clone(true)); // flipped copy
				}
				// Trim the origin overlay, if we're dealing with an exposed endpoint 
				if (type == OUTGOING && control.equals(targetOverlay.getEnd()) && targetOverlay.isEndExposed()) {			
					if (!preview) {
						targetOverlay.setEnd(curveSegs[0].getStart());
					}
				} else if (type == INCOMING && control.equals(targetOverlay.getStart()) && targetOverlay.isStartExposed()) {
					if (!preview) {
						targetOverlay.setStart(curveSegs[0].getEnd());
					}
				}
				// Otherwise, we split the overlay at the connection point.
				else {
					if (type == OUTGOING && !preview) {
    					newOverlay = targetOverlay.split(curveSegs[0].getStart(), offset, preview);
  					} else if (type == INCOMING && !preview) { 
  						newOverlay = targetOverlay.split(curveSegs[0].getEnd(), offset, preview);
  					} // else is just a preview, so don't modify existing track
				}
				
				curveOverlay = new TrackOverlay(curveSegs);
				
				// connect the old overlay with the curve
				if (newOverlay != null) {
					curveOverlay.connect(newOverlay);						
				}
				targetOverlay.connect(curveOverlay);
	            
	            
            // Don't cut the corner. Handles cases where there is more than one targetOverlay, or not enough room to cut.
   			} else  {
   				// filter the overlays to try to find a straight one.
				function curveFilter(item:TrackOverlay, index:int, vector:Vector.<TrackOverlay>):Boolean {return !item.isCurved()}					
				var straightTargetOverlays:Vector.<TrackOverlay> = targetOverlays.filter(curveFilter);
   				if (straightTargetOverlays.length >= 1) {
   					targetOverlay = straightTargetOverlays[0];
   				} else {
   					throw new TrackError("Must have a straight overlay..."); // for now? Could make a srcVec if at an endpoint of a curve.
   				}
   				
   				var reverse:Boolean;
   				var overlay:TrackOverlay;
   				if (type == OUTGOING) {
   					srcVec = Utility.calcVectorFromLatLngs(targetOverlay.getStart(), targetOverlay.getEnd());
   					reverse = false;
   					curveSegs.push(makeCurveSeg_NoCut(srcLatLng.clone(), srcVec, destLatLng.clone(), this.radius, reverse, preview));
   					if (bidir) {
   						curveSegs.push(curveSegs[curveSegs.length-1].clone(true)); // make a flipped copy
   					}
	        		curveOverlay = new TrackOverlay(curveSegs);
	        			        			         		
	    			// if the src was exactly at targetOverlay.start or targetOverlay.end, we're good 
	    			if (srcLatLng.equals(targetOverlay.getStart()) || srcLatLng.equals(targetOverlay.getEnd())) {
	    				for each (overlay in targetOverlays) {
	    					overlay.connect(curveOverlay); // won't do anything if srcLatLng == start
	    				}
	    			} else if (!preview) { // otherwise, split the overlay
	    				newOverlay = targetOverlay.split(srcLatLng.clone(), offset, preview);
	    				curveOverlay.connect(newOverlay);
	    				for each (overlay in targetOverlays) {
	    					curveOverlay.connect(overlay);
	    				}         				
	    			} // else it's just a preview, no need to split or connect
   					
   				} else { // type == INCOMING
   					srcVec = Utility.calcVectorFromLatLngs(targetOverlay.getEnd(), targetOverlay.getStart());
   					reverse = true;
   					curveSegs.push(makeCurveSeg_NoCut(destLatLng.clone(), srcVec, srcLatLng.clone(), this.radius, reverse, preview));
   					if (bidir) {
   						curveSegs.push(curveSegs[curveSegs.length-1].clone(true)); // make a flipped copy
   					}
   					curveOverlay = new TrackOverlay(curveSegs);

   					// if the dest was exactly at targetOverlay.start or targetOverlay.end, we're good
	    			if (destLatLng.equals(targetOverlay.getStart()) || destLatLng.equals(targetOverlay.getEnd())) {
	    				for each (overlay in targetOverlays) {
	    					overlay.connect(curveOverlay);
	    				}
	    			} else if (!preview) { // otherwise, split the overlay
	    				newOverlay = targetOverlay.split(destLatLng, offset, preview);
	    				curveOverlay.connect(newOverlay);
	    				for each (overlay in targetOverlays) {
	    					curveOverlay.connect(overlay);
	    				}
	    			} // else it's just a preview, no need to split or connect           					
       			}
        	}
   			return curveOverlay;		
		}
		
		public function makeStraightSeg(origin:LatLng, dest:LatLng, preview:Boolean, id:String=null):TrackSegment {
			if (id == null) {
				id = IdGenerator.getTrackSegId(TrackSegment.forwardExt);
			}
			return new TrackSegment(id, label, origin, dest, maxSpeed, offset, offset, 0, 0, null, preview);
		}

		public function canCut(a:LatLng, b:LatLng, c:LatLng, radius:Number):Boolean {
			var ba:Vector3D = Utility.calcVectorFromLatLngs(b, a);
			var bc:Vector3D = Utility.calcVectorFromLatLngs(b, c);
			var abcAngle:Number = Vector3D.angleBetween(ba, bc);
			var dist:Number = radius / Math.tan(abcAngle/2);
			return dist < ba.length && dist < bc.length;
		}
		
		/** Does not alter the parameters. */
		public function makeCurveSeg_Cut(anchor1:LatLng, control:LatLng, anchor2:LatLng, radius:Number, maxLength:Number, preview:Boolean):TrackSegment {
			// alias the inputs to short names, and make copies
			var a:LatLng = anchor1.clone();
			var b:LatLng = control.clone();
			var c:LatLng = anchor2.clone();
			
			var ba:Vector3D = Utility.calcVectorFromLatLngs(b, a); // in meters
			var bc:Vector3D = Utility.calcVectorFromLatLngs(b, c);
			
			var abcAngle:Number = Utility.signedAngleBetween(ba, bc);
			
   			// D is the center of the circle. Since AB and BC are tangent to the circle, angles DAB and DCB are 90 degrees.
   			// Consider quadrilateral ABCD. Given angle ABC, and angles DAB and DCB, we know angle ADC.
   			// var adcAngle:Number = 2*Math.PI - Math.PI/2 - Math.PI/2 - abcAngle   			   			   			
   			var arcAngle:Number = Math.PI - Math.abs(abcAngle);   			
   			
   			// Furthermore the direction of rotation is flipped between abcAngle and arcAngle, so the sign flips.
   			arcAngle *= (abcAngle > 0) ? -1 : 1
			
			// find the distance from the control point to either of the tangent points.
			var dist:Number = radius / Math.tan(Math.abs(abcAngle/2)); // in meters	
			if (dist > maxLength) {
				throw new TrackError("Requires too deep of cut.");
			}		
			
			// rescale vectors to the tangent points
			ba.scaleBy(dist/ba.length);
			bc.scaleBy(dist/bc.length);

			// make latlng's for the tangent points
			var abNewLatLng:LatLng = Utility.calcLatLngFromVector(b, ba);			
			var bcNewLatLng:LatLng = Utility.calcLatLngFromVector(b, bc);
			
			// Find the vector perpendicular to ba that points in same general direction as bc.
			var tmp:Vector3D = ba.crossProduct(bc);
			tmp.negate();			
			var ad:Vector3D = ba.crossProduct(tmp);
			ad.scaleBy(radius/ad.length);
			var bd:Vector3D = ba.add(ad);
						
			var centerLatLng:LatLng = Utility.calcLatLngFromVector(b, bd);
			var id:String = IdGenerator.getTrackSegId(TrackSegment.forwardExt);
			
			var start:LatLng;
			var end:LatLng;
			start = abNewLatLng;
			end = bcNewLatLng;
			return new TrackSegment(id, label, start, end, maxSpeed, offset, offset, radius, arcAngle, centerLatLng, preview);
		}
		
		/** Returns a single curved Tracksegment which originates at anchor, and is tangent to originVec.
		 *  The curved TrackSegment will terminate such that a line segment from it's endpoint to dest will
		 *  be tangent to it.
		 *  If reverse is True, then direction of the line segment is reversed, but the endpoints remain the same.
		 */
		public function makeCurveSeg_NoCut(anchor:LatLng, originVec:Vector3D, dest:LatLng, radius:Number, reverse:Boolean, preview:Boolean):TrackSegment {				
			originVec.normalize();			
			var destVec:Vector3D = Utility.calcVectorFromLatLngs(anchor, dest);
			var destAngle:Number = Utility.signedAngleBetween(originVec, destVec);			
			
			/* Find where the center of the circle should be. */			
			var centerVec:Vector3D;						
			var center:LatLng;
			var dir:String;
			if (destAngle > 0) { // CCW
				// assume that the dest does not lie within the circle
				dir = 'CCW'
				centerVec = new Vector3D(-originVec.y, originVec.x); // CCW perpendicular
				centerVec.scaleBy(radius/centerVec.length);
				center = Utility.calcLatLngFromVector(anchor, centerVec);
				
				// if dest IS within the circle, loop around from the other direction
				if (center.distanceFrom(dest) < radius) {
					dir = 'CW'
					centerVec = new Vector3D(originVec.y, -originVec.x); // CW perpendicular
					centerVec.scaleBy(radius/centerVec.length);
					center = Utility.calcLatLngFromVector(anchor, centerVec);
				}
				
			} else { // destAngle < 0, CW
				// assume that the dest does not lie within the circle
				dir = 'CW'
				centerVec = new Vector3D(originVec.y, -originVec.x); // CW perpendicular
				centerVec.scaleBy(radius/centerVec.length);
				center = Utility.calcLatLngFromVector(anchor, centerVec);
				
				// if dest IS within the circle, loop around from the other direction
				if (center.distanceFrom(dest) < radius) {
					dir = 'CCW'
					centerVec = new Vector3D(-originVec.y, originVec.x); // CCW perpendicular
					centerVec.scaleBy(radius/centerVec.length);
					center = Utility.calcLatLngFromVector(anchor, centerVec);
				}
			}			
						
			//var centerDestVec:Vector3D = Utility.calcVectorFromLatLngs(center, dest);
			var centerDestVec:Vector3D = destVec.subtract(centerVec);			
			
			var rotAxis:Vector3D = Vector3D.Z_AXIS.clone();
			if (dir == 'CCW') {			 
				rotAxis.negate();
			}
			
			var alpha:Number = Math.asin(radius / centerDestVec.length); // The angle formed by tangent|dest|center
			var epsilon:Number = Math.PI/2 - alpha; // the angle formed by tangent|center|dest. Use the fact that it's a right triangle.
			
			// rotate centerDestVec by epsilon to get the tangent point.
			var matrix:Matrix3D = new Matrix3D();
			matrix.appendRotation(epsilon * (180/Math.PI), rotAxis); // uses degrees instead of radians
			var centerTangentVec:Vector3D = matrix.transformVector(centerDestVec);
			centerTangentVec.scaleBy(radius/centerTangentVec.length); // rescale to have length 'radius'
			var tangent:LatLng = Utility.calcLatLngFromVector(center, centerTangentVec);
			// find arcAngle
			var centerAnchorVec:Vector3D = centerVec.clone();
			centerAnchorVec.negate();
			var arcAngle:Number = Utility.signedAngleBetween(centerAnchorVec, centerTangentVec);
			// check if arcAngle 'looped around' due to an angle greater than 180 degrees
			if (arcAngle > 0 && dir == 'CW') {
				arcAngle = -2*Math.PI + arcAngle;
			} else if (arcAngle < 0 && dir == 'CCW') {
				arcAngle = 2*Math.PI + arcAngle;
			}			
			
			var start:LatLng;
			var end:LatLng;
			if (!reverse) {
				start = anchor;
				end = tangent;
			} else {
				start = tangent;
				end = anchor;
				arcAngle *= -1;
			}

			return new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
										label,
										start,
										end,
										maxSpeed,
										offset,
										offset,
										radius,
										arcAngle,
										center,
										preview);
		}
		
		/** Removes a segment from the global store. Does not remove segment from any overlays! */
        public function removeTrackSegment(seg:TrackSegment):void {
        	var otherId:String;
        	var otherSeg:TrackSegment;
        	
        	function removeTSID (item:String, index:int, vector:Vector.<String>):Boolean {return item != seg.id};
        	
        	// remove all connections to the seg
        	for each (otherId in seg.next_ids) {
        		otherSeg = getTrackSegment(otherId);
        		Undo.assign(otherSeg, 'prev_ids', otherSeg.prev_ids);
        		otherSeg.prev_ids = otherSeg.prev_ids.filter(removeTSID);
        	}
        	for each (otherId in seg.prev_ids) {
				otherSeg = getTrackSegment(otherId);
				Undo.assign(otherSeg, 'next_ids', otherSeg.next_ids); 
				otherSeg.next_ids = otherSeg.next_ids.filter(removeTSID);
        	}

			// remove segment from Tracks.     	
			Undo.pushMicro(this, function():void {segments[seg.id] = seg;});
			delete segments[seg.id];
        }

		/** Returns a vector containing all TrackSegments, sorted by id */
		public function getSortedTrackSegments():Vector.<TrackSegment> {
			var vec:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			for each (var seg:TrackSegment in segments) {
				vec.push(seg);
			}
			
			var cmp:Function = function(a:TrackSegment, b:TrackSegment):Number {
				var aNum:Number = Number(a.id.slice(0, a.id.indexOf("_")));
				var bNum:Number = Number(b.id.slice(0, b.id.indexOf("_")));
				return aNum - bNum;
			} 
			vec.sort(cmp);
			return vec;
		}
		
		
		/** Removes an overlay, and all of the segments it contains. */ 
		public function removeTrackOverlay(overlay:TrackOverlay):void {
			// remove from the display
			if (overlay.isCurved()) {
				Undo.pushMicro(Globals.curvedTrackPane, Globals.curvedTrackPane.addOverlay, overlay);
				Globals.curvedTrackPane.removeOverlay(overlay);
			} else {
				Undo.pushMicro(Globals.straightTrackPane, Globals.straightTrackPane.addOverlay, overlay);
				Globals.straightTrackPane.removeOverlay(overlay); 
			}
			
			// remove vehicles on the track 
			for each (var vo:VehicleOverlay in Globals.vehicles.overlays) {
				if (vo.getTrackOverlay() == overlay) {
					// Surprised that I can alter a vector as I iterate through it, but seems to work fine.
					Globals.vehicles.removeVehicleOverlay(vo);
				}
			}
			
			// remove each segment
			for each (var ts:TrackSegment in overlay.segments) {				
				removeTrackSegment(ts);
			}
			
			// remove overlay from Tracks.
			function removeOL (item:TrackOverlay, index:int, vector:Vector.<TrackOverlay>):Boolean {return item != overlay;};
			Undo.assign(this, 'overlays', overlays);
			overlays = overlays.filter(removeOL);
		}

		/** Attempts to level out the track. Returns the steepest grade remaining.
		 * @throws Error If unable to reduce all grades to threshold. */
		public function autoLevel(minOffset:Number, maxOffset:Number, minThreshold:Number, maxThreshold:Number, forceLevelCurve:Boolean, stepSize:Number=0.1):void {			
			var overlay:TrackOverlay;
			var seg:TrackSegment;
			var rev_seg:TrackSegment; 
			
			if (overlays.length == 0) {
				return; // no work to do
			}
			
			/* Setup */
			// Change all segments to the minimum offset
			for each (seg in segments) {
				seg.setStartOffset(minOffset);
				seg.setEndOffset(minOffset);
			}
			
			// if forcing curve segments to be level, level them
			if (forceLevelCurve) {
				for each (seg in segments) {
					if (seg.isCurved()) {
						seg.makeLevel(maxOffset); // may throw error
					} 
				}
			}
			
			// place track overlays in a heap, sorted by gradient
			function cmpFunc(a:TrackOverlay, b:TrackOverlay):Number {
				return Math.abs(a.getGrade()) - Math.abs(b.getGrade());
			}
			var heap:Heap = new Heap(Globals.tracks.overlays.length, cmpFunc);
			
			for each (overlay in overlays) {
				heap.enqueue(overlay);
			}
			
			/* while there exists an overlay with a grade > threshold, and whose lower end is not yet
			 * at maxOffset, increase the offset of the lower end (thus raising that end, and reducing
			 * the grade).
			 */ 
			overlay = heap.dequeue();
			var grade:Number = overlay.getGrade();
			var result:Boolean = true;
			while (Math.abs(grade) > minThreshold) {
				trace("Heap_Grade: ", grade);
				var prevId:String, nextId:String;
				var nextSeg:TrackSegment, prevSeg:TrackSegment;
				seg = overlay.segments[0];
				if (grade > 0) { // uphill
					if (seg.getStartOffset() <= maxOffset - stepSize) { // low end can still be raised 
						seg.setStartOffset(seg.getStartOffset() + stepSize); // setting the offset automatically adjusts the offset of connected segments
						
						// enforce keeping the curves level, if enabled
						if (forceLevelCurve) {
							for (prevId in seg.prev_ids) {
								prevSeg = getTrackSegment(prevId);
								// keep walking along previous segs so long as they are all curved. Level them.
								while (prevSeg.isCurved()) {
									prevSeg.makeLevel(maxOffset);
									if (prevSeg.prev_ids.length > 0) {
										prevSeg = getTrackSegment(prevSeg.prev_ids[0]);
									} else {
										break; // break the while
									}
								}
							}
						}
						
						if (overlay.bidirectional) {
							rev_seg = overlay.segments[1];
							rev_seg.setEndOffset(rev_seg.getEndOffset() + stepSize);
						}
						heap.enqueue(overlay); // put it back in the heap
					} else { // this overlay has been leveled as much as possible.
						result = false; // we've failed to meet the threshold value for all tracks
						// if the adjacent segments slope in the same direction, attempting to level them
						// will make the slope on this already steep section worse. So we remove both this overlay, and
						// any connected overlays from the algorithm.
						for each (nextId in seg.next_ids) {
							nextSeg = getTrackSegment(nextId);
							if (nextSeg.getGrade() > 0) { // also uphill
								heap.remove(getTrackOverlay(nextId));							
							}
						}
						if (overlay.bidirectional) {
							seg = overlay.segments[1];
							for each (prevId in seg.prev_ids) {
								prevSeg = getTrackSegment(prevId);
								if (prevSeg.getGrade() < 0) { // prev reverse seg is downhill (ie, also uphill ...)
									heap.remove(getTrackOverlay(prevId));
								}
							}
						}						 
					}
				} else { // grade <= 0. Downhill. Similar to uphill.
					if (seg.getEndOffset() <= maxOffset - stepSize) {
						seg.setEndOffset(seg.getEndOffset() + stepSize);
						
						// enforce keeping the curves level, if enabled
						if (forceLevelCurve) {
							for (nextId in seg.next_ids) {
								nextSeg = getTrackSegment(nextId);
								// keep walking along previous segs so long as they are all curved. Level them.
								while (nextSeg.isCurved()) {
									nextSeg.makeLevel(maxOffset);
									if (nextSeg.next_ids.length > 0) {
										nextSeg = getTrackSegment(nextSeg.next_ids[0]);
									} else {
										break; // break the while
									}
								}
							}
						}
						
						if (overlay.bidirectional) {
							rev_seg = overlay.segments[1];
							rev_seg.setEndOffset(rev_seg.getStartOffset() + stepSize);
						}
						heap.enqueue(overlay);
					} else {
						result = false;
						for each (prevId in seg.prev_ids) {
							prevSeg = getTrackSegment(prevId);
							if (prevSeg.getGrade() < 0) { // also downhill
								heap.remove(getTrackOverlay(prevId));
							}
						}
						if (overlay.bidirectional) {
							seg = overlay.segments[1];
							for each (nextId in seg.next_ids) {
								nextSeg = getTrackSegment(nextId);
								if (nextSeg.getGrade() > 0) {
									heap.remove(getTrackOverlay(nextId));
								}
							}
						}
					}
				}
				
				overlay = heap.dequeue();
				if (overlay == null) {
					break; // terminate
				} else {
					grade = overlay.getGrade();		
				}
			}
		}
		
		
		public function toDataXML():XML {
			var xml:XML = <TrackSegments></TrackSegments>
			var sortedSegs:Vector.<TrackSegment> = getSortedTrackSegments();
			for each (var ts:TrackSegment in sortedSegs) {				
				xml.appendChild(ts.toXML());
			}
			return xml;
		}

		public function fromDataXML(xml:XMLList):void {
			trace("xml:", xml.toXMLString());
			var used:Object = new Object; // a dict to track which TrackSegments already have an overlay
			
			// Create all TrackSegments
			for each (var trackSegXml:XML in xml) {
				var ts:TrackSegment = TrackSegment.fromXML(trackSegXml); // adds segment to the global store	
			}
			
			// Create overlays
			for each (var seg:TrackSegment in segments) {
				var create:Boolean = true;
				var parallels:Vector.<TrackSegment> = new Vector.<TrackSegment>();
				for each (var otherId:String in seg.parallel_ids) {
					if (!used[otherId]) {
						parallels.push(segments[otherId]);
						used[otherId] = otherId;
					} else {
						create = false;
						break;
					}
				}
				
				// create overlay if one doesn't already exist for this set of segments
				if (create) {
					parallels.push(seg); 
					new TrackOverlay(parallels); // automatically added to the global store
				}
			}
		}

		/** Generate xml from current preferences */
		public function toPrefsXML():XML {
			var xml:XML = <Tracks max_speed={maxSpeed}
			                      offset={offset}
			                      bidirectional={bidirectional}
			                      radius={radius}
			                      min_offset={minOffset}
			                      max_offset={maxOffset} />
			return xml;
		}

		/** Generate xml from hard-coded default preferences */
		public function toDefaultPrefsXML():XML {
			var xml:XML = <Tracks
							max_speed="15"
							offset="3"
							bidirectional="false"
							radius="20"
							minOffset="1"
							maxOffset="9"
						  />
			return xml;
		}
		
		public function fromPrefsXML(xml:XMLList):void {
			maxSpeed = xml.@max_speed;
			offset = xml.@offset;
			bidirectional = xml.@bidirectional == 'false' || xml.@bidirectional == '0' ? false : true;
			radius=xml.@radius;
			minOffset=xml.@minOffset;
			maxOffset=xml.@maxOffset;
		}

			/** Total lane km of track */
		public function get totalLength():Number {
			var totalLength:Number = 0;
			for each (var seg:TrackSegment in segments) {
				totalLength += seg.length;
			}
			return totalLength;
		}

		/** Total linear right of way required. Assumes that additional lanes do not require additional ROW */
		public function get totalROW():Number {
			var totalROW:Number = 0;
			for each (var overlay:TrackOverlay in overlays) {
				totalROW += overlay.segments[0].length;
			}
			return totalROW;
		}
		
		public function get totalElevationChange():Number {
			var totalElev:Number = 0;
			for each (var seg:TrackSegment in segments) {
				totalElev += Math.abs(seg.endElev - seg.startElev);
			}
			return totalElev;
		}
		
		/** Steepest slope in the track network. A track with a 45 degree incline has a slope of 1.*/
		public function getMaxGrade():Number {
			var maxGrade:Number = 0;
			var grade:Number;
			for each (var overlay:TrackOverlay in overlays) {
				grade = overlay.getGrade();
				if (Math.abs(grade) > Math.abs(maxGrade)) {
					maxGrade = grade;
				}
			}
			return maxGrade;			
		}		
		
		/** Clears stored data. Intended to be called prior to loading an xml file. */
		public function reinitialize():void {						
			// remove the overlays from the display
			Globals.straightTrackPane.clear();
			Globals.curvedTrackPane.clear();

			segments = new Dictionary();		
			overlays = new Vector.<TrackOverlay>();
		}

		/** Retrieve a TrackSegment by it's ID. */
		public function getTrackSegment(id:String):TrackSegment {			
			return segments[id];
		}
		
		/** Retrieve the TrackOverlay that contains the Tracksegment matching id. */
		public function getTrackOverlay(id:String):TrackOverlay {
			var match:TrackOverlay;
			outerLoop: for each (var overlay:TrackOverlay in overlays) {
				for each (var ts:TrackSegment in overlay.segments) {
					if (ts.id == id) {
						match = overlay;
						break outerLoop;
					}
				}
			}			
			return match;			
		}

		public function getLatLngBounds():LatLngBounds {
			var bounds:LatLngBounds = new LatLngBounds();
			for each (var overlay:TrackOverlay in overlays) {
				bounds.union(overlay.getLatLngBounds());
			}
			return bounds;
		}
		
		public function validate():Array {
			var errors:Array = new Array();
			var track:TrackSegment;					
			/* TODO: Push some of the checks down to the Track class */	
			/* Do checks on individual segments */
			for each (track in this.segments) {
				if (isNaN(track.startElev)) {
					errors.push("BUG:" + track.id +  " start elevation is invalid.");					
				}
						
				if (isNaN(track.endElev)) {
					errors.push("BUG:" + track.id +  " end elevation is invalid.");
				}
				
				if (track.prev_ids.length == 0) {
					errors.push("The start point for " + track.id + " has no connections."); 
				}
				if (track.next_ids.length == 0) {
					errors.push("The end point for " + track.id + " has no connections.");
				}
				if (isNaN(track.length)) {
					errors.push("BUG: " + track.id + "has NaN for it's length.");
				}
			} 

			/* Check that the connections are mutual, and that elevations match on connected segments */
			var next:TrackSegment;
			var prev:TrackSegment;			
			for each (track in Globals.tracks.segments) {				
				for each (var nextId:String in track.next_ids) {
					next = Globals.tracks.getTrackSegment(nextId);
					if (next == null) {
						errors.push("BUG:" + track.id + " has a 'next' connection to " + nextId + ", but that segment does not exist.");
						continue;
					}				
					if (track.endElev.toFixed(1) != next.startElev.toFixed(1)) {
						errors.push("BUG:" + track.id + " end elevation: " + track.endElev.toFixed(1) + " does not match " + nextId + " start elevation: " + next.startElev.toFixed(1));
						
					}
					if (next.prev_ids.indexOf(track.id) == -1) {
						errors.push("BUG:" + track.id + " has a 'next' connection to " + nextId + " but the corresponding 'prev' id is missing.");
					}
				}
				
				for each (var prevId:String in track.prev_ids) {
					prev = Globals.tracks.getTrackSegment(prevId);
					if (prev == null) {
						errors.push("BUG:" + track.id + " has a 'prev' connection to " + prevId + ", but that segment does not exist.");
						continue;
					}
					if (track.startElev.toFixed(1) != prev.endElev.toFixed(1)) {
						errors.push("BUG:" + track.id + " start elevation: " + track.startElev.toFixed(1) + " does not match " + prevId + " end elevation: " + prev.endElev.toFixed(1));
					}
					if (prev.next_ids.indexOf(track.id) == -1) {
						errors.push("BUG:" + track.id + " has a 'prev' connection to " + prevId + " but the corresponding 'next' id is missing.");
					}
					
				}				
			}
			return errors;
		}
					
	}
}

/* Sample KML text for a single LineString */
//<?xml version="1.0" encoding="UTF-8"?>
//<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
//<Document>
//	<name>KmlFile</name>
//	<Style id="style3">
//		<LineStyle>
//			<color>73ff0000</color>
//			<width>5</width>
//		</LineStyle>
//	</Style>
//	<Placemark>
//		<name>boardwalk loop</name>
//		<styleUrl>#style3</styleUrl>
//		<LineString>
//			<tessellate>1</tessellate>
//			<coordinates>
//-122.023514,36.968582,0 -122.020424,36.967758,0 -122.018967,36.967827,0 -122.017807,36.964706,0 -122.023727,36.962891,0 -122.026131,36.964947,0 -122.023384,36.96793,0 </coordinates>
//		</LineString>
//	</Placemark>
//</Document>
//</kml>
