package edu.ucsc.track_builder
{		
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	import com.google.maps.MapMouseEvent;
	
	import de.polygonal.ds.Heap;
	
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;
	import flash.utils.Dictionary;
	
	import mx.controls.Alert;

	/** Acts as a factory for TrackSegments and TrackOverlays.
	 * Keeps references to both of them.
	 * Contains some handlers for GUI events relating to tracks. */	
	[Bindable]
	public class Tracks
	{
		/* Values that are controlled by the GUI */
		public var label:String = "";		
		public var straightMaxSpeed:Number;
		public var curveMaxSpeed:Number;
		public var offset:Number = 0; 		
		public var radius:Number;
		public var minOffset:Number;
		public var maxOffset:Number;
		/** For bidirectional track. Indicates which side vehicles move forward on. One of: Tracks.LEFT or Tracks.RIGHT. */ 
		public var driveSide:String;
		/** Distance separating the connecting ramp from the main line. */
		public var rampOffset:Number;
		/** Length of the decel segment of a connecting ramp. */
		public var decelLength:Number;
		/** Length of the accel segment of a connecting ramp. */
		public var accelLength:Number;
		/** Max speed on the turn segment of a connecting ramp. */
		public var turnMaxSpeed:Number;
		/** Radius for the turn segment of a connecting ramp. */
		public var turnRadius:Number;
		/** Max speed on the s-curve segment of a connecting ramp. */
		public var sCurveMaxSpeed:Number;
		/** Radius for the curved segments in an S-curve. */		
		public var sCurveRadius:Number;
		
		/** Meters between elevation checks along a TrackSegment */
		public var elevationSpacing:Number = 10; // TODO: Hardcoded. Expose to UI, add to prefs file!
		
		public var _bidirectional:Boolean;
		public function get bidirectional():Boolean {return _bidirectional;}
		public function set bidirectional(value:Boolean):void {
			if (value != this._bidirectional) {
				this._bidirectional = value;
				var color:uint = value ? TrackOverlay.bidirLineColor : TrackOverlay.unidirLineColor;
				if (Globals.originMarker != null) {
					Globals.originMarker.setColor(color);
				}
			}
		}		

		/* For bidirectional track that is not vertically stacked. Right hand side indicates that vehicles
		 * move forward on the right hand track, as is the convention in the United States. Left hand side
		 * would have vehicles moving forward on the left hand track, as is the convention in the U.K.
		 */
		public static const RIGHT:String = "Right";
		public static const LEFT:String = "Left";
		public static const OFF_RAMP:String = "OffRamp";
		public static const ON_RAMP:String = "OnRamp";
		public static const MIN_ANGLE:Number = 0.0872664626; // 5 degrees
		public static const MAX_ANGLE:Number = 3.05432619;   // 175 degrees

		protected const INCOMING:String = "INCOMING";
		protected const OUTGOING:String = "OUTGOING";

		/** Holds all TrackSegment objects in an associative array. Uses id as key, and a TrackSegment 
		 * instance as the value. */ 
		public var segments:Dictionary;

		/** Holds all TrackOverlay objects. */		
		public var overlays:Vector.<TrackOverlay>; // displays the track segments on the map


		/** Tracks Constructor
		 * @param driveSide Indicates which side of bidirectional track vehicles drive on.
		 *                  Should be one of Tracks.RHS or Tracks.LHS.
		 */
		public function Tracks(driveSide:String) {
			 segments = new Dictionary();
			 overlays = new Vector.<TrackOverlay>();
			 if (driveSide == Tracks.RIGHT || driveSide == Tracks.LEFT) {
			 	this.driveSide = driveSide;
			 } else {
			 	throw new Error("Side must equal one of: Tracks.RIGHT, Tracks.LEFT");
			 }			
		}

		public function onMapClick(event:MapMouseEvent):void {
			Undo.undo(Undo.PREVIEW); // destroy the 'live preview' track segment
			
			// Do nothing if the markers are overlapped.
			if (Globals.originMarker.getLatLng().equals(Globals.destMarker.getLatLng())) {
				return;
			}
						
			var marker:SnappingMarker = Globals.getActiveMarker();
			
			// If we're moving the origin, the first click just sets the position of it.		
			if (marker == Globals.originMarker) {
				Globals.setActiveMarker(Globals.destMarker);
				return;
			}		
			
			Undo.startCommand(Undo.USER);
			
			/* Force the markers to snap to the line (if attached to one) */
			// trace(Globals.originMarker.overlay);
			Globals.originMarker.snapTo();
			Globals.destMarker.snapTo();
						
			try {
				var destOverlays:Vector.<TrackOverlay> = Globals.destMarker.getTrackOverlays();			
				var trail:Vector.<TrackOverlay> = makeTrail(Globals.originMarker.getTrackOverlays(),
						  									destOverlays,
						  									Globals.originMarker.getLatLng(),
						  									Globals.destMarker.getLatLng(),
						  									false); // not a preview
								
				// indicate that we should prompt for save on quit
				Undo.assign(Globals, 'dirty', Globals.dirty);
				Globals.dirty = true;
				
				// After an undo, the user will want the origin marker in the same state as when she clicked.
				Undo.pushMicro(Globals.originMarker, Globals.originMarker.setSnapOverlay, Globals.originMarker.getSnapOverlay());
				Undo.pushMicro(Globals.originMarker, Globals.originMarker.setLatLng, Globals.originMarker.getLatLng());					
				
				// if the newly created track connects to some existing track, go back to using a freely moving originMarker.
				if (destOverlays.length > 0) {					
					Globals.setActiveMarker(Globals.originMarker); // undo is handled internally		
					Globals.originMarker.setSnapOverlay(null); // cut it loose from whatever overlay it's attached to.
				} else { // else move the originMarker to the exposed end of the new track.					
					Globals.originMarker.setSnapOverlay(trail[trail.length-1]);
					Globals.originMarker.snapTo(trail[trail.length-1].getEnd());
				}
				
				Undo.endCommand();
			} catch (err:TrackError) { // error thrown from makeTrail
				Undo.endCommand();
				Undo.undo(Undo.USER); // undo any side effects from before the error.
				Alert.show(err.message); // explain to user what went wrong
			}			
		} 

		public function onMapMouseMove(event:MapMouseEvent):void {
			var marker:SnappingMarker = Globals.getActiveMarker();
			if (marker == Globals.originMarker) { // moving the originMarker
				
			} else { // normal case
				makePreview();
			}
		} 				

		/** Makes a temporary trail showing where the track will go if the user clicks. */
		public function makePreview():void {
			Undo.undo(Undo.PREVIEW); // get rid of the old 'live preview', if one exists
			
			// Do nothing if the markers are overlapped.
			if (Globals.originMarker.getLatLng().equals(Globals.destMarker.getLatLng())) {
				return;
			}
			
			var marker:SnappingMarker = Globals.getActiveMarker();
			if (marker == Globals.destMarker) {				
				Undo.startCommand(Undo.PREVIEW);
				try {
					var originOverlays:Vector.<TrackOverlay>;
					var destOverlays:Vector.<TrackOverlay>;
					
					/* Want to avoid the case that a marker is over some Overlay, but is not
					 * affiliated to it. This can be caused by the user placing a marker near
					 * a trackSegment, then zooming out.
					 */			
					if (Globals.originMarker.getSnapOverlay() == null) {
						originOverlays = new Vector.<TrackOverlay>();
					} else {
						originOverlays = Globals.originMarker.getTrackOverlays(); 
					}
					
					if (Globals.destMarker.getSnapOverlay() == null) {
						destOverlays = new Vector.<TrackOverlay>();
					} else {
						destOverlays = Globals.destMarker.getTrackOverlays();
					}
					makeTrail(originOverlays,
							  destOverlays,
							  Globals.originMarker.getLatLng().clone(),
							  Globals.destMarker.getLatLng().clone(),
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
		                          			                          	
			if (originLatLng.equals(destLatLng)) {
				throw new TrackError("destLatLng equals originLatLng");
			} else if (isNaN(originLatLng.lat()) || isNaN(destLatLng.lat())) {
				throw new TrackError("originLatLng or destLatLng has an invalid value.");
			} else if (originOverlays.length >= 1 && destOverlays.length >= 1 && originOverlays[0] == destOverlays[0]) {
				throw new TrackError("originOverlay equals destOverlays");
			}
			
			var returnOverlays:Vector.<TrackOverlay>;
			var overlay:TrackOverlay; // for iterating through Vectors.
			if (!bidirectional) {
				returnOverlays = makeUnidirectionalTrail(originOverlays, destOverlays, originLatLng, destLatLng, preview);
			} else {
				returnOverlays = makeBidirectionalTrail(originOverlays, destOverlays, originLatLng, destLatLng, preview);
			}		

			return returnOverlays;
        }

		protected function makeUnidirectionalTrail(originOverlays:Vector.<TrackOverlay>,
		                          				 destOverlays:Vector.<TrackOverlay>,
		                          				 originLatLng:LatLng,
		                          				 destLatLng:LatLng,
		                          				 preview:Boolean):Vector.<TrackOverlay> {
			var resultOverlays:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();						
			
			// Create outgoing curved segment
			if (originOverlays.length >= 1) {
				var originCurve:TrackOverlay = makeCurve(originOverlays, originLatLng.clone(), destLatLng.clone(), OUTGOING, false, preview); 
				resultOverlays.push(originCurve);
				originLatLng = originCurve.getEnd();				
			}

			// reserve a id for the straight section connecting the two curves (created later), so that the ids will be in sequential order
			var straightId:String = IdGenerator.getTrackSegId(TrackSegment.forwardExt);
			
			// Create incoming curved segment
			if (destOverlays.length >= 1) {
				var destCurve:TrackOverlay = makeCurve(destOverlays, originLatLng.clone(), destLatLng.clone(), INCOMING, false, preview); 
				resultOverlays.push(destCurve);
				destLatLng = destCurve.getStart();
			}

			// If any curved overlays were required, they've been created, and the origin/dest moved appropriately.
			// Now add a straight seg/overlay that connects the current origin/dest.			
			var straightOverlay:TrackOverlay = makeStraight(originLatLng.clone(), destLatLng.clone(), false, preview,
			                                                NaN, NaN, NaN, straightId); 
		
			// connect the straight seg/overlay
			var overlay:TrackOverlay;
			for each (overlay in resultOverlays) {
				straightOverlay.connect(overlay);
			}

			// collect all the overlays, and sort them in connected order
			resultOverlays.push(straightOverlay);					
			resultOverlays = resultOverlays.sort(TrackOverlay.compareById);
			return resultOverlays;
		}

		protected function makeBidirectionalTrail(originOverlays:Vector.<TrackOverlay>,
					                           destOverlays:Vector.<TrackOverlay>,
											   originLatLng:LatLng,
		                          			   destLatLng:LatLng,
		                          			   preview:Boolean):Vector.<TrackOverlay> {
			var resultOverlays:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();
			var newVec:Vector3D;
	        var overlay:TrackOverlay; // for general iteration
			
			// Track Overlays that should be connected to the straight overlay.
			var toConnect:Vector.<TrackOverlay> = new Vector.<TrackOverlay>();
				
			/* Starting from an existing track */
			if (originOverlays.length >= 1) {
				newVec = Utility.calcVectorFromLatLngs(originLatLng, destLatLng);
				// At one end or the other, and the end is exposed.
				if ((originLatLng.equals(originOverlays[0].getStart()) && originOverlays[0].isStartExposed()) ||
				    	(originLatLng.equals(originOverlays[0].getEnd()) && originOverlays[0].isEndExposed())) {
					var originCurve:TrackOverlay = makeCurve(originOverlays, originLatLng.clone(), destLatLng.clone(), this.OUTGOING, true, preview);
					resultOverlays.push(originCurve);
					toConnect.push(originCurve);
					originLatLng = originCurve.getEnd();
			    }
			    // Starting from the middle of an unidirectional track.
				else if (!originOverlays[0].bidirectional) {
					var originBundle:TrackBundle;
					var existingVec:Vector3D = Utility.calcVectorFromLatLngs(originOverlays[0].getStart(), originOverlays[0].getEnd());					
					var crossProd:Vector3D = existingVec.crossProduct(newVec);					
					
					// Decide whether to use a trumpet or a Y interchange based on which side we're approaching
					// the existing track from, and which side of the bidirectional track vehicles travel forward on.					
					if ((crossProd.z > 0 && this.driveSide == Tracks.RIGHT) ||
								(crossProd.z < 0 && this.driveSide == Tracks.LEFT)) {
						try {
							originBundle = makeHalfTrumpetInterchange(originLatLng, originOverlays[0].segments[0], destLatLng, newVec, this.radius, Tracks.LEFT, 10, 0.5, preview);
						} catch (err:TrackError) {
							if (err.errorID == TrackError.INSUFFICIENT_TRACK) {
								originBundle = makeHalfTrumpetInterchange(originLatLng, originOverlays[0].segments[0], destLatLng, newVec, this.radius, Tracks.RIGHT, 10, 0.5, preview);
							} else {
								throw err;
							}
						}
					} else {
						originBundle = makeHalfYInterchange(originLatLng, originOverlays[0].segments[0], destLatLng, newVec, this.radius, preview);
					}
					
					originLatLng = originBundle.connectNewLatLng; // move the origin point.
					if (!preview) { // don't split the track if it's just a preview.						
						toConnect = toConnect.concat(originBundle.connectNewOverlays);
						resultOverlays = resultOverlays.concat(originBundle.overlays);
						
						// Split the originOverlay and connect the ramps					
						var toSplitOverlay:TrackOverlay = originOverlays[0];
						var nextSplitOverlay:TrackOverlay;
						originBundle.sortConnectExistingArrays(toSplitOverlay.getStart()); // sort by distance
						for (var i:int = 0; i < originBundle.connectExistingLatLngs.length; ++i) {
							try {
								nextSplitOverlay = toSplitOverlay.split(originBundle.connectExistingLatLngs[i], this.offset, preview);
							} catch (err:TrackError) {
								// Has already been split here. No need to do anything.
							}
							// Info about direction of the connection isn't retained. Use a wee bit of brute force and try
							// connecting to either side of the split point.
							originBundle.connectExistingOverlays[i].connect(toSplitOverlay);
							originBundle.connectExistingOverlays[i].connect(nextSplitOverlay);
							
							toSplitOverlay = nextSplitOverlay;
						}
					}
				}
				// Starting from the middle of a bidirectinal track.
				else if (originOverlays[0].bidirectional) { // If starting from a bidirectional track					
					originBundle = makeFullTrumpetInterchange(originLatLng, originOverlays[0], destLatLng, newVec, this.radius, Tracks.LEFT, 10, 0.5, preview);					
					originLatLng = originBundle.connectNewLatLng; // move the origin point
					toConnect = toConnect.concat(originBundle.connectNewOverlays);
					resultOverlays = resultOverlays.concat(originBundle.overlays);
					
				}
				// Starting at an unexposed end (other cases too?)
				else {
					throw new TrackError("Cannot create a bidirectional track at this location.");
				}
			}
	        
	        /* Ending on an existing track */
	        if (destOverlays.length >=1) {
				newVec = Utility.calcVectorFromLatLngs(destLatLng, originLatLng);
				if (isNaN(newVec.length)) {
					throw new TrackError("Unknown problem.");
				}
				// At one end or the other, and the end is exposed.
				if ((destLatLng.equals(destOverlays[0].getStart()) && destOverlays[0].isStartExposed()) ||
				    	(destLatLng.equals(destOverlays[0].getEnd()) && destOverlays[0].isEndExposed())) {
					var destCurve:TrackOverlay = makeCurve(destOverlays, originLatLng.clone(), destLatLng.clone(), this.INCOMING, true, preview);
					resultOverlays.push(destCurve);
					toConnect.push(destCurve);
					destLatLng = destCurve.getStart();
		    	}
			    // Ending in the middle of an unidirectional track.
				else if (!destOverlays[0].bidirectional) {
					var destBundle:TrackBundle;

					var existingVec:Vector3D = Utility.calcVectorFromLatLngs(destOverlays[0].getStart(), destOverlays[0].getEnd());
					var crossProd:Vector3D = existingVec.crossProduct(newVec);
					
					// Decide whether to use a trumpet or a Y interchange based on which side we're approaching
					// the existing track from, and which side of the bidirectional track vehicles travel forward on.					
					if ((crossProd.z > 0 && this.driveSide == Tracks.RIGHT) ||
								(crossProd.z < 0 && this.driveSide == Tracks.LEFT)) {
						try {
							destBundle = makeHalfTrumpetInterchange(destLatLng, destOverlays[0].segments[0], originLatLng, newVec, this.radius, Tracks.LEFT, 10, 0.5, preview);
						} catch (err:TrackError) {
							if (err.errorID == TrackError.INSUFFICIENT_TRACK) {
								destBundle = makeHalfTrumpetInterchange(destLatLng, destOverlays[0].segments[0], originLatLng, newVec, this.radius, Tracks.RIGHT, 10, 0.5, preview);
							} else {
								throw err;
							}
						}
					} else {
						destBundle = makeHalfYInterchange(destLatLng, destOverlays[0].segments[0], originLatLng, newVec, this.radius, preview);
					}
					
					destLatLng = destBundle.connectNewLatLng; // move the dest point.
					if (!preview) {  // don't split the track if it's just a preview.						
						toConnect = toConnect.concat(destBundle.connectNewOverlays);
						resultOverlays = resultOverlays.concat(destBundle.overlays);
						
						// Split the destOverlay and connect the ramps					
						var toSplitOverlay:TrackOverlay = destOverlays[0];
						var nextSplitOverlay:TrackOverlay;
						destBundle.sortConnectExistingArrays(toSplitOverlay.getStart());
						for (var i:int = 0; i < destBundle.connectExistingLatLngs.length; ++i) {
							nextSplitOverlay = toSplitOverlay.split(destBundle.connectExistingLatLngs[i], this.offset, preview);
							// Info about direction of the connection isn't retained. Use a wee bit of brute force and try
							// connecting to either side of the split point.
							destBundle.connectExistingOverlays[i].connect(toSplitOverlay);
							destBundle.connectExistingOverlays[i].connect(nextSplitOverlay);
							
							toSplitOverlay = nextSplitOverlay;
						}
					}				
				}
				
				// Ending in the middle of a bidirectional track.
				else if (destOverlays[0].bidirectional) {				
					destBundle = makeFullTrumpetInterchange(destLatLng, destOverlays[0], originLatLng, newVec, this.radius, Tracks.LEFT, 10, 0.5, preview);					
					destLatLng = destBundle.connectNewLatLng; // move the origin point
					toConnect = toConnect.concat(destBundle.connectNewOverlays);
					resultOverlays = resultOverlays.concat(destBundle.overlays);
					
				}
				// Starting at an unexposed end (other cases too?)
				else {
					throw new TrackError("Cannot create a bidirectional track at this location.");
				}
	        }
       
	        var straightOverlay:TrackOverlay = makeStraight(originLatLng, destLatLng, true, preview);
	        resultOverlays.push(straightOverlay);	       

	        for each (overlay in toConnect) {
	        	straightOverlay.connect(overlay);	
	        }
	        	        
			return resultOverlays;
		}

		/** Creates a single curve segment and wraps it in an overlay.*/
		protected function makeCurve(targetOverlays:Vector.<TrackOverlay>,
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
		
		/** Creates straight track segment(s) and wraps it in an overlay.
		 * @param origin The start of the segment.
		 * @param dest The end of the segment.
		 * @param bidirectional If true, a reversed segment is also created and added to the overlay.
		 * @param maxSpeed (Optional) Defaults to this.straightMaxSpeed if not specified. Use NaN to leave at default.
		 * @param startOffset (Optional) Defaults to this.Offset if not specified. Use NaN to leave at default.
		 * @param endOffset (Optional) Defaults to this.Offset if not specified. Use NaN to leave at default.
		 * @param id (Optional) Uses a particular id. Otherwise gets an id from the IdGenerator. Use null to leave at default.
		 * @param id2 (Optional) A second id for a bidirectional segment. Use null to leave at default.
		 */
		public function makeStraight(origin:LatLng, dest:LatLng, bidirectional:Boolean, preview:Boolean,
		                             maxSpeed:Number=NaN, startOffset:Number=NaN, endOffset:Number=NaN,
		                             id:String=null, id2:String=null):TrackOverlay {
			if (id == null) {
				id = IdGenerator.getTrackSegId(TrackSegment.forwardExt);
			}
			if (isNaN(maxSpeed)) maxSpeed = this.straightMaxSpeed;
			if (isNaN(startOffset)) startOffset = this.offset;
			if (isNaN(endOffset)) endOffset = this.offset;
			var segs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			segs.push(new TrackSegment(id, label, origin, dest, maxSpeed, startOffset, endOffset, 0, 0, null, preview));
			if (bidirectional) { 
				segs.push(segs[0].clone(true, id2)); // if id2 is null, clone fetches a new id from IdGenerator.
			}
			return new TrackOverlay(segs); 
		}

		protected function canCut(a:LatLng, b:LatLng, c:LatLng, radius:Number):Boolean {
			var ba:Vector3D = Utility.calcVectorFromLatLngs(b, a);
			var bc:Vector3D = Utility.calcVectorFromLatLngs(b, c);
			var abcAngle:Number = Utility.angleBetween(ba, bc);
			var dist:Number = radius / Math.tan(abcAngle/2);
			return dist < ba.length && dist < bc.length;
		}
		
		/** Does not alter the parameters. */
		protected function makeCurveSeg_Cut(anchor1:LatLng, control:LatLng, anchor2:LatLng, radius:Number, maxLength:Number, preview:Boolean):TrackSegment {
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
			return new TrackSegment(id, label, start, end, this.curveMaxSpeed, offset, offset, radius, arcAngle, centerLatLng, preview);
		}
		
		/** Returns a single curved Tracksegment which originates at anchor, and is tangent to originVec.
		 *  The curved TrackSegment will terminate such that a line segment from it's endpoint to dest will
		 *  be tangent to it.
		 *  If reverse is True, then direction of the line segment is reversed, but the endpoints remain the same.
		 */
		protected function makeCurveSeg_NoCut(anchor:LatLng, originVec:Vector3D, dest:LatLng, radius:Number, reverse:Boolean, preview:Boolean):TrackSegment {				
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
										this.curveMaxSpeed,
										offset,
										offset,
										radius,
										arcAngle,
										center,
										preview);
		}

		/** Creates a 'trumpet' interchange between an existing unidirectional TrackSegment and a
		 * in-the-process-of-being-created bidirectional track. The trumpet created by this
		 * function consists of the following segments, where each segment continues from the end
		 * of the previous segment unless otherwise noted:
		 * <ul>
		 *   <li><em>Approach:</em> A straight piece of bidir track that is aligned with newVec and which raises
		 * the track from <code>this.offset</code> meters above the ground to <code>clearance</code>
		 * meters above the existing track. Ends at <code>intersection</code>. FIXME: Specify where it begins.</li>
		 *   <li><em>Extend:</em> A straight piece of bidir track that is aligned with newVec and which travels
		 * some distance beyond <code>intersection</code>. The exact distance depends on
		 * the convergance angle, but is equal to <code>radius</code> in the case where the tracks
		 * intersect at 90 degrees.</li>
		 *   <li><em>Trumpet:</em> A curved piece of bidir track that curves around so as to end perpendicular 
		 * to <code>existing</code> and <code>radius</code> radius meters away. The radius of curvature is 
		 * <code>radius</code> and whether it curves to the left or to the right is determined by <code>side</code>.</li>
		 *   <li><em>Inside:</em> A curved piece of unidirectional track that connects <code>existing</code> with
		 * the end of <em>Trumpet</em>. The direction of the piece depends on <code>Tracks.driveSide</code> and
		 * <code>side</code>, but if you consider it to start at the end of the <em>Trumpet</em> then it curves towards
		 * <code>side</code>. Its radius is <code>radius</code>.</li>
		 *   <li><em>Outside:</em> A curved piece of unidirectional track that also connects <code>existing</code> with
		 * the end of <em>Trumpet</em>. This piece travels in the opposite direction as <em>Inside</em>, and curves
		 * towards the opposite side.
		 * </ul>
		 * 
		 * The interchange TrackSegments are not connected to <code>existing</code>. The returned TrackBundle contains
		 * information about which segments need to be connected and where. Making the connections is deferred
		 * in order to avoid complications that arise with bidirectional track -- splitting one segment of a bidir
		 * track splits the parallel track too.
		 * 
		 * @param intersection LatLng at which the <code>existing</code> track and the new track intersect.
		 * @param existing The existing unidirectional track that is being connected to.
		 * @param newLatLng LatLng which is indicated by <code>newVec</code>.
		 * @param newVec A Vector3D which the new track is to be aligned with. It points to <code>newLatLng</code>
		 * when originating at intersection.
		 * @param radius The radius of curvature for all curved segments in the interchange. Measured in radians.
		 * @param side One of <code>Tracks.LEFT</code> or <code>Tracks.RIGHT</code>. It indicates which direction the
		 * trumpet will curve towards.
		 * @param clearance The vertical distance by which the new track should rise above the existing track at
		 * <code>intersection</code>. Measured in meters.
		 * @param slope The slope used by the <em>Approach</em> segment when climbing. 
		 * @param preview Whether this function is called as part of the live track placement preview, or as the result
		 *  of a user click. Some work is ommitted if it is only part of a preview.
		 * 
		 * @return a <code>TrackBundle</code> containing all of the new segments, wrapped in <code>TrackOverlays</code>.
		 */
		protected function makeHalfTrumpetInterchange(
				intersection:LatLng, existing:TrackSegment, newLatLng:LatLng, newVec:Vector3D, 
				radius:Number, side:String, clearance:Number, slope:Number, preview:Boolean):TrackBundle {
			var bundle:TrackBundle = new TrackBundle();
			
			var nVec:Vector3D = newVec.clone(); // n indicates that it is aligned with 'newVec'
			nVec.normalize();
			
		 	 /* Establish the convention that the trumpet should always curve in the same direction as eVec. */			
			var eVec:Vector3D;  // e indicates that it is aligned with the 'existing' segment.
            if (side == this.driveSide) { // both are Tracks.LEFT or Tracks.RIGHT
            	// trumpet curves in opposite direction as existing.
            	eVec = Utility.calcVectorFromLatLngs(existing.getEnd(), existing.getStart());
		 	} else { // one is Tracks.LEFT, the other is Tracks.RIGHT
            	// trumpet curves in same direction as existing. 
		 		eVec = Utility.calcVectorFromLatLngs(existing.getStart(), existing.getEnd());
		 	}
		 	eVec.normalize();
		 			 	
		 	var angle:Number = Math.PI - Utility.angleBetween(eVec, nVec);
		 	if (angle < Tracks.MIN_ANGLE || angle > Tracks.MAX_ANGLE) {
				throw new TrackError("Vectors are too nearly parallel.");
			}
		 	var tanHalfAngle:Number = Math.tan(angle/2);
		 	
			// Check that the ramps will fit on 'existing'
			var tStartCurveDist:Number = radius/tanHalfAngle; // dist at which the new bidir track begins curving to form the trumpet
			if (side == this.driveSide) {
				if (tStartCurveDist + 2*radius > existing.getStart().distanceFrom(intersection)) {
			    	throw new TrackError("Trumpet interchange does not fit.", TrackError.INSUFFICIENT_TRACK);
			 	}
	        } else {
	        	if (tStartCurveDist + 2*radius > existing.getEnd().distanceFrom(intersection)) {
	        		throw new TrackError("Trumpet interchange does not fit.", TrackError.INSUFFICIENT_TRACK);
	        	}
	        }

			// Create a bunch of vectors to calculate LatLngs with.
			// 'a' refers the length of the segment in the diagram: prt-sim/docs/TrumpetInterchange 
		 	var nAVector:Vector3D = nVec.clone();
 		 	var eAVector:Vector3D = eVec.clone();
		 	var eRadiusVector:Vector3D = eVec.clone();

			// Scale them to useful lengths.
 		 	nAVector.negate(); // point towards the trumpet start
		 	nAVector.scaleBy(tStartCurveDist);
 		 	eRadiusVector.scaleBy(radius);
		 	eAVector.scaleBy(tStartCurveDist);

			// Create a vector that is perpendicular to eVec, and which points towards the trumpet side.		 	
  		 	var ePerpRadiusVector:Vector3D; 
 		 	var zVec:Vector3D = nVec.crossProduct(eVec);
 		 	ePerpRadiusVector = zVec.crossProduct(eVec); 		 	
		 	ePerpRadiusVector.scaleBy(radius/ePerpRadiusVector.length);
		 		        
		 	/* Create TrackSegments and TrackOverlays */
		 	var tArcAngle:Number =  Math.PI/2 + angle // 2*Math.PI - (Math.PI - angle + rampArcAngle)
			var tArcLength:Number = tArcAngle*radius;	 	
 		 	var rampArcLength:Number = radius*Math.PI/2; // ramps are always 90 degrees
	
		 	// approach seg
		 	var approachLength:Number = clearance / slope;
		 	var approachStart:LatLng = Utility.calcLatLngFromVector(intersection, nVec, approachLength); 
		 	var approachOverlay:TrackOverlay = makeStraight(approachStart, intersection, true, preview, NaN,
		 	                                                this.offset, this.offset + clearance);
			bundle.addOverlay(approachOverlay);
			bundle.markAsConnectNew(approachOverlay);
			bundle.setConnectNewLatLng(approachStart);
		 	
            // extend seg (from intersection to trumpet start)
            var extendSegHLength:Number = tStartCurveDist;
            var totalTrumpetHLength:Number = extendSegHLength + tArcLength + rampArcLength;
            var tStartCurveOffset:Number = this.offset + clearance * (totalTrumpetHLength - extendSegHLength)/totalTrumpetHLength;
		 	var tStartCurveLatLng:LatLng = Utility.calcLatLngFromVector(intersection, nAVector);
            var extendOverlay:TrackOverlay = makeStraight(intersection, tStartCurveLatLng, true, preview, NaN,
                                                          this.offset + clearance, tStartCurveOffset);
			bundle.addOverlay(extendOverlay);
			approachOverlay.connect(extendOverlay);
			
			// outside and inside ramps
			var eRampsIntersectVector:Vector3D = eAVector.add(eRadiusVector);
			var eRampsIntersectLatLng:LatLng = Utility.calcLatLngFromVector(intersection, eRampsIntersectVector);
			var insideRampOverlay:TrackOverlay;
			var outsideRampOverlay:TrackOverlay;
		 	var trueEVec:Vector3D = Utility.calcVectorFromLatLngs(existing.getStart(), existing.getEnd()); 
		 	trueEVec.normalize(); // unit vector that points along the true direction of existing
		 	var trueNegEVec:Vector3D = trueEVec.clone();
			trueNegEVec.negate();
		 	var ePerpUnitVector:Vector3D = ePerpRadiusVector.clone();
		 	ePerpUnitVector.normalize();
 			var tSplitLatLng:LatLng; //where the trumpet loop splits into two unidirectional tracks.
 		 	var tSplitOffset:Number = this.offset + clearance * (totalTrumpetHLength - extendSegHLength - tArcLength)/(totalTrumpetHLength);
			if (side == this.driveSide) {
				insideRampOverlay = makeTangentCurve( // trumpet -> existing 
						eRampsIntersectLatLng, ePerpUnitVector, trueEVec, this.curveMaxSpeed,  
						this.offset + tSplitOffset, this.offset, radius, false, preview);
				outsideRampOverlay = makeTangentCurve( // existing -> trumpet
						eRampsIntersectLatLng, trueNegEVec, ePerpUnitVector, this.curveMaxSpeed,
						this.offset + tSplitOffset, this.offset, radius, false, preview);
				tSplitLatLng = insideRampOverlay.getStart();
			} else { // side != this.driveSide
				insideRampOverlay = makeTangentCurve( // existing -> trumpet
						eRampsIntersectLatLng, trueNegEVec, ePerpUnitVector, this.curveMaxSpeed,
						this.offset + tSplitOffset, this.offset, radius, false, preview);
				outsideRampOverlay = makeTangentCurve( // trumpet -> existing
						eRampsIntersectLatLng, ePerpUnitVector, trueEVec, this.curveMaxSpeed,
						this.offset + tSplitOffset, this.offset, radius, false, preview);
				tSplitLatLng = insideRampOverlay.getEnd();
			}
			bundle.addOverlay(insideRampOverlay);
			bundle.addOverlay(outsideRampOverlay);		

			// Making the ramps' connections to "existing" is deferred to the calling function. 			
			if (side == this.driveSide) { // the outside ramp occurs first when travelling along existing.
				bundle.setConnectExisting(outsideRampOverlay.getStart(), outsideRampOverlay);
				bundle.setConnectExisting(insideRampOverlay.getEnd(), insideRampOverlay);				
			} else { // side != this.driveSide  -- the inside ramp occurs first
				bundle.setConnectExisting(insideRampOverlay.getStart(), insideRampOverlay);
				bundle.setConnectExisting(outsideRampOverlay.getEnd(), outsideRampOverlay);
			}

			// trumpet loop
 		 	var tCenterLatLng:LatLng = Utility.calcLatLngFromVector(intersection, eAVector.add(ePerpRadiusVector));			
			var trumpetSegFwd:TrackSegment;
			var signedArcAngle:Number;
		 	if (side == Tracks.RIGHT) { // Trumpet curves to the right, or Clockwise. Use negative arcAngle.
		 		signedArcAngle = -Math.abs(tArcAngle);
			} else { // side == Tracks.LEFT   Trumpet curves to the left, or CounterClockwise. Use positive arcAngle.
				signedArcAngle = Math.abs(tArcAngle);
			}
			trumpetSegFwd = new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
			            this.label, tStartCurveLatLng, tSplitLatLng, this.curveMaxSpeed, this.offset + tStartCurveOffset,
				        this.offset + tSplitOffset, radius, signedArcAngle,	tCenterLatLng, preview);			
			var trumpetSegRev:TrackSegment = trumpetSegFwd.clone(true);
			var trumpetSegs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			trumpetSegs.push(trumpetSegFwd);
			trumpetSegs.push(trumpetSegRev);
			var trumpetOverlay:TrackOverlay = new TrackOverlay(trumpetSegs);
			bundle.addOverlay(trumpetOverlay);
			extendOverlay.connect(trumpetOverlay);
			trumpetOverlay.connect(insideRampOverlay);
			trumpetOverlay.connect(outsideRampOverlay);

			return bundle;
	 	}
		 			
		/** Creates a Y interchange between an existing unidirectional track and a new bidirectional track.
		 * Note that this function should only be used when the bidirectional track is approaching the
		 * unidirectional track from the same side as Tracks.driveSide. That is, if the vehicles travel
		 * forward on the right side of bidirectional track, then this function should only be used when
		 * the bidirectional track is approaching unidirectional track from the right. Otherwise, use
		 * a trumpet interchange.
		 * 
		 * The interchange consists of two connecting ramps, and a straight section connecting them.
		 * If interiorConnection is true, then the straight segment is made bidirectional, otherwise it is unidirectional.
		 *
		 * @param intersection The point at which the new track would intersect the existing, if it were to continue on straight.  
		 * @param existing The existing unidirectional TrackSegment that we are connecting to.
		 * @param newVec A Vector3D which the new track is to be aligned with. It points towards the origin
		 * when originating at intersection.
		 * @param radius The curvature radius for the ramps, in meters.
		 * @param preview Whether the interchange is being created as part of the live track placement preview, or as the result of a user click.
		 * @param interiorConnection (Optional) Default is false. When true, the piece of straight connecting track on
		 * newVec is bidirectional rather than unidirectional. This provides an interior connection point.
		 * @see #makeHalfTrumpetInterchange
		 * @see #makeConnectingRamp
		 * @return The TrackBundle's connection point falls on newVec, at the start/end of the furthest ramp.
		 * If interiorConnection is true, then the TrackBundle's altConnectLatLng and altConnectOverlays fields are used
		 * to return information about the interior connection point. 
		 */		
		protected function makeHalfYInterchange(
				intersection:LatLng, existing:TrackSegment, newLatLng:LatLng, newVec:Vector3D, radius:Number,
				preview:Boolean, interiorConnection:Boolean=false):TrackBundle {
			var bundle:TrackBundle = new TrackBundle();
			var nVec:Vector3D = newVec.clone();
			var eVec:Vector3D = Utility.calcVectorFromLatLngs(existing.getStart(), existing.getEnd());						
			nVec.normalize();
			eVec.normalize();
			var negEVec:Vector3D = eVec.clone();
			negEVec.negate();

			// ramp from new to existing
			var neRamp:TrackBundle = makeConnectingRamp(intersection, nVec, eVec, this.sCurveRadius, this.sCurveMaxSpeed,
					this.turnRadius, this.turnMaxSpeed, this.rampOffset, this.decelLength, this.accelLength, preview);
			var neRampStart:LatLng = neRamp.overlays[0].getStart();
			var neRampEnd:LatLng = neRamp.overlays[neRamp.overlays.length-1].getEnd();
			
			var iterOverlay:TrackOverlay;					
			for each (iterOverlay in neRamp.overlays) {
				bundle.addOverlay(iterOverlay);
			}
			
			if (neRampEnd.distanceFrom(intersection) > existing.getEnd().distanceFrom(intersection)) {
				throw new TrackError("New to Existing ramp extends beyond the end of Existing.", TrackError.INSUFFICIENT_TRACK);
			}
			if (neRampStart.distanceFrom(intersection) > newLatLng.distanceFrom(intersection)) {
				throw new TrackError("New to Existing ramp extends beyond newLatLng", TrackError.INSUFFICIENT_TRACK);
			}
			
			// ramp from existing to new
			eVec.negate();
			var enRamp:TrackBundle = makeConnectingRamp(intersection, eVec, nVec, this.sCurveRadius, this.sCurveMaxSpeed,
					this.turnRadius, this.turnMaxSpeed, this.rampOffset, this.decelLength, this.accelLength, preview);
			var enRampStart:LatLng = enRamp.overlays[0].getStart();
			var enRampEnd:LatLng = enRamp.overlays[enRamp.overlays.length-1].getEnd();			
			
			for each (iterOverlay in enRamp.overlays) {
				bundle.addOverlay(iterOverlay);
			}
			
			if (enRampStart.distanceFrom(intersection) > existing.getStart().distanceFrom(intersection)) {
				throw new TrackError("Existing to New ramp extends beyond the start of Existing.", TrackError.INSUFFICIENT_TRACK);
			}
			if (enRampEnd.distanceFrom(intersection) > newLatLng.distanceFrom(intersection)) {
				throw new TrackError("Existing to New ramp extends beyond newLatLng", TrackError.INSUFFICIENT_TRACK);
			}

			// The ramps split/merge from the main line at approx the same spot.
			if (neRampStart.distanceFrom(enRampEnd) < 0.0001) { // FIXME: Somewhat arbitrary distance threshold.
				bundle.markAsConnectNew(neRamp.overlays[0]);
				bundle.markAsConnectNew(enRamp.overlays[enRamp.overlays.length-1]);
				bundle.setConnectNewLatLng(neRampStart);
				
				if (interiorConnection) {
					bundle.markAsConnectAlt(neRamp.overlays[0]);
					bundle.markAsConnectAlt(enRamp.overlays[enRamp.overlays.length-1]);
					bundle.setConnectAltLatLng(neRampStart);
				}
			} else {
				/* Create a straight connecting segment between the two ramp endpoints. */
				var straight:TrackOverlay;
				if (!interiorConnection) {
					straight = makeStraight(enRampEnd, neRampStart, false, preview);
				} else { // interiorConnection == true
					straight = makeStraight(enRampEnd, neRampStart, true, preview);
					bundle.markAsConnectAlt(straight);
				} 
				bundle.addOverlay(straight);
				bundle.markAsConnectNew(straight);
					
				// new->existing splits closer to the intersection. Connect straight to it.	
				if (neRampStart.distanceFrom(intersection) < enRampEnd.distanceFrom(intersection)) {
					straight.connect(neRamp.overlays[0]);
					bundle.markAsConnectNew(enRamp.overlays[enRamp.overlays.length-1]);
					bundle.setConnectNewLatLng(enRampEnd);
					
					if (interiorConnection) {
						bundle.markAsConnectAlt(neRamp.overlays[0]);
						bundle.setConnectAltLatLng(neRampStart);	
					}
				} else { // existing->new merges closer to the intersection. Connect straight to it.
					straight.connect(enRamp.overlays[enRamp.overlays.length-1]);
					bundle.markAsConnectNew(neRamp.overlays[0]);
					bundle.setConnectNewLatLng(neRampStart);
					
					if (interiorConnection) {
						bundle.markAsConnectAlt(enRamp.overlays[enRamp.overlays.length-1]);
						bundle.setConnectAltLatLng(enRampEnd);
					}
				}			
			}
			
			// Delay altering 'existing' until the calling function is ready to do it.
			bundle.setConnectExisting(enRampStart, enRamp.overlays[0]);
			bundle.setConnectExisting(neRampEnd, neRamp.overlays[neRamp.overlays.length-1]);
									 
			return bundle;
		}		

		/** Makes an interchange between two bidirectional tracks. The interchange is comprised of a Half-Trumpet and
		 * a Half-Y interchange.
		 */
		protected function makeFullTrumpetInterchange(
				intersection:LatLng, existing:TrackOverlay, newLatLng:LatLng, newVec:Vector3D, 
				radius:Number, side:String, clearance:Number, maxSlope:Number, preview:Boolean):TrackBundle {
					
			
			var ySeg:TrackSegment;
			var trumpetSeg:TrackSegment;
			
			// Figure out which segment should be connected via a Y and which should be connected via a Trumpet.
			var eVec:Vector3D = Utility.calcVectorFromLatLngs(existing.getStart(), existing.getEnd());
			var zVec:Vector3D = eVec.crossProduct(newVec);
			if ((zVec.z < 0 && this.driveSide == Tracks.RIGHT) || (zVec.z > 0 && this.driveSide == Tracks.LEFT)) {
				ySeg = existing.segments[0];
				trumpetSeg = existing.segments[1];
			} else {
				ySeg = existing.segments[1];
				trumpetSeg = existing.segments[0];
			}
				
			var yBundle:TrackBundle = makeHalfYInterchange(
					intersection, ySeg, newLatLng, newVec, radius, preview, true);
			
			var slope:Number = clearance / yBundle.connectAltLatLng.distanceFrom(intersection);
			if (slope > maxSlope) {
				throw new TrackError("Insufficient distance between the intersection point and the nearest offramp from the new track. With the supplied maximum slope, the vertical clearance will not be met.");
			}			
					
			try {
				var trumpetBundle:TrackBundle = makeHalfTrumpetInterchange(
						intersection, trumpetSeg, newLatLng, newVec, radius, side, clearance, slope, preview);
			} catch (err:TrackError) {
				if (err.errorID == TrackError.INSUFFICIENT_TRACK) {
					var otherSide:String = side==Tracks.LEFT ? Tracks.RIGHT : Tracks.LEFT; // toggle the side
					trumpetBundle = makeHalfTrumpetInterchange(intersection, trumpetSeg, newLatLng, newVec, radius, otherSide, clearance, slope, preview);
				} else {
					throw err;
				}
			}

			
			for each (var overlay:TrackOverlay in yBundle.connectAltOverlays) {
				for each (var otherOverlay:TrackOverlay in trumpetBundle.connectNewOverlays) {
					overlay.connect(otherOverlay);
				}
			}

			var resultBundle:TrackBundle = new TrackBundle();
			resultBundle.overlays = yBundle.overlays.concat(trumpetBundle.overlays);
			resultBundle.connectNewOverlays = yBundle.connectNewOverlays.concat(trumpetBundle.connectNewOverlays);
			resultBundle.setConnectNewLatLng(yBundle.connectNewLatLng);
			resultBundle.connectExistingLatLngs = yBundle.connectExistingLatLngs.concat(trumpetBundle.connectExistingLatLngs);
			resultBundle.connectExistingOverlays = yBundle.connectExistingOverlays.concat(trumpetBundle.connectExistingOverlays);						
			
			if (!preview) { // don't bother with splitting if just a preview								
				// Split 'existing' and connect the ramps					
				var toSplitOverlay:TrackOverlay = existing;
				var nextSplitOverlay:TrackOverlay;
				resultBundle.sortConnectExistingArrays(toSplitOverlay.getStart()); // sort by proximity to start
				for (var i:int = 0; i < resultBundle.connectExistingLatLngs.length; ++i) {
					try {
						nextSplitOverlay = toSplitOverlay.split(resultBundle.connectExistingLatLngs[i], this.offset, preview);
					} catch (err:TrackError) {
						// Has already been split here. No need to do anything.
					}
					// Info about direction of the connection isn't retained. Use a wee bit of brute force and try
					// connecting to either side of the split point.
					resultBundle.connectExistingOverlays[i].connect(toSplitOverlay);
					resultBundle.connectExistingOverlays[i].connect(nextSplitOverlay);
					
					toSplitOverlay = nextSplitOverlay;
				}
			}

			return resultBundle;	
		}

		/** Creates an S-Curve which consists of two curved TrackOverlays of equal length which curve in opposite
		 * directions. Only supply one of startLatLng or endLatLng.
		 * 
		 * @param vec A unit vector that indicates the direction of the start of the S-curve. Must be length one. 
		 * @param lateralOffset Amount of lateral travel in the S-curve. Measured in meters.
		 * @param radius The radius for both curved segments. Must be >= lateralOffset/2.0, or a TrackError will be thrown. 
		 * @param side One of Tracks.LEFT or TRACKS.RIGHT. Indicates the direction of the initial curvature when 
		 * starting at the start of the S-curve.
		 * @param bidirectional Whether the S-Curve should have bidirectional track.
		 * @param maxSpeed The max speed rating. Measured in meters per second.
		 * @param startOffset The vertical offset from ground level at startLatLng.
		 * @param endOffset The vertical offset from ground level at endLatLng.
		 * @param preview Whether the Overlays are being created for the live preview or not.
		 * @param startLatLng The starting point for the S-curve. Only supply this or endLatLng, not both.
		 * @param endLatLng The ending point for the S-curve. Only supply this or startLatLng, not both.
		 */
		public function makeSCurve(vec:Vector3D, lateralOffset:Number, radius:Number, side:String,
				bidirectional:Boolean, maxSpeed:Number, startOffset:Number, endOffset:Number, preview:Boolean,
				startLatLng:LatLng=null, endLatLng:LatLng=null):TrackBundle
		{
			// Defend against incorrect parameters.
			if ((startLatLng == null && endLatLng == null) || (startLatLng != null && endLatLng != null)) {
				throw new Error("Must provide exactly one of: startLatLng, endLatLng");
			}			
			if (radius < lateralOffset/2.0) {
				throw new TrackError("The S-Curve radius must be greater than or equal to lateralOffset/2.0");
			}
			
			// Calc some useful values.
			var angle:Number = Math.acos((radius - lateralOffset/2.0)/radius); // arc angle
			var len:Number = 2*Math.sin(angle)*radius; // length along vec, from startLatLng to endLatLng
			var curveIntersectDist:Number = Math.tan(angle/2.0)*radius; // distance from endpoint along vec
			
			// make a vector that is perpendicular to vec, and which points towards side.
			var perpVec:Vector3D;
			if (side == Tracks.LEFT) {
				perpVec = Vector3D.Z_AXIS.crossProduct(vec);
			} else if (side == Tracks.RIGHT) {
				perpVec = vec.crossProduct(Vector3D.Z_AXIS);
			}
			perpVec.scaleBy(lateralOffset);

			// Find the missing endpoint.
			var lenVec:Vector3D = vec.clone();
			var diagVec:Vector3D;		 
			if (endLatLng == null) {
				lenVec.scaleBy(len);
				diagVec = lenVec.add(perpVec);
				endLatLng = Utility.calcLatLngFromVector(startLatLng, diagVec);
			} else { // startLatLng == null
				lenVec.scaleBy(-len);
				perpVec.negate();
				diagVec = lenVec.add(perpVec);
				startLatLng = Utility.calcLatLngFromVector(endLatLng, diagVec);						
			}						

			var negVec:Vector3D = vec.clone();
			negVec.negate();
			var middleOffset:Number = (startOffset + endOffset)/2.0;
			var signedAngle:Number = (side == Tracks.RIGHT) ? -angle : angle;

			// create the first Curve overlay
			var curveOneIntersect:LatLng = Utility.calcLatLngFromVector(startLatLng, vec, curveIntersectDist); 
			var curveOneV2:Vector3D = vec.clone();
			Utility.rotate(curveOneV2, signedAngle);
			var curveOne:TrackOverlay = makeTangentCurve(
					curveOneIntersect, negVec, curveOneV2, maxSpeed, startOffset, middleOffset, radius,
					bidirectional, preview);
			
			// create the second Curve overlay
			var curveTwoIntersect:LatLng = Utility.calcLatLngFromVector(endLatLng, negVec, curveIntersectDist); 
			var curveTwoV1:Vector3D = negVec.clone();
			Utility.rotate(curveTwoV1, signedAngle);
			var curveTwo:TrackOverlay = makeTangentCurve(
					curveTwoIntersect, curveTwoV1, vec, maxSpeed, middleOffset, endOffset, radius,
					bidirectional, preview);

			var result:TrackBundle = new TrackBundle();
			curveOne.connect(curveTwo);
			result.addOverlay(curveOne);
			result.addOverlay(curveTwo);
			result.setConnectExisting(startLatLng, curveOne);
			result.setConnectNewLatLng(endLatLng);
			result.markAsConnectNew(curveTwo);
			
			return result;			
		}
				 
		/** Makes a [on|off] ramp consisting of two curved segments (the S-curve) and a straight segment.
		 * The s_curve moves the track <code>lateralOffset</code> meters away from vec and leaves it parallel to vec.
		 * The straight segment is <code>length - sCurveLength</code> meters long. If the ramp would exceed
		 * <code>length</code> meters, then a StationLengthError is thrown.
		 * 
		 * If the sCurveLength must be known explicitly, it may be calculated as:
		 *  alpha = acos((radius - lateralOffset/2.0) / radius)
		 *  sCurveLength = 2*sin(alpha)*radius
		 * 
		 * @param vec A unit vector that points in the direction of the bypass. Must have length 1.
		 * @param latlng Starting location for the ramp.
		 * @param length Total track length for the entire ramp (including straight segment). Measured in meters.
		 * @param lateralOffset Desired distance from the bypass and the station line. Measured in meters.
		 * @param radius Desired radius for the two curve segments which make up the S-curve. The actual radius used
		 * will be: <code>max(radius, lateralOffset/2.0)</code>. Measured in meters.
		 * @param side One of Tracks.LEFT or Tracks.RIGHT.
		 * @param rampType One of Tracks.OFF_RAMP or Tracks.ON_RAMP 
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
				rampType:String, maxSpeed:Number, startOffset:Number, endOffset:Number, preview:Boolean 
				):TrackBundle
		{
			var bundle:TrackBundle = new TrackBundle();
					
			// If the radius is less than the lateral offset, we would need to introduce a straight segment in the
			// S-curve. Rather than doing this, just increase the radius.
			radius = Math.max(radius, lateralOffset/2.0);
			var angle:Number = Math.acos((radius - lateralOffset/2.0)/radius);
			
			// Assumes both curves in the S-curve are the same length.
			var sCurveLength:Number = 2*angle*radius
			var straightLength:Number = length - sCurveLength;
			if (straightLength < 0.001) {
				throw new StationLengthError("The requested station ramp length is too short, given the requested lateral offset and curve radius.");
			} 
			
			if (rampType == Tracks.ON_RAMP) {
				var straightEnd:LatLng = Utility.calcLatLngFromVector(latlng, vec, straightLength);
				var straight:TrackOverlay = Globals.tracks.makeStraight(latlng, straightEnd, false, preview, maxSpeed, startOffset, endOffset);
				bundle.addOverlay(straight);
				bundle.setConnectExisting(latlng, straight);				
				latlng = straight.getEnd(); // start the S-curve at latlng, regardless of whether an ON_RAMP or OFF_RAMP
			}
			
			var sCurveIntersectDist:Number = Math.tan(angle/2.0)*radius;
			var curveOneIntersect:LatLng = Utility.calcLatLngFromVector(latlng, vec, sCurveIntersectDist);
			var curveOneV1:Vector3D = vec.clone();
			curveOneV1.negate();
			var curveOneV2:Vector3D = vec.clone();						
			if ((side == Tracks.LEFT && rampType == Tracks.OFF_RAMP) || (side == Tracks.RIGHT && rampType == Tracks.ON_RAMP)) { // rotate vector CCW
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
			
			if (rampType == Tracks.OFF_RAMP) {
				var straightStart:LatLng = curveTwo.getEnd();
				var straightEnd:LatLng = Utility.calcLatLngFromVector(straightStart, vec, straightLength);
				var straight:TrackOverlay = Globals.tracks.makeStraight(straightStart, straightEnd, false, preview);
				bundle.addOverlay(straight);
				bundle.setConnectNewLatLng(straightEnd);
				bundle.markAsConnectNew(straight);
			}
			
			/* Make connections within the bundle and set data for external connections. */ 
			curveOne.connect(curveTwo);
			if (rampType == Tracks.OFF_RAMP) {
				bundle.setConnectExisting(latlng, curveOne);				
				curveTwo.connect(straight); 
			} else { // rampType == ON_RAMP
				bundle.setConnectNewLatLng(curveTwo.getEnd());
				bundle.markAsConnectNew(curveTwo);
				straight.connect(curveOne);
			}
						
			return bundle;
		}

		/** Makes a unidirectional connecting ramp that consists of an offramp, a curved segment, and an onramp.
		 *
		 * @param intersection The latlng at which the two lines that are being connected intersect.
		 * @param v1 A unit vector that points away from the intersection along the main line that will host the
		 * offramp for this connection ramp. Must have a length of one.
		 * @param v2 A unit vector that points away from the intersection along the main line that will host the
		 * onramp for this connection ramp. Must have a length of one.
		 * @param s_curveRadius The radius for the S-curves found in the off/on ramps. Must be >= lateralOffset/2.0.
		 * Measured in meters.
		 * @param turnRadius The radius for the turn segment. Measured in meters.
		 * @param turnMaxSpeed Max speed rating for the turn. Measured in meters/second.
		 * @param lateralOffset The horizontal separation between the main lines and the connecting ramp. Measured in
		 * meters.
		 * @param decelLength The length of the deceleration segment. Measured in meters.
		 * @param accelLength The length of the acceleration segment. Measured in meters.
		 * @param preview Whether this is being created for the live preview system.
		 * 
		 * @return Returns a TrackBundle containing, in order:
		 * <ul>
		 *   <li>Offramp S-curve, part one</li>
		 *   <li>Offramp S-curve, part two</li>
		 *   <li>deceleration (straight)</li>
		 *   <li>turn (curved)</li>
		 *   <li>acceleration (straight)</li>
		 *   <li>Onramp S-curve, part one</li>
		 *   <li>Onramp S-curve, part two</li>
		 * </ul>
		 * 
		 * Since the bundle represents a linear structure in a fixed order, the TrackBundle's fields which contain 
		 * connection information are not populated.
		 */
		public function makeConnectingRamp(
				intersection:LatLng, v1:Vector3D, v2:Vector3D, s_curveRadius:Number, s_curveMaxSpeed:Number,  
				turnRadius:Number, turnMaxSpeed:Number, lateralOffset:Number, decelLength:Number, accelLength:Number,
				preview:Boolean):TrackBundle
		{
			// Find the anchor for the turn (aka intersection). It is lateralOffset meters away from both v1 and v2.
			var angle:Number = Utility.angleBetween(v1, v2);
			if (angle < 1E-4) {
				throw new TrackError("Angle between v1 and v2 is too small"); // FIXME: Why wasn't this caught higher up in the call stack?
			}
			var alongV1:Vector3D = v1.clone();
			alongV1.scaleBy(lateralOffset/Math.tan(angle/2.0));			
			var z:Vector3D = v1.crossProduct(v2);
			var perpV1:Vector3D = z.crossProduct(v1); // perp from v1, points towards the side v2 is on.
			perpV1.scaleBy(lateralOffset/perpV1.length);
			var turnAnchor:LatLng = Utility.calcLatLngFromVector(intersection, alongV1.add(perpV1));
				
			// Create the turn.			
			var turnOverlay:TrackOverlay = makeTangentCurve(
					turnAnchor, v1, v2, turnMaxSpeed, this.offset, this.offset, turnRadius, false, preview);					
			
			// Create the deceleration segment
			var decelOrigin:LatLng = Utility.calcLatLngFromVector(turnOverlay.getStart(), v1, decelLength);
			var decelOverlay:TrackOverlay = makeStraight(decelOrigin, turnOverlay.getStart(), false, preview);
			
			// Create the acceleration segment
			var accelDest:LatLng = Utility.calcLatLngFromVector(turnOverlay.getEnd(), v2, accelLength);
			var accelOverlay:TrackOverlay = makeStraight(turnOverlay.getEnd(), accelDest, false, preview);
												
			// Create the offramp S-curve.
			var negV1:Vector3D = v1.clone();
			negV1.negate();
			var offRampSCurve:TrackBundle = makeSCurve(negV1, lateralOffset, s_curveRadius, Globals.tracks.driveSide,
					false, s_curveMaxSpeed, this.offset, this.offset, preview, null, decelOverlay.getStart());
			
			// Create the onramp S-curve.
			var onRampSide:String = Globals.tracks.driveSide == Tracks.RIGHT ? Tracks.LEFT : Tracks.RIGHT;
			var onRampSCurve:TrackBundle = makeSCurve(v2, lateralOffset, s_curveRadius, onRampSide,
					false, s_curveMaxSpeed, this.offset, this.offset, preview, accelOverlay.getEnd(), null);
			
			// Make connections
			offRampSCurve.connectNewOverlays[0].connect(decelOverlay);
			decelOverlay.connect(turnOverlay);
			turnOverlay.connect(accelOverlay);
			accelOverlay.connect(onRampSCurve.connectExistingOverlays[0]);
			
			// Create a result TrackBundle
			var result:TrackBundle = new TrackBundle();
			var iterOverlay:TrackOverlay;
			for each (iterOverlay in offRampSCurve.overlays) {
				result.addOverlay(iterOverlay);
			}
			for each (iterOverlay in [decelOverlay, turnOverlay, accelOverlay]) {
				result.addOverlay(iterOverlay);
			}
			for each (iterOverlay in onRampSCurve.overlays) {
				result.addOverlay(iterOverlay);
			}
			
			return result;
			
		}
		
		/** Makes a curved TrackSegment that is tangent to both v1 and v2 with the given radius. If minDist is supplied,
		 * the tangent points will be at least minDist meters from the intersection and the radius may be larger.
		 * This function has no information about the actual TrackSegments involved. It is up to the calling function
		 * to check that the returned curve is compatible with the relevant segment.
		 * 
		 * @param intersection The common origin for v1 and v2.
		 * @param v1 A unit vector which eminates from the intersection and points along the origin TrackSegment.
		 * @param v2 A unit vector which eminates from the intersection and points along the destination TrackSegment.
		 * @param maxSpeed The desired maximum vehicle speed for the TrackSegment.
		 * @param startOffset The desired distance between the track and the ground at the segment's start.
		 * @param endOffset The desired distance between the track and the ground at the segment's end.
		 * @param radius The ramp radius, in meters. A larger radius may be used if the optional minDist parameter is supplied.
		 * @param bidirectional If true, then a reverse track segment will also be created.
		 * @param preview Whether the interchange is being created as part of the live track placement preview, or as the result of a user click.
		 * @param minDist Optional parameter that forces the tangent points to be at least minDist meters away from the intersection.
		 * 
		 * @return A TrackOverlay which starts on v1 and curves to v2. No connections are made.
		 */
		public function makeTangentCurve(intersection:LatLng, v1:Vector3D, v2:Vector3D, maxSpeed:Number,
										  startOffset:Number, endOffset:Number, radius:Number,
		                                  bidirectional:Boolean, preview:Boolean, minDist:Number=0):TrackOverlay {
			var angle:Number = Utility.angleBetween(v1, v2);
			if (angle < Tracks.MIN_ANGLE || angle > Tracks.MAX_ANGLE) {
				throw new TrackError("Vectors are too nearly parallel.");
			} 
			var tanHalfAngle:Number = Math.tan(angle/2);
			var dist:Number = radius/tanHalfAngle;
			if (dist < minDist) {
				radius = tanHalfAngle * minDist;
				dist = minDist;
			}

			// Make a vector that is perpendicular with v1, and which faces the same side as v2. Of length 'radius'.
			var zVec:Vector3D = v1.crossProduct(v2);
			var v1PerpVec:Vector3D = zVec.crossProduct(v1);			
			v1PerpVec.scaleBy(radius/v1PerpVec.length);
						
			var v1DistVec:Vector3D = v1.clone();
			var v2DistVec:Vector3D = v2.clone();			
			v1DistVec.scaleBy(dist);
			v2DistVec.scaleBy(dist);
			
			var centerVec:Vector3D = v1DistVec.add(v1PerpVec);
			
			var v1LatLng:LatLng = Utility.calcLatLngFromVector(intersection, v1DistVec);
			var v2LatLng:LatLng = Utility.calcLatLngFromVector(intersection, v2DistVec);
			var centerLatLng:LatLng = Utility.calcLatLngFromVector(intersection, centerVec);
			
			/* Find magnitude and sign of arcAngle. */
			var arcAngle:Number;
			if ((zVec.z > 0 && this.driveSide == Tracks.RIGHT) || (zVec.z < 0 && this.driveSide == Tracks.LEFT)) {
				arcAngle = (Math.PI - angle) * -1; // Clockwise rotation
			} else if ((zVec.z < 0 && this.driveSide == Tracks.RIGHT) || (zVec.z > 0 && this.driveSide == Tracks.LEFT)) {
				arcAngle = (Math.PI - angle); // CounterClockwise rotation
			} else {
				throw new Error("Unknown error.");
			}

			/* Create the TrackSegments */			
			var segs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			segs.push(new TrackSegment(
					IdGenerator.getTrackSegId(TrackSegment.forwardExt), this.label, v1LatLng, v2LatLng,
					maxSpeed, startOffset, endOffset, radius, arcAngle, centerLatLng, preview));
			if (bidirectional) {
				segs.push(segs[0].clone(true));
			}
			var overlay:TrackOverlay = new TrackOverlay(segs);
			return overlay;			 
		}		

		/** Removes a TrackSegment, and all Vehicles that reside on the segment as well as their VehicleOverlays. 
		 * If the TrackSegment's overlay does not contain any other TrackSegments, then it too is removed. Undoable.
		 * @param seg The TrackSegment to remove.
		 */
		public function remove(seg:TrackSegment):void {			
			// remove vehicles on the track 
			for each (var vehicle:Vehicle in Globals.vehicles.vehicles) {
				if (vehicle.location === seg) {
					vehicle.overlay.remove();
					vehicle.remove();
				}
			}			
		
			if (seg.overlay.segments.length == 1) {
				seg.overlay.remove();
			} else {
				// remove seg from its overlay.segments
				function removeTS(item:TrackSegment, index:int, vector:Vector.<TrackSegment>):Boolean {return item !== seg};
				Undo.assign(seg.overlay, "segments", seg.overlay.segments);
				seg.overlay.segments = seg.overlay.segments.filter(removeTS);
			}
			
			seg.remove();
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
					if (used[otherId] == null) {
						parallels.push(segments[otherId]);
						used[seg.id] = true;
						used[otherId] = true;
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
			var xml:XML = <Tracks straight_max_speed={straightMaxSpeed}
			                      offset={offset}
			                      bidirectional={bidirectional}
			                      curve_max_speed={curveMaxSpeed}
			                      radius={radius}
			                      min_offset={minOffset}
			                      max_offset={maxOffset}
			                      drive_side={driveSide}
			                      ramp_offset={rampOffset}
			                      decel_length={decelLength}
			                      accel_length={accelLength}
			                      turn_max_speed={turnMaxSpeed}
			                      turn_radius={turnRadius}
			                      s_curve_max_speed={sCurveMaxSpeed}
			                      s_curve_radius={sCurveRadius}
			                      
	                      />
			return xml;
		}

		/** Generate xml from hard-coded default preferences */
		public function toDefaultPrefsXML():XML {
			var xml:XML = <Tracks
							straight_max_speed="15"
							offset="3"
							bidirectional="false"
							curve_max_speed="15"
							radius="65"
							min_offset="1"
							max_offset="9"
							drive_side="Right"
							ramp_offset="3"
							decel_length="60"
							accel_length="60"
							turn_max_speed="15"
							turn_radius="65"
							s_curve_max_speed="15"
							s_curve_radius="200"
						  />
			return xml;
		}
		
		public function fromPrefsXML(xml:XML):void {
			straightMaxSpeed = xml.@straight_max_speed;
			offset = xml.@offset;
			bidirectional = xml.@bidirectional == 'false' || xml.@bidirectional == '0' ? false : true;
			curveMaxSpeed = xml.@curve_max_speed;
			radius = xml.@radius;
			minOffset = xml.@min_offset;
			maxOffset = xml.@max_offset;
			driveSide = xml.@drive_side;
			rampOffset = xml.@ramp_offset;
			decelLength = xml.@decel_length;
			accelLength = xml.@accel_length;
			turnMaxSpeed = xml.@turn_max_speed;
			turnRadius = xml.@turn_radius;
			sCurveMaxSpeed = xml.@s_curve_max_speed;
			sCurveRadius = xml.@s_curve_radius;

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
				if (track.length == 0) {
					errors.push(track.id + " has zero length.");
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
