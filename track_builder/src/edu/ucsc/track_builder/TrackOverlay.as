/** Handles the display and user interaction of lines drawn on the map.
 * TODO: Create an "ArrowLineOptions" class modeled after PolyLineOptions
 * TODO: Break some functionality into a TrackBundle class, leaving this class purely for display stuff.
 */

package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	import com.google.maps.MapEvent;
	import com.google.maps.interfaces.IMap;
	import com.google.maps.interfaces.IPane;
	import com.google.maps.overlays.OverlayBase;
	
	import flash.display.CapsStyle;
	import flash.display.NativeMenu;
	import flash.display.NativeMenuItem;
	import flash.display.Shape;
	import flash.display.Sprite;
	import flash.events.*;
	import flash.geom.Point;
	import flash.geom.Vector3D;
	
	import mx.controls.Alert;
	import mx.events.CloseEvent;
	import mx.events.ToolTipEvent;
	import mx.managers.ToolTipManager;

	public class TrackOverlay extends OverlayBase
	{
		public static const FWD:uint = 0;
		public static const REV:uint = 1;
		
		public static const NORMAL_MODE:uint = 0;
		public static const GRADE_MODE:uint = 1;
		public static const ELEVATION_MODE:uint = 2;
		public static var displayMode:uint = NORMAL_MODE;
				
		
		public static var gradeColorMap:ColorMapper = new ColorMapper(0, 0.2, ColorMapper.GnYlRd);
		
		public var segments:Vector.<TrackSegment>;
				
		/* The sense of start and end for a TrackOverlay is inherited from the first segment. */
		public function getStart():LatLng {return segments[0].getStart()} // segment provides a clone
		public function setStart(latlng:LatLng):void {this.segments[0].setStart(latlng);}
		
		public function getEnd():LatLng {return segments[0].getEnd()} // segment provides a clone
		public function setEnd(latlng:LatLng):void {return this.segments[0].setEnd(latlng);}
		
		public function get length():Number {return segments[0].length;}
		public function get h_length():Number {return segments[0].h_length;} /** horizontal length, ignores vertical component */
		
		public function isCurved():Boolean {return segments[0].isCurved();}
		public function getCenter():LatLng {return segments[0].getCenter();}
		
		public function get bidirectional():Boolean {return segments.length > 1;}
		
		private var line:Shape;     // the line drawn on the map
		private var highlight_line:Shape; // a thicker, different color behind the line to indicate that it's highlighted.
		private var hit_line:Sprite; // an invisible, thicker line for checking if the line was clicked on
		private var circle:Shape;    // an indicator of the true track shape
        
        /* Set by preferences file */
        public static var line_thickness:uint; // in pixels
        public static var hitLineThickness:uint; // in pixels
        public static var unidirLineColor:uint; // unidirectional track	
        public static var bidirLineColor:uint;  // bidirectional track
        public static var line_alpha:uint;            
        public static var head_thickness:Number; // width of the line used to draw the head, in pixels
        public static var head_color:uint;
        public static var head_alpha:Number;      
		public static var head_angle:Number; // Angle away from the line in radians. 
		public static var head_length:Number; // in pixels
        public static var highlightColor:uint;
        			
		public static var hitLineCapStyle:String = CapsStyle.ROUND; 
		
		// indicates that this overlay is part of the preview, demoing how the track will be placed if the user clicks.
		public var preview:Boolean;
		
		public var gradeColor:uint;
		public var elevationColor:uint;
		
		public var lineColor:uint;
		
		public var highlight:Boolean;
		
		// DEBUGGING purposes
		public static var showHitLine:Boolean;
		public static var showCurveCircles:Boolean;
		
		/* Constructor */
		/** @param segments Parallel TrackSegments that are represented on-screen by the same line. */
		public function TrackOverlay(segments:Vector.<TrackSegment>)
		{
			super();
			
			this.segments = segments;
			for each (var seg:TrackSegment in segments) {
				seg.overlay = this;
			} 
						
			this.preview = segments[0].preview;
			
			line = new Shape();
			highlight_line = new Shape();
			hit_line = new Sprite();
			circle = new Shape();
						
			hit_line.mouseEnabled = false; // prevent the hit_line from stealing events			
			hitArea = hit_line; // takes advantage of the fact that OverlayBase extends Sprite (though undocumented).
            
            highlight = false;
            
            addEventListener(MapEvent.OVERLAY_ADDED, onOverlayAdded, false, 0, true); // weak_ref=true
            addEventListener(MapEvent.OVERLAY_REMOVED, onOverlayRemoved, false, 0, true);           									
			
			if (!preview) {				
				addEventListener(MouseEvent.ROLL_OVER, onRollOver, false, 0, true);

	            toolTip = "Dummy text"; // requires some text to trigger tooltips. Text changed in onToolTip
	            addEventListener(ToolTipEvent.TOOL_TIP_SHOW, onToolTip, false, 0, true);
			}		
	
			contextMenu = makeContextMenu();
	
			/* Side effects */
			// add myself to the map display
			if (isCurved()) {
            	Globals.curvedTrackPane.addOverlay(this);
            	Undo.pushMicro(Globals.curvedTrackPane, Globals.curvedTrackPane.removeOverlay, this);
   			} else {
            	Globals.straightTrackPane.addOverlay(this);
            	Undo.pushMicro(Globals.straightTrackPane, Globals.straightTrackPane.removeOverlay, this);
      		}
			
			// add a reference to the global store.
			Globals.tracks.overlays.push(this);
			Undo.pushMicro(Globals.tracks.overlays, Globals.tracks.overlays.pop);
		}

        /** Reverses all of the constructor's side effects (with Undo support) */
        public function remove():void {	
			// remove myself from the map display
			if (isCurved()) {
				Undo.pushMicro(Globals.curvedTrackPane, Globals.curvedTrackPane.addOverlay, this);
            	Globals.curvedTrackPane.removeOverlay(this);
            	
   			} else {
   				Undo.pushMicro(Globals.straightTrackPane, Globals.straightTrackPane.addOverlay, this);
            	Globals.straightTrackPane.removeOverlay(this);
      		}
			
			// Remove myself from Tracks.overlays
			function removeMe(item:TrackOverlay, index:int, vector:Vector.<TrackOverlay>):Boolean {return item !== this};
			Undo.assign(Globals.tracks, "overlays", Globals.tracks.overlays);
			Globals.tracks.overlays = Globals.tracks.overlays.filter(removeMe, this);				
        }
		
		public override function get contextMenu():NativeMenu {return makeContextMenu();}
		
		public function makeContextMenu():NativeMenu {
			var menu:NativeMenu = new NativeMenu();
			
			if (segments.length == 1) { // only one segment associated with overlay
				var labelMenuItem:NativeMenuItem = new NativeMenuItem(segments[0].id);
				labelMenuItem.enabled = false;
				menu.addItem(labelMenuItem);
			
				var selectMenuItem:NativeMenuItem = new NativeMenuItem("Select");
				selectMenuItem.addEventListener(Event.SELECT, onSelect);
				menu.addItem(selectMenuItem);
				
				var deleteMenuItem:NativeMenuItem = new NativeMenuItem("Delete");
				deleteMenuItem.addEventListener(Event.SELECT, onDelete);
				menu.addItem(deleteMenuItem);
				
				 // Add Station. Only show if straight and if not part of an existing station.
				if (!isCurved() && (Globals.stations.trackSegment2Station[segments[0].id] == null)) {
					var addStationMenu:NativeMenuItem = new NativeMenuItem("Add Station");
					addStationMenu.addEventListener(Event.SELECT, onAddStation);
					menu.addItem(addStationMenu);
				}
			} else { // multiple segments
				var selectMenu:NativeMenu = new NativeMenu();
				var seg:TrackSegment;
				selectMenu.addEventListener(Event.SELECT, onSelect);
				selectMenu.addItem(new NativeMenuItem("All"));
				selectMenu.addItem(new NativeMenuItem("", true));
				for each (seg in segments) {
					selectMenu.addItem(new NativeMenuItem(seg.id));
				}
				menu.addSubmenu(selectMenu, "Select");
				
				var deleteMenu:NativeMenu = new NativeMenu();
				deleteMenu.addEventListener(Event.SELECT, onDelete);
				deleteMenu.addItem(new NativeMenuItem("All"));
				deleteMenu.addItem(new NativeMenuItem("", true));
				for each (seg in segments) {			
					deleteMenu.addItem(new NativeMenuItem(seg.id));
				}
				menu.addSubmenu(deleteMenu, "Delete");					
			}		

			return menu;
		}
		
		public function onSelect(event:Event):void {
			trace("TrackOverlay.onSelect");
			highlight = true;
			update();
		}
		
		public function onDeselect(event:Event):void {
			trace("TrackOverlay.onDeselect");
			highlight = false;
			update();
		}
		
		public function onDelete(event:Event):void {
			trace("onDelete: ", event.target, event.target.label);			
			/* Delete the whole station if a component track segment is deleted. */
			var station:Station = Globals.stations.trackSegment2Station[this.segments[0].id];
			if (station != null) {
				Alert.yesLabel = "Delete";				
				Alert.show(this.segments[0].id + " is part of " + station.id + " and cannot be deleted without deleting the station.",
				           "Delete Station?", Alert.YES|Alert.CANCEL, null, function(event:CloseEvent):void {
				                 if (event.detail==Alert.YES) { // clicked 'Delete'
				                 	Undo.startCommand(Undo.USER);
				                 	Globals.stations.remove(station);
				                 	Undo.endCommand();
				                 }
				           });
				Alert.yesLabel = "Yes";
			}
			
			else {
				Undo.startCommand(Undo.USER);
				if (event.target.label == "Delete") { // only one TrackSegment
					Globals.tracks.remove(this.segments[0]);  // handles removing the overlay from the display and breaking track connections.
				} else if (event.target.label == "All") { // Multiple TrackSegments
					var segs:Vector.<TrackSegment> = this.segments.slice();
					for each (var seg:TrackSegment in segs) {
						Globals.tracks.remove(seg);
					}
				} else { // Multiple TrackSegments, only delete one of them
					var ts:TrackSegment = Globals.tracks.getTrackSegment(event.target.label);
	    	    	Globals.tracks.remove(ts); // remove the segment globally. breaks track connections			
				}
				Undo.endCommand();			
			}
			
		}

		public function onAddStation(event:Event):void {
			trace("onAddStation: ", event.target, event.target.label);			
			try {
				Undo.startCommand(Undo.USER);
				var pt:Point = new Point(mouseX, mouseY);
				Globals.stations.makeOfflineStation(this, snapTo(pt), false);
				Undo.assign(Globals, 'dirty', Globals.dirty);
				Globals.dirty = true; // mark that changes have occured since last save
				Undo.endCommand();
			} catch (err:StationLengthError) {
				Undo.endCommand();
				Undo.undo(Undo.USER);
				Alert.show(err.message);
			}			
		}
		
		public function onRollOver(event:MouseEvent):void {
//			trace("trackOverlay.onRollOver", event.currentTarget);
			var marker:SnappingMarker = Globals.getActiveMarker();
			switch (Globals.tool) {
				case Globals.TRACK_TOOL:
				    if (!isCurved()) {
				    	marker.setSnapOverlay(this);
				    	marker.snapTo(marker.getLatLng());
				    }
					break;
				case Globals.STATION_TOOL:
					if (!isCurved()) {
						marker.setSnapOverlay(this);
						marker.snapTo(marker.getLatLng());
					}
					break;
				case Globals.VEHICLE_TOOL:
					marker.setSnapOverlay(this);
					marker.snapTo(marker.getLatLng());
					break;
				case Globals.SELECT_TOOL:
					break;
				default:
					throw new Error("Unknown tool type.");
 
			}
		}

		/* Not happy with the string formatting */
		public function onToolTip(event:ToolTipEvent):void {
			var seg:TrackSegment = segments[0];
			var txt:String = seg.label ? seg.label + "\n" : "";
			for each (seg in segments) {
				txt += "id: " + seg.id + "\n";
			}
			txt += "MaxSpeed: " + seg.maxSpeed + " meters/sec" +
			      "\nLength: "   + seg.length.toFixed(2) + " meters" + 
			      "\nGrade: " + ((seg.endElev - seg.startElev) / seg.length * 100).toFixed(1) + "%";
			      
			if (seg.isCurved()) { // a curved segment
				txt += "\nRadius: " + seg.radius + " meters" +
				       "\nAngle:  " + (seg.arcAngle * 180/Math.PI).toFixed(1) + " degrees"; 
			}
			
			txt += "\nGround:\t" + seg.startGround.toFixed(2) + "\t" + seg.endGround.toFixed(2) +
			       "\nOffset:  \t" + seg.getStartOffset().toFixed(2) + "\t" + seg.getEndOffset().toFixed(2) +
			       "\nElev:       \t" + seg.startElev.toFixed(2)   + "\t" + seg.endElev.toFixed(2);
			                			
			ToolTipManager.currentToolTip.text = txt;
//			trace("TrackOverlay.onToolTip:", seg.radius, seg.arcAngle*180/Math.PI);
		}
		
		/** Forces a redraw. Call after adding a new Segment*/
		public function update():void
		{
			positionOverlay(false); // redraw
		}
		
        public override function getDefaultPane(map:IMap):IPane
        {
        	if (isCurved()) {
        		return Globals.curvedTrackPane;
        	} else {
        		return Globals.straightTrackPane;
        	}
        }

        private function onOverlayAdded(event:MapEvent):void
        {
            addChild(hit_line);
            addChild(highlight_line);
            addChild(line);
            
            if (segments[0].isCurved()) {
            	addChild(circle);
            }  
        }

        private function onOverlayRemoved(event:MapEvent):void
        {
            removeChild(hit_line);
            removeChild(highlight_line);
            removeChild(line);
            if (segments[0].isCurved()) {
            	removeChild(circle);
            }    
        }       
        
        public override function positionOverlay(zoomChanged:Boolean):void
        {
			if (!pane) {
				pane = isCurved() ? Globals.curvedTrackPane : Globals.straightTrackPane;
			} 
			// clear old drawings
            line.graphics.clear();
            highlight_line.graphics.clear();
            hit_line.graphics.clear();   
            
			var seg:TrackSegment = segments[0];  // arbitrarily choose a segment
            var endPt:Point = pane.fromLatLngToPaneCoords(seg.getEnd());
            var startPt:Point = pane.fromLatLngToPaneCoords(seg.getStart()); 
            var centerPt:Point;
            var radiusDrawDist:Number;
            if (seg.isCurved()) {
            	centerPt = pane.fromLatLngToPaneCoords(seg.getCenter());
            	radiusDrawDist = Point.distance(startPt, centerPt);            	
            }        
            
			var line_angle:Number;
	
            // move to the tail of the arrow          
            line.graphics.moveTo(startPt.x, startPt.y);
            highlight_line.graphics.moveTo(startPt.x, startPt.y);
            hit_line.graphics.moveTo(startPt.x, startPt.y);

            // hit line
            if (preview) {
            	hit_line.graphics.lineStyle(); // don't draw it
            } else if (showHitLine) {            
				hit_line.graphics.lineStyle(hitLineThickness, 0xFFFFFF, 0.2, false, "normal", hitLineCapStyle); // 0 alpha = invisible
            } else {
            	hit_line.graphics.lineStyle(hitLineThickness, 0, 0); // draw, but with 0 alpha (invisible)
            }
			if (!centerPt) { // a straight line segment
				hit_line.graphics.lineTo(endPt.x, endPt.y);
			} else {  // a curved segment
            	Utility.arcTo(hit_line.graphics, startPt, centerPt, seg.arcAngle, radiusDrawDist);
   			}

        	/* draw the highlight, if needed */
        	if (highlight) {
        		highlight_line.graphics.lineStyle(line_thickness+4, highlightColor, line_alpha);
	            if (!centerPt) { // a straight line segment            	
    	        	highlight_line.graphics.lineTo(endPt.x, endPt.y);
        	    } else { // a curved segment
            		Utility.arcTo(highlight_line.graphics, startPt, centerPt, seg.arcAngle, radiusDrawDist);    	
            	}
         	}
         	
			/* draw the line */
			switch (displayMode) {
				case NORMAL_MODE:
					if (!bidirectional) {
						line.graphics.lineStyle(line_thickness, unidirLineColor, line_alpha);
					} else { // bidirectional == true
						line.graphics.lineStyle(line_thickness, bidirLineColor, line_alpha);
					}
					break;
				case GRADE_MODE:
					var grade:Number = Math.abs(getGrade());
					if (isNaN(grade)) { // no elevation info
						if (!bidirectional) {
							line.graphics.lineStyle(line_thickness, unidirLineColor, line_alpha);
						} else { // bidirectional == true
							line.graphics.lineStyle(line_thickness, bidirLineColor, line_alpha);
						}
					} else { // normal case
						line.graphics.lineStyle(line_thickness, gradeColorMap.valueToColor(Math.abs(getGrade())), line_alpha);
					}						
					break;
				case ELEVATION_MODE:
					// Use gradiant
//					line.graphics.lineStyle(line_thickness, elevationColorMap.valueToColor(getAve
					break;
				default:
					throw new Error("Unknown displayMode: " + displayMode);	
			}
            
            if (!centerPt) { // a straight line segment            	
	        	line.graphics.lineTo(endPt.x, endPt.y);
    	    } else { // a curved segment
        		Utility.arcTo(line.graphics, startPt, centerPt, seg.arcAngle, radiusDrawDist);    	
        	}

			// if single directional, draw an arrowhead indicating direction.
			if (!bidirectional && (Point.distance(startPt, endPt) > head_length*2.0)) { // don't draw arrowheads if bidir or track segment is rendered too short. 
                line.graphics.lineStyle(head_thickness, head_color, head_alpha);
                
                if (!centerPt) {// a straight line segment
                	line_angle = Math.atan2( endPt.y - startPt.y, endPt.x - startPt.x ) + Math.PI; // + PI to reverse the direction
                } else {
                	line_angle = Math.atan2( endPt.y - centerPt.y, endPt.x - centerPt.x);
                	if (seg.arcAngle > 0) { // CCW
                		line_angle += Math.PI/2; // rotate 90 deg CCW
                	} else { // CW
                		line_angle -= Math.PI/2; // rotate 90 deg CW
                	}
                }
                
                // draw the left part of the arrowhead                
                var head_left_angle:Number = line_angle - head_angle;
                var head_left:Point = new Point(endPt.x + head_length * Math.cos(head_left_angle), endPt.y + head_length * Math.sin(head_left_angle));
                
                line.graphics.lineTo(head_left.x, head_left.y);
                line.graphics.moveTo(endPt.x, endPt.y); // move back to the tip
                
                // draw the right part of the arrowhead
                var head_right_angle:Number = line_angle + head_angle;
                var head_right:Point = new Point(endPt.x + head_length * Math.cos(head_right_angle),
                                               endPt.y + head_length * Math.sin(head_right_angle));
				line.graphics.lineTo(head_right.x, head_right.y);
				line.graphics.moveTo(endPt.x, endPt.y); // move back to the tip
            }					

			// Draw a light circle to indicate the true shape of the track curvature
			if (TrackOverlay.showCurveCircles && seg.isCurved()) {
				circle.graphics.clear(); // clear the old
				circle.graphics.lineStyle(1, unidirLineColor, 0.3, true);								
				circle.graphics.drawCircle(centerPt.x, centerPt.y, radiusDrawDist);
			} else {
				circle.graphics.clear();
			}
        }        

		/** Returns the closest LatLng to the point. If mousePt is within endpointSnapDist from
		 * an endpoint, then will return the latlng of the endpoint instead.
		 * either mousePt or latlng must be provided.*/
		public function snapTo(mousePt:Point=null, latlng:LatLng=null, endpointSnapDist:uint=15):LatLng {
			if (!mousePt) {
				mousePt = Globals.map.fromLatLngToViewport(latlng);
			}
			if (!pane) {
				pane = isCurved() ? Globals.curvedTrackPane : Globals.straightTrackPane;
			}
									
			var seg:TrackSegment = segments[0];
			
			var a:Point = pane.fromLatLngToPaneCoords(seg.getStart());
			var b:Point = pane.fromLatLngToPaneCoords(seg.getEnd());
			var c:Point = mousePt;			

			if (endpointSnapDist > 0) {
				if (Point.distance(a, c) < endpointSnapDist) {
					return seg.getStart();
				} else if (Point.distance(b, c) < endpointSnapDist) {
					return seg.getEnd();
				} // else snap to the line, below
			}
			
			var result:LatLng;	
			if (!seg.isCurved()) { // a straight track
				var ab:Vector3D = new Vector3D(b.x - a.x, b.y - a.y);			
				var ac:Vector3D = new Vector3D(c.x - a.x, c.y - a.y);
				
				var ba:Vector3D = new Vector3D(a.x - b.x, a.y - b.y);
				var bc:Vector3D = new Vector3D(c.x - b.x, c.y - b.y);				
		
				if (Utility.angleBetween(ab, ac) > Math.PI / 2) {
					result = pane.fromPaneCoordsToLatLng(a);
				} else if (Utility.angleBetween(ba, bc) > Math.PI / 2) {
					result = pane.fromPaneCoordsToLatLng(b);
				} else {
					var dist:Number = ab.dotProduct(ac) / ab.lengthSquared;
					ab.scaleBy(dist); // the vector ad
					var d:Point = new Point(ab.x + a.x, ab.y + a.y); // the result
					result = pane.fromPaneCoordsToLatLng(d);
				}
			} else { // a curved segment
				var dest:LatLng = pane.fromPaneCoordsToLatLng(mousePt);				
				return Utility.calcLatLngFromDist(seg.radius, seg.getCenter(), dest);
			}
			
			return result;	
		}

		/** Exhaustively try all connections between track segments held in this overlay
		 * and in other. */
		public function connect(other:TrackOverlay):void {
			for each (var ts:TrackSegment in this.segments) {
				for each (var otherTs:TrackSegment in other.segments) {
					ts.connect(otherTs);
				}
			}
		}
		
		// FIXME: Specifying the offset should be optional or removed.
		public function split(latlng:LatLng, elevOffset:Number, preview_:Boolean):TrackOverlay {	
			var newOverlaySegs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
			
			// Forward seg
			newOverlaySegs.push(segments[0].split(latlng, elevOffset, preview_));
			
			// Reverse seg
			if (segments.length > 1) {
				var newReverseSeg:TrackSegment = segments[1].split(latlng, elevOffset, preview_);
				newOverlaySegs.push(segments[1]); // The new overlay gets the existing (now trimmed) reverse seg
				Undo.assignElement(this, 'segments', 1, this.segments[1]);
				segments[1] = newReverseSeg;      // The old overlay gets the newly created reverse seg
				
				// Fix up the parallel_ids
				Undo.assign(segments[0], 'parallel_ids', segments[0].parallel_ids);
				Undo.assign(segments[1], 'parallel_ids', segments[1].parallel_ids);
				Undo.assign(newOverlaySegs[0], 'parallel_ids', newOverlaySegs[0].parallel_ids);
				Undo.assign(newOverlaySegs[1], 'parallel_ids', newOverlaySegs[1].parallel_ids);
				segments[0].parallel_ids = Vector.<String>([segments[1].id]);
				segments[1].parallel_ids = Vector.<String>([segments[0].id]);
				newOverlaySegs[0].parallel_ids = Vector.<String>([newOverlaySegs[1].id]);
				newOverlaySegs[1].parallel_ids = Vector.<String>([newOverlaySegs[0].id]);
			}

			var newTrackOverlay:TrackOverlay = new TrackOverlay(newOverlaySegs); 

			// Update Vehicles on 'this' with new locations if on far side of split.
			// Update the VehicleOverlays with the new TrackOverlay
			// Forward direction first 
			var v_overlays:Vector.<VehicleOverlay> = Globals.vehicles.getVehicleOverlaysFromTrackOverlay(this);
			var newSeg:TrackSegment = newOverlaySegs[0];
			var trimmedSeg:TrackSegment = segments[0];			 
			for each (var v_overlay:VehicleOverlay in v_overlays) {
				var vehicle:Vehicle = v_overlay.vehicle;					
				if (vehicle.location == trimmedSeg && vehicle.position > trimmedSeg.length) {
					Undo.assign(vehicle, 'location', vehicle.location);
					vehicle.location = newSeg;
					v_overlay.setTrackOverlay(newTrackOverlay);
				}
			}
			
			// Reverse direction
			if (segments.length > 1) {
				newSeg = segments[1];
				trimmedSeg = newOverlaySegs[1];
				for each (v_overlay in v_overlays) {
					vehicle = v_overlay.vehicle;					
					if (vehicle.location == trimmedSeg && vehicle.position > trimmedSeg.length) {
						Undo.assign(vehicle, 'location', vehicle.location);
						vehicle.location = newSeg;
					}
				}		
			}

			return newTrackOverlay;
		}

		/** Returns the latlng of position, where position is measured as meters along the
		 * segment, disregarding z axis distance.  */
		public function getLatLng(position:Number):LatLng {
			return segments[0].getLatLng(position);
		}

		public function isStartExposed():Boolean {
			if (segments.length == 1) {
				return segments[0].isStartExposed();
			} else if (segments.length == 2) {
				return segments[0].isStartExposed() || segments[1].isEndExposed();
			} else {
				throw Error("Unexpected 'segments' length: " + segments.length.toString);
			}
		}
		
		public function isEndExposed():Boolean {
			if (segments.length == 1) {
				return segments[0].isEndExposed();
			} else if (segments.length == 2) {
				return segments[0].isEndExposed() || segments[1].isStartExposed();
			} else {
				throw Error("Unexpected 'segments' length: " + segments.length.toString);
			}
		}

		public function getLatLngBounds():LatLngBounds {
			var bounds:LatLngBounds = new LatLngBounds();
			bounds.extend(getStart());
			bounds.extend(getEnd());
			return bounds;
		}
		
		/** Returns the slope of the forward segment assoctiated with this overlay. Example, a 45 degree grade will return 1. */
		public function getGrade():Number {
			var seg:TrackSegment = segments[0];
			return seg.getGrade();
		}
			
		public static function toPrefsXML():XML {
			var xml:XML = <TrackOverlay
							line_thickness={line_thickness}
							hit_line_thickness={hitLineThickness}
							unidir_line_color={unidirLineColor}
							bidir_line_color={bidirLineColor}
							line_alpha={line_alpha}
							highlight_color={highlightColor}
							head_thickness={head_thickness}
							head_color={head_color}
							head_alpha={head_alpha}
							head_angle={head_angle}
							head_length={head_length}
							show_hitline={showHitLine}
							show_curve_circles={showCurveCircles}
						  />
			return xml;
		}
		
		public static function fromPrefsXML(xml:XML):void {
			line_thickness=xml.@line_thickness;
			hitLineThickness=xml.@hit_line_thickness;
			unidirLineColor=xml.@unidir_line_color;
			bidirLineColor=xml.@bidir_line_color;
			line_alpha=xml.@line_alpha;
			highlightColor=xml.@highlight_color;
			head_thickness=xml.@head_thickness;
			head_color=xml.@head_color;
			head_alpha=xml.@head_alpha;
			head_angle=xml.@head_angle;
			head_length=xml.@head_length;
			showHitLine= xml.@show_hitline == 'true' ? true : false;
			showCurveCircles= xml.@show_curve_circles == 'true' ? true : false;
		}
		
		public static function toDefaultPrefsXML():XML {
			// head_angle is Math.PI/6
			var xml:XML = <TrackOverlay
							line_thickness="3"
							hit_line_thickness="30"
							unidir_line_color="0xA0A0FF" // light blue
							bidir_line_color="0x6060FF"  // darker blue
							line_alpha="1"
							highlight_color="0xFFFF00"
							head_thickness="4"
							head_color="0x6060FF"
							head_alpha="0.8"
							head_angle="0.523598776"
							head_length="10"
							show_hitline="false"
							show_curve_circles="false"
														
						  />
			return xml;
		}
		
		/** Used by Vector.sort to sort Track Overlays by the Id of their first segment. */
		public static function compareById(x:TrackOverlay, y:TrackOverlay):Number {
			var x_numeric:uint = uint(x.segments[0].id.split('_')[0]);
			var y_numeric:uint = uint(y.segments[0].id.split('_')[0]);
			return x_numeric - y_numeric;
		}
	}
}