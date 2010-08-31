/** Unlike the ArrowLine class, which handles displaying all TrackSegments at once, the
 * StationDisplay class only handles a single station.
 */ 

package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	import com.google.maps.MapEvent;
	import com.google.maps.interfaces.IMap;
	import com.google.maps.interfaces.IPane;
	import com.google.maps.overlays.OverlayBase;
	
	import flash.display.GradientType;
	import flash.display.NativeMenu;
	import flash.display.NativeMenuItem;
	import flash.display.Shape;
	import flash.events.Event;
	import flash.geom.Matrix;
	import flash.geom.Point;
	import flash.geom.Vector3D;
	
	import mx.events.ToolTipEvent;
	import mx.managers.ToolTipManager;

	public class StationOverlay extends OverlayBase
	{
		public var map:IMap;
		public var station:Station;
		public var stationOutline:Shape;
		public var stationArea:Shape;
		private var radius_pix_width:uint;
		private var radius_pix_height:uint;
		private var gradiantBox:Matrix = new Matrix();
	
		// affects gradiant details for the coverage
		public static var coverageColors:Array = new Array();
		public static var coverageAlphas:Array = new Array();
		public static var coverageRatios:Array = new Array();
	
		public function StationOverlay(station:Station)
		{
			super();			
			this.station = station;
			station.overlay = this;
			
			stationOutline = new Shape();	
			stationArea = new Shape();
			
//            toolTip = "Dummy text"; // requires some text to trigger tooltips. Text changed in onToolTip
            contextMenu = getContextMenu();

            /* Side effects */           
            addEventListener(MapEvent.OVERLAY_ADDED, onOverlayAdded, false, 0, true); // weak_ref=true
            addEventListener(MapEvent.OVERLAY_REMOVED, onOverlayRemoved, false, 0, true);            
            addEventListener(ToolTipEvent.TOOL_TIP_SHOW, onToolTip, false, 0, true);            
            
            // add myself to the map
            Undo.pushMicro(Globals.stationPane, Globals.stationPane.removeOverlay, this);
            Globals.stationPane.addOverlay(this);  
            
			// add a reference to the global store.
			Globals.stations.overlays.push(this);
			Undo.pushMicro(Globals.stations.overlays, Globals.stations.overlays.pop);         
		}

        /** Reverses all of the constructor's side effects (with Undo support) */
		public function remove():void {
			// remove the reference from the global store
			function removeMe(item:StationOverlay, index:int, vector:Vector.<StationOverlay>):Boolean {return item !== this};
			Undo.assign(Globals.stations, "overlays", Globals.stations.overlays);
			Globals.stations.overlays = Globals.stations.overlays.filter(removeMe, this);
			
			// remove the overlay from the map
   			Undo.pushMicro(Globals.stationPane, Globals.stationPane.addOverlay, this);
        	Globals.stationPane.removeOverlay(this);		
        }

        public override function getDefaultPane(map:IMap):IPane
        {
			return Globals.stationPane;
        }

        private function onOverlayAdded(event:MapEvent):void
        {
        	addChild(stationArea);
        	addChild(stationOutline);
            positionOverlay(false); // force screen refresh
        }

        private function onOverlayRemoved(event:MapEvent):void
        {        	
            removeChild(stationArea);
            removeChild(stationOutline);
        }        
        
        public override function positionOverlay(zoomChanged:Boolean):void
        {
        	if (zoomChanged) {
        		calcCoverageRadiusPix();        		
        	}
        	
        	
        	// TODO: Use foreground property to store the rendered shape.
        	
        	// convert from LatLng to screen coordinates
        	var seg:TrackSegment = station.allSegments[uint(station.allSegments.length/2)];
        	var startPt:Point = pane.fromLatLngToPaneCoords(seg.getStart(), true);
        	var endPt:Point = pane.fromLatLngToPaneCoords(seg.getEnd(), true);
        	var centerPt:Point = pane.fromLatLngToPaneCoords(seg.midpoint, true);

			drawStationArea(centerPt);
        	drawStationOutline();
        	
        	// Draw the queue, unload, and load
        	// ...
        	
        }

		public function drawStationArea(centerPt:Point):void {
        	stationArea.graphics.clear();
    		stationArea.graphics.moveTo(centerPt.x, centerPt.y);
			
        	// Draw the station coverage area
//        	stationArea.graphics.beginFill(0xFF0000, 0.2);
			gradiantBox.createGradientBox(radius_pix_width*2, radius_pix_height*2, 0, centerPt.x - radius_pix_width, centerPt.y - radius_pix_height);
			stationArea.graphics.beginGradientFill(GradientType.RADIAL,
			                                       StationOverlay.coverageColors,
			                                       StationOverlay.coverageAlphas,
			                                       StationOverlay.coverageRatios,
			                                       gradiantBox)
        	stationArea.graphics.lineStyle(1, coverageColors[1], 0.12); // Thin, final color, mostly transparent line 
        	stationArea.graphics.drawCircle(centerPt.x, centerPt.y, (radius_pix_height + radius_pix_width) / 2); // use average
//        	stationShape.graphics.drawEllipse(centerPt.x - radius_pix_width,
//        	                                  centerPt.y - radius_pix_height,
//        	                                  radius_pix_width,
//        	                                  radius_pix_height); // uses upper left corner, relative to the registration point of the parent display object        	
        	stationArea.graphics.endFill(); 			
		}

		public function drawStationOutline():void {
			stationOutline.graphics.clear();			
		}

		public function getLatLngBounds():LatLngBounds {
			var bounds:LatLngBounds = new LatLngBounds();
			for each (var seg:TrackSegment in station.allSegments) {
				bounds.extend(seg.getStart());
				bounds.extend(seg.getEnd());
			}
			return bounds;
		}

        public function onToolTip(event:ToolTipEvent):void {
        	var txt:String = station.label ? station.label + "\n" : "";
//        	txt += "Queue:   \t"  + station.queueSlots +
//        		 "\nUnload:  \t" + station.unloadSlots + 
//        	     "\nLoad:       \t"   + station.loadSlots +        	     
//        	     "\nStorage: \t" + station.storageSlots +
//        	     "\nCoverage:\t" + station.coverageRadius;
        	ToolTipManager.currentToolTip.text = txt;
        }

		public function getContextMenu():NativeMenu {
			var menu:NativeMenu = new NativeMenu();
			
			var labelMenuItem:NativeMenuItem = new NativeMenuItem(this.station.id);
			labelMenuItem.enabled = false;
			menu.addItem(labelMenuItem);
						
			var selectMenuItem:NativeMenuItem = new NativeMenuItem("Select");
			selectMenuItem.addEventListener(Event.SELECT, onSelect);
			menu.addItem(selectMenuItem);
				
			var deleteMenuItem:NativeMenuItem = new NativeMenuItem("Delete");
			deleteMenuItem.addEventListener(Event.SELECT, onDelete);
			menu.addItem(deleteMenuItem);
			return menu;
		}
        
        public function onSelect(event:Event):void {
        	trace("StationOverlay.onSelect");
        }
        
        public function onDeselect(event:Event):void {
        	trace("StationOverlay.onDeselect");
        }
        
        public function onDelete(event:Event):void {
        	trace("onDelete");
        	Undo.startCommand(Undo.USER);
        	Globals.stations.remove(this.station);
        	Undo.endCommand()        	
        }        
        
        /** Calculate the radius in Pixels (at current zoom level). Since the map uses a mercator projection,
        * the 'circle' may be skinnier on the horizontal axis. */
        private function calcCoverageRadiusPix():void {
        	
        	var seg:TrackSegment = station.allSegments[uint(station.allSegments.length/2)];
			var xLatLng:LatLng = Utility.calcLatLngFromVector(seg.midpoint, new Vector3D(station.coverageRadius, 0));
			var yLatLng:LatLng = Utility.calcLatLngFromVector(seg.midpoint, new Vector3D(0, station.coverageRadius));
			radius_pix_width = pane.fromLatLngToPaneCoords(xLatLng).x - pane.fromLatLngToPaneCoords(seg.midpoint).x;
			radius_pix_height = pane.fromLatLngToPaneCoords(seg.midpoint).y - pane.fromLatLngToPaneCoords(yLatLng).y; // y increases downward
		}

		public static function fromPrefsXML(xml:XML):void {
			for each (var color:XML in xml.Coverage.Color) {
				coverageColors.push(uint(color));
			}
			for each (var alpha:XML in xml.Coverage.Alpha) {
				coverageAlphas.push(Number(alpha));
			}
			for each (var ratio:XML in xml.Coverage.Ratio) {
				coverageRatios.push(uint(ratio));
			}
		}

		/** Generate xml from current preferences */
		public static function toPrefsXML():XML {
			var xml:XML = <StationOverlay>
			                <Coverage/>
			              </StationOverlay>
			for each (var color:uint in coverageColors) {
				xml.Coverage.appendChild(<Color>{color}</Color>);
			}
			for each (var alpha:Number in coverageAlphas) {
				xml.Coverage.appendChild(<Alpha>{alpha}</Alpha>);
			}
			for each (var ratio:uint in coverageRatios) {
				xml.Coverage.appendChild(<Ratio>{ratio}</Ratio>);
			}
			return xml;
		}

		/** Generate xml from hard-coded default preferences. */
		public static function toDefaultPrefsXML():XML {
			// Went with a more hierarchial style of XML for this so as to support
			//  the user adding more than 2 colors in the gradiant.
			var xml:XML = <StationOverlay>
			                  <Coverage>
		                     	<Color>0xFF9900</Color>
		                     	<Color>0xFF9900</Color>
		                     	<Alpha>0.3</Alpha>
		                     	<Alpha>0.05</Alpha>
		                        <Ratio>127</Ratio>
		                     	<Ratio>255</Ratio>			                     
			                  </Coverage>
			              </StationOverlay>
			return xml;
		}
	}
}