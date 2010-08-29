package edu.ucsc.track_builder
{
	import com.google.maps.MapEvent;
	import com.google.maps.interfaces.IMap;
	import com.google.maps.interfaces.IPane;
	import com.google.maps.overlays.OverlayBase;
	
	import flash.display.CapsStyle;
	import flash.display.Graphics;
	import flash.display.NativeMenu;
	import flash.display.NativeMenuItem;
	import flash.display.Shape;
	import flash.events.Event;
	import flash.geom.Point;
	import flash.geom.Vector3D;
	
	import mx.events.ToolTipEvent;
	import mx.managers.ToolTipManager;

	public class VehicleOverlay extends OverlayBase
	{
		public var vehicle:Vehicle;
		protected var trackOverlay:TrackOverlay;

		public static var v_hr_width:Number; // high res
		public static var v_hr_length:Number; // high res
		public static var v_lr_size:Number; // low res
		public static var line_thickness:Number;
		public static var line_color:uint; // red
		public static var fill_color:uint; // red
		public static var highlightThickness:uint;
		public static var highlightColor:uint;
		public static var zoomThreshold:uint;
				
		public var highlight:Boolean = false;		
		private var icon:Shape;
		
		/** Constructor */
		public function VehicleOverlay(vehicle:Vehicle, trackOverlay:TrackOverlay, preview:Boolean=false)
		{	
			super();
			this.vehicle = vehicle;
			this.trackOverlay = trackOverlay;
			
            addEventListener(MapEvent.OVERLAY_ADDED, onOverlayAdded, false, 0, true); // weak_ref=true
            addEventListener(MapEvent.OVERLAY_REMOVED, onOverlayRemoved, false, 0, true);

			icon = new Shape();

			/* Side effect - add to display */
			Undo.pushMicro(Globals.vehiclePane, Globals.vehiclePane.removeOverlay, this);
			Globals.vehiclePane.addOverlay(this);

			
			if (!preview) {
				toolTip = "dummyText"; // requires some text to trigger tooltips
				addEventListener(ToolTipEvent.TOOL_TIP_SHOW, onToolTip, false, 0, true);
				
				contextMenu = getContextMenu();
	
				/* Side effect - add to global store */
				Undo.pushMicro(Globals.vehicles.overlays, Globals.vehicles.overlays.pop);
				Globals.vehicles.overlays.push(this);
			}
		}	

		public function getTrackOverlay():TrackOverlay {
			return trackOverlay;
		}

		public function setTrackOverlay(trackOverlay:TrackOverlay):void {
			Undo.pushMicro(this, this.setTrackOverlay, this.trackOverlay);
			this.trackOverlay = trackOverlay;
		}

		public function getContextMenu():NativeMenu {
			var menu:NativeMenu = new NativeMenu();
			
			var labelMenuItem:NativeMenuItem = new NativeMenuItem(this.vehicle.id);
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
			trace("VehicleOverlay.onSelect");
			highlight = true;
			Globals.vehiclePane.updatePosition();
		}
		
		public function onDeselect(event:Event):void {
			trace("VehicleOverlay.onDeselect");
			highlight = false;
			Globals.vehiclePane.updatePosition();
		}
		
		public function onDelete(event:Event):void {
			Globals.vehicles.removeVehicleOverlay(this); // also removes vehicle
		}

		public function onToolTip(event:ToolTipEvent):void {
			var txt:String = vehicle.id;
			if (vehicle.label != "") {
				txt += "\nLabel:\t\t" + vehicle.label;
			}
			txt += "\nModel:\t\t" + vehicle.modelName
			     + "\nPosition:\t" + vehicle.position.toFixed(1) + " meters"
			     + "\nVelocity:\t" + vehicle.velocity + " m/s"
			     + "\nElevation:\t" + vehicle.elevation.toFixed(0) + " meters";
			     
			ToolTipManager.currentToolTip.text = txt;
		}

		/** An icon indicating where the nose of the vehicle is. */
		private function drawHighResIcon():void
		{
			var g:Graphics = icon.graphics;
			g.clear();
						
			var angle:Number;
			if (!vehicle.location.isCurved()) { // a straight segment
				var startPt:Point = pane.fromLatLngToPaneCoords(vehicle.location.getStart());
				var endPt:Point = pane.fromLatLngToPaneCoords(vehicle.location.getEnd());
				angle = Math.atan2(endPt.y - startPt.y, endPt.x - startPt.x);
			} else { // a curved segment
				var dv:Vector3D = Utility.calcVectorFromLatLngs(vehicle.location.getCenter(), vehicle.latlng);
				var perp_dv:Vector3D = new Vector3D(dv.y, -dv.x);
				var se:Vector3D = Utility.calcVectorFromLatLngs(vehicle.location.getStart(), vehicle.location.getEnd()); // start->end
				if (Utility.angleBetween(perp_dv, se) < Math.PI/2) { // facing the right dir
					angle = Math.atan2(-perp_dv.y, perp_dv.x); // invert y direction due to y-down screen coordinates
				} else { // wrong dir
					angle = Math.atan2(-perp_dv.y, perp_dv.x) + Math.PI; // flip direction, and invert y direction due to y-down screen coordinates
				}		
			}

			// convert from LatLng to screen coordinates
			var nosePt:Point = pane.fromLatLngToPaneCoords(vehicle.latlng);
			var cosAngle:Number = Math.cos(angle);
			var sinAngle:Number = Math.sin(angle);
			var cosPerpAngle:Number = Math.cos(angle + Math.PI / 2);
			var sinPerpAngle:Number = Math.sin(angle + Math.PI / 2);
			var halfWidth:Number = v_hr_width / 2;
			
			// recall that the y-axis is positive downwards
			var frontPt:Point = new Point(nosePt.x - cosAngle*halfWidth, nosePt.y - sinAngle*halfWidth);
			var leftFrontPt:Point = new Point(frontPt.x - cosPerpAngle*halfWidth, frontPt.y - sinPerpAngle*halfWidth);
			var rightFrontPt:Point = new Point(frontPt.x + cosPerpAngle*halfWidth, frontPt.y + sinPerpAngle*halfWidth); 			
			var leftBackPt:Point = new Point(leftFrontPt.x - cosAngle*v_hr_length, leftFrontPt.y - sinAngle*v_hr_length);
			var rightBackPt:Point = new Point(rightFrontPt.x - cosAngle*v_hr_length, rightFrontPt.y - sinAngle*v_hr_length);			
			
			// Draw a curved nose section
			if (highlight) {
				g.lineStyle(highlightThickness, highlightColor, 1, false, "normal", CapsStyle.NONE);
			} else {
				g.lineStyle(line_thickness, line_color, 1, false, "normal", CapsStyle.NONE);
			}
			g.beginFill(fill_color, 0.8);
			g.moveTo(rightFrontPt.x, rightFrontPt.y);
			g.curveTo(nosePt.x, nosePt.y, leftFrontPt.x, leftFrontPt.y);
			
			// Draw the rest of the body
			g.lineTo(leftBackPt.x, leftBackPt.y);
			g.lineTo(rightBackPt.x, rightBackPt.y);
			g.lineTo(rightFrontPt.x, rightFrontPt.y);
		}		

		private function drawLowResIcon():void {
			var g:Graphics = icon.graphics;
			g.clear();
			
			if (highlight) {
				g.lineStyle(highlightThickness, highlightColor, 1, false, "normal", CapsStyle.NONE);
			} else {
				g.lineStyle(line_thickness, line_color, 1, false, "normal", CapsStyle.NONE);
			}
			g.beginFill(fill_color, 1);
			var pt:Point = pane.fromLatLngToPaneCoords(vehicle.latlng);
			var offset:Number = v_lr_size/2; // to center it on the latlng
			g.drawRect(pt.x-offset, pt.y-offset, v_lr_size, v_lr_size);	
		}
		
		/** Forces a redraw. Call after adding a new Segment*/
		public function update():void
		{
			positionOverlay(false); // redraw
		}
		 
        public override function getDefaultPane(map:IMap):IPane
        {
            return Globals.vehiclePane;
        }
		
        private function onOverlayAdded(event:MapEvent):void
        {
        	addChild(icon);
            update();
        }

        private function onOverlayRemoved(event:MapEvent):void
        {
            removeChild(icon);
        }   
	
        public override function positionOverlay(zoomChanged:Boolean):void
        {
        	if (Globals.map.getZoom() >= zoomThreshold) { 
        		drawHighResIcon();
        	} else {
        		drawLowResIcon();
        	}
        	
        }

		public static function fromPrefsXML(xml:XML):void {
			v_hr_width = xml.@v_hr_width;
			v_hr_length = xml.@v_hr_length;
			v_lr_size = xml.@v_lr_size;
			line_thickness = xml.@line_thickness;
			line_color = xml.@line_color;
			fill_color = xml.@fill_color;
			highlightThickness = xml.@highlight_thickness;
			highlightColor = xml.@highlight_color;
			zoomThreshold = xml.@zoom_threshold;
		}

		/** Generate xml from current preferences */		
		public static function toPrefsXML():XML {
			var xml:XML = <VehicleOverlay
						  	v_hr_width={v_hr_width}
						  	v_hr_length={v_hr_length}
						  	v_lr_size={v_lr_size}
						  	line_thickness={line_thickness}
						  	line_color={line_color}
						  	fill_color={fill_color}
						  	highlight_thickness={highlightThickness}
						  	highlight_color={highlightColor}
						  	zoom_threshold={zoomThreshold}						  	
						  />
			return xml;
		}

		/** Generate xml from hard-coded default preferences */
		public static function toDefaultPrefsXML():XML {
			var xml:XML = <VehicleOverlay
						  	v_hr_width="15"
						  	v_hr_length="15"
						  	v_lr_size="5"
						  	line_thickness="1"
						  	line_color="0xFF3030"
						  	fill_color="0xFF3030"
						  	highlight_thickness="5"
						  	highlight_color="0xFFFF00"
						  	zoom_threshold="16"
						  />
			return xml;
		}
	
	}
}