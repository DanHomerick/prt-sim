package edu.ucsc.track_builder
{
	import com.google.maps.Map;
	import com.google.maps.MapMouseEvent;
	import com.google.maps.controls.ZoomControl;
	import com.google.maps.interfaces.IPane;
	
	import flash.display.CapsStyle;
	import flash.display.NativeWindow;
	import flash.display.NativeWindowDisplayState;
	import flash.events.MouseEvent;
	import flash.filesystem.File;
	import flash.geom.Rectangle;
	
	import mx.controls.ToggleButtonBar;
	
	public class Globals
	{
		public static const SELECT_TOOL:int = 0;
		public static const TRACK_TOOL:int = 1;
		public static const STATION_TOOL:int = 2;
		public static const VEHICLE_TOOL:int = 3;

		public static var tool:int; // Which tool is being used. Uses above constants.	
		
		// Holds instances of the top level containers
		public static var tracks:Tracks = new Tracks();
		public static var stations:Stations = new Stations();
		public static var vehicles:Vehicles = new Vehicles();
		
		// a couple new pane ids, in addition to the existing ones:
		// http://code.google.com/apis/maps/documentation/flash/reference.html#PaneId
		public static var stationPane:IPane;
		public static var curvedTrackPane:IPane;
		public static var straightTrackPane:IPane;		
		public static var vehiclePane:IPane;
		public static var markerPane:IPane;
		
		public static var map:Map; // The google map		
		public static var menu:Menu; // my menu, NOT mx.controls.menu
		public static var toolBar:ToggleButtonBar;
		public static var status:String;
		
		public static var elevationService:ElevationService = new ElevationService(10); // max number of simultaneous requests
				
		public static var dataXMLFile:File; // The XML file currently being worked with.
		public static var prefsXMLFile:File = File.applicationStorageDirectory.resolvePath("preferences.xml"); // The prefs file
		public static var gtfXML:XMLList = null
		
		public static var dirty:Boolean; // has been changed since last save. 
						
		public static var originMarker:OriginMarker;		
		public static var destMarker:DestMarker;

//		// which, if any, TrackOverlay the mouse is currently over.
//		public static var rollOver:TrackOverlay = null;
		
		// indicator that the origin has been placed for the first time.
		public static var haveClicked:Boolean = false;

		// Used to 'remind' SaveAs what it should do after asynchronously saving.
		public static var postSave:Function = null;

		public static var zoomControl:ZoomControl;

		public static function onToolChange(newTool:int):void {
			var trackOverlay:TrackOverlay;
			var stationOverlay:StationOverlay;
			var vehicleOverlay:VehicleOverlay;
			// Unhook the listeners associated with the current tool
		    switch(Globals.tool) {
		    	case TRACK_TOOL:	
		    		map.removeEventListener(MapMouseEvent.CLICK, tracks.onMapClick);
		    		map.removeEventListener(MapMouseEvent.MOUSE_MOVE, tracks.onMapMouseMove);
		    		Undo.undo(Undo.PREVIEW); // remove any preview stuff
		    		break;
		    	case STATION_TOOL:
		    		map.removeEventListener(MapMouseEvent.CLICK, stations.onMapClick);
		    		map.removeEventListener(MapMouseEvent.MOUSE_MOVE, stations.onMapMouseMove);
		    		Undo.undo(Undo.PREVIEW); // remove any preview stuff
		    		break;
		    	case VEHICLE_TOOL:
		    		map.removeEventListener(MapMouseEvent.CLICK, Globals.vehicles.onMapClick);
					map.removeEventListener(MapMouseEvent.MOUSE_MOVE, Globals.vehicles.onMapMouseMove);
					Undo.undo(Undo.PREVIEW); // remove any preview stuff
		    		break;
		    	case SELECT_TOOL:
		    		for each (stationOverlay in stations.overlays) {
		    			stationOverlay.removeEventListener(MouseEvent.ROLL_OVER, stationOverlay.onSelect);
		    			stationOverlay.removeEventListener(MouseEvent.ROLL_OVER, stationOverlay.onDeselect);
		    		}		    		
		    		for each (vehicleOverlay in vehicles.overlays) {
		    			vehicleOverlay.removeEventListener(MouseEvent.ROLL_OVER, vehicleOverlay.onSelect);
		    			vehicleOverlay.removeEventListener(MouseEvent.ROLL_OVER, vehicleOverlay.onDeselect);
		    		}
		    		break;
		    	default:
		    		trace("Unknown case in onToolChange");
		    }
		    
		    // Hook up any listeners associated with the new tool
		    switch(newTool) {
		    	case TRACK_TOOL:
		    		map.addEventListener(MapMouseEvent.CLICK, tracks.onMapClick);
		    		map.addEventListener(MapMouseEvent.MOUSE_MOVE, tracks.onMapMouseMove);
		    		Globals.originMarker.visible = true;
		    		TrackOverlay.hitLineCapStyle = CapsStyle.ROUND; // Makes the endpoints more accesible for being snapped to.
		    		break;
		    	case STATION_TOOL:		    	
		    		map.addEventListener(MapMouseEvent.CLICK, stations.onMapClick);
		    		map.addEventListener(MapMouseEvent.MOUSE_MOVE, stations.onMapMouseMove);
		    		Globals.originMarker.visible = false;
		    		TrackOverlay.hitLineCapStyle = CapsStyle.NONE; // prevents overlay overlap
		    		break;
		    	case VEHICLE_TOOL:
		    		map.addEventListener(MapMouseEvent.CLICK, Globals.vehicles.onMapClick);
					map.addEventListener(MapMouseEvent.MOUSE_MOVE, Globals.vehicles.onMapMouseMove);
					Globals.originMarker.visible = false;	    		
		    		TrackOverlay.hitLineCapStyle = CapsStyle.NONE; // prevents short curved segments from being obscured 
		    		break;
		    	case SELECT_TOOL:
		    		for each (stationOverlay in stations.overlays) {
		    			stationOverlay.addEventListener(MouseEvent.ROLL_OVER, stationOverlay.onSelect);
		    			stationOverlay.addEventListener(MouseEvent.ROLL_OVER, stationOverlay.onDeselect);
		    		}		    		
		    		for each (vehicleOverlay in vehicles.overlays) {
		    			vehicleOverlay.addEventListener(MouseEvent.ROLL_OVER, vehicleOverlay.onSelect);
		    			vehicleOverlay.addEventListener(MouseEvent.ROLL_OVER, vehicleOverlay.onDeselect);
		    		}
		    		Globals.originMarker.visible = false;
		    		TrackOverlay.hitLineCapStyle = CapsStyle.NONE; // prevents short curved segments from being obscured
		    		break;	
		    	default:
		    		trace("Unknown case in onToolChange");
		    }
		    		    
		    Globals.tool = newTool; // Set tool to the new value
		}	

		public static function reinitialize():void {
			tracks.reinitialize();
			stations.reinitialize();
			vehicles.reinitialize();	
			gtfXML = null;
			
			map.clearOverlays();
			Globals.map.addOverlay(Globals.originMarker);
			Globals.map.addOverlay(Globals.destMarker);
			Globals.originMarker.overlay = null;
			Globals.destMarker.overlay = null;
			originMarker.setLatLng(Globals.map.getCenter());
			
			haveClicked = false;
			
			IdGenerator.reinitialize();
			Undo.reinitialize();
			dirty = false;
		}

		public static function toPrefsXML():XML {
			var mainWindow:NativeWindow = map.stage.nativeWindow;
			var xml:XML = <General>
			                <MainWindow x={mainWindow.x}
			                            y={mainWindow.y}
			                            width={mainWindow.width}
			                            height={mainWindow.height}
			                            display_state={mainWindow.displayState}
			                             />
						  </General>
			return xml;						  
		}
		
		public static function toDefaultPrefsXML():XML {
			var xml:XML = <General>
							<MainWindow x="100"
									    y="100"
										width="1000"
										height="750"									
										display_state={NativeWindowDisplayState.NORMAL}/>		
						  </General>
			return xml;
		}
		
		public static function fromPrefsXML(xml:XMLList):void {
			var mainWindow:NativeWindow = map.stage.nativeWindow;
			var x:Number = Number(xml.MainWindow.@x);
			var y:Number = Number(xml.MainWindow.@y);
			var width:Number = Number(xml.MainWindow.@width);
			var height:Number = Number(xml.MainWindow.@height);
			mainWindow.bounds = new Rectangle(x, y, width, height);
			
			var display:String = xml.MainWindow.@display_state;
			switch (display) {
				case NativeWindowDisplayState.NORMAL:
					break;
				case NativeWindowDisplayState.MAXIMIZED:
					mainWindow.maximize();
					break;
				case NativeWindowDisplayState.MINIMIZED:
					// do nothing. Starting minimized is rarely the desired behavior, and it seems buggy too.
					break;
				default:
					break;					
			}
			
		}

		public static function validate():Array {
			var errors:Array = new Array();
			errors = errors.concat(Globals.tracks.validate());
			errors = errors.concat(Globals.vehicles.validate());
			return errors;
		}

	}
}