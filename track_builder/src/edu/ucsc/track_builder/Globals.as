package edu.ucsc.track_builder
{
	import com.google.maps.Map;
	import com.google.maps.MapMouseEvent;
	import com.google.maps.MapType;
	import com.google.maps.controls.ZoomControl;
	import com.google.maps.interfaces.IPane;
	
	import flash.display.CapsStyle;
	import flash.display.NativeWindow;
	import flash.display.NativeWindowDisplayState;
	import flash.events.Event;
	import flash.events.EventDispatcher;
	import flash.events.MouseEvent;
	import flash.filesystem.File;
	import flash.geom.Rectangle;
	
	import mx.controls.ToggleButtonBar;
	
	public class Globals extends EventDispatcher
	{
		/* Tool TabBar indexes */
		public static const SELECT_TOOL:int = 0;
		public static const TRACK_TOOL:int = 1;
		public static const STATION_TOOL:int = 2;
		public static const VEHICLE_TOOL:int = 3;
		
		/* MapType TabBar indexes */
		public static const MAP_MAPTYPE:int = 0;
		public static const SATELLITE_MAPTYPE:int = 1;
		public static const HYBRID_MAPTYPE:int = 2;
		public static const TERRAIN_MAPTYPE:int = 3;

		public static var tool:int; // Which tool is being used. Uses above constants.	
		
		// Holds instances of the top level containers
		[Bindable] public static var tracks:Tracks;
		[Bindable] public static var stations:Stations;
		[Bindable] public static var vehicles:Vehicles;		
		[Bindable] public static var vehicleModels:VehicleModels;
		[Bindable] public static var weather:Weather;	
		public static var menu:Menu; // my menu, NOT mx.controls.menu
		
		// a couple new pane ids, in addition to the existing ones:
		// http://code.google.com/apis/maps/documentation/flash/reference.html#PaneId
		public static var stationPane:IPane;
		public static var curvedTrackPane:IPane;
		public static var straightTrackPane:IPane;		
		public static var vehiclePane:IPane;
		public static var markerPane:IPane;
		
		public static var mainWindow:NativeWindow;
		public static var map:Map; // The google map				
		public static var toolBar:ToggleButtonBar;
		public static var mapTypeBar:ToggleButtonBar;
				
		public static var dataXMLFile:File; // The XML file currently being worked with.
		public static var prefsXMLFile:File = File.applicationStorageDirectory.resolvePath("preferences.xml"); // The prefs file
		public static var gtfXML:XMLList = null
		
		public static var dirty:Boolean; // has been changed since last save. 
						
		public static var originMarker:SnappingMarker;		
		public static var destMarker:SnappingMarker;
		protected static var activeMarker:SnappingMarker;

		// Used to 'remind' SaveAs what it should do after asynchronously saving.
		public static var postSave:Function = null;

		public static var zoomControl:ZoomControl;

		public static function initialize(mainWindow_:NativeWindow):void {
			mainWindow = mainWindow_
			tracks = new Tracks(Tracks.RIGHT);
		    stations = new Stations();
			vehicles = new Vehicles();		
			vehicleModels = new VehicleModels();	
			weather = new Weather();	
			menu = new Menu();	
		}

		public static function reinitialize():void {
			tracks.reinitialize();
			stations.reinitialize();
			vehicles.reinitialize();
			weather = new Weather();
			dataXMLFile = null;	
			gtfXML = null;
			
			map.clearOverlays();
			Globals.map.addOverlay(Globals.originMarker);
			Globals.map.addOverlay(Globals.destMarker);
			Globals.originMarker.setSnapOverlay(null);
			Globals.destMarker.setSnapOverlay(null);
			originMarker.setLatLng(Globals.map.getCenter());		
			setActiveMarker(Globals.originMarker);
			
			IdGenerator.reinitialize();
			Undo.reinitialize();
			dirty = false;
		}

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

		public static function getActiveMarker():SnappingMarker {
			return activeMarker;
		}

		public static function setActiveMarker(marker:SnappingMarker):void {
			Undo.pushMicro(Globals, Globals.setActiveMarker, Globals.activeMarker);
			if (activeMarker != null) {
				Globals.map.removeEventListener(MapMouseEvent.MOUSE_MOVE, activeMarker.onMapMouseMove);
				Globals.map.removeEventListener(MouseEvent.MOUSE_MOVE, activeMarker.onMouseMove);
				Globals.map.removeEventListener(MapMouseEvent.ROLL_OUT, activeMarker.onMapRollOut);
			}
			activeMarker = marker;
			if (activeMarker != null) {
				Globals.map.addEventListener(MapMouseEvent.MOUSE_MOVE, activeMarker.onMapMouseMove);
				Globals.map.addEventListener(MouseEvent.MOUSE_MOVE, activeMarker.onMouseMove);
				Globals.map.addEventListener(MapMouseEvent.ROLL_OUT, activeMarker.onMapRollOut);
			}
			
			// If switching to the originMarker, get rid of the existing 'live preview', if one exists
			if (marker == Globals.originMarker) {
				Undo.undo(Undo.PREVIEW); 
			}
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
		
		public static function fromPrefsXML(xml:XML):void {
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

		/** Zooms the map in one level. If already at the max zoom level, will switch map styles to zoom further if possible. */
		public static function zoomIn(evt:Event):void {
			var curr_max_zoom:Number = Globals.map.getMaxZoomLevel(Globals.map.getCurrentMapType());					
			if (Globals.map.getZoom() == curr_max_zoom) {
				if (Globals.map.getMaxZoomLevel(MapType.SATELLITE_MAP_TYPE) > curr_max_zoom) {
					Globals.map.setMapType(MapType.SATELLITE_MAP_TYPE); // TODO: Need to update the toolbar, or change the MapType via the toolbar.
					Globals.mapTypeBar.selectedIndex = Globals.SATELLITE_MAPTYPE
					Globals.map.zoomIn();
				}
			} else {
				Globals.map.zoomIn();
			}
		}

		/** Zooms the map out one level. */
		public static function zoomOut(evt:Event):void {
			Globals.map.zoomOut();	
		}

	}
}