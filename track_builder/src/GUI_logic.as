import com.google.maps.LatLng;
import com.google.maps.MapMouseEvent;
import com.google.maps.MapType;
import com.google.maps.PaneId;
import com.google.maps.controls.ControlPosition;
import com.google.maps.controls.ScaleControl;
import com.google.maps.controls.ScaleControlOptions;
import com.google.maps.controls.ZoomControl;
import com.google.maps.controls.ZoomControlOptions;
import com.google.maps.interfaces.IPaneManager;

import edu.ucsc.track_builder.Globals;
import edu.ucsc.track_builder.Menu;
import edu.ucsc.track_builder.SnappingMarker;
import edu.ucsc.track_builder.TrackOverlay;
import edu.ucsc.track_builder.XMLHandler;
import edu.ucsc.track_builder.elevation.ElevationService;

import flash.desktop.NativeApplication;
import flash.display.NativeWindow;
import flash.events.Event;
import flash.events.KeyboardEvent;
import flash.events.TimerEvent;
import flash.ui.Keyboard;
import flash.utils.Timer;

import mx.core.Application;
import mx.events.FlexEvent;
import mx.events.ItemClickEvent;

/* TODO: Use  Maps API to grab a image */	
internal var elevationTimer:Timer = new Timer(150, 1); // 150 ms
// a hack for to know where the MouseMoveEvent that triggered the timer was at.
internal var lastMouseMoveEvent:MapMouseEvent; 
internal var app:Application;

/** Create the window / application menu */
internal function initApp(event:FlexEvent):void {
	trace("this.initApp:", this, this.type);
	// Create new panes for the various overlays. createPane inserts into the middle of the
	// pane stack. The final pane stack is, in descending order:
	// PANE_FLOAT, PANE_MARKER, vehiclePane, straightTrackPane, curvedTrackPane, stationPane, PANE_OVERLAYS, PANE_MAP
	Globals.map = map;
	XMLHandler.loadPrefsXML(Globals.prefsXMLFile); // load and use preferences	

	Globals.toolBar = toolBar;
	Globals.mapTypeBar = mapTypeBar;
	Globals.menu = new Menu();
	Globals.dirty = false;
	stage.addEventListener(Event.CLOSING, Globals.menu.onExiting);
	NativeApplication.nativeApplication.addEventListener(Event.EXITING, Globals.menu.onExiting);		
	
	// Handle some OS-dependent muckery for the menu
	if (NativeApplication.supportsMenu) { // MacOS X
		trace("on MacOS X");
		this.nativeApplication.menu = Globals.menu.baseMenu;
	}
	
	if (NativeWindow.supportsMenu) { // Windows
		trace("on Windows");
		this.nativeWindow.menu = Globals.menu.baseMenu;
	}
	
	// Load the cached elevation data
	ElevationService.readCache();	
		
	// Add listeners	
	elevationTimer.addEventListener(TimerEvent.TIMER_COMPLETE, onElevationTimerComplete);
	
	NativeApplication.nativeApplication.addEventListener(KeyboardEvent.KEY_DOWN, onKeyDown);	
}

internal function onMapReady(event:Event):void {
	var defaultLatLng:LatLng = new LatLng(37.0,-122.06); // UC Santa Cruz (Has to go somewhere...)
    map.setCenter(defaultLatLng, 14, MapType.NORMAL_MAP_TYPE);
    map.enableScrollWheelZoom();
    var zoomPosition:ControlPosition = new ControlPosition(ControlPosition.ANCHOR_BOTTOM_RIGHT, 3, 35);
    Globals.zoomControl = new ZoomControl(new ZoomControlOptions({position:zoomPosition}));
    map.addControl(Globals.zoomControl);
    var scalePosition:ControlPosition = new ControlPosition(ControlPosition.ANCHOR_BOTTOM_LEFT, 100, 5);
    map.addControl(new ScaleControl(new ScaleControlOptions({position:scalePosition,
                                                             units:ScaleControlOptions.UNITS_METRIC_ONLY})));
                                                             
	/* Set up panes */
	var pm:IPaneManager = map.getPaneManager();
	Globals.vehiclePane = pm.createPane(2);
	Globals.straightTrackPane = pm.createPane(2);
	Globals.curvedTrackPane = pm.createPane(2);
	Globals.stationPane = pm.createPane(2);
	Globals.markerPane = pm.getPaneById(PaneId.PANE_MARKER);
	
	/* Setup the markers */
	var markerColor:uint = Globals.tracks.bidirectional ? TrackOverlay.bidirLineColor : TrackOverlay.unidirLineColor;
	Globals.originMarker = new SnappingMarker(defaultLatLng, SnappingMarker.makeCircleIcon(markerColor), true);
	Globals.destMarker = new SnappingMarker(defaultLatLng, SnappingMarker.makeCircleIcon(markerColor), false);
	Globals.setActiveMarker(Globals.originMarker);

	/* initialize the default tool */
	Globals.tool = Globals.TRACK_TOOL;
	Globals.onToolChange(Globals.TRACK_TOOL);
			
	/* update status bar when mouse moves in the map area */
	Globals.map.addEventListener(MapMouseEvent.MOUSE_MOVE, onMapMouseMove); 
}

internal function onMapMouseMove(event:MapMouseEvent):void {
	// update status bar
	this.status = "Distance: " + Math.round(Globals.originMarker.getLatLng().distanceFrom(event.latLng)) + " meters" +
	              "\t\tLat: " + event.latLng.lat().toFixed(5) + ", Lng: " + event.latLng.lng().toFixed(5) +
	              "\t\tAltitude: ???? meters";
	              
	/* So that the app doesn't spam the elevation webservice providers with requests
 	 * we wait for the mouse to stop moving for a moment before sending.
 	 */
 	elevationTimer.reset();
 	elevationTimer.start();
 	lastMouseMoveEvent = event;
}


internal function onElevationTimerComplete(event:TimerEvent):void {
//	var latlng:LatLng = lastMouseMoveEvent.latLng;
//	ElevationService.requestElevations(Vector.<LatLng>([latlng]));
}

///** Updates the elevation in the status bar once the data comes in. */ 
//internal function onElevationComplete(event:ElevationEvent):void {
//	var status:String = this.status;
//	var elevation:Number = event.elevation;	
//	status = status.replace("???", elevation.toFixed(2)); 
//	this.status = status;
//}
//
//internal function onElevationError(event:Event):void {
//	trace("Elevation Error:", event);	
//}

//internal function onToolChooser(event:ItemClickEvent):void {
//    Globals.tool = event.index;
//    this.detailStack.selectedIndex = event.index; // fires an event that is listened for by Globals.onToolChange
//    trace('Tool changed to: ' + event.label);
//}


/** Switch between google map styles */
internal function onMapTypeChooser(event:ItemClickEvent):void {
    if (event.index == 0) {
    	trace("Map changed to NORMAL");
        Globals.map.setMapType(MapType.NORMAL_MAP_TYPE);
    } else if (event.index == 1) {
        trace("Map changed to SATELLITE");
        Globals.map.setMapType(MapType.SATELLITE_MAP_TYPE);
    } else if (event.index == 2) {
    	trace("Map changed to HYBRID");
        Globals.map.setMapType(MapType.HYBRID_MAP_TYPE);
    } else { // Terrain
    	trace("Map changed to PHYSICAL");
        Globals.map.setMapType(MapType.PHYSICAL_MAP_TYPE);
    }
}


internal function onKeyDown(event:KeyboardEvent):void
{		
	var tool:int = -1;
	switch (event.keyCode) {
		case Keyboard.ESCAPE:
			Globals.setActiveMarker(Globals.originMarker);
			Globals.originMarker.visible = true;
			break;
		case Keyboard.F1:
			tool = Globals.SELECT_TOOL;
			break;
		case Keyboard.F2:
			tool = Globals.TRACK_TOOL;
			break;
		case Keyboard.F3:
			tool = Globals.STATION_TOOL;
			break;
		case Keyboard.F4:
			tool = Globals.VEHICLE_TOOL;
			break;
		case Keyboard.NUMPAD_ADD:
		case Keyboard.EQUAL:
			Globals.zoomIn(null);
			break;
		case Keyboard.MINUS:
		case Keyboard.NUMPAD_SUBTRACT:
			if (!event.shiftKey) {
				Globals.zoomOut(null);
			}
			break;		
		default:
			break;
	}
	
	if (tool != -1) {	   
		toolBar.selectedIndex = tool; // manually set the index for the UI toolbar
		Globals.onToolChange(tool);
	}
}
