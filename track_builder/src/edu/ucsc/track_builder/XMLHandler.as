package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	
	import mx.controls.Alert;
	import mx.events.CloseEvent;
	import mx.utils.ObjectUtil;
	
	public class XMLHandler
	{
		/* ******** Generating & Saving Functons *************/
		
		public static var schemaLocation:String = "TODO"
		public static var ns:Namespace = new Namespace("TODO");
		public static function generateDataXML():XML {
			var xml:XML = <Network/>
			xml.appendChild(StaticMap.toDataXML());
			xml.appendChild(Globals.tracks.toDataXML());	
			xml.appendChild(Globals.stations.toDataXML(Globals.vehicles.vehicles));
			xml.appendChild(Globals.vehicleModels.toDataXML(Globals.vehicles.vehicles));
			xml.appendChild(Globals.vehicles.toDataXML());
			var sortedSegs:Vector.<TrackSegment> = Globals.tracks.getSortedTrackSegments();
			xml.appendChild(makeWaypointsXml(sortedSegs)); // ignored when loading a saved file
			xml.appendChild(makeSwitchesXml(sortedSegs)); // ignored when loading a saved file
			xml.appendChild(makeMergesXml(sortedSegs));   // ignored when loading a saved file
			
			if (Globals.gtfXML != null) {
				xml.appendChild(Globals.gtfXML);
			}			
			return xml;
		}

		public static function generatePrefsXML():XML {
			var xml:XML = <Preferences/>
			xml.appendChild(Globals.toPrefsXML());
			xml.appendChild(Globals.menu.toPrefsXML());
			xml.appendChild(Globals.tracks.toPrefsXML());
			xml.appendChild(TrackOverlay.toPrefsXML());
			xml.appendChild(Globals.stations.toPrefsXML());
			xml.appendChild(StationOverlay.toPrefsXML());		
			xml.appendChild(Globals.vehicles.toPrefsXML());
			xml.appendChild(VehicleOverlay.toPrefsXML());
			xml.appendChild(Globals.vehicleModels.toPrefsXML());			
			return xml;
		}

		public static function generateDefaultPrefsXML():XML {
			var xml:XML = <Preferences/>
			xml.appendChild(Globals.toDefaultPrefsXML());
			xml.appendChild(Globals.menu.toDefaultPrefsXML());
			xml.appendChild(Globals.tracks.toDefaultPrefsXML());
			xml.appendChild(TrackOverlay.toDefaultPrefsXML());
			xml.appendChild(Globals.stations.toDefaultPrefsXML());
			xml.appendChild(StationOverlay.toDefaultPrefsXML());		
			xml.appendChild(Globals.vehicles.toDefaultPrefsXML());
			xml.appendChild(VehicleOverlay.toDefaultPrefsXML());
			xml.appendChild(Globals.vehicleModels.toDefaultPrefsXML());
			return xml;
		}


		/** Waypoints aren't needed from a layout standpoint, but they are part of the sim/controller design.
		 * Rather than model them while building the network, they are only described in the XML.
		 */ 
		public static function makeWaypointsXml(sortedSegs:Vector.<TrackSegment>):XML {
			/* Make waypoints for the beginning of segments only. */
			IdGenerator.resetWaypointId(); // Restart the count each time we save.
			var waypointsXml:XML = new XML(<Waypoints/>);
			for each (var ts:TrackSegment in sortedSegs) {
				if (ts.prev_ids.length == 1) { // it's not a merge, and it's not an endpoint
					var prevSeg:TrackSegment = Globals.tracks.getTrackSegment(ts.prev_ids[0]) // the only one
					if (prevSeg.next_ids.length != 1) { // check that this isn't just one leg of a switch
						continue; // it's a switch, not a waypoint.
					}
					// it's a waypoint
					var maxSpeed:Number = Math.min(ts.maxSpeed, prevSeg.maxSpeed);
					var waypointXml:XML = new XML(<Waypoint id={IdGenerator.getWaypointId()} max_speed={maxSpeed}>
					                                  <LatLng lat={ts.getStart().lat().toFixed(7)}
					                                          lng={ts.getStart().lng().toFixed(7)}
					                                          ground_level={ts.startGround}
					                                          offset={ts.getStartOffset()}					                                          
					                                          elevation={ts.startElev} />
					                                  <Incoming>{ts.prev_ids[0]}</Incoming>
					                                  <Outgoing>{ts.id}</Outgoing>
					                            </Waypoint>);
					waypointsXml.appendChild(waypointXml);					                                  
				} 
			}
			return waypointsXml;
		}

		/** Switches aren't needed from a layout standpoint, but they are part of the sim/controller design.
		 * Rather than model them while building the network, they are only described in the XML
		 */ 
		public static function makeSwitchesXml(sortedSegs:Vector.<TrackSegment>):XML {
			IdGenerator.resetSwitchId(); // Restart the count each time we save.
			var switchesXml:XML = new XML(<Switches/>);		
			for each (var ts:TrackSegment in sortedSegs) {
				if (ts.next_ids.length >= 2) {
					// It's a switch. Calc the max speed as the speed of the source edge
					var switchXml:XML = new XML(<Switch id={IdGenerator.getSwitchId()} max_speed={ts.maxSpeed} >
					                                  <LatLng lat={ts.getEnd().lat().toFixed(7)}
					                                          lng={ts.getEnd().lng().toFixed(7)}
					                                          elevation={ts.endElev} />
					                                  <Incoming>{ts.id}</Incoming>
					                            </Switch>);
					for each (var next_id:String in ts.next_ids) {
						switchXml.appendChild(new XML(<Outgoing>{next_id}</Outgoing>));						
					}
					switchesXml.appendChild(switchXml);					                                  
				} 
			}
			return switchesXml;
		}

		/** Merges aren't needed from a layout standpoint, but they are part of the sim/controller design.
		 * Rather than model them while building the network, they are only described in the XML
		 */ 		
		public static function makeMergesXml(sortedSegs:Vector.<TrackSegment>):XML {
			IdGenerator.resetMergeId(); // Restart the count each time we save.
			var mergesXml:XML = new XML(<Merges/>);		
			for each (var ts:TrackSegment in sortedSegs) {
				if (ts.prev_ids.length >= 2) {
					// it's a merge. Calc max speed as the speed of the outgoing edge
					var mergeXml:XML = new XML(<Merge id={IdGenerator.getMergeId()} max_speed={ts.maxSpeed} >
					                                  <LatLng lat={ts.getStart().lat().toFixed(7)}
					                                          lng={ts.getStart().lng().toFixed(7)}
					                                          elevation={ts.startElev} />					                                  
					                            </Merge>);					
					for each (var prev_id:String in ts.prev_ids) {
						mergeXml.appendChild(new XML(<Incoming>{prev_id}</Incoming>));						
					}
					mergeXml.appendChild(new XML(<Outgoing>{ts.id}</Outgoing>));
					mergesXml.appendChild(mergeXml);					                                  
				} 
			}
			return mergesXml; 
		}
		
		/** Saves XML output to file. */
		public static function doSaveXml(file:File, xml:XML):void {
			var fs:FileStream = new FileStream();
			try {
				fs.open(file, FileMode.WRITE); // open synchronously
				fs.writeUTFBytes('<?xml version="1.0" encoding="utf-8"?>' + '\n');
				fs.writeUTFBytes(xml.toXMLString());
			} catch(e:Error) {
				Alert.show(e.message, "Save Failed", Alert.CANCEL);
				trace("Failed:", e.message);
			} finally {
				fs.close();
			}
		}
		
		/* ************************ Parsing & Loading functions ************************ */
	
		/* Converts the xml file to an XML object. */
		public static function parseDataXML(data:String):void
		{
			var xml:XML = new XML(data);
			Globals.vehicleModels.fromDataXML(xml.VehicleModels.VehicleModel) 
			Globals.tracks.fromDataXML(xml.TrackSegments.TrackSegment);
			Globals.stations.fromDataXML(xml.Stations.Station);
			Globals.vehicles.fromDataXML(xml.Vehicles.Vehicle);
			Globals.gtfXML = xml.GoogleTransitFeed;
		}
		
		public static function parsePrefsXML(data:String):void
		{
			var xml:XML = new XML(data);
			try {
				Globals.fromPrefsXML(XML(xml.General));
			} catch (err:TypeError) {
				trace("Unable to load General Prefs. Using defaults.")
				Globals.fromPrefsXML(Globals.toDefaultPrefsXML());
			}
			
			try {
				Globals.menu.fromPrefsXML(XML(xml.Menu));
				Globals.menu.refreshOpenRecentMenu();
			} catch (err:TypeError) {
				trace("Unable to load Menu Prefs. Using defaults.")
				Globals.menu.fromPrefsXML(Globals.menu.toDefaultPrefsXML());
			}
			
			try {
				Globals.tracks.fromPrefsXML(XML(xml.Tracks));
			} catch (err:TypeError) {
				trace("Unable to load Tracks Prefs. Using defaults.")
				Globals.tracks.fromPrefsXML(Globals.tracks.toDefaultPrefsXML());
			}
			
			try {
				TrackOverlay.fromPrefsXML(XML(xml.TrackOverlay));
			} catch (err:TypeError) {
				trace("Unable to load TrackOverlay Prefs. Using defaults.")
				TrackOverlay.fromPrefsXML(TrackOverlay.toDefaultPrefsXML());
			}
			
			try {
				Globals.stations.fromPrefsXML(XML(xml.Stations));
			} catch (err:TypeError) {
				trace("Unable to load Stations Prefs. Using defaults.")
				Globals.stations.fromPrefsXML(Globals.stations.toDefaultPrefsXML());
			}
			
			try {
				StationOverlay.fromPrefsXML(XML(xml.StationOverlay));
			} catch (err:TypeError) {
				trace("Unable to load StationOverlay Prefs. Using defaults.")				
				StationOverlay.fromPrefsXML(StationOverlay.toDefaultPrefsXML());
			}
			
			try {
				Globals.vehicleModels.fromPrefsXML(XML(xml.VehicleModels)); // must be loaded before vehicles
			} catch (err:TypeError) {
				trace("Unable to load VehicleModels Prefs. Using defaults.")
				Globals.vehicleModels.fromPrefsXML(Globals.vehicleModels.toDefaultPrefsXML());
			}
			
			try {
				Globals.vehicles.fromPrefsXML(XML(xml.Vehicles));
			} catch (err:TypeError) {
				trace("Unable to load Vehicles Prefs. Using defaults.")
				Globals.vehicles.fromPrefsXML(Globals.vehicles.toDefaultPrefsXML());
			}
			
			try {
				VehicleOverlay.fromPrefsXML(XML(xml.VehicleOverlay));
			} catch (err:TypeError) {
				trace("Unable to load VehicleOverlay Prefs. Using defaults.")
				VehicleOverlay.fromPrefsXML(VehicleOverlay.toDefaultPrefsXML());
			}
		}
		
		/** Load XML from a file. */
		public static function loadDataXML(file:File):void {			
			if (file != null) {
				var fs:FileStream = new FileStream();
				try {
					fs.open(file, FileMode.READ);
					var data:String = "";
					while (fs.bytesAvailable) {
						data += fs.readUTFBytes(fs.bytesAvailable);
					}
				} catch (err:Error) {
					trace(err.message);
					Alert.show(err.message, "Load Failed", Alert.CANCEL);
				} finally {
					fs.close();
				}
				
				// Reset everything to a fresh state
				Globals.reinitialize();
				
				// Parse xml data. Creates the objects and overlays.
				try {
					parseDataXML(data);
				} catch (err:ModelError) {					
					var knownModel:VehicleModel = Globals.vehicleModels.getModelByName(err.model.modelName);
					
					var alertMsg:String = 'File contains a version of the VehicleModel "'+knownModel.modelName+'" which ' + 
							'is different than the one stored in the program. Loading the file will overwrite the ' + 
							'version stored in the program.\n\n'

					alertMsg += knownModel.modelName + '\n'
					alertMsg += 'PROPERTY, FROM PROGRAM, FROM FILE\n'					
					for each (var prop:String in ObjectUtil.getClassInfo(knownModel).properties) {
						trace(prop, knownModel[prop], err.model[prop]);
						if (knownModel[prop] != err.model[prop]) {
							alertMsg += prop+', '+knownModel[prop]+', '+err.model[prop]+'\n';
						}
					}							

					Alert.yesLabel = 'Continue';
					Alert.buttonWidth = 100;
					Alert.show(alertMsg,
							   'Name Conflict',
						       Alert.YES|Alert.CANCEL,
						       null,
					           function clickHandler(clsEvent:CloseEvent):void {
					           		if (clsEvent.detail == Alert.YES) { // clicked 'Overwrite'
       				           			Globals.vehicleModels.removeModelByName(knownModel.modelName);
       				           			XMLHandler.loadDataXML(file); // the original event that triggered loadDataXML   
					           		}
					           		Alert.yesLabel = 'Yes'; // reset the buttons to default values
					           		Alert.buttonWidth = 60;
					           });					           
					return; // Abort and wait for user interaction
				} catch (err:Error) {
					Alert.show(err.message, "XML Parsing Error", Alert.CANCEL);
					Globals.reinitialize();
					return; // Abort and wait for user interaction
				}
				
				// Attach appropriate listeners to the freshly created overlays
				Globals.onToolChange(Globals.tool);
				
				// Update the id generator, so that new ids don't duplicate existing ones
				var sortedSegs:Vector.<TrackSegment> = Globals.tracks.getSortedTrackSegments();
				if (sortedSegs.length > 0) {
					IdGenerator.updateTrackSegId(sortedSegs[sortedSegs.length-1].id);
				}
				if (Globals.stations.stations.length > 0) {
					IdGenerator.updateStationId(Globals.stations.stations[Globals.stations.stations.length-1].id);
				}
				if (Globals.vehicles.vehicles.length > 0) {
					IdGenerator.updateVehicleId(Globals.vehicles.vehicles[Globals.vehicles.vehicles.length-1].id);
				}
				
				// Change the location and zoom to show the entire track.
				var bounds:LatLngBounds = Globals.tracks.getLatLngBounds();
				var center:LatLng = bounds.getCenter();
				Globals.map.setCenter(center);
				var zoom:Number = Globals.map.getBoundsZoomLevel(bounds);
				Globals.map.setZoom(zoom);
				
				// Reset the markers
				Globals.originMarker.setSnapOverlay(null);
				Globals.originMarker.setLatLng(center);
				Globals.destMarker.setSnapOverlay(null);
				Globals.destMarker.setLatLng(center);
				Globals.setActiveMarker(Globals.originMarker);				
												
				Globals.dataXMLFile = file; // Point to the new "save" file				
				Globals.dirty = false;      // Mark the file as 'clean'
				Globals.setActiveMarker(Globals.originMarker);
			}		
		} 
		
		/** Loads and parses the prefs XML. If prefs XML file doesn't exist, a default one is created and used.
		 */
		public static function loadPrefsXML(file:File):void {
			if (file.exists) { // prefs file exists
				var fs:FileStream = new FileStream();
				try {
					fs.open(file, FileMode.READ); // open
					var data:String = fs.readUTFBytes(fs.bytesAvailable); // read
				} catch (err:Error) {
					Alert.show(err.message);
				} finally {
					fs.close();
				}				
				parsePrefsXML(data);
			} else { // no prefs file has been created, or it was deleted
				var prefs:XML = XMLHandler.generateDefaultPrefsXML(); // create default prefs
				doSaveXml(Globals.prefsXMLFile, prefs);                 // save them
				parsePrefsXML(prefs.toXMLString());		    // use them
			}
		}			
	}
}