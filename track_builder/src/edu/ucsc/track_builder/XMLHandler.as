package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	
	import flash.events.Event;
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	
	import mx.controls.Alert;
	
	public class XMLHandler
	{
		/* ******** Generating & Saving Functons *************/
		
		public static var schemaLocation:String = "TODO"
		public static var ns:Namespace = new Namespace("TODO");
		public static function generateDataXML():XML {
			var xml:XML = <Network/>
			xml.appendChild(StaticMap.toDataXML());
			xml.appendChild(Globals.tracks.toDataXML());	
			xml.appendChild(Globals.stations.toDataXML());
			xml.appendChild(VehicleModel.toXML()); // TEMP
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
			xml.appendChild(Globals.tracks.toPrefsXML());
			xml.appendChild(TrackOverlay.toPrefsXML());
			xml.appendChild(Globals.stations.toPrefsXML());
			xml.appendChild(StationOverlay.toPrefsXML());		
			xml.appendChild(Globals.vehicles.toPrefsXML());
			xml.appendChild(VehicleOverlay.toPrefsXML());
			return xml;
		}

		public static function generateDefaultPrefsXML():XML {
			var xml:XML = <Preferences/>
			xml.appendChild(Globals.toDefaultPrefsXML());
			xml.appendChild(Globals.tracks.toDefaultPrefsXML());
			xml.appendChild(TrackOverlay.toDefaultPrefsXML());
			xml.appendChild(Globals.stations.toDefaultPrefsXML());
			xml.appendChild(StationOverlay.toDefaultPrefsXML());		
			xml.appendChild(Globals.vehicles.toDefaultPrefsXML());
			xml.appendChild(VehicleOverlay.toDefaultPrefsXML());
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
			Globals.tracks.fromDataXML(xml.TrackSegments.TrackSegment);
			Globals.stations.fromDataXML(xml.Stations.Station);
			Globals.vehicles.fromDataXML(xml.Vehicles.Vehicle);
			Globals.gtfXML = xml.GoogleTransitFeed;
		}
		
		public static function parsePrefsXML(data:String):void
		{
			var xml:XML = new XML(data);
			Globals.fromPrefsXML(xml.General);
			Globals.tracks.fromPrefsXML(xml.Tracks);
			TrackOverlay.fromPrefsXML(xml.TrackOverlay);
			Globals.stations.fromPrefsXML(xml.Stations);
			StationOverlay.fromPrefsXML(xml.StationOverlay);
			Globals.vehicles.fromPrefsXML(xml.Vehicles);	
			VehicleOverlay.fromPrefsXML(xml.VehicleOverlay);
		}
		
		/** Load XML from a file. */
		public static function loadDataXML(event:Event):void {					
			var file:File = event.target as File;
			file.removeEventListener(Event.SELECT, loadDataXML); // remove the event listener that got me here
			if (file != null) {
				var fs:FileStream = new FileStream();
				try {
					fs.open(file, FileMode.READ);		// open
					var data:String = fs.readUTFBytes(fs.bytesAvailable); // read
				} catch (err:Error) {
					Alert.show(err.message);
				} finally {
					fs.close(); // close
				}
				
				// Reset everything to a fresh state
				Globals.reinitialize();
				
				// Parse xml data. Creates the objects and overlays.
				parseDataXML(data); 
				
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
				Globals.map.setZoom(zoom-1); // -1 to account for a station's coverage area not being considered
				
				// Reset the markers
				Globals.originMarker.setSnapOverlay(null);
				Globals.originMarker.setLatLng(center);
				Globals.destMarker.setSnapOverlay(null);
				Globals.destMarker.setLatLng(center);
				Globals.setActiveMarker(Globals.originMarker);				
												
				Globals.dataXMLFile = file; // Point to the new "save" file				
				Globals.dirty = false;      // Mark the file as 'clean'
				Globals.setActiveMarker(Globals.originMarker);
			} else {
				trace(event.target);
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