package edu.ucsc.track_builder
{
	import com.google.maps.LatLng;
	import com.google.maps.MapType;
	
	import edu.ucsc.track_builder.gtf_import.GtfImportError;
	import edu.ucsc.track_builder.gtf_import.GtfImportUI;
	
	import flash.desktop.NativeApplication;
	import flash.display.NativeMenu;
	import flash.display.NativeMenuItem;
	import flash.display.NativeWindowInitOptions;
	import flash.events.Event;
	import flash.filesystem.File;
	import flash.geom.Rectangle;
	import flash.html.HTMLLoader;
	import flash.net.URLRequest;
	
	import mx.containers.TitleWindow;
	import mx.controls.Alert;
	import mx.controls.TextArea;
	import mx.core.Application;
	import mx.events.CloseEvent;
	import mx.managers.PopUpManager;

	
	public class Menu {
		public var baseMenu:NativeMenu;	
		
		public var prefEditor:PreferencesEditor;
		public var attrEditor:AttributesEditor;
		public var autoLevelUI:AutoLevelUI;
		public var gtfImportUI:GtfImportUI;
		public var htmlHelp:HTMLLoader;	
		
		public var hideTracks:NativeMenuItem;
		public var hideStations:NativeMenuItem;
		public var hideVehicles:NativeMenuItem;
		public var viewNormal:NativeMenuItem;
		public var viewGrade:NativeMenuItem;
//		public var viewElevation:NativeMenuItem;
		
		/** Add submenus and menu items to then basemenu. For now, no references to
		 * submenus or items are kept.
		 */
		public function Menu() {
			baseMenu = new NativeMenu();
			
			/* File Menu */
			var fileMenu:NativeMenu	= new NativeMenu();
			var fileNew:NativeMenuItem = new NativeMenuItem("New");
			var fileOpen:NativeMenuItem = new NativeMenuItem("Open...");
			var fileSave:NativeMenuItem = new NativeMenuItem("Save");
			var fileSaveAs:NativeMenuItem = new NativeMenuItem("Save As...");
			var fileImportMenu:NativeMenu = new NativeMenu();
			var fileImportGTF:NativeMenuItem = new NativeMenuItem("Google Transit Feed...");
			var fileQuit:NativeMenuItem = new NativeMenuItem("Quit"); 
			
			fileNew.keyEquivalent = "n";
			fileNew.mnemonicIndex = 0;
			fileNew.addEventListener(Event.SELECT, onNew);
			
			fileOpen.keyEquivalent = "o";
			fileOpen.mnemonicIndex = 0;
			fileOpen.addEventListener(Event.SELECT, onOpen);
			
			fileSave.keyEquivalent = "s";
			fileSave.mnemonicIndex = 0;
			fileSave.addEventListener(Event.SELECT, onSave);
			
			fileSaveAs.keyEquivalent = "a";
			fileSaveAs.mnemonicIndex = 5;
			fileSaveAs.addEventListener(Event.SELECT, onSaveAs);
			
			fileImportGTF.mnemonicIndex = 0;
			fileImportGTF.addEventListener(Event.SELECT, onImportGTF);
			
			fileQuit.keyEquivalent = "q";
			fileQuit.mnemonicIndex = 0;
			fileQuit.addEventListener(Event.SELECT, onQuit);
			
			fileMenu.addItem(fileNew);
			fileMenu.addItem(fileOpen);
			fileMenu.addItem(fileSave);
			fileMenu.addItem(fileSaveAs);
			fileMenu.addItem(new NativeMenuItem("", true));
			fileImportMenu.addItem(fileImportGTF);
			fileMenu.addSubmenu(fileImportMenu, "Import");
			fileMenu.addItem(new NativeMenuItem("", true));
			fileMenu.addItem(fileQuit);			
			baseMenu.addSubmenu(fileMenu, "File");
			
			/* Edit Menu */
			var editMenu:NativeMenu = new NativeMenu();
			var undo:NativeMenuItem = new NativeMenuItem("Undo");
			undo.keyEquivalent = "z";
			undo.mnemonicIndex = 0;
			undo.addEventListener(Event.SELECT, onUndo);

			var editAttributes:NativeMenuItem = new NativeMenuItem("Attributes");
			editAttributes.keyEquivalent = "t"
			editAttributes.mnemonicIndex = 0;
			editAttributes.addEventListener(Event.SELECT, onAttributes);
			
			var editPreferences:NativeMenuItem = new NativeMenuItem("Preferences");
			editPreferences.keyEquivalent = "p";
			editPreferences.mnemonicIndex = 0;
			editPreferences.addEventListener(Event.SELECT, onPreferences);
			
			editMenu.addItem(undo);
			editMenu.addItem(new NativeMenuItem("", true));
			editMenu.addItem(editAttributes);
			editMenu.addItem(editPreferences);			
			baseMenu.addSubmenu(editMenu, "Edit");
			
			/* Select Menu */
			var selectMenu:NativeMenu = new NativeMenu();
			var selectTrackSegment:NativeMenuItem = new NativeMenuItem("Track Segment...");
			selectTrackSegment.addEventListener(Event.SELECT, onSelectTrackSegment);
			
			var selectStation:NativeMenuItem = new NativeMenuItem("Station...");
			selectStation.addEventListener(Event.SELECT, onSelectStation);
			
			var selectVehicle:NativeMenuItem = new NativeMenuItem("Vehicle...");
			selectVehicle.addEventListener(Event.SELECT, onSelectVehicle);
						
			selectMenu.addItem(selectTrackSegment);
			selectMenu.addItem(selectStation);
			selectMenu.addItem(selectVehicle);			
			baseMenu.addSubmenu(selectMenu, "Select");
			
			/* View Menu */
			var viewMenu:NativeMenu = new NativeMenu();
			
			hideTracks = new NativeMenuItem("Hide Track");
			hideTracks.checked = false;
			hideTracks.mnemonicIndex = 5;			
			hideTracks.addEventListener(Event.SELECT, function():void {
															hideTracks.checked = !hideTracks.checked; // toggle
															Globals.curvedTrackPane.visible = !hideTracks.checked;
															Globals.straightTrackPane.visible = !hideTracks.checked;
													  });
			
			hideStations = new NativeMenuItem("Hide Stations");
			hideStations.checked = false;
			hideStations.mnemonicIndex = 5;
			hideStations.addEventListener(Event.SELECT, function():void {
															hideStations.checked = !hideStations.checked;
															Globals.stationPane.visible = !hideStations.checked;
														});
			
			hideVehicles = new NativeMenuItem("Hide Vehicles");
			hideVehicles.checked = false;
			hideVehicles.mnemonicIndex = 5;
			hideVehicles.addEventListener(Event.SELECT, function():void {
															hideVehicles.checked = !hideVehicles.checked;
															Globals.vehiclePane.visible = !hideVehicles.checked;
														});			
			
			viewNormal = new NativeMenuItem("Normal");
			viewNormal.checked = true;
			viewNormal.mnemonicIndex = 0;
			viewNormal.addEventListener(Event.SELECT, onViewNormal);
			
			viewGrade = new NativeMenuItem("Grade");
			viewGrade.checked = false;
			viewGrade.mnemonicIndex = 0;
			viewGrade.addEventListener(Event.SELECT, onViewGrade);
			
//			viewElevation = new NativeMenuItem("Elevation");
//			viewElevation.checked = false;
//			viewElevation.mnemonicIndex = 0;
//			viewElevation.addEventListener(Event.SELECT, onViewElevation);
			
			var viewStatistics:NativeMenuItem = new NativeMenuItem("Statistics");
			viewStatistics.mnemonicIndex = 1;
			viewStatistics.addEventListener(Event.SELECT, onStatistics);				
			
			viewMenu.addItem(hideTracks);
			viewMenu.addItem(hideStations);
			viewMenu.addItem(hideVehicles);
			viewMenu.addItem(new NativeMenuItem("", true));
			viewMenu.addItem(viewNormal);
			viewMenu.addItem(viewGrade);
//			viewMenu.addItem(viewElevation);
			viewMenu.addItem(new NativeMenuItem("", true));
			viewMenu.addItem(viewStatistics);
			baseMenu.addSubmenu(viewMenu, "View");
			
			/* Tools Menu */			
			var toolsMenu:NativeMenu = new NativeMenu();
			
			/* I would like to show the function key shortcuts for these menu items, but AIR is buggy.
			 * See: http://bugs.adobe.com/jira/browse/SDK-17901   (Unfixed in AIR v2.0)
			 * This was the cause of issue 
			 */
			var selectToolMenu:NativeMenuItem = new NativeMenuItem("Select");
			selectToolMenu.mnemonicIndex = 0;			
			selectToolMenu.addEventListener(Event.SELECT, function():void {Globals.onToolChange(Globals.SELECT_TOOL); Globals.toolBar.selectedIndex = Globals.SELECT_TOOL});
			
			var trackToolMenu:NativeMenuItem = new NativeMenuItem("Tracks");
			trackToolMenu.mnemonicIndex = 0;
			trackToolMenu.addEventListener(Event.SELECT, function():void {Globals.onToolChange(Globals.TRACK_TOOL); Globals.toolBar.selectedIndex = Globals.TRACK_TOOL});
			
			var stationToolMenu:NativeMenuItem = new NativeMenuItem("Stations");
			stationToolMenu.mnemonicIndex = 0;
			stationToolMenu.addEventListener(Event.SELECT, function():void {Globals.onToolChange(Globals.STATION_TOOL); Globals.toolBar.selectedIndex = Globals.STATION_TOOL});
 
			var vehicleToolMenu:NativeMenuItem = new NativeMenuItem("Vehicles");
			vehicleToolMenu.mnemonicIndex = 0;
			vehicleToolMenu.addEventListener(Event.SELECT, function():void {Globals.onToolChange(Globals.VEHICLE_TOOL); Globals.toolBar.selectedIndex = Globals.VEHICLE_TOOL});			
			
			var gotoAddress:NativeMenuItem = new NativeMenuItem("Goto Address...");
			gotoAddress.mnemonicIndex = 0;
			gotoAddress.addEventListener(Event.SELECT, onAddress);

			var validate:NativeMenuItem = new NativeMenuItem("Validate");
			validate.mnemonicIndex = 0;
			validate.addEventListener(Event.SELECT, function():void {
																 var msg:String = Globals.validate().join("\n");
																 if (!msg) {
																 	msg = "No errors."
																 } 
				                                                 Alert.show(msg);
														   });
			
			toolsMenu.addItem(selectToolMenu);
			toolsMenu.addItem(trackToolMenu);
			toolsMenu.addItem(stationToolMenu);
			toolsMenu.addItem(vehicleToolMenu);
			toolsMenu.addItem(new NativeMenuItem("", true)); // separator
			toolsMenu.addItem(gotoAddress);
			toolsMenu.addItem(validate);
			baseMenu.addSubmenu(toolsMenu, "Tools");
			
			/* Debug Menu */
			var debugMenu:NativeMenu = new NativeMenu();
//			var debugShowKml:NativeMenuItem = new NativeMenuItem("Show KML");
//			debugShowKml.addEventListener(Event.SELECT, onShowKml);
			
			var debugShowXml:NativeMenuItem = new NativeMenuItem("Show XML");		
			debugShowXml.mnemonicIndex = 0;
			debugShowXml.addEventListener(Event.SELECT, onShowXml);		
			
			var debugFindDuplicateOverlays:NativeMenuItem = new NativeMenuItem("Find Duplicate Overlays");
			debugFindDuplicateOverlays.addEventListener(Event.SELECT, onFindDuplicateOverlays);
				
			var debugZoomIn:NativeMenuItem = new NativeMenuItem("Zoom In");
			debugZoomIn.addEventListener(Event.SELECT, onZoomIn);

			var autoLevel:NativeMenuItem = new NativeMenuItem("AutoLevel");
			autoLevel.mnemonicIndex = 4;
			autoLevel.addEventListener(Event.SELECT, onAutoLevel); 
				
//			debugMenu.addItem(debugShowKml);
			debugMenu.addItem(debugShowXml);
			debugMenu.addItem(debugFindDuplicateOverlays);
			debugMenu.addItem(debugZoomIn);
			debugMenu.addItem(autoLevel);
			baseMenu.addSubmenu(debugMenu, "Debug");
			
			/* Help Menu */
			var helpMenu:NativeMenu = new NativeMenu();
			var helpDoc:NativeMenuItem = new NativeMenuItem("Documentation");
			var helpAbout:NativeMenuItem = new NativeMenuItem("About");
			
			helpDoc.mnemonicIndex = 0;
			helpDoc.addEventListener(Event.SELECT, onDoc);
			
			helpAbout.mnemonicIndex = 0;
			helpAbout.addEventListener(Event.SELECT, onAbout);
			
			helpMenu.addItem(helpDoc);
			helpMenu.addItem(helpAbout);
			baseMenu.addSubmenu(helpMenu, "Help");
		}
		
		// Handler for "New" file menuitem.
		public function onNew(event:Event):void {
			rescueDirty(Globals.reinitialize);
		}
		
		// Handler for "Open" file menuitem.
		public function onOpen(event:Event):void {
			rescueDirty(function():void {
					if (Globals.dataXMLFile == null) {
						var dir:File = File.documentsDirectory;
						try {
							dir.addEventListener(Event.SELECT, XMLHandler.loadDataXML); // clears old data before load
							dir.browseForOpen("Open");					
						} catch(error:Error) {
							trace("Failed:", error.message);
						}
					} else {
						try {
							Globals.dataXMLFile.addEventListener(Event.SELECT, XMLHandler.loadDataXML); // clears old data before load
							Globals.dataXMLFile.browseForOpen("Open");					
						} catch(error:Error) {
							trace("Failed:", error.message);
						}
					}});
		}
		
		// Handler for "Save" file menuitem.
		public function onSave(event:Event):void {
			trace("Globals.xml_file: ", Globals.dataXMLFile);
			if (Globals.dataXMLFile == null) {
				var dir:File = File.documentsDirectory;
				dir = dir.resolvePath('Untitled.xml');
				try {
					dir.addEventListener(Event.SELECT, onFileChosen);
					dir.browseForSave("Save");					
				} catch(error:Error) {
					trace("Failed:", error.message);
				}
			} else {
				try {
					// Don't browse, just overwrite the current xml_file
					XMLHandler.doSaveXml(Globals.dataXMLFile, XMLHandler.generateDataXML());
				} catch(error:Error) {
					trace("Failed:", error.message);
				}
			}
		}
		
		// Handler for "SaveAs" file menuitem.
		public function onSaveAs(event:Event):void {			
			// no previous save: go to Docs folder
			if (Globals.dataXMLFile == null) {
				var dir:File = File.documentsDirectory;
				dir = dir.resolvePath('Untitled.xml');
				try { 
					dir.addEventListener(Event.SELECT, onFileChosen);
					dir.browseForSave("Save As");							
				} catch(error:Error) {
					trace("Failed:", error.message);
				}
			} else { // Go to save folder as previous save
				try {
					Globals.dataXMLFile.addEventListener(Event.SELECT, onFileChosen);
					Globals.dataXMLFile.browseForSave("Save As");					
				} catch(error:Error) {
					trace("Failed:", error.message);
				}
			}
		}		

		private function onFileChosen(evt:Event):void {
			var file:File = File(evt.target);
			if (file.extension != "xml") {
				file.nativePath += ".xml";
			}
			Globals.dataXMLFile = file;			
			Globals.dirty = false;
			Globals.dataXMLFile.removeEventListener(Event.SELECT, onFileChosen); // remove the event listener that got me here
						
			XMLHandler.doSaveXml(Globals.dataXMLFile, XMLHandler.generateDataXML()); // also saves map image
			if (Globals.postSave != null) {
				Globals.postSave();
				Globals.postSave = null; // clean up after myself
			}
		}

		public function onImportGTF(event:Event):void {
			trace(event.target.label);
			var dir:File = File.documentsDirectory;				
			try {
				dir.addEventListener(Event.SELECT, onGtfDirSelected);
				dir.browseForDirectory("Select Google Transit Feed directory:");					
			} catch(error:Error) {
				trace("Failed:", error.message);
			}
		}

		public function onGtfDirSelected(evt:Event):void {
			var dir:File = File(evt.target);	
			try {		
				if (gtfImportUI == null) {
					gtfImportUI = new GtfImportUI();
					gtfImportUI.setFeedDir(dir);
					gtfImportUI.open();
				} else if (gtfImportUI.closed) {
					// can't reopen a closed window.
					gtfImportUI = new GtfImportUI();
					gtfImportUI.setFeedDir(dir);
					gtfImportUI.open();
				} else {
					gtfImportUI.orderToFront(); // bring it into view for them
				}
			} catch (gtfErr:GtfImportError) {
				Alert.show(gtfErr.message);
			}
		}


		// Handler for "Quit" file menuitem.
		public function onQuit(event:Event):void {
			trace(event.target.label);
			/* As recommended by the docs:
			 * http://help.adobe.com/en_US/AIR/1.5/devappsflash/WS5b3ccc516d4fbf351e63e3d118676a5d46-8000.html
			 * we're trying to maintain just one exit path.
			 */  
			var exitingEvent:Event = new Event(Event.EXITING, false, true); 
			NativeApplication.nativeApplication.dispatchEvent(exitingEvent); 
			if (!exitingEvent.isDefaultPrevented()) { 
			    NativeApplication.nativeApplication.exit(); 
			} 
		}

		public function onExiting(event:Event):void {
			trace("Menu.onExiting", event.type, event.currentTarget, "Dirty: ", Globals.dirty);
			// We want to pop an Alert, which happens asychronously, so we just block the initial
			// exit command and manually exit as appropriate.
			event.preventDefault();
			ElevationService.saveCache();
			rescueDirty(Application.application.exit);
		}

		public function rescueDirty(postSave:Function):void {						
			if (Globals.dirty) { // stuff was changed
				if (Globals.dataXMLFile != null) { // there's an existing save file
					// Customize the buttons
					Alert.yesLabel = 'Save';
					Alert.noLabel = 'Discard';
					Alert.buttonWidth = 80; // Discard is a big word!
					Alert.show('Save file "'+Globals.dataXMLFile.nativePath+' ?',
					           'Save',
					           Alert.YES|Alert.NO|Alert.CANCEL,
					           null,
					           function (evt:CloseEvent):void {
					           		  if (evt.detail == Alert.YES) {
					           		  	  // overwrite the existing file
					           		  	  trace("Clicked Yes");
					           		  	  XMLHandler.doSaveXml(Globals.dataXMLFile, XMLHandler.generateDataXML()); // write data
					           		  	  XMLHandler.doSaveXml(Globals.prefsXMLFile, XMLHandler.generatePrefsXML()); // write prefs
					           		  	  postSave();
					           		  } else if (evt.detail == Alert.NO) {
					           		  	  // just save prefs and exit					           		  	  
					           		  	  trace("Clicked No");
					           		  	  XMLHandler.doSaveXml(Globals.prefsXMLFile, XMLHandler.generatePrefsXML()); // write prefs
					           		  	  postSave();
					           		  } else if (evt.detail == Alert.CANCEL) {
					           		  	  // do nothing
					           		  	  trace("Clicked Cancel");  
					           		  } else {
					           		  	  throw new Error("WTF, dude?");
					           		  }
					           })
					// restore to default values					           
					Alert.yesLabel = 'Yes';
					Alert.noLabel = 'No';
					Alert.buttonWidth = 60;
					
				} else { // no existing save file
					// Customize the buttons
					Alert.yesLabel = 'Save';
					Alert.noLabel = 'Discard';
					Alert.buttonWidth = 80;
					Alert.show('Save "Untitled"?',					
					           'Save',
					           Alert.YES|Alert.NO|Alert.CANCEL,
					           null,
					           function (evt:CloseEvent):void {
					           		  if (evt.detail == Alert.YES) {					           		  	
					           		  	  trace("Clicked Yes");					           		  	  
					           		  	  XMLHandler.doSaveXml(Globals.prefsXMLFile, XMLHandler.generatePrefsXML()); // write prefs					           		  	  
					           		  	  onSaveAs(new Event(Event.SELECT));
					           		  	  Globals.postSave = postSave; // Used and cleared by onFileChosen.
					           		  } else if (evt.detail == Alert.NO) {
					           		  	  // just save prefs and exit
					           		  	  trace("Clicked No");
					           		  	  XMLHandler.doSaveXml(Globals.prefsXMLFile, XMLHandler.generatePrefsXML()); // write prefs
					           		  	  postSave();
					           		  } else if (evt.detail == Alert.CANCEL) {
					           		  	  // do nothing
					           		  	  trace("Clicked Cancel");
					           		  } else {
					           		  	  throw new Error("WTF, dude? Learn how to program.");
					           		  }
					           })
					// restore to default values. 
					Alert.yesLabel = 'Yes';
					Alert.noLabel = 'No';
					Alert.buttonWidth = 60;   
				}
			} else { // isn't dirty
				XMLHandler.doSaveXml(Globals.prefsXMLFile, XMLHandler.generatePrefsXML()); // write prefs
				postSave(); 
			}
		}
		
		public function onViewNormal(event:Event):void {
			trace(event.target.label);
			viewNormal.checked = true;
			viewGrade.checked = false;
//			viewElevation.checked = false;
			TrackOverlay.displayMode = TrackOverlay.NORMAL_MODE;
			Utility.refreshScreen();
		}
		
		public function onViewGrade(event:Event):void {
			trace(event.target.label);
			viewNormal.checked = false;
			viewGrade.checked = true;
//			viewElevation.checked = false;
			TrackOverlay.displayMode = TrackOverlay.GRADE_MODE;
			Utility.refreshScreen();			
		}
		
		public function onViewElevation(event:Event):void {
			trace(event.target.label);
			viewNormal.checked = false;
			viewGrade.checked = false;
//			viewElevation.checked = true;
			TrackOverlay.displayMode = TrackOverlay.ELEVATION_MODE;
			Utility.refreshScreen();			
		}

		public function onStatistics(event:Event):void {
			trace(event.target.label);
			var dialog:StatisticsBox = new StatisticsBox();
			PopUpManager.addPopUp(dialog, Globals.map, false);			
		}


		public function onAddress(event:Event):void {
			trace(event.target.label);
			var addressDialog:SearchAddressBox  = new SearchAddressBox();
			PopUpManager.addPopUp(addressDialog, Globals.map, true);
	        PopUpManager.centerPopUp(addressDialog);
			//GoTo.address();
		}

		public function onAutoLevel(event:Event):void {
			trace(event.target.label);
			if (autoLevelUI == null) {
				autoLevelUI = new AutoLevelUI();
				autoLevelUI.open(); 
			} else if (autoLevelUI.closed) {
				// can't reopen a closed window.
				autoLevelUI = new AutoLevelUI();
				autoLevelUI.open();
			} else {
				autoLevelUI.orderToFront(); // bring it into view for them
			}
		}


		public function onUndo(event:Event):void {
			Undo.undo(Undo.PREVIEW); // When undoing the last action, the preview will likely become stale.
			Undo.undo(Undo.USER);
			Utility.refreshScreen();
		}

		public function onAttributes(event:Event):void {
			trace(event.target.label);
			if (attrEditor == null) {
				attrEditor = new AttributesEditor();
				attrEditor.open(); 
			} else if (attrEditor.closed) {
				// can't reopen a closed window.
				attrEditor = new AttributesEditor();
				attrEditor.open();
			} else {
				attrEditor.orderToFront(); // bring it into view for them
			}
		}
		
		public function onPreferences(event:Event):void {
			trace(event.target.label);
			if (prefEditor == null) {
				prefEditor = new PreferencesEditor();
				prefEditor.open(); 
			} else if (prefEditor.closed) {
				// can't reopen a closed window.
				prefEditor = new PreferencesEditor();
				prefEditor.open();
			} else {
				prefEditor.orderToFront(); // bring it into view for them
			}			
		}

		public function onSelectTrackSegment(event:Event):void {
			var trackCount:int = 0;
			for (var key:String in Globals.tracks.segments) {
				trackCount += 1;
			}
			if (trackCount > 0) { 
				var selectTrackGui:SelectTrack = new SelectTrack();
				PopUpManager.addPopUp(selectTrackGui, Globals.map, true);
		        PopUpManager.centerPopUp(selectTrackGui);
		    } else {
		    	Alert.show("No Track segments created.");
		    }
		}

		public function onSelectStation(event:Event):void {
			if (Globals.stations.stations.length > 0) {					
				var selectStationGui:SelectStation = new SelectStation();
				PopUpManager.addPopUp(selectStationGui, Globals.map, true);
		        PopUpManager.centerPopUp(selectStationGui);
		    } else {
		    	Alert.show("No Stations created.");
		    }				
		}

		public function onSelectVehicle(event:Event):void {
			if (Globals.vehicles.vehicles.length > 0) {
				var selectVehicleGui:SelectVehicle = new SelectVehicle();
				PopUpManager.addPopUp(selectVehicleGui, Globals.map, true);
		        PopUpManager.centerPopUp(selectVehicleGui);
		    } else {
		    	Alert.show("No vehicles created.");
		    }		        
		}
		
//		public function onShowKml(event:Event):void {
//			trace(event.target.label);
//		//	trace(track.toKML());
//		}
		
		public function onShowXml(event:Event):void {
			trace(event.target.label);
			trace(XMLHandler.generateDataXML().toXMLString());
			var window:TitleWindow = new TitleWindow();
			window.showCloseButton = true;
			window.title = "DEBUG: XML output";
			window.addEventListener(Event.CLOSE, function (evt:Event):void {PopUpManager.removePopUp(window);});
			var textArea:TextArea = new TextArea();
			textArea.text = XMLHandler.generateDataXML().toXMLString();
			textArea.wordWrap = false;
			textArea.width = 800;
			textArea.height = 600;
			window.addChild(textArea);
			PopUpManager.addPopUp(window, Globals.map);
		}

		public function onFindDuplicateOverlays(event:Event):void {
			trace(event.target.label);
			var length:uint = Globals.tracks.overlays.length;
			for (var i:int=0; i < length-1; ++i) {
				var a:TrackOverlay = Globals.tracks.overlays[i];
				var a_start:LatLng = a.getStart();
				var a_end:LatLng = a.getEnd();
				for (var j:int=i+1; j < length; ++j) {
					var b:TrackOverlay = Globals.tracks.overlays[j];
					var b_start:LatLng = b.getStart();
					var b_end:LatLng = b.getEnd();
					if ((a_start.equals(b_start) && a_end.equals(b_end)) ||
					    (a_start.equals(b_end) && a_end.equals(b_start))) {
					    	trace("Found duplicate overlay!:", a_start.toString(), a_end.toString(), "::", b_start.toString(), b_end.toString(), a===b);					    	
				    }
				}				
				
			}
			trace("Done.");
		}

		public function onZoomIn(event:Event):void {
			var curr_max_zoom:Number = Globals.map.getMaxZoomLevel(Globals.map.getCurrentMapType());					
			if (Globals.map.getZoom() == curr_max_zoom) {
				if (Globals.map.getMaxZoomLevel(MapType.SATELLITE_MAP_TYPE) > curr_max_zoom) {
					Globals.map.setMapType(MapType.SATELLITE_MAP_TYPE); // TODO: Need to update the toolbar, or change the MapType via the toolbar.
					Globals.map.zoomIn();
				}
			} else {
				Globals.map.zoomIn();
			}
		}

		public function onDoc(event:Event):void {
			trace(event.target.label);
			if (htmlHelp == null || htmlHelp.stage.nativeWindow.closed) {
				var initOptions:NativeWindowInitOptions = new NativeWindowInitOptions();
				var bounds:Rectangle = new Rectangle(10, 10, 600, 400);
				htmlHelp = HTMLLoader.createRootWindow(true, initOptions, true, bounds);
				var urlReq:URLRequest = new URLRequest("./edu/ucsc/track_builder/HelpFile.html");
				htmlHelp.load(urlReq);
				htmlHelp.stage.nativeWindow.activate();
			} else {				
				htmlHelp.stage.nativeWindow.orderToFront();
			}
			
		}
		
		public function onAbout(event:Event):void {
			Alert.show("By Dan Homerick \t danhomerick@gmail.com\nElevation data courtesy of geonames.org and NASA's Shuttle Radar Topography Mission (SRTM) dataset", "About");
		}			


	}		
}