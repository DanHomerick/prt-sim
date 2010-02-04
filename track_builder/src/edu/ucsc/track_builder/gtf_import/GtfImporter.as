/** Handles importing a GTFS feed into TrackBuilder.
 * @see http://code.google.com/transit/spec/transit_feed_specification.html
 */

package edu.ucsc.track_builder.gtf_import
{	
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	
	import edu.ucsc.graph.Edge;
	import edu.ucsc.graph.Graph;
	import edu.ucsc.graph.Node;
	import edu.ucsc.neartree.NearTree;
	import edu.ucsc.neartree.Result;
	import edu.ucsc.track_builder.Berth;
	import edu.ucsc.track_builder.Globals;
	import edu.ucsc.track_builder.IdGenerator;
	import edu.ucsc.track_builder.Platform;
	import edu.ucsc.track_builder.Station;
	import edu.ucsc.track_builder.StationOverlay;
	import edu.ucsc.track_builder.TrackOverlay;
	import edu.ucsc.track_builder.TrackSegment;
	import edu.ucsc.track_builder.Utility;
	import edu.ucsc.track_builder.Vehicle;
	import edu.ucsc.track_builder.VehicleOverlay;
	
	import flash.events.TimerEvent;
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	import flash.geom.Vector3D;
	
	import mx.collections.XMLListCollection;
	
	public class GtfImporter
	{
		public static var gtfXml:XML;
		
		/** How many places after the decimal to record coordinates in. Reducing precision can be used as a
		 * simple way of merging nodes that are in close proximity to each other. */ 
		public var PRECISION:uint = 5;
		public var STATION_BERTH_LENGTH:Number = 15; // meters
		public var STATION_BERTH_COUNT:uint = 4;
		public var STATION_COVERAGE_RADIUS:Number = 150; // meters
		public var VEHICLE_SPEED:Number = 15; // 15 meters/sec == 34 miles/hour
		
//		protected var progressBar:ProgressBar;
		public var feedDirectory:File;
		
		protected var _routes:Vector.<Route>;
		protected var _services:Object;
		protected var _trips:Vector.<Trip>;
		protected var _stops:Object;
		protected var _stopTimes:StopTimeCollection;
		
		protected var wsRegExp:RegExp;
		
		/** Constructor */
		public function GtfImporter(feedDirectory:File) {
			this.feedDirectory = feedDirectory;
			
			wsRegExp = new RegExp("^[\\s\\t]+|[\\s\\t]+$", "g"); // remove leading or trailing whitespace
		}
		
//		public function setProgressBar(progressBar:ProgressBar):void {
//			this.progressBar = progressBar;
//		}
	
		public function getRoutes():Vector.<Route> {
			if (_routes == null) {
				_routes = parseRoutes();
			}
			return _routes;
		}

		/** All available services.
		 * @throw GtfImportError Thrown if neither calendar.txt or calendar_dates.txt is found.
		 * @return An Object being used as a dictionary. The keys are the unique service_ids and the values are Service instances. */ 
		public function getServices():Object {
			if (_services == null) {
				var failCount:uint = 0;			
				try {
					_services = parseCalendar();
				} catch (err:GtfImportError) {
					failCount += 1;
				}
				
				try {
					_services = parseCalendarDates(_services);
				} catch (err:GtfImportError) {
					failCount += 1;
				}
				
				if (failCount == 2) {
					throw new GtfImportError('Failed to find either calendar.txt or calendar_dates.txt');
				}
			}
			return _services;
		}

		public function getTrips():Vector.<Trip> {
			if (_trips == null) {
				_trips = parseTrips();				
			}
			return _trips;
		}

		public function getStops():Object {
			if (_stops == null) {
				_stops = parseStops();
			}
			return _stops;
		}
		
		public function getStopTimes():StopTimeCollection {
			if (_stopTimes == null) {
				_stopTimes = parseStopTimes();
			}
			return _stopTimes;
		}
		
		/** Get all service dates for the supplied routes. Uses data from the trips.txt
		 * file to associate the routes with service dates.
		 * @param routes Contains the Route instances of interest.
		 * @return An Object being used as a dictionary. The keys are unique service_ids and the values are Service instances.
		 */   
		public function getServicesFromRoutes(routes:Array):Object {
			var trips:Vector.<Trip> = getTrips();
			trips.sort(Trip.compareBy('routeId'));
			
			var result:Object = new Object();
			var services:Object = getServices();
			for each (var route:Route in routes) {
				var route_id:String = route.id;
				var found:Boolean = false;
				for each (var trip:Trip in trips) {
					if (route_id == trip.routeId) {
						found = true;
						var service:Service = services[trip.serviceId];
						result[service.id] = service;						
					} else if (found) { // trips are sorted by route. When I hit the first match, there will be a streak, then no more.
						break;
					}
				}
			}
			return result;
		}

		// Todo: break this up...
		/** Adds edges to the graph for the offramp/boarding/onramp parts of a station.
		 * @param stopIds All stops to be added. StopIds may not be unique.
		 * @param stops An associative array where the key is a stop_id and the value is a Stop instance.
		 * @return An associative array where the key is an edgeString (in 'node.key|node.key' format) and the value is the corresponding Stop instance.
		 */
		public function addStops(stopIds:Vector.<String>, stops:Object, graph:Graph, tree:NearTree):Object {
			var stationEdges:Object = new Object();
			var nodesAdded:Object = new Object();
						
			for each (var stopId:String in stopIds) {												
				var stop:Stop = stops[stopId];
				var stationStartNode:Node = Node.fromLatLng(stop.latlng);

				/* Skip stops that have been added previously */
				if (nodesAdded[stationStartNode] != null) {
					continue;
				}
	
				/* Check if the stop overlaps an existing node (debug) */				
				if (graph.hasNode(stationStartNode)) { // overlaps
					trace("Station directly overlaps graph node");	
				} else {
					trace("Station separate from graph nodes");
				}
	
				// find the closest point to the stop. The tree is not being updated, so nodes
				// added during addStops will not be found.
				var nearResult:Result = tree.nearest(stationStartNode.latlng);
				var nearNode:Node = Node.fromLatLng(nearResult.latlng);
				
				// look through points adjacent, and of those, find the one that's closest to stop
				var adjNodes:Vector.<Node> = graph.getNeighbors(nearNode);
				var dist:Number = Number.POSITIVE_INFINITY;
				var closestAdjNode:Node;
				for each (var adjNode:Node in adjNodes) {
					var tmpDist:Number = nearNode.latlng.distanceFrom(adjNode.latlng);
					/* It's closer, and it's not a node that I just added */
					if (tmpDist < dist && nodesAdded[adjNode] == null) {
						dist = tmpDist;
						closestAdjNode = adjNode;
					}
				}
				
				// make a new latlng that will be the end of the station segment
				var direction:Vector3D = Utility.calcVectorFromLatLngs(nearNode.latlng, closestAdjNode.latlng);
				direction.scaleBy((STATION_BERTH_LENGTH+0.5)*STATION_BERTH_COUNT/direction.length); // make it as long as required
				var stationEnd:LatLng = Utility.calcLatLngFromVector(stationStartNode.latlng, direction);
				var stationEndNode:Node = Node.fromLatLng(stationEnd);
				
				/* Connect the station segment to the rest of the track in the graph. */ 				
				graph.addNode(stationStartNode);
				graph.addNode(stationEndNode);
				graph.addEdge(stationStartNode, stationEndNode);
				graph.addEdge(nearNode, stationStartNode);
				graph.addEdge(stationEndNode, closestAdjNode);

				// Keep track of what edges are associated with stations.
				var edge:Edge = new Edge(stationStartNode, stationEndNode);				
				stationEdges[edge] = stop;
				nodesAdded[stationStartNode] = true;
				nodesAdded[stationEndNode] = true;
			}
			return stationEdges;
		}
		
		/**
		 * Creates the TrackSegments and Track Overlays. Makes TrackSegments for both directions at the same time,
		 * bundles them into a Vector, and creates an overlay.
		 * @param graph A Graph instance. Two TrackSegments will be formed for each edge in the graph (one for each direction), and one TrackOverlay for each edge.
		 * @return An associative array where the keys are edge strings in tail|head format and the values are TrackOverlays.
		 * @see #.makeStations 
		 */
		public function makeTracks(graph:Graph):Object {
			trace("MakeTracks");				
			var overlays:Object = new Object();
			var edges:Vector.<Edge> = graph.getEdges();
			for each (var edge:Edge in edges) {
				var tsVec:Vector.<TrackSegment> = new Vector.<TrackSegment>(2);
				tsVec[0] = new TrackSegment(IdGenerator.getTrackSegId(TrackSegment.forwardExt),
					                                   "",
					                                   edge.tail.latlng.clone(),
					                                   edge.head.latlng.clone(),
					                                   NaN, 0, 0, NaN, 0, null, false);
				tsVec[1] = tsVec[0].clone(true);
				var trackOverlay:TrackOverlay = new TrackOverlay(tsVec);
				// store both edge directions to make lookup easier and faster
				var reverseEdge:Edge = new Edge(edge.head, edge.tail);
				overlays[edge] = trackOverlay;				
				overlays[reverseEdge] = trackOverlay;
			}
			
			return overlays;
		}
		
		/**
		 * For each node, connect all overlays that are incident with the node together. If only one overlay
		 * is adjacent to a node, then connect it to itself (allowing a U-turn).
		 * @param overlays An associative array where the keys are in edge string format and the values are TrackOverlays.
		 * @see #.makeTracks 
		 */
		public function connectOverlays(graph:Graph, overlays:Object):void {
			var nodes:Object = graph.getNodes();
			for each (var node:Node in nodes) {
				var adjNodes:Vector.<Node> = graph.getNeighbors(node);
				
				/* Node of degree 1. Connect the terminal end of the bidirectional TrackOverlay to itself. */
				if (adjNodes.length == 1) {
					trace("GtfImporter.connectOverlays: Self connecting a TrackOverlay.");
					var edge:Edge = new Edge(node, adjNodes[0]);					
					var overlay:TrackOverlay = overlays[edge];
					var fwd:TrackSegment;
					var rev:TrackSegment;
					if (overlay.getStart().equals(node.latlng)) {
						fwd = overlay.segments[TrackOverlay.FWD];
						rev = overlay.segments[TrackOverlay.REV];
						fwd.prev_ids.push(rev.id);
						rev.next_ids.push(fwd.id);
					} else {
						fwd = overlay.segments[TrackOverlay.REV];
						rev = overlay.segments[TrackOverlay.FWD];
						fwd.next_ids.push(rev.id);
						rev.prev_ids.push(fwd.id);
					}
				}
				// Normal case, where degree > 1;
				else {
					for (var i:int = 0; i < adjNodes.length; ++i) {
						var iNode:Node = adjNodes[i];
						var iEdge:Edge = new Edge(node, iNode);
						var iOverlay:TrackOverlay = overlays[iEdge];
						  
						for (var j:int = i+1; j < adjNodes.length; ++j) {
							var jNode:Node = adjNodes[j];
							var jEdge:Edge = new Edge(node, jNode);						
							var jOverlay:TrackOverlay = overlays[jEdge];
							iOverlay.connect(jOverlay);
						}
					}
				}
			}
		}
		
		/** Must be called after the TrackOverlays have been connected. The direction of the station is unknown,
		 * and is made in an arbitrary direction. Since the on/offramps are bidirectional, the station is
		 * accessible even if it is the 'wrong way.'
		 * @param stationEdges An associative array whose keys are the edges that are designated as stations (stops), in 
		 *         edge string format. Values are the corresponding Stop instances. 
		 * @param trackOverlays An associative array where the keys are in edge string format and the values are TrackOverlays.
		 * @return An associative array where keys are stopIds and values are Station instances.
		 */		
		public function makeStations(stationEdges:Object, trackOverlays:Object):Object {
			trace('MakeStations');
			var stations:Object = new Object;
			for (var edgeKey:String in stationEdges) {
				var stop:Stop = stationEdges[edgeKey];
				var boardingOverlay:TrackOverlay = trackOverlays[edgeKey];
				var boardingSeg:TrackSegment = boardingOverlay.segments[TrackOverlay.FWD];
				var segs:Vector.<TrackSegment> = new Vector.<TrackSegment>();
				segs.push(Globals.tracks.getTrackSegment(boardingSeg.prev_ids[0]));
				segs.push(boardingSeg); 
				segs.push(Globals.tracks.getTrackSegment(boardingSeg.next_ids[0]));
				
				var platforms:Vector.<Platform> = new Vector.<Platform>();
				var platform:Platform = new Platform(boardingSeg.id, 0);
				// The true boardingSeg.length varies a bit, and we want to guarantee the berths remain fully on this seg
				var berthLength:Number = Math.min(boardingSeg.length/STATION_BERTH_COUNT, STATION_BERTH_LENGTH) 
				for (var i:uint=0; i < STATION_BERTH_COUNT; ++i) {
					platform.berths.push(new Berth(i, i*berthLength, (i+1)*berthLength, true, true))
				}
				platforms.push(platform);
				var id:String = IdGenerator.getStationId();
				var station:Station = new Station(id, stop.name, segs, platforms, STATION_COVERAGE_RADIUS, 0, 0);			
				new StationOverlay(station); // placed in the global store by side effect
				
				stations[stop.id] = station;
			}
			return stations;
		}

		/** Creates vehicles and vehicles overlays and adds them to the global store.
		 * @return A associative array with tripIds as keys and Vehicle instances as values.
		 */
		public function makeVehicles(rBundles:Vector.<RouteBundle>, graph:Graph, trackOverlays:Object, stops:Object, stopTree:NearTree, stations:Object, time:Time):Object {
			trace('MakeVehicles');						
			var tripIds2vehicles:Object = new Object();
			var vehicle:Vehicle;
			var vehicleSeg:TrackSegment;
			for each (var rBundle:RouteBundle in rBundles) {
				/* Figure out how many vehicles are required for this route, and associate each trip with a vehicle.
				 * This is very similiar to the Interval Partitioning Problem (aka Interval Coloring Problem)
				 * which has a greedy solution. The difference is that here we have the extra requirement
				 * that for a particular vehicle V, each of the intervals (trips) ends at a station that
				 * is near to the next trip's starting station. Considering that we are working with
				 * rational schedules, this requirement is generally easy to meet -- where it cannot be met
				 * we simply create an additional vehicle.
				 */	
				
				var availableVehicles:Vector.<Vector.<TripBundle>> = new Vector.<Vector.<TripBundle>>();
				var vSchedule:Vector.<TripBundle>;
				
				/* Sort the trips by their starting times */
				rBundle.tripBundles.sort(function(tBundle1:TripBundle, tBundle2:TripBundle):Number {
					                         return tBundle1.beginStopTime.arrival.toSeconds() - 
					                                    tBundle2.beginStopTime.arrival.toSeconds();
										});
									
				for each (var tBundle:TripBundle in rBundle.tripBundles) {					
					var assigned:Boolean = false;
					/* Look through the existing roster of vehicles for one that is available and parked at the right stop. */	
					for each (var availVehicle:Vector.<TripBundle> in availableVehicles) {
						var availTime:Number = availVehicle[availVehicle.length-1].endStopTime.departure.toSeconds();
						var availStopId:String = availVehicle[availVehicle.length-1].endStopTime.stopId;						
						if (availTime <= tBundle.beginStopTime.arrival.toSeconds()) {
							var availStop:Stop = stops[availStopId];							
							var nearAvailStops:Vector.<Result> = stopTree.withinRadius(availStop.latlng, 100); // chose 100 meters as likely maximum station size
							for each (var nearResult:Result in nearAvailStops) {
								for each (var s:Stop in nearResult.objs) {									
									if (s.id == tBundle.beginStopTime.stopId) { 
										availVehicle.push(tBundle);
										assigned = true;
										break;								
									}									
								}
								if (assigned) break;								
							}
							if (assigned) break;							
						}						
					}
					/* If we don't find a suitable vehicle, then expand the roster of available vehicles */
					if (!assigned) {
						vSchedule = new Vector.<TripBundle>();
						vSchedule.push(tBundle);
						availableVehicles.push(vSchedule);
						
					}
				}
				
				/* Now that schedules for each required vehicle have been planned, create the vehicles at
				 * appropriate starting points.
				 */
				var placedVehicles:Vector.<Vehicle> = new Vector.<Vehicle>()
				for each (vSchedule in availableVehicles) {
					tBundle = vSchedule[0]; // consider the first trip
					
					/* Case 1: At the simulation start time, the vehicle has no active trips. Note that
					 * trips that complete prior to the sim start time are already culled from the data. */					
					if (tBundle.beginStopTime.arrival.toSeconds() > time.toSeconds()) {
						var station:Station = stations[vSchedule[0].beginStopTime.stopId];
						vehicle = makeVehicleAtStation(station, placedVehicles);							
					}

					/* For other cases, walk through all the stop times of a vehicle's first trip, 
					 * and find the approximate position of the vehicle at the sim start. 
					 * Place the vehicle at the appropriate station or on the appropriate track segment.
					 */											
					else {										
						var placedVehicle:Boolean = false;
						var stopTimes:Vector.<StopTime> = tBundle.stopTimes;
						for (var i:int=0; i < stopTimes.length; ++i) {
						    var currStopTime:StopTime = stopTimes[i]; // current
							
							/* Already left this stop */
							if (time.toSeconds() > currStopTime.departure.toSeconds()) {
								// do nothing
							}
							
							/* Case 2: Waiting at a stop */
							else if (time.toSeconds() >= currStopTime.arrival.toSeconds() && 
							    		time.toSeconds() <= currStopTime.departure.toSeconds()) {
							    station = stations[currStopTime.stopId];
							    vehicle = makeVehicleAtStation(station, placedVehicles);
							    placedVehicle = true;
							    break;											              	
							}
													
							/* Case 3: Between two stations */
							else if (time.toSeconds() <= currStopTime.arrival.toSeconds()) {	
								/* Determine the shortest path between the previous and current stations */
								var prevStopTime:StopTime = stopTimes[i-1];
								var currStop:Stop = stops[currStopTime.stopId];
								var prevStop:Stop = stops[prevStopTime.stopId];
								var currStopNode:Node = Node.fromLatLng(currStop.latlng);
								var prevStopNode:Node = Node.fromLatLng(prevStop.latlng);
								var pathEdges:Vector.<Edge> = graph.shortestPathEdges(prevStopNode, currStopNode);
								
								// map from Edges to TrackOverlays
								var pathOverlays:Vector.<TrackOverlay> = new Vector.<TrackOverlay>(pathEdges.length);
								for (var j:int=0; j < pathEdges.length; ++j) {
									pathOverlays[j] = trackOverlays[pathEdges[j]];
								}
								
								var pathDist:Number = 0;
								pathOverlays.forEach(function(item:TrackOverlay, idx:int, vec:Vector.<TrackOverlay>):void
								                          {pathDist += item.h_length;});
	
								/* Find what fraction of the time between stops has elapsed, and thus approximate what distance
								 * from the previous station the vehicle should be.
								 */
								var timeFraction:Number =  (time.toSeconds() - prevStopTime.departure.toSeconds()) /
								                             (currStopTime.arrival.toSeconds() - prevStopTime.departure.toSeconds());
								var distRemaining:Number = pathDist * timeFraction;
								
								/* Interpolate the vehicle position, and create it */
								for each (var tOverlay:TrackOverlay in pathOverlays) {
									var len:Number = tOverlay.h_length;
									if (distRemaining >= len) {
										distRemaining -= len;
										continue;
									}
									
									/* Each overlay has two segments. Infer which one the vehicle should be
									 * placed on by testing to see which segment has an endpoint closer to the
									 * next stop.
									 */
									if (tOverlay.segments[0].getEnd().distanceFrom(currStop.latlng) < 
									    	tOverlay.segments[1].getEnd().distanceFrom(currStop.latlng)) {
										vehicleSeg = tOverlay.segments[0];
							    	} else {
							    		vehicleSeg = tOverlay.segments[1];
							    	}
									
									/* Make the vehicle, and overlay */
									vehicle = new Vehicle(
							                            IdGenerator.getVehicleId(),
							                            distRemaining,
							                            VEHICLE_SPEED,
							                            0,
							                            vehicleSeg,
							                            vehicleSeg.getLatLng(distRemaining),
							                            vehicleSeg.getElevation(distRemaining),
							                            tBundle.trip.id,
							                            'BUS', // FIXME: Don't hardcode for just buses!
							                            false
							                          );
									new VehicleOverlay(vehicle, tOverlay); // added to global store as a side effect
									placedVehicle = true; 
									break; // break the iteration over pathOverlays				
								} // end iteration over pathOverlays
							} // end case 3
							if (placedVehicle) break; // break the iteration over stopTimes
						} // end iteration over stopTimes					
					} // end case 2 and case 3
					
					/* Use the just created vehicle for the remainder of the trips in its schedule. */					
					for each (tBundle in vSchedule) {
						tripIds2vehicles[tBundle.trip.id] = vehicle;
					}					
					placedVehicles.push(vehicle);
				} // end iteration over availableVehicles
			} // end iteration over rBundles					
			return tripIds2vehicles;
		}
	
		/** Creates a vehicle in station. Places the vehicle in an unoccupied berth, as close to the station
		 * exit as possible. Also creates an overlay for the vehicle, though only the vehicle is returned.
		 * @param station Expected to have a single platform.
		 * @param vehicles An object used as a dictionary, containing the vehicles that are already created. 
		 * @throws GtfImportError if no unoccupied berths are available.
	     */
		public function makeVehicleAtStation(station:Station, vehicles:Object):Vehicle {
			// Find the berth closest to the station exit which is not already occupied.
			var berth_idx:int = station.platforms[0].berths.length-1
			for each (var v:Vehicle in vehicles) {							
				if (v.location.id == station.platforms[0].trackSegId) {
					berth_idx -= 1
				}
			}
			if (berth_idx < 0) { // Whoops. Too many vehicles are already parked in that station.
				throw new GtfImportError("Insufficient vehicle berths for starting vehicles.");
			} else {
				var berth:Berth = station.platforms[0].berths[berth_idx];
				var pos:Number = berth.endPos - 0.1; // Give 10cm between the vehicle nose and the end of the berth
				var trackSeg:TrackSegment = Globals.tracks.getTrackSegment(station.platforms[0].trackSegId);  
				var vehicle:Vehicle = new Vehicle(
							              	IdGenerator.getVehicleId(),
							              	pos,
							              	0,
							              	0,
							              	trackSeg,
				                            trackSeg.getLatLng(pos),
				                            trackSeg.getElevation(pos),
				                            '',
				                            'BUS', // FIXME: Don't hardcode for Buses!
				                            false);
			}
			
			new VehicleOverlay(vehicle, Globals.tracks.getTrackOverlay(vehicle.location.id), false);
			return vehicle
		}
	
	
		/** Gives each stops' closest neighbors, and the distances separating them.
		 * @param stopTree Contains Stop instances.
		 * @param allStops An associative array where keys are stopIds and values are Stop instances.
		 * @param stations An associative array where keys are stopIds and values are Station instances.
		 * @param maxDist Maximum inter-stop distance to give info for.
		 * @returns XML containing the info.        
		 */
		public function makeNeighborsXml(stopTree:NearTree, allStops:Object, stations:Object, maxDist:Number):XML {
			var neighborsXml:XML = <Neighbors/>
			for (var stopId:String in stations) {
				var stop:Stop = allStops[stopId]							
				var station:Station = stations[stopId];
				var stationXml:XML = <Station id={station.id}/>
				var results:Vector.<Result> = stopTree.withinRadius(stop.latlng, maxDist);
				for each (var result:Result in results) {
					for each (var neighborStop:Stop in result.objs) {
						if (neighborStop.id != stop.id) { // don't include myself as a neighbor							
							var neighborStation:Station = stations[neighborStop.id]
							if (neighborStation != null) { // don't include the unimported stops	
								var dist:Number = Math.round(stop.latlng.distanceFrom(neighborStop.latlng))						
								stationXml.appendChild(<Neighbor station_id={neighborStation.id} distance={dist} />)
							}
						}
					}				
				}
				neighborsXml.appendChild(stationXml)			
			}
			return neighborsXml
		}
	
		public function makeRoutesXml(vehicles:Object, stations:Object, rBundles:Vector.<RouteBundle>):XMLListCollection {
			var collection:XMLListCollection = new XMLListCollection()
			for each (var rBundle:RouteBundle in rBundles) {
				var routeXml:XML = <Route route_id={rBundle.route.id} service_id={rBundle.service.id} />;				
				for each (var tBundle:TripBundle in rBundle.tripBundles) {
					var trip:Trip = tBundle.trip;
					var vehicle:Vehicle = vehicles[trip.id];
					var vehicleId:String = vehicle ? vehicle.id : 'undefined';
					var tripXml:XML = <Trip trip_id={trip.id} vehicle_id={vehicleId} />;
					var stopTimes:Vector.<StopTime> = tBundle.stopTimes;
					for each (var stopTime:StopTime in stopTimes) {
						var stopXml:XML = <Stop station_id={stations[stopTime.stopId].id} />
						stopXml.appendChild(<Arrival>{stopTime.arrival.toString()}</Arrival>);
						stopXml.appendChild(<Departure>{stopTime.departure.toString()}</Departure>);
						tripXml.appendChild(stopXml);
					}
					routeXml.appendChild(tripXml);
				}
				collection.addItem(routeXml)
			}				
			return collection;
		}
	
		/** Parses the txt files residing in feedDirectory and creates TrackSegments and TrackOverlays accordingly.
		 * @param routes Routes to import.
		 * @param services Services to import
		 * @param date The date being simulated.
		 * @param time The simulation start time.
		 * 		  Throws an GtfImportError if it encounters a problem.
		 */  
		public function doImport(routes:Array, services:Array, date:Date, time:Time):void {	
			/* Get the trips and shapeIds associated with the selected routes and date */
			var allTrips:Vector.<Trip> = getTrips();
			var stCollection:StopTimeCollection = getStopTimes();
			var shapeIds:Object = new Object(); // an Object used as a Set.			
			var rBundles:Vector.<RouteBundle> = new Vector.<RouteBundle>();
			for each (var route:Route in routes) {				
				for each (var service:Service in services) {
					rBundles.push(new RouteBundle(route, service));
					var rBundle:RouteBundle = rBundles[rBundles.length-1];
					for each (var trip:Trip in allTrips) {						
						if (trip.routeId == route.id && trip.serviceId == service.id) {
							var tBundle:TripBundle = new TripBundle(trip, stCollection.getByTrip(trip))
							rBundle.tripBundles.push(tBundle);
							shapeIds[trip.shapeId] = true;
						}
					}
					rBundle.removeEarlyTrips(time); // cull trips that complete before the sim starts
				}
			}
			
			/* Build a graph that holds the connectivity */
			Graph.PRECISION = PRECISION;
			var shapes:Object = parseShapes(shapeIds);
			var graph:Graph = Graph.fromShapes(shapes);
			
			/* Build a NearTree */
			NearTree.MERGE_THRESHOLD = 10; // meters
			var shapeArray:Array = new Array();			
			var latlngVec:Vector.<LatLng> = new Vector.<LatLng>();
			for each (var sv:Vector.<ShapeNode> in shapes) {
				for each (var sn:ShapeNode in sv) {
					shapeArray.push(sn);
					latlngVec.push(sn.latlng);
				}
			}		
			var tree:NearTree = NearTree.makeTree(latlngVec, shapeArray);
			
			/* Use the tree to find nodes that are in close proximity to each
			 * other. Merge them in the graph, without distorting the connectivity.
			 * Note that the tree is not updated.*/
			tree.traverse(function (array:Array):void {
					/* Make an adapter shim */
					var vec:Vector.<Node> = new Vector.<Node>();
					for each (var s:ShapeNode in array) {
						vec.push(Node.fromLatLng(s.latlng));
					}
					graph.mergeNodes(vec, vec[0].latlng); // merge all nodes in vec to vec[0]'s latlng
					trace('merged', vec.length, 'nodes.');
					});			
			trace('Graph node count (after merges):', graph.getNodeCount());

			/* Find stops that are used by the chosen routes/services and add them to the graph. */
			var allStops:Object = getStops();
			var stopIds:Vector.<String> = new Vector.<String>();						
			for each (var rBundle:RouteBundle in rBundles) {
				for each (var tBundle:TripBundle in rBundle.tripBundles) {
					stopIds = stopIds.concat(tBundle.getStopIds());
				}
			}								
			var stationEdges:Object = addStops(stopIds, allStops, graph, tree);
			tree = null; // allow garbage collection			
			
			/* Simplify straight sections into long segments. */			
			var keep:Object = new Object(); //Ensure that we do not remove one of the nodes that we added in addStops.
			for (var edgeStr:String in stationEdges) {
				var nodeStrings:Array = edgeStr.split('|');
				keep[nodeStrings[0]] = true;
				keep[nodeStrings[1]] = true;
			}
			graph.simplify(keep);

			/* Make the TrackSegment instances and TrackOverlay instances */
			var trackOverlays:Object = makeTracks(graph);
			
			/* Make connections between all overlays, based on connectivity of the graph */
			connectOverlays(graph, trackOverlays);

			/* Make Station and StationOverlay instances */
			var stations:Object = makeStations(stationEdges, trackOverlays);

			
			/* Make a near tree containing just the locations of the stops */
			var stopLatLngs:Vector.<LatLng> = new Vector.<LatLng>();
			var stopInstances:Array = new Array();
			for each (var stop:Stop in allStops) {
				stopLatLngs.push(stop.latlng);
				stopInstances.push(stop);
			}
			var stopTree:NearTree = NearTree.makeTree(stopLatLngs, stopInstances);
			
			/* Make Vehicles and VehcleOverlays */
			var vehicles:Object = makeVehicles(rBundles, graph, trackOverlays, allStops, stopTree, stations, time);
			graph = null; // allow the graph to be garbage collected

			/* Make and store the GTF save data */
			gtfXml = <GoogleTransitFeed start_time={time.toString()}/>;
			for each (var routeXml:XML in makeRoutesXml(vehicles, stations, rBundles)) { 
				gtfXml.appendChild(routeXml);
			}
			gtfXml.appendChild(makeNeighborsXml(stopTree, allStops, stations, 500));

			// Change the location and zoom to show the entire track.
			var bounds:LatLngBounds = Globals.tracks.getLatLngBounds();
			Globals.map.setCenter(bounds.getCenter());
			var zoom:Number = Globals.map.getBoundsZoomLevel(bounds);
			Globals.map.setZoom(zoom);
			
			Globals.dirty = true;
		}
		
		/** Reads the file into a string, figures out the line endings (LF or CRLF), and returns an array of line strings.
		 */
		public function readFile(file:File):Array {			
			var fileStream:FileStream = new FileStream();
			var data:Array;
			try {
				fileStream.open(file, FileMode.READ);
				var str:String = "";
				// pull the whole file into memory. Even with a 50 meg text file... Eh. RAM is plentiful.
				while (fileStream.bytesAvailable > 0) {
					str += fileStream.readUTFBytes(fileStream.bytesAvailable);
				}
				if (str.indexOf("\r\n") != -1) { // Windows line endings
					data = str.split("\r\n");
				} else {
					data = str.split("\n");
				}
				
			} catch (err:Error) {
				// translate the Error into something that is (hopefully) a little more useful to the user.
				throw new GtfImportError("Unable to find " + file.name + " file in the directory: " + file.nativePath + "\nIs this a valid Google Transit Feed?");
			} finally {
				// regardless of whether or not errors occurred, close the file (safe to do, even if file not open).
				fileStream.close(); 
			}
			
			return data;
		}

		public function parseTrips():Vector.<Trip> {
			trace("parseTrips");
			/* Field indexes */
			var ROUTE:uint = NaN; // route_id, required
			var SERVICE:uint = NaN; // service_id, required
			var TRIP:uint = NaN; // trip_id, required
			var HEADSIGN:uint = NaN; // trip_headsign, optional
			var SHORT:uint = NaN; // trip_short_name, optional
			var DIR:uint = NaN; // direction_id, optional
			var BLOCK:uint = NaN; // block_id, optional
			var SHAPE:uint = NaN; // shape_id, optional
			
			var tripsFile:File = feedDirectory.resolvePath('trips.txt');
			var tripsData:Array = readFile(tripsFile);
			
			/* Identify field indexes */
			var headerFields:Array = tripsData.shift().split(',');
			for (var i:int=0; i < headerFields.length; ++i) {
				switch(stripWS(headerFields[i])) {
					case "route_id": ROUTE = i; break;
					case "service_id": SERVICE = i; break;
					case "trip_id": TRIP = i; break;
					case "trip_headsign": HEADSIGN = i; break;
					case "trip_short_name": SHORT = i; break;
					case "direction_id": DIR = i; break;
					case "block_id": BLOCK = i; break;
					case "shape_id": SHAPE = i; break;
					default: break; // ignore it
				}
			}
			
			var trips:Vector.<Trip> = new Vector.<Trip>();
			for each (var row:String in tripsData) {
				var fields:Array = row.split(',');
				if (fields.length == 1) { // blank line or other non-data info
					continue;
				}
				trips.push(new Trip(stripWS(fields[ROUTE]),
				                    stripWS(fields[SERVICE]),
				                    stripWS(fields[TRIP]),
				                    HEADSIGN ? fields[HEADSIGN] : "",
				                    SHORT ? fields[SHORT] : "",
				                    DIR ? fields[DIR] : "",
				                    BLOCK ? stripWS(fields[BLOCK]) : "",
				                    SHAPE ? stripWS(fields[SHAPE]) : ""));		
			}
			return trips;
		}


		public function parseStops():Object {
			trace("parseStops");
			/* Field indexes */
			var ID:uint = NaN; // stop_id, required
			var NAME:uint = NaN; // stop_name, required
			var DESC:uint = NaN; // stop_desc, optional			
			var LAT:uint = NaN; // stop_lat, required
			var LON:uint = NaN; // stop_lon, required
			var TYPE:uint = NaN; // location_type, optional, may be blank
			var PARENT:uint = NaN; // parent_station, optional

			var stopsFile:File = feedDirectory.resolvePath('stops.txt');
			var stopsData:Array = readFile(stopsFile);
			var stops:Object = new Object();
			
			/* Figure out what fields are being used, and what indexes they have */									
			var headerFields:Array = stopsData.shift().split(',');
			for (var i:int=0; i < headerFields.length; ++i) {
				switch(stripWS(headerFields[i])) {
					case "stop_id": ID = i; break;
					case "stop_name": NAME = i; break;
					case "stop_desc": DESC = i; break;
					case "stop_lat": LAT = i; break;
					case "stop_lon": LON = i; break;
					case "location_type": TYPE = i; break;
					case "parent_station": PARENT = i; break;
					default: break; // ignore it
				}
			}
			
			/* Parse the csv file, creating Stop objects */
			var fields:Array;
			var latlng:LatLng;
			var type:uint;
			var parent:uint;
			var row:String;
			var id:String;
			if (DESC && PARENT) { // PARENT implies that TYPE must be used	
				for each (row in stopsData) {
					fields = row.split(',');			
					if (fields.length == 1) { // blank line or other non-data info
							continue;
					}					
					latlng = new LatLng(toPrecision(fields[LAT]), toPrecision(fields[LON]));
					type = fields[TYPE] ? 1 : 0;
					id = stripWS(fields[ID]);
					stops[id] = new Stop(id, fields[NAME], latlng, fields[DESC], type, fields[PARENT]);
				}
			} else if (DESC) {
				for each (row in stopsData) {
					fields = row.split(',');			
					if (fields.length == 1) { // blank line or other non-data info
							continue;
					}
					latlng = new LatLng(toPrecision(fields[LAT]), toPrecision(fields[LON]));
					id = stripWS(fields[ID]);
					stops[id] = new Stop(id, fields[NAME], latlng, fields[DESC]);
				}
			} else if (PARENT) {
				for each (row in stopsData) {
					fields = row.split(',');			
					if (fields.length == 1) { // blank line or other non-data info
							continue;
					}
					latlng = new LatLng(toPrecision(fields[LAT]), toPrecision(fields[LON]));
					type = fields[TYPE] ? 1 : 0;
					id = stripWS(fields[ID]);
					stops[id] = new Stop(id, fields[NAME], latlng, "", type, fields[PARENT]);
				}
			} else {
				for each (row in stopsData) {
					fields = row.split(',');			
					if (fields.length == 1) { // blank line or other non-data info
							continue;
					}
					latlng = new LatLng(toPrecision(fields[LAT]), toPrecision(fields[LON]));
					id = stripWS(fields[ID]);
					stops[id] = new Stop(id, fields[NAME], latlng);
				}
			}
			
			return stops;
		}

		public function parseRoutes():Vector.<Route> {
			trace("parseRoutes");
			/* Fields */
			var ROUTE:uint = NaN; // route_id, required
			var AGENCY:uint = NaN; // agency_id, optional
			var SHORT:uint = NaN; // route_short_name, req.
			var LONG:uint = NaN; // route_long_name, req.
			var DESC:uint = NaN; // route_desc, opt.
			var TYPE:uint = NaN; // route_type, req.
			var URL:uint = NaN; // route_url, opt.
			var COLOR:uint = NaN; // route_color, opt.
			
			var routeFile:File = feedDirectory.resolvePath('routes.txt');
			var routeData:Array = readFile(routeFile);
			
			/* Figure out what fields are being used, and what indexes they have */
			var headerFields:Array = routeData.shift().split(',');
			for (var i:int=0; i < headerFields.length; ++i) {
				switch(stripWS(headerFields[i])) {
					case "route_id": ROUTE = i; break;
					case "agency_id": AGENCY = i; break;
					case "route_short_name": SHORT = i; break;
					case "route_long_name": LONG = i; break;
					case "route_desc": DESC = i; break;
					case "route_type": TYPE = i; break;
					case "route_url": URL = i; break;
					case "route_color": COLOR = i; break;
					default: break; // ignore it
				}
			}
			
			var routes:Vector.<Route> = new Vector.<Route>();
			for each (var row:String in routeData) {
				var fields:Array = row.split(',');
				if (fields.length == 1) { // blank line or other non-data info
					continue;
				}
				routes.push(new Route(stripWS(fields[ROUTE]),
				                         isNaN(AGENCY) ? stripWS(fields[AGENCY]) : "",
				                         fields[SHORT],
				                         fields[LONG],
				                         isNaN(DESC) ? fields[DESC] : "",
				                         fields[TYPE],
				                         isNaN(URL) ? fields[URL] : "",
				                         isNaN(COLOR) ? fields[URL] : ""));
			}
			return routes;
		}

		/** @return A StopTimeCollection */
		public function parseStopTimes():StopTimeCollection {
			trace("parseStopTimes");
			/* Fields */
			var TRIP:uint = NaN; // trip_id, required
			var ARRIVAL:uint = NaN; // arrival_time, req.
			var DEPARTURE:uint = NaN; // departure_time, req.
			var STOP:uint = NaN; // stop_id, req.
			var STOP_SEQ:uint = NaN; // stop_sequence, req.
			var HEADSIGN:uint = NaN; // stop_headsign, optional
			var PICKUP:uint = NaN; // pickup_type, opt.
			var DROP_OFF:uint = NaN; // drop_off_type, opt.
			var SHAPE_DIST:uint = NaN; // shape_dist_travelled, opt.
			
			var stopTimesFile:File = feedDirectory.resolvePath('stop_times.txt');
			var stopTimesData:Array = readFile(stopTimesFile);
			
			/* Figure out what fields are being used, and what indexes they have */									
			var headerFields:Array = stopTimesData.shift().split(',');
			for (var i:int=0; i < headerFields.length; ++i) {			 
				switch(stripWS(headerFields[i])) {
					case "trip_id": TRIP = i; break;
					case "arrival_time": ARRIVAL = i; break;
					case "departure_time": DEPARTURE = i; break;
					case "stop_id": STOP = i; break;
					case "stop_sequence": STOP_SEQ = i; break;
					case "stop_headsign": HEADSIGN = i; break;
					case "pickup_type": PICKUP = i; break;
					case "drop_off_type": DROP_OFF = i; break;
					case "shape_dist": SHAPE_DIST = i; break;
					default: break; // ignore it
				}
			}
			
			var stCollection:StopTimeCollection = new StopTimeCollection();
			for each (var row:String in stopTimesData) {
				var fields:Array = row.split(',');
				if (fields.length == 1) {  // blank line or other non-data info
					continue;
				}
				stCollection.add(new StopTime(stripWS(fields[TRIP]),
				                            stripWS(fields[ARRIVAL]),
				                            stripWS(fields[DEPARTURE]),
				                            stripWS(fields[STOP]),
				                            fields[STOP_SEQ],
				                            isNaN(HEADSIGN) ? fields[HEADSIGN] : "",
				                            isNaN(PICKUP) ? fields[PICKUP] : "",
				                            isNaN(DROP_OFF) ? fields[DROP_OFF] : "",
				                            isNaN(SHAPE_DIST) ? fields[SHAPE_DIST] : ""));
			}
			
			stCollection.sortAll();

			return stCollection;
		}

		
		/**
		 * Parses calendar.txt to create <code>Service</code> instances.
		 * @return An Object being used as a dictionary. The keys are the unique service_ids and the values are Service instances.
		 */
		public function parseCalendar():Object {
			trace("parseCalendar");
			/* All fields in calendar.txt are required, but that doesn't guarantee ordering of the fields. */
			var SERVICE:uint = NaN;
			var SUN:uint = NaN;
			var MON:uint = NaN;
			var TUE:uint = NaN;
			var WED:uint = NaN;
			var THUR:uint = NaN;
			var FRI:uint = NaN;
			var SAT:uint = NaN;			
			var START:uint = NaN;
			var END:uint = NaN;
			
			var calendarFile:File = feedDirectory.resolvePath('calendar.txt');
			var calendarData:Array = readFile(calendarFile);
			
			/* Figure out what fields are being used, and what indexes they have */									
			var headerFields:Array = calendarData.shift().split(',');
			for (var i:int=0; i < headerFields.length; ++i) {			 
				switch (stripWS(headerFields[i])) {
					case "service_id": SERVICE = i; break;
					case "monday": MON = i; break;
					case "tuesday": TUE = i; break;
					case "wednesday": WED = i; break;
					case "thursday": THUR = i; break;
					case "friday": FRI = i; break;
					case "saturday": SAT = i; break;
					case "start_date": START = i; break;
					case "end_date": END = i; break;
					default: break; // ignore it
				}
			}
			
			var services:Object = new Object();
			for each (var row:String in calendarData) {
				var fields:Array = row.split(',');
				if (fields.length == 1) {
					continue; // blank line or non data line
				}
				var avail:Vector.<Boolean> = new Vector.<Boolean>(7, true);
				avail[Service.MON]  = fields[MON]  == '1' ? true : false;
				avail[Service.TUE]  = fields[TUE]  == '1' ? true : false;
				avail[Service.WED]  = fields[WED]  == '1' ? true : false;
				avail[Service.THUR] = fields[THUR] == '1' ? true : false;
				avail[Service.FRI]  = fields[FRI]  == '1' ? true : false;
				avail[Service.SAT]  = fields[SAT]  == '1' ? true : false;
				avail[Service.SUN]  = fields[SUN]  == '1' ? true : false;
				
				var startStr:String = stripWS(fields[START]);				
				var endStr:String = stripWS(fields[END]);
				/* Date strings are in YYYYMMDD format. Note that the Date constructor expects months to be zero based (wtf?), thus the -1 offset. */
				var start:Date = new Date(Number(startStr.slice(0, 4)), Number(startStr.slice(4, 6)) - 1, Number(startStr.slice(6,8)));				
				var end:Date = new Date(Number(endStr.slice(0, 4)), Number(endStr.slice(4, 6)) - 1, Number(endStr.slice(6,8)));
				
				var id:String = stripWS(fields[SERVICE]);
				services[id] = new Service(id, avail, start, end, null, null);
			}
			
			return services;
		}

		/** Parses calendar_dates.txt and addes include/exclude data to the Service objects found in services.
		 * If a service_id is found in calendar_dates.txt that is not in the services vector, a new Service will
		 * be added.
		 * @param services An Object used as a dictionary. Contains existing Service instances to be modified, keyed by service_id. May be null.
		 * @return the services object, with the contents modified and/or added to.
		 */ 
		public function parseCalendarDates(services:Object):Object {
			trace("parseCalendarDates");
			/* All fields in calendar_dates.txt are required, but that doesn't guarantee ordering of the fields. */
			var SERVICE:uint = NaN;
			var DATE:uint = NaN;
			var EXCEP_TYPE:uint = NaN;
			
			const INCLUDE:Number = 1;
			const EXCLUDE:Number = 2;
			
			var calDatesFile:File = feedDirectory.resolvePath('calendar_dates.txt');
			var calDatesData:Array = readFile(calDatesFile);

			/* Figure out what fields are being used, and what indexes they have */									
			var headerFields:Array = calDatesData.shift().split(',');
			for (var i:int=0; i < headerFields.length; ++i) {		 
				switch (stripWS(headerFields[i])) {
					case "service_id": SERVICE = i; break;
					case "date": DATE = i; break;
					case "exception_type": EXCEP_TYPE = i; break;
					default: break; // ignore it
				}
			}

			if (services == null) {
				services = new Object();
			}

			for each (var row:String in calDatesData) {
				var fields:Array = row.split(',');
				if (fields.length == 1) {
					continue; // blank line or non data line
				}

				 /* Date strings are in YYYYMMDD format. Note that the Date constructor expects months to be zero based (wtf?), thus the -1 offset. */
				var dateStr:String = stripWS(fields[DATE]);
				var date:Date = new Date(Number(dateStr.slice(0, 4)), Number(dateStr.slice(4, 6)) - 1, Number(dateStr.slice(6,8))); // YYYYMMDD
				 
				var service_id:String = stripWS(fields[SERVICE]);
				var service:Service = services[service_id];
				if (service == null) { // not found, create a new one
					service = new Service(service_id, null, date, date, null, null);
					services[service_id] = service;
				}
				
				if (Number(fields[EXCEP_TYPE]) == INCLUDE) {
					service.incl_exceptions.push(date);
				} else if (Number(fields[EXCEP_TYPE]) == EXCLUDE) {
					service.excl_exceptions.push(date);
				}
			}
			return services;			
		}

		/**
		 * Parses the shapes.txt file and generates TrackSegment and TrackOverlay objects to represent the data.
		 * The shapes.txt file is optional in GTFS.
		 * TODO: Add a progress bar!
		 * @param shapeIds An object used as a set, where each property is a shapeId to parse. If null (the default) then parse all shapes.
  		 * @return An Object used as an associative array. The keys are the latlngs of nodes (in <code>toUrlValue(PRECISION)</code> form).
		 * 		   The values are Vectors containing all trackOverlays that are incident with that latlng.
		 */ 
		public function parseShapes(shapeIds:Object=null):Object {
			trace("parseShapes");
			const ID:uint = 0; // shape_id
			const LAT:uint = 1; // shape_pt_lat
			const LON:uint = 2; // shape_pt_lon
			const SEQ:uint = 3; // shape_pt_sequence
			const DIST:uint = 4; // shape_dist_traveled (optional)	
			
			var shapesFile:File = feedDirectory.resolvePath('shapes.txt');
			var shapesData:Array = readFile(shapesFile);
			
			var shapes:Object = new Object(); // used as a dictionary. Key is the shape id, value is a vector of shapes.
			var vec:Vector.<ShapeNode>;
			var shp:ShapeNode;
			
			// create shape objects and store them in shapes			
			for (var i:int = 1; i < shapesData.length; ++i) { // skip the header line
				var line:String = shapesData[i];	
				var fields:Array = line.split(',');
				if (fields.length == 1) { // blank line or other line lacking any commas.
					continue;
				}
				var id:String = stripWS(fields[ID]);
				if (shapeIds == null || shapeIds[id] != null) {
					var latlng:LatLng = new LatLng(toPrecision(fields[LAT]), toPrecision(fields[LON]));
					shp = new ShapeNode(id, latlng, fields[SEQ]);
								
					// Store the shapes in vectors specific to a particular id
					if (shapes[shp.id] == null) { // if a vector doesn't exist for this shape yet
						vec = new Vector.<ShapeNode>();
						vec.push(shp);
						shapes[shp.id] = vec;
					} else {
						shapes[shp.id].push(shp);
					}
				}
			}
			return shapes;		
		}
		
		/** Truncates a string representation of a number to PRECISION places after the decimal */
		protected function toPrecision(num:String):Number {
			var idx:uint = num.indexOf('.');
			return Number(num.slice(0, idx+PRECISION+1));
		}
		
		/** Removes leading and trailing whitespace from a string. */
		protected function stripWS(str:String):String {
			return str.replace(wsRegExp, "");
		}
		
		public function forceRefresh(evt:TimerEvent):void {
			trace('Timer event');
			evt.updateAfterEvent();
		}
	}
	
	
	
}