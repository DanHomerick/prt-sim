package edu.ucsc.track_builder.gtf_import
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import edu.ucsc.graph.Graph;
	import edu.ucsc.graph.Node;
	import edu.ucsc.neartree.NearTree;
	
	import flash.filesystem.File;
	import flash.filesystem.FileMode;
	import flash.filesystem.FileStream;
	
	import flexunit.framework.TestCase;
	
	import mx.controls.ProgressBar;
	import mx.utils.ObjectUtil;

	public class GtfImporterTest extends TestCase
	{
		private var importer:GtfImporter;
		private var progressBar:ProgressBar;
		private var feedDir:File;
		
		override public function setUp():void {									
			feedDir = File.createTempDirectory();
			progressBar = new ProgressBar();
			importer = new GtfImporter(feedDir);
		}
		
		public function test_addStops():void {
			/* Two stops, one slightly above and one below 1,1 */
			makeGtfFile('stops.txt',
			            'stop_id,stop_name,stop_desc,stop_lat,stop_lon\n' +
			            '1,stop1,,1.0001,1\n' +
			            '2,stop2,,0.9999,1\n');
			var stops:Object = importer.getStops();
			
			/* A straight line, bisecting the two stop locations. */
			var graph:Graph = new Graph();
			var a:Node = new Node(new LatLng(1,0), '1,0');
			var b:Node = new Node(new LatLng(1,2), '1,2'); 
			graph.addNode(a);
			graph.addNode(b);
			graph.addEdge(a, b);
			
			var latlngVec:Vector.<LatLng> = Vector.<LatLng>([a.latlng, b.latlng]);
			var objects:Array = [null, null];
			var tree:NearTree = NearTree.makeTree(latlngVec, objects);
			
			var stopIds:Vector.<String> = Vector.<String>(['1', '2']);
			var test:Object = importer.addStops(stopIds, stops, graph, tree);
			var expected:Object = new Object();
			
			/* Incomplete */
		}
		
		/** Tests the case where both calendar.txt and calendar_dates.txt are present (UNIX line endings). */
		public function test_getServices_I():void {
			makeGtfFile('calendar.txt',
			            'service_id,sunday,monday,tuesday,wednesday,thursday,friday,saturday,start_date,end_date\n' +
			            '10,0,1,1,1,1,0,0,20090924,20091206'); // Mon-Thur
			makeGtfFile('calendar_dates.txt',
			            'service_id,date,exception_type\n' + 
			            '10,20091111,2\n' +
			            '10,20091210,1\n');
			
			var services:Object = importer.getServices();
			var avail:Vector.<Boolean> = Vector.<Boolean>([false,true,true,true,true,false,false]);
			/* Months are zero based, thus the -1 offset.  */
			var start:Date = new Date(2009, 9-1, 24);	
			var end:Date = new Date(2009, 12-1, 06);
			var incl_exceptions:Vector.<Date> = Vector.<Date>([new Date(2009, 12 - 1, 10)]);
			var excl_exceptions:Vector.<Date> = Vector.<Date>([new Date(2009, 11 - 1, 11)]);
			var expectedServices:Object = {'10':new Service('10', avail, start, end, incl_exceptions, excl_exceptions)};

			compareServices(expectedServices, services);
		}
		
		
		/** Tests case where only calendar.txt is present (Windows line endings, blank line at end of file) */
		public function test_getServices_II():void {
			makeGtfFile('calendar.txt',
			            'service_id,sunday,monday,tuesday,wednesday,thursday,friday,saturday,start_date,end_date\r\n' +
			            '10,0,1,1,1,1,0,0,20090924,20091206\r\n' + 
			            '\r\n');
			var services:Object = importer.getServices();
						
			var avail:Vector.<Boolean> = Vector.<Boolean>([false,true,true,true,true,false,false]);
			/* Months are zero based, thus the offset.  */
			var start:Date = new Date(2009, 9-1, 24);	
			var end:Date = new Date(2009, 12-1, 06);
			var expectedServices:Object = {'10':new Service('10', avail, start, end, null, null)}; // end
			
			compareServices(expectedServices, services);			
		}
		
		/** Tests case where only calendar_dates.txt is present (UNIX line endings) */
		public function test_getServices_III():void {
			// todo: implement
		}
		
		public function test_getServicesFromRoutes():void {
			makeGtfFile('trips.txt',
			            'route_id,service_id,trip_id\n' + 
			            'route1,serve1,trip1\n' +
			            'route1,serve2,trip2\n' +
			            'route2,serve3,trip3\n');
			            
			makeGtfFile('calendar.txt',
			            'service_id,sunday,monday,tuesday,wednesday,thursday,friday,saturday,start_date,end_date\n' + 
			            'serve1,0,1,1,1,1,0,0,20090924,20091206\n' +  // Mon-Thur
			            'serve2,0,0,0,0,0,1,0,20090924,20091206' + // Friday
                        'serve3,1,0,0,0,0,0,1,20090924,20091206'); // Sat-Sun
                        
			makeGtfFile('routes.txt',
			            'route_id,route_short_name,route_long_name,route_type\n' +
						'route1,1,Here to There,3\n' +
						'route1,1,Here to There,3\n' +
						'route2,2,There to Yonder,3\n');
						
			var routesVec:Vector.<Route> = importer.getRoutes();
			var routesArray:Array = new Array();
			for each (var route:Route in routesVec) {
				routesArray.push(route);
			}
			routesArray.pop(); // exclude route2
			
			var services:Object = importer.getServicesFromRoutes(routesArray); // expect route2 / serve3 to be excluded
			
			var avail_1:Vector.<Boolean> = Vector.<Boolean>([false,true,true,true,true,false,false]);
			var avail_2:Vector.<Boolean> = Vector.<Boolean>([false,false,false,false,false,true,false]);
			/* Months are zero based, thus the -1 offset.  */
			var start:Date = new Date(2009, 9-1, 24);	
			var end:Date = new Date(2009, 12-1, 06);
			var service_1:Service = new Service('serve1', avail_1, start, end, null, null);
			var service_2:Service = new Service('serve2', avail_2, start, end, null, null);			
			
			var expectedServices:Object = {'serve1':service_1, 'serve2':service_2};
			compareServices(expectedServices, services);
		}		
		
		/** Does various equivalency checks (assertions) on the services contained within a and b. */
		protected function compareServices(a:Object, b:Object):void {
			var info_a:Object = ObjectUtil.getClassInfo(a);
			var info_b:Object = ObjectUtil.getClassInfo(b);
			assertEquals(info_a.properties.length, info_b.properties.length); // contain same number of Services
			
			for each (var expServ:Service in a) {
				var serv:Service = b[expServ.id];
				assertNotNull(serv);
			
				assertEquals(expServ.avail.length, serv.avail.length); // avail
				for (var i:uint = 0; i < expServ.avail.length; ++i) {
					assertEquals(expServ.avail[i], serv.avail[i]);
				}
				
				assertEquals(expServ.start.time, serv.start.time); // start
				assertEquals(expServ.end.time, serv.end.time);     // end
				
				assertEquals(serv.incl_exceptions.length, expServ.incl_exceptions.length); // incl_exceptions
				for (i=0; i<expServ.incl_exceptions.length; ++i) {
					assertEquals(expServ.incl_exceptions[i].time, serv.incl_exceptions[i].time);	
				}
				
				assertEquals(serv.excl_exceptions.length, expServ.excl_exceptions.length); // excl_exceptions
				for (i=0; i<expServ.excl_exceptions.length; ++i) {
					assertEquals(expServ.excl_exceptions[i].time, serv.excl_exceptions[i].time);	
				}
			}
		}  
		
		protected function makeGtfFile(name:String, contents:String):File {
			var file:File = feedDir.resolvePath(name);
			var fs:FileStream = new FileStream();
			fs.open(file, FileMode.WRITE);
			fs.writeUTFBytes(contents);
			fs.close();
			return file;
		}
		
		override public function tearDown():void {
			feedDir.deleteDirectory(true); // delete files within
		}

	}
}