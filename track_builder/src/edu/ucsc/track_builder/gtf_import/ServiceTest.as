package edu.ucsc.track_builder.gtf_import
{
	import flexunit.framework.TestCase;

	public class ServiceTest extends TestCase
	{
		public function test_IsAvailable():void {
			var avail:Vector.<Boolean> = Vector.<Boolean>([true, true, true, true, true, true, false]); // excludes Saturday
			var start:Date = new Date(2009, 0, 2); // Jan 2nd, 2009
			var end:Date = new Date(2009, 1, 1); // Feb 1st, 20009
			var service:Service = new Service('id', avail, start, end, null, null);
			
			assertTrue(service.isAvailable(new Date(2009, 0, 2))); // date == start
			assertTrue(service.isAvailable(new Date(2009, 1, 1))); // date == end
			assertTrue(service.isAvailable(new Date(2009, 1, 1, 23, 59, 59, 999))); // last moment of end 
			assertFalse(service.isAvailable(new Date(2009, 0, 1))); // one day before start
			assertFalse(service.isAvailable(new Date(2009, 1, 2))); // one day after end
			assertFalse(service.isAvailable(new Date(2009, 0, 3))); // Jan 3rd, 2009 -- a Saturday
			
			/* Excercise the include/exclude exceptions */  
			var incl:Vector.<Date> = Vector.<Date>([new Date(2009, 0, 3), new Date(2009, 0, 4), new Date(2010, 0, 1)]); // Sat Jan 3rd, Mon Jan 4th, and Fri Jan 1, 2010. 
			var excl:Vector.<Date> = Vector.<Date>([new Date(2009, 0, 2), // start date
			                                        new Date(2009, 1, 1), // end date
			                                        new Date(2009, 0, 5), // Mon Jan 5th
			                                        new Date(2009, 0, 10), //  Sat Jan 10th (redundant)
			                                        new Date(2010, 0, 2)]); // Sat Jan 2nd, 2010 (outside of start-end range)
			service = new Service('id_2', avail, start, end, incl, excl);
			assertTrue(service.isAvailable(new Date(2009, 0, 3))); // inclusion date
			assertTrue(service.isAvailable(new Date(2009, 0, 4))); // redundant inclusion date
			assertTrue(service.isAvailable(new Date(2010, 0, 1))); // inclusion date outside of start-end range
			assertFalse(service.isAvailable(new Date(2009, 0, 2))); // exclusion of start date
			assertFalse(service.isAvailable(new Date(2009, 1, 1))); // exclusion of end date
			assertFalse(service.isAvailable(new Date(2009, 0, 5))); // exclusion
			assertFalse(service.isAvailable(new Date(2009, 0, 10))); // redundant exclusion
			assertFalse(service.isAvailable(new Date(2010, 0, 2))); // exclusion outside of range
			
			/* Check that all objects were copied to Service, rather than stored in Service */
			for (var i:uint = 0; i < avail.length; ++i) {
				avail[i] = !avail[i]; // flip bits
			}
			for (i=0; i < incl.length; ++i) {
				incl[i] = new Date(2000, 0, 1); // Jan 1st, 2000
			}
			for (i=0; i < excl.length; ++i) {
				excl[i] = new Date(2000, 0, 2); // Jan 2nd, 2000
			}
			
			/* Nothing changed from before */
			assertTrue(service.isAvailable(new Date(2009, 0, 3))); // inclusion date
			assertTrue(service.isAvailable(new Date(2009, 0, 4))); // redundant inclusion date
			assertTrue(service.isAvailable(new Date(2010, 0, 1))); // inclusion date outside of start-end range
			assertFalse(service.isAvailable(new Date(2009, 0, 2))); // exclusion of start date
			assertFalse(service.isAvailable(new Date(2009, 1, 1))); // exclusion of end date
			assertFalse(service.isAvailable(new Date(2009, 0, 5))); // exclusion
			assertFalse(service.isAvailable(new Date(2009, 0, 10))); // redundant exclusion
			assertFalse(service.isAvailable(new Date(2010, 0, 2))); // exclusion outside of range
			assertFalse(service.isAvailable(new Date(2000, 0, 1))); // new inclusion date has no effect
		}		
	}
}