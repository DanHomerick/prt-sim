package edu.ucsc.track_builder.gtf_import
{
	import __AS3__.vec.Vector;
	
	import flexunit.framework.TestCase;

	public class TimeTest extends TestCase
	{
		public function test_all():void {			
			var t:Time = Time.fromString("1:01:10");
			assertEquals(3670, t.time);
			assertEquals(3670, t.toSeconds());
			assertEquals("01:01:10", t.toString());
			
			t = Time.fromString("01:01:10");
			assertEquals(3670, t.time);
			assertEquals(3670, t.toSeconds());
			assertEquals("01:01:10", t.toString());
			
			t = Time.fromHMS(1, 1, 10);
			assertEquals(3670, t.time);
			assertEquals(3670, t.toSeconds());
			assertEquals("01:01:10", t.toString());
		}
	}
}