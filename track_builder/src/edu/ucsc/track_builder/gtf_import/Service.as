package edu.ucsc.track_builder.gtf_import
{
	import __AS3__.vec.Vector;
	
	public class Service
	{
		public static const SUN:uint = 0;
		public static const MON:uint = 1;
		public static const TUE:uint = 2;
		public static const WED:uint = 3;
		public static const THUR:uint = 4;
		public static const FRI:uint = 5;
		public static const SAT:uint = 6;		
		
		public var id:String;
		public var avail:Vector.<Boolean>;		
		public var start:Date;
		public var end:Date;
		public var incl_exceptions:Vector.<Date>;
		public var excl_exceptions:Vector.<Date>;
		
		/* Constructor */
		/** 
		 * If any of <code>availability</code>, <code>start</code>, or <code>end</code> are null, they must all be null.
		 * @param service_id A unique string
		 * @param availability Indicates service availability for days of the week. Should be in order indicated by Service.MON, etc. May be null.
		 * @param start Date that the availability starts. Date fields other than <code>year</code>, <code>month</code>, and <code>date</code> are discarded. May be null.
		 * @param end Date that the availability ends. Date fields other than <code>year</code>, <code>month</code>, and <code>date</code> are discarded. May be null.
		 * @param incl_exceptions Dates that are included in service availability, overriding other information. May be null.
		 * @param excl_exceptions Dates that are excluded from service availability, overriding other information. May be null.
		 */
		public function Service(service_id:String,
		                        availability:Vector.<Boolean>,
		                        start:Date,
		                        end:Date,
		                        incl_exceptions:Vector.<Date>,
		                        excl_exceptions:Vector.<Date>)
		{
			this.id = service_id;
			
			/* Copy the availibity vector */		
			if (availability != null) {
				avail = new Vector.<Boolean>(7, true);
				for (var i:uint=0; i < avail.length; ++i) {
					avail[i] = new Boolean(availability[i]);
				}
			} else {
				avail = Vector.<Boolean>([false, false, false, false, false, false, false]);				
			}
			
			/* Copy start and end dates, truncating to just year, month, and date. */
			if (start) {
				this.start = new Date(start.fullYear, start.month, start.date);
			} // else leave this.start as null
			if (end) {
				this.end = new Date(end.fullYear, end.month, end.date);
			} // else leave this.end as null
			
			/* Copy each inclusion date, truncating to just year, month, and date. */ 
			this.incl_exceptions = new Vector.<Date>();
			if (incl_exceptions != null) {
				for each (var incl:Date in incl_exceptions) {
					this.incl_exceptions.push(new Date(incl.fullYear, incl.month, incl.date));
				}
			}
			
			/* Copy each exclusion date, truncating to just year, month, and date. */
			this.excl_exceptions = new Vector.<Date>();
			if (excl_exceptions != null) {
				for each (var excl:Date in excl_exceptions) {
					this.excl_exceptions.push(new Date(excl.fullYear, excl.month, excl.date));
				} 
			}
		}
		
		/** Returns true if service is available on date.
		 * @param date The date to check. Fields other than <code>year</code>, <code>month</code>, and <code>date</code> are discarded.
		 */
		public function isAvailable(date:Date):Boolean {
			/* Truncate the date to just year, month, and date, so that dates can be compared via their time field, including equality checks. */
			var date_:Date = new Date(date.fullYear, date.month, date.date);
			var date_time:Number = date_.time;
			
			var result:Boolean = false;
			if (avail) {
				result = avail[date_.day]; // day ordering for Service class matches that of Date class
			}
			
			// Check that it's in range
			if (start && end) {
				if (date_time < start.time || date_time > end.time) {
					result = false;
				}
			}
				
			// Out of range or wrong day may still be allowed by exception
			if (!result && incl_exceptions) {
				for each (var incl_date:Date in incl_exceptions) {
					if (date_time == incl_date.time) {
						return true; // I'm done!
					}
				}
			}
			
			// or denied...
			if (result && excl_exceptions) {
				for each (var excl_date:Date in excl_exceptions) {
					if (date_time == excl_date.time) {
						return false; // I'm done!
					}
				}
			}
			
			// if no include/exclude exception 
			return result;
		}
	}
}