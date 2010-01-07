package edu.ucsc.track_builder.gtf_import
{
	/** For GTF data, the main concerns are holding a day or date, and comparing times regardless of what date they're on.
	 * Additionally, GTF uses a time convention that allows hours to go above 23, allowing for times that are past midnight
	 * to be treated as part of the previous service day. The standard Date class isn't a particularly good fit for this
	 * usage, thus this custom class.
	 * 
	 * Comparisons between instances of this class should be done on the toString() values (or, equivalently, the .time property). 
	 */
	public class Time
	{
		public var time:Number;
		
		public function Time()
		{
		}
		
		/** Standardizes to a HH:MM:SS format.
		 * @param str A string in either HH:MM:SS or H:MM:SS format.
		 * @return A new Time instance.
		 */
		public static function fromString(str:String):Time {
			/* GTF allows for the leading 0 to be ommitted */
			var t:Time = new Time();
			var hour:uint;
			var min:uint;
			var sec:uint;
			try {
				if (str.length == 8) {
					hour = uint(str.slice(0,2));
					min = uint(str.slice(3,5));
					sec = uint(str.slice(6,8));
				} else if (str.length == 7) {
					hour = uint(str.slice(0,1));
					min = uint(str.slice(2,4));
					sec = uint(str.slice(5,7));
				} else {
					throw new Error();
				}
			} catch (err:Error) {
				throw new Error("Badly formatted time string: " + str);
			}
			
			t.time = hour*3600 + min*60 + sec;
			return t;
		}
		
		public static function fromHMS(hour:uint, min:uint, sec:uint):Time {
			var t:Time = new Time();
			t.time = hour*3600 + min*60 + sec;
			return t;
		}
		
		public function toSeconds():Number {
			return time;
		}
		
		/** Time in HH:MM:SS format. */
		public function toString():String {
			var hour:uint = time/3600;
			var min:uint = (time%3600)/60;
			var sec:uint = time%60;
			var result:String = (hour < 10 ? '0' + hour.toString() : hour.toString()) + ':' +
			                    (min  < 10 ? '0' + min.toString()  : min.toString())  + ':' +
			                    (sec  < 10 ? '0' + sec.toString()  : sec.toString()); 
			return result;
		}
	}
}