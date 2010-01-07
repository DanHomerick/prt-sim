package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	/* Useful site: http://colorbrewer2.org/ */	
	public class ColorMapper
	{
		/** A 6-class Green-Yellow-Red diverging color scheme, from ColorBrewer2.org */
		public static const GnYlRd:String = "GnYlRd";
		private static const GnYlRd_COLORS:Vector.<uint> = Vector.<uint>([0x1A9850, 0x91CF60, 0xD9EF8B, 0xFEE08B, 0xFC8D59, 0xD73027]);
		
		private var colors:Vector.<uint>;
		private var minValue:Number = 0;
		private var maxValue:Number = 1; 
		private var thresholds:Vector.<Number>;
		
		
		public function ColorMapper(minValue:Number, maxValue:Number, scheme:String=GnYlRd)
		{
			thresholds = new Vector.<Number>();
			switch (scheme) {
				case GnYlRd:
					colors = GnYlRd_COLORS;
					break;
				default:
					throw new Error("Unknown color scheme");
					break;		
			}
			
			// Calculate the threshold values at which the colors will change.
			var step:Number = (maxValue - minValue) / colors.length; 
			for (var i:uint = 1; i <= colors.length; ++i) { // exclude minValue. 
				thresholds.push(minValue + step*i);
			}
		}
		
		/** Maps a value to a color according to the scheme chosen. Values which are out of range are given
		 * the same color as the nearest in-range value.
		 */
		public function valueToColor(value:Number):uint {
			var color:Number = colors[colors.length-1];
			for (var i:int = 0; i < thresholds.length; ++i) {			
				if (value < thresholds[i]) {
					color = colors[i];
					break;
				}
			}
			return color;
		}
		
	}
}

0xD73027; 0xFC8D59; 0xFEE08B; 0xD9EF8B; 0x91CF60; 0x1A9850; 