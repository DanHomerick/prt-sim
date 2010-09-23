package edu.ucsc.track_builder
{
	[Bindable]
	public class Weather
	{
		/** Measured in Kelvin */
		public var temperature:Number;
		
		/** Average elevation of the region or track segments, measured in meters above sea level */
		public var elevation:Number;
		
		/** Absolute Pressure measured in kiloPascals. Note that reported barometric pressures are
		 * frequently 'Mean Sea Level Pressure' (MSLP), not absolute pressure. */
		public var pressure:Number; 
		
		/** Measured in kg/m^3 */
		public var airDensity:Number;
		
		/** Measured in meters/second */  
		public var windSpeed:Number;
		
		/** Measured in degrees from North, e.g. a wind blowing from the East has a windDirection of 90. */
		public var windDirection:Number;		
		
		public const P_STD:Number = 101325; // Pascals
		public const T_STD:Number = 288.15; // in Kelvin, equiv to 15 C
		public const g:Number = 9.80665; // m/s^2     gravitational acceleration
		public const R:Number = 8.31447; // J/(mol*K) Universal gas constant
		public const M:Number = 0.0289644; // kg/mol for dry air
		
		public function Weather(temperature:Number=NaN,
		                        elevation:Number=NaN,
		                        pressure:Number=NaN,
		                        airDensity:Number=NaN,
		                        windSpeed:Number=NaN,
		                        windDirection:Number=NaN)
		{
			this.temperature = temperature;
			this.elevation = elevation;
			this.pressure = pressure;
			this.airDensity = airDensity;
			this.windSpeed = windSpeed;
			this.windDirection = windDirection; 
		}

		/** Only updates values which have not already been set. */
		public function setToDefaults():void {
			if (isNaN(this.temperature)) {
				this.temperature = 298.15 // 25 C (77 F)
			}
			if (isNaN(this.elevation)) {
				var ele:Number = Globals.tracks.getAverageElevation();
				if (isNaN(ele)) {
					ele = 1; // No tracks placed. Use near sea level.	
				}
				this.elevation = ele;	
			}
			if (isNaN(this.pressure)) {
				this.pressure = calcAirPressure(this.elevation, this.temperature);
			}
			if (isNaN(this.airDensity)) {
				this.airDensity = calcAirDensity(this.pressure, this.temperature);
			}
			if (isNaN(this.windSpeed)) {
				this.windSpeed = 0;
			}
			if (isNaN(this.windDirection)) {
				this.windDirection = 0;
			}
		}

		public function toDataXML():XML {
			setToDefaults(); // Sets any unset values to default values.
			
			var xml:XML = <Weather
			                 temperature={temperature}
			                 elevation={elevation}
			                 pressure={pressure}
			                 air_density={airDensity}
			                 wind_speed={windSpeed}
			                 wind_direction={windDirection}
			               />
			return xml;
		}

		/** Loads data from the Weather XML entity. Values that are missing from the XML are provided with
		 * reasonable defaults. Depends on TrackSegment info being already loaded if elevation info is missing from XML.
		 */
		public function fromDataXML(xml:XMLList):void {
			 if ('@temperature' in xml) {
			 	this.temperature = Number(xml.@temperature);
			 }
			 
			 if ('@elevation' in xml) {
			 	this.elevation = Number(xml.@elevation);
			 }
			 
			 if ('@pressure' in xml) {
			 	this.pressure = Number(xml.@pressure);
			 }	
			 
			 if ('@air_density' in xml) {
			 	this.airDensity = Number(xml.@air_density);
			 }
			 
			 if ('@wind_speed' in xml) {
			 	this.windSpeed = Number(xml.@wind_speed);
			 }
			 
			 if ('@wind_direction' in xml) {
			 	this.windDirection = Number(xml.@wind_direction);
			 }
			 
			 setToDefaults();
		}
		
		/** Finds the pressure at a given altitude and temperature, measured in Pascals. Assumes the air to be dry 
		 * (moist air is less dense). Note that the air pressure supplied in weather reports has typically
		 * been adjusted to remove the effects of elevation, whereas this pressure is unadjusted.
		 * @see http://en.wikipedia.org/wiki/Density_of_air
		 * @see http://hyperphysics.phy-astr.gsu.edu/hbase/kinetic/barfor.html
		 */
		public function calcAirPressure(altitude:Number, temperature:Number):Number {
			var h:Number = altitude
			var T:Number = temperature
			
			// Modified formula to use measured temperature instead of lapse temperature.
			var pressure:Number;
			if (Math.abs(T - T_STD) > 0.1) {
				var base:Number = T/T_STD;
				var exp:Number = g*M*h/(R*T);
				pressure = P_STD*Math.pow(base, exp) // pressure	
			} else {
				pressure = P_STD;
			}
			return pressure;
		}
		
		/** Finds the density of the air in units of kg/m^3.
		 * @param pressure Absolute air pressure measured in Pascals.
		 * @see http://en.wikipedia.org/wiki/Density_of_air
		 */
		public function calcAirDensity(pressure:Number, temperature:Number):Number {
			return pressure*M/(R*temperature);
		}		
	}
}