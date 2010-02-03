/** This whole class is just a quickie stub until the abilility to configure different models of
  * vehicles is actually implemented.
  */

package edu.ucsc.track_builder
{	
	public class VehicleModel
	{
		public static const DEFAULT:String = "DEFAULT"
		
		public function VehicleModel()
		{
		}
	
		/** TEMP! */
		public static function toXML():XML {
			var xml:XML = <VehicleModels>
						  	<VehicleModel model={DEFAULT}
						  		          length="2.0"
						  		          passenger_capacity="4"
						  		          mass="450">
						  		<Jerk normal_max="2.5" normal_min="-2.5" emergency_max="20" emergency_min="-20" />
						  		<Acceleration normal_max="5.0" normal_min="-5.0" emergency_max="5.0" emergency_min="-25" />
						  		<Velocity normal_max="60.0" normal_min="0.0" emergency_max="65.0" emergency_min="-5.0" />
						  	</VehicleModel>		  	
						  	<VehicleModel model="BUS"
						  		          length="14.0"
						  		          passenger_capacity="50"
						  		          mass="9200">
						  		<Jerk normal_max="2.5" normal_min="-2.5" emergency_max="20" emergency_min="-20" />
						  		<Acceleration normal_max="0.6" normal_min="-5.0" emergency_max="0.75" emergency_min="-25" />
						  		<Velocity normal_max="32.0" normal_min="0.0" emergency_max="65.0" emergency_min="-5.0" />
						  	</VehicleModel>	
			              </VehicleModels>;												      
			return xml;
		}

	}
}