package edu.ucsc.track_builder
{
	import mx.utils.ObjectUtil;
	
	[Bindable]
	public class VehicleModel
	{		
		public var modelName:String;
		public var length:Number;
		public var passengerCapacity:int;		
		public var mass:Number;
		public var jerkMaxNorm:Number;
		public var jerkMinNorm:Number;
		public var jerkMaxEmerg:Number;
		public var jerkMinEmerg:Number;
		public var accelMaxNorm:Number;
		public var accelMinNorm:Number;
		public var accelMaxEmerg:Number;
		public var accelMinEmerg:Number;
		public var velMaxNorm:Number;
		public var velMinNorm:Number;
		public var velMaxEmerg:Number;
		public var velMinEmerg:Number;
		public var frontalArea:Number;
		public var dragCoefficient:Number;
		public var powertrainEfficiency:Number;
		public var regenerativeBrakingEfficiency:Number;
		
		public function VehicleModel(modelName:String, length:Number, passengerCapacity:int, mass:Number,
		                             jerkMaxNorm:Number, jerkMinNorm:Number, jerkMaxEmerg:Number, jerkMinEmerg:Number,
		                             accelMaxNorm:Number, accelMinNorm:Number, accelMaxEmerg:Number, accelMinEmerg:Number,
		                             velMaxNorm:Number, velMinNorm:Number, velMaxEmerg:Number, velMinEmerg:Number,
		                             frontalArea:Number, dragCoefficient:Number, powertrainEfficiency:Number,
		                             regenerativeBrakingEfficiency:Number)
		{
			this.modelName = modelName;
			this.length = length;
			this.passengerCapacity = passengerCapacity;
			this.mass = mass;
			this.jerkMaxNorm = jerkMaxNorm;
			this.jerkMinNorm = jerkMinNorm;
			this.jerkMaxEmerg = jerkMaxEmerg;
			this.jerkMinEmerg = jerkMinEmerg;
			this.accelMaxNorm = accelMaxNorm;
			this.accelMinNorm = accelMinNorm;
			this.accelMaxEmerg = accelMaxEmerg;
			this.accelMinEmerg = accelMinEmerg;
			this.velMaxNorm = velMaxNorm;
			this.velMinNorm = velMinNorm;
			this.velMaxEmerg = velMaxEmerg;
			this.velMinEmerg = velMinEmerg;
			this.frontalArea = frontalArea;
			this.dragCoefficient = dragCoefficient;
			this.powertrainEfficiency = powertrainEfficiency;
			this.regenerativeBrakingEfficiency = regenerativeBrakingEfficiency; 
		}
	
		public function toXML():XML {
			var xml:XML = <VehicleModel
							model_name={modelName}
							length={length}
							passenger_capacity={passengerCapacity}
							mass={mass}
							frontal_area={frontalArea}
							drag_coefficient={dragCoefficient}
							powertrain_efficiency={powertrainEfficiency}
							regenerative_braking_efficiency={regenerativeBrakingEfficiency}
							>
					  		<Jerk
					  			normal_max={jerkMaxNorm}
					  		    normal_min={jerkMinNorm}
					  		    emergency_max={jerkMaxEmerg}
					  		    emergency_min={jerkMinEmerg} />
					  		<Acceleration
					  			normal_max={accelMaxNorm}
					  		    normal_min={accelMinNorm}
					  		    emergency_max={accelMaxEmerg}
					  		    emergency_min={accelMinEmerg} />
					  		<Velocity
					  			normal_max={velMaxNorm}
					  		    normal_min={velMinNorm}
					  		    emergency_max={velMaxEmerg}
					  		    emergency_min={velMinEmerg} />
						  </VehicleModel>		  	
			return xml;
		}

		public static function fromXML(xml:XML):VehicleModel {
			if (!('@model_name' in xml)) {
				throw new Error('A VehicleModel in the file has no "model_name" attribute.');
			}
			return new VehicleModel(
					xml.@model_name,
					Number(xml.@length),
					int(xml.@passenger_capacity),
					Number(xml.@mass),
					Number(xml.Jerk.@normal_max),
					Number(xml.Jerk.@normal_min),
					Number(xml.Jerk.@emergency_max),
					Number(xml.Jerk.@emergency_min),
					Number(xml.Acceleration.@normal_max),
					Number(xml.Acceleration.@normal_min),
					Number(xml.Acceleration.@emergency_max),
					Number(xml.Acceleration.@emergency_min),
					Number(xml.Velocity.@normal_max),
					Number(xml.Velocity.@normal_min),
					Number(xml.Velocity.@emergency_max),
					Number(xml.Velocity.@emergency_min),
					Number(xml.@frontal_area),
					Number(xml.@drag_coefficient),
					Number(xml.@powertrain_efficiency),
					Number(xml.@regenerative_braking_efficiency));
		}
				
		public function equals(other:VehicleModel):Boolean {
			return ObjectUtil.compare(this, other) == 0;
		}		
	}
}