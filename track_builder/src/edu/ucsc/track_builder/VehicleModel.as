package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
		
	public class VehicleModel
	{
		public static const DEFAULT:String = "DEFAULT"
		
		public var model:String;
		public var length:Number;
		public var passenger_capacity:int;		
		public var mass:Number;
		public var jerk_max_norm:Number;
		public var jerk_min_norm:Number;
		public var jerk_max_emerg:Number;
		public var jerk_min_emerg:Number;
		public var accel_max_norm:Number;
		public var accel_min_norm:Number;
		public var accel_max_emerg:Number;
		public var accel_min_emerg:Number;
		public var vel_max_norm:Number;
		public var vel_min_norm:Number;
		public var vel_max_emerg:Number;
		public var vel_min_emerg:Number;		
		
		public function VehicleModel(model:String, length:Number, passenger_capacity:int, mass:Number,
		                             jerk_max_norm:Number, jerk_min_norm:Number, jerk_max_emerg:Number, jerk_min_emerg:Number,
		                             accel_max_norm:Number, accel_min_norm:Number, accel_max_emerg:Number, accel_min_emerg:Number,
		                             vel_max_norm:Number, vel_min_norm:Number, vel_max_emerg:Number, vel_min_emerg:Number)
		{
			this.model = model;
			this.length = length;
			this.passenger_capacity = passenger_capacity;			
			this.mass = mass;
			this.jerk_max_norm = jerk_max_norm;
			this.jerk_min_norm = jerk_min_norm;
			this.jerk_max_emerg = jerk_max_emerg;
			this.jerk_min_emerg = jerk_min_emerg;
			this.accel_max_norm = accel_max_norm;
			this.accel_min_norm = accel_min_norm;
			this.accel_max_emerg = accel_max_emerg;
			this.accel_min_emerg = accel_min_emerg;
			this.vel_max_norm = vel_max_norm;
			this.vel_min_norm = vel_min_norm;
			this.vel_max_emerg = vel_max_emerg;
			this.vel_min_emerg = vel_min_emerg;		
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
						  		<Acceleration normal_max="1.0" normal_min="-5.0" emergency_max="1.5" emergency_min="-25" />
						  		<Velocity normal_max="32.0" normal_min="0.0" emergency_max="65.0" emergency_min="-5.0" />
						  	</VehicleModel>	
			              </VehicleModels>;												      
			return xml;
		}

		public static function fromXML(xml:XML):VehicleModel {
			return new VehicleModel(
					xml.@model,
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
					Number(xml.Velocity.@emergency_min));
		}
	}
}