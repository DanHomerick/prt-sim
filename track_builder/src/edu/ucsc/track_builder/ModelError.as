package edu.ucsc.track_builder
{
	public class ModelError extends Error
	{
		public var model:VehicleModel;
		
		public function ModelError(model:VehicleModel, message:String="", id:int=0)
		{
			super(message, id);
			this.model = model;
		}
		
	}
}