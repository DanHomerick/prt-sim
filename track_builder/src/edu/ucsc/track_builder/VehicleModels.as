package edu.ucsc.track_builder
{
	import __AS3__.vec.Vector;
	
	import flash.events.EventDispatcher;
	
	/** Container class for VehicleModel objects. */
	public class VehicleModels extends EventDispatcher // allows binding
	{
		[Bindable]
		public var models:Array; // Using an Array instead of a Vector because ArrayCollection doesn't support Vectors.
		
		public function VehicleModels()
		{
			super();
			this.models = new Array();
		}
				
		/** Returns XML containing all vehicle models. */
		public function toPrefsXML():XML {
			var xml:XML = <VehicleModels/>; 		
			for each (var model:VehicleModel in this.models) {
				xml.appendChild(model.toXML());
			}
			return xml;
		}

		/** Returns XML containing only those vehicle models which are used in vehicles. */
		public function toDataXML(vehicles:Vector.<Vehicle>):XML {
			var scenarioModels:Vector.<VehicleModel> = getModelsInUse(vehicles);
			
			// Sort by model name, so that they will be in a consistant order
			scenarioModels.sort(function cmp(x:VehicleModel, y:VehicleModel):Number {
				                             if (x.modelName < y.modelName) {return -1;}
				                             else if (x.modelName == y.modelName) {return 0;}
				                             else {return 1;}
			                             }
			) 
			
			var xml:XML = <VehicleModels/>;
			for each (var model:VehicleModel in scenarioModels) {
				xml.appendChild(model.toXML())
			}
			
			return xml;
		}							
			
		/** Generate xml from hard-coded default preferences */
		public function toDefaultPrefsXML():XML {
			var xml:XML = <VehicleModels>
						  	<VehicleModel model_name="PRT_DEFAULT"
						  		          length="4.0"
						  		          passenger_capacity="3"
						  		          mass="450"
						  		          frontal_area="1.7"
						  		          drag_coefficient="0.2"
						  		          powertrain_efficiency="0.8"
						  		          regenerative_braking_efficiency="0.0">
						  		<Jerk normal_max="5.0" normal_min="-5.0" emergency_max="20" emergency_min="-20" />
						  		<Acceleration normal_max="5.0" normal_min="-5.0" emergency_max="5.0" emergency_min="-25" />
						  		<Velocity normal_max="30.0" normal_min="0.0" emergency_max="45.0" emergency_min="0.0" />
						  	</VehicleModel>		  	
						  	<VehicleModel model_name="BUS_DEFAULT"
						  		          length="14.0"
						  		          passenger_capacity="50"
						  		          mass="9200">
						  		<Jerk normal_max="2.5" normal_min="-2.5" emergency_max="20" emergency_min="-20" />
						  		<Acceleration normal_max="1.0" normal_min="-5.0" emergency_max="1.5" emergency_min="-25" />
						  		<Velocity normal_max="32.0" normal_min="0.0" emergency_max="65.0" emergency_min="0.0" />
						  	</VehicleModel>	
			              </VehicleModels>;	
			return xml;
		}
		
		public function fromPrefsXML(xml:XML):void {
			this.models = new Array();
			for each (var modelXML:XML in xml.VehicleModel) {
				var model:VehicleModel = VehicleModel.fromXML(modelXML);
				this.models.push(model)					
			}
		}
		
		/** Loads vehicle models from the scenario file and adds them to the list of known models.
		 * @param xml An XMLList of 'VehicleModel' entities.
		 * @throw ModelError Thrown when a model name conflicts with a known model and the models
		 * do not contain identical data.
		 */
		public function fromDataXML(xml:XMLList):void {
			for each (var modelXML:XML in xml) {
				var model:VehicleModel = VehicleModel.fromXML(modelXML);
				var knownModel:VehicleModel = Globals.vehicleModels.getModelByName(model.modelName);
				// If the model we're loading duplicates an existing model's name,
				// but the model data is different... 
				if (knownModel != null && !model.equals(knownModel)) { 					
					// Change the model's current name to a new one, if a name mapping has been provided 					
					throw new ModelError(model, "Name conflict");
				} else if (!model.equals(knownModel)) {
					this.models.push(model); // add the new model to the known models
				} // else do nothing.				
			}				
		}
		 
		public function get modelNames():Array {
			var results:Array = [];
			for each (var m:VehicleModel in this.models) {
				results.push(m.modelName);
			}
			return results;
		}
		
		/** Get the VehicleModels that are being used by at least one of the supplied vehicles. */
		public function getModelsInUse(vehicles:Vector.<Vehicle>):Vector.<VehicleModel> {
			// Create a set of the model names used in current scenario.
			var modelNamesSet:Object = new Object();
			for each (var v:Vehicle in vehicles) {
				modelNamesSet[v.modelName] = true;
			}

			// Create a mapping from VehicleModel names to VehicleModel objects
			var namesToModels:Object = new Object();
			for each (var m:VehicleModel in this.models) {
				namesToModels[m.modelName] = m
			}
				
			// Create a vector containing just the VehicleModels that are used in the scenario
			var inUseModels:Vector.<VehicleModel> = new Vector.<VehicleModel>();
			for (var modelName:String in modelNamesSet) {
				inUseModels.push(namesToModels[modelName]);
			}
			
			return inUseModels;
		}
		
		public function getModelByName(name:String):VehicleModel {
			var model:VehicleModel;
			for each (model in this.models) {
				if (model.modelName == name) {
					return model;
				}
			}
			return null;	
		}
		
		public function removeModelByName(name:String):void {
			for (var i:int=0; i < models.length; ++i) {
				if (models[i].modelName == name) {
					models.splice(i, 1); // remove this element
					break;
				}
			}
		}
	}
}