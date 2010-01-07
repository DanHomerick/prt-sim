package com.mariosh.components
{
	import com.mariosh.components.GradientEditorClasses.GradientBox;
	import com.mariosh.components.GradientEditorClasses.GradientSlider;
	
	import flash.events.Event;
	import flash.events.MouseEvent;
	
	import mx.core.UIComponent;

	[Event(name="selectedColorChange", type="flash.events.Event")]
	[Event(name="selectedAlphaChange", type="flash.events.Event")]
	[Event(name="thumbChange", type="flash.events.Event")]	
	public class GradientEditor extends UIComponent
	{
		private var _gradientBox:GradientBox
		private var _gradientSlider:GradientSlider;
		
		public function GradientEditor()
		{
			super();
		}
		
		//-------------------------------
		// Colors
		//-------------------------------
		private var _colors:Array = [0x000000, 0xffffff];
		private var colorsChanged:Boolean = false;
		public function set colors(value:Array):void
		{
			if(_colors != value)
			{
				_colors = value;
				colorsChanged = true;
				invalidateProperties();
				dispatchEvent(new Event("colorsChange"));
			}
		}
		
		[Bindable(event="colorsChange")]
		public function get colors():Array
		{
			return _colors;
		}
		//----------------------------
		// Alphas
		//----------------------------
		private var _alphas:Array = [1,1];
		private var alphasChanged:Boolean = false;
		public function set alphas(value:Array):void
		{
			if(_alphas != value)
			{
				_alphas = value;
				alphasChanged = true;
				invalidateProperties();
				dispatchEvent(new Event("alphasChange"));
			}
		}
		
		[Bindable(event="alphasChange")]
		public function get alphas():Array
		{
			return _alphas;
		}
		//---------------------------
		// Ratios
		//---------------------------
		private var _ratios:Array = [0, 255];
		public function set ratios(value:Array):void
		{
			if(_ratios != value)
			{
				_ratios = value;
				invalidateProperties();
				dispatchEvent(new Event("ratiosChange"));
			}
		}
		
		[Bindable(event="ratiosChange")]
		public function get ratios():Array
		{
			return _ratios;
		}
		//---------------------------
		// Gradient Rotation
		//---------------------------
		private var _gradientRotation:Number;
		public function set gradientRotation(value:Number):void
		{
			_gradientRotation = value;
		}
		
		public function get gradientRotation():Number
		{
			return _gradientRotation;
		}
		//---------------------------
		// Enabled
		//---------------------------
		private var _enabled:Boolean;
		private var enableChanged:Boolean = false;
		override public function set enabled(value:Boolean):void
		{
			super.enabled = value;
			if(_enabled != value)
			{
				_enabled = value;
				enableChanged = true;
				invalidateProperties();
			}
			
		}
		//-------------------------
		// Selected Color
		//-------------------------
		private var _selectedColor:uint;
		private var selectedColorChanged:Boolean = false;
		public function set selectedColor(value:uint):void
		{
			if(_selectedColor != value)
			{
				_selectedColor = value;
				selectedColorChanged = true;
				invalidateProperties();
				dispatchEvent(new Event("selectedColorChange"));
			}
		}
		
		[Bindable(event="selectedColorChange")]
		public function get selectedColor():uint
		{
			return _selectedColor;
		}
		//--------------------------------
		// Selected Alpha
		//--------------------------------
		private var _selectedAlpha:Number = 1;
		private var selectedAlphaChanged:Boolean = false;
		public function set selectedAlpha(value:Number):void
		{
			if(_selectedAlpha != value)
			{
				_selectedAlpha = value;
				selectedAlphaChanged = true;
				invalidateProperties();
				dispatchEvent(new Event("selectedAlphaChange"));
			}
		}
		
		[Bindable(event="selectedAlphaChange")]
		public function get selectedAlpha():Number
		{
			return _selectedAlpha;
		}
		//----------------------------
		private var _selectedRatio:Number;
		//----------------------------
		private var ratiosChanged:Boolean = false;
		private function onRatiosChange(event:Event):void
		{
			_selectedRatio = _ratios[_gradientSlider.selectedIndex];
			
			for(var i:int=0; i<_ratios.length; i++)
			{
				for(var j:int=i; j<_ratios.length; j++)
				{
					if(_ratios[i] > _ratios[j])
					{
						var ratio:Number = _ratios[j];
						var color:uint = _colors[j];
						var alpha:Number = _alphas[j];
						_ratios.splice(j, 1);
						_colors.splice(j, 1);
						_alphas.splice(j, 1);
						_ratios.splice(i, 0, ratio);
						_colors.splice(i, 0, color);
						_alphas.splice(i, 0, alpha);
					}
				}
			}
			
			//update selected index
			for(i=0; i<_ratios.length; i++)
			{
				if(_selectedRatio == _ratios[i])
					_gradientSlider.selectedIndex = i;
			}
			
			ratiosChanged = true;
			
			invalidateProperties(); 
		}
		
		private function addThumb(e:MouseEvent):void
		{
			var defaultColor:uint = 0x000000;
			var sp:Number = (e.localX/_gradientBox.width)*255;
			for(var i:int=0; i<_ratios.length;i++)
			{
				if(_ratios[0] > sp)
				{
					_colors.unshift(defaultColor);
					_alphas.unshift(1);
					_ratios.unshift(sp);
					_gradientSlider.selectedIndex = i;
					break;
				}
					
				if(_ratios[0] < sp && _ratios[_ratios.length-1] > sp)
				{
					if(_ratios[i] < sp && _ratios[i+1] > sp)
					{
						_colors.splice(i+1, 0, defaultColor);
						_alphas.splice(i+1, 0, 1);
						_ratios.splice(i+1, 0, sp);
						_gradientSlider.selectedIndex = i+1;
						break;
					}	
				}
				if(_ratios[_ratios.length-1] < sp)
				{
					_colors.push(defaultColor);
					_alphas.push(1);
					_ratios.push(sp);
					_gradientSlider.selectedIndex = _ratios.length-1;
					break;
				}
			}
			
			_gradientSlider.colors = _colors;
			_gradientSlider.ratios = _ratios;
			
			_gradientBox.colors = _colors;
			_gradientBox.alphas = _alphas;
			_gradientBox.ratios = _ratios;
		}
		
		private function onThumbChange(event:Event):void
		{
			_selectedColor = _colors[_gradientSlider.selectedIndex];
			_selectedAlpha = _alphas[_gradientSlider.selectedIndex];
			dispatchEvent(new Event("selectedColorChange"));
			dispatchEvent(new Event("selectedAlphaChange"));
			dispatchEvent(event); // "thumbChange"
		}
		
		private function onThumbRemove(event:Event):void
		{
			_colors.splice(_gradientSlider.selectedIndex, 1);
			_alphas.splice(_gradientSlider.selectedIndex, 1);
			_ratios.splice(_gradientSlider.selectedIndex, 1);
			_gradientSlider.removeThumb();
			invalidateProperties();
		}
		
		//----------------------------
		// Override methods
		//----------------------------
		override protected function createChildren():void
		{
			super.createChildren();
			
			if(!_gradientBox)
			{
				_gradientBox = new GradientBox();
				_gradientBox.percentWidth = 100;
				_gradientBox.y = 0;
				_gradientBox.addEventListener(MouseEvent.CLICK, addThumb);
				addChild(_gradientBox);
			}
			
			if(!_gradientSlider)
			{
				_gradientSlider = new GradientSlider();
				_gradientSlider.percentWidth = 100;
				_gradientSlider.addEventListener("thumbChange", onThumbChange);
				_gradientSlider.addEventListener("ratiosChange", onRatiosChange);
				_gradientSlider.addEventListener("thumbRemove", onThumbRemove);
				addChild(_gradientSlider);
			}
		}
		
		override protected function commitProperties():void
		{
			super.commitProperties();
			
			if(colorsChanged)
			{
				selectedColor = _colors[_gradientSlider.selectedIndex];
				colorsChanged = false;
			}
			
			if(alphasChanged)
			{
				selectedAlpha = _alphas[_gradientSlider.selectedIndex];
				alphasChanged = false;
			}
			
			if(selectedColorChanged)
			{
				_colors[_gradientSlider.selectedIndex] = _selectedColor;
				selectedColorChanged = false;
			}
			
			if(enableChanged)
			{
				_gradientBox.enabled = _enabled;
				_gradientSlider.enabled = _enabled;
				enableChanged = false;
			}
			
			if(selectedAlphaChanged)
			{
				_alphas[_gradientSlider.selectedIndex] = _selectedAlpha;
				selectedAlphaChanged = false;
			}
			
			_gradientBox.colors = _colors;
			_gradientBox.alphas = _alphas;
			_gradientBox.ratios = _ratios;
			
			_gradientSlider.colors = _colors;
			_gradientSlider.ratios = _ratios;
		}
		
		override protected function updateDisplayList(uW:Number, uH:Number):void
		{
			super.updateDisplayList(uW, uH);
	
			_gradientBox.setActualSize(uW, uH-15);
			_gradientSlider.setActualSize(uW, 14);
			_gradientSlider.move(0, uH-14);
		}
		
	}
}