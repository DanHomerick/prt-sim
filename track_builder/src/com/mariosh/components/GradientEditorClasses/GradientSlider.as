package com.mariosh.components.GradientEditorClasses
{
	import flash.events.Event;
	import flash.events.MouseEvent;
	
	import mx.core.UIComponent;

	[Event(name="ratiosChange", type="flash.events.Event")]
	[Event(name="thumbChange", type="flash.events.Event")]
	[Event(name="thumbRemove", type="flash.events.Event")]
	public class GradientSlider extends UIComponent
	{
		private var _thumb:GradientThumb;
		
		private var _xOffset:Number;
		private var _xMin:Number = 0;
		private var _xMax:Number;
		
		public function GradientSlider()
		{
			super();
		}
		
		//-------------------------
        // Colors
        //-------------------------
        private var _colors:Array;
        private var colorsChanged:Boolean = false;
        public function set colors(value:Array):void
        {
        	_colors = value;
        	colorsChanged = true;
        	invalidateProperties();	
        }
        
        public function get colors():Array
        {
        	return _colors;
        }
        //--------------------------
		// Ratios
		//--------------------------
		private var _ratios:Array;
        private var ratiosChanged:Boolean = false;
        public function set ratios(value:Array):void
        {
        	_ratios = value;
        	ratiosChanged = true;
        	invalidateProperties();
        }
        
        public function get ratios():Array
        {
        	return _ratios;
        }
        //--------------------------
		// Selected Index
		//--------------------------
		private var _selectedIndex:int = 0;
		private var selectedIndexChanged:Boolean = false;
		public function set selectedIndex(value:int):void
		{
			if(_selectedIndex != value)
			{
				_selectedIndex = value;
				selectedIndexChanged = true;
				invalidateProperties();
			}
		}
		
		public function get selectedIndex():int
		{
			return _selectedIndex;
		}
		//--------------------------
		private function onThumbDown(event:MouseEvent):void
		{
			stage.addEventListener(MouseEvent.MOUSE_MOVE, onThumbMove);
			stage.addEventListener(MouseEvent.MOUSE_UP, onThumbUp);
			
			_thumb = event.target as GradientThumb;
			_thumb.selected = true;
			//setChildIndex(_thumb, numChildren-1);
			
			var thumb:GradientThumb;
			for(var i:int=0; i<numChildren; i++)
			{
				thumb = UIComponent(getChildAt(i)) as GradientThumb;
				if(thumb == _thumb)
					_selectedIndex = i;
				else
					thumb.selected = false;
			}
			
			_xOffset = this.mouseX - _thumb.x;
			
			dispatchEvent(new Event("thumbChange"));
		}
		
		private function onThumbUp(event:MouseEvent):void
		{
			stage.removeEventListener(MouseEvent.MOUSE_MOVE, onThumbMove);
			stage.removeEventListener(MouseEvent.MOUSE_UP, onThumbUp);
			
			dispatchEvent(new Event("ratiosChange"));
			
			// remove thumb if is invisible
			if(!_thumb.visible)
				dispatchEvent(new Event("thumbRemove"));
		}
		
		private function onThumbMove(event:MouseEvent):void
		{
			 _thumb.x = this.mouseX - _xOffset;
			if(_thumb.x <= _xMin)
				_thumb.x = _xMin;
			if(_thumb.x >= _xMax)
				_thumb.x = _xMax;
			event.updateAfterEvent(); 
			
			_ratios[getChildIndex(_thumb)] = Math.round((_thumb.x / _xMax) * 255);
			
			if(_ratios.length < 3)
				return;
				
			if(this.mouseY > this.height+6)
			{
				_thumb.visible = false;
			}
			else
				_thumb.visible = true;
		}
		
		public function removeThumb():void
		{
			removeChildAt(_selectedIndex);
			_selectedIndex = 0;
			dispatchEvent(new Event("thumbChange"));
		}
		
		//------------------------------------
		// Override methods
		//------------------------------------
		override protected function commitProperties():void
		{
			super.commitProperties();
		
			if(colorsChanged)
			{
				if(_colors.length != numChildren)
				{
					//remove all thumbs
					var num:Number = numChildren;
					for(var i:int=0; i<num; i++)
					{
						removeChildAt(0);
					}
					
					//add new thumbs
					for(i=0; i<_colors.length; i++)
					{
						_thumb = new GradientThumb();
						_thumb.color = _colors[i];
						if(_selectedIndex == i)
							_thumb.selected = true;
						_thumb.addEventListener(MouseEvent.MOUSE_DOWN, onThumbDown);
						addChild(_thumb);
					} 
				}
				else
				{
					//update colors thumbs
					for(i=0; i<_colors.length; i++)
					{
						_thumb = UIComponent(getChildAt(i)) as GradientThumb;
						_thumb.color = _colors[i];
						
						if(_selectedIndex == i)
							_thumb.selected = true;
						else
							_thumb.selected = false;
					}
				}
				colorsChanged = false;
			}
			
			invalidateDisplayList();	
		}
		
		override protected function updateDisplayList(uW:Number, uH:Number):void
		{
			super.updateDisplayList(uW, uH);
			
			_xMax = uW - 9;
			
			var obj:UIComponent;
			
			for(var i:int=0; i<numChildren; i++)
			{
				obj = UIComponent(getChildAt(i));
				var thumb:GradientThumb = obj as GradientThumb;
				thumb.move((_ratios[i]/255)*_xMax, 0);
			}
		}
		
	}
}