package com.mariosh.components.GradientEditorClasses
{
	import flash.display.Graphics;
	
	import mx.core.UIComponent;

	public class GradientThumb extends UIComponent
	{
		public function GradientThumb()
		{
			super();
		}
		
		private var _color:uint = 0xff0000;
		public function set color(value:uint):void
		{
			if(_color != value)
			{
				_color = value;	
				invalidateDisplayList();
			}
		}
		
		public function get color():uint
		{
			return _color;
		}
		
		private var _selected:Boolean;
		public function set selected(value:Boolean):void
		{
			if(_selected != value)
			{
				_selected = value;
				invalidateDisplayList();
			}
		}
		
		public function get selected():Boolean
		{
			return _selected;
		}
		
		// 0 - 255
		private var _value:Number;
		public function set value(value:Number):void
		{
			_value = value;
		}
		
		public function get value():Number
		{
			return _value;
		}
		
		override protected function measure():void
		{
			super.measure();
		}
		
		override protected function updateDisplayList(uW:Number, uH:Number):void
		{
			super.updateDisplayList(uW, uH);
			
			var g:Graphics = graphics;
			g.clear();
			g.lineStyle(1);
			
			if(_selected)
				g.beginFill(0x000000);
			else
				g.beginFill(0xcccccc);
			
			g.moveTo(0, 4);
			g.lineTo(4, 0);
			g.lineTo(8, 4);
			g.lineTo(0, 4);
			g.endFill();
			
			g.beginFill(0xffffff);
			g.drawRect(0, 4, 8, 8);
			g.endFill();
			
			g.lineStyle(1, _color, 1);
			g.beginFill(_color);
			g.drawRect(2, 6, 4, 4);
			g.endFill();
		}
		
	}
}