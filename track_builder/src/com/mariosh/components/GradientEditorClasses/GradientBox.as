package com.mariosh.components.GradientEditorClasses
{
	import flash.display.GradientType;
	import flash.display.Graphics;
	import flash.events.MouseEvent;
	import flash.geom.Matrix;
	
	import mx.containers.Box;
	import mx.utils.GraphicsUtil;

	public class GradientBox extends Box
    {
        public function GradientBox()
        {
            super();
        }
        
        //-------------------------
        // Colors
        //-------------------------
        private var _colors:Array;
        public function set colors(value:Array):void
        {
        	_colors = value;
        	invalidateDisplayList();	
        }
        
        public function get colors():Array
        {
        	return _colors;
        }
        //--------------------------
        // Alphas
        //--------------------------
        private var _alphas:Array;
        public function set alphas(value:Array):void
        {
        	_alphas = value;
        	invalidateDisplayList();
        }
        
        public function get alphas():Array
        {
        	return _alphas;
        }
        //--------------------------
		// Ratios
		//--------------------------
		private var _ratios:Array = [0, 255];
        public function set ratios(value:Array):void
        {
        	_ratios = value;
        	invalidateDisplayList();
        }
        
        public function get ratios():Array
        {
        	return _ratios;
        }
        //--------------------------
        // Enabled
        //--------------------------
        private var _enabled:Boolean;
       	override public function set enabled(value:Boolean):void
       	{
       		super.enabled = value;
       		
       		if(_enabled != value)
       		{
       			_enabled = value;
       			invalidateDisplayList();
       		}
       	}
        //--------------------------
        private function handleRollOver(e:MouseEvent):void
        {
        	
        }
        
        private function handleRollOut(e:MouseEvent):void
        {
        	
        }

        //----------------------------------------------
        // override methods
        //----------------------------------------------
        
        override protected function createChildren():void
        {
        	super.createChildren();
        	addEventListener(MouseEvent.ROLL_OVER, handleRollOver);
        	addEventListener(MouseEvent.ROLL_OUT, handleRollOut);
        }

        override protected function updateDisplayList(uW:Number, uH:Number):void
        {
            super.updateDisplayList(uW, uH);

            var g:Graphics = graphics;
            var m:Matrix = horizontalGradientMatrix(0, 0, uW, uH);

            g.clear();
            
            if(_enabled)
            {
            	//draw Grid
            	g.beginFill(0xffffff);
				g.drawRect(0, 0, uW, uH);
				g.endFill();
				g.lineStyle(1, 0xcccccc);
				for(var i:int=1; i<uW/10; i++)
				{
					g.moveTo(i*10, 0);
					g.lineTo(i*10, uH);
				}
				for(i=1; i< uH/10; i++)
				{
					g.moveTo(0, 10*i);
					g.lineTo(uW, i*10);
				}
				
				if(_colors == null)
					return;
				if(_alphas == null)
					return;
				if(_ratios == null)
					return;
					
				g.beginGradientFill(GradientType.LINEAR,
										_colors,
										_alphas,
										_ratios,
										m);
				
	            GraphicsUtil.drawRoundRectComplex(g, 0, 0, uW, uH, 0, 0, 0, 0);
	            g.endFill();
            }
        }
    }
}