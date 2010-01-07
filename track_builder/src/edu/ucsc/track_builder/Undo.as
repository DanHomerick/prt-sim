package edu.ucsc.track_builder
{	
	/** An all-static class that handles undos. Stores a series of 'micro' function/argument pairs to undo a command. In general, whenever
	 * a function causes a side-effect, it should pushMicro a function + params to undo the effect. Keeps two separate undo stacks, one to
	 * undo user commands, and another for programatic use.
	 */	
	public class Undo
	{
		// keep separate stacks for user commands, and for programmer use
		public static const NONE:uint = 0;
		public static const USER:uint = 1;
		public static const PREVIEW:uint = 2;
	
		private static var stacks:Array = new Array();
		stacks[NONE] = null;
		stacks[USER] = new Array();
		stacks[PREVIEW] = new Array();
		private static var recording:int = NONE;

		/** Call at the beginning of a operation to begin storing an undo procedure.
		 * For user initiated commands, pass Undo.USER as stackId. For commands that will
		 * be undone under programatic control, use Undo.PROG as stackId.
		 */ 
		public static function startCommand(stackId:uint):void {
			if (recording != NONE) {
				throw new Error("Undo is becoming recursive. Bad programmer, bad.");
			}
			recording = stackId;
			stacks[stackId].push(new Array());
		}
		
		public static function endCommand():void {
			if (recording == NONE) {
				throw new Error("Ending an Undo command while recording is already false");
			}
			recording = NONE;
		}

		/** Add the micro undo to the latest command undo.
		 * thisArg: The object to which the function should be applied
		 * fnc: A function that undoes a particular action
		 * args: arguments to fnc
		 */
		public static function pushMicro(thisArg:*, fnc:Function, ... args):void {
			if (recording == NONE) {
				return;
			}
			stacks[recording][stacks[recording].length-1].push(new Micro(thisArg, fnc, args));
		}
		
		/** Undo a command, one micro step at a time */
		public static function undo(stackId:uint):void {
			if (stacks[stackId].length > 0) {
				var cmd:Array = stacks[stackId].pop();
				while (cmd.length > 0) {
					var micro:Micro = cmd.pop();
					micro.fnc.apply(micro.thisArg, micro.args);
				}				
			} // else we haven't done anything that can be undone yet
			Utility.refreshScreen(); // this is important!
		}
		
		public static function assign(thisArg:*, prop:String, value:Object):void {
			if (recording == NONE) {
				return;
			}
			var fnc:Function = function(prop:String, value:Object):void {
				this[prop] = value; // apply will set 'this' to 'thisArg' when the function is called. 
			}
			stacks[recording][stacks[recording].length-1].push(new Micro(thisArg, fnc, [prop, value]));			
		}	
		
		/** Similar to assign, but used when setting an element of a container, such as an Array or Vector. */
		public static function assignElement(thisArg:*, prop:String, idx:uint, value:Object):void {
			if (!recording) {return;}
			var fnc:Function = function(prop:String, idx:uint, value:Object):void {
				this[prop][idx] = value;
			}
			stacks[recording][stacks[recording].length-1].push(new Micro(thisArg, fnc, [prop, idx, value]));
		}
		
		public static function reinitialize():void {
			stacks[NONE] = null;
			stacks[USER] = new Array();
			stacks[PREVIEW] = new Array();
			recording = NONE;
		}
	}
}

/** Just a data holding class. A micro step in the undoing of a USER or PROG command. */
internal class Micro
{
	public var thisArg:*
	public var fnc:Function;
	public var args:Array;
	
	/* Constructor */
	public function Micro(thisArg:*, fnc:Function, args:Array)
	{
		this.thisArg = thisArg;
		this.fnc = fnc;
		this.args = args; 	
	}
}