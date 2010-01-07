package edu.ucsc.neartree
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	/** See: "A Template for the Nearest Neighbor Problem" by Larry Andrews
	 * http://www.ddj.com/cpp/184401449?pgno=1
	 */
	public class NearTree
	{
		protected var leftObjs:Array;
		protected var leftLatLng:LatLng;
		protected var leftBranch:NearTree;
		protected var maxLeft:Number;
		
		protected var rightObjs:Array;
		protected var rightLatLng:LatLng;		
		protected var rightBranch:NearTree;		
		protected var maxRight:Number;
		
		/** Threshold, in meters, below which two LatLngs are bundled together. Upon inserting an object
		 * whose LatLng is within <code>MERGE_THRESHOLD</code> meters of an existing LatLng in the tree,
		 * then the object's latlng is discarded, and the object is treated as though the LatLngs
		 * had been identical.
		 */
		public static var MERGE_THRESHOLD:Number = 3; // meters
		
		public function NearTree()
		{
			leftObjs = new Array();
			rightObjs = new Array();
			maxLeft  = -1;
			maxRight = -1;
		}
		
		public function insert(latlng:LatLng, obj:Object):void {
			var leftDist:Number;
			var rightDist:Number;

			if (!leftLatLng) {
				leftLatLng = latlng;
				leftObjs.push(obj);
				return;
			} else {
				leftDist = latlng.distanceFrom(leftLatLng);
				if (leftDist < MERGE_THRESHOLD) {
					leftObjs.push(obj);
					return;
				}
			}
			
			if (!rightLatLng) {
				rightLatLng = latlng;
				rightObjs.push(obj);
				return;
			} else {
				rightDist = latlng.distanceFrom(rightLatLng);
				if (rightDist < MERGE_THRESHOLD) {
					rightObjs.push(obj);
					return;
				}
			}
			
			if (leftDist > rightDist) {
				if (rightBranch == null) {
					rightBranch = new NearTree();
				}
				if (maxRight < rightDist) {
					maxRight = rightDist;
				}
				rightBranch.insert(latlng, obj);				
			}				
			else {
				if (leftBranch == null) {
					leftBranch = new NearTree();
				}
				if (maxLeft < leftDist) {
					maxLeft = leftDist
				}
				leftBranch.insert(latlng, obj);
			}
		}
		
		public function withinRadius(probe:LatLng, radius:Number):Vector.<Result> {
			var vec:Vector.<Result> = new Vector.<Result>();
			_withinRadius(probe, radius, vec);
			return vec;
		}
		
		/** Finds all objects within <code>radius</code> distance of <code>probe</code> and returns the objects in an array.
		 * @param probe The point of interest.
		 * @param radius The max distance, in meters.
		 * @param vec The result objects found. Is modified by the function.		 
		 */
		protected function _withinRadius(probe:LatLng, radius:Number, vec:Vector.<Result>):void {		
			var leftProbeDist:Number;
			var rightProbeDist:Number;
			var result:Result;

			if (leftLatLng) {
				leftProbeDist = probe.distanceFrom(leftLatLng);
				if (leftProbeDist <= radius) {
					result = new Result();
					result.latlng = leftLatLng;
					result.objs = leftObjs;
					vec.push(result);
				}
			}
			if (rightLatLng) {
				rightProbeDist = probe.distanceFrom(rightLatLng);
				if (rightProbeDist <= radius) {
					result = new Result();
					result.latlng = rightLatLng;
					result.objs = rightObjs;
					vec.push(result);
				}
			}
			
			/* Test if the branches below might contain objects within the search radius. */
			if (leftBranch && (radius + maxLeft >= leftProbeDist)) {
				leftBranch._withinRadius(probe, radius, vec); 
			}
			if (rightBranch && (radius + maxRight >= rightProbeDist)) {
				rightBranch._withinRadius(probe, radius, vec);
			}
		}
		
		/** Finds the objects nearest to probe. All objs in Result are equidistant to probe, within the precision used to build the graph. */
		public function nearest(probe:LatLng, dist:Number=Number.MAX_VALUE):Result {
			var result:Result = new Result();
			_nearest(probe, dist, result); // contents of result are modified
			return result;			
		}
		
		/** Helper function for nearest. Modifies the contents of the result object. */
		protected function _nearest(probe:LatLng, dist:Number, result:Result):Number {
			var leftProbeDist:Number;
			var rightProbeDist:Number;
						
			if (leftLatLng) {
				leftProbeDist = probe.distanceFrom(leftLatLng);
				if (leftProbeDist <= dist) {
					dist = leftProbeDist;
					result.latlng = leftLatLng;
					result.objs = leftObjs;	
				}
			}
			
			if (rightLatLng) {
				rightProbeDist = probe.distanceFrom(rightLatLng);
				if (rightProbeDist <= dist) {
					dist = rightProbeDist;
					result.latlng = rightLatLng;
					result.objs = rightObjs;
				}
			}
			
			/* Test if the branches below might contain objects within the search radius. */
			if (leftBranch && (dist + maxLeft >= leftProbeDist)) {
				/* Recurse, and set dist to min of (recurse result, original dist) */ 
				dist = Math.min(leftBranch._nearest(probe, dist, result), dist);
			}
			
			if (rightBranch && (dist + maxRight >= rightProbeDist)) {
				dist = Math.min(rightBranch._nearest(probe, dist, result), dist);
			}
			
			return dist;
		}
		
		/** Does an preorder traversal of the tree, calling fnc twice at each node. fnc is called
		 * with leftObjs and then with rightObjs as an argument.
		 * @param thisArg If the function is a bound method, pass the object the method is bound to.
		 * @param fnc A function that takes an array of Objects as an argument.
		 */ 
		public function traverse(fnc:Function):void {
			if (leftObjs.length) {
				fnc(leftObjs);
				if (leftBranch) {
					leftBranch.traverse(fnc);
				}
			}
			
			if (rightObjs.length) {
				fnc(rightObjs);
				if (rightBranch) {
					rightBranch.traverse(fnc);
				}
			}

		}
		
		/** Creates a NearTree by inserting nodes in random order. latlngs and objects are parallel arrays. */
		public static function makeTree(latlngs:Vector.<LatLng>, objects:Array):NearTree {			
			var tree:NearTree = new NearTree();
			/* Create working copies of the vectors. Does not duplicate the LatLng or Object instances.*/
			var lls:Vector.<LatLng> = latlngs.slice(); 
			var objs:Array = objects.slice();
			
			/* Add the objects to the tree in random order. */	
			var i:uint;			
			while (lls.length > 0) {
				i = uint(Math.random()*lls.length); // choose a valid index at random
				tree.insert(lls[i], objs[i]);    // add the item
				/* remove the item from the vectors. */
				if (i == lls.length-1) {
					lls.pop();
					objs.pop();
				} else {
					lls[i] = lls.pop();
					objs[i] = objs.pop();
				}
			}
			return tree;
		}	
	}
}