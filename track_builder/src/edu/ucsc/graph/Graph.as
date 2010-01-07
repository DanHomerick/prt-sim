package edu.ucsc.graph
{
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	import com.google.maps.LatLngBounds;
	
	import edu.ucsc.track_builder.gtf_import.ShapeNode;
	
	import mx.utils.ObjectUtil;
	
	/** An undirected graph of a transit network, represented by adjacency lists.
	 * Each unique coordinate pair is treated as a node in the graph. Each node has an
	 * adjacency list containing LatLng objects.  
	 * The constructor will merge ShapeNodes that share a latlng. The shape_ids are
	 * discarded in the process.
	 */
	public class Graph
	{
		/** The number of places after the decimal for which precision is carried */ 
		static public var PRECISION:uint = 5;
		
		protected var nodes:Object;
		protected var adjList:Object;	

		public function Graph()			
		{
			nodes = new Object();
			adjList = new Object();
		}

		public static function fromShapes(shapes:Object):Graph {
			var graph:Graph = new Graph(); 
			
			for each (var vec:Vector.<ShapeNode> in shapes) {
				vec.sort(ShapeNode.cmp);
				if (vec.length >= 2) {
					/* setup */					
					var currNode:Node;
					var prevNode:Node = Node.fromLatLng(vec[0].latlng);					
					graph.addNode(prevNode); // does nothing if node is already in the graph.
					
					/* loop through all shapeNodes, creating adjacency */
					for (var i:uint = 1; i < vec.length; ++i) {
						currNode = Node.fromLatLng(vec[i].latlng);
						if (currNode.equals(prevNode)) { // Two nodes in sequence are at the same coordinates, within precision)
							continue;			        // skip this node as though it didn't exist
						}
						graph.addNode(currNode);  // does nothing if node is already in the graph.						
						graph.addEdge(prevNode, currNode); // adds a bidirectional edge (without creating a multigraph).						
						prevNode = currNode;
					}
				}
			}
			return graph;
		}

		public function hasNode(node:Node):Boolean {
			return nodes[node] != null;
		}

		/** Adds a node. If the node exists already, it does nothing. */
		public function addNode(node:Node):void {
			if (nodes[node] == null) {
				nodes[node] = node;
				adjList[node] = new Vector.<Node>();
			} // else the node was already in the graph. Do nothing. 	
		}
		
		/** Adds an edge to the undirected graph. Nodes must exist in the graph already. If the edge already exists, it does nothing.
		 * If the edge would be form a self-loop, it does nothing. */
		public function addEdge(a:Node, b:Node):void {
			if (a.equals(b)) {
				return;
			}
			var aList:Vector.<Node> = adjList[a];
			var bList:Vector.<Node> = adjList[b];
			
			/* Check if a is already adjacent to b. We assume that the graph is valid, and don't check the other direction */ 
			var adjacent:Boolean = aList.some(function(item:Node, idx:Number, vec:Vector.<Node>):Boolean {
				                                             return b.equals(item);});
			if (!adjacent) {
				aList.push(b);
				bList.push(a);
			} // else the nodes already have an edge between them. Do nothing.				
		}

		/** Removes nodes from the graph which meet the following criteria:
		 *   1. Node has a degree of 2 (adjacent to exactly two other nodes).
		 *   2. Is in line with the other two nodes, as determined using the triangle inequality.
		 * @param keep An associative array whose keys are Node strings. The values are not used, and may be anything except 'null'.
		 */
		public function simplify(keep:Object=null):void {
			trace("SimplifyGraph");
			var removedCnt:uint = 0;
			for each (var nodeB:Node in nodes) {
				/* If a keep object was passed in, and the current node is in it, continue */
				if (keep != null && keep[nodeB] != null) {
					continue;
				}
				
				var adjNodesB:Vector.<Node> = adjList[nodeB];
				if (adjNodesB.length == 2) {
					var nodeA:Node = adjNodesB[0];
					var nodeC:Node = adjNodesB[1];
					
					// use triangle inequality to see if they are in a straight line with each other
					var abDist:Number = nodeA.latlng.distanceFrom(nodeB.latlng);
					var bcDist:Number = nodeB.latlng.distanceFrom(nodeC.latlng);
					var acDist:Number = nodeA.latlng.distanceFrom(nodeC.latlng);
					
//					trace("ab:", abDist, "bc:", bcDist, "ab+bc:", abDist+bcDist, "ac:", acDist, "diff:", abDist+bcDist-acDist, "relative diff:", acDist/(abDist+bcDist) );				
					/* 0.99905 = 5deg; 0.99619 = 10deg; 0.991444 = 15 deg; 0.98481 = 20deg */
					if (acDist/(abDist+bcDist) > 0.99905) { 
						removeNode(nodeB);
						++removedCnt;
					}  
				}
			}
			
			trace("simplifyGraph. nodes removed:", removedCnt); 
		}

		/** Merges 2 or more nodes, keeping the connectivity intact. Replaces the nodes with a new node, placed at the center
		 * of the boundary formed by the nodes.
		 * @param latlngs A Vector containing LatLng instances whose coordinates should be merged in the graph.
		 * @throws ArgumentError if any of the LatLng's aren't nodes in the graph.
		 */ 
		public function mergeNodes(targets:Vector.<Node>, center:LatLng=null):void {
			if (targets.length < 2) {
				if (targets.length == 1 && center != null) {
					targets[0].latlng = center;
				}
				return;
			}
			targets = targets.slice(); // work with a copy, in case the vector is from the graph
			
			var centerNode:Node;
			if (center == null) {
				/* Find the center of the positions.
				 * Use center rather than the average to avoid issues when coordinates cross a hemisphere boundary. */
				var bounds:LatLngBounds = new LatLngBounds();
				for each (var node:Node in targets) {
					if (nodes[node] != null) { // check that node is in graph.
						bounds.extend(node.latlng);					
					} // else silently ignore this Node, since it's not part of the graph.
				}
				centerNode = Node.fromLatLng(bounds.getCenter());
				
			} else {
				centerNode = Node.fromLatLng(center);
			}			
			
			if (nodes[centerNode] == null) {
				addNode(centerNode);
			}
			
			/* Each mergeNode will be removed. Prior to it's removal, remove all inbound edges and recreate them on center. */ 
			for each (var target:Node in targets) { 
				for each (var adjNode:Node in adjList[target].slice()) { // work with a copy, so that alterations to the vector don't affect the iteration
					removeEdge(adjNode, target); // remove the edge between mergeNode and adjNode
					addEdge(centerNode, adjNode); // add an edge (both directions) between center and the adjNode, if not already present, and if not a selfloop
				}
			}
			
			for each (target in targets) {
				if (!target.equals(centerNode)) {
					deleteNode(target);
				}				
			}
		}
		
		public function getNode(key:String):Node {
			return nodes[key];
		}
		
		public function getNodes():Object {
			return nodes;
		}
		
		public function getNeighbors(node:Node):Vector.<Node> {
			return adjList[node];
		}
		
		/** Removes an edge from the graph. */ 
		public function removeEdge(node1:Node, node2:Node):void {
			removeEdgeDir(node1, node2);
			removeEdgeDir(node2, node1);
		}		
		
		/** @return Contains a single Edge for each edge in the graph. The order of the edges
		 * in the vector is arbitrary, as is the ordering of the nodes in an edge. */
		public function getEdges():Vector.<Edge> {
			var seen:Object = new Object();
			var result:Vector.<Edge> = new Vector.<Edge>();
			
			for each (var node:Node in nodes) {
				var adjNodes:Vector.<Node> = adjList[node];
				for each (var adjNode:Node in adjNodes) {
					if (seen[adjNode] == null) {
						result.push(new Edge(node, adjNode));
					}
				}
				seen[node] = true;
			}
			return result;
		}
		
		/** Helper function for removeEdge. Removes just one direction. */
		protected function removeEdgeDir(tail:Node, head:Node):void {
			var nodeList:Vector.<Node> = adjList[tail];
			for (var i:int = nodeList.length-1; i > -1; --i) {
				if (nodeList[i].equals(head)) {	
					if (i == nodeList.length-1) {
						nodeList.pop(); // just discard it
					} else {
						// the lists are unordered, so remove an item from the vector by 
						// overwriting it with the last element. Note that the last element
						// has already been checked.
						nodeList[i] = nodeList.pop();
					}
					break;
				}
			}
		}
		
		/** Removes a node, leaving all of its neighbors connected to each other. */
		public function removeNode(n:Node):void {
			var nList:Vector.<Node> = getNeighbors(n).slice(); // work with a copy of the vector, so that we can alter the original without messing up iteration 
			
			/* For each neighbor of node n */
			for (var i:int=0; i < nList.length; ++i) {
				var iNode:Node = nList[i]; 
				
				/* Connect it to all of n's other neighbors, without causing duplicate edges */
				for (var j:int=i+1; j < nList.length; ++j) {
					var jNode:Node = nList[j];					
					addEdge(iNode, jNode);
				}
				
				/* Remove the edge that connects it with node n */
				removeEdge(n, iNode);
			}
			
			deleteNode(n);
		}
		
		/** Deletes a node, ensuring that all incident edges are removed. */
		public function deleteNode(n:Node):void {
			var adjNodes:Vector.<Node> = adjList[n];
			if (adjNodes == null) {
				return;
			}
			for each (var adjNode:Node in adjNodes.slice()) { // make a working copy
				removeEdge(n, adjNode);
			}
			delete nodes[n];
			delete adjList[n];
		}
		
		/** Breadth first search of the graph. Visited nodes are saved between calls to breadthFirst. 
		 * @param node  The graph node at which the traversal starts.
		 * @param visitEdge A callback function which is invoked for every child of a node.
		 *              The tail (the visited node) is accessible through
		 *              the function's first argument, and the head (the child node) is
		 *              accessible through the function's second argument.
		 * @returns An object acting as a dictionary, with keys in LatLng.toUrlValue(precision) format, and values of LatLngs. The object contains all LatLngs that were visited. 
		 * @see #.getUnvisitedKeys
		 */
		public function breadthFirst(node:Node, visitEdge:Function):Object
		{
			var visited:Object = new Object();
			
			var fifo:Vector.<Node> = new Vector.<Node>(); // new items to the rear, pull from the front.
			fifo.push(node);
			
			while (fifo.length > 0) {
				var n:Node = fifo.shift();
				
				if (!visited[n]) {
					var nList:Vector.<Node> = adjList[n];	
					fifo = fifo.concat(nList); // add children to the FIFO					
					for each (var adjNode:Node in nList) {
						if (!visited[adjNode]) {						
							visitEdge(n, adjNode);
						}
					}
					visited[n] = true;
				}
			}
			return visited;			
		}
		
		/**
		 * Finds the shortest path from source to target using Dijkstra's algo.
		 * @param source The starting node.
		 * @param target The destination node.
		 * @return The shortest path from source to target. If target is unreachable from source, returns <code>null</code>.
		 */
		public function shortestPath(source:Node, target:Node):Vector.<Node> {			
			var dist:Object = new Object(); // mapping from a Node to a distance (Number).
			var previous:Object = new Object();
			
			/* Compare by dist value, where a low distance yields a high priority. null is treated as lowest priority. */
			function cmp(a:Node, b:Node):Number {
				var distA:Number = dist[a];
				var distB:Number = dist[b];
				return distB - distA; // compare by actual distances. Give positive value (high priority) to 'a' if it's a shorter dist.
			}
								
			/* Initialize queue and distances */
			var queue:Vector.<Node> = new Vector.<Node>();
			for each (var n:Node in nodes) {
				queue.push(n);
				dist[n] = Number.POSITIVE_INFINITY;
			} 		
			dist[source] = 0;
			
			var result:Vector.<Node>;
			while (queue.length) {
				queue.sort(cmp); // resort the entire queue each time, since the distance metric has changed for several elements.
				var u:Node = queue.pop();
				
				if (!u.equals(target)) { // regular case				
					/* check if target is unreachable */
					if (dist[u] == Number.POSITIVE_INFINITY) {
						break;
					}
					var neighbors:Vector.<Node> = getNeighbors(u);
					for each (var v:Node in neighbors) {					
						var alt:Number = dist[u] + u.latlng.distanceFrom(v.latlng);
						var distV:Number = dist[v];
						if (alt < distV) {
							dist[v] = alt;
							previous[v] = u;
						}
					}
				}  else { // we've reached target
					result = new Vector.<Node>();
					while (previous[u] != null) {
						result.push(u);
						u = previous[u];
					}
					result.push(u);
					result.reverse();
					break;		
				}
			}			
			
			return result;
		}
		
		public function shortestPathEdges(source:Node, target:Node):Vector.<Edge> {
			var path:Vector.<Node> = shortestPath(source, target);
			var edges:Vector.<Edge> = new Vector.<Edge>();			
			for (var i:int=0; i < path.length-1; ++i) { // stop 1 short of the last element
				edges.push(new Edge(path[i], path[i+1]));
			}
			return edges;
		}
		
		/** 
		 * @param visited An object in the same format as is returned by breadthFirst.
		 * @returns A Vector containing all nodes in the graph that are not contained in visited.
		 */
		public function getUnvisitedKeys(visited:Object):Vector.<String> {
			var vec:Vector.<String> = new Vector.<String>();
			for (var key:String in adjList) {
				if (visited[key] == null) {
					 vec.push(key);
				}
			}
			return vec;
		}
		
		public function getRandomNode():Node {
			var keys:Array = ObjectUtil.getClassInfo(nodes).properties;
			var idx:uint = uint(Math.random()*keys.length); // choose a valid index at random
			var node:Node = nodes[keys[idx]];
			return node;
		}
		
		public function getNodeCount():Number {
			var keys:Array = ObjectUtil.getClassInfo(nodes).properties;
			return keys.length;
		}
		
		/** Returns true if the edge exists, and is properly formed. Mainly for debugging purposes. */
		public function isEdge(nodeA:Node, nodeB:Node):Boolean {
			var listA:Vector.<Node> = adjList[nodeA];
			var listB:Vector.<Node> = adjList[nodeB];
			var ab:Boolean = listA.some(function(item:Node, idx:Number, vec:Vector.<Node>):Boolean {
				                                             return nodeB.equals(item);});
			var ba:Boolean = listB.some(function(item:Node, idx:Number, vec:Vector.<Node>):Boolean {
				                                             return nodeA.equals(item);});
			if (ab && ba) {
				return true;
			} else {
				return false;
			}
		}
		
		/** For debugging purposes. Finds invalid edges and gives a trace for each one. */
		public function dumpInvalidEdges():void {
			for each (var node:Node in nodes) {
				var list:Vector.<Node> = getNeighbors(node);
				for each (var adjNode:Node in list) {
					if (getNeighbors(adjNode) == null) {
						trace(adjNode.toString(), 'adjacent to', node.toString(), 'is nonexistant.');
					}
					
					else if (!isEdge(node, adjNode)) {
						trace(node.toString(), '->', adjNode.toString(), 'exists, but reverse direction is missing.');
						isEdge(node, adjNode); // for stepping through with debugger
					}						
				}
			}
		}		
	}
}