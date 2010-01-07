package edu.ucsc.graph
{	
	import __AS3__.vec.Vector;
	
	import com.google.maps.LatLng;
	
	import edu.ucsc.track_builder.gtf_import.ShapeNode;
	
	import flexunit.framework.TestCase;

	public class GraphTest extends TestCase
	{
		private var precision:Number;
		
		public function setUp1():Graph {
			var shapes:Object = new Object();
			var vec1:Vector.<ShapeNode> = new Vector.<ShapeNode>();
			vec1.push(new ShapeNode('1', new LatLng(1,1), 1));
			vec1.push(new ShapeNode('1', new LatLng(1,2), 2));
			vec1.push(new ShapeNode('1', new LatLng(1,3), 3));
						
			var vec2:Vector.<ShapeNode> = new Vector.<ShapeNode>();			
			vec2.push(new ShapeNode('2', new LatLng(2,1), 1));
			vec2.push(new ShapeNode('2', new LatLng(1,1), 2)); 
			vec2.push(new ShapeNode('2', new LatLng(1,2.00001), 3));
			shapes['1'] = vec1;
			shapes['2'] = vec2;			
			
			precision = 5;			
			return Graph.fromShapes(shapes);			
		}
		
		/** An alternate test graph that contains cycles. Returns the graph. */
		public function setUp2():Graph {
			var shapes:Object = new Object();
			/* A square */
			var vec1:Vector.<ShapeNode> = new Vector.<ShapeNode>();
			vec1.push(new ShapeNode('1', new LatLng(1,1), 1));
			vec1.push(new ShapeNode('1', new LatLng(1,2), 2));
			vec1.push(new ShapeNode('1', new LatLng(2,2), 3));
			vec1.push(new ShapeNode('1', new LatLng(2,1), 4));
			vec1.push(new ShapeNode('1', new LatLng(1,1), 5));
			
			/* A SE to NW crossbar for the square */
			var vec2:Vector.<ShapeNode> = new Vector.<ShapeNode>();
			vec2.push(new ShapeNode('2', new LatLng(2,1), 1));
			vec2.push(new ShapeNode('2', new LatLng(1,2), 2));
			
			shapes['1'] = vec1;
			shapes['2'] = vec2;			
		
			precision = 5;	
			return Graph.fromShapes(shapes);						
		}
		
		/** Identical to setUp2, except that extra nodes have been added. The extra nodes are
		 * positioned such that they should be removed by simplify. */
		public function setUp2_ext():Graph {
			var shapes:Object = new Object();
			/* A square */
			var vec1:Vector.<ShapeNode> = new Vector.<ShapeNode>();
			vec1.push(new ShapeNode('1', new LatLng(1,1), 1));
			vec1.push(new ShapeNode('1', new LatLng(1,1.5), 2));
			vec1.push(new ShapeNode('1', new LatLng(1,2), 3));
			vec1.push(new ShapeNode('1', new LatLng(1.5,2), 4));
			vec1.push(new ShapeNode('1', new LatLng(2,2), 5));
			vec1.push(new ShapeNode('1', new LatLng(2,1.5), 6));
			vec1.push(new ShapeNode('1', new LatLng(2,1), 7));
			vec1.push(new ShapeNode('1', new LatLng(1.5,1), 8));
			vec1.push(new ShapeNode('1', new LatLng(1,1), 9));
			
			/* A SE to NW crossbar for the square */
			var vec2:Vector.<ShapeNode> = new Vector.<ShapeNode>();
			vec2.push(new ShapeNode('2', new LatLng(2,1), 1));
			vec2.push(new ShapeNode('2', new LatLng(1.5,1.5), 2));
			vec2.push(new ShapeNode('2', new LatLng(1,2), 3));
			
			shapes['1'] = vec1;
			shapes['2'] = vec2;			
		
			precision = 5;
			return Graph.fromShapes(shapes);
		}

		public function test_addNode():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			/* Node not in graph */
			var node10_10:Node = new Node(new LatLng(10,10), '10,10');
			graph.addNode(node10_10);
			assertTrue(graph.hasNode(node10_10));
			assertNotNull(graph.getNeighbors(node10_10));
			assertEquals(0, graph.getNeighbors(node10_10).length);
			
			/* Node already in graph */
			var nodeList:Vector.<Node> = graph.getNeighbors(node1_1);			
			graph.addNode(nodeList[0]);
			assertTrue(nodeVecsMatch(nodeList, graph.getNeighbors(node1_1)));			
		}

		public function test_deleteNode():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			graph.deleteNode(node1_1);
			assertFalse(graph.hasNode(node1_1));
			assertNull(graph.getNeighbors(node1_1));
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1, node2_2]), graph.getNeighbors(node1_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_2]), graph.getNeighbors(node2_1))); 
		}	

		public function test_removeNode():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			graph.removeNode(node2_1);
			assertFalse(graph.hasNode(node2_1));
			assertNull(graph.getNeighbors(node2_1)); // check that node is removed
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_2]), graph.getNeighbors(node1_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node2_2]), graph.getNeighbors(node1_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node1_2]), graph.getNeighbors(node2_2)));
		}
		
		public function test_addEdge():void {
			var graph:Graph = setUp2();
			
			/* Edge uses existing nodes */
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			graph.addEdge(node1_1, node2_2);			
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_1, node2_2]), graph.getNeighbors(node1_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node1_2, node2_2]), graph.getNeighbors(node2_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node2_1, node2_2]), graph.getNeighbors(node1_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node1_2, node2_1]), graph.getNeighbors(node2_2)));
			
			/* Edge uses (and resuses) new nodes */
			var node10_10:Node= new Node(new LatLng(10,10), '10,10');
			var node20_20:Node= new Node(new LatLng(20,20), '20,20');
			graph.addNode(node10_10);
			graph.addNode(node20_20);
			graph.addEdge(node10_10,node20_20);
			assertTrue(nodeVecsMatch(Vector.<Node>([node20_20]), graph.getNeighbors(node10_10)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node10_10]), graph.getNeighbors(node20_20)));				
		}

		public function test_removeEdge():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			graph.removeEdge(node1_1, node1_2);
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1]), graph.getNeighbors(node1_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1, node2_2]), graph.getNeighbors(node1_2)));
			
			graph.removeEdge(node2_1, node1_2);
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1]), graph.getNeighbors(node1_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node2_2]), graph.getNeighbors(node2_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_2]), graph.getNeighbors(node1_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_1]), graph.getNeighbors(node2_2)));
		}		
		
		public function test_getEdges():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			assertTrue(edgeVecsMatch(Vector.<Edge>([new Edge(node1_1, node1_2),
			                                       new Edge(node1_1, node2_1),
			                                       new Edge(node1_2, node2_1),
			                                       new Edge(node1_2, node2_2),
			                                       new Edge(node2_1, node2_2)]),
			                         graph.getEdges()));
		}


		/** Check that adjacency lists are as correct, without regard to their ordering */		
		public function test_getNeighbors():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_1]), graph.getNeighbors(node1_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node2_1, node2_2]), graph.getNeighbors(node1_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1, node1_2]), graph.getNeighbors(node2_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node1_2, node2_2]), graph.getNeighbors(node2_1)));
		}		
		
		/** Merges two adjacent nodes */
		public function test_mergeNodes_I():void {
			var graph:Graph = setUp1();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node15_1:Node = new Node(new LatLng(1.5,1), '1.5,1');
			var node1_200001:Node = new Node(new LatLng(1,2.00001), '1,2.00001');
			
			var mergeVec:Vector.<Node> = Vector.<Node>([node2_1, node1_1]);
			graph.mergeNodes(mergeVec);
			assertNull("2,1 Node destroyed", graph.getNeighbors(node2_1));
			assertNull("1,1 Node destroyed", graph.getNeighbors(node1_1));
			
			assertNotNull("1.5,1 Node created", graph.getNeighbors(node15_1));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node1_200001]), graph.getNeighbors(node15_1)));
		}
		
		/** Merges two nodes that are separated by another node */ 
		public function test_mergeNodes_II():void {
			var graph:Graph = setUp1();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node1_3:Node = new Node(new LatLng(1,3), '1,3');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node15_15:Node = new Node(new LatLng(1.5,1.5), '1.5,1.5');
			
			var mergeVec:Vector.<Node> = Vector.<Node>([node2_1, node1_2]);
			graph.mergeNodes(mergeVec);
			assertNull("2,1 Node destroyed", graph.getNeighbors(node2_1));
			assertNull("1,2 Node destroyed", graph.getNeighbors(node1_2));
			
			assertNotNull("1.5,1.5 Node created", graph.getNeighbors(node15_15));
			var expectedVec:Vector.<Node> = Vector.<Node>([node1_1, node1_3]);
			assertTrue(nodeVecsMatch(graph.getNeighbors(node15_15), expectedVec));
		}
		
		/** Merges all nodes in the graph */
		public function test_mergeNodes_III():void {						
			var graph:Graph = setUp1();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node1_200001:Node = new Node(new LatLng(1,2), '1,2.00001');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node1_3:Node = new Node(new LatLng(1,3), '1,3');
			var node15_2:Node = new Node(new LatLng(1.5,2), '1.5,2');
			
			var mergeVec:Vector.<Node> = Vector.<Node>([node2_1,
			                                                node1_1,
			                                                node1_2,
			                                                node1_200001,
			                                                node1_3]);
			graph.mergeNodes(mergeVec);
			assertNull("2,1 Node destroyed", graph.getNeighbors(node2_1));
			assertNull("1,1 Node destroyed", graph.getNeighbors(node1_1));
			assertNull("1,2 Node destroyed", graph.getNeighbors(node1_2));
			assertNull("1,2.00001 Node destroyed", graph.getNeighbors(node1_200001));
			assertNull("1,3 Node destroyed", graph.getNeighbors(node1_3));
			
			assertNotNull("1.5,2 Node created", graph.getNeighbors(node15_2)); // center
			assertTrue(graph.getNeighbors(node15_2).length == 0);
		}
				
		/** Merge two nodes such that the new center point matches an existing node */
		public function test_mergeNodes_IV():void {
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node1_200001:Node = new Node(new LatLng(1,2), '1,2.00001');
			var node1_3:Node = new Node(new LatLng(1,3), '1,3');			
			var graph:Graph = setUp1();
			
			var mergeVec:Vector.<Node> = Vector.<Node>([node1_1, node1_3]);
			graph.mergeNodes(mergeVec);
			
			var expectedVec:Vector.<Node> = Vector.<Node>([node2_1, node1_200001]);
			assertTrue(nodeVecsMatch(expectedVec, graph.getNeighbors(node1_2))); // the center for the merge
		}

		/** Construct an input vector that contains the same LatLng instances that are stored in the Graph. */
		public function test_mergeNodes_V():void {			
			var graph:Graph = setUp2();			
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			var vec:Vector.<Node> = graph.getNeighbors(node1_2); // adjacent to 1,1 and 2,1 and 2,2
			graph.mergeNodes(vec, new LatLng(2,1)); // merge all three to just 2,1
			
			assertTrue(graph.hasNode(node2_1));
			assertTrue(graph.hasNode(node1_2));
			assertFalse(graph.hasNode(node1_1));			
			assertFalse(graph.hasNode(node2_2));
			
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2]), graph.getNeighbors(node2_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1]), graph.getNeighbors(node1_2)));
		}
		
		/** Depends on getNeighbors() and getKeys() being correct. */
		public function test_simplify_I():void {
			var graph:Graph = setUp1();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node1_200001:Node = new Node(new LatLng(1,2), '1,2.00001');
			var node1_3:Node = new Node(new LatLng(1,3), '1,3');		
			
			graph.simplify(); // expected to remove node 1,2
			assertTrue(nodeVecsMatch(Vector.<Node>([node2_1, node1_200001, node1_3]), graph.getNeighbors(node1_1)));
			
			assertTrue(graph.hasNode(node2_1));
			assertTrue(graph.hasNode(node1_1));
			assertTrue(graph.hasNode(node1_3));
			assertTrue(graph.hasNode(node1_200001));
			
			assertFalse(graph.hasNode(node1_2));			
		}
		
		/** Tests on a graph that contains cycles. Depends on getNeighbors() and getKeys() being correct. */
		public function test_simplify_II():void {
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			var node15_15:Node = new Node(new LatLng(1.5,1.5), '1.5,1.5');
			var node15_2:Node = new Node(new LatLng(1.5,2), '1.5,2');			

			var graph:Graph = setUp2_ext();
			graph.simplify();
			
			/* Check that the nodes are correct */
			assertTrue(graph.hasNode(node1_1));
			assertTrue(graph.hasNode(node2_1));
			assertTrue(graph.hasNode(node1_2));
			assertTrue(graph.hasNode(node2_2));			
			assertFalse(graph.hasNode(node15_15));
			assertFalse(graph.hasNode(node15_2));
			
			/* Check that each nodes adjacency is correct */
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_1]), graph.getNeighbors(node1_1)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node2_2, node2_1]), graph.getNeighbors(node1_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_2, node2_1]), graph.getNeighbors(node2_2)));
			assertTrue(nodeVecsMatch(Vector.<Node>([node1_1, node2_2, node1_2]), graph.getNeighbors(node2_1)));
		}
		
		public function test_simplify_III():void {
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			var node15_15:Node = new Node(new LatLng(1.5,1.5), '1.5,1.5');
			var node15_2:Node = new Node(new LatLng(1.5,2), '1.5,2');	
			var graph:Graph = setUp2_ext();
			
			/* Test that simplify(keep) retains the requested nodes */
			var keep:Object = new Object();
			keep[node15_15] = node15_15;
			
			assertTrue(graph.hasNode(node15_2));
			graph.simplify(keep);
			assertTrue(graph.hasNode(node1_1));
			assertTrue(graph.hasNode(node15_15));
			assertFalse(graph.hasNode(node15_2));
		}
			
		
		/** Test that all nodes and edges which are connected to the starting node are visited exactly once in a BFS order. */
		public function test_breadthFirst():void {			
//			var graph:Graph = setUp2();
//			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
//			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
//			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
//			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
//			
//			var visitedHeads:Vector.<Node> = new Vector.<Node>();
//			var visitedTails:Vector.<Node> = new Vector.<Node>();
//			
//			function visit(head:LatLng, tail:LatLng):void {
//				visitedHeads.push(head);
//				visitedTails.push(tail);	
//			}
//			
//			graph.breadthFirst(node1_1, visit);
//			
//			assertEquals(visitedHeads.length, visitedTails.length);
//			assertEquals(5, visitedHeads.length);
//			var latlng_11:LatLng = node1_1;
//			var latlng_12:LatLng = new LatLng(1,2);
//			var latlng_21:LatLng = new LatLng(2,1);
//			var latlng_22:LatLng = new LatLng(2,2);
//			assertTrue(visitedHeads[0].equals(latlng_11));
//			assertTrue(visitedHeads[1].equals(latlng_11));
//			assertTrue(visitedHeads[2].equals(latlng_12) || visitedHeads[2].equals(latlng_21));
//			if (visitedHeads[2].equals(latlng_12)) {
//				assertTrue(visitedHeads[3].equals(latlng_12));
//				assertTrue(visitedHeads[4].equals(latlng_21));
//			} else { // moved to latlng_21
//				assertTrue(visitedHeads[3].equals(latlng_21));
//				assertTrue(visitedHeads[4].equals(latlng_12));
//			}
//			
//			assertTrue(visitedTails[0].equals(latlng_12) || visitedTails[0].equals(latlng_21));
//			assertTrue(visitedTails[1].equals(latlng_12) || visitedTails[1].equals(latlng_21));
//			if (visitedHeads[2].equals(latlng_12)) {
//				assertTrue(visitedTails[2].equals(latlng_21) || visitedTails[2].equals(latlng_22));
//				assertTrue(visitedTails[3].equals(latlng_21) || visitedTails[3].equals(latlng_22));
//			} else { // moved to latlng_21
//				assertTrue(visitedTails[2].equals(latlng_12) || visitedTails[2].equals(latlng_22));
//				assertTrue(visitedTails[3].equals(latlng_12) || visitedTails[3].equals(latlng_22));
//			}
//			assertTrue(visitedTails[4].equals(latlng_22));
		}
		

		
		public function test_getRandomNode():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			for (var i:int = 0; i < 50; ++i) {
				var node:Node = graph.getRandomNode();
				assertNotNull(node);
				assertNotNull(graph.getNeighbors(node));
			}
		}
		
		public function test_shortestPath():void {
			var graph:Graph = setUp2();
			var node1_1:Node = new Node(new LatLng(1,1), '1,1');
			var node1_2:Node = new Node(new LatLng(1,2), '1,2');
			var node2_1:Node = new Node(new LatLng(2,1), '2,1');
			var node2_2:Node = new Node(new LatLng(2,2), '2,2');
			
			/* Simple, one edge path. */
			assertTrue(pathsMatch(Vector.<Node>([node1_1, node1_2]), graph.shortestPath(node1_1, node1_2)));
			/* Two edge path. */
			assertTrue(pathsMatch(Vector.<Node>([node1_1, node2_1, node2_2]), graph.shortestPath(node1_1, node2_2)));
		}	
		
		/** Checks that two Node Vectors have contents that equal each other, without regard to position within the vector. */
		private function nodeVecsMatch(vec1:Vector.<Node>, vec2:Vector.<Node>):Boolean {
			trace('vec1', vec1, 'vec2:', vec2);			
			if (vec1.length != vec2.length) {
				return false;
			}
			
			var v2:Vector.<Node> = vec2.concat(); // make copy
			var numRemoved:int = 0;
			for each (var a:Node in vec1) {
				for (var i:int = 0; i < v2.length; ++i) {
					if (a.equals(v2[i])) {
						if (i == v2.length-1) {
							v2.pop();
						} else {
							v2[i] = v2.pop();
						}
						++numRemoved;
					}
				}
			}
			if (numRemoved == vec1.length && v2.length == 0) {
				return true;
			} else {
				return false;
			}
		}
		
		/** Checks that two Edge Vectors have contents that equal each other, without regard to position within the vector. */
		private function edgeVecsMatch(vec1:Vector.<Edge>, vec2:Vector.<Edge>):Boolean {
			trace('vec1', vec1, 'vec2:', vec2);			
			if (vec1.length != vec2.length) {
				return false;
			}
			
			var v2:Vector.<Edge> = vec2.concat(); // make copy
			var numRemoved:int = 0;
			for each (var a:Edge in vec1) {
				for (var i:int = 0; i < v2.length; ++i) {
					if (a.equals(v2[i])) {
						if (i == v2.length-1) {
							v2.pop();
						} else {
							v2[i] = v2.pop();
						}
						++numRemoved;
					}
				}
			}
			if (numRemoved == vec1.length && v2.length == 0) {
				return true;
			} else {
				return false;
			}
		}
		
		/** Checks that two paths are identical. */
		private function pathsMatch(a:Vector.<Node>, b:Vector.<Node>):Boolean {
			if (a.length != b.length) {
				return false;
			}
			
			for (var i:int=0; i < a.length; ++i) {
				if (!a[i].equals(b[i])) {
					return false;
				}
			}
			
			return true;
		}
	}
}