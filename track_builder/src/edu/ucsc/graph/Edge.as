package edu.ucsc.graph
{	
	/** Represents an undirected edge in a Graph.*/
	public class Edge
	{
		public var tail:Node;
		public var head:Node;
		public var key:String;
		
		public function Edge(tail:Node, head:Node) {
			this.tail = tail;
			this.head = head;
			this.key = tail.key + '|' + head.key;
		}
		
		public function toString():String {
			return key;
		}
		
		/** Since an Edge is meant to be undirected, equals retuns true even when other is reversed. */
		public function equals(other:Edge):Boolean {
			if (this.head.equals(other.head) && this.tail.equals(other.tail)) {
				return true;
			} else if (this.head.equals(other.tail) && this.tail.equals(other.head)) {
				return true;
			} else {
				return false;
			}
		}
		
	}
}