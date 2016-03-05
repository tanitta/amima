module amima.edge;

import amima.node;

/++
+/
class Edge(T)if(__traits(isFloating, T)){
	public{
		this(Node!(T) from, Node!(T) to){
			_fromNode = from;
			_toNode = to;
		}
		///
		const(Node!T) fromNode()const{return _fromNode;}
		
		///
		const(Node!T) toNode()const{return _toNode;}
		
		///
		T weight()const{return _weight;}
	}//public

	private{
		Node!T _fromNode;
		Node!T _toNode;
		T _weight;
	}//private
}//class Edge
