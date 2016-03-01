module amima.edge;

/++
+/
class Edge(T)if(__traits(isFloating, T)){
	import amima.node;
	public{
		this(Node!T fromNode, Node!T toNode){
			_fromNode = fromNode;
			_toNode= toNode;
		}
		///
		@property
		const(Node!T) fromNode()const{return _fromNode;}
		
		///
		@property
		const(Node!T) toNode()const{return _toNode;}
		
		///
		@property
		T weight()const{return _weight;}
	}//public

	private{
		Node!T _fromNode;
		Node!T _toNode;
		T _weight;
	}//private
}//class Edge
