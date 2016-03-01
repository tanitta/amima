module amima.system;

/++
+/
class System(T)if(__traits(isFloating, T)){
	import amima.node;
	import amima.edge;
	public{
		
	}//public

	private{
		// Node!(T)[] _nodes;
		// Edge!(T)[] _edges;
		Edge!(T) _edge;
	}//private
}//class System
