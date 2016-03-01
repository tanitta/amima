module amima.system;
import amima.node;
import amima.edge;

/++
+/
class System(T)if(__traits(isFloating, T)){
	public{
		
	}//public

	private{
		Node!(T)[] _nodes;
		Edge!(T)[] _edges;
	}//private
}//class System
