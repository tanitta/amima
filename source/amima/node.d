module amima.node;

import armos.math.vector:Vector;
import amima.edge;
import amima.activation;
/++
+/
class Node(T)if(__traits(isFloating, T)){
	alias V = Vector!(T, 3);
	
	public{
		/++
		+/
		this(in V position){
			_activationFunction = FitzHughNagumoModel!(T)();
			_position = position;
		}
		
		///
		@property
		T v()const{return _activationFunction.v;}
		
		///
		@property
		T w()const{return _activationFunction.w;}
		
		///
		@property
		V position()const{return _position;}
		
		/++
		+/
		void update(in T unitTime){
			T _sum = T(0);
			foreach (fromEdge; _fromEdges) {
				_sum += fromEdge.fromNode.v * fromEdge.weight;
			}
			_activationFunction.update(unitTime, _sum);
		}
	}//public

	private{
		FitzHughNagumoModel!(T) _activationFunction;
		
		Edge!(T)[] _fromEdges;
		Edge!(T)[] _toEdges;
		
		V _position;
	}//private
}//class Node
unittest{
	auto n = new Node!double();
}
