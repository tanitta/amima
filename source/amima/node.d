module amima.node;

/++
+/
class Node(T)if(__traits(isFloating, T)){
	import amima.edge;
	import amima.activation;
	public{
		Edge!(T)[] _fromEdges;
		Edge!(T)[] _toEdges;
		
		/++
		+/
		this(){
			// _activationFunction = FitzHughNagumoModel!(T);
			// _sum = T(0);
		}
		
		///
		@property
		T v()const{return _activationFunction.v;}
		
		///
		@property
		T w()const{return _activationFunction.w;}
		
		/++
		+/
		void update(in T unitTime){
			_sum = T(0);
			foreach (fromEdge; _fromEdges) {
				_sum += fromEdge.fromNode.v * fromEdge.weight;
			}
			_activationFunction.update(unitTime, _sum);
		}
	}//public

	private{
		T _sum;
		FitzHughNagumoModel!(T) _activationFunction;
	}//private
}//class Node
unittest{
	auto n = new Node!double();
}
