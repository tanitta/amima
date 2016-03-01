module amima.activation;

/++
+/
struct FitzHughNagumoModel(T)if(__traits(isFloating, T)){
	public{
		/++
		+/
		this(in T v0 = T(-1.9), in T w0 = T(-1.5)){
			_v = v0;
			_w = w0;
		}
		
		/++
		+/
		@property
		T v()const{return _v;}
		
		/++
		+/
		@property
		T w()const{return _w;}
		
		/++
		+/
		void update(in T unitTime, in T iExt = T(0)){
			T vOld = _v;
			T wOld = _w;
			
			_v = unitTime * (vOld - T(1.0)/T(3.0)* vOld * vOld * vOld - wOld + iExt)+vOld;
			_w = unitTime * T(0.08) * (vOld+ T(0.7) - T(0.8) * wOld) + wOld;
		}
	}//publi

	private{
		T _v;
		T _w;
	}//private
}//struct FitzHughNagumoModel
unittest{
	static assert(__traits(compiles, (){
		auto f = FitzHughNagumoModel!(double)();
	}));
	static assert(!__traits(compiles, (){
		auto f = FitzHughNagumoModel!(int)();
	}));
	static assert(!__traits(compiles, (){
		auto f = FitzHughNagumoModel!(string)();
	}));
}
