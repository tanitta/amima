import armos;
import amima;

class TestApp : ar.BaseApp{
	amima.System!(double) _system;
	
	this(){
	}
	
	void setup(){
		_system = new amima.System!(double);
	}
	
	void update(){}
	
	void draw(){}
	
	void keyPressed(int key){}
	
	void keyReleased(int key){}
	
	void mouseMoved(ar.Vector2i position, int button){}
	
	void mousePressed(ar.Vector2i position, int button){}
	
	void mouseReleased(ar.Vector2i position, int button){}
}

void main(){
	ar.run(new TestApp);
}
