// #pragma once
#include <pharticle/pharticle.hpp>
#include <Eigen/Core>
#include "ofMain.h"
#include "ofxUI.h"


namespace amima {
	class Config {
		public:
			double unit_time_;

			double viscosity_coefficient_;

			ofColor neural_node_;
			ofColor neural_edge_;
			double neural_edge_damper_;
			double neural_edge_spring_;
			double neural_node_radius_;

			Config(){
				unit_time_ = 1.0/300.0;
				viscosity_coefficient_ = 1.0;
				double h = ofMap(20,0,260,0,255);
				neural_node_.setHsb(h,0,200);
				neural_edge_.setHsb(h,0,200);
				neural_edge_spring_ = 20;
				neural_edge_damper_ = 10;
				neural_node_radius_ = 2;
			};
			virtual ~Config(){};
			
			ofColor NeuralColor(double signal){
				ofColor c;
				double l = ofMap(signal,-2.0,2.8,0.0,1.0);
				double s = -4.0*(l - 0.5)*(l - 0.50)+1.0;
				double v = 0.7*l+0.3;
				c.setSaturation(ofMap(s*0.8,0.0,1.0,0.0,255.0));
				c.setBrightness(ofMap(v,0.0,1.0,0.0,255.0));
				c.setHue(230.0-110.0*l);
				return c;
			}
	};
	
	class FitzHughNagumoModel {
		public:
			double v_;
			double w_;
			double unit_time_;
			FitzHughNagumoModel(double v_0 = 1.1, double w_0 = 1.1, double unit_time = 1.0):v_(v_0),w_(w_0),unit_time_(unit_time){
			
			};
			
			virtual ~FitzHughNagumoModel(){};
			
			void update(double i_ext){
				double v_old = v_;
				double w_old = w_;
			
				v_ = unit_time_ * (v_old-1.0/3.0* v_old*v_old*v_old -w_old+i_ext)+v_old;
				w_ = unit_time_ * 0.08 * (v_old + 0.7 - 0.8 * w_old) + w_old;
			};
	};
	
	class BaseEntity {
		protected:
			Config& config_;
		public:
			BaseEntity(Config& config):config_(config){};
			virtual ~BaseEntity(){};
	};
	
	class BaseNode :public BaseEntity{
		public:
			pharticle::Particle particle_;
			BaseNode(Config& config):
				BaseEntity(config),
				particle_()
		{
			particle_.position_ = (Eigen::Vector3d(
						ofRandom(0,20.0),
						ofRandom(0,20.0),
						ofRandom(0,20.0)));
			particle_.radius_ = config_.neural_node_radius_;
		};
			virtual ~BaseNode(){};

			void set_mass(double mass){particle_.mass_ = mass;};
			void set_position(Eigen::Vector3d position){particle_.position_ = position;};
			void set_velocity(Eigen::Vector3d velocity){particle_.velocity_ = velocity;};
			void set_acceleration(Eigen::Vector3d acceleration){particle_.acceleration_ = acceleration;};

			double mass()const{return particle_.mass_;};
			Eigen::Vector3d position()const{return particle_.position_;};
			Eigen::Vector3d velocity()const{return particle_.velocity_;};
			Eigen::Vector3d acceleration()const{return particle_.acceleration_;};
			pharticle::Particle& particle(){return particle_;};


			void add_force(Eigen::Vector3d force){
				particle_.acceleration_ += force / particle_.mass_;
			};

			virtual void draw()const{};
	};
	class NeuralEdge;
	class NeuralNode : public amima::BaseNode{
		public:
			FitzHughNagumoModel activation_function_;
			std::vector<std::shared_ptr<amima::NeuralEdge>> froms_;
			std::vector<std::shared_ptr<amima::NeuralEdge>> tos_;;
			double sum_;
			bool is_static_;
			
			NeuralNode(Config& config):BaseNode(config),activation_function_(1.1,1.1,config_.unit_time_*10.0),is_static_(false),sum_(0){};

			virtual ~NeuralNode(){};
			
			void update();
			
			void draw()const override{
				ofSetColor(config_.NeuralColor(activation_function_.v_));
				ofDrawSphere(position()[0],position()[1],position()[2],2);
			}

	};

	template<typename T> 
	class BaseEdge :public BaseEntity{
		public:
			std::shared_ptr<T> from_;
			std::shared_ptr<T> to_;
			double length_;
			std::function<Eigen::Vector3d(pharticle::Particle&, pharticle::Particle&)> tension_function;

			BaseEdge(Config& config, std::shared_ptr<T> from, std::shared_ptr<T> to, double length):
				BaseEntity(config),
				from_(from),
				to_(to),
				length_(length)
		{};

			std::shared_ptr<T> from(){
				return from_;
			};
			std::shared_ptr<T> to(){
				return to_;
			};
			double length()const{
				return length_;
			};

			virtual ~BaseEdge(){};

			virtual void draw()const{};

			virtual void update_tension(){};

			void add_constraint_to(pharticle::Engine& engine_ref){
				// pharticle::ConstraintPair constraint_pair();
				auto f = [=](pharticle::Particle& p1, pharticle::Particle& p2){
					Eigen::Vector3d v(0,0,0);
					return v;
				};
				// engine_ref.add_constraint_pair_as_both_directions_consist_of(from()->particle(), to_->particle(),f);
			}
	};
	
	

	class NeuralNode;
	class NeuralEdge : public amima::BaseEdge<NeuralNode>{
		private:
		public:
			double weight_;
			NeuralEdge(Config& config, std::shared_ptr<NeuralNode> from, std::shared_ptr<NeuralNode> to, double length):
				BaseEdge(config, from, to, length)
		{};

			virtual ~NeuralEdge(){};

			double weight()const{
				return weight_;
			};

			void adjust_weight(double diff){
				weight_ += diff;
			};

			void update(){
			}
			void draw()const override{
				double v_avg = (from_->activation_function_.v_+to_->activation_function_.v_)*0.5;
				ofSetColor(config_.NeuralColor(v_avg));
				
				ofPoint from_position(from_->position()[0],from_->position()[1],from_->position()[2]);
				ofPoint to_position(to_->position()[0],to_->position()[1],to_->position()[2]);
				ofLine(from_position, to_position);
			};
	};
	
	void NeuralNode::update(){
		if (is_static_) {
			activation_function_.update(sum_);
		}else{
			double sum_ = 0;
			for (auto&& edge : froms_) {
				sum_ += edge->from_->activation_function_.v_;
			}
			activation_function_.update(sum_);
		}
	};

	enum class Type {
		Neuron,
		Mascle
	};

	class World {
		Config& config_;
		pharticle::Engine ph_engine_;

		public:
		std::vector<std::shared_ptr<NeuralNode>> nodes_;
		std::vector<std::shared_ptr<NeuralEdge>> edges_;
		World(Config& config):config_(config){
		};

		virtual ~World(){};

		std::shared_ptr<NeuralNode> add_node(Type type){
			if (type == Type::Neuron) {
				std::shared_ptr<NeuralNode> neural_node(new NeuralNode(config_));
				nodes_.push_back(neural_node);
				return neural_node;
			}
			if (type == Type::Mascle) {
			}
		}

		std::shared_ptr<NeuralEdge> add_edge(Type type, double length, std::shared_ptr<NeuralNode> from, std::shared_ptr<NeuralNode> to, double weight = 1.0){
			if (type == Type::Neuron) {
				std::shared_ptr<NeuralEdge> neural_edge(new NeuralEdge(config_, from, to, length));
				neural_edge->weight_ = weight;
				edges_.push_back(neural_edge);
				to->froms_.push_back(neural_edge);
				from->tos_.push_back(neural_edge);
				return neural_edge;
			}

			if (type == Type::Mascle) {
			}

		};

		std::shared_ptr<NeuralNode> add_node_from(Type type, double length, std::shared_ptr<NeuralNode> from, double weight = 1.0){
			if (type == Type::Neuron) {
				std::shared_ptr<NeuralNode> new_neural_node = add_node(type);
				nodes_.push_back(new_neural_node);
				add_edge(type, length, from, new_neural_node, weight);
				return new_neural_node;
			}
			if (type == Type::Mascle) {
			}
		};

		std::shared_ptr<NeuralNode> add_node_to(Type type, double length, std::shared_ptr<NeuralNode> to, double weight = 1.0){
			if (type == Type::Neuron) {
				std::shared_ptr<NeuralNode> new_neural_node = add_node(type);
				nodes_.push_back(new_neural_node);
				add_edge(type, length, new_neural_node, to, weight);
				return new_neural_node;
			}
			if (type == Type::Mascle) {
			}
		};

		private:
		void setup_physics(){
			ph_engine_.set_unit_time(config_.unit_time_);
			ph_engine_.set_collision_reaction_force([=](pharticle::Particle& p1, pharticle::Particle& p2){
					Eigen::Vector3d f(0,0,0);
					Eigen::Vector3d p(0,0,0);
					p = (p2.position_ - p1.position_);

					double r = p1.radius_ + p2.radius_;
					Eigen::Vector3d v_r = r*p.normalized();
					f = (v_r - p)*500*p2.mass_;
					return f;
					});
		}

		public:
		void setup(){
			setup_physics();
		};
		
		void expand(){
			for (auto&& node : nodes_) {
				double spring = ( node->froms_.size() + node->tos_.size() )*0.1 ;
				std::cout<<node->froms_.size() + node->tos_.size()<<std::endl;
				if (node->froms_.size() + node->tos_.size() >= 2) {
					Eigen::Vector3d current_position = node->particle_.position_;
					Eigen::Vector3d positions_sum(0,0,0);
					for (auto&& edge : node->froms_) {
						positions_sum += edge->from_->particle_.position_;
					}
					for (auto&& edge : node->tos_) {
						positions_sum += edge->to_->particle_.position_;
					}
					Eigen::Vector3d target_position = positions_sum / ( node->froms_.size() + node->tos_.size());
					// Eigen::Vector3d core_force = ( target_position - current_position )*-2.0;
					// node->add_force(core_force);
					for (auto&& edge : node->froms_) {
						Eigen::Vector3d core_to_near = edge->from_->particle_.position_ - current_position;
						Eigen::Vector3d force = ( edge->from_->particle_.position_ - target_position);
						// edge->from_->add_force(( -force + core_to_near.normalized()*core_to_near.dot(force)  )*spring);
						edge->from_->add_force((force)*spring);
					}
					for (auto&& edge : node->tos_) {
						Eigen::Vector3d core_to_near = edge->to_->particle_.position_ - current_position;
						Eigen::Vector3d force = ( edge->to_->particle_.position_ - target_position);
						// edge->to_->add_force(( -force + core_to_near.normalized()*core_to_near.dot(force)  )*spring);
						edge->to_->add_force((force)*spring);
					}
				}
			}
		};

		private:
		void update_neuron(){
			for (auto&& node : nodes_) {
				node->update();
			}
		}
		
		void update_physics(){
			expand();
			for (auto&& node : nodes_) {
				node->add_force(-node->particle().velocity_*config_.viscosity_coefficient_);
				// node->add_force(-node->particle().mass_*9.8*Eigen::Vector3d(0,-1,0));
				ph_engine_.add(node->particle());
			}

			for (auto&& edge : edges_) {
				auto func_neural_edge = [=](pharticle::Particle& p1, pharticle::Particle& p2){
					double k = config_.neural_edge_spring_;
					double c = config_.neural_edge_damper_;
					Eigen::Vector3d direct_of_force = (p1.position_ - p2.position_).normalized();
					double norm = (p1.position_- p2.position_).norm();
					
					Eigen::Vector3d spring_force = direct_of_force * -(edge->length()-norm) * k;
					Eigen::Vector3d damper_force = direct_of_force.dot(p1.velocity_ -p2.velocity_)*c*direct_of_force;
					Eigen::Vector3d tension = spring_force + damper_force;
					return tension;
				};

				ph_engine_.add_constraint_pair_as_both_directions_consist_of(edge->from()->particle_,edge->to()->particle_,func_neural_edge);
			}
			
			ph_engine_.update();
		}

		public:
		
		void update(){
			for (int i = 0; i < 10; i++) {
				// for (int j = 0; j < 10; j++) {
				update_neuron();
				// }
				update_physics();
			}
		};

		void draw()const{
			for (auto&& edge : edges_) {
				edge->draw();
			}
			
			for (auto&& node : nodes_) {
				node->draw();
			}
		}
	};
} // namespace amima

class ofApp : public ofBaseApp{
	ofxUISuperCanvas *gui;
    ofxUIMovingGraph *mg;
	public:
		std::vector<float> logs_;
		amima::Config config_;
		amima::World world_;

		ofEasyCam camera_;

		ofApp():
			config_(),
			world_(config_)
	{}
		virtual ~ofApp(){}
		void setup_gui(){
			gui = new ofxUISuperCanvas("amima");
			gui->addSpacer();
			for (int i = 0; i < 256; i++) {
				logs_.push_back(0.0);
			}
			gui->addLabel("action potential");
			mg = gui->addMovingGraph("FitzHughNagumoModel", logs_, 256, -3.0, 3.0);
			ofAddListener(gui->newGUIEvent,this,&ofApp::guiEvent);
		};

		void setup_graphic(){
			ofBackground(16);
			ofSetLineWidth(4);
		}

		void setup_world(){
			world_.setup();
			std::shared_ptr<amima::NeuralNode> node1 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node2 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node3 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node4 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node5 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node6 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node7 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node8 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node9 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node10 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node11 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node12 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node13 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node14 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node15 = world_.add_node(amima::Type::Neuron);
			world_.add_edge(amima::Type::Neuron, 20.0, node1, node2);
			world_.add_edge(amima::Type::Neuron, 20.0, node2, node3);
			world_.add_edge(amima::Type::Neuron, 20.0, node3, node4);
			world_.add_edge(amima::Type::Neuron, 20.0, node4, node5);
			world_.add_edge(amima::Type::Neuron, 20.0, node5, node6);
			world_.add_edge(amima::Type::Neuron, 20.0, node6, node7);
			world_.add_edge(amima::Type::Neuron, 20.0, node7, node8);
			world_.add_edge(amima::Type::Neuron, 20.0, node8, node9);
			world_.add_edge(amima::Type::Neuron, 20.0, node9, node10);
			world_.add_edge(amima::Type::Neuron, 20.0, node10, node11);
			world_.add_edge(amima::Type::Neuron, 20.0, node11, node12);
			world_.add_edge(amima::Type::Neuron, 20.0, node12, node13);
			world_.add_edge(amima::Type::Neuron, 20.0, node13, node14);
			world_.add_edge(amima::Type::Neuron, 20.0, node14, node15);
			world_.add_edge(amima::Type::Neuron, 20.0, node15, node2);
			// node2->particle_.b_static_ = true;
			node1->is_static_ = true;
			
			std::shared_ptr<amima::NeuralNode> node16 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node17 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node18 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node19 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node20 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node21 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node22 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node23 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node24 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node25 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node26 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node27 = world_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::NeuralNode> node28 = world_.add_node(amima::Type::Neuron);
			world_.add_edge(amima::Type::Neuron, 20.0, node4, node17);
			world_.add_edge(amima::Type::Neuron, 20.0, node17, node18);
			world_.add_edge(amima::Type::Neuron, 20.0, node6, node19);
			world_.add_edge(amima::Type::Neuron, 20.0, node19, node20);
			world_.add_edge(amima::Type::Neuron, 20.0, node8, node21);
			world_.add_edge(amima::Type::Neuron, 20.0, node21, node22);
			world_.add_edge(amima::Type::Neuron, 20.0, node10, node23);
			world_.add_edge(amima::Type::Neuron, 20.0, node23, node24);
			world_.add_edge(amima::Type::Neuron, 20.0, node25, node12);
			world_.add_edge(amima::Type::Neuron, 20.0, node26, node25);
			world_.add_edge(amima::Type::Neuron, 20.0, node14, node27);
			world_.add_edge(amima::Type::Neuron, 20.0, node27, node28);
			world_.add_edge(amima::Type::Neuron, 20.0, node18, node25);
		}

		void setup(){
			setup_gui();
			setup_world();
			setup_graphic();
		};

		void update(){
			mg->addPoint(world_.nodes_[1]->activation_function_.v_);
			logs_[255] = world_.nodes_[1]->activation_function_.v_;
			for(int i = 1; i < 256; i++) {
				logs_[i-1] = logs_[i];
			}
			world_.update();
		};

		void draw(){
			double target_x =world_.nodes_[1]->particle_.position_[0];
			double target_y =world_.nodes_[1]->particle_.position_[1];
			double target_z =world_.nodes_[1]->particle_.position_[2];
			// ofNode target(target_x, target_y, target_z);
			// camera_.setTarget(ofVec3f(target_x, target_y, target_z));
			// camera_.setAutoDistance(true);
			// camera_.setDistance(100);
			camera_.begin();
			// ofDrawGrid(10,10,10);
			world_.draw();
			camera_.end();
			double x = world_.nodes_[1]->activation_function_.v_;
			double y = world_.nodes_[1]->activation_function_.w_;
		};

		void keyPressed(int key){};
		void keyReleased(int key){};
		void mouseMoved(int x, int y ){
			world_.nodes_[0]->sum_ = ofMap(x,0,ofGetWidth(),-10.0,10.0);
		};
		void mouseDragged(int x, int y, int button){};
		void mousePressed(int x, int y, int button){};
		void mouseReleased(int x, int y, int button){};
		void windowResized(int w, int h){};
		void dragEvent(ofDragInfo dragInfo){};
		void gotMessage(ofMessage msg){};
		void guiEvent(ofxUIEventArgs &e){};
		
		void exit(){
			delete gui;
		}
};


