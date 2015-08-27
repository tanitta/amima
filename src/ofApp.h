#pragma once

#ifdef Success
#undef Success
#endif
#include <Eigen/Core>
#include <pharticle/pharticle.hpp>
#include "ofMain.h"

namespace amima {
	class Config {
		public:
			double unit_time_;

			double viscosity_coefficient_;

			ofColor neural_node_;
			ofColor neural_edge_;
			double neural_edge_damper_;
			double neural_edge_spring_;

			Config(){
				unit_time_ = 1.0/30.0;
				viscosity_coefficient_ = 0.05;
				double h = ofMap(20,0,260,0,255);
				neural_node_.setHsb(h,255,255);
				neural_edge_.setHsb(h,255,255);
				neural_edge_spring_ = 0.1;
				neural_edge_damper_ = 0.2;
			};
			virtual ~Config(){};

	};
	class BaseEntity {
		protected:
			Config config_;
		public:
			BaseEntity(){};
			virtual ~BaseEntity(){};
	};

	class BaseNode;
	class BaseEdge :public BaseEntity{
		protected:
			std::shared_ptr<amima::BaseNode> from_;
			std::shared_ptr<amima::BaseNode> to_;
			double length_;
			std::function<Eigen::Vector3d(pharticle::Particle&, pharticle::Particle&)> tension_function;

		public:
			BaseEdge(std::shared_ptr<amima::BaseNode> from, std::shared_ptr<amima::BaseNode> to, double length):
				from_(from),
				to_(to),
				length_(length)
		{};

			std::shared_ptr<amima::BaseNode> from(){
				return from_;
			};
			std::shared_ptr<amima::BaseNode> to(){
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

	class BaseNode :public BaseEntity{
		private:
			pharticle::Particle particle_;
		public:
			BaseNode():
				particle_()
				// mass_(1.0),
				// // position_(0,0,0),
				// position_(ofRandom(0,1.0),ofRandom(0,1.0),ofRandom(0,1.0)),
				// velocity_(0,0,0),
				// acceleration_(0,0,0)
		{
			particle_.position_ = (Eigen::Vector3d(ofRandom(0,1.0),ofRandom(0,1.0),ofRandom(0,1.0)));
			particle_.radius_ = 2.0;
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

	class NeuralNode;
	class NeuralEdge : public amima::BaseEdge{
		private:
			double weight_;
		public:
			NeuralEdge(std::shared_ptr<amima::BaseNode> from, std::shared_ptr<amima::BaseNode> to, double length):BaseEdge(from, to, length)
		{};

			virtual ~NeuralEdge(){};

			double weight()const{
				return weight_;
			};

			void adjust_weight(double diff){
				weight_ += diff;
			};

			void draw()const override{
				ofSetColor(config_.neural_edge_);
				ofPoint from_position(from_->position()[0],from_->position()[1],from_->position()[2]);
				ofPoint to_position(to_->position()[0],to_->position()[1],to_->position()[2]);
				ofLine(from_position, to_position);
			}

			void update_tension(){
				double k = config_.neural_edge_spring_;
				double c = config_.neural_edge_damper_;
				Eigen::Vector3d direct_of_force = (to_->position() - from_->position()).normalized();
				double norm = (to_->position() - from_->position()).norm();
				Eigen::Vector3d spring_force = direct_of_force * (length()-norm) * k;
				Eigen::Vector3d tension = spring_force;

				from_->add_force(-tension);
				to_->add_force(tension);
			};
	};

	class NeuralNode : public amima::BaseNode{
		private:
			std::vector<std::shared_ptr<amima::NeuralEdge>> connections_;

		public:
			NeuralNode(){};

			virtual ~NeuralNode(){};

			void draw()const override{
				ofSetColor(config_.neural_node_);
				ofDrawSphere(position()[0],position()[1],position()[2],2);
			}

	};
	enum class Type {
		Neuron,
		Mascle
	};
	class Creature {
		std::vector<std::shared_ptr<BaseNode>> nodes_;
		std::vector<std::shared_ptr<BaseEdge>> edges_;
		Config config_;
		public:
		Creature(){};
		virtual ~Creature(){};

		std::shared_ptr<BaseNode> add_node(Type type){
			if (type == Type::Neuron) {
				std::shared_ptr<NeuralNode> neural_node(new NeuralNode);
				nodes_.push_back(neural_node);
				return neural_node;
			}
			if (type == Type::Mascle) {
			}
		}

		std::shared_ptr<BaseEdge> add_edge(Type type, double length, std::shared_ptr<BaseNode> from, std::shared_ptr<BaseNode> to){
			if (type == Type::Neuron) {
				std::shared_ptr<NeuralEdge> neural_edge(new NeuralEdge(from, to, length));
				edges_.push_back(neural_edge);
				return neural_edge;
			}

			if (type == Type::Mascle) {
			}

		};

		std::shared_ptr<BaseNode> add_node_from(Type type, double length, std::shared_ptr<BaseNode> from){
			if (type == Type::Neuron) {
				std::shared_ptr<BaseNode> new_neural_node = add_node(type);
				nodes_.push_back(new_neural_node);
				add_edge(type, length, from, new_neural_node);
				return new_neural_node;
			}
			if (type == Type::Mascle) {
			}
		};

		std::shared_ptr<BaseNode> add_node_to(Type type, double length, std::shared_ptr<BaseNode> to){
			if (type == Type::Neuron) {
				std::shared_ptr<BaseNode> new_neural_node = add_node(type);
				nodes_.push_back(new_neural_node);
				add_edge(type, length, new_neural_node, to);
				return new_neural_node;
			}
			if (type == Type::Mascle) {
			}
		};
		
		void set_to(pharticle::Engine& engine_ref){
		}

		void update(){
			for (auto&& edge : edges_) {
				edge->update_tension();
			}

			for (auto&& node : nodes_) {
				// node->update_position(config_.unit_time_);
			}
		};

		void draw()const{
			for (auto&& node : nodes_) {
				node->draw();
			}

			for (auto&& edge : edges_) {
				edge->draw();
			}
		}

		void add_to(pharticle::Engine& engine_ref){
			for (auto&& node : nodes_) {
				std::cout<<"particle_position"<<std::endl;
				std::cout<<node->particle().position_<<std::endl;
				node->add_force(-node->particle().velocity_*0.001);
				engine_ref.add(node->particle());
			}
		}
	};

} // namespace amima

class ofApp : public ofBaseApp{

	public:
		amima::Creature creature_;
		pharticle::Engine ph_engine_;

		ofEasyCam camera_;

		ofApp():ph_engine_(){}
		virtual ~ofApp(){}

		void setup_graphic(){
			ofBackground(32);
		}

		void setup(){
			std::shared_ptr<amima::BaseNode> node1 = creature_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::BaseNode> node2 = creature_.add_node(amima::Type::Neuron);
			creature_.add_edge(amima::Type::Neuron, 20.0, node1, node2);
			;
			auto node3 = creature_.add_node_from(
					amima::Type::Neuron, 20.0, 
					creature_.add_node_from(
						amima::Type::Neuron, 20.0, 
						creature_.add_node_from(
							amima::Type::Neuron, 20.0, 
							node2
							)
						)
					);
			creature_.add_edge(amima::Type::Neuron, 20.0, node3, node1);
			setup_graphic();
			
			ph_engine_.set_collision_reaction_force([](pharticle::Particle& p1, pharticle::Particle& p2){
				Eigen::Vector3d f(0,0,0);
				Eigen::Vector3d p(0,0,0);
				p = (p2.position_ - p1.position_);
				
				double r = p1.radius_ + p2.radius_;
				Eigen::Vector3d v_r = r*p.normalized();
				f = (v_r - p)*0.2*p2.mass_;
				// f +=( - p2.velocity_)*5;
				return f;
			});
		};
		void update(){
			// ph_engine_.add
			creature_.add_to(ph_engine_);
			ph_engine_.update();
			// std::cout<<ph_engine_.collision_pairs().size()<<std::endl;
			creature_.update();
		};
		void draw(){
			camera_.begin();
				creature_.draw();
			camera_.end();
		};

		void keyPressed(int key){};
		void keyReleased(int key){};
		void mouseMoved(int x, int y ){};
		void mouseDragged(int x, int y, int button){};
		void mousePressed(int x, int y, int button){};
		void mouseReleased(int x, int y, int button){};
		void windowResized(int w, int h){};
		void dragEvent(ofDragInfo dragInfo){};
		void gotMessage(ofMessage msg){};

};
