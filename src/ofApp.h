#pragma once

#include "ofMain.h"

#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>


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
				neural_edge_damper_ = 0.1;
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
	};
	
	class BaseNode :public BaseEntity{
		private:
			double mass_;
			Eigen::Vector3d position_;
			Eigen::Vector3d velocity_;
			Eigen::Vector3d acceleration_;
		public:
			BaseNode():
				mass_(1.0),
				// position_(0,0,0),
				position_(ofRandom(0,1.0),ofRandom(0,1.0),ofRandom(0,1.0)),
				velocity_(0,0,0),
				acceleration_(0,0,0)
		{};
			virtual ~BaseNode(){};
			
			void set_mass(double mass){mass_ = mass;};
			void set_position(Eigen::Vector3d position){position_ = position;};
			void set_velocity(Eigen::Vector3d velocity){velocity_ = velocity;};
			void set_acceleration(Eigen::Vector3d acceleration){acceleration_ = acceleration;};
			
			double mass()const{return mass_;};
			Eigen::Vector3d position()const{return position_;};
			Eigen::Vector3d velocity()const{return velocity_;};
			Eigen::Vector3d acceleration()const{return acceleration_;};
			
			
			void add_force(Eigen::Vector3d force){
				acceleration_ += force / mass_;
			};
			
			void update_position(double unit_time){
				add_force(-velocity_*config_.viscosity_coefficient_);
				velocity_ += acceleration_ * unit_time;
				position_ += velocity_ * unit_time;
				acceleration_ << 0,0,0;
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
				// Eigen::Vector3d damper_force = direct_of_force * (length()-norm) * k;
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
			
			// std::shared_ptr<BaseNode> add_node_random_position_from(Type type, std::shared_ptr<BaseNode> from){
			// 	std::shared_ptr<BaseNode> new_node = add_node_from(type, from);
			// 	new_node->set_position();
			// }
			
			void update(){
				for (auto&& edge : edges_) {
					edge->update_tension();
				}
				
				for (auto&& node : nodes_) {
					node->update_position(config_.unit_time_);
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
	};
	
} // namespace amima

class ofApp : public ofBaseApp{

	public:
		ofEasyCam camera_;
		amima::Creature creature_;
		void setup_graphic(){
			ofBackground(32);
		}
		
		void setup(){
			std::shared_ptr<amima::BaseNode> node1 = creature_.add_node(amima::Type::Neuron);
			std::shared_ptr<amima::BaseNode> node2 = creature_.add_node(amima::Type::Neuron);
			creature_.add_edge(amima::Type::Neuron, 20.0, node1, node2);
			;
			creature_.add_node_from(
				amima::Type::Neuron, 20.0, 
				creature_.add_node_from(
					amima::Type::Neuron, 20.0, 
					creature_.add_node_from(
						amima::Type::Neuron, 20.0, 
						node2
					)
				)
			);
			
			setup_graphic();
		};
		void update(){
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
