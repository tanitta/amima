#pragma once

#include "ofMain.h"

#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>


namespace amima {
	class Config {
		public:
			ofColor neural_node_;
			
			Config(){
				neural_node_.setHsb(0,0,255);
			};
			virtual ~Config(){};
			
	};
	class BaseEntity {
		protected:
			Config color_config_;
		public:
			BaseEntity(){};
			virtual ~BaseEntity(){};
	};
	
	class BaseNode;
	class BaseEdge :public BaseEntity{
		private:
			std::shared_ptr<amima::BaseNode> from_;
			std::shared_ptr<amima::BaseNode> to_;
			
		public:
			BaseEdge(std::shared_ptr<amima::BaseNode> from, std::shared_ptr<amima::BaseNode> to):
				from_(from),
				to_(to)
		{};
			
			virtual ~BaseEdge(){};
			virtual void draw();
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
				position_(0,0,0),
				velocity_(0,0,0),
				acceleration_(0,0,0)
		{};
			virtual ~BaseNode(){};
			
			void set_mass(){};
			void set_position(){};
			void set_velocity(){};
			void set_acceleration(){};
			
			double mass()const{return mass_;};
			Eigen::Vector3d position()const{return position_;};
			Eigen::Vector3d velocity()const{return velocity_;};
			Eigen::Vector3d acceleration()const{return acceleration_;};
			
			void update_position(double unit_time){
				velocity_ += acceleration_ * unit_time;
				position_ += velocity_ * unit_time;
			};
			virtual void draw(){};
	};
	
	class NeuralNode;
	class NeuralEdge : public amima::BaseEdge{
		private:
			double weight_;
		public:
			NeuralEdge(std::shared_ptr<amima::BaseNode> from, std::shared_ptr<amima::BaseNode> to):BaseEdge(from, to)
		{};
			
			virtual ~NeuralEdge(){};
			
			double weight()const{
				return weight_;
			};
			
			void adjust_weight(double diff){
				weight_ += diff;
			};
			
	};
	
	class NeuralNode : public amima::BaseNode{
		private:
			std::vector<std::shared_ptr<amima::NeuralEdge>> connections_;
			
		public:
			NeuralNode(){};
			
			virtual ~NeuralNode(){};
			
			void draw()override{
				std::cout<<"Draw"<<std::endl;
				ofSetColor(color_config_.neural_node_);
				ofDrawSphere(position()[0],position()[1],position()[2],mass());
			}
			
	};
	enum class Type {
			Neuron,
			Mascle
	};
	class Creature {
		std::vector<std::shared_ptr<BaseNode>> nodes_;
		std::vector<std::shared_ptr<BaseEdge>> edges_;
		public:
			Creature(){};
			virtual ~Creature(){};
			
			void add_node(Type type){
				if (type == Type::Neuron) {
					std::shared_ptr<NeuralNode> neural_node(new NeuralNode);
					nodes_.push_back(neural_node);
				}
				if (type == Type::Mascle) {
				}
				std::cout<<nodes_.size()<<std::endl;
			}
			
			void add_edge();
			void draw(){
				for (auto&& node : nodes_) {
					node->draw();
				}
			}
	};
	
} // namespace amima

class ofApp : public ofBaseApp{

	public:
		ofEasyCam camera_;
		amima::Creature creature_;
		void setup(){
			creature_.add_node(amima::Type::Neuron);
		};
		void update(){};
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
