#pragma once
#ifdef Success
#undef Success
#endif
#include "Eigen/Core"
namespace amima {
	class BaseNode {
		private:
			double mass_;
			Eigen::Vector3d position_;
			Eigen::Vector3d velocity_;
			Eigen::Vector3d acceleration_;
		public:
			BaseNode(){};
			virtual ~BaseNode(){};
			
			void set_mass(){};
			void set_position(){};
			void set_velocity(){};
			void set_acceleration(){};
			
			double mass()const{};
			Eigen::Vector3d position()const{};
			Eigen::Vector3d velocity()const{};
			Eigen::Vector3d acceleration()const{};
			
			void update_position(double unit_time){
				velocity_ += acceleration_ * unit_time;
				position_ += velocity_ * unit_time;
			};
	};
} // namespace amima
