#pragma once
#include "base_node.hpp"

namespace amima {
	class BaseEdge {
		private:
			std::shared_ptr<amima::BaseNode> from_;
			std::shared_ptr<amima::BaseNode> to_;
			
		public:
			BaseEdge(std::shared_ptr<amima::BaseNode> from, std::shared_ptr<amima::BaseNode> to):
				from_(from),
				to_(to)
		{};
			
			virtual ~BaseEdge(){};
	};
} // namespace amima
