#include "auto_complete_graph/VertexAndEdge/EdgeXYPrior.hpp"
#include "auto_complete_graph/VertexAndEdge/VertexXYPrior.hpp"

Eigen::Vector2d g2o::EdgeXYPriorACG::getOrientation2D(const g2o::VertexXYPrior& from) const
{
// 	std::cout << "From " << from.estimate().toVector();
	g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>(this->vertices()[0]);
	g2o::VertexXYPrior* ptr2 = dynamic_cast<g2o::VertexXYPrior*>(this->vertices()[1]);
	assert(ptr != NULL);
	assert(ptr2 != NULL);
	assert(ptr == &from || ptr2 == &from);
	Eigen::Vector2d from_1;
	Eigen::Vector2d toward;
	if(ptr == &from){
		from_1 = ptr->estimate();
		toward = ptr2->estimate();
	}
	else if(ptr2 == &from){
		from_1 = ptr2->estimate();
		toward = ptr->estimate();
	}
	else{
		assert(true == false && "Weird, the original vertex wasn't found before");
	}

// 	std::cout << " toward " << toward << std::endl;
//	Eigen::Vector2d pose_from1 = toward - from_1;
//	Eigen::Vector2d pose_prior; pose_prior << pose_from1(0), pose_from1(1);
	return toward - from_1;

}