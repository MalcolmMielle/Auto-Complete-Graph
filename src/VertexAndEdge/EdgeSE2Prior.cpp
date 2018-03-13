#include "auto_complete_graph/VertexAndEdge/EdgeSE2Prior.hpp"
#include "auto_complete_graph/VertexAndEdge/VertexSE2Prior.hpp"

Eigen::Vector2d g2o::EdgeSE2Prior_malcolm::getOrientation2D(const g2o::VertexSE2Prior& from) const
{
// 	std::cout << "From " << from.estimate().toVector(); 
	g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>(this->vertices()[0]);
	g2o::VertexSE2Prior* ptr2 = dynamic_cast<g2o::VertexSE2Prior*>(this->vertices()[1]);
	assert(ptr != NULL);
	assert(ptr2 != NULL);
	assert(ptr == &from || ptr2 == &from);
	Eigen::Vector3d from_1;
	Eigen::Vector3d toward;
	if(ptr == &from){
		from_1 = ptr->estimate().toVector();
		toward = ptr2->estimate().toVector();
	}
	else if(ptr2 == &from){
		from_1 = ptr2->estimate().toVector();
		toward = ptr->estimate().toVector();
	}
	else{
		assert(true == false && "Weird, the original vertex wasn't found before");
	}
	
// 	std::cout << " toward " << toward << std::endl;
	
	Eigen::Vector3d pose_from1 = toward - from_1;
	Eigen::Vector2d pose_prior; pose_prior << pose_from1(0), pose_from1(1);
	return pose_prior;
	
}