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

//
//void g2o::EdgeXYPriorACG::computeError()
//{
////	std::cout << "Compute error XY Prior" << std::endl;
////	_error = 0;
////	if(use_distance_error) {
//		const VertexPointXYACG *v1 = static_cast<const VertexPointXYACG *>(_vertices[0]);
//		const VertexPointXYACG *v2 = static_cast<const VertexPointXYACG *>(_vertices[1]);
//		_error = _error + (v2->estimate() - v1->estimate()) - _measurement;
////	}
//
////	if(use_angle_error) {
////		double angle_error = 0;
////		for (auto vertex : vertices()) {
////			g2o::VertexXYPrior *ptr = dynamic_cast<g2o::VertexXYPrior *>(vertex);
////			angle_error += ptr->angleDiff(this);
////		}
////		_error = _error + (360 * angle_error / 2 * M_PI);
////	}
//
//
//
//}