#ifndef AUTOCOMPLETEGRAPH_EDGENDTCELL_02072018
#define AUTOCOMPLETEGRAPH_EDGENDTCELL_02072018

#include "EdgePointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeXYPrior.hpp"
#include "VertexXYPrior.hpp"
#include "VertexNDTCell.hpp"

#include "auto_complete_graph/utils.hpp"

namespace  g2o {

	class VertexXYPrior;
}


namespace g2o{


	class EdgeNDTCell : public EdgePointXYACG // BaseBinaryEdge<1, double, VertexNDTCell, VertexPointXYACG>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		AASS::acg::EdgeInterfaceMalcolm interface;
		EdgeNDTCell(g2o::EdgeXYPriorACG* edge_prior);

		g2o::EdgeXYPriorACG* _prior_edge_collinear = NULL;
		double resolution_of_map;

		//Compute the distance to a certain edge
		virtual void computeError()
		{


//			const VertexNDTCell* v1 = static_cast<const VertexNDTCell*>(_vertices[0]);
//			const VertexPointXYACG* v2 = static_cast<const VertexPointXYACG*>(_vertices[1]);
//			_error = (v2->estimate()-v1->estimate())-_measurement;

//			std::cout << "USING THE NDT CELL ERROR FUNCTION ! -> " << (v2->estimate()-v1->estimate())-_measurement << std::endl;
//
//			assert(_prior_edge_collinear != NULL);
//
			VertexNDTCell* ptr_ndt_cell = dynamic_cast<g2o::VertexNDTCell*>(this->vertices()[0]);
			assert(ptr_ndt_cell != NULL);

			Eigen::Vector2d C = ptr_ndt_cell->estimate();
			Eigen::Vector2d A = dynamic_cast<g2o::VertexXYPrior*>(_prior_edge_collinear->vertices()[0])->estimate().head(2);
			Eigen::Vector2d B = dynamic_cast<g2o::VertexXYPrior*>(_prior_edge_collinear->vertices()[1])->estimate().head(2);

			Eigen::Matrix<double, 2, 1, Eigen::ColMajor> error;

//			Eigen::Vector2d AB = B - A;
//			Eigen::Vector2d AC = C - A;
//			Eigen::Vector2d BC = C - B;
//
			Eigen::Vector2d CCp = Eigen::Vector2d::Zero();
//			//Check if closest segment point is on the line segment of the wall
////			First, check to see if the nearest point on the line AB is beyond B (as in the example above) by taking AB ⋅ BC. If this value is greater than 0, it means that the angle between AB and BC is between -90 and 90, exclusive, and therefore the nearest point on the segment AB will be B
//			if(AB.dot(BC) >= 0){
//				CCp = -BC;
//			}else if( (-AB).dot(AC) >= 0 ){
//				CCp = -AC;
//			}else {
//				//Vector to line
//				double a1 = AC.dot(AB / AB.norm());
//				Eigen::Vector2d ACp = a1 * (AB / AB.norm());
//				Eigen::Vector2d CCp = -AC + ACp;
//			}
//
//			std::tie (std::ignore, CCp) = AASS::acg::distancePointSegment(C, A, B);
			std::tie (std::ignore, CCp) = AASS::acg::distancePointLine2(C, A, B);


			//If less than the size of the cells than ignore the error !!!!
//			if(CCp.norm() < resolution_of_map){
//				CCp = Eigen::Vector2d::Zero();
//			}



//			error << AASS::acg::distancePointLine(C, A, B) - _measurement;
			error << CCp(0), CCp(1);
			_error = error;

//			std::cout << "USING THE NDT CELL ERROR FUNCTION ! -> " << error << std::endl;

		}

		virtual bool setMeasurementFromState() {
			_measurement << 0 , 0;
			return true;
		}

		g2o::EdgeXYPriorACG* getPriorWall(){return _prior_edge_collinear;}
		const g2o::EdgeXYPriorACG* getPriorWall() const {return _prior_edge_collinear;}

//		virtual bool read(std::istream& is);
//		virtual bool write(std::ostream& os) const;
//
//		virtual void setMeasurement(double m){
//			_measurement = m;
//		}
//
//		virtual bool setMeasurementData(const number_t* d){
//			_measurement = d[0];
//			return true;
//		}
//
//		virtual bool getMeasurementData(number_t* d) const {
////			Eigen::Map<Vector2> m(d);
//			*d = _measurement;
//			return true;
//		}
//
//		virtual int measurementDimension() const {return 1;}
//
//		virtual bool setMeasurementFromState() {
//			_measurement = 0;
////			const VertexPointXYACG* v1 = static_cast<const VertexPointXYACG*>(_vertices[0]);
////			const VertexPointXYACG* v2 = static_cast<const VertexPointXYACG*>(_vertices[1]);
////			_measurement = v2->estimate()-v1->estimate();
////			return false;
//		}


		virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 0.;}
//#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
//		virtual void linearizeOplus();
//#endif



	};


}
#endif
