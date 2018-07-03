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


	class EdgeNDTCell : public BaseBinaryEdge<1, double, VertexPointXYACG, VertexNDTCell>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		AASS::acg::EdgeInterfaceMalcolm interface;
		EdgeNDTCell(g2o::EdgeXYPriorACG* edge_prior);

		g2o::EdgeXYPriorACG* _prior_edge_collinear = NULL;

		//Compute the distance to a certain edge
		virtual void computeError()
		{
			std::cout << "USING THE NDT CELL ERROR FUNCTION ! " << std::endl;

			assert(_prior_edge_collinear != NULL);

			Eigen::Vector2d p0 = dynamic_cast<g2o::VertexNDTCell*>(this->vertices()[0])->estimate();
			Eigen::Vector2d p1 = dynamic_cast<g2o::VertexXYPrior*>(_prior_edge_collinear->vertices()[0])->estimate().head(2);
			Eigen::Vector2d p2 = dynamic_cast<g2o::VertexXYPrior*>(_prior_edge_collinear->vertices()[1])->estimate().head(2);

			Eigen::Matrix<double, 1, 1, Eigen::ColMajor> error;
			error << AASS::acg::distancePointLine(p0, p1, p2);
			_error = error;

//			const VertexPointXYACG* v1 = static_cast<const VertexPointXYACG*>(_vertices[0]);
//			const VertexPointXYACG* v2 = static_cast<const VertexPointXYACG*>(_vertices[1]);
//			_error = (v2->estimate()-v1->estimate())-_measurement;

//			_error = distance_point_to_line;
		}

		g2o::EdgeXYPriorACG* getPriorWall(){return _prior_edge_collinear;}
		const g2o::EdgeXYPriorACG* getPriorWall() const {return _prior_edge_collinear;}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(double m){
			_measurement = m;
		}

		virtual bool setMeasurementData(const number_t* d){
			_measurement = d[0];
			return true;
		}

		virtual bool getMeasurementData(number_t* d) const {
//			Eigen::Map<Vector2> m(d);
//			d = _measurement;
			return true;
		}

		virtual int measurementDimension() const {return 1;}

		virtual bool setMeasurementFromState() {
//			const VertexPointXYACG* v1 = static_cast<const VertexPointXYACG*>(_vertices[0]);
//			const VertexPointXYACG* v2 = static_cast<const VertexPointXYACG*>(_vertices[1]);
//			_measurement = v2->estimate()-v1->estimate();
			return false;
		}


		virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 0.;}
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
		virtual void linearizeOplus();
#endif



	};


}
#endif