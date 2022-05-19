#ifndef AUTOCOMPLETEGRAPH_EDGENDTCELL_02072018
#define AUTOCOMPLETEGRAPH_EDGENDTCELL_02072018

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgePointXYACG.hpp"
#include "EdgeXYPrior.hpp"
#include "VertexNDTCell.hpp"
#include "VertexXYPrior.hpp"

#include "auto_complete_graph/utils.hpp"

namespace g2o {

class VertexXYPrior;
}

namespace g2o {

class EdgeNDTCell : public EdgePointXYACG {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AASS::acg::EdgeInterfaceMalcolm interface;
    EdgeNDTCell(g2o::EdgeXYPriorACG* edge_prior);

    g2o::EdgeXYPriorACG* _prior_edge_collinear = NULL;
    double resolution_of_map;

    // Compute the distance to a certain edge
    virtual void computeError() {
        VertexNDTCell* ptr_ndt_cell =
            dynamic_cast<g2o::VertexNDTCell*>(this->vertices()[0]);
        assert(ptr_ndt_cell != NULL);

        Eigen::Vector2d C = ptr_ndt_cell->estimate();
        Eigen::Vector2d A = dynamic_cast<g2o::VertexXYPrior*>(
                                _prior_edge_collinear->vertices()[0])
                                ->estimate()
                                .head(2);
        Eigen::Vector2d B = dynamic_cast<g2o::VertexXYPrior*>(
                                _prior_edge_collinear->vertices()[1])
                                ->estimate()
                                .head(2);

        Eigen::Matrix<double, 2, 1, Eigen::ColMajor> error;

        Eigen::Vector2d CCp = Eigen::Vector2d::Zero();
        //			Check if closest segment point is on the line
        // segment of the wall 			First, check to see if the
        // nearest point on the line AB is beyond B (as in the example above) by
        // taking AB ⋅ BC. If this value is greater than 0, it means that the
        // angle between AB and BC is between -90 and 90, exclusive, and
        // therefore the nearest point on the segment AB will be B
        std::tie(std::ignore, CCp) = AASS::acg::distancePointLine2(C, A, B);
        error << CCp(0), CCp(1);
        _error = error;
    }

    virtual bool setMeasurementFromState() {
        _measurement << 0, 0;
        return true;
    }

    g2o::EdgeXYPriorACG* getPriorWall() { return _prior_edge_collinear; }
    const g2o::EdgeXYPriorACG* getPriorWall() const {
        return _prior_edge_collinear;
    }

    virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                             OptimizableGraph::Vertex*) {
        return 0.;
    }
};

}  // namespace g2o
#endif
