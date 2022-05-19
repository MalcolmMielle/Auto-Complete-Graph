#ifndef AUTOCOMPLETEGRAPH_GRAPHELEMENT_30042018
#define AUTOCOMPLETEGRAPH_GRAPHELEMENT_30042018

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ctime>
#include <fstream>
#include <random>
#include <set>
#include <vector>
#include "Eigen/Core"
#include "PriorLoaderInterface.hpp"
#include "covariance.hpp"
#include "g2o/types/slam2d/parameter_se2_offset.h"

namespace AASS {

namespace acg {

/**
 * @brief The graph class containing all elements from the prior. Needed for the
 * templated version of ACGLocalization :(.
 */
template <typename VERTEXTYPE, typename EDGETYPE>
class GraphElements {
   protected:
    ///@brief vector storing all node from the prior
    std::set<VERTEXTYPE*> _nodes;
    ///@brief vector storing all edge between the prior nodes
    std::set<EDGETYPE*> _edges;
    g2o::ParameterSE2Offset* _sensorOffset;

   public:
    GraphElements(const g2o::SE2& sensoffset) {
        _sensorOffset = new g2o::ParameterSE2Offset;
        _sensorOffset->setOffset(sensoffset);
        _sensorOffset->setId(0);
    }

    /** Accessor**/
    typename std::set<VERTEXTYPE*>& getNodes() { return _nodes; }
    const typename std::set<VERTEXTYPE*>& getNodes() const { return _nodes; }
    ///@brief vector storing all edge between the prior nodes
    typename std::set<EDGETYPE*>& getEdges() { return _edges; }
    const typename std::set<EDGETYPE*>& getEdges() const { return _edges; }

    virtual VERTEXTYPE* addPose(const g2o::SE2& se2,
                                const PriorAttr& priorAttr,
                                int index) = 0;
    virtual VERTEXTYPE* addPose(const Eigen::Vector3d& lan,
                                const PriorAttr& priorAttr,
                                int index) = 0;
    virtual VERTEXTYPE* addPose(double x,
                                double y,
                                double theta,
                                const PriorAttr& priorAttr,
                                int index) = 0;

    virtual EDGETYPE* addEdge(const g2o::SE2& se2,
                              g2o::HyperGraph::Vertex* v1,
                              g2o::HyperGraph::Vertex* v2) = 0;

    virtual void clear() {
        _nodes.clear();
        _edges.clear();
    }

    bool removeVertex(g2o::HyperGraph::Vertex* v1) {
        if (v1 != NULL) {
            auto it = _nodes.find(v1);

            if (it != _nodes.end()) {
                _nodes.erase(it);
            } else {
                return false;
            }
        }
        return true;
    }
};
}  // namespace acg
}  // namespace AASS

#endif