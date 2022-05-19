#ifndef AUTOCOMPLETEGRAPH_ACGPRIOR_13042018
#define AUTOCOMPLETEGRAPH_ACGPRIOR_13042018

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

#include "GraphElements.hpp"

namespace AASS {

namespace acg {

/**
 * @brief The graph class containing all elements from the prior. Needed for the
 * templated version of ACGLocalization :(.
 */
template <typename VERTEXTYPE, typename EDGETYPE>
class AutoCompleteGraphPrior : public GraphElements<VERTEXTYPE, EDGETYPE> {
   protected:
    Eigen::Vector2d _priorNoise;
    double _prior_rot;

    bool _use_user_prior_cov = false;

   public:
    AutoCompleteGraphPrior(const Eigen::Vector2d& pn,
                           double rp,
                           const g2o::SE2& sensoffset)
        : _priorNoise(pn),
          _prior_rot(rp),
          GraphElements<VERTEXTYPE, EDGETYPE>(sensoffset){};
    AutoCompleteGraphPrior(const g2o::SE2& sensoffset)
        : GraphElements<VERTEXTYPE, EDGETYPE>(sensoffset){};

    void setPriorNoise(double a, double b) { _priorNoise << a, b; }
    void setPriorRot(double r) { _prior_rot = r; }

    void useUserCovForPrior(bool u) { _use_user_prior_cov = u; }
    bool isUsingUserCovForPrior() const { return _use_user_prior_cov; }

    // FUNTION TO ADD A PRIOR GRAPH INTO THE GRAPH
    /**
     * @brief Directly use the prior graph to init the prior part of the ACG
     *
     */
    virtual int addPriorGraph(const PriorLoaderInterface::PriorGraph& graph,
                              int first_index) = 0;
    virtual void checkNoRepeatingPriorEdge() {
        for (auto edge : this->_edges) {
            auto vertex = edge->vertices()[0];
            auto vertex2 = edge->vertices()[1];
            for (auto edge_second : this->_edges) {
                if (edge != edge_second) {
                    auto vertex_second = edge_second->vertices()[0];
                    auto vertex_second2 = edge_second->vertices()[1];

                    if (vertex_second == vertex && vertex_second2 == vertex2) {
                        throw std::runtime_error("Same prior edge");
                    } else if (vertex_second == vertex2 &&
                               vertex_second2 == vertex) {
                        throw std::runtime_error("Same prior edge");
                    }
                }
            }
        }
    }

    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr
    toPointCloud(double resolution, double z_elevation, double varz) const = 0;
};
}  // namespace acg
}  // namespace AASS

#endif