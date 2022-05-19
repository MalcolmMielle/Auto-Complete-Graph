#ifndef AUTOCOMPLETEGRAPH_EDGEPRIOROBSERVATION_01052018
#define AUTOCOMPLETEGRAPH_EDGEPRIOROBSERVATION_01052018

#include "EdgeLandmark.hpp"

namespace g2o {

class EdgePriorObservation : public g2o::EdgeLandmark_malcolm {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLandmark_malcolm* equivalent_landmark_observation_edge = NULL;
    EdgePriorObservation(EdgeLandmark_malcolm* equivalent_landmark_observation)
        : equivalent_landmark_observation_edge(equivalent_landmark_observation),
          g2o::EdgeLandmark_malcolm(){};
};

}  // namespace g2o
#endif