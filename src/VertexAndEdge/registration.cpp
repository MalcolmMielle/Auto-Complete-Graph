// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "auto_complete_graph/VertexAndEdge/registration.hpp"

#include "g2o/core/factory.h"

#include "g2o/stuff/macros.h"

#include <iostream>
//#include <auto_complete_graph/VertexAndEdge/VertexSE2RobotPose.hpp>

namespace g2o {

G2O_REGISTER_TYPE_GROUP(acgvertex);

G2O_REGISTER_TYPE(VERTEX_SE2_ROBOTPOSE, VertexSE2RobotPose);
G2O_REGISTER_TYPE(VERTEX_SE2_PRIOR, VertexSE2Prior);
G2O_REGISTER_TYPE(VERTEX_SE2_LANDMARK, VertexLandmarkNDT);
G2O_REGISTER_TYPE(VERTEX_XY_PRIOR, VertexXYPrior);

G2O_REGISTER_TYPE(EDGE_SE2_PRIOR, EdgeSE2Prior_malcolm);
G2O_REGISTER_TYPE(EDGE_LANDMARK, EdgeLandmark_malcolm);
G2O_REGISTER_TYPE(EDGE_LINK, EdgeLinkXY_malcolm);
G2O_REGISTER_TYPE(EDGE_LOCALIZATION, EdgeLocalization);
G2O_REGISTER_TYPE(EDGE_ODOMETRY, EdgeOdometry_malcolm);
G2O_REGISTER_TYPE(EDGE_XY_PRIOR_ACG, EdgeXYPriorACG);
G2O_REGISTER_TYPE(EDGE_XY_XY, EdgePointXYACG);

//	EdgeXYPrior e;

}  // namespace g2o
