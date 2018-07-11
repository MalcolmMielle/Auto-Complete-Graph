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

#include "auto_complete_graph/VertexAndEdge/EdgeNDTCell.hpp"

//#ifdef G2O_HAVE_OPENGL
//#include "g2o/stuff/opengl_wrapper.h"
//#include "g2o/stuff/opengl_primitives.h"
//#endif

namespace g2o {

	EdgeNDTCell::EdgeNDTCell(g2o::EdgeXYPriorACG* edge_prior) : _prior_edge_collinear(edge_prior), EdgePointXYACG() //BaseBinaryEdge<1, double, VertexNDTCell, VertexPointXYACG>()
	{
//		Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
//		Eigen::Matrix2d information_prior = cov.inverse();
//		setInformation(information_prior);
//		_information.setIdentity();
//		_error.setZero();
	}

//	bool EdgeNDTCell::read(std::istream& is)
//	{
//		throw(std::runtime_error("NOy implemented"));
////		Vector2 p;
////		is >> p[0] >> p[1];
////		setMeasurement(p);
////		for (int i = 0; i < 2; ++i)
////			for (int j = i; j < 2; ++j) {
////				is >> information()(i, j);
////				if (i != j)
////					information()(j, i) = information()(i, j);
////			}
////		return true;
//	}
//
//	bool EdgeNDTCell::write(std::ostream& os) const
//	{
//		throw(std::runtime_error("NOy implemented"));
////		Vector2 p = measurement();
////		os << p.x() << " " << p.y();
////		for (int i = 0; i < 2; ++i)
////			for (int j = i; j < 2; ++j)
////				os << " " << information()(i, j);
////		return os.good();
//	}


//#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
//	void EdgeNDTCell::linearizeOplus()
//	{
//		_jacobianOplusXi=-Eigen::Matrix<double, 1, Di, Eigen::RowMajor>::AlignedMapType::Identity();
//		_jacobianOplusXj= Eigen::Matrix<double, 1, Dj, Eigen::RowMajor>::AlignedMapType::Identity();
//	}
//#endif


} // end namespace
