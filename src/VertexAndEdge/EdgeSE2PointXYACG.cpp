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

#include "auto_complete_graph/VertexAndEdge/EdgeSE2PointXYACG.hpp"

namespace g2o {

	EdgeSE2PointXYACG::EdgeSE2PointXYACG() :
			BaseBinaryEdge<2, Vector2, VertexSE2ACG, VertexPointXYACG>()
	{
	}

	bool EdgeSE2PointXYACG::read(std::istream& is)
	{
		is >> _measurement[0] >> _measurement[1];
		is >> information()(0,0) >> information()(0,1) >> information()(1,1);
		information()(1,0) = information()(0,1);
		return true;
	}

	bool EdgeSE2PointXYACG::write(std::ostream& os) const
	{
		os << measurement()[0] << " " << measurement()[1] << " ";
		os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
		return os.good();
	}

	void EdgeSE2PointXYACG::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
	{
		assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2ACG position_in_robot_frame by VertexPointXYACG");

		VertexSE2ACG* vi     = static_cast<VertexSE2ACG*>(_vertices[0]);
		VertexPointXYACG* vj = static_cast<VertexPointXYACG*>(_vertices[1]);
		if (from.count(vi) > 0 && to == vj) {
			vj->setEstimate(vi->estimate() * _measurement);
		}
	}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
	void EdgeSE2PointXYACG::linearizeOplus()
	{
		const VertexSE2ACG* vi     = static_cast<const VertexSE2ACG*>(_vertices[0]);
		const VertexPointXYACG* vj = static_cast<const VertexPointXYACG*>(_vertices[1]);
		const number_t& x1        = vi->estimate().translation()[0];
		const number_t& y1        = vi->estimate().translation()[1];
		const number_t& th1       = vi->estimate().rotation().angle();
		const number_t& x2        = vj->estimate()[0];
		const number_t& y2        = vj->estimate()[1];

		number_t aux_1 = std::cos(th1) ;
		number_t aux_2 = -aux_1 ;
		number_t aux_3 = std::sin(th1) ;

		_jacobianOplusXi( 0 , 0 ) = aux_2 ;
		_jacobianOplusXi( 0 , 1 ) = -aux_3 ;
		_jacobianOplusXi( 0 , 2 ) = aux_1*y2-aux_1*y1-aux_3*x2+aux_3*x1 ;
		_jacobianOplusXi( 1 , 0 ) = aux_3 ;
		_jacobianOplusXi( 1 , 1 ) = aux_2 ;
		_jacobianOplusXi( 1 , 2 ) = -aux_3*y2+aux_3*y1-aux_1*x2+aux_1*x1 ;

		_jacobianOplusXj( 0 , 0 ) = aux_1 ;
		_jacobianOplusXj( 0 , 1 ) = aux_3 ;
		_jacobianOplusXj( 1 , 0 ) = -aux_3 ;
		_jacobianOplusXj( 1 , 1 ) = aux_1 ;
	}
#endif



} // end namespace
