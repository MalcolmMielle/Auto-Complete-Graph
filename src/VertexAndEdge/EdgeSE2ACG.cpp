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

#include "auto_complete_graph/VertexAndEdge/EdgeSE2ACG.hpp"

namespace g2o {

	EdgeSE2ACG::EdgeSE2ACG() :
			BaseBinaryEdge<3, SE2, VertexSE2ACG, VertexSE2ACG>()
	{
	}

	bool EdgeSE2ACG::read(std::istream& is)
	{
		Vector3 p;
		is >> p[0] >> p[1] >> p[2];
		setMeasurement(SE2(p));
		_inverseMeasurement = measurement().inverse();
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
			}
		return true;
	}

	bool EdgeSE2ACG::write(std::ostream& os) const
	{
		Vector3 p = measurement().toVector();
		os << p.x() << " " << p.y() << " " << p.z();
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}

	void EdgeSE2ACG::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
	{
		VertexSE2ACG* fromEdge = static_cast<VertexSE2ACG*>(_vertices[0]);
		VertexSE2ACG* toEdge   = static_cast<VertexSE2ACG*>(_vertices[1]);
		if (from.count(fromEdge) > 0)
			toEdge->setEstimate(fromEdge->estimate() * _measurement);
		else
			fromEdge->setEstimate(toEdge->estimate() * _inverseMeasurement);
	}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
	void EdgeSE2ACG::linearizeOplus()
	{
		const VertexSE2ACG* vi = static_cast<const VertexSE2ACG*>(_vertices[0]);
		const VertexSE2ACG* vj = static_cast<const VertexSE2ACG*>(_vertices[1]);
		number_t thetai = vi->estimate().rotation().angle();

		Vector2 dt = vj->estimate().translation() - vi->estimate().translation();
		number_t si=std::sin(thetai), ci=std::cos(thetai);

		_jacobianOplusXi(0, 0) = -ci; _jacobianOplusXi(0, 1) = -si; _jacobianOplusXi(0, 2) = -si*dt.x()+ci*dt.y();
		_jacobianOplusXi(1, 0) =  si; _jacobianOplusXi(1, 1) = -ci; _jacobianOplusXi(1, 2) = -ci*dt.x()-si*dt.y();
		_jacobianOplusXi(2, 0) =  0;  _jacobianOplusXi(2, 1) = 0;   _jacobianOplusXi(2, 2) = -1;

		_jacobianOplusXj(0, 0) = ci; _jacobianOplusXj(0, 1)= si; _jacobianOplusXj(0, 2)= 0;
		_jacobianOplusXj(1, 0) =-si; _jacobianOplusXj(1, 1)= ci; _jacobianOplusXj(1, 2)= 0;
		_jacobianOplusXj(2, 0) = 0;  _jacobianOplusXj(2, 1)= 0;  _jacobianOplusXj(2, 2)= 1;

		const SE2& rmean = _inverseMeasurement;
		Matrix3 z = Matrix3::Zero();
		z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
		z(2, 2) = 1.;
		_jacobianOplusXi = z * _jacobianOplusXi;
		_jacobianOplusXj = z * _jacobianOplusXj;
	}
#endif



} // end namespace
