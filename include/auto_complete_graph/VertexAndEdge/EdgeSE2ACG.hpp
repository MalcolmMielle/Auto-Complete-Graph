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

#ifndef AUTOCOMPLETEGRAPH_EDGESE2ACG_12032018
#define AUTOCOMPLETEGRAPH_EDGESE2ACG_12032018

#include "VertexSE2ACG.hpp"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
//#include "g2o/types/slam2d/g2o_types_slam2d_api.h"

namespace g2o {

	/**
	 * \brief 2D edge between two Vertex2
	 */
	class EdgeSE2ACG : public BaseBinaryEdge<3, SE2, VertexSE2ACG, VertexSE2ACG>
{
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE2ACG();

	void computeError()
	{
		const VertexSE2ACG* v1 = static_cast<const VertexSE2ACG*>(_vertices[0]);
		const VertexSE2ACG* v2 = static_cast<const VertexSE2ACG*>(_vertices[1]);
		SE2 delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
		_error = delta.toVector();
	}
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void setMeasurement(const SE2& m){
		_measurement = m;
		_inverseMeasurement = m.inverse();
	}

	virtual bool setMeasurementData(const number_t* d){
		_measurement=SE2(d[0], d[1], d[2]);
		_inverseMeasurement = _measurement.inverse();
		return true;
	}

	virtual bool getMeasurementData(number_t* d) const {
		Vector3 v=_measurement.toVector();
		d[0] = v[0];
		d[1] = v[1];
		d[2] = v[2];
		return true;
	}

	virtual int measurementDimension() const {return 3;}

	virtual bool setMeasurementFromState() {
		const VertexSE2ACG* v1 = static_cast<const VertexSE2ACG*>(_vertices[0]);
		const VertexSE2ACG* v2 = static_cast<const VertexSE2ACG*>(_vertices[1]);
		_measurement = v1->estimate().inverse()*v2->estimate();
		_inverseMeasurement = _measurement.inverse();
		return true;
	}


	virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
	virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
	virtual void linearizeOplus();
#endif
	protected:
	SE2 _inverseMeasurement;
};



} // end namespace

#endif
