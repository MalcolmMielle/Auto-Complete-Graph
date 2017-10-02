#include "auto_complete_graph/OptimizableAutoCompleteGraph.hpp"


void AASS::acg::OptimizableAutoCompleteGraph::setRobustKernelAllEdges(g2o::RobustKernel* ptr, double width)
{
	for (SparseOptimizer::VertexIDMap::const_iterator it = this->vertices().begin(); it != this->vertices().end(); ++it) {
		OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
		v->setMarginalized(false);
	}		
	
	auto idmapedges = this->edges();
	if(ptr != NULL){
		for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
			std::cout << "Robust Kern" << std::endl;
			OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*ite);
			e->setRobustKernel(ptr);
			e->robustKernel()->setDelta(width);
		}
	}
}
