#include "auto_complete_graph/VisuACG.hpp"

bool AASS::acg::VisuAutoCompleteGraph::printNDTMap(lslgeneric::NDTMap* map, const std::string& frame_name, ndt_map::NDTMapMsg& mapmsg)
{
	return lslgeneric::toMessage(map, mapmsg, frame_name);
}


void AASS::acg::VisuAutoCompleteGraph::printPrior(const std::vector< g2o::VertexSE2Prior* >& prior_corners)
{

}

void AASS::acg::VisuAutoCompleteGraph::toRviz(const AASS::acg::AutoCompleteGraph& acg)
{

}


bool AASS::acg::VisuAutoCompleteGraph::fuseNDTMap(const AASS::acg::AutoCompleteGraph& acg, lslgeneric::NDTMap& out_map)
{

	
	
	
}
