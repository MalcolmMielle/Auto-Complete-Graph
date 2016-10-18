#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016

#include "auto_complete_graph/ACG.hpp"

namespace AASS {

namespace acg{	
	
	
	class VisuAutoCompleteGraph{ 
	
	protected:
		ros::NodeHandle _nh;
		ros::Publisher _last_ndtmap;
		
	public:
		VisuAutoCompleteGraph(){
			_last_ndtmap = _nh.advertise<ndt_map::NDTMapMsg>("lastgraphmap", 10);
		}
		
		void toRviz(const AutoCompleteGraph& acg);
		
	private:
		
		bool fuseNDTMap(const AutoCompleteGraph& acg, lslgeneric::NDTMap& out_map);
		bool printNDTMap(lslgeneric::NDTMap* map, const std::__cxx11::string& frame_name, ndt_map::NDTMapMsg& mapmsg);
		void printPrior(const std::vector<g2o::VertexSE2Prior*>& prior_corners);
		
		
	};
}
}

#endif