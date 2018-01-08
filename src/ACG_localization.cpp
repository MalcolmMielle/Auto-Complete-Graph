#include "auto_complete_graph/ACG_localization.hpp"


AASS::acg::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(double x, double y, double theta, int from_id, const Eigen::Matrix3d& information)
{
	g2o::SE2 se2(x, y, theta);
	return addLocalization(se2, from_id, information);

}

AASS::acg::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(const g2o::SE2& localization, int from_id, const Eigen::Matrix3d& information)
{
	
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	return addLocalization(localization, from_ptr, information);

}

AASS::acg::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(const g2o::SE2& localization, g2o::HyperGraph::Vertex* v1, const Eigen::Matrix3d& information)
{
	
	assert(information.isZero(1e-10) == false);
	
	EdgeLocalization* edge_loc = new EdgeLocalization;
	edge_loc->vertices()[0] = v1 ;
	edge_loc->vertices()[1] = _vertex_reference_for_montecarlo;
	edge_loc->setMeasurement(localization);
	edge_loc->setInformation(information);
	
// 	odometry->interface.setAge(_age_start_value);
	
	_optimizable_graph.addEdge(edge_loc);
	_edges_localization.push_back(edge_loc);
	return edge_loc;

}


AASS::acg::VertexSE2Prior* AASS::acg::AutoCompleteGraphLocalization::setPriorReference()
{
	assert(_nodes_prior.size() > 0);
	_vertex_reference_for_montecarlo = _nodes_prior[0];
}



void AASS::acg::AutoCompleteGraphLocalization::updateNDTGraph(const graph_map::GraphMapMsg& ndt_graph)
{
	
	std::cout << "Node in graph " << ndt_graph.nodes.size() << std::endl;
	
	if(_previous_number_of_node_in_ndtgraph != ndt_graph.nodes.size() ){
		
// 		auto factors = ndt_graph.factors();
		
		size_t i;
		i = _previous_number_of_node_in_ndtgraph - 1;
		
		for (i; i < ndt_graph.nodes.size() - 1 ; ++i) {
			
			AASS::acg::VertexSE2RobotPose* robot_ptr;
			g2o::SE2 robot_pos;
			auto map = addElementNDT(ndt_graph, i, &robot_ptr, robot_pos);
			assert(robot_ptr != NULL);
			std::cout << "TEST pointer " << std::endl; std::cout << robot_ptr->getPose().matrix() << std::endl;
			//********************** Extract the corners *****************//
			extractCornerNDTMap(map, robot_ptr, robot_pos);
			//********************** Add the time stamp ******************//
// 			robot_ptr->setTime(ndt_graph.nodes[i].time_last_update);
			
		}
		
		_previous_number_of_node_in_ndtgraph = ndt_graph.nodes.size();
		
	}
	
	

}

std::shared_ptr<perception_oru::NDTMap> AASS::acg::AutoCompleteGraphLocalization::addElementNDT(const graph_map::GraphMapMsg& ndt_graph, int element, AASS::acg::VertexSE2RobotPose** robot_ptr, g2o::SE2& robot_pos)
{
	
	//Return Gaussian white noise
// 	auto randomNoise = [](double mean, double deviationt) -> double {
// 		std::default_random_engine engine{std::random_device()() };
// 		std::normal_distribution<double> dist(mean, deviationt);
// 		return dist(engine);
// 	};
	
	Eigen::Vector3d diff_vec = getLastTransformation();
	
	//Calculate noise

	std::cout << "Checking node nb " << element << std::endl;
	
	//RObot pose
// 			ndt_feature::NDTFeatureNode& feature = dynamic_cast<ndt_feature::NDTFeatureNode&>(ndt_graph.getNodeInterface(i));
	std::cout << "Copy feature" << std::endl;
	
	auto geometry_pose = ndt_graph.nodes[element].pose;
	Eigen::Affine3d affine;
	tf::poseMsgToEigen(geometry_pose, affine);
// 			feature.copyNDTFeatureNode( (const ndt_feature::NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
// 	Eigen::Affine3d affine = Eigen::Affine3d(ndt_graph.getNodeInterface(element).getPose());
	
	Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
	
	//TODO make this work
// 			isometry2d =  diff * isometry2d;
	
	//BUG IS AFTER
	std::cout << "print" << std::endl;
	
	robot_pos = g2o::SE2(isometry2d);
// 			std::cout << "robot pose done : " << isometry2d.matrix() << std::endl;
	g2o::SE2 diff_vec_se2(diff_vec);
// 			std::cout << "diff vec done" << diff_vec << std::endl;
	robot_pos = robot_pos * diff_vec_se2;
	std::cout << "multiply" << std::endl;
		
	auto map_msg =  ndt_graph.nodes[element].ndt_map;
	perception_oru::NDTMap* map;
	perception_oru::NDTMap* map_copy;
	perception_oru::LazyGrid* lz;
	std::string frame;
	bool good2 = perception_oru::fromMessage(lz, map_copy, map_msg, frame);
	std::shared_ptr<perception_oru::NDTMap> shared_map(map_copy);
		
// 	std::cout << "get res" << std::endl;
// // 			double resolution = dynamic_cast<ndt_feature::NDTFeatureNode&>( ndt_graph.getNodeInterface(i) ).map->params_.resolution;
// // 			Use a a msg to copy to a new pointer so it doesn't get forgotten :|
// 	ndt_map::NDTMapMsg msg;
// // 			ATTENTION Frame shouldn't be fixed
// 	bool good = perception_oru::toMessage(map, msg, "/world");
// 	perception_oru::NDTMap* map_copy;
// 	perception_oru::LazyGrid* lz;
// 	std::string frame;
// 	bool good2 = perception_oru::fromMessage(lz, map_copy, msg, frame);
// 	std::shared_ptr<perception_oru::NDTMap> shared_map(map_copy);

	*robot_ptr = addRobotPose(robot_pos, affine, shared_map);
	assert(*robot_ptr != NULL);
	//Add Odometry if it is not the first node
	
	if(element > 0 ){
		std::cout << "adding the odometry" << std::endl;
		
		auto geometry_pose_factor = ndt_graph.factors[element - 1].diff;
		Eigen::Affine3d affine_factor;
		tf::poseMsgToEigen(geometry_pose_factor, affine_factor);
		Eigen::Isometry2d isometry2d_odometry = Affine3d2Isometry2d(affine_factor);
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
		g2o::SE2 odometry(isometry2d_odometry);
// 		g2o::SE2 odometry = NDTFeatureLink2EdgeSE2(links[element - 1]);
		
		std::cout << " ref " << ndt_graph.factors[element-1].prev.data << " and mov " << ndt_graph.factors[element-1].next.data << std::endl;
		
// 		assert( links[element-1].getRefIdx() < _nodes_ndt.size() );
// 		assert( links[element-1].getMovIdx() < _nodes_ndt.size() );
		
		auto from = _nodes_ndt[ ndt_graph.factors[element-1].prev.data ] ;
		auto toward = _nodes_ndt[ ndt_graph.factors[element-1].next.data ] ;
		
		std::cout << "Saving cov " << std::endl;
		//TODO : transpose to 3d and use in odometry!
		
		auto cov_msg = ndt_graph.factors[element - 1].covariance;
		std::vector<double>::const_iterator it;
		it=cov_msg.data.begin();
// 		std::cout << "Cov size " << m.cov.data.size() << std::endl;
		assert(cov_msg.data.size() == 9);
		Eigen::Matrix3d cov;
		cov <<  *it, *(it+1), *(it+2), 
				*(it+3), *(it+4), *(it+5), 
				*(it+6), *(it+7), *(it+8);
		
// 		tf::matrixEigenToMsg(cov, ndt_graph.factors[element - 1].covariance);
		
		
		std::cout << "COV " << cov << std::endl;
		
		std::cout << "Saving cov to 2d" << std::endl;
		Eigen::Matrix3d cov_2d;
		cov_2d << 	cov(0, 0), 	cov(0, 1), 	0,
					cov(1, 0), 	cov(1, 1), 	0,
					0, 		 	0, 			cov(5, 5);
					
		std::cout << "Saving information " << std::endl;
		Eigen::Matrix3d information = cov_2d.inverse();
		
// 				if(noise_flag = true && i != 0){
// 					odometry = odometry * noise_se2;
// 				}
		
		std::cout << "Saving odometry " << std::endl;
		addOdometry(odometry, from, toward, information);
	}
	
	return shared_map;

}

