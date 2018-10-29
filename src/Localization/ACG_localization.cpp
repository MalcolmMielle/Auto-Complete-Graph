#include "auto_complete_graph/Localization/ACG_localization.hpp"

bool AASS::acg::AutoCompleteGraphLocalization::verifyInformationMatrices(bool verbose) const
{
	bool allEdgeOk = true;
	Eigen::SelfAdjointEigenSolver<g2o::MatrixX> eigenSolver;
	for (g2o::OptimizableGraph::EdgeSet::const_iterator it = _optimizable_graph.edges().begin(); it != _optimizable_graph.edges().end(); ++it) {
		g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);
		g2o::MatrixX::MapType information(e->informationData(), e->dimension(), e->dimension());
		// test on symmetry

		information = information * 1000;
		information = information.array().round();
		information = information / 1000;

		bool isSymmetric = information.transpose() == information;
		bool okay = isSymmetric;
		if (isSymmetric) {
			// compute the eigenvalues
			eigenSolver.compute(information, Eigen::EigenvaluesOnly);
			bool isSPD = eigenSolver.eigenvalues()(0) >= 0.;
			okay = okay && isSPD;
		}
		allEdgeOk = allEdgeOk && okay;
		if (! okay) {
			if (verbose) {
				std::cerr << "In a " << getEdgeType(e) << " ";
				if (! isSymmetric)
					std::cerr << "Information Matrix for an edge is not symmetric:";
				else
					std::cerr << "Information Matrix for an edge is not SPD:";
				for (size_t i = 0; i < e->vertices().size(); ++i)
					std::cerr << " " << e->vertex(i)->id();
				if (isSymmetric)
					std::cerr << "\teigenvalues: " << eigenSolver.eigenvalues().transpose();
				std::cerr << " information :\n" << information << "\n" << std::endl;
			}
		}
	}
	return allEdgeOk;
}



g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addRobotLocalization(const g2o::SE2& se2_robot_pose, const Eigen::Affine3d& affine_original_robot_pose, Eigen::Vector3d to_robot_localization, const Eigen::Matrix3d& cov_localization, const std::shared_ptr< perception_oru::NDTMap >& map){

//	std::cout << "Adding the robot pose localization" << std::endl;
	g2o::VertexSE2RobotLocalization* robotlocalization =  new g2o::VertexSE2RobotLocalization(to_robot_localization);
//	robotlocalization->setEquivalentRobotPose(equivalent_robot_pose);

	robotlocalization->setEstimate(se2_robot_pose);
	robotlocalization->setId(new_id_);
	++new_id_;

	robotlocalization->setMap(map);
	robotlocalization->setPose(affine_original_robot_pose);
	robotlocalization->setCovariance(cov_localization);
	robotlocalization->initial_transfo = robotlocalization->estimate();

	_optimizable_graph.addVertex(robotlocalization);

// 	NDTNodeAndMap nodeAndMap(robot, map, affine);

	_nodes_localization.push_back(robotlocalization);

	//WORST HACK ON EARTH. IT'S ONLY HERE FOR EXPORTING THE MESSAGE AND NOT BREAKING ALL THE VISUALIZATION :(
	//PLEASE OWN SELF; REMOVE THIS ONE DAY !!!!!
	_nodes_ndt.push_back(robotlocalization);

//	std::cout << "Localization added " << std::endl;

	return robotlocalization;
}
g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addRobotLocalization(const Eigen::Vector3d& rob_localization, const Eigen::Affine3d& affine_original_robot_pose, Eigen::Vector3d to_robot_localization, const Eigen::Matrix3d& cov_localization, const std::shared_ptr< perception_oru::NDTMap >& map){
	g2o::SE2 se2(rob_localization(0), rob_localization(1), rob_localization(2));
	return addRobotLocalization(se2, affine_original_robot_pose, to_robot_localization, cov_localization, map);
}
g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addRobotLocalization(double x, double y, double theta, const Eigen::Affine3d& affine_original_robot_pose, Eigen::Vector3d to_robot_localization, const Eigen::Matrix3d& cov_localization, const std::shared_ptr< perception_oru::NDTMap >& map){
	Eigen::Vector3d robot1;
	robot1 << x, y, theta;
	return addRobotLocalization(robot1, affine_original_robot_pose, to_robot_localization, cov_localization, map);
}





g2o::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(double x, double y, double theta, int from_id, const Eigen::Matrix3d& information)
{
	g2o::SE2 se2(x, y, theta);
	return addLocalization(se2, from_id, information);

}

g2o::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(const g2o::SE2& localization, int from_id, const Eigen::Matrix3d& information)
{
	
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	return addLocalization(localization, from_ptr, information);

}

g2o::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(const g2o::SE2& localization, g2o::HyperGraph::Vertex* v1, const Eigen::Matrix3d& information)
{
	
	assert(information.isZero(1e-10) == false);
	
	g2o::EdgeLocalization* edge_loc = new g2o::EdgeLocalization;
	edge_loc->vertices()[0] = v1 ;
	edge_loc->vertices()[1] = _vertex_reference_for_montecarlo;
	edge_loc->setMeasurement(localization);
	edge_loc->setInformation(information);
	
// 	odometry->interface.setAge(_age_start_value);
	
	_optimizable_graph.addEdge(edge_loc);
	_edges_localization.push_back(edge_loc);
	return edge_loc;

}


g2o::VertexNDTCell* AASS::acg::AutoCompleteGraphLocalization::addNDTCellVertex(const Eigen::Vector2d pose, const boost::shared_ptr<perception_oru::NDTCell>& cell, g2o::VertexSE2RobotLocalization* robot_node){
//	std::cout << "Adding vertex ndt cell" << std::endl;
	g2o::VertexNDTCell* ndt_cell_v = new g2o::VertexNDTCell();
//	std::cout << "Set cell " << std::endl;
	ndt_cell_v->setCell(cell);
	ndt_cell_v->setId(new_id_);
	++new_id_;
	ndt_cell_v->setEstimate(pose);
	_optimizable_graph.addVertex(ndt_cell_v);
	_vertices_ndt_cell.insert(ndt_cell_v);

//	std::cout << "Set landmark " << std::endl;
	//Add link to robot map so we had an obbservation edge
	Eigen::Vector2d pose_landmark = cell->getMean().head(2);
	//TODO CRASH :(
//	auto edge = addNDTCellObservation(pose_landmark, robot_node, ndt_cell_v, cell->getCov().block(0,0,2,2));
	auto edge = addNDTCellObservation(pose_landmark, robot_node, ndt_cell_v);

	robot_node->ndt_cells.push_back(ndt_cell_v);

//	std::cout << "Added vertex ndt cell" << std::endl;

	return ndt_cell_v;
}


g2o::EdgeNDTCell* AASS::acg::AutoCompleteGraphLocalization::addNDTCellAssociation(g2o::HyperGraph::Vertex* v_ndt_cell, g2o::EdgeXYPriorACG* wall, const Eigen::Matrix2d& covariance_landmark) {

	g2o::EdgeNDTCell* edge_ndt = new g2o::EdgeNDTCell(wall);
	edge_ndt->vertices()[0] = v_ndt_cell ;
	edge_ndt->vertices()[1] = wall->vertices()[0] ;

//	Eigen::Matrix2d cov_tmp = covariance_landmark;
//
//	std::cout << "COV " << cov_tmp << std::endl;
//
//	bool low = false;
//	double min_val = 0;
//	for (size_t i = 0 ; i <cov_tmp.cols(); ++i){
//		for (size_t j = 0; j < cov_tmp.rows(); ++j){
//			if(cov_tmp(i,j) < _min_value_cov_ndt_cell && cov_tmp(i,j) > -_min_value_cov_ndt_cell){
//				low = true;
//				min_val = cov_tmp(i,j);
//			}
//		}
//	}
//
//	if(low == true) {
////		cov_tmp.array() = cov_tmp.array() * ( _min_value_cov_ndt_cell / min_val);
//	}
//	std::cout << "COV " << cov_tmp << std::endl;



	Eigen::Matrix2d information_landmark = covariance_landmark.inverse();
	edge_ndt->setInformation(information_landmark);
//	edge_ndt->setMeasurement(localization);
//	edge_ndt->setInformation(information);

// 	odometry->interface.setAge(_age_start_value);

	_optimizable_graph.addEdge(edge_ndt);
	_edges_ndt_cell.insert(edge_ndt);

	assert(verifyInformationMatrices(true) == true);

	return edge_ndt;

}


g2o::EdgePriorObservation* AASS::acg::AutoCompleteGraphLocalization::addPriorObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix2d& covariance_landmark, g2o::EdgeLandmark_malcolm* equivalent_landmark_observation_edge){

	//Making sure the same edge is not added twice

	//for(auto land_edge : _edge_landmark){
	//	if(v1 == land_edge->vertices()[0] && v2 == land_edge->vertices()[1]){
	//		throw std::runtime_error("Edge observation already added");
	//	}
	//	else if(v1 == land_edge->vertices()[1] && v2 == land_edge->vertices()[0]){
	//		throw std::runtime_error("Edge observation already added");
	//	}
	//}

	//Might have more than one observation ot hte prior
	//assert(findObservation(v1, v2) == NULL);

	Eigen::Matrix2d information_landmark = covariance_landmark.inverse();

	g2o::EdgePriorObservation* landmarkObservation =  new g2o::EdgePriorObservation(equivalent_landmark_observation_edge);
	landmarkObservation->vertices()[0] = v1;
	landmarkObservation->vertices()[1] = v2;
	landmarkObservation->setMeasurement(pos);

	assert(information_landmark.isZero(1e-10) == false);

	landmarkObservation->setInformation(information_landmark);
	landmarkObservation->setParameterId(0, _sensorOffset->id());


// 	landmarkObservation->interface.setAge(_age_start_value);

	_optimizable_graph.addEdge(landmarkObservation);
	_edge_prior_observation.push_back(landmarkObservation);

	assert(verifyInformationMatrices(true) == true);

	return landmarkObservation;
}


inline g2o::EdgePriorObservation* AASS::acg::AutoCompleteGraphLocalization::addPriorObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, g2o::EdgeLandmark_malcolm* equivalent_landmark_observation_edge){
	Eigen::Matrix2d covariance_landmark;
	covariance_landmark.fill(0.);
	covariance_landmark(0, 0) = _landmarkNoise[0]*_landmarkNoise[0];
	covariance_landmark(1, 1) = _landmarkNoise[1]*_landmarkNoise[1];

//	throw std::runtime_error("Do not use user inputed values in landmark observation");

// 			covariance_landmark(2, 2) = 13;//<- Rotation covariance landmark is more than 4PI
	return addPriorObservation(pos, v1, v2, covariance_landmark, equivalent_landmark_observation_edge);
}

g2o::EdgePriorObservation* AASS::acg::AutoCompleteGraphLocalization::addPriorObservation(const g2o::Vector2& pos, int from_id, int toward_id, g2o::EdgeLandmark_malcolm* equivalent_landmark_observation_edge){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addPriorObservation(pos, from_ptr, toward_ptr, equivalent_landmark_observation_edge);
}



g2o::EdgeNDTCellObservation* AASS::acg::AutoCompleteGraphLocalization::addNDTCellObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix2d& covariance_landmark){

	//Making sure the same edge is not added twice

	//for(auto land_edge : _edge_landmark){
	//	if(v1 == land_edge->vertices()[0] && v2 == land_edge->vertices()[1]){
	//		throw std::runtime_error("Edge observation already added");
	//	}
	//	else if(v1 == land_edge->vertices()[1] && v2 == land_edge->vertices()[0]){
	//		throw std::runtime_error("Edge observation already added");
	//	}
	//}

	//Might have more than one observation ot hte prior
	//assert(findObservation(v1, v2) == NULL);

	Eigen::Matrix2d information_landmark = covariance_landmark.inverse();

	g2o::EdgeNDTCellObservation* landmarkObservation =  new g2o::EdgeNDTCellObservation;
	landmarkObservation->vertices()[0] = v1;
	landmarkObservation->vertices()[1] = v2;
	landmarkObservation->setMeasurement(pos);

	assert(information_landmark.isZero(1e-10) == false);

	landmarkObservation->setInformation(information_landmark);
	landmarkObservation->setParameterId(0, _sensorOffset->id());


// 	landmarkObservation->interface.setAge(_age_start_value);

	_optimizable_graph.addEdge(landmarkObservation);
	_edges_ndt_cell_observation.insert(landmarkObservation);

//	std::cout << "Testing before adding OBSERVATIOn" << std::endl;
	assert(landmarkObservation->information().isZero(1e-10) == false);

	return landmarkObservation;
}


g2o::EdgeNDTCellObservation* AASS::acg::AutoCompleteGraphLocalization::addNDTCellObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
	Eigen::Matrix2d covariance_landmark;
	covariance_landmark.fill(0.);
	covariance_landmark(0, 0) = _landmarkNoise[0]*_landmarkNoise[0];
	covariance_landmark(1, 1) = _landmarkNoise[1]*_landmarkNoise[1];

//	throw std::runtime_error("Do not use user inputed values in landmark observation");

// 			covariance_landmark(2, 2) = 13;//<- Rotation covariance landmark is more than 4PI
	return addNDTCellObservation(pos, v1, v2, covariance_landmark);
}

g2o::EdgeNDTCellObservation* AASS::acg::AutoCompleteGraphLocalization::addNDTCellObservation(const g2o::Vector2& pos, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addNDTCellObservation(pos, from_ptr, toward_ptr);
}








g2o::VertexXYPrior* AASS::acg::AutoCompleteGraphLocalization::setPriorReference()
{
	if(_prior->getNodes().size() > 0) {
		_vertex_reference_for_montecarlo = *(_prior->getNodes().begin());
		return _vertex_reference_for_montecarlo;
	}
	return NULL;
}


void AASS::acg::AutoCompleteGraphLocalization::updateNDTGraph(const auto_complete_graph::GraphMapLocalizationMsg& ndt_graph_localization) {

	if(_use_robot_maps == true) {
		addNDTGraph(ndt_graph_localization);
		updatePriorObservations();
	}
//	updateLocalizationEdges(ndt_graph_localization);


}


void AASS::acg::AutoCompleteGraphLocalization::addNoiseOdometry(g2o::EdgeOdometry_malcolm* edge_odometry, g2o::VertexSE2RobotLocalization* robot, double percentage_noise) {

	double rand_num_perc = _dis(_gen);
	//FOR TESTING REMOVE!
	if (rand_num_perc > 0) {
		rand_num_perc = percentage_noise;
	} else {
		rand_num_perc = -percentage_noise;
	}
	std::cout << "Percentage of noise: " << rand_num_perc << std::endl;

	g2o::SE2 old_odom_se2 = edge_odometry->measurement();

	std::cout << "Get length and angle" << std::endl;
	double length = edge_odometry->measurement().toVector().head(2).norm();
	double angle = edge_odometry->measurement().toVector()(2);

	std::cout << "New odo" << std::endl;
	Eigen::Vector3d new_odo;
	new_odo.head(2).array() = edge_odometry->measurement().toVector().head(2).array() + (length * rand_num_perc);
//		odometry.setTranslation(new_odo);
	new_odo(2) = angle + (angle * rand_num_perc);
//		Eigen::Rotation2D<double> ro(new_rot);
//		odometry.setRotation(ro);
	edge_odometry->setMeasurement(new_odo);

	std::cout << "info" << std::endl;

	Eigen::Matrix3d inf_2d = edge_odometry->information();
	Eigen::Matrix3d cov_2d = inf_2d.inverse();
	std::cout << "Old cov " << cov_2d << std::endl;

	cov_2d.block(0, 0, 2, 2).array() = cov_2d.block(0, 0, 2, 2).array() * 100; // + (length * std::abs(rand_num_perc)) );
	cov_2d(2, 2) = cov_2d(2, 2) + (std::abs(angle) * std::abs(rand_num_perc));


//	cov_2d << 3, 0, 0,
//			0, 3, 0,
//			0, 0, 0.1745329;

	std::cout << "New cov " << cov_2d << std::endl;
	edge_odometry->setInformation(cov_2d.inverse());


	std::cout << "move robot pose" << std::endl;
	//move robot pose and ndt cells !
	g2o::SE2 new_odom_se2 = edge_odometry->measurement();
	g2o::SE2 noise = old_odom_se2.inverse() * new_odom_se2;

//	g2o::VertexSE2RobotLocalization* loc = edge_odometry->vertices()[1];
	robot->setEstimate(robot->estimate() * noise);
	robot->initial_noisy_estimate = robot->estimate() * noise;

	std::cout << "move ndt cells" << std::endl;
	for(auto ndt_cell: robot->ndt_cells){
		Eigen::Vector3d mean = ndt_cell->getCell()->getMean();
		mean(2) = 0;
		g2o::SE2 mean_se2(mean);
		g2o::SE2 rob_pose = robot->estimate();
		Eigen::Vector2d mean_ndt_pose = (rob_pose * mean_se2).toVector().head(2);
//		Eigen::Vector2d ndt_pose = ndt_cell->estimate();
//		ndt_pose = ndt_pose * noise;
		ndt_cell->setEstimate(mean_ndt_pose);
	}

	std::cout << "Verify information after adding noise" << std::endl;
	verifyInformationMatrices(true);

}

void AASS::acg::AutoCompleteGraphLocalization::addNDTGraph(const auto_complete_graph::GraphMapLocalizationMsg& ndt_graph_localization)
{
	if(_use_robot_maps == true) {
		ROS_DEBUG_STREAM("Node in graph " << ndt_graph_localization.graph_map.nodes.size() );

		if (_previous_number_of_node_in_ndtgraph != ndt_graph_localization.graph_map.nodes.size()) {

// 		auto factors = ndt_graph.factors();

			size_t i;
			i = _previous_number_of_node_in_ndtgraph - 1;

			assert(i >= 0);

			for (i; i < ndt_graph_localization.graph_map.nodes.size() - 1; ++i) {

//				g2o::VertexSE2RobotPose *robot_ptr;
				g2o::VertexSE2RobotLocalization *robot_localization_ptr;
				std::shared_ptr<perception_oru::NDTMap> map;
//			g2o::SE2 robot_pos;
				std::tie(robot_localization_ptr, map) = addElementNDT(ndt_graph_localization, i);
				assert(robot_localization_ptr != NULL);

				robot_localization_ptr->time_sec = ndt_graph_localization.graph_map.nodes[i].time_sec.data;
				robot_localization_ptr->time_nsec = ndt_graph_localization.graph_map.nodes[i].time_nsec.data;

//				std::cout << "TEST pointer " << std::endl;
//				std::cout << robot_localization_ptr->getPose().matrix() << std::endl;
				//********************** Extract the corners *****************//
				if (_extract_corners) {
					extractCornerNDTMap(map, robot_localization_ptr);
					//NEED TO ADD OBSERVATION EDGES BETWEEN MCL AND CORNERS HERE TODO
					AddObservationsMCLPrior();
				}
				//********************** Link to wall in EM ******************//
				if(_match_robot_maps == true) {
					createWallAssociations(robot_localization_ptr);
				}else{
					ROS_WARN("Not using NDT map for optimization");
				}
				//********************** Add the time stamp ******************//
// 			robot_ptr->setTime(ndt_graph.nodes[i].time_last_update);

			}

			_previous_number_of_node_in_ndtgraph = ndt_graph_localization.graph_map.nodes.size();

		}

//	updateLinks();

//	std::cout << "Test no double links" << std::endl;
//	noDoubleLinks();

		ROS_DEBUG_STREAM("After ");

		assert(_nodes_ndt.size() == _nodes_localization.size());

	}
	
	

}

std::tuple<g2o::VertexSE2RobotLocalization*, std::shared_ptr<perception_oru::NDTMap> > AASS::acg::AutoCompleteGraphLocalization::addElementNDT(const auto_complete_graph::GraphMapLocalizationMsg& ndt_graph_localization, int element)
{
	///ATTENTION Indexes start at 1 instead of 0 :/
//	std::cout << "good indexes " << ndt_graph_localization.graph_map.nodes[element].id.data << " = " << element << " voilaaa " << std::endl;
	assert(ndt_graph_localization.graph_map.nodes[element].id.data == element);
	//Return Gaussian white noise
// 	auto randomNoise = [](double mean, double deviationt) -> double {
// 		std::default_random_engine engine{std::random_device()() };
// 		std::normal_distribution<double> dist(mean, deviationt);
// 		return dist(engine);
// 	};
	
	Eigen::Vector3d diff_vec;
	if(_add_odometry_noise && _not_incremental) {
		diff_vec << 0, 0, 0;
	}else{
		//Ignore the previous noise on the odometry. A bit HACKY
		diff_vec = getLastTransformation();
		//**************** Calculate the previous transformations if there has already been something added *** //

// 	if(_previous_number_of_node_in_ndtgraph != 0){
//		if(_nodes_localization.size() != 0){
//			auto node = _nodes_localization[_nodes_localization.size() - 1];
//			auto original3d = _nodes_localization[_nodes_localization.size() - 1]->getPose();
//
//			/*******' Using Vec********/
//			Eigen::Isometry2d original2d_aff = Affine3d2Isometry2d(original3d);
//			auto shift_vec = node->estimate().toVector();
//			g2o::SE2 se2_tmptmp(original2d_aff);
//			auto original3d_vec = se2_tmptmp.toVector();
//			diff_vec = original3d_vec - shift_vec;
//
//			/*****************************/
//		}

	}
	
	//Calculate noise

//	std::cout << "Checking node nb " << element << std::endl;
	
	//RObot pose
// 			ndt_feature::NDTFeatureNode& feature = dynamic_cast<ndt_feature::NDTFeatureNode&>(ndt_graph.getNodeInterface(i));
//	std::cout << "Copy feature" << std::endl;
	
	auto geometry_pose = ndt_graph_localization.graph_map.nodes[element].pose;
	Eigen::Affine3d affine;
	tf::poseMsgToEigen(geometry_pose, affine);
// 			feature.copyNDTFeatureNode( (const ndt_feature::NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
// 	Eigen::Affine3d affine = Eigen::Affine3d(ndt_graph.getNodeInterface(element).getPose());
	
	Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
	
	//TODO make this work
// 			isometry2d =  diff * isometry2d;
	
	//BUG IS AFTER
//	std::cout << "print" << std::endl;
	
	auto robot_pos = g2o::SE2(isometry2d);
// 			std::cout << "robot pose done : " << isometry2d.matrix() << std::endl;
	g2o::SE2 diff_vec_se2(diff_vec);
// 			std::cout << "diff vec done" << diff_vec << std::endl;

	///WARNING trying to change this to fit GraphMap model. Node are at the exact pose so no need to translate them AND factor I don't know yet
	robot_pos = robot_pos * diff_vec_se2;
//	std::cout << "multiply" << std::endl;
		
	auto map_msg =  ndt_graph_localization.graph_map.nodes[element].ndt_map;

	perception_oru::NDTMap* map;
	perception_oru::NDTMap* map_copy;
	perception_oru::LazyGrid* lz;
	std::string frame;
	bool good2 = perception_oru::fromMessage(lz, map_copy, map_msg, frame);
	std::shared_ptr<perception_oru::NDTMap> shared_map(map_copy);



	assert(shared_map->getAllInitializedCells().size() != 0);
	assert(shared_map->getAllInitializedCells().size() == map_msg.cells.size());
		
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

//	auto robot_ptr = addRobotPose(robot_pos, affine, shared_map);

	auto robot_loc_ptr = addLocalizationVertex(ndt_graph_localization, element, shared_map, robot_pos);

//	robot_ptr->setIndexGraphMap(ndt_graph_localization.graph_map.nodes[element].id.data);
	robot_loc_ptr->setIndexGraphMap(ndt_graph_localization.graph_map.nodes[element].id.data);

	assert(robot_loc_ptr != NULL);
	//Add Odometry if it is not the first node
	
	if(element > 0 ){

		if(_do_own_registration) {
			//From my understanding, for now factors are othe same as all the registrations before. Hence I make it a big registration between the submaps ;)

			auto from = _nodes_localization[ndt_graph_localization.graph_map.factors[element - 1].prev.data];
			auto toward = _nodes_localization[ndt_graph_localization.graph_map.factors[element - 1].next.data];
			g2o::SE2 from_vec = from->estimate();
			g2o::SE2 toward_vec = toward->estimate();
			g2o::SE2 odom = from_vec.inverse() * toward_vec;

//			std::cout << "From " << from_vec.toVector() << " toward " << toward_vec.toVector() << " thus odom " << odom.toVector() << std::endl;
//			std::cout << "Make sure that this is correct before moving forward" << std::endl;
//			int waitwait;
//			std::cin >> waitwait;


			Eigen::Affine3d odom_affine = se2ToAffine3d(odom);

			Eigen::Affine3d odom_register;
			Eigen::MatrixXd odom_cov;


			auto cov_msg = ndt_graph_localization.graph_map.factors[element - 1].covariance;
			std::vector<double>::const_iterator it;
			it = cov_msg.data.begin();
//			ROS_DEBUG_STREAM("Cov size " << cov_msg.data.size() );
			assert(cov_msg.data.size() == 36); //6x6 matrix

//			std::vector<double>::const_iterator it_2;
//			it_2 = cov_msg.data.begin();
//			ROS_DEBUG_STREAM( cov_msg.data.size() );
//			assert(cov_msg.data.size() == 36);

			Eigen::MatrixXd cov_3d(6, 6);
			for (size_t i = 0; i < 6; ++i) {
				for (size_t j = 0; j < 6; ++j) {
					cov_3d(i, j) = cov_msg.data[(6 * i) + j];
				}
			}
////			std::cout << "Saving cov to 2d" << std::endl;
//			Eigen::Matrix3d cov_2d;
//			cov_2d << cov_3d(0, 0), cov_3d(0, 1), 0,
//					cov_3d(1, 0), cov_3d(1, 1), 0,
//					0, 0, cov_3d(5, 5);

			std::tie(odom_register, odom_cov) = registerSubmaps(*from, *toward, odom_affine, 2, cov_3d);

			Eigen::Isometry2d isometry2d_odometry = Affine3d2Isometry2d(odom_register);
			g2o::SE2 odometry(isometry2d_odometry);

//			std::cout << "Saving cov to 2d" << std::endl;
			Eigen::Matrix3d cov_2d;
			cov_2d << odom_cov(0, 0), odom_cov(0, 1), 0,
					odom_cov(1, 0), odom_cov(1, 1), 0,
					0, 0, odom_cov(5, 5);

// 		tf::matrixEigenToMsg(cov, ndt_graph.factors[element - 1].covariance);
//			std::cout << "Saving information " << std::endl;
			Eigen::Matrix3d information = cov_2d.inverse();

//			std::cout << "Saving odometry " << odometry.toVector() << " from " << from << " toward " << toward << " info " << information << " " << std::endl;
			addOdometry(odometry, from, toward, information);

		}
		else {

//			std::cout << "adding the odometry" << std::endl;

			//The factors are expressed as transformation between the node expressed in the world frame. They need to be translated to the robot frame I believe?
			auto geometry_pose_factor = ndt_graph_localization.graph_map.factors[element - 1].diff;
			Eigen::Affine3d affine_factor;
			tf::poseMsgToEigen(geometry_pose_factor, affine_factor);
			Eigen::Isometry2d isometry2d_odometry = Affine3d2Isometry2d(affine_factor);

			////////OLD VERSION
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
			g2o::SE2 odometry(isometry2d_odometry);

			////////NEW VERSION
			//We need to transform the odometry into he robot pose coordinate frame see https://en.wikipedia.org/wiki/Change_of_basis#Change_of_coordinates_of_a_vector

			//Find the matrix describing the rotation and left multiply the odometry by the inverse of it.
//		double angle = robot_pos.toVector()[2];
//		Eigen::Matrix2d rot;
//		rot << std::cos(angle), - std::sin(angle),
//			   std::sin(angle),   std::cos(angle);
//		Eigen::Vector2d odom_2d_translation = isometry2d_odometry.translation();
//		Eigen::Vector2d odom_2d_translation_robot_frame = rot.inverse() * odom_2d_translation;
//		Eigen::Vector3d odom_3d_translation_robot_frame;
//		odom_3d_translation_robot_frame << odom_2d_translation_robot_frame[0], odom_2d_translation_robot_frame[1], 0;
//		g2o::SE2 odometry(odom_3d_translation_robot_frame);
//
//		std::cout << "Odom " << odom_2d_translation_robot_frame[0] << " " << odom_2d_translation_robot_frame[1] << std::endl;

//			std::cout << "ODOM " << odometry.toVector() << std::endl;
//		exit(0);

// 		g2o::SE2 odometry = NDTFeatureLink2EdgeSE2(links[element - 1]);

//			std::cout << " ref " << ndt_graph_localization.graph_map.factors[element - 1].prev.data << " and mov "
//			          << ndt_graph_localization.graph_map.factors[element - 1].next.data << " ndt node size "
//			          << _nodes_localization.size() << std::endl;

			///ATTENTION Indexes of graph_map start at 1 instead of 0 :/
			assert(ndt_graph_localization.graph_map.factors[element - 1].prev.data < _nodes_localization.size());
			assert(ndt_graph_localization.graph_map.factors[element - 1].next.data < _nodes_localization.size());
			assert(ndt_graph_localization.graph_map.factors[element - 1].prev.data >= 0);
			assert(ndt_graph_localization.graph_map.factors[element - 1].next.data >= 0);

			auto from = _nodes_localization[ndt_graph_localization.graph_map.factors[element - 1].prev.data];
			auto toward = _nodes_localization[ndt_graph_localization.graph_map.factors[element - 1].next.data];

			assert(from->getIndexGraphMap() == ndt_graph_localization.graph_map.factors[element - 1].prev.data);
			assert(toward->getIndexGraphMap() == ndt_graph_localization.graph_map.factors[element - 1].next.data);

			auto from_vec = from->estimate().toVector();
			auto toward_vec = toward->estimate().toVector();
			g2o::SE2 odom_tmp = toward->estimate() * (from->estimate()).inverse();
//			std::cout << "TRansof" << from_vec << " toward " << toward_vec << " is " << toward_vec - from_vec
//			          << std::endl;
//			std::cout << "But the composition is " << (odometry * from->estimate()).toVector() << std::endl;
//			std::cout << "But the composition is " << (odom_tmp * from->estimate()).toVector() << std::endl;
//			int wait = 0;
//			std::cin >> wait;


//			std::cout << "Saving cov " << std::endl;
			//TODO : transpose to 3d and use in odometry!

			auto cov_msg = ndt_graph_localization.graph_map.factors[element - 1].covariance;

			std::vector<double>::const_iterator it;
			it = cov_msg.data.begin();
			ROS_DEBUG_STREAM("Cov size " << cov_msg.data.size() );
			assert(cov_msg.data.size() == 36); //6x6 matrix

			std::vector<double>::const_iterator it_2;
			it_2 = cov_msg.data.begin();
			ROS_DEBUG_STREAM( cov_msg.data.size() );
			assert(cov_msg.data.size() == 36);

			Eigen::MatrixXd cov_3d(6, 6);
			for (size_t i = 0; i < 6; ++i) {
				for (size_t j = 0; j < 6; ++j) {
					cov_3d(i, j) = cov_msg.data[(6 * i) + j];
				}
			}

//			std::cout << "Saving cov to 2d" << std::endl;
			Eigen::Matrix3d cov_2d;
			cov_2d << cov_3d(0, 0), cov_3d(0, 1), 0,
					cov_3d(1, 0), cov_3d(1, 1), 0,
					0, 0, cov_3d(5, 5);

// 		tf::matrixEigenToMsg(cov, ndt_graph.factors[element - 1].covariance);
//			std::cout << "Saving information " << std::endl;
			Eigen::Matrix3d information = cov_2d.inverse();

// 				if(noise_flag = true && i != 0){
// 					odometry = odometry * noise_se2;
// 				}

//			std::cout << "Saving odometry " << odometry.toVector() << " from " << from << " toward " << toward
//			          << " info " << information << " " << std::endl;
			addOdometry(odometry, from, toward, information);
//			std::cout << ">Done" << std::endl;
		}
	}

//	auto robot_localization_ptr = addLocalizationVertex(ndt_graph_localization, element, shared_map, robot_ptr);
	std::tuple<g2o::VertexSE2RobotLocalization*, std::shared_ptr<perception_oru::NDTMap> > tuple(robot_loc_ptr, shared_map);
	return tuple;

}

g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalizationVertex(
		const auto_complete_graph::GraphMapLocalizationMsg &ndt_graph_localization, int element,
		const std::shared_ptr<perception_oru::NDTMap> &shared_map, const g2o::SE2& robot_pose) {

	auto localization_msg = ndt_graph_localization.localizations[element];
	AASS::acg::Localization localization;
	AASS::acg::fromMessage(localization_msg, localization);
//	std::cout << "Reading MCL Localization position_in_robot_frame : " << localization.mean[0] << ", " << localization.mean[1] << ", " << localization.mean[2] << std::endl;
	g2o::SE2 se2loc_global_frame(localization.mean[0], localization.mean[1], localization.mean[2]);
	Eigen::Matrix3d cov = localization.cov;
	g2o::SE2 se2loc = robot_pose.inverse() * se2loc_global_frame ;

//	std::cout << (robot_pose * se2loc).toVector() << " == " << se2loc_global_frame.toVector() << std::endl;
//	assert( (robot_pose * se2loc).toVector() == se2loc_global_frame.toVector());


	auto affine3d = se2ToAffine3d(robot_pose, _z_elevation);
	return addRobotLocalization(robot_pose, affine3d, se2loc.toVector(), cov, shared_map);
	//No localization for now
//	addLocalization(se2loc, robot_ptr, localization.cov.inverse());

}



//TODO: refactor this!
void AASS::acg::AutoCompleteGraphLocalization::extractCornerNDTMap(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotLocalization* robot_localization)
{

//	int corners_added = 0, observations_added = 0;

	//HACK For now : we translate the Corner extracted and not the ndt-maps
	auto cells = map->getAllCellsShared();
//	std::cout << "got all cell shared " <<  cells.size() << std::endl;
	double x2, y2, z2;
	map->getCellSizeInMeters(x2, y2, z2);
//	std::cout << "got all cell sized" << std::endl;
	double cell_size = x2;

	std::vector<AASS::acg::NDTCornerGraphElement> corners_end;
	getAllCornersNDTTranslatedToGlobalAndRobotFrame(map, robot_localization, corners_end);

	/***************** ADD THE CORNERS INTO THE GRAPH***********************/

	Eigen::Matrix3d cov = robot_localization->getCovariance();
	//Remove the rotationnal component
	Eigen::Matrix2d cov_2d;
	cov_2d << cov(0,0), cov(0,1),
			cov(1,0), cov(1,1);

	//If corners are found too close we fuse them:
	std::set<g2o::VertexLandmarkNDT*> corner_seen_here;

	for(auto corner : corners_end){
//		corners_added++;
// 		std::cout << "checking corner : " << _nodes_landmark.size() << std::endl ;  /*corners_end[i].print()*/ ; std::cout << std::endl;
		bool seen = false;
		g2o::VertexLandmarkNDT* ptr_landmark_seen = NULL;
		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			g2o::Vector2 landmark = _nodes_landmark[j]->estimate();
			cv::Point2f point_land(landmark(0), landmark(1));

			double res = cv::norm(point_land - corner.position_in_robot_frame);

//			std::cout << "res : " << std::endl;

			//If we found the landmark, we save the data
			if( res < cell_size * 2){
				seen = true;
				ptr_landmark_seen = _nodes_landmark[j];
			}
		}

		auto ptr_find = corner_seen_here.find(ptr_landmark_seen);
		//If the corner was not added this turn around
		if(ptr_find == corner_seen_here.end()) {

			Eigen::Matrix3d eigen_vec = corner.getEigenVectors();
			Eigen::Vector3d eigen_val = corner.getEigenValues();
			perception_oru::ndt_feature_finder::EigenSort2D(eigen_val, eigen_vec);
			Eigen::Matrix2d eigen_vec_2d;
			eigen_vec_2d = eigen_vec.block(0,0,2,2);
			Eigen::Vector2d eigen_val_2d;
			eigen_val_2d = eigen_val.segment(0,2);

			Eigen::Matrix2d cov_landmark = getCovariance(eigen_vec_2d, eigen_val_2d);


			if (seen == false) {
//				std::cout << "New point " << i << std::endl;
// 			assert(i < ret_export.size());
				g2o::Vector2 position_globalframe;
				position_globalframe << corner.position_in_robot_frame.x, corner.position_in_robot_frame.y;
// 			g2o::VertexLandmarkNDT* ptr = addLandmarkPose(vec, ret_export[i].getMeanOpenCV(), 1);

				cv::Point2f p_observation;
//				std::cout << "New point " << i << std::endl;
				p_observation.x = corner.getObservations()(0);
//				std::cout << "New point " << i << std::endl;
				p_observation.y = corner.getObservations()(1);
//				std::cout << "New point " << i << std::endl;
				g2o::VertexLandmarkNDT *ptr = addLandmarkPose(position_globalframe, p_observation, 1);


//				ptr->setCovarianceObservation(cov_2d);


				ptr->addAnglesOrientations(corner.getAngles(), corner.getOrientations());
				ptr->first_seen_from = robot_localization;

				//TESTING to visualise which cells gave the corner
				ptr->cells_that_gave_it_1 = corner.cells1;
				ptr->cells_that_gave_it_2 = corner.cells2;
				ptr->robotpose_seen_from = robot_localization->estimate();
				//END OF TEST

				corner_seen_here.insert(ptr);
//				std::cout << "Adding corner " << ptr << " from " << corner.getNodeLinkedPtr() << " and " << robot_localization << std::endl;

				g2o::EdgeLandmark_malcolm* edge_landmark;
				if(_use_corner_covariance) {
					//Use the covariance given by the corner itself
					edge_landmark = addLandmarkObservation(corner.getObservations(), corner.getNodeLinkedPtr(), ptr, cov_landmark);
				}else{
					edge_landmark = addLandmarkObservation(corner.getObservations(), corner.getNodeLinkedPtr(), ptr);
				}
				edge_landmark->original_observation = corner.getObservations();

				//Last parmater will be usel so to make sur eit's unused it's set to -1
				ptr->addLocalization(robot_localization, edge_landmark, robot_localization->estimate().toVector(), robot_localization->getCovariance(), corner.getObservations(), corner.getObservations(), -1);

				//Add covariance given by MCL instead of the default one.
//				addLandmarkObservation(corner.getObservations(), robot_localization, ptr, cov_2d);
				//Link between the prior and MCL

//				addObservationMCLToPrior(ptr);

//				observations_added++;
//				observations_added++;

			} else {

				g2o::EdgeLandmark_malcolm* edge_landmark;


//				std::cout << "Point seen: " << ptr_landmark_seen << " from " << corner.getNodeLinkedPtr() << " and " << robot_localization << std::endl;
				if(_use_corner_covariance) {
					//Use the covariance given by the corner itself
					edge_landmark = addLandmarkObservation(corner.getObservations(), corner.getNodeLinkedPtr(), ptr_landmark_seen, cov_landmark);
				}else{
					edge_landmark = addLandmarkObservation(corner.getObservations(), corner.getNodeLinkedPtr(), ptr_landmark_seen);
				}
				edge_landmark->original_observation = corner.getObservations();

				ptr_landmark_seen->addLocalization(robot_localization, edge_landmark, robot_localization->estimate().toVector(), robot_localization->getCovariance(), corner.getObservations(), corner.getObservations(), -1);
				//Add covariance given by MCL instead of the default one.
//				addLandmarkObservation(corner.getObservations(), robot_localization, ptr_landmark_seen, cov_2d);
				//Link between the prior and MCL

//				addObservationMCLToPrior(ptr_landmark_seen);

//				observations_added++;
//				observations_added++;

				corner_seen_here.insert(ptr_landmark_seen);
			}
		}
		else{
//			std::cout << "Created in this loop" << std::endl;
		}
	}
//	assert(corners_added * 2 == observations_added);

}


void AASS::acg::AutoCompleteGraphLocalization::getAllCornersNDTTranslatedToGlobalAndRobotFrame(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotPose* robot_ptr, std::vector<AASS::acg::NDTCornerGraphElement>& corners_end)
{

	perception_oru::ndt_feature_finder::NDTCorner cornersExtractor;
//	std::cout << "hopidy" << std::endl;
	auto ret_export = cornersExtractor.getAllCorners(*map);
//	std::cout << "gotall corner" << std::endl;
// 	auto ret_opencv_point_corner = cornersExtractor.getAccurateCvCorners();
//	std::cout << "got all accurate corners" << std::endl;
// 	auto angles = cornersExtractor.getAngles();
// 	std::cout << "got all angles" << std::endl;

	auto it = ret_export.begin();

//	std::cout << "Found " << ret_export.size() << " corners " << std::endl;
	//Find all the observations :

	//**************** HACK: translate the corners now : **************//

// 	int count_tmp = 0;
	for(it ; it != ret_export.end() ; ++it){
//		std::cout << "Corner size " << it->getOrientations().size() << std::endl;
		//Limited to corners that possess an orientation.
		if(it->getOrientations().size() > 0){
			Eigen::Vector3d landmark_robot_frame;
			// 		vec << it->x, it->y, angles[count_tmp].second;
//			std::cout << "Corner size " << std::endl;
//			vec << it->getMeanOpenCV().x, it->getMeanOpenCV().y, it->getOrientations()[0];
			landmark_robot_frame << it->getMeanOpenCV().x, it->getMeanOpenCV().y, robot_ptr->estimate().toVector()(2);

//			std::cout << "Corner size " << std::endl;
			cv::Point2f p_out;
			Eigen::Vector3d landmark_globalframe;
			translateFromRobotFrameToGlobalFrame(landmark_robot_frame, robot_ptr->estimate(), landmark_globalframe);

//			std::cout << "Corner size " << std::endl;
			p_out.x = landmark_globalframe(0);
			p_out.y = landmark_globalframe(1);

			Eigen::Vector2d observation; observation << landmark_robot_frame(0), landmark_robot_frame(1);
			std::vector<double> orientations;
			std::vector<double> angles_width;

//			std::cout << "Corner size " << std::endl;
			double angle_landmark = it->getOrientations()[0];

//			std::cout << "Corner size " << std::endl;
			for(auto it_orientation = it->getOrientations().begin() ; it_orientation != it->getOrientations().end() ; ++it_orientation){
//				std::cout << "Pushing back orientation" << std::endl;
				orientations.push_back((*it_orientation));
			}
			for(auto it_angles = it->getAngles().begin() ; it_angles != it->getAngles().end() ; ++it_angles){
//				std::cout << "Pushing back angle" << std::endl;
				angles_width.push_back((*it_angles));
			}



			//***************************Old version for testing***************************//
			// 		Eigen::Vector2d trueObservation2d = robot_pos.inverse() * real_obs;
			// 		std::cout << trueObservation(0) << " == " << trueObservation2d(0) << std::endl;
			// 		assert(trueObservation(0) == trueObservation2d(0));
			// 		std::cout << trueObservation(1) << " == " << trueObservation2d(1) << std::endl;
			// 		assert(trueObservation(1) == trueObservation2d(1));
			// 		observation2d_test = trueObservation2d;
			// 		std::cout << observation(0) << " == " << observation2d_test(0) << " minus " << observation(0) - observation2d_test(0) << std::endl;
			// 		assert(trueObservation(0) == trueObservation2d(0));
			// 		std::cout << observation(1) << " == " << observation2d_test(1) << " minus " << observation(1) - observation2d_test(1) << std::endl;
			// 		int a ;
			// 		std::cin >> a;
			//******************************************************************************//



			// 		std::cout << "Node transfo " << ndt_graph.getNode(i).T.matrix() << std::endl;
//			std::cout << "Position node " << robot_ptr->estimate().toVector() << std::endl;
//			std::cout << " vec " << landmark_robot_frame << std::endl;
			// 		std::cout << "Well " << robot_pos.toVector() + vec << "==" << ndt_graph.getNode(i).T * vec << std::endl;

			//ATTENTION THIS IS NOT TRUE BUT REALLY CLOSE
			// 				assert (robot_pos + vec == ndt_graph.getNode(i).T * vec);

//			std::cout << "NEW POINT : "<< p_out << std::endl;

			NDTCornerGraphElement cor(p_out, *it);

//			std::cout << cor.getEigenValues()  << " == " << it->getEigenValues()  << std::endl;
//			std::cout << cor.getEigenVectors() << " == " << it->getEigenVectors() << std::endl;
			assert(cor.getEigenValues() == it->getEigenValues());
			assert(cor.getEigenVectors() == it->getEigenVectors());

			cor.addAllObserv(robot_ptr, observation);
			cor.cells1 = it->getCells1();
			cor.cells2 = it->getCells2();
			corners_end.push_back(cor);
			// 		count_tmp++;
		}

	}

}


void AASS::acg::AutoCompleteGraphLocalization::testInfoNonNul(const std::string& before) const {

//	std::cout << "Test info non nul after " << before << std::endl;
	auto idmapedges = _optimizable_graph.edges();

	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
		assert((*ite)->vertices().size() >= 1);

		g2o::EdgeLandmark_malcolm* ptr = dynamic_cast<g2o::EdgeLandmark_malcolm*>(*ite);
		g2o::EdgeXYPriorACG* ptr1 = dynamic_cast<g2o::EdgeXYPriorACG*>(*ite);
		g2o::EdgeOdometry_malcolm* ptr2 = dynamic_cast<g2o::EdgeOdometry_malcolm*>(*ite);
		g2o::EdgeLinkXY_malcolm* ptr3 = dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite);
		g2o::EdgeNDTCellObservation* ptr4 = dynamic_cast<g2o::EdgeNDTCellObservation*>(*ite);
		g2o::EdgeNDTCell* ptr5 = dynamic_cast<g2o::EdgeNDTCell*>(*ite);
//		g2o::EdgeLocalization* ptr4 = dynamic_cast<g2o::EdgeLocalization*>(*ite);
		if(ptr != NULL){
			assert(ptr->information().isZero(1e-10) == false);
		}
		else if(ptr1 != NULL){
			assert(ptr1->information().isZero(1e-10) == false);
		}
		else if(ptr2 != NULL){
			assert(ptr2->information().isZero(1e-10) == false);
		}
		else if(ptr3 != NULL){
			assert(ptr3->information().isZero(1e-10) == false);
		}
		else if(ptr4 != NULL){
//			std::cout << "EDGE NDT CELL OBSERVATION" << std::endl;
//			std::cout << "INFO : " << ptr3->information() << std::endl;
			assert(ptr4->information().isZero(1e-10) == false);
		}
		else if(ptr5 != NULL){
//			std::cout << "EDGE NDT CELL" << std::endl;
			assert(ptr5->information().isZero(1e-10) == false);
		}
//		else if(ptr4 != NULL){
//			assert(ptr4->information().isZero(1e-10) == false);
//		}
		else{
			throw std::runtime_error("Didn't find the type of the edge :S");
		}
	}

	assert(verifyInformationMatrices(true) == true);


}



std::tuple<Eigen::Affine3d, Eigen::MatrixXd> AASS::acg::AutoCompleteGraphLocalization::registerSubmaps(const g2o::VertexSE2RobotPose& from, const g2o::VertexSE2RobotPose& toward, Eigen::Affine3d &transformation, int nb_neighbor, const Eigen::MatrixXd& default_cov) {
//	void ndt_feature::NDTFeatureGraph::registerSubmaps(NDTFeatureLink &link, int nb_neighbours, bool keepScore) {
	perception_oru::NDTMatcherD2D matcher_d2d;
	matcher_d2d.n_neighbours = nb_neighbor;

//	std::cout << "Matching : " << from.getIndexGraphMap() << " with " << toward.getIndexGraphMap() << std::endl;
//		Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
//	std::cout << "Transform between the two " <<transformation.matrix() << std::endl;
//	int a ;
// 	std::cout << "PAUSE before match" << std::endl;
// 	std::cin >> a;

	Eigen::Affine3d before_T = transformation;

	bool converged = matcher_d2d.match(*(from.getMap().get()), *(toward.getMap().get()), transformation, true);

//	std::cout << "Transform between the two new " <<transformation.matrix() << std::endl;

// 	int a ;
// 	std::cout << "PAUSE before cov" << std::endl;
// 	std::cin >> a;

	//Adding the covariance into the link
	Eigen::MatrixXd cov(6,6);

	bool same = true;
	for(size_t i = 0; i < 4 ; ++i){

		for(size_t  j = 0 ; j < 4 ; ++j){
			if(before_T(i,j) != transformation(i,j)){
				same = false;
			}
		}
	}

	if(!same){
		cov.setZero();
		matcher_d2d.covariance(*(from.getMap().get()), *(toward.getMap().get()), transformation, cov);
	}
	else{
		ROS_INFO_STREAM( "NOTHING HAPPENED using default matrix" );
		cov = default_cov;
		if(!testSPD(cov)){
			std::cout << "THIS IS BAD COV GIVEN WAS NOT SPD, switching to default" << std::endl;
			cov << 0.1, 0, 0, 0, 0, 0,
					0, 0.1, 0, 0, 0, 0,
					0, 0, 0.1, 0, 0, 0,
					0, 0, 0, 0.1, 0, 0,
					0, 0, 0, 0, 0.1, 0,
					0, 0, 0, 0, 0, 0.1;
		}

		int a;
		std::cin >> a;
// 		exit(0);
	}
//	std::cout << "Size of Covariance : " << cov.rows() << " AND COLS " << cov.cols() << std::endl;

	assert(cov.rows() == 6);
	assert(cov.cols() == 6);
//	std::cout << "PAUSE got cov : " << cov << "\n";
//	std::cout << "COVARIANCE BY MATCHER " << cov.inverse() << "\n";

	if(!converged){
// 		throw std::runtime_error("ndt_map registration didn't converge");
		ROS_ERROR_STREAM( "USing odometry input a number to continue. That is bad sometimes :/" );

		transformation = before_T;
		cov = default_cov;
		if(!testSPD(cov)){
			std::cout << "THIS IS BAD COV GIVEN WAS NOT SPD, switching to default" << std::endl;
			cov << 0.1, 0, 0, 0, 0, 0,
					0, 0.1, 0, 0, 0, 0,
					0, 0, 0.1, 0, 0, 0,
					0, 0, 0, 0.1, 0, 0,
					0, 0, 0, 0, 0.1, 0,
					0, 0, 0, 0, 0, 0.1;
		}

		int a;
		std::cin >> a;
	}


	std::cout << "Testing COV after registration" << std::endl;
	if(!testSPD(cov)){
//		throw std::runtime_error("ndt_map registration didn't converge");
		ROS_ERROR_STREAM( "Cov is not SDP. That is bad :/" );

		transformation = before_T;
		cov = default_cov;
		if(!testSPD(cov)){
			std::cout << "THIS IS BAD COV GIVEN WAS NOT SPD, switching to default" << std::endl;
			cov << 0.1, 0, 0, 0, 0, 0,
					0, 0.1, 0, 0, 0, 0,
					0, 0, 0.1, 0, 0, 0,
					0, 0, 0, 0.1, 0, 0,
					0, 0, 0, 0, 0.1, 0,
					0, 0, 0, 0, 0, 0.1;
		}

		int a;
		std::cin >> a;
	}


// 	exit(0);
// 	std::cin >> a;

//	std::cout << "End of link" << std::endl;

	return std::tuple<Eigen::Affine3d, Eigen::MatrixXd>(transformation, cov);
}



g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraphLocalization::addLinkBetweenMaps(const g2o::Vector2& pos, g2o::VertexXYPrior* v2, g2o::VertexLandmarkNDT* v1, const g2o::VertexSE2RobotLocalization* mcl_pose){

	throw std::runtime_error("Those links are not usable anymore in Localization. Use AutoCompleteGraph for that");
//	std::cout << "Adding link" << std::endl;
//
//	for(auto link_edge : _edge_link){
//		if(v1 == link_edge->vertices()[0] && v2 == link_edge->vertices()[1]){
//			throw std::runtime_error("Edge link already added");
//		}
//		else if(v1 == link_edge->vertices()[1] && v2 == link_edge->vertices()[0]){
//			throw std::runtime_error("Edge link already added");
//		}
//	}
//
//
//	Eigen::Matrix3d covariance_link_tmp=  v1->getCovarianceObservation(mcl_pose);
//	Eigen::Matrix2d covariance_link =  covariance_link_tmp.block(0, 0, 2, 2);
////	covariance_link.fill(0.);
////	covariance_link(0, 0) = _linkNoise[0]*_linkNoise[0];
////	covariance_link(1, 1) = _linkNoise[1]*_linkNoise[1];
//
//// 	std::cout << "Link cov " << covariance_link << std::endl;
//
//// 			covariance_link(2, 2) = 13;//<- Rotation covariance link is more than 4PI
//	Eigen::Matrix2d information_link = covariance_link.inverse();
//
//	assert(information_link.isZero(1e-10) == false);
//// 	std::cout << "Link cov2 " << covariance_link << std::endl;
//
//	g2o::EdgeLinkXY_malcolm* linkObservation = new g2o::EdgeLinkXY_malcolm;
//
//// 	std::cout << "Link cov3 " << covariance_link << std::endl;
//// 	g2o::HyperGraph::Vertex* from = dynamic_cast<g2o::HyperGraph::Vertex*>(v2);
//	assert(v2 != NULL);
//	linkObservation->vertices()[0] = v2;
//	assert(linkObservation->vertices()[0] == v2);
//// 	std::cout << "Link cov4 " << covariance_link << std::endl;
//// 	g2o::HyperGraph::Vertex* toward = dynamic_cast<g2o::HyperGraph::Vertex*>(v1);
//	assert(v1 != NULL);
//	linkObservation->vertices()[1] = v1;
//	assert(linkObservation->vertices()[1] == v1);
//// 	std::cout << "Link cov5 " << covariance_link << std::endl;
//	linkObservation->setMeasurement(pos);
//// 	std::cout << "Link cov6 " << covariance_link << std::endl;
//	linkObservation->setInformation(information_link);
//// 	std::cout << "Link cov7 " << _sensorOffset->id() << std::endl;
//	linkObservation->setParameterId(0, _sensorOffset->id());
//
//// 	std::cout << "Adding edge!" << v2 << " and " << linkObservation->vertices()[0] << " at " << __LINE__ << " " << __FILE__<< "age start " << _age_start_value << " for " << /*linkObservation->interface->getAge() <<*/ std::endl;
//	//Adding the link age
//
//	//ATTENTION NOT WORKING FOR I DON'T KNOW WHAT REASON
//// 	bool outout = false;
//// 	std::cout << "Bool" << outout << std::endl;
//// 	outout = linkObservation->interface->setAge(_age_start_value);
//// 	std::cout << "Bool" << outout << std::endl;
//// 	assert(outout == 1);
//// 	std::cout << "Age set" << std::endl;
//// 	assert(linkObservation->interface->manuallySetAge() == true);
//
//	//HACK REPLACEMENT
//	EdgeInterface edgeinter;
//// 	std::cout << "Setting age value" << std::endl;
//// 	std::cout << "Setting age value" << std::endl;
//	edgeinter.setAge(_age_start_value);
//// 	std::cout << "Manueal" << std::endl;
//	assert(edgeinter.manuallySetAge() == true);
//
//	_optimizable_graph.addEdge(linkObservation);
//	_edge_link.push_back(linkObservation);
//	_edge_interface_of_links.push_back(edgeinter);
//
//	assert(_edge_interface_of_links.size() == _edge_link.size());
//
//// 	std::cout << "Done" << std::endl;
//	return linkObservation;
}

void AASS::acg::AutoCompleteGraphLocalization::AddObservationsMCLPrior() {

}


void AASS::acg::AutoCompleteGraphLocalization::addObservationMCLToPrior(const g2o::VertexLandmarkNDT* landmark) {

//	std::cout << "Start for landmark " << landmark << " at pose " << landmark->estimate() << std::endl;
//
//	if(_use_mcl_cov_to_find_prior_observed) {
//		for (auto prior_corner : _prior->getNodes()) {
//			std::cout << "NEW PRIOR" << std::endl;

//			if (_use_mcl_observation_on_prior) {
//
//				std::unordered_set <std::shared_ptr<AASS::acg::LocalizationPointer>> localization = landmark->getLocalization();
//
//				for (auto loc: localization) {
//
//					//The landmark pose should be in the mcl pose frame and not the robot mapping frame.
//					Eigen::Vector2d pose_landmark = loc->landmarkSeenByMCLInGlobalFrame().head(2);
//					Eigen::Vector2d pose_landmark_test = landmark->estimate();
//					Eigen::Vector2d pose_prior = prior_corner->estimate();
//
//					double euclidean_distance = (pose_prior - pose_landmark).norm();
//					std::cout << "Poses : " << pose_prior << " - " << pose_landmark << " ( actual landmark pose: "
//					          << pose_landmark_test << " ) distance " << euclidean_distance << std::endl;
//
//					if (euclidean_distance <= 5) {
//
//						auto is_found = std::find(loc->_prior_linked_using_landmark.begin(),
//						                          loc->_prior_linked_using_landmark.end(), prior_corner);
//						if (is_found == loc->_prior_linked_using_landmark.end()) {
////				loc->vertex_mcl_pose->estimate().toVector().head(2);
////				assert(pose_landmark(0) == loc->vertex->estimate().toVector().head(2)(0));
////				assert(pose_landmark(1) == loc->vertex->estimate().toVector().head(2)(1));
//
//							//Create rotation covariance
////						assert(loc->cov(2, 2) >= -1);
////						assert(loc->cov(2, 2) <= 1);
//
////						Eigen::Rotation2D<double> rot2(loc->cov(2, 2));
//
//							//TODO : TO TEST
//							Eigen::Matrix2d cov_inv_2d;
//							bool test = false;
//							if (test == true) {
//
//								Eigen::Vector2d mcl_pose = loc->vertex_mcl_pose->estimate().toVector().head(2);
//								double euclidean_distance_to_mcl = (pose_landmark - mcl_pose).norm();
//								double opposite_side = euclidean_distance_to_mcl * std::tan(loc->cov(2, 2));
//								Eigen::Vector2d eigen = (pose_landmark - mcl_pose) / euclidean_distance_to_mcl;
//								Eigen::Vector2d main_eigen;
//								main_eigen << -eigen(1), eigen(0);
//
//								Eigen::Matrix2d eigenvec;
//								eigenvec << main_eigen(0), eigen(0),
//										main_eigen(1), eigen(1);
//
//								//For testing
//								std::pair<double, double> eigenval;
//								eigenval.first = opposite_side;
//								eigenval.second = opposite_side / 10;
//								Eigen::Matrix2d cov_rotation = getCovariance(eigenvec, eigenval);
//
//								Eigen::Matrix2d cov_2d;
//								cov_2d << loc->cov(0, 0) / _scaling_factor_gaussian, loc->cov(0, 1) /
//								                                                     _scaling_factor_gaussian,
//										loc->cov(1, 0) / _scaling_factor_gaussian, loc->cov(1, 1) /
//								                                                   _scaling_factor_gaussian;
//								cov_2d = cov_2d + cov_rotation;
//								cov_2d = cov_2d / 2;
//
////							Eigen::Matrix2d cov_inv_2d;
//								bool exist = false;
//								cov_2d.computeInverseWithCheck(cov_inv_2d, exist);
//								assert(exist == true);
//
//							} else {
////							Eigen::Matrix2d icov = loc.cov_inverse();
//
//								//https://math.stackexchange.com/questions/708994/proof-of-the-inverse-of-a-scalar-times-a-matrix
//								//Divided by 2 so that we double the size of the covariance
////						Eigen::Matrix2d cov_inv_2d;
//								cov_inv_2d << loc->cov_inverse(0, 0) / _scaling_factor_gaussian,
//										loc->cov_inverse(0, 1) / _scaling_factor_gaussian,
//										loc->cov_inverse(1, 0) / _scaling_factor_gaussian, loc->cov_inverse(1, 1) /
//								                                                           _scaling_factor_gaussian;
//							}
//							double probabilistic_distance = (pose_prior - pose_landmark).dot(
//									cov_inv_2d * (pose_prior - pose_landmark));
//
//							//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
//
//							//That score is stupid as a measurement measure
//							//					double score = 0.1 + 0.9 * exp(-_scaling_factor_gaussian * probabilistic_distance); /// / 2.0);
//
//							//Instead we will say that if the probabilistic distance is less than the euclidean distance, we associated them ! That makes a lot more sense. Indeed, it is less if each points are "in the covariance. See test_prior_localization.cpp.
//
//							std::cout << "Poses : " << pose_prior << " - " << pose_landmark
//							          << " ( actual landmark pose: " << pose_landmark_test << " ) inverse cov\n"
//							          << cov_inv_2d.matrix() << "\n and cov\n" << loc->cov.matrix()
//							          << " \nprobabilistic_distance " << probabilistic_distance
//							          << " euclidean distance " << euclidean_distance << " threshold "
//							          << _threshold_of_score_for_creating_a_link << std::endl;
//
//							if (probabilistic_distance <= euclidean_distance) {
////					if (score >= _threshold_of_score_for_creating_a_link) {
//								//					g2o::Vector2 vec;
//								//					vec << 0, 0;
//
//								//					throw std::runtime_error("DO not use MCL observation yet");
//
//								//ATTENTION MASSIVE TODO
//								//					g2o::VertexSE2RobotPose* rpose = loc->vertex->getEquivalentRobotPose();
//								//					g2o::EdgeLandmark_malcolm* observation = findObservation(rpose, landmark);
//
//								//TODO USE MCL COVARIANCE
//								g2o::EdgeLandmark_malcolm *observation_mcl = addLandmarkObservation(loc->observation,
//								                                                                    loc->vertex_mcl_pose,
//								                                                                    prior_corner,
//																									loc->cov.block(0,0,2,2));
//								//To know link was already done !
////							loc->link_created = true;
//								loc->_prior_linked_using_landmark.insert(prior_corner);
//								bool test_pointer_type = false;
//								for (auto vertex : observation_mcl->vertices()) {
//									g2o::VertexXYPrior *ptr = dynamic_cast<g2o::VertexXYPrior *>(vertex);
//									if (ptr != NULL) {
//										test_pointer_type = true;
//									}
//								}
//								assert(test_pointer_type == true);
//
//								bool test_pointer_type2 = false;
//								for (auto vertex : observation_mcl->vertices()) {
//									g2o::VertexSE2RobotLocalization *ptr = dynamic_cast<g2o::VertexSE2RobotLocalization *>(vertex);
//									if (ptr != NULL) {
//										test_pointer_type2 = true;
//									}
//								}
//								assert(test_pointer_type2 == true);
//
//								prior_corner->landmarks.insert(landmark);
//								_number_of_links_to_prior++;
//								//									addPriorObservation();
//								//									addLinkBetweenMaps(vec, prior_node, landmark, loc.vertex);
//							}
//						}
//
//					}
//				}
//
//			} else {
//				std::cout << "What else to use :( " << std::endl;
//				throw std::runtime_error("no matching function");
//			}
//
//		}
//	}
//	else{

//		std::cout << "New corner matching " << std::endl;

		std::unordered_set <std::shared_ptr<AASS::acg::LocalizationPointer>> localization = landmark->getLocalization();

		for (auto loc: localization) {

//			assert(loc->vertex_robot_pose == loc->vertex_mcl_pose->getEquivalentRobotPose());

//			std::cout << "Number of landmark observation" << _edge_landmark.size() << std::endl;

			for(auto edge : _edge_landmark){

				if(loc->vertex_mcl_pose == edge->vertices()[0] && landmark == edge->vertices()[1] || loc->vertex_mcl_pose == edge->vertices()[1] && landmark == edge->vertices()[0]){
					if(loc->observation != edge->original_observation){
						ROS_ERROR_STREAM( "Not the same ? " << loc->observation << " == " << edge->original_observation );
					}
					assert(loc->observation == edge->original_observation);
				}

			}


			//The landmark pose should be in the mcl pose frame and not the robot mapping frame.
			Eigen::Vector2d pose_landmark = loc->landmarkSeenByMCLInGlobalFrame().head(2);
			Eigen::Vector2d pose_landmark_test = landmark->estimate();
			//Other strategy based on edge
//			if (landmarkgaussianonanedge){
//				for (auto corner_prior : edge->vertices()) {
//					cov = cov_mcl * eucliddistance;
//				}
//			}
//			else{
				for (auto prior_corner : _prior->getNodes()) {

					auto is_found = std::find(loc->_prior_linked_using_landmark.begin(),
					                          loc->_prior_linked_using_landmark.end(), prior_corner);

					if (is_found == loc->_prior_linked_using_landmark.end()) {

						Eigen::Vector2d pose_prior = prior_corner->estimate();
						double euclidean_distance = (pose_prior - pose_landmark).norm();

						if (euclidean_distance <= 5) {

							auto angles_width = prior_corner->getAnglesAndOrientations();
//							assert(angles_width.size() > 0);

							if (this->_flag_use_corner_orientation == false ||
							    (this->_flag_use_corner_orientation == true &&
							     landmark->sameOrientation(angles_width) )) {

								Eigen::Matrix2d cov_inv_2d =
										loc->cov_inverse.block(0, 0, 2, 2) / _scaling_factor_gaussian;
//								double probabilistic_distance = (pose_prior - pose_landmark).dot(
//										cov_inv_2d * (pose_prior - pose_landmark));

								//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)

								//That score is stupid as a measurement measure
								//					double score = 0.1 + 0.9 * exp(-_scaling_factor_gaussian * probabilistic_distance); /// / 2.0);

								//Instead we will say that if the probabilistic distance is less than the euclidean distance, we associated them ! That makes a lot more sense. Indeed, it is less if each points are "in the covariance. See test_prior_localization.cpp.



								//Calculating collision probability as in the space paper. Doesn't work so no citation for you :(. Just remove the factor and we actually get the proba distribution ;)

								double radius_around_landmark = 1;
//								double factor = radius_around_landmark * radius_around_landmark / (2 * std::sqrt(loc->determinant) );
								double mahalanobis_distance = (pose_prior - pose_landmark).dot( cov_inv_2d * (pose_prior - pose_landmark));

								double pdf_collision = std::exp( - mahalanobis_distance ) ;

								assert(pdf_collision >= 0);
								assert(pdf_collision <= 1);

//								std::cout << "Poses : " << pose_prior << " - " << pose_landmark
//								          << " ( actual landmark pose: " << pose_landmark_test << " ) inverse cov\n"
//								          << cov_inv_2d.matrix() << "\n and cov\n" << loc->cov.matrix()
//								          << " \nprobabilistic_distance " << mahalanobis_distance
//		                                    << " euclidean distance " << euclidean_distance << " threshold "
//		                                    << " score " << pdf_collision << " threshold "
//								          << _threshold_of_score_for_creating_a_link << std::endl;


//								if (mahalanobis_distance <= euclidean_distance) {
								if(mahalanobis_distance <= 5.99){
//								if (pdf_collision >= _threshold_of_score_for_creating_a_link) {


//									std::cout << "CREATING OBSERVATION " << std::endl;
									//Cov is cov factor square of distance
									Eigen::Matrix2d cov =
											loc->cov.block(0, 0, 2, 2); // * (euclidean_distance * euclidean_distance);

//									std::cout << "Mcl cov " << loc->cov.block(0, 0, 2, 2).matrix()
//									          << " euclidean distance "
//									          << euclidean_distance << " cov used for observation " << cov.matrix()
//									          << std::endl;


									//Making sure the observation doesn't already exist !
									for(auto edge : _edge_landmark){
										if(loc->vertex_mcl_pose == edge->vertices()[0] && prior_corner == edge->vertices()[1] || loc->vertex_mcl_pose == edge->vertices()[1] && prior_corner == edge->vertices()[0]){
											throw std::runtime_error("OBSERVATION OF THE PRIOR ALREADY EXIST :( !!!!");
										}

									}


									g2o::EdgeLandmark_malcolm *current_landmark_observation;
									//Making the observatio correspond to the original observation
									for(auto edge : _edge_landmark) {
										if (loc->vertex_mcl_pose == edge->vertices()[0] &&
										    landmark == edge->vertices()[1] ||
										    loc->vertex_mcl_pose == edge->vertices()[1] &&
										    landmark == edge->vertices()[0]) {

											if (loc->observation != edge->original_observation) {
												ROS_INFO_STREAM("Not the same ? " << loc->observation << " == "
												          << edge->original_observation );
											}

											current_landmark_observation = edge;

											assert(loc->observation == edge->original_observation);
										}
									}

//									std::cout << "Old observation " << loc->observation << " and new observation " << current_landmark_observation->measurement() << std::endl;

									g2o::EdgePriorObservation *observation_mcl = addPriorObservation(
											loc->observation,
											loc->vertex_mcl_pose,
											prior_corner, cov, loc->edge_observation_from_mcl_pose);
									//To know link was already done !
//							loc->link_created = true;
									loc->_prior_linked_using_landmark.insert(prior_corner);

//									prior_corner->landmarks.insert(landmark);
									_number_of_links_to_prior++;

//									int aa;
//									std::cin >> aa;

								}

							}
						}

					}
//				}
			}
		}
//	}

//	std::cout << "Stop" << std::endl;
//	int aaa;
//	std::cin >> aaa;


}

//void AASS::acg::AutoCompleteGraphLocalization::addObservationMCLToPrior(const g2o::VertexLandmarkNDT* landmark) const {
//
//	for(auto prior_corner : _prior->getNodes() ){
//
//		if (_use_mcl_observation_on_prior) {
//
//			std::unordered_set<std::shared_ptr<AASS::acg::LocalizationPointer> > localization = landmark->getLocalization();
//			for (auto loc: localization) {
//
//				Eigen::Vector2d pose_landmark = loc->vertex->estimate().toVector().head(2);
//
//				assert(pose_landmark(0) == loc->vertex->estimate().toVector().head(2)(0));
//				assert(pose_landmark(1) == loc->vertex->estimate().toVector().head(2)(1));
//
//				Eigen::Vector2d pose_prior = prior_corner->estimate();
////							Eigen::Matrix2d icov = loc.cov_inverse();
//				Eigen::Matrix2d cov_inv_2d;
//				cov_inv_2d << loc->cov_inverse(0, 0), loc->cov_inverse(0, 1),
//						loc->cov_inverse(1, 0), loc->cov_inverse(1, 1);
//
//				double l = (pose_prior - pose_landmark).dot(cov_inv_2d * (pose_prior - pose_landmark));
//				double score = 0.1 + 0.9 * exp(-_scaling_factor_gaussian * l / 2.0);
//
//				if (score >= _threshold_of_score_for_creating_a_link) {
////					g2o::Vector2 vec;
////					vec << 0, 0;
//
////					throw std::runtime_error("DO not use MCL observation yet");
//
//					//ATTENTION MASSIVE TODO
//					g2o::VertexSE2RobotPose* rpose = loc->vertex->getEquivalentRobotPose();
//
//					g2o::EdgeLandmark_malcolm* observation = findObservation(rpose, landmark);
//
//					g2o::EdgeLandmark_malcolm* observation_mcl = addLandmarkObservation(observation->measurement(), loc->vertex, prior_corner);
//
////									addPriorObservation();
////									addLinkBetweenMaps(vec, prior_node, landmark, loc.vertex);
//				}
//			}
//		}
//		else{
//			std::cout << "What else to use :( " << std::endl;
//			throw std::runtime_error("no mathcing function");
//		}
//
//	}
//
//}



//int AASS::acg::AutoCompleteGraphLocalization::createNewLinks()
//{
//	std::cout << "Creating the new links in Localization" << std::endl;
//	int count = 0;
//	if(_use_links_prior) {
//// 	std::vector < std::pair < g2o::VertexLandmarkNDT*, g2o::VertexSE2Prior*> > links;
//
//		std::cout << "Number new landmarks " << _nodes_landmark.size() << std::endl;
//		std::cout << "Prior " << _prior->getNodes().size() << std::endl;
//// 	if(_nodes_prior.size() > 0){
//
//		//Update ALL links
////		auto it = _nodes_landmark.begin();
//		for (auto landmark : _nodes_landmark) {
//// 		std::cout << "Working on links " << std::endl;
//			Eigen::Vector2d pose_landmark = landmark->estimate();
////			auto it_prior = _nodes_prior.begin();
//			for (auto prior_node : _prior->getNodes()) {
//
//				//Don't add the same link twice
//				if (linkAlreadyExist(landmark, prior_node) == false) {
//
//					//TODO TEST IT
//					if (_flag_use_corner_orientation == false ||
//					    (_flag_use_corner_orientation == true && landmark->sameOrientation(prior_node->getAnglesAndOrientations() ))) {
//
//						Eigen::Vector2d pose_prior = prior_node->estimate();
////						Eigen::Vector2d pose_prior;
////						pose_prior << pose_tmp(0), pose_tmp(1);
//
//						if (_use_links_prior_classic_ssrr) {
//
//							std::cout << "Using classic SSRR links please switch to AutoCompleteGraph" << std::endl;
//							throw std::runtime_error("g2o::VertexXYPrior. If that's what you wish");
//
//							double norm_tmp = (pose_prior - pose_landmark).norm();
//							// 				std::cout << "new" << std::endl;
//
//							if (_use_covariance_for_links) {
//
//								throw std::runtime_error(
//										"This parameter is not usable. The reason for this is that one obstacle can be seen from multiple MCL poses. Hence it is impossible to choose which MCL covariance to use for one link which can be created from multiple MCL poses. Please use the _use_mcl_observation_on_prior parameter ;).");
////							//Find distance needed using covariance of corner. STOLEN FROM LOCALIZATION *monkey which hides it eyes*
////							Eigen::Matrix2d icov = landmark->getInverseCovarianceObservation();
////							double l = (pose_prior - pose_landmark).dot(icov * (pose_prior - pose_landmark));
////							double score = 0.1 + 0.9 * exp(-_scaling_factor_gaussian * l / 2.0);
////
////							if(score >= _threshold_of_score_for_creating_a_link){
////								g2o::Vector2 vec;
////								vec << 0, 0;
////								addLinkBetweenMaps(vec, prior_node, landmark, );
////							}
//							}
//
//								//Update the link
//							else if (norm_tmp <= _min_distance_for_link_in_meter) {
//								std::cout << "NORM " << norm_tmp << "min dist " << _min_distance_for_link_in_meter
//								          << std::endl;
//								// 					ptr_closest = *it_prior;
//								// 					norm = norm_tmp;
//								//Pushing the link
//								// 					std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
//								// 					links.push_back(std::pair<g2o::VertexLandmarkNDT*, g2o::VertexSE2Prior*>(*it, *it_prior));
//
//								g2o::Vector2 vec;
//								vec << 0, 0;
//								addLinkBetweenMaps(vec, prior_node, landmark);
//
//								++count;
//							}
//						}
//
//						if (_use_mcl_observation_on_prior) {
//
//							std::unordered_set<std::shared_ptr<AASS::acg::LocalizationPointer> > localization = landmark->getLocalization();
//							for (auto loc: localization) {
//
////							Eigen::Matrix2d icov = loc.cov_inverse();
//								Eigen::Matrix2d cov_2d;
//								cov_2d << loc->cov_inverse(0,0), loc->cov_inverse(0,1),
//										 loc->cov_inverse(1,0), loc->cov_inverse(1,1);
//
//										double l = (pose_prior - pose_landmark).dot( cov_2d * (pose_prior - pose_landmark));
//								double score = 0.1 + 0.9 * exp(-_scaling_factor_gaussian * l / 2.0);
//
//								if (score >= _threshold_of_score_for_creating_a_link) {
//									g2o::Vector2 vec;
//									vec << 0, 0;
//
//									throw std::runtime_error("DO not use MCL observation yet");
//
//									//ATTENTION MASSIVE TODO
//
//
////									addPriorObservation();
////									addLinkBetweenMaps(vec, prior_node, landmark, loc.vertex);
//								}
//							}
//						}
//					} else {
//
//						std::cout << "Not same orientation" << std::endl;
//					}
//				} else {
//
//					std::cout << "Already exist" << std::endl;
//				}
//			}
//			//Pushing the link
//// 			std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
//// 			links.push_back(std::pair<g2o::VertexLandmarkNDT*, g2o::VertexSE2Prior*>(*it, ptr_closest));
//		}
//
//// 	auto it_links = links.begin();
//// 	for(it_links ; it_links != links.end() ; it_links++){
//// 		g2o::Vector2 vec;
//// 		vec << 0, 0;
//// 		std::cout << "Creating " << it_links->second << " and " << it_links->first << std::endl;
//// 		addLinkBetweenMaps(vec, it_links->second, it_links->first);
//// 	}
//// 	}
//	}
//
//		return count;
//
//}


void AASS::acg::AutoCompleteGraphLocalization::updatePriorObservations() {

	updateExistingPriorObservations();
	for(auto landmark : _nodes_landmark){
		addObservationMCLToPrior(landmark);
	}
}

void AASS::acg::AutoCompleteGraphLocalization::updateExistingPriorObservations() {

	for(auto edge_observation : _edge_prior_observation){

		g2o::EdgeLandmark_malcolm* equiv = edge_observation->equivalent_landmark_observation_edge;
		edge_observation->setMeasurement(equiv->measurement());

	}
}


void AASS::acg::AutoCompleteGraphLocalization::createWallAssociations(){

	for (auto robot : _nodes_localization){
		createWallAssociations(robot);
	}

}

void AASS::acg::AutoCompleteGraphLocalization::createWallAssociations(g2o::VertexSE2RobotLocalization* robot){

	auto map = robot->getMap();
	double cx, cy, cz;
	map->getCellSizeInMeters(cx, cy, cz);
	double minval = std::min(cx, std::min(cy, cz));

	std::cout << "Create link to robot map" << std::endl;
	int count_walls = 0;
	for(auto wall : _prior->getEdges()){

		std::vector<std::tuple< g2o::VertexNDTCell*, Eigen::Vector2d, double> > already_created_ndt_cells_to_update;
		auto cells = collisionsNDTMapWithPriorEdge(*robot, *wall, already_created_ndt_cells_to_update);
//		std::cout << "Found links " << cells.size() << std::endl;
		for(auto cell : cells){

			auto ndtcell_ver  = addNDTCellVertex(std::get<1>(cell), std::get<0>(cell), robot);

			//Use cell size for covariance
//			double cx, cy, cz;
//			robot->getMap()->getCellSizeInMeters(cx, cy, cz);
//			double minval = std::min(cx, std::min(cy, cz));

//			Eigen::Matrix3d ndt_cov = cell.first->getCov();
			Eigen::Matrix3d ndt_cov = robot->getCovariance() * _scaling_factor_gaussian;
			Eigen::Matrix2d ndt_cov_2d;
			ndt_cov_2d << ndt_cov(0,0), ndt_cov(0,1),
					ndt_cov(1,0), ndt_cov(1,1);

			double coeff_mult = 1;
			if(std::get<2>(cell) >= 1){
				coeff_mult = std::get<2>(cell);
			}else{
				coeff_mult = coeff_mult + std::get<2>(cell);
			}
//			coeff_mult = coeff_mult * cells.size();

			ndt_cov_2d.array() = ndt_cov_2d.array() * coeff_mult;

			auto ndtcell_edge = addNDTCellAssociation(ndtcell_ver, wall, ndt_cov_2d);
			ndtcell_edge->resolution_of_map = minval;

			count_walls++;

		}
		//DOESN'T WORK IN THE OPTIMIZATION STEP: Use bundle of NDT Cells
//		for (auto cell_node : already_created_ndt_cells_to_update){
//
//			auto link_ndt_cell_exist = [this] (g2o::HyperGraph::Vertex* v_ndt_cell, g2o::EdgeXYPriorACG* wall) -> bool  {
//				//Check if already exist
//
//				for (auto edge_ndt : _edges_ndt_cell) {
//					if( (wall->vertices()[0] == edge_ndt->vertices()[0] && v_ndt_cell == edge_ndt->vertices()[1] ) ||
//					    (wall->vertices()[0] == edge_ndt->vertices()[1] && v_ndt_cell == edge_ndt->vertices()[0] )	){
//						return true;
//					}
//				}
//				return false;
//			};
//			auto ndt_cell_observed = [this] (g2o::HyperGraph::Vertex* v_ndt_cell, g2o::VertexSE2RobotLocalization* v_robot) -> bool  {
//				//Check if already exist
//
//				for (auto edge_ndt : _edges_ndt_cell_observation) {
//					if( (v_robot == edge_ndt->vertices()[0] && v_ndt_cell == edge_ndt->vertices()[1] ) ||
//					    (v_robot == edge_ndt->vertices()[1] && v_ndt_cell == edge_ndt->vertices()[0] )	){
//						return true;
//					}
//				}
//				return false;
//			};
//
//			//Link the cell to all robot pose
//			for(auto cells : std::get<0>(cell_node)->getEquivalentNDTCells()) {
//				if(!ndt_cell_observed()) {
//					Eigen::Vector2d pose_landmark = std::get<0>(cell_node)->getCell()->getMean().head(2);
////	auto edge = addNDTCellObservation(pose_landmark, robot_node, ndt_cell_v, cell->getCov().block(0,0,2,2));
//					auto edge = addNDTCellObservation(pose_landmark, robot, std::get<0>(cell_node));
//				}
//			}
//
//			if(!link_ndt_cell_exist(std::get<0>(cell_node), wall)) {
//
//				std::cout << "New ndtcell double link" << std::endl;
//				int a = 0;
//				std::cin >> a;
//
//				auto ndtcell_ver  = addNDTCellVertex(std::get<1>(cell_node), std::get<0>(cell_node)->getCell(), robot);
//
//				//Use cell size for covariance
////			double cx, cy, cz;
////			robot->getMap()->getCellSizeInMeters(cx, cy, cz);
////			double minval = std::min(cx, std::min(cy, cz));
//
////			Eigen::Matrix3d ndt_cov = cell.first->getCov();
//				Eigen::Matrix3d ndt_cov = robot->getCovariance() * _scaling_factor_gaussian;
//				Eigen::Matrix2d ndt_cov_2d;
//				ndt_cov_2d << ndt_cov(0,0), ndt_cov(0,1),
//						ndt_cov(1,0), ndt_cov(1,1);
//				ndt_cov_2d.array() = ndt_cov_2d.array() + std::get<2>(cell_node);
//
//				auto ndtcell_edge = addNDTCellAssociation(ndtcell_ver, wall, ndt_cov_2d);
//				ndtcell_edge->resolution_of_map = minval;
//
//				count_walls++;
//			}
//
//
//		}
	}

	if(_add_odometry_noise){
		int count = 0;
//		std::cout << "Loc poses : " << _nodes_localization.size() << " " << _edge_odometry.size() << std::endl;
		for(auto odom_edge : _edge_odometry){
			if(odom_edge->vertices()[0] == robot || odom_edge->vertices()[1] == robot){
				addNoiseOdometry(odom_edge, robot, _noise_percentage);
				count++;
			}
		}
		if(_edge_odometry.size() > 0) {
			assert(count == 1);
		}
	}


	std::cout << count_walls << " cells have been linked" << std::endl;
}



//https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-basic-concepts/
// First, check to see if the nearest point on the line AB is beyond B (as in the example above) by taking AB ⋅ BC. If this value is greater than 0, it means that the angle between AB and BC is between -90 and 90, exclusive, and therefore the nearest point on the segment AB will be B
std::vector<std::tuple< boost::shared_ptr<perception_oru::NDTCell>, Eigen::Vector2d, double> > AASS::acg::AutoCompleteGraphLocalization::collisionsNDTMapWithPriorEdge(const g2o::VertexSE2RobotLocalization& robot_pose_vertex, const g2o::EdgeXYPriorACG& wall, std::vector<std::tuple< g2o::VertexNDTCell*, Eigen::Vector2d, double> >& already_created_ndt_cells_to_update){

//	std::cout << "Collsision with prior wall" << std::endl;
	std::vector<std::tuple< boost::shared_ptr<perception_oru::NDTCell>, Eigen::Vector2d, double> > set_ret;

	Eigen::Vector2d p1 = dynamic_cast<g2o::VertexXYPrior*>(wall.vertices()[0])->estimate().head(2);
	Eigen::Vector2d p2 = dynamic_cast<g2o::VertexXYPrior*>(wall.vertices()[1])->estimate().head(2);

	auto map = robot_pose_vertex.getMap();
	double cx, cy, cz;
	map->getCellSizeInMeters(cx, cy, cz);
	double minval = std::min(cx, std::min(cy, cz));

	g2o::SE2 loc_pose( robot_pose_vertex.localizationInGlobalFrame() );

	auto cells = map->getAllCellsShared();

//	std::vector<boost::shared_ptr<perception_oru::NDTCell> > cells;

	//Keep one cell every three
//	for(int i = 0 ; i < cells_all.size() ; i = i + 3){
//		cells.push_back(cells_all[i]);
//	}

	for(auto cell : cells){

		perception_oru::NDTCell *cell_closest = NULL;

		Eigen::Vector3d mean = cell->getMean();
		mean(2) = 0;
		g2o::SE2 mean_se2(mean);
		Eigen::Vector2d p0 = (loc_pose * mean_se2).toVector().head(2);

		bool cell_close_by = false;
		//Only add if no node at the place of the cell

		//TODO: HACK FOR NOT HAVING TO MANY NDT CELL REDUNDANT. Change it so that it links to only one ndt cell and multiple pose. REMOVE WHEN USING BUNDLE OF NDT
//		When adding noise, we need all ndt because the maps are so far apart.
		if(!_add_odometry_noise) {
			for (auto cell_nodes : _vertices_ndt_cell) {
				if (cell_nodes->estimate()(0) + minval >= p0(0) && cell_nodes->estimate()(0) - minval <= p0(0) &&
				    cell_nodes->estimate()(1) + minval >= p0(1) && cell_nodes->estimate()(1) - minval <= p0(1)) {
					cell_close_by = true;
				}
			}
		}

		if(cell_close_by == false) {
			Eigen::Vector2d p1p0 = p0 - p1;
			Eigen::Vector2d p1p2 = p2 - p1;
			Eigen::Vector2d p2p0 = p0 - p2;

			//AVOID CORNERS BECAUSE IT CAUSE DE GRAPH TO CRASH TEST
			if (p1p0.norm() >= _min_distance_to_corner &&
				p2p0.norm() >= _min_distance_to_corner && (
					(p0 - loc_pose.toVector().head(2) ).norm() < _max_distance_of_ndt_cell_to_robot ||
					 _max_distance_of_ndt_cell_to_robot < 0)
				) {
//
//			//Check if closest segment point is on the line segment of the wall:
				if (p1p2.dot(p2p0) <= 0 && (-p1p2).dot(p1p0) <= 0) {


					//		pcl::PointXYZ p;
					//		p.x = p0[0];
					//		p.y = p0[1];
					//		p.z = mean[2];
					//		map->getCellAtPoint(p, cell_closest);
					//        Eigen::Vector2d p1 = dynamic_cast<g2o::VertexXYPrior*>(wall.vertices()[0])->estimate().head(2);
//				std::cout << "p1 " << p1 << std::endl;
					//		Eigen::Vector2d p2 = dynamic_cast<g2o::VertexXYPrior*>(wall.vertices()[1])->estimate().head(2);
//				std::cout << "p2 " << p2 << std::endl;
//				std::cout << "p0 " << p0 << std::endl;

					double distance = -1;
					Eigen::Vector2d ccp;
					std::tie(distance, ccp) = distancePointSegment(p0, p1, p2);

//				std::cout << "min value " << minval << " distance " << distance << std::endl;
//				if (distance <= minval) {

					//USE COLLISION TO FIND MATCHING ?
					Eigen::Matrix2d cov_inv_2d =
							robot_pose_vertex.getCovariance().inverse().block(0, 0, 2, 2) / _scaling_factor_gaussian;
					double radius_around_landmark = 1;
//								double factor = radius_around_landmark * radius_around_landmark / (2 * std::sqrt(loc->determinant) );
					double mahalanobis_distance = (p0 - (p0 + ccp)).dot(cov_inv_2d * (p0 - (p0 + ccp)));

					double pdf_collision = std::exp(-mahalanobis_distance);

					assert(pdf_collision >= 0);
					assert(pdf_collision <= 1);

//					if (pdf_collision >= _threshold_of_score_for_creating_a_link) {
					if(mahalanobis_distance <= 5.99){

						//USE COVARIANCE INSTEAD
						//			std::cout << "Close" << std::endl;

						g2o::SE2 rob_pose = robot_pose_vertex.estimate();
						Eigen::Vector2d mean_robot_pose = (rob_pose * mean_se2).toVector().head(2);
						auto resultat = std::make_tuple(cell, mean_robot_pose.head(2), distance);

//					auto close_ndt_cell = [this, minval] (const Eigen::Vector2d& p0) -> g2o::VertexNDTCell*  {
//						//Check if already exist
//						for (auto cell_nodes : _vertices_ndt_cell) {
//							if (cell_nodes->estimate()(0) + minval >= p0(0) && cell_nodes->estimate()(0) - minval <= p0(0) &&
//							    cell_nodes->estimate()(1) + minval >= p0(1) && cell_nodes->estimate()(1) - minval <= p0(1)) {
//								return cell_nodes;
//							}
//						}
//						return NULL;
//					};

//					g2o::VertexNDTCell* cell_node = close_ndt_cell(p0);
//					if(cell_node == NULL) {
						set_ret.push_back(resultat);
//					}
//					else{
//						auto resultat_node = std::make_tuple(cell_node, mean_robot_pose.head(2), distance);
//						already_created_ndt_cells_to_update.push_back(resultat_node);
//					}

					} else {
//					std::cout << "Not Close" << std::endl;
					}
				}
			}else{
				std::cout << " CELL TOO CLOSE TO WALL !! " << p0  << " and " << p1 << " norms " << p1p0.norm() << " " << p2 << " norms " << p2p0.norm() << " Min allowed distance is " << _min_distance_to_corner << " and max distance to robot " << _max_distance_of_ndt_cell_to_robot << " with distance being " << (p0 - loc_pose.toVector().head(2) ).norm() << " thus condition " << ( (p0 - loc_pose.toVector().head(2) ).norm() < _max_distance_of_ndt_cell_to_robot || _max_distance_of_ndt_cell_to_robot < 0)  << std::endl;			}
		}

	}

	return set_ret;


}
