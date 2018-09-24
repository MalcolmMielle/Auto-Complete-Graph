//#include "ACGBase.hpp"

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexSE2RobotPose* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addRobotPose(const g2o::SE2& se2, const Eigen::Affine3d& affine, const std::shared_ptr< perception_oru::NDTMap >& map){

	ROS_DEBUG("Adding the robot pose ");
	g2o::VertexSE2RobotPose* robot =  new g2o::VertexSE2RobotPose();

	robot->setEstimate(se2);
	robot->setId(new_id_);
	++new_id_;

	robot->setMap(map);
	robot->setPose(affine);
	robot->initial_transfo = robot->estimate();

	_optimizable_graph.addVertex(robot);

// 	NDTNodeAndMap nodeAndMap(robot, map, affine);

	_nodes_ndt.push_back(robot);
	return robot;
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexSE2RobotPose* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addRobotPose(const Eigen::Vector3d& rob, const Eigen::Affine3d& affine, const std::shared_ptr< perception_oru::NDTMap >& map){
	g2o::SE2 se2(rob(0), rob(1), rob(2));
	return addRobotPose(se2, affine, map);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexSE2RobotPose* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addRobotPose(double x, double y, double theta, const Eigen::Affine3d& affine, const std::shared_ptr< perception_oru::NDTMap >& map){
	Eigen::Vector3d robot1;
	robot1 << x, y, theta;
	return addRobotPose(robot1, affine, map);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexLandmarkNDT* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkPose(const g2o::Vector2& estimate, const cv::Point2f& position, int strength){
	g2o::VertexLandmarkNDT* landmark = new g2o::VertexLandmarkNDT();
	landmark->setId(new_id_);
	++new_id_;
// 	std::cout << "Setting the id " << _optimizable_graph.vertices().size() << std::endl;
	landmark->setEstimate(estimate);
	landmark->position = position;
	_optimizable_graph.addVertex(landmark);
	_nodes_landmark.push_back(landmark);
	return landmark;
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexLandmarkNDT* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkPose(double x, double y, const cv::Point2f& position, int strength){
	g2o::Vector2 lan;
	lan << x, y;
	return addLandmarkPose(lan, position, strength);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline VertexPrior* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorLandmarkPose(const g2o::SE2& se2, const PriorAttr& priorAttr){
//	VertexPrior* priorlandmark = new VertexPrior();
//	priorlandmark->setId(new_id_);
//	++new_id_;
//	priorlandmark->setEstimate(se2);
//	priorlandmark->priorattr = priorAttr;
//
	auto landmark = _prior->addPose(se2, priorAttr, new_id_);
	++new_id_;
	_optimizable_graph.addVertex(landmark);
//	_nodes_prior.push_back(priorlandmark);
	return landmark;
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline VertexPrior* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorLandmarkPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr){
	g2o::SE2 se2(lan(0), lan(1), lan(2));
	return addPriorLandmarkPose(se2, priorAttr);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline VertexPrior* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorLandmarkPose(double x, double y, double theta, const AASS::acg::PriorAttr& priorAttr){
	Eigen::Vector3d lan;
	lan << x, y, theta;
	return addPriorLandmarkPose(lan, priorAttr);
}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix3d& information_tmp){


	for(auto odom_edge : _edge_odometry){
		if(v1 == odom_edge->vertices()[0] && v2 == odom_edge->vertices()[1]){
			throw std::runtime_error("Edge odometry already added");
		}
		else if(v1 == odom_edge->vertices()[1] && v2 == odom_edge->vertices()[0]){
			throw std::runtime_error("Edge odometry already added");
		}
	}

	Eigen::Matrix3d information;
	if(_use_user_robot_pose_cov == true){
		Eigen::Matrix3d covariance_robot;
		covariance_robot.fill(0.);
		covariance_robot(0, 0) = _transNoise[0]*_transNoise[0];
		covariance_robot(1, 1) = _transNoise[1]*_transNoise[1];
	// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
		covariance_robot(2, 2) = _rotNoise * _rotNoise;
		information = covariance_robot.inverse();

		throw std::runtime_error("Do not use user inputed values in odometry");
	}
	else{
		 information = information_tmp;
	}

	assert(information.isZero(1e-10) == false);

	g2o::EdgeOdometry_malcolm* odometry = new g2o::EdgeOdometry_malcolm;
	odometry->vertices()[0] = v1 ;
	odometry->vertices()[1] = v2 ;
	odometry->setMeasurement(se2);
	odometry->setInformation(information);

// 	odometry->interface.setAge(_age_start_value);

	_optimizable_graph.addEdge(odometry);
	_edge_odometry.push_back(odometry);

	assert(verifyInformationMatrices(true) == true);

	return odometry;
}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(const g2o::SE2& observ, int from_id, int toward_id, const Eigen::Matrix3d& information){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addOdometry(observ, from_ptr, toward_ptr, information);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(double x, double y, double theta, int from_id, int toward_id, const Eigen::Matrix3d& information){
	g2o::SE2 se2(x, y, theta);
	return addOdometry(se2, from_id, toward_id, information);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){

    Eigen::Matrix3d covariance;
	
	covariance.fill(0.);
	covariance(0, 0) = _transNoise[0]*_transNoise[0];
	covariance(1, 1) = _transNoise[1]*_transNoise[1];
	covariance(2, 2) = _rotNoise*_rotNoise;
	Eigen::Matrix3d information = covariance.inverse();

	throw std::runtime_error("Do not use user inputed values in odometry");

	return addOdometry(se2, v1, v2, information);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(const g2o::SE2& observ, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addOdometry(observ, from_ptr, toward_ptr);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(double x, double y, double theta, int from_id, int toward_id){
	g2o::SE2 se2(x, y, theta);
	return addOdometry(se2, from_id, toward_id);
}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeLandmark_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix2d& covariance_landmark){

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

	g2o::EdgeLandmark_malcolm* landmarkObservation =  new g2o::EdgeLandmark_malcolm;
	landmarkObservation->vertices()[0] = v1;
	landmarkObservation->vertices()[1] = v2;
	landmarkObservation->setMeasurement(pos);

	assert(information_landmark.isZero(1e-10) == false);

	landmarkObservation->setInformation(information_landmark);
	landmarkObservation->setParameterId(0, _sensorOffset->id());


// 	landmarkObservation->interface.setAge(_age_start_value);

	_optimizable_graph.addEdge(landmarkObservation);
	_edge_landmark.push_back(landmarkObservation);

	assert(verifyInformationMatrices(true) == true);

	return landmarkObservation;
}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeLandmark_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkObservation(const g2o::Vector2& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
	Eigen::Matrix2d covariance_landmark;
	covariance_landmark.fill(0.);
	covariance_landmark(0, 0) = _landmarkNoise[0]*_landmarkNoise[0];
	covariance_landmark(1, 1) = _landmarkNoise[1]*_landmarkNoise[1];

//	throw std::runtime_error("Do not use user inputed values in landmark observation");

// 			covariance_landmark(2, 2) = 13;//<- Rotation covariance landmark is more than 4PI
	return addLandmarkObservation(pos, v1, v2, covariance_landmark);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeLandmark_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkObservation(const g2o::Vector2& pos, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addLandmarkObservation(pos, from_ptr, toward_ptr);
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline EdgePrior* AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){

	auto priorObservation = _prior->addEdge(se2, v1, v2);
// 	priorObservation->interface.setAge(_age_start_value);
// 	priorObservation->interface.setOriginalValue(se2);
	_optimizable_graph.addEdge(priorObservation);

// 	EdgePriorAndInitialValue epiv(priorObservation, se2);
//	_edge_prior.push_back(priorObservation);
//	std::cout << "After adding an edge" << std::endl;
//	checkNoRepeatingPriorEdge();

    assert(verifyInformationMatrices(true) == true);

	return priorObservation;
}

// void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addEdgePrior(g2o::SE2 observ, int from, int toward){
// 	std::tuple<g2o::SE2, int, int> obs1(observ, from, toward);
// 	addEdgePrior(obs1);
// }
// void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addEdgePrior(double x, double y, double theta, int from, int toward){
// 	g2o::SE2 se2(x, y, theta);
// 	addEdgePrior(se2, from, toward);
//
// }




//FUNCTION TO REMOVE A VERTEX
//TODO : remove all link edges from list
template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::removeVertex(g2o::HyperGraph::Vertex* v1){
	//Prior
	auto ptr = _prior->removeVertex(v1);
	if(ptr != NULL){
		_optimizable_graph.removeVertex(ptr);
	}

	//VertexPrior* ptr = dynamic_cast<VertexPrior*>(v1);
	g2o::VertexSE2* ptr_se2 = dynamic_cast<g2o::VertexSE2*>(v1);
	g2o::VertexLandmarkNDT* ptr_se3 = dynamic_cast<g2o::VertexLandmarkNDT*>(v1);

//	if(ptr != NULL){
//		std::cout <<"Find prior" << std::endl;
//		int index = findPriorNode(v1);
//		assert(index != -1);
//		std::vector<VertexPrior*>::iterator which = _nodes_prior.begin() + index;
//		_nodes_prior.erase(which);
//	}
	//Robot node
	if( ptr_se2 != NULL){
		int index = findRobotNode(v1);
		assert(index != -1);
		auto which = _nodes_ndt.begin() + index;
		_nodes_ndt.erase(which);

	}
	//Landmark Node
	else if( ptr_se3 != NULL){
		int index = findLandmarkNode(v1);
		assert(index != -1);
		std::vector<g2o::VertexLandmarkNDT*>::iterator which = _nodes_landmark.begin() + index;
		_nodes_landmark.erase(which);
	}
	else{
		throw std::runtime_error("Vertex type not found in list");
	}
	_optimizable_graph.removeVertex(v1, false);
}

// TODO
// void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::removeEdge(g2o::HyperGraph::Edge* v1){
// 	//Prior
// 	g2o::EdgeLandmark_malcolm* ptr = dynamic_cast<g2o::EdgeLandmark_malcolm*>(v1);
// 	g2o::EdgeSE2Prior_malcolm* ptr_se2 = dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(v1);
// 	g2o::EdgeOdometry_malcolm* ptr_se3 = dynamic_cast<g2o::EdgeOdometry_malcolm*>(v1);
//
// 	if(ptr != NULL){
// 		int index = findPriorNode(v1);
// 		assert(index != -1);
// 		auto which = _edge_landmark.begin() + index;
// 		_nodes_prior.erase(which);
// 	}
// 	//Robot node
// 	else if( ptr_se2 != NULL){
// 		int index = findRobotNode(v1);
// 		assert(index != -1);
// 		auto which = _nodes_ndt.begin() + index;
// 		_nodes_ndt.erase(which);
//
// 	}
// 	//Landmark Node
// 	else if( ptr_se3 != NULL){
// 		int index = findLandmarkNode(v1);
// 		assert(index != -1);
// 		auto which = _nodes_landmark.begin() + index;
// 		_nodes_landmark.erase(which);
// 	}
// 	else{
// 		throw std::runtime_error("Vertex type not found in list");
// 	}
// 	_optimizable_graph.removeVertex(v1, true);
// }

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline int AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::findRobotNode(g2o::HyperGraph::Vertex* v){
	int pos = 0;
	auto it = _nodes_ndt.begin();
	for(it ; it != _nodes_ndt.end() ; ++it){
		if(*it == v){
			return pos;
		}
		++pos;
	}
	return -1;
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline int AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::findLandmarkNode(g2o::HyperGraph::Vertex* v){
	int pos = 0;
	auto it = _nodes_landmark.begin();
	for(it ; it != _nodes_landmark.end() ; ++it){
		if(*it == v){
			return pos;
		}
		++pos;
	}
	return -1;

}
//int AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::findPriorNode(g2o::HyperGraph::Vertex* v){
//	int pos = 0;
//	auto it = _nodes_prior.begin();
//	for(it ; it != _nodes_prior.end() ; ++it){
//		if(*it == v){
//			return pos;
//		}
//		++pos;
//	}
//	return -1;
//
//}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorGraph(const PriorLoaderInterface::PriorGraph& graph){

	new_id_ = _prior->addPriorGraph(graph, new_id_);

	for(auto node : _prior->getNodes()){
		_optimizable_graph.addVertex(node);
	}
	for(auto edge : _prior->getEdges()){
		_optimizable_graph.addEdge(edge);
	}


//	std::pair< PriorLoaderInterface::PriorGraph::VertexIterator, PriorLoaderInterface::PriorGraph::VertexIterator > vp;
//	std::deque<PriorLoaderInterface::PriorGraph::Vertex> vec_deque;
//	std::vector<VertexPrior*> out_prior;
//
//	std::cout << "NOOOOOOW" << _optimizable_graph.vertices().size() << std::endl << std::endl;
//
//	assert( _nodes_prior.size() == 0 );
//
//	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
//// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
//		auto v = *vp.first;
//		//ATTENTION Magic number
//
//// 		std::cout << "Prior Landmark : " << graph[v].getX() << " " << graph[v].getY() << std::endl;
//
//
//		VertexPrior* res = addPriorLandmarkPose(graph[v].getX(), graph[v].getY(), 0, graph[v]);
//		vec_deque.push_back(v);
//		out_prior.push_back(res);
//// 		_nodes_prior.push_back(res);
//	}
//
//	assert( _edge_prior.size() == 0);
//
//	int count = 0;
//	int self_link = 0 ;
//	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
//// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
//		auto v = *vp.first;
//		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
//		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Edge e;
//
//		std::vector<PriorLoaderInterface::PriorGraph::Vertex> tmp_for_double_edges;
//
//		for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph));
//			out_i != out_end; ++out_i) {
//			e = *out_i;
//// 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e, (graph));
//			auto targ = boost::target(e, (graph));
//
//			//Avoiding double edge to the same vertex target but that is not a self loop!
//
//			int idx = -1;
//			for(size_t ii = count ; ii < vec_deque.size() ; ++ii){
//				if(targ == vec_deque[ii]){
//					idx = ii;
//				}
//			}
//			for(auto it_tmp_double = tmp_for_double_edges.begin() ; it_tmp_double != tmp_for_double_edges.end() ; ++it_tmp_double){
//				if(*it_tmp_double == targ){
//					//it's a double we cancel idx and add it to the self_lopp count
//					idx = count;
//				}
//			}
//			tmp_for_double_edges.push_back(targ);
//
//			if(idx == -1){
//				//SKIP
//// 				throw std::runtime_error("Didn't find the vertex target :(");
//			}
//			else if(idx == count){
//				self_link = self_link + 1 ;
//			}
//			else{
//
//				assert(idx != count);
//
//				VertexPrior* from = out_prior[count]; //<-Base
//				VertexPrior* toward = out_prior[idx]; //<-Targ
//
//				double x_diff = graph[targ].getX() - graph[v].getX();
//				double y_diff = graph[targ].getY() - graph[v].getY();
//
//// 				std::cout << "Index" << idx << " " <<count << std::endl;
//
//// 				std::cout << "Because : " << graph[targ].getX() << " - " << graph[v].getX() << " and " << graph[targ].getY() << " - " << graph[v].getY() << std::endl;
//
//// 				std::cout << "diff: " <<x_diff << " " << y_diff << std::endl;
//
//				g2o::SE2 se2(x_diff, y_diff, 0);
//
//// 				std::cout << "SE2 pushed in edge \n" << se2.toVector() << std::endl;
//
//				auto edge_out = addEdgePrior(se2, from, toward);
//// 				_edge_prior.push_back(edge_out);
//			}
//
//		}
//		++count;
//	}
//
//// 	std::cout << _edge_prior.size() << " == " << graph.getNumEdges() << " - " << self_link / 2 << std::endl;
//// 	std::cout << _nodes_prior.size() << " == " << graph.getNumVertices() << std::endl;
//
//	assert( _nodes_prior.size() == graph.getNumVertices() );
//	//Self link / 2 because they are seen twice
//	assert( _edge_prior.size() == graph.getNumEdges() - (self_link / 2) );
//
//	std::cout << "After update graph prior" << std::endl;
//	checkNoRepeatingPriorEdge();

}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline Eigen::Vector3d AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::getLastTransformation()
{
	Eigen::Vector3d diff_vec; diff_vec << 0, 0, 0;
	//**************** Calculate the previous transformations if there as already been something added *** //

// 	if(_previous_number_of_node_in_ndtgraph != 0){
	if(_nodes_ndt.size() != 0){
		auto node = _nodes_ndt[_nodes_ndt.size() - 1];
		auto original3d = _nodes_ndt[_nodes_ndt.size() - 1]->getPose();

		/*******' Using Vec********/
		Eigen::Isometry2d original2d_aff = Affine3d2Isometry2d(original3d);
		auto shift_vec = node->estimate().toVector();
		g2o::SE2 se2_tmptmp(original2d_aff);
		auto original3d_vec = se2_tmptmp.toVector();
		diff_vec = original3d_vec - shift_vec;

		/*****************************/

// 			auto shift = node->estimate().toIsometry();


// 		double angle = atan2(shift.rotation()(1,0), shift.rotation()(0,0));//rot.angle();//acosa2d.rotation()(0,1)/a2d.rotation()(0,0);
// 		auto shift3d = Eigen::Translation3d(shift.translation()(0), shift.translation()(1), 0.) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
// 			auto getRobustYawFromAffine3d = [](const Eigen::Affine3d &a) -> double {
// 				// To simply get the yaw from the euler angles is super sensitive to numerical errors which will cause roll and pitch to have angles very close to PI...
// 				Eigen::Vector3d v1(1,0,0);
// 				Eigen::Vector3d v2 = a.rotation()*v1;
// 				double dot = v1(0)*v2(0)+v1(1)*v2(1); // Only compute the rotation in xy plane...
// 				double angle = acos(dot);
// 				// Need to find the sign
// 				if (v1(0)*v2(1)-v1(1)*v2(0) > 0)
// 					return angle;
// 				return -angle;
// 			};
//


// 			auto original2d = Eigen::Translation2d(original3d.translation().topRows<2>()) * Eigen::Rotation2D<double>(getRobustYawFromAffine3d(original3d));

// 			Eigen::Affine2d original2d;
// 			original2d.matrix() << original3d(0, 0), original3d(0, 1), original3d(0, 3),
// 								   original3d(1, 0), original3d(1, 1), original3d(1, 3),
// 												  0,                0, original3d(2, 3);



// 			auto diff_affine = original2d.inverse() * shift;
// 			diff.translation() = diff_affine.translation();
// 			diff.linear() = diff_affine.rotation();
	}

	return diff_vec;
}



template< typename Prior, typename VertexPrior, typename EdgePrior>
inline std::shared_ptr<perception_oru::NDTMap> AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addElementNDT(ndt_feature::NDTFeatureGraph& ndt_graph, const std::vector< ndt_feature::NDTFeatureLink >& links, int element, double deviation, g2o::VertexSE2RobotPose** robot_ptr, g2o::SE2& robot_pos)
{

	//Return Gaussian white noise
	auto randomNoise = [](double mean, double deviationt) -> double {
		std::default_random_engine engine{std::random_device()() };
		std::normal_distribution<double> dist(mean, deviationt);
		return dist(engine);
	};

	Eigen::Vector3d diff_vec = getLastTransformation();

	//Calculate noise
	auto noise_x = randomNoise(0, deviation);
	auto noise_y = randomNoise(0, deviation);
	auto noise_t = randomNoise(0, deviation);
	g2o::SE2 noise_se2(noise_x, noise_y, noise_t);

	std::cout << "Checking node nb " << element << std::endl;

	//RObot pose
// 			ndt_feature::NDTFeatureNode& feature = dynamic_cast<ndt_feature::NDTFeatureNode&>(ndt_graph.getNodeInterface(i));
	std::cout << "Copy feature" << std::endl;

// 			feature.copyNDTFeatureNode( (const ndt_feature::NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
	Eigen::Affine3d affine = Eigen::Affine3d(ndt_graph.getNodeInterface(element).getPose());
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
//	std::cout << "multiply" << std::endl;

	perception_oru::NDTMap* map = ndt_graph.getMap(element);

//	std::cout << "get res" << std::endl;
// 			double resolution = dynamic_cast<ndt_feature::NDTFeatureNode&>( ndt_graph.getNodeInterface(i) ).map->params_.resolution;
// 			Use a a msg to copy to a new pointer so it doesn't get forgotten :|
	ndt_map::NDTMapMsg msg;
// 			ATTENTION Frame shouldn't be fixed
	bool good = perception_oru::toMessage(map, msg, "/world");
	perception_oru::NDTMap* map_copy;
	perception_oru::LazyGrid* lz;
	std::string frame;
	bool good2 = perception_oru::fromMessage(lz, map_copy, msg, frame);
	std::shared_ptr<perception_oru::NDTMap> shared_map(map_copy);

	*robot_ptr = addRobotPose(robot_pos, affine, shared_map);
	assert(*robot_ptr != NULL);
	//Add Odometry if it is not the first node
	if(element > 0 ){
		ROS_DEBUG("adding the odometry");

		g2o::SE2 odometry = NDTFeatureLink2EdgeSE2(links[element - 1]);
		ROS_DEBUG_STREAM(" ref " << links[element-1].getRefIdx() << " and mov " << links[element-1].getMovIdx() );

		assert( links[element-1].getRefIdx() < _nodes_ndt.size() );
		assert( links[element-1].getMovIdx() < _nodes_ndt.size() );

		auto from = _nodes_ndt[ links[element-1].getRefIdx() ] ;
		auto toward = _nodes_ndt[ links[element-1].getMovIdx() ] ;

		ROS_DEBUG("Saving cov ");
		//TODO : transpose to 3d and use in odometry!
		Eigen::MatrixXd cov = links[element - 1].cov_3d;

		ROS_DEBUG_STREAM("COV " << cov);

		ROS_DEBUG_STREAM("Saving cov to 2d");
		Eigen::Matrix3d cov_2d;
		cov_2d << 	cov(0, 0), 	cov(0, 1), 	0,
					cov(1, 0), 	cov(1, 1), 	0,
					0, 		 	0, 			cov(5, 5);

		ROS_DEBUG_STREAM("Saving information ");
		Eigen::Matrix3d information = cov_2d.inverse();

// 				if(noise_flag = true && i != 0){
// 					odometry = odometry * noise_se2;
// 				}

		ROS_DEBUG_STREAM("Saving odometry ");
		addOdometry(odometry, from, toward, information);
	}

	return shared_map;

}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::getAllCornersNDTTranslatedToGlobalAndRobotFrame(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos, std::vector<AASS::acg::NDTCornerGraphElement>& corners_end)
{


	perception_oru::ndt_feature_finder::NDTCorner cornersExtractor;
	ROS_DEBUG_STREAM("hopidy");
	auto ret_export = cornersExtractor.getAllCorners(*map);
	ROS_DEBUG_STREAM("gotall corner");
// 	auto ret_opencv_point_corner = cornersExtractor.getAccurateCvCorners();
	ROS_DEBUG_STREAM("got all accurate corners");
// 	auto angles = cornersExtractor.getAngles();
// 	std::cout << "got all angles");

	auto it = ret_export.begin();

	ROS_DEBUG_STREAM("Found " << ret_export.size() << " corners " );
	//Find all the observations :

	//**************** HACK: translate the corners now : **************//

// 	int count_tmp = 0;
	for(it ; it != ret_export.end() ; ++it){
		ROS_DEBUG_STREAM("Corner size " << it->getOrientations().size() );
		//Limited to corners that possess an orientation.
		if(it->getOrientations().size() > 0){
			Eigen::Vector3d vec;
	// 		vec << it->x, it->y, angles[count_tmp].second;
			ROS_DEBUG_STREAM("Corner size " );
			vec << it->getMeanOpenCV().x, it->getMeanOpenCV().y, it->getOrientations()[0];

			ROS_DEBUG_STREAM("Corner size " );
			cv::Point2f p_out;
			Eigen::Vector3d landmark_robotframe;
			translateFromRobotFrameToGlobalFrame(vec, robot_pos, landmark_robotframe);

			ROS_DEBUG_STREAM("Corner size " );
			p_out.x = landmark_robotframe(0);
			p_out.y = landmark_robotframe(1);

			Eigen::Vector2d observation; observation << vec(0), vec(1);
			std::vector<double> orientations;
			std::vector<double> angles_width;

			ROS_DEBUG_STREAM("Corner size " );
			double angle_landmark = vec(2);

			ROS_DEBUG_STREAM("Corner size " );
			for(auto it_orientation = it->getOrientations().begin() ; it_orientation != it->getOrientations().end() ; ++it_orientation){
				ROS_DEBUG_STREAM("Pushing back orientation" );
				orientations.push_back((*it_orientation));
			}
			for(auto it_angles = it->getAngles().begin() ; it_angles != it->getAngles().end() ; ++it_angles){
				ROS_DEBUG_STREAM("Pushing back angle" );
				angles_width.push_back((*it_angles));
			}

	// 						std::cout << "MOVE : "<< it -> x << " " << it-> y << std::endl;

	// 		auto vec_out_se2 = _nodes_ndt[i]->estimate();

			//Calculate obstacle position_in_robot_frame in global coordinates.

	// 		//Node it beong to :
	// 		g2o::SE2 vec_out_se2 = g2o::SE2(robot_pos);
	// 		//Pose of landmajr
	// 		g2o::SE2 se2_tmp(vec);
	// 		//Composition
	// 		vec_out_se2 = vec_out_se2 * se2_tmp;
	// 		Eigen::Vector3d vec_out = vec_out_se2.toVector();
	//
	// 		//Pose of landmark in global reference
	// 		cv::Point2f p_out(vec_out(0), vec_out(1));
	//
	// 		Eigen::Vector2d real_obs; real_obs << p_out.x, p_out.y;
	// 		Eigen::Vector2d observation2d_test;
	// 		//Projecting real_obs into robot coordinate frame
	//
	// 		auto trueObservation_tmp = robot_pos.inverse() * vec_out_se2;
	// 		Eigen::Vector3d trueObservation = trueObservation_tmp.toVector();




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
			ROS_DEBUG_STREAM("Position node " << robot_pos.toVector() );
			ROS_DEBUG_STREAM(" vec " << vec );
	// 		std::cout << "Well " << robot_pos.toVector() + vec << "==" << ndt_graph.getNode(i).T * vec << std::endl;

			//ATTENTION THIS IS NOT TRUE BUT REALLY CLOSE
	// 				assert (robot_pos + vec == ndt_graph.getNode(i).T * vec);

					ROS_DEBUG_STREAM("NEW POINT : "<< p_out);

			NDTCornerGraphElement cor(p_out, *it);
			cor.addAllObserv(robot_ptr, observation);
			cor.cells1 = it->getCells1();
			cor.cells2 = it->getCells2();
			corners_end.push_back(cor);
	// 		count_tmp++;
		}

	}
	//At this point, we have all the corners
// 	assert(corners_end.size() - c_size == ret_opencv_point_corner_tmp.size());
// 	assert(corners_end.size() == ret_export.size());
// 	assert(corners_end.size() - c_size == angles_tmp.size());
// 	assert(corners_end.size() == angles.size());
// 	c_size = corners_end.size();

}


//TODO: refactor this!
template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::extractCornerNDTMap(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos)
{
	//HACK For now : we translate the Corner extracted and not the ndt-maps
	auto cells = map->getAllCellsShared();
	std::cout << "got all cell shared " <<  cells.size() << std::endl;
	double x2, y2, z2;
	map->getCellSizeInMeters(x2, y2, z2);
	std::cout << "got all cell sized" << std::endl;
	double cell_size = x2;

	std::vector<AASS::acg::NDTCornerGraphElement> corners_end;
	getAllCornersNDTTranslatedToGlobalAndRobotFrame(map, robot_ptr, robot_pos, corners_end);

	/***************** ADD THE CORNERS INTO THE GRAPH***********************/

	for(size_t i = 0 ; i < corners_end.size() ; ++i){
// 		std::cout << "checking corner : " << _nodes_landmark.size() << std::endl ;  /*corners_end[i].print()*/ ; std::cout << std::endl;
		bool seen = false;
		g2o::VertexLandmarkNDT* ptr_landmark_seen = NULL;
		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			g2o::Vector2 landmark = _nodes_landmark[j]->estimate();
			cv::Point2f point_land(landmark(0), landmark(1));

			double res = cv::norm(point_land - corners_end[i].position_in_robot_frame);

			ROS_DEBUG_STREAM("res : ");

			//If we found the landmark, we save the data
			if( res < cell_size * 2){
				seen = true;
				ptr_landmark_seen = _nodes_landmark[j];
			}
		}
		if(seen == false){
			ROS_DEBUG_STREAM( "New point " << i );
// 			assert(i < ret_export.size());
			g2o::Vector2 position_globalframe;
			position_globalframe << corners_end[i].position_in_robot_frame.x, corners_end[i].position_in_robot_frame.y ;
// 			g2o::VertexLandmarkNDT* ptr = addLandmarkPose(vec, ret_export[i].getMeanOpenCV(), 1);

			cv::Point2f p_observation;
			ROS_DEBUG_STREAM("New point " << i );
			p_observation.x = corners_end[i].getObservations()(0);
			ROS_DEBUG_STREAM("New point " << i );
			p_observation.y = corners_end[i].getObservations()(1);
			ROS_DEBUG_STREAM("New point " << i );
			g2o::VertexLandmarkNDT* ptr = addLandmarkPose(position_globalframe, p_observation, 1);
			ptr->addAnglesOrientations(corners_end[i].getAngles(), corners_end[i].getOrientations());
			ptr->first_seen_from = robot_ptr;

			//TESTING to visualise which cells gave the corner
			ptr->cells_that_gave_it_1 = corners_end[i].cells1;
			ptr->cells_that_gave_it_2 = corners_end[i].cells2;
			ptr->robotpose_seen_from = robot_pos;
			//END OF TEST


			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr);


		}
		else{
			//TODO
			ROS_DEBUG_STREAM("Point seen " );
			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr_landmark_seen);
		}
	}

}





template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph, bool noise_flag, double deviation){

	ROS_DEBUG_STREAM("BEfore " );
// 	printCellsNum();
	ROS_DEBUG_STREAM("Should we check " <<ndt_graph.getNbNodes() << ">" << _previous_number_of_node_in_ndtgraph );

	//************* Add and create all new nodes, and extract the corners from the new NDT maps *********//

	if(ndt_graph.getNbNodes() > _previous_number_of_node_in_ndtgraph){

		ROS_DEBUG_STREAM("Found new nodes");

		//TODO just get the links that are interesting for you instead of all of them !
		//Doing the registration here myself

		auto links = ndt_graph.getOdometryLinks();

		//Update the link in all node not already added

		//Need at least three node to start considering link since the last one is garbage
		if(_previous_number_of_node_in_ndtgraph >= 2){
			//Ignore last link
			for (size_t i = _previous_number_of_node_in_ndtgraph - 2; i < links.size() - 1; i++) {
		      ROS_DEBUG_STREAM("updating link : " << i << " (size of links :" << links.size() << ")" );
				ndt_graph.updateLinkUsingNDTRegistration(links[i], 10, true);
			}
		}
		else{
			ndt_graph.updateLinksUsingNDTRegistration(links, 10, true);
		}



		//Calculate the original transformation of all

		//Assume that all node in the NDT graph must been analysed
		size_t i;
		i = _previous_number_of_node_in_ndtgraph - 1;
// 		std::vector<cv::Point2f> ret_opencv_point_corner;
// 		std::vector<std::pair<double, double> > angles;

		//ATTENTION Ignore the last node because it's not good usually !
		for (i; i < ndt_graph.getNbNodes() - 1 ; ++i) {

			g2o::VertexSE2RobotPose* robot_ptr;
			g2o::SE2 robot_pos;
			auto map = addElementNDT(ndt_graph, links, i, deviation, &robot_ptr, robot_pos);
			assert(robot_ptr != NULL);
			ROS_DEBUG_STREAM("TEST pointer " << robot_ptr->getPose().matrix() );
			//********************** Extract the corners *****************//
			extractCornerNDTMap(map, robot_ptr, robot_pos);
			//********************** Add the time stamp ******************//
			robot_ptr->setTime(ndt_graph.getNode(i).time_last_update);

		}
		//Save new number of nodes to update

// 		assert(corners_end.size() == ret_opencv_point_corner.size());
// 		std::vector<g2o::VertexLandmarkNDT*> all_new_landmarks;

		// ************ Add the landmarks that were added the corners_end ************************************//

		//Add stuff directly in the optimization graph :
		_previous_number_of_node_in_ndtgraph = ndt_graph.getNbNodes();
	}


}



//template< typename Prior, typename VertexPrior, typename EdgePrior>
//inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::updateLinks()
//{
//
//	if(_use_links_prior) {
//		std::cout << "Create new links" << std::endl;
//// 	int count = countLinkToMake();
//		int count2 = createNewLinks();
//// 	if(count != count2){
//// 		std::cout << "Weird different detection" << std::endl;
//// 		throw std::runtime_error("ARF NOT GOOD COUNT in update link");
//// 	}
//		std::cout << "updateLink no small forgotten" << std::endl;
//// 	checkLinkNotForgotten();
//		removeBadLinks();
//		std::cout << "update link not too big" << std::endl;
//		checkLinkNotTooBig();
//	}
//}









template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::testNoNanInPrior(const std::string& before) const {

    _prior->testNoNanInPrior(before);

//	std::cout << "Test No nan in prior after " << before << std::endl;
//	auto it = _prior->getNodes().begin();
//	for(it ; it != _prior->getNodes().end() ; ++it){
//		VertexPrior* v_ptr = dynamic_cast<VertexPrior*>((*it));
//		if(v_ptr == NULL){
//			throw std::runtime_error("not good vertex type");
//		}
//		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
//		assert(!std::isnan(pose1[0]));
//		assert(!std::isnan(pose1[1]));
//		assert(!std::isnan(pose1[2]));
//
//	}
//
//	std::cout << "Testing the edges now" << std::endl;
//
//	auto edges = _prior->getEdges();
//	auto it_edge = edges.begin();
//	for(it_edge ; it_edge != edges.end() ; ++it_edge){
//		VertexPrior* v_ptr = dynamic_cast<VertexPrior*>((*it_edge)->vertices()[0]);
//		if(v_ptr == NULL){
//			throw std::runtime_error("no");
//		}
//		VertexPrior* v_ptr2 = dynamic_cast<VertexPrior*>((*it_edge)->vertices()[1]);
//		if(v_ptr2 == NULL){
//			throw std::runtime_error("no2");
//		}
//		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
//		Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
//
//		assert(!std::isnan(pose1[0]));
//		assert(!std::isnan(pose1[1]));
//		assert(!std::isnan(pose1[2]));
//
//		assert(!std::isnan(pose2[0]));
//		assert(!std::isnan(pose2[1]));
//		assert(!std::isnan(pose2[2]));
//	}

}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::testInfoNonNul(const std::string& before) const {

//TODO CORRECT THIS
//	std::cout << "Test info non nul after " << before << std::endl;
//	auto idmapedges = _optimizable_graph.edges();
//
//	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
//		assert((*ite)->vertices().size() >= 1);
//
//		g2o::EdgeLandmark_malcolm* ptr = dynamic_cast<g2o::EdgeLandmark_malcolm*>(*ite);
//		//Doesn't work with template TODO
//		EdgePrior* ptr1 = dynamic_cast<EdgePrior*>(*ite);
//		g2o::EdgeOdometry_malcolm* ptr2 = dynamic_cast<g2o::EdgeOdometry_malcolm*>(*ite);
//		if(ptr != NULL){
//			assert(ptr->information().isZero(1e-10) == false);
//		}
//		else if(ptr1 != NULL){
//			assert(ptr1->information().isZero(1e-10) == false);
//		}
//		else if(ptr2 != NULL){
//			assert(ptr2->information().isZero(1e-10) == false);
//		}
//		else{
//			throw std::runtime_error("Didn't find the type of the edge :S");
//		}
//	}


}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::updatePriorEdgeCovariance()
{

    _prior->updatePriorEdgeCovariance();

//	std::cout << "DO NOT USE " << std::endl;
//	testNoNanInPrior("no dtat");
//	assert(false);
//
//	auto edges = _prior->getEdges();
//	auto it = edges.begin();
//	for(it ; it != edges.end() ; ++it){
//		VertexPrior* v_ptr = dynamic_cast<VertexPrior*>((*it)->vertices()[0]);
//		if(v_ptr == NULL){
//			throw std::runtime_error("no");
//		}
//		VertexPrior* v_ptr2 = dynamic_cast<VertexPrior*>((*it)->vertices()[1]);
//		if(v_ptr2 == NULL){
//			throw std::runtime_error("no2");
//		}
//		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
//		Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
//
//		// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
//		// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;
//
//		double tre[3];
//		(*it)->getMeasurementData(tre);
//
//		Eigen::Vector2d length; length << tre[0], tre[1] ;
//
//		Eigen::Vector2d eigenvec;
//		eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
//
//		double newnorm = (pose1 - pose2).norm();
//		double len_norm = length.norm();
//		std::cout << "new norm " << newnorm << " because " << pose1 << " " << pose2 << " and lennorm " << len_norm << "because  " <<length << std::endl;
//		g2o::SE2 oldnormse2 = (*it)->interface.getOriginalValue();
//		Eigen::Vector3d vecold = oldnormse2.toVector();
//		double oldnorm = (vecold).norm();
//		std::cout << "oldnorm" << oldnorm << std::endl;
//		assert(oldnorm >= 0);
//
//		//Using the diff so we cannot shrink it or stretch it easily.
//		double diff_norm = std::abs(oldnorm - newnorm);
//		std::cout << "Diff norm " << diff_norm << std::endl;
//		assert(diff_norm >= 0);
//		if(diff_norm > oldnorm){
//			diff_norm = oldnorm;
//		}
//		diff_norm = std::abs(diff_norm);
//		std::cout << "Diff norm " << diff_norm << std::endl;
//		assert(diff_norm >= 0);
//		assert(diff_norm <= oldnorm);
//
//		//Normalizes it
//		double normalizer_own = 1 / oldnorm;
//
//		//Between 0 and 1 between 0 and oldnorm
//// 		double diff_norm_normalized = 1 - (diff_norm * normalizer_own);
//// 		double min = 0;
//// 		double max = oldnorm;
//// 		double max_range = 1;
//// 		double min_range = 0;
//
//		//Between 0 and 1 between oldnorm / 2 and oldnorm
//		//This is between 0 and oldnorm/2 because we work on the diff and not on the length ==> best length is 0 diff and worse will be half of oldnorm
//		double min = 0;
//		double max = (oldnorm / 2);
//		double max_range = 1;
//		double min_range = 0;
//
//		double diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
//
//		double new_cov = diff_norm_normalized;
//
//		std::cout << "min " << min << " max " << max << " max_range " << max_range << " min_range " << min_range << " diff norm  " << diff_norm << " cov " << new_cov << std::endl;
//
//		assert(new_cov <= 1);
//
//		//Sometime the optimization in one turn goes under the limit so need to correct those cases ;)
//// 		assert(new_cov >= 0);
//
//		if(new_cov <= 0.001){
//			//Apparently the vaqlue in the edge does not get changed so it's useless modifying it ?
//			//See :
//// 			double tre[3];
//// 			(*it)->getMeasurementData(tre);
//// 			Eigen::Vector2d length; length << tre[0], tre[1] ;
//
//
//			new_cov = 0.001;
//		}
//
//		//Scale it again.depending on user inputed value
//		if(_use_user_prior_cov == true){
//			new_cov = new_cov * _priorNoise(0);
//			assert(new_cov <= _priorNoise(0));
//			assert(new_cov >= 0);
//		}
//		else{
//		//Scale it again. depending on oldnorm/10
//			new_cov = new_cov * (oldnorm / 10);
//			assert(new_cov <= (oldnorm / 10));
//			assert(new_cov >= 0);
//		}
//
//
//	// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
//		std::pair<double, double> eigenval(new_cov, _priorNoise(1));
//
//		std::cout << "Eigen vec " << eigenvec << " egenval " << eigenval.first << " " << eigenval.second << std::endl;
//
//		Eigen::Matrix2d cov = getCovarianceSingleEigenVector(eigenvec, eigenval);
//
//	// 			std::cout << "Covariance prior " << std::endl << cov.format(cleanFmt) << std::endl;
//
//		Eigen::Matrix3d covariance_prior;
//		covariance_prior.fill(0.);
//		covariance_prior(0, 0) = cov(0, 0);
//		covariance_prior(0, 1) = cov(0, 1);
//		covariance_prior(1, 0) = cov(1, 0);
//		covariance_prior(1, 1) = cov(1, 1);
//	// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
//		covariance_prior(2, 2) = _prior_rot * _prior_rot;
//		Eigen::Matrix3d information_prior = covariance_prior.inverse();
//
//		std::cout << "ALL INFO \n" << information_prior << "\n new cov " << new_cov << " cov mat " << cov << std::endl;
//
//		(*it)->setInformation(information_prior);
//
//	}




}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void  AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::setKernelSizeDependingOnAge(g2o::OptimizableGraph::Edge* e, bool step){


    e->robustKernel()->setDelta(100);

    //throw std::runtime_error("DO NOT USE AGE");
	//g2o::EdgeLandmark_malcolm* v_land = dynamic_cast<g2o::EdgeLandmark_malcolm*>(e);
	//EdgePrior* v_prior = dynamic_cast<EdgePrior*>(e);
	//g2o::EdgeOdometry_malcolm* v_odom = dynamic_cast<g2o::EdgeOdometry_malcolm*>(e);

	//double age = -1;

	//assert(_min_age >= 0);
	//assert(_max_age >= 0);
	//assert(_min_age <= _max_age);

	//if(v_land != NULL){
	//	age = v_land->interface.getAge();
	//	v_land->interface.setAge(age + 1);

// 				std::cout << "kernel size : " << age << std::endl;
	//	e->robustKernel()->setDelta(100);
	//}
	//else if(v_prior != NULL){
	//	age = v_prior->interface.getAge();
	//	v_prior->interface.setAge(age + 1);

// 				std::cout << "kernel size : " << age << std::endl;
	//	e->robustKernel()->setDelta(100);
//
	//}
	//else if(v_odom != NULL){
	//	age = v_odom->interface.getAge();
	//	v_odom->interface.setAge(age + 1);

// 				std::cout << "kernel size : " << age << std::endl;
	//	e->robustKernel()->setDelta(100);

	//}

// 			std::cout << "AGE : " << age << std::endl;

// 			age = 1 / age;


}






template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::getExtremaPrior(double& size_x, double& size_y) const{
	auto edges = _prior->getEdges();

	double max_x, min_x, max_y, min_y;
	bool flag_init = false;
	auto it = edges.begin();
	for(it ; it != edges.end() ; ++it){
		for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
			geometry_msgs::Point p;
			g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
			auto vertex = ptr->estimate().toVector();
			if(flag_init == false){
				flag_init = true;
				max_x = vertex(0);
				max_y = vertex(1);
				min_x = vertex(0);
				min_y = vertex(1);
			}
			else{
				if(max_x < vertex(0)){
					max_x = vertex(0);
				}
				if(max_y < vertex(1)){
					max_y = vertex(1);
				}
				if(min_x > vertex(0)){
					min_x = vertex(0);
				}
				if(min_y > vertex(1)){
					min_y = vertex(1);
				}
			}

		}

	}

	max_x = std::abs(max_x);
	max_y = std::abs(max_y);
	min_x = std::abs(min_x);
	min_y = std::abs(min_y);

	if(max_x > min_x){
		size_x = max_x + 10;
	}
	else{
		size_x = min_x + 10;
	}
	if(max_y > min_y){
		size_y = max_y + 10;
	}
	else{
		size_y = min_y + 10;
	}
}



////OPTIMIZATION

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::prepare(){
	_optimizable_graph.prepare();
}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline int AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::optimize_simple(int max_iter){

    std::cout << "Check before optimization" << std::endl;
    testInfoNonNul("At first");

	int count = 0;
	std::deque<double> _chi_kernel;
	for ( ; count < max_iter && count < 2 ; ++count){
		ROS_INFO( "Optimizing two times at least because we need to " );
		_optimizable_graph.optimize(1);
		_optimizable_graph.computeActiveErrors();
		_chi_kernel.push_back(_optimizable_graph.chi2());
		saveErrorStep();

		testNoNanInPrior("optimized with huber");
		testInfoNonNul("optimized with huber");
		testNoNanInPrior("update prior edge cov after opti huber");
	}
	if(max_iter >= 2){
		while(ErrorStable(_chi_kernel) == false && count < max_iter){
			count++;
			ROS_INFO( "Optimizing until error stops it " );
			_optimizable_graph.optimize(1);
			_optimizable_graph.computeActiveErrors();
			_chi_kernel.push_back(_optimizable_graph.chi2());
			saveErrorStep();

			testNoNanInPrior("optimized with huber");
			testInfoNonNul("optimized with huber");
			testNoNanInPrior("update prior edge cov after opti huber");
		}
	}

	ROS_INFO_STREAM("optimized " << count << " times.");

	if(count >= max_iter){

	    ROS_ERROR( "\n\n****** ATTENTION THE OPTIMIZATION PROBABLY FAILED. It iterated for too long, please double check the resulting map ! *****\n" );

	}

    return count;
	//int a;
	//std::cout << "Enter anything to pass " <<count << std::endl;
	//std::cin >> a;

}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline std::pair<int, int> AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::optimize(int max_iter){

    int count_huber = 0;
    int count_dcs = 0;

	_chi2s.clear();
	std::cout << "Checking that all info mat are positive semi def" << std::endl;
	assert(_optimizable_graph.verifyInformationMatrices(true) == true);

	ROS_DEBUG_STREAM("BEFORE THE OPTIMIZATION BUT AFTER ADDING A NODE" );

	/********** HUBER kernel ***********/

// 			_optimizable_graph.setHuberKernel();

	_flag_optimize = checkAbleToOptimize();

	if(_flag_optimize == true){
		ROS_DEBUG_STREAM( "OPTIMIZE" );
// 				checkRobotPoseNotMoved("before opti");

        if(_flag_use_robust_kernel){
            if(_flag_use_huber_kernel){
                std::cout << "Using Huber kernel" << std::endl;
                setAgeingHuberKernel();

    // 				checkRobotPoseNotMoved("set age in huber kernel");
                testNoNanInPrior("set age in huber kernel");
                testInfoNonNul("set age in huber kernel");
                testNoNanInPrior("update prior edge cov");

                count_huber = optimize_simple(max_iter);

    // 				//Avoid overshoot of the cov
    // 				for(size_t i = 0 ; i < iter ; ++i){
    // 					_optimizable_graph.optimize(1);
    // // 					checkRobotPoseNotMoved("optimized with huber");
    // 					testNoNanInPrior("optimized with huber");
    // 					testInfoNonNul("optimized with huber");
    // 					//Update prior edge covariance
    // // 					updatePriorEdgeCovariance();
    // 					testNoNanInPrior("update prior edge cov after opti huber");
    // 					saveErrorStep();
    // 				}

            }
            /********** DCS kernel ***********/
            if(_flag_use_dcs_kernel){
                std::cout << "Using DCS kernel" << std::endl;
                setAgeingDCSKernel();

                testNoNanInPrior("set age in DCS kernel");

                count_dcs = optimize_simple(max_iter);

    // 				for(size_t i = 0 ; i < iter*2 ; ++i){
    // 					_optimizable_graph.optimize(1);
    // // 					checkRobotPoseNotMoved("optimized with dcs");
    // 					testNoNanInPrior("optimized with dcs");
    // 					testInfoNonNul("optimized with dcs");
    // 					//Update prior edge covariance
    // // 					updatePriorEdgeCovariance();
    // 					testNoNanInPrior("update prior edge cov after opti dcs");
    // 					saveErrorStep();
    // 				}
            }
        }else{
            testNoNanInPrior("set age in huber kernel");
            testInfoNonNul("set age in huber kernel");
            testNoNanInPrior("update prior edge cov");

            count_huber = optimize_simple(max_iter);
        }

	}
	else{
		ROS_DEBUG_STREAM("No Optimization :(" );
	}


	//exportChi2s();

	return std::pair<int, int >(count_huber, count_dcs);

// 			checkRobotPoseNotMoved("after opti");
// 			cv::Mat d;
// 			createDescriptorNDT(d);

}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::setAgeingHuberKernel(){

// 			for (SparseOptimizer::VertexIDMap::const_iterator it = this->vertices().begin(); it != this->vertices().end(); ++it) {
// 				OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
// 				v->setMarginalized(false);
// 			}

	auto idmapedges = _optimizable_graph.edges();
	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "Robust Kern" << std::endl;
		g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*ite);
		auto huber = new g2o::RobustKernelHuber();
		e->setRobustKernel(huber);
		setKernelSizeDependingOnAge(e, true);
	}
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::setAgeingDCSKernel(){
// 			for (SparseOptimizer::VertexIDMap::const_iterator it = this->vertices().begin(); it != this->vertices().end(); ++it) {
// 				OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
// 				v->setMarginalized(false);
// 			}

	auto idmapedges = _optimizable_graph.edges();
	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "Robust Kern" << std::endl;
		g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*ite);
		auto dcs = new g2o::RobustKernelDCS();
		e->setRobustKernel(dcs);
		setKernelSizeDependingOnAge(e, false);
	}
}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline bool AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::ErrorStable(const std::deque<double>& _chi_kernel, int max_steps){
	if(_chi_kernel.size() < 2){
		throw std::runtime_error("Cannot determine error difference with less than 2 optimization cycles.");
	}
	int max_steps_count = 0;
	std::vector<double> error_diff;
	double error_mean = 0;
	auto it = _chi_kernel.rbegin();
	++it;
	for(; it != _chi_kernel.rend() ; ++it ){
		if(max_steps_count == max_steps){
			break;
		}
		error_diff.push_back(std::abs( *it - *(it - 1)) );
		error_mean = error_mean + std::abs( (*it - *(it - 1)) );
		++max_steps_count;
	}
	error_mean = error_mean / max_steps_count;

	it = _chi_kernel.rbegin();
	++it;
	max_steps_count = 0;
	for(; it != _chi_kernel.rend() ; ++it ){
		if(max_steps_count == max_steps){
			break;
		}
		ROS_DEBUG_STREAM(std::abs( *it - *(it - 1) ) << " ");
		++max_steps_count;
	}
	// std::cout << std::endl;
	ROS_INFO_STREAM("Error is " << error_mean << "threshold is " << _error_threshold_stop_optimization );
	if(error_mean < _error_threshold_stop_optimization){
		return true;
	}
	return false;
}

template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::clearPrior(){
	ROS_DEBUG_STREAM( "IMPORTANT size " << _optimizable_graph.vertices().size() );
//	int i = 0;

	for(auto vertex : _prior->getNodes()){
		_optimizable_graph.removeVertex(vertex, false);
	}
	_prior->clear();
// 			std::cout <<"Done final " << _nodes_prior.size() << " i " << i <<std::endl;
	assert(_prior->getNodes().size() == 0);

// 			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
// 				_optimizable_graph.removeVertex(it, true);
// 			}
// 			std::cout <<"clearing the edges " << std::endl;
//	_edge_prior.clear();

	//Making sure all edge prior were removed.
//	auto idmapedges = _optimizable_graph.edges();
//	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
//		assert( dynamic_cast<EdgePrior*>(*ite) == NULL );
//	}
	ROS_DEBUG_STREAM( "IMPORTANT size " << _optimizable_graph.vertices().size() );
	ROS_DEBUG_STREAM( "DONE removing " );
}


//template< typename Prior, typename VertexPrior, typename EdgePrior>
//inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::clearLinks(){
//	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
//	int i = 0;
//
//	for(auto it = _edge_link.begin() ; it != _edge_link.end() ;){
//
//		_optimizable_graph.removeEdge(*it);
//		it = _edge_link.erase(it);
//
//	}
//// 			std::cout <<"Done final " << _nodes_prior.size() << " i " << i <<std::endl;
//	assert(_edge_link.size() == 0);
//
//// 			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
//// 				_optimizable_graph.removeVertex(it, true);
//// 			}
//// 			std::cout <<"clearing the edges " << std::endl;
//
//	//Making sure all edge prior were removed.
//	auto idmapedges = _optimizable_graph.edges();
//	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
//// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
//		assert( dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite) == NULL );
//	}
//	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
//	std::cout << "DONE removing " << std::endl;
//}







//void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::checkNoRepeatingPriorEdge(){
//	for(auto it_vertex = _nodes_prior.begin() ; it_vertex != _nodes_prior.end() ; ++it_vertex){
//		std::vector<std::pair<double, double> > out;
//		// 			std::cout << "edges " << std::endl;
//		auto edges = (*it_vertex)->edges();
//		// 			std::cout << "edges done " << std::endl;
//		std::vector<g2o::EdgeSE2Prior_malcolm*> edges_prior;
//
//		for ( auto ite = edges.begin(); ite != edges.end(); ++ite ){
//			// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
//			g2o::EdgeSE2Prior_malcolm* ptr = dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite);
//			if(ptr != NULL){
//				//Make sure not pushed twice
//				for(auto ite2 = edges_prior.begin(); ite2 != edges_prior.end(); ++ite2 ){
//					assert(ptr != *ite2);
//				}
//				// 						std::cout << " pushed edges " << std::endl;
//				edges_prior.push_back(ptr);
//				// 						std::cout << "pushed edges done " << std::endl;
//			}
//		}
//		for(auto it = edges_prior.begin() ; it != edges_prior.end() ; ++it){
//			for(auto ite2 = it +1 ; ite2 != edges_prior.end() ; ++ite2 ){
//				assert((*it)->getOrientation2D(**it_vertex) != (*ite2)->getOrientation2D(**it_vertex));
//			}
//		}
//	}
//	for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
//		for(auto ite2 = it +1 ; ite2 != _edge_prior.end() ; ++ite2 ){
//			assert(it != ite2);
//		}
//	}
//}


template< typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::checkRobotPoseNotMoved(const std::string& when){
	ROS_DEBUG_STREAM("testing after " << when );
	for(auto it = _nodes_ndt.begin() ; it != _nodes_ndt.end() ; ++it){
		int init_x = (*it)->initial_transfo.toVector()(0) ;
		int init_y = (*it)->initial_transfo.toVector()(1) ;
		int init_z = (*it)->initial_transfo.toVector()(2) * 10;

		int update_x = (*it)->estimate().toVector()(0) ;
		int update_y = (*it)->estimate().toVector()(1) ;
		int update_z = (*it)->estimate().toVector()(2) * 10;

		if(init_x != update_x || init_y != update_y || init_z != update_z){
			ROS_DEBUG_STREAM(" init "  << init_x << " "<< init_y << " "<< init_z <<  " == " << update_x << " "<< update_y << " "<< update_z );
			throw std::runtime_error("MOVE BASE");
		}
	}

}
