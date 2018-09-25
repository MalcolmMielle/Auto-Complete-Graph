#include "auto_complete_graph/Localization/ACGPriorXY.hpp"

g2o::EdgeXYPriorACG* AASS::acg::AutoCompleteGraphPriorXY::addEdge(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){

	for(auto prior_edge : _edges){
		if(v1 == prior_edge->vertices()[0] && v2 == prior_edge->vertices()[1]){
			throw std::runtime_error("Edge link already added");
		}
		else if(v1 == prior_edge->vertices()[1] && v2 == prior_edge->vertices()[0]){
			throw std::runtime_error("Edge link already added");
		}
	}

	//Get Eigen vector
	g2o::VertexXYPrior* v_ptr = dynamic_cast<g2o::VertexXYPrior*>(v1);
	g2o::VertexXYPrior* v_ptr2 = dynamic_cast<g2o::VertexXYPrior*>(v2);
	Eigen::Vector2d pose1 = v_ptr->estimate();
	Eigen::Vector2d pose2 = v_ptr2->estimate();

// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;

	Eigen::Vector2d eigenvec;
	eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
	double newnorm_old = (pose1 - pose2).norm();

// 	double test_newnorm = newnorm_old / 2;
	//ATTENTION NOT A MAGIC NUMBER
	double newnorm = (newnorm_old *  _priorNoise(0)) / 100;
//	double newnorm_other = (newnorm_old *  _priorNoise(1)) / 100;

	int test_tt = newnorm * 100;
	int test_ttt = newnorm_old * 100;
// 	std::cout << newnorm << " " << test_ttt << " test " << test_tt << " base " << _priorNoise(0) << " "<< _priorNoise(1) << std::endl;

	assert(test_tt <= test_ttt);
	assert(newnorm >= 0);

	std::pair<double, double> eigenval(newnorm, _priorNoise(1));
// 	std::pair<double, double> eigenval(_priorNoise(0), _priorNoise(1));

	Eigen::Matrix2d cov = getCovarianceSingleEigenVector(eigenvec, eigenval);

	cov = cov * 1000;
	cov = cov.array().round();
	cov = cov / 1000;



// 			std::cout << "Covariance prior " << std::endl << cov << std::endl;
//	Eigen::Matrix2d covariance_prior;
//	covariance_prior.fill(0.);
//	covariance_prior(0, 0) = cov(0, 0);
//	covariance_prior(0, 1) = cov(0, 1);
//	covariance_prior(1, 0) = cov(1, 0);
//	covariance_prior(1, 1) = cov(1, 1);
// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
//	covariance_prior(2, 2) = _prior_rot * _prior_rot;

	//CHECK INVERTIBILITY OF THE MATRIX
	Eigen::Matrix2d information_prior = cov.inverse();

	information_prior = information_prior * 1000;
	information_prior = information_prior.array().round();
	information_prior = information_prior / 1000;



// 			std::cout << "Information prior " << std::endl << information_prior << std::endl;

	assert(information_prior.isZero(1e-10) == false);
	assert(information_prior == information_prior.transpose());

	g2o::EdgeXYPriorACG* priorObservation =  new g2o::EdgeXYPriorACG;
	priorObservation->vertices()[0] = v1;
	priorObservation->vertices()[1] = v2;
	Eigen::Vector3d se3_vec = se2.toVector();
	Eigen::Vector2d se_no_rotation = se3_vec.head(2);
	priorObservation->setMeasurement(se_no_rotation);
	priorObservation->setInformation(information_prior);
	priorObservation->setParameterId(0, _sensorOffset->id());

// 	priorObservation->interface.setAge(_age_start_value);
// 	priorObservation->interface.setOriginalValue(se2);

//	_optimizable_graph.addEdge(priorObservation);

// 	EdgePriorAndInitialValue epiv(priorObservation, se2);
	_edges.insert(priorObservation);

//	std::cout << "After adding an edge" << std::endl;
	checkNoRepeatingPriorEdge();

	return priorObservation;
}


g2o::VertexXYPrior* AASS::acg::AutoCompleteGraphPriorXY::addPose(const g2o::SE2& se2, const PriorAttr& priorAttr, int index){
	g2o::VertexXYPrior* priorlandmark = new g2o::VertexXYPrior();
	priorlandmark->setId(index);
	Eigen::Vector2d pose = se2.toVector().head(2);
	priorlandmark->setEstimate(pose);
	priorlandmark->priorattr = priorAttr;

	//Check that the node is not here in double
	for(auto node : _nodes){
		assert(node->estimate() != priorlandmark->estimate() );
	}

	_nodes.insert(priorlandmark);
	return priorlandmark;
}
g2o::VertexXYPrior* AASS::acg::AutoCompleteGraphPriorXY::addPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr, int index){
	g2o::SE2 se2(lan(0), lan(1), lan(2));
	return addPose(se2, priorAttr, index);
}
g2o::VertexXYPrior* AASS::acg::AutoCompleteGraphPriorXY::addPose(double x, double y, double theta, const AASS::acg::PriorAttr& priorAttr, int index){
	Eigen::Vector3d lan;
	lan << x, y, theta;
	return addPose(lan, priorAttr, index);
}






int AASS::acg::AutoCompleteGraphPriorXY::addPriorGraph(const PriorLoaderInterface::PriorGraph& graph, int first_index){

	std::pair< PriorLoaderInterface::PriorGraph::VertexIterator, PriorLoaderInterface::PriorGraph::VertexIterator > vp;
	std::deque<PriorLoaderInterface::PriorGraph::Vertex> vec_deque;
	std::vector<g2o::VertexXYPrior*> out_prior;

//	std::cout << "NOOOOOOW" << _optimizable_graph.vertices().size() << std::endl << std::endl;

	assert( _nodes.size() == 0 );

	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
		auto v = *vp.first;
		//ATTENTION Magic number

// 		std::cout << "Prior Landmark : " << graph[v].getX() << " " << graph[v].getY() << std::endl;


		g2o::VertexXYPrior* res = addPose(graph[v].getX(), graph[v].getY(), 0, graph[v], first_index);
		first_index++;
		vec_deque.push_back(v);
		out_prior.push_back(res);
// 		_nodes.push_back(res);
	}

	assert( _edges.size() == 0);

	int count = 0;
	int self_link = 0 ;
	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
		auto v = *vp.first;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Edge e;

		std::vector<PriorLoaderInterface::PriorGraph::Vertex> tmp_for_double_edges;

		for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph));
		     out_i != out_end; ++out_i) {
			e = *out_i;
// 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e, (graph));
			auto targ = boost::target(e, (graph));

			//Avoiding double edge to the same vertex target but that is not a self loop!

			int idx = -1;
			for(size_t ii = count ; ii < vec_deque.size() ; ++ii){
				if(targ == vec_deque[ii]){
					idx = ii;
				}
			}
			for(auto it_tmp_double = tmp_for_double_edges.begin() ; it_tmp_double != tmp_for_double_edges.end() ; ++it_tmp_double){
				if(*it_tmp_double == targ){
					//it's a double we cancel idx and add it to the self_lopp count
					idx = count;
				}
			}
			tmp_for_double_edges.push_back(targ);

			if(idx == -1){
				//SKIP
// 				throw std::runtime_error("Didn't find the vertex target :(");
			}
			else if(idx == count){
				self_link = self_link + 1 ;
			}
			else{

				assert(idx != count);

				g2o::VertexXYPrior* from = out_prior[count]; //<-Base
				g2o::VertexXYPrior* toward = out_prior[idx]; //<-Targ

				double x_diff = graph[targ].getX() - graph[v].getX();
				double y_diff = graph[targ].getY() - graph[v].getY();

// 				std::cout << "Index" << idx << " " <<count << std::endl;

// 				std::cout << "Because : " << graph[targ].getX() << " - " << graph[v].getX() << " and " << graph[targ].getY() << " - " << graph[v].getY() << std::endl;

// 				std::cout << "diff: " <<x_diff << " " << y_diff << std::endl;

				g2o::SE2 se2(x_diff, y_diff, 0);

// 				std::cout << "SE2 pushed in edge \n" << se2.toVector() << std::endl;

				auto edge_out = addEdge(se2, from, toward);
// 				_edges.push_back(edge_out);
			}

		}
		++count;
	}

// 	std::cout << _edges.size() << " == " << graph.getNumEdges() << " - " << self_link / 2 << std::endl;
// 	std::cout << _nodes.size() << " == " << graph.getNumVertices() << std::endl;

	assert( _nodes.size() == graph.getNumVertices() );
	//Self link / 2 because they are seen twice
	assert( _edges.size() == graph.getNumEdges() - (self_link / 2) );

	std::cout << "After update graph prior" << std::endl;
	checkNoRepeatingPriorEdge();

	return first_index;

}

//void AASS::acg::AutoCompleteGraphPriorXY::clear(){
////	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
//	int i = 0;
//	for(auto it = _nodes.begin() ; it != _nodes.end() ; ++it){
//
//		for(auto it1 = it + 1 ; it1 != _nodes.end() ;++it1){
//			assert(*it != *it1);
//			++i;
//		}
//	}
//
//
//	for(auto it = _nodes.begin() ; it != _nodes.end() ;){
//// 				auto it_tmp = it;
//// 				assert(*it_tmp == *it);
//// 				++it;
//// 				assert(*it_tmp != *it);
//// 				std::cout <<"removing the vertex " << *it << std::endl;
//// 				if (it != _nodes.end()) {
//// 					std::cout <<"Done " << _nodes.size() <<std::endl;
//		//DIESNT WORK :(
//// 					this->removeVertex(*it);
//		_optimizable_graph.removeVertex(*it, false);
//		it = _nodes.erase(it);
//// 					std::cout <<"removed the vertex " << std::endl;
//// 				}
//// 				it = it_tmp;
//// 				++it;
//
//	}
//// 			std::cout <<"Done final " << _nodes.size() << " i " << i <<std::endl;
//	assert(_nodes.size() == 0);
//
//// 			for(auto it = _edges.begin() ; it != _edges.end() ; ++it){
//// 				_optimizable_graph.removeVertex(it, true);
//// 			}
//// 			std::cout <<"clearing the edges " << std::endl;
//	_edges.clear();
//	_edge_link.clear();
//	_edge_interface_of_links.clear();
//
//	//Making sure all edge prior were removed.
//	auto idmapedges = _optimizable_graph.edges();
//	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
//// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeXYPrior*>(*ite) << std::endl;
//		assert( dynamic_cast<g2o::EdgeXYPrior*>(*ite) == NULL );
//		assert( dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite) == NULL );
//	}
//	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
//	std::cout << "DONE removing " << std::endl;
//}

//void AASS::acg::AutoCompleteGraphPriorXY::checkNoRepeatingPriorEdge(){
//	for(auto it_vertex = _nodes.begin() ; it_vertex != _nodes.end() ; ++it_vertex){
//		std::vector<std::pair<double, double> > out;
//		// 			std::cout << "edges " << std::endl;
//		auto edges = (*it_vertex)->edges();
//		// 			std::cout << "edges done " << std::endl;
//		std::vector<g2o::EdgeXYPrior*> edges_prior;
//
//		for ( auto ite = edges.begin(); ite != edges.end(); ++ite ){
//			// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeXYPrior*>(*ite) << std::endl;
//			g2o::EdgeXYPrior* ptr = dynamic_cast<g2o::EdgeXYPrior*>(*ite);
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
//			auto ite2 = it;
//			ite2++;
//			for( ; ite2 != edges_prior.end() ; ++ite2 ){
//				assert((*it)->getOrientation2D(**it_vertex) != (*ite2)->getOrientation2D(**it_vertex));
//			}
//		}
//	}
//	for(auto it = _edges.begin() ; it != _edges.end() ; ++it){
//		auto ite2 = it;
//		ite2++;
//		for(; ite2 != _edges.end() ; ++ite2 ){
//			assert(it != ite2);
//		}
//	}
//}



void AASS::acg::AutoCompleteGraphPriorXY::updatePriorEdgeCovariance()
{

	std::cout << "DO NOT USE " << std::endl;
	testNoNanInPrior("no dtat");
	assert(false);

	auto edges = getEdges();
	auto it = edges.begin();
	for(it ; it != edges.end() ; ++it){
		g2o::VertexXYPrior* v_ptr = dynamic_cast<g2o::VertexXYPrior*>((*it)->vertices()[0]);
		if(v_ptr == NULL){
			throw std::runtime_error("no");
		}
		g2o::VertexXYPrior* v_ptr2 = dynamic_cast<g2o::VertexXYPrior*>((*it)->vertices()[1]);
		if(v_ptr2 == NULL){
			throw std::runtime_error("no2");
		}
		Eigen::Vector2d pose1 = v_ptr->estimate();
		Eigen::Vector2d pose2 = v_ptr2->estimate();

		// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
		// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;

		double tre[3];
		(*it)->getMeasurementData(tre);

		Eigen::Vector2d length; length << tre[0], tre[1] ;

		Eigen::Vector2d eigenvec;
		eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);

		double newnorm = (pose1 - pose2).norm();
		double len_norm = length.norm();
		std::cout << "new norm " << newnorm << " because " << pose1 << " " << pose2 << " and lennorm " << len_norm << "because  " <<length << std::endl;
		g2o::SE2 oldnormse2 = (*it)->interface.getOriginalValue();
		Eigen::Vector3d vecold = oldnormse2.toVector();
		double oldnorm = (vecold).norm();
		std::cout << "oldnorm" << oldnorm << std::endl;
		assert(oldnorm >= 0);

		//Using the diff so we cannot shrink it or stretch it easily.
		double diff_norm = std::abs(oldnorm - newnorm);
		std::cout << "Diff norm " << diff_norm << std::endl;
		assert(diff_norm >= 0);
		if(diff_norm > oldnorm){
			diff_norm = oldnorm;
		}
		diff_norm = std::abs(diff_norm);
		std::cout << "Diff norm " << diff_norm << std::endl;
		assert(diff_norm >= 0);
		assert(diff_norm <= oldnorm);

		//Normalizes it
		double normalizer_own = 1 / oldnorm;

		//Between 0 and 1 between 0 and oldnorm
// 		double diff_norm_normalized = 1 - (diff_norm * normalizer_own);
// 		double min = 0;
// 		double max = oldnorm;
// 		double max_range = 1;
// 		double min_range = 0;

		//Between 0 and 1 between oldnorm / 2 and oldnorm
		//This is between 0 and oldnorm/2 because we work on the diff and not on the length ==> best length is 0 diff and worse will be half of oldnorm
		double min = 0;
		double max = (oldnorm / 2);
		double max_range = 1;
		double min_range = 0;

		double diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );

		double new_cov = diff_norm_normalized;

		std::cout << "min " << min << " max " << max << " max_range " << max_range << " min_range " << min_range << " diff norm  " << diff_norm << " cov " << new_cov << std::endl;

		assert(new_cov <= 1);

		//Sometime the optimization in one turn goes under the limit so need to correct those cases ;)
// 		assert(new_cov >= 0);

		if(new_cov <= 0.001){
			//Apparently the vaqlue in the edge does not get changed so it's useless modifying it ?
			//See :
// 			double tre[3];
// 			(*it)->getMeasurementData(tre);
// 			Eigen::Vector2d length; length << tre[0], tre[1] ;


			new_cov = 0.001;
		}

		//Scale it again.depending on user inputed value
		if(_use_user_prior_cov == true){
			new_cov = new_cov * _priorNoise(0);
			assert(new_cov <= _priorNoise(0));
			assert(new_cov >= 0);
		}
		else{
			//Scale it again. depending on oldnorm/10
			new_cov = new_cov * (oldnorm / 10);
			assert(new_cov <= (oldnorm / 10));
			assert(new_cov >= 0);
		}


		// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
		std::pair<double, double> eigenval(new_cov, _priorNoise(1));

		std::cout << "Eigen vec " << eigenvec << " egenval " << eigenval.first << " " << eigenval.second << std::endl;

		Eigen::Matrix2d cov = getCovarianceSingleEigenVector(eigenvec, eigenval);

		// 			std::cout << "Covariance prior " << std::endl << cov.format(cleanFmt) << std::endl;

//		Eigen::Matrix3d covariance_prior;
//		covariance_prior.fill(0.);
//		covariance_prior(0, 0) = cov(0, 0);
//		covariance_prior(0, 1) = cov(0, 1);
//		covariance_prior(1, 0) = cov(1, 0);
//		covariance_prior(1, 1) = cov(1, 1);
		// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
//		covariance_prior(2, 2) = _prior_rot * _prior_rot;
		Eigen::Matrix2d information_prior = cov.inverse();

		std::cout << "ALL INFO \n" << information_prior << "\n new cov " << new_cov << " cov mat " << cov << std::endl;

		(*it)->setInformation(information_prior);

	}




}


void AASS::acg::AutoCompleteGraphPriorXY::testNoNanInPrior(const std::string& before) const {

//	std::cout << "Test No nan in prior after " << before << std::endl;
	auto it = getNodes().begin();
	for(it ; it != getNodes().end() ; ++it){
		g2o::VertexXYPrior* v_ptr = dynamic_cast<g2o::VertexXYPrior*>((*it));
		if(v_ptr == NULL){
			throw std::runtime_error("not good vertex type");
		}
		Eigen::Vector2d pose1 = v_ptr->estimate();
		assert(!std::isnan(pose1[0]));
		assert(!std::isnan(pose1[1]));
//		assert(!std::isnan(pose1[2]));

	}

//	std::cout << "Testing the edges now" << std::endl;

	auto edges = getEdges();
	auto it_edge = edges.begin();
	for(it_edge ; it_edge != edges.end() ; ++it_edge){
		g2o::VertexXYPrior* v_ptr = dynamic_cast<g2o::VertexXYPrior*>((*it_edge)->vertices()[0]);
		if(v_ptr == NULL){
			throw std::runtime_error("no");
		}
		g2o::VertexXYPrior* v_ptr2 = dynamic_cast<g2o::VertexXYPrior*>((*it_edge)->vertices()[1]);
		if(v_ptr2 == NULL){
			throw std::runtime_error("no2");
		}
		Eigen::Vector2d pose1 = v_ptr->estimate();
		Eigen::Vector2d pose2 = v_ptr2->estimate();

		assert(!std::isnan(pose1[0]));
		assert(!std::isnan(pose1[1]));
//		assert(!std::isnan(pose1[2]));

		assert(!std::isnan(pose2[0]));
		assert(!std::isnan(pose2[1]));
//		assert(!std::isnan(pose2[2]));
	}

}

pcl::PointCloud<pcl::PointXYZ>::Ptr AASS::acg::AutoCompleteGraphPriorXY::toPointCloud(double resolution, double z_elevation, double varz) const {

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
	int nb_points = 0;
	auto edges = getEdges();
//	std::cout << "Converting edges : " << edges.size() << std::endl;

	for(auto it = edges.begin(); it != edges.end() ; ++it){

		std::vector<Eigen::Vector3d> points;

		for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
//			geometry_msgs::Point p;
			g2o::VertexXYPrior* ptr = dynamic_cast<g2o::VertexXYPrior*>((*ite2));
			auto vertex_t = ptr->estimate();
			Eigen::Vector3d vertex; vertex << vertex_t(0), vertex_t(1), z_elevation;
//			vertex[2] = z_elevation;
			//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
//				Eigen::Vector2d veve; veve << vertex(0), vertex(1);
//                 std::cout << "Pushing " << veve << std::endl;
			points.push_back(vertex);
		}
		assert(points.size() == 2);

		Eigen::Vector3d slope = points[1] - points[0];
//			std::cout << "from " << points[0] << " tot " << points[1] << " slope " << slope << std::endl;


		slope = slope / slope.norm();
//			std::cout << "Then slope " << slope << std::endl;
		slope = slope * resolution;
//			std::cout << "Final slope " << slope << std::endl;

//			int wait;
//			std::cin>>wait;

		Eigen::Vector3d point = points[0];

		pcl::PointXYZ pcl_point;
		pcl_point.x = point[0];
		pcl_point.y = point[1];
		pcl_point.z = point[2];
		pcl_pc->push_back(pcl_point);
		nb_points++;

		while( (points[1] - point).norm() >= resolution) {

//				std::cout << "Adding point " << pcl_point.x << " " << pcl_point.y << " " << pcl_point.z << " nbpt " << nb_points << " "  << slope[0] << " " << slope[1] << " " << slope[2] << std::endl;
			point = point + slope;
			pcl_point.x = point[0];
			pcl_point.y = point[1];
			pcl_point.z = point[2];
			pcl_pc->push_back(pcl_point);
			nb_points++;

		}

	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_noise(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ pt;
	//add some variance on z
	for(int i=0; i<pcl_pc->points.size(); i++) {
		pt = pcl_pc->points[i];
		pt.z += varz*((double)rand())/(double)INT_MAX;
		pcl_pc_noise->points.push_back(pt);

	}

//	std::cout << "Adding data " << nb_points << std::endl;

	pcl_pc_noise->width = nb_points;
	pcl_pc_noise->height = 1;
	pcl_pc_noise->is_dense = false;

	return pcl_pc_noise;

}