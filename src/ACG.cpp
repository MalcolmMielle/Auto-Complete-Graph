#include "auto_complete_graph/ACG.hpp"


g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraph::addLinkBetweenMaps(const g2o::Vector2& pos, g2o::VertexSE2Prior* v2, g2o::VertexLandmarkNDT* v1){
	std::cout << "Adding link" << std::endl;

	for(auto link_edge : _edge_link){
		if(v1 == link_edge->vertices()[0] && v2 == link_edge->vertices()[1]){
			throw std::runtime_error("Edge link already added");
		}
		else if(v1 == link_edge->vertices()[1] && v2 == link_edge->vertices()[0]){
			throw std::runtime_error("Edge link already added");
		}
	}


	Eigen::Matrix2d covariance_link;
	covariance_link.fill(0.);
	covariance_link(0, 0) = _linkNoise[0]*_linkNoise[0];
	covariance_link(1, 1) = _linkNoise[1]*_linkNoise[1];

// 	std::cout << "Link cov " << covariance_link << std::endl;

// 			covariance_link(2, 2) = 13;//<- Rotation covariance link is more than 4PI
	Eigen::Matrix2d information_link = covariance_link.inverse();

	assert(information_link.isZero(1e-10) == false);
// 	std::cout << "Link cov2 " << covariance_link << std::endl;

	g2o::EdgeLinkXY_malcolm* linkObservation = new g2o::EdgeLinkXY_malcolm;

// 	std::cout << "Link cov3 " << covariance_link << std::endl;
// 	g2o::HyperGraph::Vertex* from = dynamic_cast<g2o::HyperGraph::Vertex*>(v2);
	assert(v2 != NULL);
	linkObservation->vertices()[0] = v2;
	assert(linkObservation->vertices()[0] == v2);
// 	std::cout << "Link cov4 " << covariance_link << std::endl;
// 	g2o::HyperGraph::Vertex* toward = dynamic_cast<g2o::HyperGraph::Vertex*>(v1);
	assert(v1 != NULL);
	linkObservation->vertices()[1] = v1;
	assert(linkObservation->vertices()[1] == v1);
// 	std::cout << "Link cov5 " << covariance_link << std::endl;
	linkObservation->setMeasurement(pos);
// 	std::cout << "Link cov6 " << covariance_link << std::endl;
	linkObservation->setInformation(information_link);
// 	std::cout << "Link cov7 " << _sensorOffset->id() << std::endl;
	linkObservation->setParameterId(0, _sensorOffset->id());

// 	std::cout << "Adding edge!" << v2 << " and " << linkObservation->vertices()[0] << " at " << __LINE__ << " " << __FILE__<< "age start " << _age_start_value << " for " << /*linkObservation->interface->getAge() <<*/ std::endl;
	//Adding the link age

	//ATTENTION NOT WORKING FOR I DON'T KNOW WHAT REASON
// 	bool outout = false;
// 	std::cout << "Bool" << outout << std::endl;
// 	outout = linkObservation->interface->setAge(_age_start_value);
// 	std::cout << "Bool" << outout << std::endl;
// 	assert(outout == 1);
// 	std::cout << "Age set" << std::endl;
// 	assert(linkObservation->interface->manuallySetAge() == true);

	//HACK REPLACEMENT
	EdgeInterface edgeinter;
// 	std::cout << "Setting age value" << std::endl;
// 	std::cout << "Setting age value" << std::endl;
	edgeinter.setAge(_age_start_value);
// 	std::cout << "Manueal" << std::endl;
	assert(edgeinter.manuallySetAge() == true);

	_optimizable_graph.addEdge(linkObservation);
	_edge_link.push_back(linkObservation);
	_edge_interface_of_links.push_back(edgeinter);

	assert(_edge_interface_of_links.size() == _edge_link.size());

// 	std::cout << "Done" << std::endl;
	return linkObservation;
}


//g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraphBase<Prior, g2o::VertexSE2Prior, EdgePrior>::addLinkBetweenMaps(const g2o::Vector2& pos, int from_id, int toward_id){
//	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
//	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
//
//	g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>(from_ptr);
//	g2o::VertexLandmarkNDT* ptr2;
//	if(ptr != NULL){
//		g2o::VertexLandmarkNDT* ptr2 = dynamic_cast<g2o::VertexLandmarkNDT*>(toward_ptr);
//		if(ptr2 == NULL){
//			throw std::runtime_error("Pointers are not of compatible type. First pointer should be prior while the second is not a PointXY while it should");
//		}
//	}
//// 	else{
//// 		ptr = dynamic_cast<g2o::VertexSE2Prior*>(toward_ptr);
//// 		if(ptr != NULL){
//// 			ptr2 = dynamic_cast<g2o::VertexLandmarkNDT*>(from_ptr);
//// 			if(ptr2 == NULL){
//// 				throw std::runtime_error("Pointers are not of compatible type. Second pointer is a SE2 while the first is not a PointXY");
//// 			}
//// 		}
//// 		else{
//// 			throw std::runtime_error("Pointers are not of compatible type. No pointer point to a VertexSE2Prior");
//// 		}
//// 	}
//	return addLinkBetweenMaps(pos, ptr, ptr2);
//}


std::vector <g2o::EdgeLinkXY_malcolm* >::iterator AASS::acg::AutoCompleteGraph::removeLinkBetweenMaps(g2o::EdgeLinkXY_malcolm* v1)
{

	std::vector <g2o::EdgeLinkXY_malcolm* >::iterator next_el;
	auto it = _edge_link.begin();
	int size = _edge_link.size();
	int place = 0;
	for(it; it != _edge_link.end() ;){
		if(*it == v1){
			std::cout << "Found"<<std::endl;
			next_el = _edge_link.erase(it);
			break;
		}
		else{
			place++;
			it++;
		}
	}

	assert(place != size);
	_edge_interface_of_links.erase(_edge_interface_of_links.begin() + place);

	_optimizable_graph.removeEdge(v1);
	assert(_edge_link.size() == size - 1);
	assert(_edge_interface_of_links.size() == _edge_link.size());

	std::cout << "Out of remove edge" << std::endl;

	return next_el;

}



void AASS::acg::AutoCompleteGraph::updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph, bool noise_flag, double deviation){


	AutoCompleteGraphBase<AutoCompleteGraphPriorSE2, g2o::VertexSE2Prior, g2o::EdgeSE2Prior_malcolm>::updateNDTGraph(ndt_graph, noise_flag, deviation);

	//TODO, update links using the newly found landmarks. No need to update the rest obviously (<-HAHA that was so wrrrrongggg need to update it since they moved :P!)

	updateLinks();

	std::cout << "Test no double links" << std::endl;
	noDoubleLinks();

	std::cout << "After " << std::endl;
// 	printCellsNum();

}

void AASS::acg::AutoCompleteGraph::updateLinks()
{

	if(_use_links_prior) {
		std::cout << "Create new links" << std::endl;
// 	int count = countLinkToMake();
		int count2 = createNewLinks();
// 	if(count != count2){
// 		std::cout << "Weird different detection" << std::endl;
// 		throw std::runtime_error("ARF NOT GOOD COUNT in update link");
// 	}
		std::cout << "updateLink no small forgotten" << std::endl;
// 	checkLinkNotForgotten();
		removeBadLinks();
		std::cout << "update link not too big" << std::endl;
		checkLinkNotTooBig();
	}
}

void AASS::acg::AutoCompleteGraph::testInfoNonNul(const std::string& before) const {

	AutoCompleteGraphBase<AutoCompleteGraphPriorSE2, g2o::VertexSE2Prior, g2o::EdgeSE2Prior_malcolm>::testInfoNonNul(before);

	std::cout << "Test info non nul after " << before << std::endl;
	auto idmapedges = _optimizable_graph.edges();

	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
		assert((*ite)->vertices().size() >= 1);

		g2o::EdgeLinkXY_malcolm* ptr3 = dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite);
		if(ptr3 != NULL){
			assert(ptr3->information().isZero(1e-10) == false);
		}

	}


}



bool AASS::acg::AutoCompleteGraph::linkAlreadyExist(g2o::VertexLandmarkNDT* v_pt, g2o::VertexSE2Prior* v_prior)
{
	auto it = _edge_link.begin();
	linkAlreadyExist(v_pt, v_prior, it);
}



bool AASS::acg::AutoCompleteGraph::linkAlreadyExist(g2o::VertexLandmarkNDT* v_pt, g2o::VertexSE2Prior* v_prior, std::vector <g2o::EdgeLinkXY_malcolm* >::iterator& it)
{
// 	std::cout << "Testing links" << std::endl;
	for (it ; it != _edge_link.end() ; ++it){

		assert((*it)->vertices().size() == 2);

// 		std::cout << " LINK " << _edge_link.size() << std::endl;
		g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>((*it)->vertices()[0]);
		if(ptr == NULL){
			std::cout << ptr << " and " << (*it)->vertices()[0] << std::endl;
			throw std::runtime_error("Links do not have the good vertex type. Prior lae");
		}
		g2o::VertexLandmarkNDT* ptr2 = dynamic_cast<g2o::VertexLandmarkNDT*>((*it)->vertices()[1]);
		if(ptr2 == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Landmark");
		}

// 		std::cout << "if Testing links" << std::endl;
		if(v_pt == ptr2 && v_prior == ptr){
			return true;
		}

// 		std::cout << "after Testing links" << std::endl;

	}
	return false;

}


bool AASS::acg::AutoCompleteGraph::noDoubleLinks()
{
	auto it = _edge_link.begin();
	for (it ; it != _edge_link.end() ; ){

		assert((*it)->vertices().size() == 2);

// 		std::cout << " LINK " << _edge_link.size() << std::endl;
		g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>((*it)->vertices()[0]);
		if(ptr == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Prior ndl");
		}
		g2o::VertexLandmarkNDT* ptr2 = dynamic_cast<g2o::VertexLandmarkNDT*>((*it)->vertices()[1]);
		if(ptr2 == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Landmark");
		}

// 		for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
// 			g2o::VertexSE2Prior* ptr_tmp = dynamic_cast<g2o::VertexSE2Prior*>((*ite2));
// 			g2o::VertexLandmarkNDT* ptr2_tmp = dynamic_cast<g2o::VertexLandmarkNDT*>((*ite2));
//
// 			if(ptr_tmp != NULL){
// 				std::cout << "Got a VertexSE2" << std::endl;
// 				ptr = ptr_tmp;
// 			}
// 			else if(ptr2_tmp != NULL){
// 				std::cout << "Got a VertexPOINTXY" << std::endl;
// 				ptr2 = ptr2_tmp;
// 			}
// 			else{
// 				throw std::runtime_error("Links do not have the good vertex type");
// 			}
// 		}

		++it;
		if( it != _edge_link.end()){
			auto it_tmp = it;
			if(linkAlreadyExist(ptr2, ptr, it_tmp) == true){
				return false;
			}
		}

	}
	return true;

}

void  AASS::acg::AutoCompleteGraph::setKernelSizeDependingOnAge(g2o::OptimizableGraph::Edge* e, bool step){

	AutoCompleteGraphBase<AutoCompleteGraphPriorSE2, g2o::VertexSE2Prior, g2o::EdgeSE2Prior_malcolm>::setKernelSizeDependingOnAge(e, step);


	g2o::EdgeLinkXY_malcolm* v_linkxy = dynamic_cast<g2o::EdgeLinkXY_malcolm*>(e);
	double age = -1;

	assert(_min_age >= 0);
	assert(_max_age >= 0);
	assert(_min_age <= _max_age);

	if(v_linkxy != NULL){
// 		assert(v_linkxy->interface->manuallySetAge());
// 		age = v_linkxy->interface->getAge();

		//Finding age
		int place = -1;
		for(int i = 0; i < _edge_link.size() ; ++i){
			if(_edge_link[i] == v_linkxy){
				place = i;
				break;
			}
		}

		assert(place != -1);
		assert(_edge_interface_of_links[place].manuallySetAge());
		age = _edge_interface_of_links[place].getAge();

		//Updating age: The step can go up and down it doesn't matter
// 		std::cout << "Age at first " << age <<std::endl;
		if(step == true){
			if(age + _age_step <= _max_age && age + _age_step >= _min_age || _max_age == -1){
				_edge_interface_of_links[place].setAge(age + _age_step);
				double age_tmp = _edge_interface_of_links.at(place).getAge();
			}
			else if (age + _age_step > _max_age){
				_edge_interface_of_links[place].setAge(_max_age);
				double age_tmp = _edge_interface_of_links.at(place).getAge();
				assert(age_tmp >= age);
			}
			else if (age + _age_step < _min_age){
				_edge_interface_of_links[place].setAge(_min_age);
				double age_tmp = _edge_interface_of_links.at(place).getAge();
				assert(age_tmp <= age);
			}
		}

		assert(_edge_interface_of_links.at(place).getAge() <= _max_age);
		assert(_edge_interface_of_links.at(place).getAge() >= _min_age);

		//Keep going
// 		std::cout << "kernel size : " << age << " age max " << _max_age << " start " << _age_start_value << std::endl;
		e->robustKernel()->setDelta(age);
		if(_max_age != -1){
			assert(age <= _max_age);
		}

	}
	// 			std::cout << "AGE : " << age << std::endl;

// 			age = 1 / age;


}

std::pair<int, int > AASS::acg::AutoCompleteGraph::optimize(int max_iter){


	std::cout << "BEFORE THE OPTIMIZATION BUT AFTER ADDING A NODE" << std::endl;
	overCheckLinks();

	auto ret = AutoCompleteGraphBase<AutoCompleteGraphPriorSE2, g2o::VertexSE2Prior, g2o::EdgeSE2Prior_malcolm>::optimize(max_iter);

	std::cout << "AFTER THE OPTIMIZATION CREATE" << std::endl;
	int count = countLinkToMake();
	int count2 = createNewLinks();
// 			if(count != count2){
// 				std::cout << "Weird different detection" << std::endl;
// 				throw std::runtime_error("ARF NOT GOOD COUNT");
// 			}
// 			overCheckLinks();

	removeBadLinks();
	std::cout << "AFTER THE OPTIMIZATION REMOVE" << std::endl;
	overCheckLinks();

	return ret;

// 			checkRobotPoseNotMoved("after opti");
// 			cv::Mat d;
// 			createDescriptorNDT(d);

}

void AASS::acg::AutoCompleteGraph::clearPrior(){
//	int i = 0;

	AutoCompleteGraphBase<AutoCompleteGraphPriorSE2, g2o::VertexSE2Prior, g2o::EdgeSE2Prior_malcolm>::clearPrior();


// 			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
// 				_optimizable_graph.removeVertex(it, true);
// 			}
// 			std::cout <<"clearing the edges " << std::endl;
//	_edge_prior.clear();
	_edge_link.clear();
	_edge_interface_of_links.clear();

	//Making sure all edge prior were removed.
	auto idmapedges = _optimizable_graph.edges();
	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
		assert( dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite) == NULL );
	}
	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
	std::cout << "DONE removing " << std::endl;
}

void AASS::acg::AutoCompleteGraph::clearLinks(){
	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
	int i = 0;

	for(auto it = _edge_link.begin() ; it != _edge_link.end() ;){

		_optimizable_graph.removeEdge(*it);
		it = _edge_link.erase(it);

	}
// 			std::cout <<"Done final " << _nodes_prior.size() << " i " << i <<std::endl;
	assert(_edge_link.size() == 0);

// 			for(auto it = _edge_prior.begin() ; it != _edge_prior.end() ; ++it){
// 				_optimizable_graph.removeVertex(it, true);
// 			}
// 			std::cout <<"clearing the edges " << std::endl;

	//Making sure all edge prior were removed.
	auto idmapedges = _optimizable_graph.edges();
	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
// 				std::cout << "pointer " << dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite) << std::endl;
		assert( dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite) == NULL );
	}
	std::cout << "IMPORTANT size " << _optimizable_graph.vertices().size() << std::endl;
	std::cout << "DONE removing " << std::endl;
}
