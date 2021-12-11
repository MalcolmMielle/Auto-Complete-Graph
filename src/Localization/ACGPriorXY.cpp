#include "auto_complete_graph/Localization/ACGPriorXY.hpp"

g2o::EdgeXYPriorACG *AASS::acg::AutoCompleteGraphPriorXY::addEdge(const g2o::SE2 &se2, g2o::HyperGraph::Vertex *v1, g2o::HyperGraph::Vertex *v2)
{

	for (auto prior_edge : _edges)
	{
		if (v1 == prior_edge->vertices()[0] && v2 == prior_edge->vertices()[1])
		{
			throw std::runtime_error("Edge link already added");
		}
		else if (v1 == prior_edge->vertices()[1] && v2 == prior_edge->vertices()[0])
		{
			throw std::runtime_error("Edge link already added");
		}
	}

	//Get Eigen vector
	g2o::VertexXYPrior *v_ptr = dynamic_cast<g2o::VertexXYPrior *>(v1);
	g2o::VertexXYPrior *v_ptr2 = dynamic_cast<g2o::VertexXYPrior *>(v2);
	Eigen::Vector2d pose1 = v_ptr->estimate();
	Eigen::Vector2d pose2 = v_ptr2->estimate();

	Eigen::Vector2d eigenvec;
	eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
	double newnorm_old = (pose1 - pose2).norm();

	// 	double test_newnorm = newnorm_old / 2;
	//ATTENTION NOT A MAGIC NUMBER
	double newnorm = (newnorm_old * _priorNoise(0)) / 100;

	int test_tt = newnorm * 100;
	int test_ttt = newnorm_old * 100;

	assert(test_tt <= test_ttt);
	assert(newnorm >= 0);

	std::pair<double, double> eigenval(newnorm, _priorNoise(1));
	// 	std::pair<double, double> eigenval(_priorNoise(0), _priorNoise(1));

	Eigen::Matrix2d cov = getCovarianceSingleEigenVector(eigenvec, eigenval);

	cov = cov * 1000;
	cov = cov.array().round();
	cov = cov / 1000;

	//CHECK INVERTIBILITY OF THE MATRIX
	Eigen::Matrix2d information_prior = cov.inverse();

	information_prior = information_prior * 1000;
	information_prior = information_prior.array().round();
	information_prior = information_prior / 1000;

	assert(information_prior.isZero(1e-10) == false);
	assert(information_prior == information_prior.transpose());

	g2o::EdgeXYPriorACG *priorObservation = new g2o::EdgeXYPriorACG;
	priorObservation->vertices()[0] = v1;
	priorObservation->vertices()[1] = v2;
	Eigen::Vector3d se3_vec = se2.toVector();
	Eigen::Vector2d se_no_rotation = se3_vec.head(2);
	priorObservation->setMeasurement(se_no_rotation);
	priorObservation->setInformation(information_prior);
	priorObservation->setParameterId(0, _sensorOffset->id());
	_edges.insert(priorObservation);

	checkNoRepeatingPriorEdge();

	return priorObservation;
}

g2o::VertexXYPrior *AASS::acg::AutoCompleteGraphPriorXY::addPose(const g2o::SE2 &se2, const PriorAttr &priorAttr, int index)
{
	g2o::VertexXYPrior *priorlandmark = new g2o::VertexXYPrior();
	priorlandmark->setId(index);
	Eigen::Vector2d pose = se2.toVector().head(2);
	priorlandmark->setEstimate(pose);
	priorlandmark->priorattr = priorAttr;

	//Check that the node is not here in double
	for (auto node : _nodes)
	{
		assert(node->estimate() != priorlandmark->estimate());
	}

	_nodes.insert(priorlandmark);
	return priorlandmark;
}
g2o::VertexXYPrior *AASS::acg::AutoCompleteGraphPriorXY::addPose(const Eigen::Vector3d &lan, const PriorAttr &priorAttr, int index)
{
	g2o::SE2 se2(lan(0), lan(1), lan(2));
	return addPose(se2, priorAttr, index);
}
g2o::VertexXYPrior *AASS::acg::AutoCompleteGraphPriorXY::addPose(double x, double y, double theta, const AASS::acg::PriorAttr &priorAttr, int index)
{
	Eigen::Vector3d lan;
	lan << x, y, theta;
	return addPose(lan, priorAttr, index);
}

int AASS::acg::AutoCompleteGraphPriorXY::addPriorGraph(const PriorLoaderInterface::PriorGraph &graph, int first_index)
{

	std::pair<PriorLoaderInterface::PriorGraph::VertexIterator, PriorLoaderInterface::PriorGraph::VertexIterator> vp;
	std::deque<PriorLoaderInterface::PriorGraph::Vertex> vec_deque;
	std::vector<g2o::VertexXYPrior *> out_prior;

	assert(_nodes.size() == 0);

	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first)
	{
		auto v = *vp.first;
		g2o::VertexXYPrior *res = addPose(graph[v].getX(), graph[v].getY(), 0, graph[v], first_index);
		first_index++;
		vec_deque.push_back(v);
		out_prior.push_back(res);
	}

	assert(_edges.size() == 0);

	int count = 0;
	int self_link = 0;
	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first)
	{
		auto v = *vp.first;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Edge e;

		std::vector<PriorLoaderInterface::PriorGraph::Vertex> tmp_for_double_edges;

		for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph));
			 out_i != out_end; ++out_i)
		{
			e = *out_i;
			auto targ = boost::target(e, (graph));

			int idx = -1;
			for (size_t ii = count; ii < vec_deque.size(); ++ii)
			{
				if (targ == vec_deque[ii])
				{
					idx = ii;
				}
			}
			for (auto it_tmp_double = tmp_for_double_edges.begin(); it_tmp_double != tmp_for_double_edges.end(); ++it_tmp_double)
			{
				if (*it_tmp_double == targ)
				{
					//it's a double we cancel idx and add it to the self_lopp count
					idx = count;
				}
			}
			tmp_for_double_edges.push_back(targ);

			if (idx == -1)
			{
			}
			else if (idx == count)
			{
				self_link = self_link + 1;
			}
			else
			{

				assert(idx != count);

				g2o::VertexXYPrior *from = out_prior[count]; //<-Base
				g2o::VertexXYPrior *toward = out_prior[idx]; //<-Targ

				double x_diff = graph[targ].getX() - graph[v].getX();
				double y_diff = graph[targ].getY() - graph[v].getY();

				g2o::SE2 se2(x_diff, y_diff, 0);

				auto edge_out = addEdge(se2, from, toward);
			}
		}
		++count;
	}

	assert(_nodes.size() == graph.getNumVertices());
	//Self link / 2 because they are seen twice
	assert(_edges.size() == graph.getNumEdges() - (self_link / 2));

	std::cout << "After update graph prior" << std::endl;
	checkNoRepeatingPriorEdge();

	return first_index;
}

void AASS::acg::AutoCompleteGraphPriorXY::updatePriorEdgeCovariance()
{

	std::cout << "DO NOT USE " << std::endl;
	testNoNanInPrior("no dtat");
	assert(false);

	auto edges = getEdges();
	auto it = edges.begin();
	for (it; it != edges.end(); ++it)
	{
		g2o::VertexXYPrior *v_ptr = dynamic_cast<g2o::VertexXYPrior *>((*it)->vertices()[0]);
		if (v_ptr == NULL)
		{
			throw std::runtime_error("no");
		}
		g2o::VertexXYPrior *v_ptr2 = dynamic_cast<g2o::VertexXYPrior *>((*it)->vertices()[1]);
		if (v_ptr2 == NULL)
		{
			throw std::runtime_error("no2");
		}
		Eigen::Vector2d pose1 = v_ptr->estimate();
		Eigen::Vector2d pose2 = v_ptr2->estimate();

		double tre[3];
		(*it)->getMeasurementData(tre);

		Eigen::Vector2d length;
		length << tre[0], tre[1];

		Eigen::Vector2d eigenvec;
		eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);

		double newnorm = (pose1 - pose2).norm();
		double len_norm = length.norm();
		std::cout << "new norm " << newnorm << " because " << pose1 << " " << pose2 << " and lennorm " << len_norm << "because  " << length << std::endl;
		g2o::SE2 oldnormse2 = (*it)->interface.getOriginalValue();
		Eigen::Vector3d vecold = oldnormse2.toVector();
		double oldnorm = (vecold).norm();
		std::cout << "oldnorm" << oldnorm << std::endl;
		assert(oldnorm >= 0);

		//Using the diff so we cannot shrink it or stretch it easily.
		double diff_norm = std::abs(oldnorm - newnorm);
		std::cout << "Diff norm " << diff_norm << std::endl;
		assert(diff_norm >= 0);
		if (diff_norm > oldnorm)
		{
			diff_norm = oldnorm;
		}
		diff_norm = std::abs(diff_norm);
		std::cout << "Diff norm " << diff_norm << std::endl;
		assert(diff_norm >= 0);
		assert(diff_norm <= oldnorm);

		//Normalizes it
		double normalizer_own = 1 / oldnorm;

		//Between 0 and 1 between oldnorm / 2 and oldnorm
		//This is between 0 and oldnorm/2 because we work on the diff and not on the length ==> best length is 0 diff and worse will be half of oldnorm
		double min = 0;
		double max = (oldnorm / 2);
		double max_range = 1;
		double min_range = 0;

		double diff_norm_normalized = 1 - ((((max_range - min_range) * (diff_norm - min)) / (max - min)) + min_range);

		double new_cov = diff_norm_normalized;

		std::cout << "min " << min << " max " << max << " max_range " << max_range << " min_range " << min_range << " diff norm  " << diff_norm << " cov " << new_cov << std::endl;

		assert(new_cov <= 1);

		if (new_cov <= 0.001)
		{

			new_cov = 0.001;
		}

		//Scale it again.depending on user inputed value
		if (_use_user_prior_cov == true)
		{
			new_cov = new_cov * _priorNoise(0);
			assert(new_cov <= _priorNoise(0));
			assert(new_cov >= 0);
		}
		else
		{
			//Scale it again. depending on oldnorm/10
			new_cov = new_cov * (oldnorm / 10);
			assert(new_cov <= (oldnorm / 10));
			assert(new_cov >= 0);
		}

		std::pair<double, double> eigenval(new_cov, _priorNoise(1));

		Eigen::Matrix2d cov = getCovarianceSingleEigenVector(eigenvec, eigenval);

		Eigen::Matrix2d information_prior = cov.inverse();

		(*it)->setInformation(information_prior);
	}
}

void AASS::acg::AutoCompleteGraphPriorXY::testNoNanInPrior(const std::string &before) const
{

	auto it = getNodes().begin();
	for (it; it != getNodes().end(); ++it)
	{
		g2o::VertexXYPrior *v_ptr = dynamic_cast<g2o::VertexXYPrior *>((*it));
		if (v_ptr == NULL)
		{
			throw std::runtime_error("not good vertex type");
		}
		Eigen::Vector2d pose1 = v_ptr->estimate();
		assert(!std::isnan(pose1[0]));
		assert(!std::isnan(pose1[1]));
	}

	auto edges = getEdges();
	auto it_edge = edges.begin();
	for (it_edge; it_edge != edges.end(); ++it_edge)
	{
		g2o::VertexXYPrior *v_ptr = dynamic_cast<g2o::VertexXYPrior *>((*it_edge)->vertices()[0]);
		if (v_ptr == NULL)
		{
			throw std::runtime_error("no");
		}
		g2o::VertexXYPrior *v_ptr2 = dynamic_cast<g2o::VertexXYPrior *>((*it_edge)->vertices()[1]);
		if (v_ptr2 == NULL)
		{
			throw std::runtime_error("no2");
		}
		Eigen::Vector2d pose1 = v_ptr->estimate();
		Eigen::Vector2d pose2 = v_ptr2->estimate();

		assert(!std::isnan(pose1[0]));
		assert(!std::isnan(pose1[1]));

		assert(!std::isnan(pose2[0]));
		assert(!std::isnan(pose2[1]));
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AASS::acg::AutoCompleteGraphPriorXY::toPointCloud(double resolution, double z_elevation, double varz) const
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
	int nb_points = 0;
	auto edges = getEdges();

	for (auto it = edges.begin(); it != edges.end(); ++it)
	{

		std::vector<Eigen::Vector3d> points;

		for (auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end(); ++ite2)
		{
			g2o::VertexXYPrior *ptr = dynamic_cast<g2o::VertexXYPrior *>((*ite2));
			auto vertex_t = ptr->estimate();
			Eigen::Vector3d vertex;
			vertex << vertex_t(0), vertex_t(1), z_elevation;

			points.push_back(vertex);
		}
		assert(points.size() == 2);

		Eigen::Vector3d slope = points[1] - points[0];

		slope = slope / slope.norm();
		slope = slope * resolution;

		Eigen::Vector3d point = points[0];

		pcl::PointXYZ pcl_point;
		pcl_point.x = point[0];
		pcl_point.y = point[1];
		pcl_point.z = point[2];
		pcl_pc->push_back(pcl_point);
		nb_points++;

		while ((points[1] - point).norm() >= resolution)
		{
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
	for (int i = 0; i < pcl_pc->points.size(); i++)
	{
		pt = pcl_pc->points[i];
		pt.z += varz * ((double)rand()) / (double)INT_MAX;
		pcl_pc_noise->points.push_back(pt);
	}

	pcl_pc_noise->width = nb_points;
	pcl_pc_noise->height = 1;
	pcl_pc_noise->is_dense = false;

	return pcl_pc_noise;
}