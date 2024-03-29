#include "auto_complete_graph/ACGPriorSE2.hpp"

g2o::EdgeSE2Prior_malcolm* AASS::acg::AutoCompleteGraphPriorSE2::addEdge(
    const g2o::SE2& se2,
    g2o::HyperGraph::Vertex* v1,
    g2o::HyperGraph::Vertex* v2) {
    for (auto prior_edge : _edges) {
        if (v1 == prior_edge->vertices()[0] &&
            v2 == prior_edge->vertices()[1]) {
            throw std::runtime_error("Edge link already added");
        } else if (v1 == prior_edge->vertices()[1] &&
                   v2 == prior_edge->vertices()[0]) {
            throw std::runtime_error("Edge link already added");
        }
    }

    // Get Eigen vector
    g2o::VertexSE2Prior* v_ptr = dynamic_cast<g2o::VertexSE2Prior*>(v1);
    g2o::VertexSE2Prior* v_ptr2 = dynamic_cast<g2o::VertexSE2Prior*>(v2);
    Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
    Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();

    // 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt)
    // << std::endl; 			std::cout << "Poses 2 " << std::endl <<
    // pose2.format(cleanFmt)
    // << std::endl;

    Eigen::Vector2d eigenvec;
    eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
    // 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt)
    // << std::endl;
    double newnorm_old = (pose1 - pose2).norm();

    // 	double test_newnorm = newnorm_old / 2;
    // ATTENTION NOT A MAGIC NUMBER
    double newnorm = (newnorm_old * _priorNoise(0)) / 100;

    int test_tt = newnorm * 100;
    int test_ttt = newnorm_old * 100;
    // 	std::cout << newnorm << " " << test_ttt << " test " << test_tt << " base
    // " << _priorNoise(0) << " "<< _priorNoise(1) << std::endl;

    assert(test_tt <= test_ttt);
    assert(newnorm >= 0);

    std::pair<double, double> eigenval(newnorm, _priorNoise(1));
    // 	std::pair<double, double> eigenval(_priorNoise(0), _priorNoise(1));

    Eigen::Matrix2d cov = getCovarianceSingleEigenVector(eigenvec, eigenval);

    // 			std::cout << "Covariance prior " << std::endl <<
    // cov.format(cleanFmt)
    // << std::endl;

    Eigen::Matrix3d covariance_prior;
    covariance_prior.fill(0.);
    covariance_prior(0, 0) = cov(0, 0);
    covariance_prior(0, 1) = cov(0, 1);
    covariance_prior(1, 0) = cov(1, 0);
    covariance_prior(1, 1) = cov(1, 1);
    // 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is
    // more than 4PI
    covariance_prior(2, 2) = _prior_rot * _prior_rot;
    Eigen::Matrix3d information_prior = covariance_prior.inverse();
    // 			std::cout << "Information prior " << std::endl <<
    // cov.format(cleanFmt)
    // << std::endl;

    assert(information_prior.isZero(1e-10) == false);

    g2o::EdgeSE2Prior_malcolm* priorObservation = new g2o::EdgeSE2Prior_malcolm;
    priorObservation->vertices()[0] = v1;
    priorObservation->vertices()[1] = v2;
    priorObservation->setMeasurement(se2);
    priorObservation->setInformation(information_prior);
    priorObservation->setParameterId(0, _sensorOffset->id());

    // 	priorObservation->interface.setAge(_age_start_value);
    // 	priorObservation->interface.setOriginalValue(se2);

    //	_optimizable_graph.addEdge(priorObservation);

    // 	EdgePriorAndInitialValue epiv(priorObservation, se2);
    _edges.insert(priorObservation);

    std::cout << "After adding an edge" << std::endl;
    checkNoRepeatingPriorEdge();

    return priorObservation;
}

g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraphPriorSE2::addPose(
    const g2o::SE2& se2,
    const PriorAttr& priorAttr,
    int index) {
    g2o::VertexSE2Prior* priorlandmark = new g2o::VertexSE2Prior();
    priorlandmark->setId(index);
    priorlandmark->setEstimate(se2);
    priorlandmark->priorattr = priorAttr;
    _nodes.insert(priorlandmark);
    return priorlandmark;
}
g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraphPriorSE2::addPose(
    const Eigen::Vector3d& lan,
    const PriorAttr& priorAttr,
    int index) {
    g2o::SE2 se2(lan(0), lan(1), lan(2));
    return addPose(se2, priorAttr, index);
}
g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraphPriorSE2::addPose(
    double x,
    double y,
    double theta,
    const AASS::acg::PriorAttr& priorAttr,
    int index) {
    Eigen::Vector3d lan;
    lan << x, y, theta;
    return addPose(lan, priorAttr, index);
}

int AASS::acg::AutoCompleteGraphPriorSE2::addPriorGraph(
    const PriorLoaderInterface::PriorGraph& graph,
    int first_index) {
    std::pair<PriorLoaderInterface::PriorGraph::VertexIterator,
              PriorLoaderInterface::PriorGraph::VertexIterator>
        vp;
    std::deque<PriorLoaderInterface::PriorGraph::Vertex> vec_deque;
    std::vector<g2o::VertexSE2Prior*> out_prior;

    //	std::cout << "NOOOOOOW" << _optimizable_graph.vertices().size() <<
    // std::endl << std::endl;

    assert(_nodes.size() == 0);

    for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
        // 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
        // AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
        auto v = *vp.first;
        // ATTENTION Magic number

        // 		std::cout << "Prior Landmark : " << graph[v].getX() << "
        // "
        // << graph[v].getY() << std::endl;

        g2o::VertexSE2Prior* res =
            addPose(graph[v].getX(), graph[v].getY(), 0, graph[v], first_index);
        first_index++;
        vec_deque.push_back(v);
        out_prior.push_back(res);
        // 		_nodes.push_back(res);
    }

    assert(_edges.size() == 0);

    int count = 0;
    int self_link = 0;
    for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
        // 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
        // AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
        auto v = *vp.first;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::EdgeIterator
            out_i,
            out_end;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::Edge e;

        std::vector<PriorLoaderInterface::PriorGraph::Vertex>
            tmp_for_double_edges;

        for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph));
             out_i != out_end; ++out_i) {
            e = *out_i;
            // 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
            // AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e,
            // (graph));
            auto targ = boost::target(e, (graph));

            // Avoiding double edge to the same vertex target but that is not a
            // self loop!

            int idx = -1;
            for (size_t ii = count; ii < vec_deque.size(); ++ii) {
                if (targ == vec_deque[ii]) {
                    idx = ii;
                }
            }
            for (auto it_tmp_double = tmp_for_double_edges.begin();
                 it_tmp_double != tmp_for_double_edges.end(); ++it_tmp_double) {
                if (*it_tmp_double == targ) {
                    // it's a double we cancel idx and add it to the self_lopp
                    // count
                    idx = count;
                }
            }
            tmp_for_double_edges.push_back(targ);

            if (idx == -1) {
                // SKIP
                //  				throw std::runtime_error("Didn't find
                //  the vertex target :(");
            } else if (idx == count) {
                self_link = self_link + 1;
            } else {
                assert(idx != count);

                g2o::VertexSE2Prior* from = out_prior[count];  //<-Base
                g2o::VertexSE2Prior* toward = out_prior[idx];  //<-Targ

                double x_diff = graph[targ].getX() - graph[v].getX();
                double y_diff = graph[targ].getY() - graph[v].getY();

                // 				std::cout << "Index" << idx << "
                // "
                // <<count << std::endl;

                // 				std::cout << "Because : " <<
                // graph[targ].getX() << " - " << graph[v].getX() << " and " <<
                // graph[targ].getY() << " - " << graph[v].getY() << std::endl;

                // 				std::cout << "diff: " <<x_diff << " "
                // << y_diff << std::endl;

                g2o::SE2 se2(x_diff, y_diff, 0);

                // 				std::cout << "SE2 pushed in edge \n"
                // << se2.toVector() << std::endl;

                auto edge_out = addEdge(se2, from, toward);
                // 				_edges.push_back(edge_out);
            }
        }
        ++count;
    }

    // 	std::cout << _edges.size() << " == " << graph.getNumEdges() << " - " <<
    // self_link / 2 << std::endl; 	std::cout << _nodes.size() << " == " <<
    // graph.getNumVertices() << std::endl;

    assert(_nodes.size() == graph.getNumVertices());
    // Self link / 2 because they are seen twice
    assert(_edges.size() == graph.getNumEdges() - (self_link / 2));

    std::cout << "After update graph prior" << std::endl;
    checkNoRepeatingPriorEdge();

    return first_index;
}

void AASS::acg::AutoCompleteGraphPriorSE2::updatePriorEdgeCovariance() {
    std::cout << "DO NOT USE " << std::endl;
    testNoNanInPrior("no dtat");
    assert(false);

    auto edges = getEdges();
    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        g2o::VertexSE2Prior* v_ptr =
            dynamic_cast<g2o::VertexSE2Prior*>((*it)->vertices()[0]);
        if (v_ptr == NULL) {
            throw std::runtime_error("no");
        }
        g2o::VertexSE2Prior* v_ptr2 =
            dynamic_cast<g2o::VertexSE2Prior*>((*it)->vertices()[1]);
        if (v_ptr2 == NULL) {
            throw std::runtime_error("no2");
        }
        Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
        Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();

        // 			std::cout << "Poses 1 " << std::endl <<
        // pose1.format(cleanFmt) << std::endl; 			std::cout <<
        // "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;

        double tre[3];
        (*it)->getMeasurementData(tre);

        Eigen::Vector2d length;
        length << tre[0], tre[1];

        Eigen::Vector2d eigenvec;
        eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);

        double newnorm = (pose1 - pose2).norm();
        double len_norm = length.norm();
        std::cout << "new norm " << newnorm << " because " << pose1 << " "
                  << pose2 << " and lennorm " << len_norm << "because  "
                  << length << std::endl;
        g2o::SE2 oldnormse2 = (*it)->interface.getOriginalValue();
        Eigen::Vector3d vecold = oldnormse2.toVector();
        double oldnorm = (vecold).norm();
        std::cout << "oldnorm" << oldnorm << std::endl;
        assert(oldnorm >= 0);

        // Using the diff so we cannot shrink it or stretch it easily.
        double diff_norm = std::abs(oldnorm - newnorm);
        std::cout << "Diff norm " << diff_norm << std::endl;
        assert(diff_norm >= 0);
        if (diff_norm > oldnorm) {
            diff_norm = oldnorm;
        }
        diff_norm = std::abs(diff_norm);
        std::cout << "Diff norm " << diff_norm << std::endl;
        assert(diff_norm >= 0);
        assert(diff_norm <= oldnorm);

        // Normalizes it
        double normalizer_own = 1 / oldnorm;

        // Between 0 and 1 between 0 and oldnorm
        //  		double diff_norm_normalized = 1 - (diff_norm *
        //  normalizer_own); 		double min = 0; 		double max =
        //  oldnorm; 		double
        //  max_range = 1; 		double min_range = 0;

        // Between 0 and 1 between oldnorm / 2 and oldnorm
        // This is between 0 and oldnorm/2 because we work on the diff and not
        // on the length ==> best length is 0 diff and worse will be half of
        // oldnorm
        double min = 0;
        double max = (oldnorm / 2);
        double max_range = 1;
        double min_range = 0;

        double diff_norm_normalized =
            1 - ((((max_range - min_range) * (diff_norm - min)) / (max - min)) +
                 min_range);

        double new_cov = diff_norm_normalized;

        std::cout << "min " << min << " max " << max << " max_range "
                  << max_range << " min_range " << min_range << " diff norm  "
                  << diff_norm << " cov " << new_cov << std::endl;

        assert(new_cov <= 1);

        // Sometime the optimization in one turn goes under the limit so need to
        // correct those cases ;)
        //  		assert(new_cov >= 0);

        if (new_cov <= 0.001) {
            // Apparently the vaqlue in the edge does not get changed so it's
            // useless modifying it ? See :
            //  			double tre[3];
            //  			(*it)->getMeasurementData(tre);
            //  			Eigen::Vector2d length; length <<
            //  tre[0], tre[1] ;

            new_cov = 0.001;
        }

        // Scale it again.depending on user inputed value
        if (_use_user_prior_cov == true) {
            new_cov = new_cov * _priorNoise(0);
            assert(new_cov <= _priorNoise(0));
            assert(new_cov >= 0);
        } else {
            // Scale it again. depending on oldnorm/10
            new_cov = new_cov * (oldnorm / 10);
            assert(new_cov <= (oldnorm / 10));
            assert(new_cov >= 0);
        }

        // 				std::cout << "EigenVec " << std::endl <<
        // eigenvec.format(cleanFmt) << std::endl;
        std::pair<double, double> eigenval(new_cov, _priorNoise(1));

        std::cout << "Eigen vec " << eigenvec << " egenval " << eigenval.first
                  << " " << eigenval.second << std::endl;

        Eigen::Matrix2d cov =
            getCovarianceSingleEigenVector(eigenvec, eigenval);

        // 			std::cout << "Covariance prior " << std::endl <<
        // cov.format(cleanFmt) << std::endl;

        Eigen::Matrix3d covariance_prior;
        covariance_prior.fill(0.);
        covariance_prior(0, 0) = cov(0, 0);
        covariance_prior(0, 1) = cov(0, 1);
        covariance_prior(1, 0) = cov(1, 0);
        covariance_prior(1, 1) = cov(1, 1);
        // 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior
        // landmark is more than 4PI
        covariance_prior(2, 2) = _prior_rot * _prior_rot;
        Eigen::Matrix3d information_prior = covariance_prior.inverse();

        std::cout << "ALL INFO \n"
                  << information_prior << "\n new cov " << new_cov
                  << " cov mat " << cov << std::endl;

        (*it)->setInformation(information_prior);
    }
}

void AASS::acg::AutoCompleteGraphPriorSE2::testNoNanInPrior(
    const std::string& before) const {
    std::cout << "Test No nan in prior after " << before << std::endl;
    auto it = getNodes().begin();
    for (it; it != getNodes().end(); ++it) {
        g2o::VertexSE2Prior* v_ptr = dynamic_cast<g2o::VertexSE2Prior*>((*it));
        if (v_ptr == NULL) {
            throw std::runtime_error("not good vertex type");
        }
        Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
        assert(!std::isnan(pose1[0]));
        assert(!std::isnan(pose1[1]));
        assert(!std::isnan(pose1[2]));
    }

    std::cout << "Testing the edges now" << std::endl;

    auto edges = getEdges();
    auto it_edge = edges.begin();
    for (it_edge; it_edge != edges.end(); ++it_edge) {
        g2o::VertexSE2Prior* v_ptr =
            dynamic_cast<g2o::VertexSE2Prior*>((*it_edge)->vertices()[0]);
        if (v_ptr == NULL) {
            throw std::runtime_error("no");
        }
        g2o::VertexSE2Prior* v_ptr2 =
            dynamic_cast<g2o::VertexSE2Prior*>((*it_edge)->vertices()[1]);
        if (v_ptr2 == NULL) {
            throw std::runtime_error("no2");
        }
        Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
        Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();

        assert(!std::isnan(pose1[0]));
        assert(!std::isnan(pose1[1]));
        assert(!std::isnan(pose1[2]));

        assert(!std::isnan(pose2[0]));
        assert(!std::isnan(pose2[1]));
        assert(!std::isnan(pose2[2]));
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
AASS::acg::AutoCompleteGraphPriorSE2::toPointCloud(double resolution,
                                                   double z_elevation,
                                                   double varz) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(
        new pcl::PointCloud<pcl::PointXYZ>);
    int nb_points = 0;
    auto edges = getEdges();
    std::cout << "Converting edges : " << edges.size() << std::endl;

    for (auto it = edges.begin(); it != edges.end(); ++it) {
        std::vector<Eigen::Vector3d> points;

        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            //			geometry_msgs::Point p;
            g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
            auto vertex = ptr->estimate().toVector();
            vertex[2] = z_elevation;
            // Getting the translation out of the transform :
            // https://en.wikipedia.org/wiki/Transformation_matrix
            //				Eigen::Vector2d veve; veve << vertex(0),
            // vertex(1);
            //                  std::cout << "Pushing " << veve << std::endl;
            points.push_back(vertex);
        }
        assert(points.size() == 2);

        Eigen::Vector3d slope = points[1] - points[0];
        //			std::cout << "from " << points[0] << " tot " <<
        // points[1] << " slope " << slope << std::endl;

        slope = slope / slope.norm();
        //			std::cout << "Then slope " << slope <<
        // std::endl;
        slope = slope * resolution;
        //			std::cout << "Final slope " << slope <<
        // std::endl;

        //			int wait;
        //			std::cin>>wait;

        Eigen::Vector3d point = points[0];

        pcl::PointXYZ pcl_point;
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        pcl_pc->push_back(pcl_point);
        nb_points++;

        while ((points[1] - point).norm() >= resolution) {
            //				std::cout << "Adding point " << pcl_point.x << " "
            //<< pcl_point.y << " " << pcl_point.z << " nbpt " << nb_points << "
            // "
            //<< slope[0] << " " << slope[1] << " " << slope[2] << std::endl;
            point = point + slope;
            pcl_point.x = point[0];
            pcl_point.y = point[1];
            pcl_point.z = point[2];
            pcl_pc->push_back(pcl_point);
            nb_points++;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_noise(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt;
    // add some variance on z
    for (int i = 0; i < pcl_pc->points.size(); i++) {
        pt = pcl_pc->points[i];
        pt.z += varz * ((double)rand()) / (double)INT_MAX;
        pcl_pc_noise->points.push_back(pt);
    }

    std::cout << "Adding data " << nb_points << std::endl;

    pcl_pc_noise->width = nb_points;
    pcl_pc_noise->height = 1;
    pcl_pc_noise->is_dense = false;

    return pcl_pc_noise;
}