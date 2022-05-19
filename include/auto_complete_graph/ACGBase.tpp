template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexSE2RobotPose*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addRobotPose(
    const g2o::SE2& se2,
    const Eigen::Affine3d& affine,
    const std::shared_ptr<perception_oru::NDTMap>& map) {
    ROS_DEBUG("Adding the robot pose ");
    g2o::VertexSE2RobotPose* robot = new g2o::VertexSE2RobotPose();

    robot->setEstimate(se2);
    robot->setId(new_id_);
    ++new_id_;

    robot->setMap(map);
    robot->setPose(affine);
    robot->initial_transfo = robot->estimate();

    _optimizable_graph.addVertex(robot);

    _nodes_ndt.push_back(robot);
    return robot;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexSE2RobotPose*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addRobotPose(
    const Eigen::Vector3d& rob,
    const Eigen::Affine3d& affine,
    const std::shared_ptr<perception_oru::NDTMap>& map) {
    g2o::SE2 se2(rob(0), rob(1), rob(2));
    return addRobotPose(se2, affine, map);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexSE2RobotPose*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addRobotPose(
    double x,
    double y,
    double theta,
    const Eigen::Affine3d& affine,
    const std::shared_ptr<perception_oru::NDTMap>& map) {
    Eigen::Vector3d robot1;
    robot1 << x, y, theta;
    return addRobotPose(robot1, affine, map);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexLandmarkNDT* AASS::acg::
    AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkPose(
        const g2o::Vector2& estimate,
        const cv::Point2f& position,
        int strength) {
    g2o::VertexLandmarkNDT* landmark = new g2o::VertexLandmarkNDT();
    landmark->setId(new_id_);
    ++new_id_;
    landmark->setEstimate(estimate);
    landmark->position = position;
    _optimizable_graph.addVertex(landmark);
    _nodes_landmark.push_back(landmark);
    return landmark;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::VertexLandmarkNDT* AASS::acg::
    AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addLandmarkPose(
        double x,
        double y,
        const cv::Point2f& position,
        int strength) {
    g2o::Vector2 lan;
    lan << x, y;
    return addLandmarkPose(lan, position, strength);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline VertexPrior* AASS::acg::
    AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorLandmarkPose(
        const g2o::SE2& se2,
        const PriorAttr& priorAttr) {
    auto landmark = _prior->addPose(se2, priorAttr, new_id_);
    ++new_id_;
    _optimizable_graph.addVertex(landmark);
    return landmark;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline VertexPrior* AASS::acg::
    AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorLandmarkPose(
        const Eigen::Vector3d& lan,
        const PriorAttr& priorAttr) {
    g2o::SE2 se2(lan(0), lan(1), lan(2));
    return addPriorLandmarkPose(se2, priorAttr);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline VertexPrior* AASS::acg::
    AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorLandmarkPose(
        double x,
        double y,
        double theta,
        const AASS::acg::PriorAttr& priorAttr) {
    Eigen::Vector3d lan;
    lan << x, y, theta;
    return addPriorLandmarkPose(lan, priorAttr);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(
    const g2o::SE2& se2,
    g2o::HyperGraph::Vertex* v1,
    g2o::HyperGraph::Vertex* v2,
    const Eigen::Matrix3d& information_tmp) {
    for (auto odom_edge : _edge_odometry) {
        if (v1 == odom_edge->vertices()[0] && v2 == odom_edge->vertices()[1]) {
            throw std::runtime_error("Edge odometry already added");
        } else if (v1 == odom_edge->vertices()[1] &&
                   v2 == odom_edge->vertices()[0]) {
            throw std::runtime_error("Edge odometry already added");
        }
    }

    Eigen::Matrix3d information;
    if (_use_user_robot_pose_cov == true) {
        Eigen::Matrix3d covariance_robot;
        covariance_robot.fill(0.);
        covariance_robot(0, 0) = _transNoise[0] * _transNoise[0];
        covariance_robot(1, 1) = _transNoise[1] * _transNoise[1];
        // landmark is more than 4PI
        covariance_robot(2, 2) = _rotNoise * _rotNoise;
        information = covariance_robot.inverse();

        throw std::runtime_error("Do not use user inputed values in odometry");
    } else {
        information = information_tmp;
    }

    assert(information.isZero(1e-10) == false);

    g2o::EdgeOdometry_malcolm* odometry = new g2o::EdgeOdometry_malcolm;
    odometry->vertices()[0] = v1;
    odometry->vertices()[1] = v2;
    odometry->setMeasurement(se2);
    odometry->setInformation(information);
    _optimizable_graph.addEdge(odometry);
    _edge_odometry.push_back(odometry);

    assert(verifyInformationMatrices(true) == true);

    return odometry;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(
    const g2o::SE2& observ,
    int from_id,
    int toward_id,
    const Eigen::Matrix3d& information) {
    g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
    g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
    return addOdometry(observ, from_ptr, toward_ptr, information);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(
    double x,
    double y,
    double theta,
    int from_id,
    int toward_id,
    const Eigen::Matrix3d& information) {
    g2o::SE2 se2(x, y, theta);
    return addOdometry(se2, from_id, toward_id, information);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(
    const g2o::SE2& se2,
    g2o::HyperGraph::Vertex* v1,
    g2o::HyperGraph::Vertex* v2) {
    Eigen::Matrix3d covariance;

    covariance.fill(0.);
    covariance(0, 0) = _transNoise[0] * _transNoise[0];
    covariance(1, 1) = _transNoise[1] * _transNoise[1];
    covariance(2, 2) = _rotNoise * _rotNoise;
    Eigen::Matrix3d information = covariance.inverse();

    throw std::runtime_error("Do not use user inputed values in odometry");

    return addOdometry(se2, v1, v2, information);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(
    const g2o::SE2& observ,
    int from_id,
    int toward_id) {
    g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
    g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
    return addOdometry(observ, from_ptr, toward_ptr);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeOdometry_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addOdometry(
    double x,
    double y,
    double theta,
    int from_id,
    int toward_id) {
    g2o::SE2 se2(x, y, theta);
    return addOdometry(se2, from_id, toward_id);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeLandmark_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    addLandmarkObservation(const g2o::Vector2& pos,
                           g2o::HyperGraph::Vertex* v1,
                           g2o::HyperGraph::Vertex* v2,
                           const Eigen::Matrix2d& covariance_landmark) {
    Eigen::Matrix2d information_landmark = covariance_landmark.inverse();

    g2o::EdgeLandmark_malcolm* landmarkObservation =
        new g2o::EdgeLandmark_malcolm;
    landmarkObservation->vertices()[0] = v1;
    landmarkObservation->vertices()[1] = v2;
    landmarkObservation->setMeasurement(pos);

    assert(information_landmark.isZero(1e-10) == false);

    landmarkObservation->setInformation(information_landmark);
    landmarkObservation->setParameterId(0, _sensorOffset->id());

    _optimizable_graph.addEdge(landmarkObservation);
    _edge_landmark.push_back(landmarkObservation);

    assert(verifyInformationMatrices(true) == true);

    return landmarkObservation;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeLandmark_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    addLandmarkObservation(const g2o::Vector2& pos,
                           g2o::HyperGraph::Vertex* v1,
                           g2o::HyperGraph::Vertex* v2) {
    Eigen::Matrix2d covariance_landmark;
    covariance_landmark.fill(0.);
    covariance_landmark(0, 0) = _landmarkNoise[0] * _landmarkNoise[0];
    covariance_landmark(1, 1) = _landmarkNoise[1] * _landmarkNoise[1];
    return addLandmarkObservation(pos, v1, v2, covariance_landmark);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline g2o::EdgeLandmark_malcolm*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    addLandmarkObservation(const g2o::Vector2& pos,
                           int from_id,
                           int toward_id) {
    g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
    g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
    return addLandmarkObservation(pos, from_ptr, toward_ptr);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline EdgePrior*
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addEdgePrior(
    const g2o::SE2& se2,
    g2o::HyperGraph::Vertex* v1,
    g2o::HyperGraph::Vertex* v2) {
    auto priorObservation = _prior->addEdge(se2, v1, v2);
    _optimizable_graph.addEdge(priorObservation);

    assert(verifyInformationMatrices(true) == true);

    return priorObservation;
}

// FUNCTION TO REMOVE A VERTEX
// TODO : remove all link edges from list
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::removeVertex(
    g2o::HyperGraph::Vertex* v1) {
    // Prior
    auto ptr = _prior->removeVertex(v1);
    if (ptr != NULL) {
        _optimizable_graph.removeVertex(ptr);
    }

    g2o::VertexSE2* ptr_se2 = dynamic_cast<g2o::VertexSE2*>(v1);
    g2o::VertexLandmarkNDT* ptr_se3 = dynamic_cast<g2o::VertexLandmarkNDT*>(v1);

    // Robot node
    if (ptr_se2 != NULL) {
        int index = findRobotNode(v1);
        assert(index != -1);
        auto which = _nodes_ndt.begin() + index;
        _nodes_ndt.erase(which);
    }
    // Landmark Node
    else if (ptr_se3 != NULL) {
        int index = findLandmarkNode(v1);
        assert(index != -1);
        std::vector<g2o::VertexLandmarkNDT*>::iterator which =
            _nodes_landmark.begin() + index;
        _nodes_landmark.erase(which);
    } else {
        throw std::runtime_error("Vertex type not found in list");
    }
    _optimizable_graph.removeVertex(v1, false);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline int
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::findRobotNode(
    g2o::HyperGraph::Vertex* v) {
    int pos = 0;
    auto it = _nodes_ndt.begin();
    for (it; it != _nodes_ndt.end(); ++it) {
        if (*it == v) {
            return pos;
        }
        ++pos;
    }
    return -1;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline int AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    findLandmarkNode(g2o::HyperGraph::Vertex* v) {
    int pos = 0;
    auto it = _nodes_landmark.begin();
    for (it; it != _nodes_landmark.end(); ++it) {
        if (*it == v) {
            return pos;
        }
        ++pos;
    }
    return -1;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::addPriorGraph(
    const PriorLoaderInterface::PriorGraph& graph) {
    new_id_ = _prior->addPriorGraph(graph, new_id_);

    for (auto node : _prior->getNodes()) {
        _optimizable_graph.addVertex(node);
    }
    for (auto edge : _prior->getEdges()) {
        _optimizable_graph.addEdge(edge);
    }
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline Eigen::Vector3d
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    getLastTransformation() {
    Eigen::Vector3d diff_vec;
    diff_vec << 0, 0, 0;
    //**************** Calculate the previous transformations if there as
    // already been something added *** //

    // 	if(_previous_number_of_node_in_ndtgraph != 0){
    if (_nodes_ndt.size() != 0) {
        auto node = _nodes_ndt[_nodes_ndt.size() - 1];
        auto original3d = _nodes_ndt[_nodes_ndt.size() - 1]->getPose();

        /*******' Using Vec********/
        Eigen::Isometry2d original2d_aff = Affine3d2Isometry2d(original3d);
        auto shift_vec = node->estimate().toVector();
        g2o::SE2 se2_tmptmp(original2d_aff);
        auto original3d_vec = se2_tmptmp.toVector();
        diff_vec = original3d_vec - shift_vec;
    }

    return diff_vec;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    getAllCornersNDTTranslatedToGlobalAndRobotFrame(
        const std::shared_ptr<perception_oru::NDTMap>& map,
        g2o::VertexSE2RobotPose* robot_ptr,
        const g2o::SE2& robot_pos,
        std::vector<AASS::acg::NDTCornerGraphElement>& corners_end) {
    perception_oru::ndt_feature_finder::NDTCorner cornersExtractor;
    ROS_DEBUG_STREAM("hopidy");
    auto ret_export = cornersExtractor.getAllCorners(*map);
    ROS_DEBUG_STREAM("gotall corner");
    ROS_DEBUG_STREAM("got all accurate corners");

    auto it = ret_export.begin();

    ROS_DEBUG_STREAM("Found " << ret_export.size() << " corners ");
    // Find all the observations :

    //**************** HACK: translate the corners now : **************//

    for (it; it != ret_export.end(); ++it) {
        ROS_DEBUG_STREAM("Corner size " << it->getOrientations().size());
        // Limited to corners that possess an orientation.
        if (it->getOrientations().size() > 0) {
            Eigen::Vector3d vec;
            ROS_DEBUG_STREAM("Corner size ");
            vec << it->getMeanOpenCV().x, it->getMeanOpenCV().y,
                it->getOrientations()[0];

            ROS_DEBUG_STREAM("Corner size ");
            cv::Point2f p_out;
            Eigen::Vector3d landmark_robotframe;
            translateFromRobotFrameToGlobalFrame(vec, robot_pos,
                                                 landmark_robotframe);

            ROS_DEBUG_STREAM("Corner size ");
            p_out.x = landmark_robotframe(0);
            p_out.y = landmark_robotframe(1);

            Eigen::Vector2d observation;
            observation << vec(0), vec(1);
            std::vector<double> orientations;
            std::vector<double> angles_width;

            ROS_DEBUG_STREAM("Corner size ");
            double angle_landmark = vec(2);

            ROS_DEBUG_STREAM("Corner size ");
            for (auto it_orientation = it->getOrientations().begin();
                 it_orientation != it->getOrientations().end();
                 ++it_orientation) {
                ROS_DEBUG_STREAM("Pushing back orientation");
                orientations.push_back((*it_orientation));
            }
            for (auto it_angles = it->getAngles().begin();
                 it_angles != it->getAngles().end(); ++it_angles) {
                ROS_DEBUG_STREAM("Pushing back angle");
                angles_width.push_back((*it_angles));
            }

            ROS_DEBUG_STREAM("Position node " << robot_pos.toVector());
            ROS_DEBUG_STREAM(" vec " << vec);

            ROS_DEBUG_STREAM("NEW POINT : " << p_out);

            NDTCornerGraphElement cor(p_out, *it);
            cor.addAllObserv(robot_ptr, observation);
            cor.cells1 = it->getCells1();
            cor.cells2 = it->getCells2();
            corners_end.push_back(cor);
        }
    }
}

// TODO: refactor this!
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    extractCornerNDTMap(const std::shared_ptr<perception_oru::NDTMap>& map,
                        g2o::VertexSE2RobotPose* robot_ptr,
                        const g2o::SE2& robot_pos) {
    // HACK For now : we translate the Corner extracted and not the ndt-maps
    auto cells = map->getAllCellsShared();
    std::cout << "got all cell shared " << cells.size() << std::endl;
    double x2, y2, z2;
    map->getCellSizeInMeters(x2, y2, z2);
    std::cout << "got all cell sized" << std::endl;
    double cell_size = x2;

    std::vector<AASS::acg::NDTCornerGraphElement> corners_end;
    getAllCornersNDTTranslatedToGlobalAndRobotFrame(map, robot_ptr, robot_pos,
                                                    corners_end);

    /***************** ADD THE CORNERS INTO THE GRAPH***********************/

    for (size_t i = 0; i < corners_end.size(); ++i) {
        bool seen = false;
        g2o::VertexLandmarkNDT* ptr_landmark_seen = NULL;
        for (size_t j = 0; j < _nodes_landmark.size(); ++j) {
            g2o::Vector2 landmark = _nodes_landmark[j]->estimate();
            cv::Point2f point_land(landmark(0), landmark(1));

            double res =
                cv::norm(point_land - corners_end[i].position_in_robot_frame);

            ROS_DEBUG_STREAM("res : ");

            // If we found the landmark, we save the data
            if (res < cell_size * 2) {
                seen = true;
                ptr_landmark_seen = _nodes_landmark[j];
            }
        }
        if (seen == false) {
            ROS_DEBUG_STREAM("New point " << i);
            g2o::Vector2 position_globalframe;
            position_globalframe << corners_end[i].position_in_robot_frame.x,
                corners_end[i].position_in_robot_frame.y;

            cv::Point2f p_observation;
            ROS_DEBUG_STREAM("New point " << i);
            p_observation.x = corners_end[i].getObservations()(0);
            ROS_DEBUG_STREAM("New point " << i);
            p_observation.y = corners_end[i].getObservations()(1);
            ROS_DEBUG_STREAM("New point " << i);
            g2o::VertexLandmarkNDT* ptr =
                addLandmarkPose(position_globalframe, p_observation, 1);
            ptr->addAnglesOrientations(corners_end[i].getAngles(),
                                       corners_end[i].getOrientations());
            ptr->first_seen_from = robot_ptr;

            // TESTING to visualise which cells gave the corner
            ptr->cells_that_gave_it_1 = corners_end[i].cells1;
            ptr->cells_that_gave_it_2 = corners_end[i].cells2;
            ptr->robotpose_seen_from = robot_pos;
            // END OF TEST

            addLandmarkObservation(corners_end[i].getObservations(),
                                   corners_end[i].getNodeLinkedPtr(), ptr);
        } else {
            // TODO
            ROS_DEBUG_STREAM("Point seen ");
            addLandmarkObservation(corners_end[i].getObservations(),
                                   corners_end[i].getNodeLinkedPtr(),
                                   ptr_landmark_seen);
        }
    }
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    testNoNanInPrior(const std::string& before) const {
    _prior->testNoNanInPrior(before);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::testInfoNonNul(
    const std::string& before) const {}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    updatePriorEdgeCovariance() {
    _prior->updatePriorEdgeCovariance();
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    setKernelSizeDependingOnAge(g2o::OptimizableGraph::Edge* e, bool step) {
    e->robustKernel()->setDelta(100);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    getExtremaPrior(double& size_x, double& size_y) const {
    auto edges = _prior->getEdges();

    double max_x, min_x, max_y, min_y;
    bool flag_init = false;
    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            geometry_msgs::Point p;
            g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
            auto vertex = ptr->estimate().toVector();
            if (flag_init == false) {
                flag_init = true;
                max_x = vertex(0);
                max_y = vertex(1);
                min_x = vertex(0);
                min_y = vertex(1);
            } else {
                if (max_x < vertex(0)) {
                    max_x = vertex(0);
                }
                if (max_y < vertex(1)) {
                    max_y = vertex(1);
                }
                if (min_x > vertex(0)) {
                    min_x = vertex(0);
                }
                if (min_y > vertex(1)) {
                    min_y = vertex(1);
                }
            }
        }
    }

    max_x = std::abs(max_x);
    max_y = std::abs(max_y);
    min_x = std::abs(min_x);
    min_y = std::abs(min_y);

    if (max_x > min_x) {
        size_x = max_x + 10;
    } else {
        size_x = min_x + 10;
    }
    if (max_y > min_y) {
        size_y = max_y + 10;
    } else {
        size_y = min_y + 10;
    }
}

////OPTIMIZATION

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::prepare() {
    _optimizable_graph.prepare();
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline int AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    optimize_simple(int max_iter) {
    std::cout << "Check before optimization" << std::endl;
    testInfoNonNul("At first");

    int count = 0;
    std::deque<double> _chi_kernel;
    for (; count < max_iter && count < 2; ++count) {
        ROS_INFO("Optimizing two times at least because we need to ");
        _optimizable_graph.optimize(1);
        _optimizable_graph.computeActiveErrors();
        _chi_kernel.push_back(_optimizable_graph.chi2());
        saveErrorStep();

        testNoNanInPrior("optimized with huber");
        testInfoNonNul("optimized with huber");
        testNoNanInPrior("update prior edge cov after opti huber");
    }
    if (max_iter >= 2) {
        while (ErrorStable(_chi_kernel) == false && count < max_iter) {
            count++;
            ROS_INFO("Optimizing until error stops it ");
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

    if (count >= max_iter) {
        ROS_ERROR(
            "\n\n****** ATTENTION THE OPTIMIZATION PROBABLY FAILED. It "
            "iterated for too long, please double check the resulting map ! "
            "*****\n");
    }

    return count;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline std::pair<int, int>
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::optimize(
    int max_iter) {
    int count_huber = 0;
    int count_dcs = 0;

    _chi2s.clear();
    std::cout << "Checking that all info mat are positive semi def"
              << std::endl;
    assert(_optimizable_graph.verifyInformationMatrices(true) == true);

    ROS_DEBUG_STREAM("BEFORE THE OPTIMIZATION BUT AFTER ADDING A NODE");

    /********** HUBER kernel ***********/

    _flag_optimize = checkAbleToOptimize();

    if (_flag_optimize == true) {
        ROS_DEBUG_STREAM("OPTIMIZE");

        if (_flag_use_robust_kernel) {
            if (_flag_use_huber_kernel) {
                std::cout << "Using Huber kernel" << std::endl;
                setAgeingHuberKernel();

                testNoNanInPrior("set age in huber kernel");
                testInfoNonNul("set age in huber kernel");
                testNoNanInPrior("update prior edge cov");

                count_huber = optimize_simple(max_iter);
            }
            /********** DCS kernel ***********/
            if (_flag_use_dcs_kernel) {
                std::cout << "Using DCS kernel" << std::endl;
                setAgeingDCSKernel();

                testNoNanInPrior("set age in DCS kernel");

                count_dcs = optimize_simple(max_iter);
            }
        } else {
            testNoNanInPrior("set age in huber kernel");
            testInfoNonNul("set age in huber kernel");
            testNoNanInPrior("update prior edge cov");

            count_huber = optimize_simple(max_iter);
        }
    } else {
        ROS_DEBUG_STREAM("No Optimization :(");
    }

    return std::pair<int, int>(count_huber, count_dcs);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    setAgeingHuberKernel() {
    auto idmapedges = _optimizable_graph.edges();
    for (auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite) {
        g2o::OptimizableGraph::Edge* e =
            static_cast<g2o::OptimizableGraph::Edge*>(*ite);
        auto huber = new g2o::RobustKernelHuber();
        e->setRobustKernel(huber);
        setKernelSizeDependingOnAge(e, true);
    }
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    setAgeingDCSKernel() {
    auto idmapedges = _optimizable_graph.edges();
    for (auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite) {
        g2o::OptimizableGraph::Edge* e =
            static_cast<g2o::OptimizableGraph::Edge*>(*ite);
        auto dcs = new g2o::RobustKernelDCS();
        e->setRobustKernel(dcs);
        setKernelSizeDependingOnAge(e, false);
    }
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline bool
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::ErrorStable(
    const std::deque<double>& _chi_kernel,
    int max_steps) {
    if (_chi_kernel.size() < 2) {
        throw std::runtime_error(
            "Cannot determine error difference with less than 2 optimization "
            "cycles.");
    }
    int max_steps_count = 0;
    std::vector<double> error_diff;
    double error_mean = 0;
    auto it = _chi_kernel.rbegin();
    ++it;
    for (; it != _chi_kernel.rend(); ++it) {
        if (max_steps_count == max_steps) {
            break;
        }
        error_diff.push_back(std::abs(*it - *(it - 1)));
        error_mean = error_mean + std::abs((*it - *(it - 1)));
        ++max_steps_count;
    }
    error_mean = error_mean / max_steps_count;

    it = _chi_kernel.rbegin();
    ++it;
    max_steps_count = 0;
    for (; it != _chi_kernel.rend(); ++it) {
        if (max_steps_count == max_steps) {
            break;
        }
        ROS_DEBUG_STREAM(std::abs(*it - *(it - 1)) << " ");
        ++max_steps_count;
    }
    ROS_INFO_STREAM("Error is " << error_mean << "threshold is "
                                << _error_threshold_stop_optimization);
    if (error_mean < _error_threshold_stop_optimization) {
        return true;
    }
    return false;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::clearPrior() {
    ROS_DEBUG_STREAM("IMPORTANT size " << _optimizable_graph.vertices().size());
    //	int i = 0;

    for (auto vertex : _prior->getNodes()) {
        _optimizable_graph.removeVertex(vertex, false);
    }
    _prior->clear();
    assert(_prior->getNodes().size() == 0);

    ROS_DEBUG_STREAM("IMPORTANT size " << _optimizable_graph.vertices().size());
    ROS_DEBUG_STREAM("DONE removing ");
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    checkRobotPoseNotMoved(const std::string& when) {
    ROS_DEBUG_STREAM("testing after " << when);
    for (auto it = _nodes_ndt.begin(); it != _nodes_ndt.end(); ++it) {
        int init_x = (*it)->initial_transfo.toVector()(0);
        int init_y = (*it)->initial_transfo.toVector()(1);
        int init_z = (*it)->initial_transfo.toVector()(2) * 10;

        int update_x = (*it)->estimate().toVector()(0);
        int update_y = (*it)->estimate().toVector()(1);
        int update_z = (*it)->estimate().toVector()(2) * 10;

        if (init_x != update_x || init_y != update_y || init_z != update_z) {
            ROS_DEBUG_STREAM(" init " << init_x << " " << init_y << " "
                                      << init_z << " == " << update_x << " "
                                      << update_y << " " << update_z);
            throw std::runtime_error("MOVE BASE");
        }
    }
}
