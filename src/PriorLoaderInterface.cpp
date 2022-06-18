#include "auto_complete_graph/PriorLoaderInterface.hpp"

void AASS::acg::PriorLoaderInterface::transformOntoSLAM() {
    assert(_same_point_prior.size() >= 2);

    cv::Point2f srcTri[2];
    cv::Point2f dstTri[2];
    srcTri[0] = _same_point_prior[0];
    srcTri[1] = _same_point_prior[1];

    dstTri[0] = _same_point_slam[0];
    dstTri[1] = _same_point_slam[1];

    // Translate to origin
    cv::Point2d translation = -srcTri[0];
    double angle = 0;
    cv::Mat warp_mat = (cv::Mat_<double>(3, 3) << std::cos(angle),
                        -std::sin(angle), translation.x, std::sin(angle),
                        std::cos(angle), translation.y, 0, 0, 1);
    AffineTransformGraph(warp_mat);

    // Rotate
    Eigen::Vector2d vece1s;
    vece1s << srcTri[1].x - srcTri[0].x, srcTri[1].y - srcTri[0].y;
    Eigen::Vector2d vece2s;
    vece2s << dstTri[1].x - dstTri[0].x, dstTri[1].y - dstTri[0].y;

    angle = atan2(vece2s(1), vece2s(0)) - atan2(vece1s(1), vece1s(0));
    if (angle < 0)
        angle += 2 * M_PI;

    // 				std::cout << "Angle : "<< angle << std::endl;
    // 				angle = angle * 2 * 3.14157 / 360;
    warp_mat = (cv::Mat_<double>(3, 3) << std::cos(angle), -std::sin(angle), 0,
                std::sin(angle), std::cos(angle), 0, 0, 0, 1);
    // 				std::cout << "mat " << warp_mat << std::endl;
    AffineTransformGraph(warp_mat);

    // Scale
    cv::Point2f vec1 = srcTri[1] - srcTri[0];
    cv::Point2f vec2 = dstTri[1] - dstTri[0];
    cv::Mat mat_transfo =
        (cv::Mat_<float>(2, 2) << vec2.x / vec1.x, 0, 0, vec2.y / vec1.y);
    double l1 = std::sqrt((vec1.x * vec1.x) + (vec1.y * vec1.y));
    double l2 = std::sqrt((vec2.x * vec2.x) + (vec2.y * vec2.y));
    double ratio = l2 / l1;

    std::pair<PriorVertexIterator, PriorVertexIterator> vp;
    // vertices access all the vertix
    // Classify them in order
    //  				std::cout << "Gph size : " <<
    //  _graph.getNumVertices() << std::endl; int i = 0 ;
    for (vp = boost::vertices(_prior_graph); vp.first != vp.second;
         ++vp.first) {
        // 					std::cout << "going throught grph " <<
        // i
        // << std::endl; ++i;
        PriorVertex v = *vp.first;
        // ATTENTION !
        cv::Point2d point;
        point.x = _prior_graph[v].getX();
        point.y = _prior_graph[v].getY();

        cv::Point2d point_out = point * ratio;

        _prior_graph[v].setX(point_out.x);
        _prior_graph[v].setY(point_out.y);
    }
    // 				//Translate back
    translation = srcTri[0];
    angle = 0;
    warp_mat = (cv::Mat_<double>(3, 3) << std::cos(angle), -std::sin(angle),
                translation.x, std::sin(angle), std::cos(angle), translation.y,
                0, 0, 1);
    AffineTransformGraph(warp_mat);
    //
    // 				//Translate to point
    translation = dstTri[0] - srcTri[0];
    angle = 0;
    warp_mat = (cv::Mat_<double>(3, 3) << std::cos(angle), -std::sin(angle),
                translation.x, std::sin(angle), std::cos(angle), translation.y,
                0, 0, 1);
    AffineTransformGraph(warp_mat);
}

void AASS::acg::PriorLoaderInterface::extractCornerPrior() {
    // 		AASS::das::BasementPriorLine basement;

    _prior_graph.clear();

    cv::Mat src = cv::imread(_file, 1);
    cv::cvtColor(src, _img_gray, cv::COLOR_BGR2GRAY);

    AASS::vodigrex::LineFollowerGraphCorners<> graph_corners;
    graph_corners.setD(2);
    //	graph_corners.setMaxDeviation((45 * M_PI) / 180);
    graph_corners.setMaxDeviation(_max_deviation_for_corner);
    graph_corners.inputMap(_img_gray);
    graph_corners.thin();
    auto prior_graph = graph_corners.getGraph();

    bettergraph::PseudoGraph<PriorAttr, AASS::vodigrex::SimpleEdge> prior_out;
    convertGraph(prior_graph, prior_out);
    // Very convulted code TODO
    AASS::acg::PriorLoaderInterface::PriorGraph simple_graph;
    bettergraph::toSimpleGraph<PriorAttr, AASS::vodigrex::SimpleEdge>(
        prior_out, simple_graph);

    // Check if same vertex twice

    for (auto vp = boost::vertices(simple_graph); vp.first != vp.second;
         ++vp.first) {
        auto vp1 = vp;
        ++vp1.first;
        for (vp1; vp1.first != vp1.second; ++vp1.first) {
            if (simple_graph[*vp.first].x == simple_graph[*vp1.first].x &&
                simple_graph[*vp.first].y == simple_graph[*vp1.first].y) {
                throw std::runtime_error(
                    "Same vertex twice in input prior graph");
            }
        }
    }
    _prior_graph = simple_graph;
}

void AASS::acg::PriorLoaderInterface::initialize(
    const std::vector<cv::Point2f>& pt_slam,
    const std::vector<cv::Point2f>& pt_prior) {
    assert(pt_slam.size() >= 2);
    assert(pt_prior.size() == pt_slam.size());

    std::cout << pt_prior[0] << " " << pt_slam[0] << std::endl
              << pt_prior[1] << " " << pt_slam[1] << std::endl;

    _same_point_prior.clear();
    _same_point_slam.clear();

    auto randomNoise = [](double mean, double deviationt) -> double {
        std::default_random_engine engine{std::random_device()()};
        std::normal_distribution<double> dist(mean, deviationt);
        return dist(engine);
    };

    // 				cv::Point center = cv::Point(7, 08);

    cv::Mat rot_mat = cv::getRotationMatrix2D(_center, _angle, _scale);
    std::cout << rot_mat << std::endl;
    // 				exit(0);

    auto rotatef = [](const cv::Mat& rot_mat,
                      const cv::Point2d point) -> cv::Point2d {
        // Matrix multiplication
        cv::Mat point_m = (cv::Mat_<double>(3, 1) << point.x, point.y, 1);
        cv::Mat mat_out = rot_mat * point_m;
        // 					std::cout << "Mat out " <<
        // mat_out
        // << std::endl;
        cv::Point2d point_out;
        point_out.x = mat_out.at<double>(0);
        point_out.y = mat_out.at<double>(1);
        return point_out;
    };

    auto noise_x = randomNoise(0, _deviation);
    auto noise_y = randomNoise(0, _deviation);

    cv::Point2f out =
        cv::Point2f(pt_slam[0].x + noise_x, pt_slam[0].y + noise_y);
    cv::Point2f slam_point = rotatef(rot_mat, out);

    _same_point_prior.push_back(pt_prior[0]);
    _same_point_slam.push_back(slam_point);

    noise_x = randomNoise(0, _deviation);
    noise_y = randomNoise(0, _deviation);
    out = cv::Point2f(pt_slam[1].x + noise_x, pt_slam[1].y + noise_y);
    slam_point = rotatef(rot_mat, out);

    _same_point_prior.push_back(pt_prior[1]);
    _same_point_slam.push_back(slam_point);
}

void AASS::acg::PriorLoaderInterface::rotateGraph(const cv::Mat& rot_mat) {
    auto transf = [](const cv::Mat& rot_mat,
                     const cv::Point2d point) -> cv::Point2d {
        // Matrix multiplication
        cv::Mat point_m = (cv::Mat_<double>(3, 1) << point.x, point.y);
        cv::Mat mat_out = rot_mat * point_m;
        // 					std::cout << "Mat out " <<
        // mat_out
        // << std::endl;
        cv::Point2d point_out;
        point_out.x = mat_out.at<double>(0);
        point_out.y = mat_out.at<double>(1);
        return point_out;
    };

    std::pair<PriorVertexIterator, PriorVertexIterator> vp;
    // vertices access all the vertix
    // Classify them in order
    //  				std::cout << "Gph size : " <<
    //  _graph.getNumVertices() << std::endl;
    int i = 0;
    for (vp = boost::vertices(_prior_graph); vp.first != vp.second;
         ++vp.first) {
        // 					std::cout << "going throught grph " <<
        // i
        // << std::endl; ++i;
        PriorVertex v = *vp.first;
        // ATTENTION !
        cv::Point2d point;
        point.x = _prior_graph[v].getX();
        point.y = _prior_graph[v].getY();

        auto point_out = transf(rot_mat, point);

        // Matrix multiplication

        std::cout << "Point out " << point_out << std::endl;

        std::cout << "OLD value " << _prior_graph[v].getX() << " "
                  << _prior_graph[v].getY() << std::endl;
        _prior_graph[v].setX(point_out.x);
        _prior_graph[v].setY(point_out.y);
        _corner_prior_matched.push_back(point_out);
        std::cout << "New value " << _corner_prior_matched[i].x << " "
                  << _corner_prior_matched[i].y << std::endl;
        std::cout << "New value " << _prior_graph[v].getX() << " "
                  << _prior_graph[v].getY() << std::endl;
        ++i;
    }
}

void AASS::acg::PriorLoaderInterface::AffineTransformGraph(
    const cv::Mat& warp_transfo) {
    auto transf = [](const cv::Mat& warp_transfo,
                     const cv::Point2d point) -> cv::Point2d {
        // Matrix multiplication
        cv::Mat point_m = (cv::Mat_<double>(3, 1) << point.x, point.y, 1);
        cv::Mat mat_out = warp_transfo * point_m;
        // 					std::cout << "Mat out " <<
        // mat_out
        // << std::endl;
        cv::Point2d point_out;
        point_out.x = mat_out.at<double>(0);
        point_out.y = mat_out.at<double>(1);
        //		std::cout << "reutnr point " << point_out.x << " " <<
        // point_out.y << std::endl;
        return point_out;
    };

    std::pair<PriorVertexIterator, PriorVertexIterator> vp;
    // vertices access all the vertix
    // Classify them in order
    //  				std::cout << "Gph size : " <<
    //  _graph.getNumVertices() << std::endl;
    int i = 0;
    for (vp = boost::vertices(_prior_graph); vp.first != vp.second;
         ++vp.first) {
        // 					std::cout << "going throught grph " <<
        // i
        // << std::endl; ++i;
        PriorVertex v = *vp.first;
        // ATTENTION !
        cv::Point2d point;
        point.x = _prior_graph[v].getX();
        point.y = _prior_graph[v].getY();

        cv::Point2d point_out = transf(warp_transfo, point);

        // Matrix multiplication

        // 					std::cout << "Point out " <<
        // point_out
        // << std::endl;

        // 					std::cout << "OLD value " <<
        // _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
        _prior_graph[v].setX(point_out.x);
        _prior_graph[v].setY(point_out.y);
        _corner_prior_matched.push_back(point_out);
        // 					std::cout << "New value " <<
        // _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y <<
        // std::endl; 					std::cout << "New value "
        // << _prior_graph[v].getX() << " "
        // << _prior_graph[v].getY() << std::endl;
        ++i;
    }
}

void AASS::acg::PriorLoaderInterface::noTwiceSameEdge(
    bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                             AASS::vodigrex::SimpleEdge> graph) {
    //	std::cout << "No twice same edge" << std::endl;
    std::pair<
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::VertexIterator,
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::VertexIterator>
        vp;

    std::vector<bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                         AASS::vodigrex::SimpleEdge>::Edge>
        e_vec;

    for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
        std::cout << "Vertex" << std::endl;
        // 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
        // AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::Vertex v =
            *vp.first;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::EdgeIterator
            out_i,
            out_end;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::Edge e;

        std::vector<bettergraph::PseudoGraph<
            AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex>
            vertices_out_edge;

        std::vector<bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                             AASS::vodigrex::SimpleEdge>::Edge>
            edge_seen;

        //		std::cout << "Vertiuce size " <<
        // vertices_out_edge.size()
        //<< std::endl; 		std::cout << " v has " <<
        // graph.getNumEdges(v) << std::endl;

        for (boost::tie(out_i, out_end) = boost::out_edges(v, graph);
             out_i != out_end; ++out_i) {
            //			std::cout << "Vertiuce size " <<
            //vertices_out_edge.size()
            //<< std::endl; 			std::cout << "Removing an edge
            // test" << std::endl;
            e = *out_i;

            for (std::vector<bettergraph::PseudoGraph<
                     AASS::vodigrex::SimpleNode,
                     AASS::vodigrex::SimpleEdge>::Edge>::iterator it =
                     edge_seen.begin();
                 it != edge_seen.end(); ++it) {
                if (e == *it) {
                    //					std::cout << "EDGE seen on this node
                    //:("
                    //<< std::endl;
                    assert(true == false);
                }
            }

            e_vec.push_back(e);
            edge_seen.push_back(e);
        }
    }

    //	std::cout << "So no twice for now! " << std::endl;
}

void AASS::acg::PriorLoaderInterface::toSimpleGraph(
    bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                             AASS::vodigrex::SimpleEdge>& prior) {
    noTwiceSameEdge(prior);

    std::pair<
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::VertexIterator,
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::VertexIterator>
        vp;
    std::deque<PriorLoaderInterface::PriorGraph::Vertex> vec_deque;

    //	std::cout << "prior" << prior.getNumVertices() << std::endl <<
    // std::endl;

    int nb = prior.getNumVertices();

    for (vp = boost::vertices(prior); vp.first != vp.second; ++vp.first) {
        std::cout << "Vertex" << std::endl;
        // 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
        // AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
        auto v = *vp.first;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::EdgeIterator
            out_i,
            out_end;
        bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                 AASS::vodigrex::SimpleEdge>::Edge e;

        std::vector<bettergraph::PseudoGraph<
            AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex>
            vertices_out_edge;

        std::vector<bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                             AASS::vodigrex::SimpleEdge>::Edge>
            edge_seen;

        std::cout << "Vertiuce size " << vertices_out_edge.size() << std::endl;
        std::cout << " v has " << prior.getNumEdges(v) << std::endl;

        for (boost::tie(out_i, out_end) = boost::out_edges(v, prior);
             out_i != out_end; ++out_i) {
            std::cout << "Vertiuce size " << vertices_out_edge.size()
                      << std::endl;
            std::cout << "Removing an edge test" << std::endl;
            e = *out_i;

            for (auto it = edge_seen.begin(); it != edge_seen.end(); ++it) {
                if (e == *it) {
                    std::cout << "EDGE seen :(" << std::endl;
                    assert(true == false);
                }
            }

            std::cout << "Vertiuce size " << vertices_out_edge.size()
                      << " and edge " << e << std::endl;
            edge_seen.push_back(e);

            std::cout << "Vertiuce size " << vertices_out_edge.size()
                      << std::endl;

            // 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
            // AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e,
            // (graph));
            auto targ = boost::target(e, prior);
            std::cout << "Vertiuce size " << vertices_out_edge.size()
                      << std::endl;
            for (auto it = vertices_out_edge.begin();
                 it != vertices_out_edge.end(); ++it) {
                if (targ == *it) {
                    std::cout << "Removing an edge" << std::endl;
                    prior.removeEdge(e);
                    assert(nb == prior.getNumVertices());
                }
            }
            std::cout << "Psuh back end targ " << std::endl;
            vertices_out_edge.push_back(targ);
        }

        std::cout << "Our for vertex" << std::endl;
    }
}

void AASS::acg::PriorLoaderInterface::convertGraph(
    const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                   AASS::vodigrex::SimpleEdge>& input,
    bettergraph::PseudoGraph<AASS::acg::PriorAttr, AASS::vodigrex::SimpleEdge>&
        output) {
    output.clear();
    std::pair<typename bettergraph::PseudoGraph<
                  AASS::vodigrex::SimpleNode,
                  AASS::vodigrex::SimpleEdge>::VertexIterator,
              typename bettergraph::PseudoGraph<
                  AASS::vodigrex::SimpleNode,
                  AASS::vodigrex::SimpleEdge>::VertexIterator>
        vp;

    std::deque<typename bettergraph::PseudoGraph<
        PriorAttr, AASS::vodigrex::SimpleEdge>::Vertex>
        vec_deque;

    std::deque<typename bettergraph::PseudoGraph<
        AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex>
        vec_deque_input;

    // Add all vertex
    for (vp = boost::vertices(input); vp.first != vp.second; ++vp.first) {
        // 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
        // AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
        typename bettergraph::PseudoGraph<
            AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v =
            *vp.first;
        typename bettergraph::PseudoGraph<
            PriorAttr, AASS::vodigrex::SimpleEdge>::Vertex vertex_out;

        PriorAttr priorattr(input[v]);

        output.addVertex(vertex_out, priorattr);

        vec_deque.push_back(vertex_out);
        vec_deque_input.push_back(v);
    }

    // Simply add all edges
    int self_link = 0;
    for (vp = boost::vertices(input); vp.first != vp.second; ++vp.first) {
        typename bettergraph::PseudoGraph<
            AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v =
            *vp.first;
        typename bettergraph::PseudoGraph<
            AASS::vodigrex::SimpleNode,
            AASS::vodigrex::SimpleEdge>::EdgeIterator out_i,
            out_end;
        typename bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode,
                                          AASS::vodigrex::SimpleEdge>::Edge e;

        typename bettergraph::PseudoGraph<
            PriorAttr, AASS::vodigrex::SimpleEdge>::Vertex output_src;
        for (int i = 0; i < vec_deque.size(); ++i) {
            if (vec_deque_input[i] == v) {
                output_src = vec_deque[i];
            }
        }

        for (boost::tie(out_i, out_end) = boost::out_edges(v, (input));
             out_i != out_end; ++out_i) {
            e = *out_i;
            typename bettergraph::PseudoGraph<
                AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex
                targ = boost::target(e, input);
            typename bettergraph::PseudoGraph<
                AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex
                src = boost::source(e, input);
            typename bettergraph::PseudoGraph<
                PriorAttr, AASS::vodigrex::SimpleEdge>::Vertex output_targ;
            for (int i = 0; i < vec_deque.size(); ++i) {
                if (vec_deque_input[i] == targ) {
                    output_targ = vec_deque[i];
                }
            }

            typename bettergraph::PseudoGraph<
                PriorAttr, AASS::vodigrex::SimpleEdge>::Edge e_out_output;
            output.addEdge(e_out_output, output_src, output_targ, input[e]);
        }
    }

    // 				assert(2 * output.getNumEdges() ==
    // input.getNumEdges());
    assert(output.getNumVertices() == input.getNumVertices());
}
