#ifndef AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016
#define AUTOCOMPLETEGRAPH_VISUALIZATIONACG_18102016

#include "acg_conversion.hpp"
#include "auto_complete_graph/ACGBase.hpp"
#include "ndt_feature_finder/ndt_cell_2d_utils.hpp"
#include "ndt_map/NDTVectorMapMsg.h"
#include "occupancy_grid_utils/combine_grids.h"
#include "utils.hpp"

namespace AASS {

namespace acg {

template <typename Prior, typename VertexPrior, typename EdgePrior>
class VisuAutoCompleteGraphBase {
   protected:
    ros::NodeHandle _nh;
    ros::Publisher _last_ndtmap_occ;
    ros::Publisher _last_ndtmap2_occ;
    ros::Publisher _last_ndtmap_full_occ;
    ros::Publisher _prior_map_occ;
    ros::Publisher _marker_pub;
    ros::Publisher _ndt_node_pub;
    ros::Publisher _prior_node_pub;
    ros::Publisher _corner_ndt_node_pub;
    ros::Publisher _ndtmap;
    ros::Publisher _angles_pub;
    ros::Publisher _angles_prior_pub;
    ros::Publisher _anglesw_pub;
    ros::Publisher _anglesw_prior_pub;
    ros::Publisher _gaussian_pub;
    ros::Publisher _gaussian_pub2;
    ros::Publisher _acg_gdim;
    ros::Publisher _acg_gdim_om;
    ros::Publisher _observation_edge_pub;

    int _nb_of_zone;
    std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
    std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids_original;
    visualization_msgs::Marker _prior_edge_markers;
    visualization_msgs::Marker _observation_edge_markers;
    visualization_msgs::Marker _ndt_node_markers;
    visualization_msgs::Marker _prior_node_markers;
    visualization_msgs::Marker _corner_ndt_node_markers;
    visualization_msgs::Marker _angles_markers;
    visualization_msgs::Marker _anglesw_markers;
    visualization_msgs::Marker _angles_prior_markers;
    visualization_msgs::Marker _anglesw_prior_markers;
    visualization_msgs::Marker _gaussian_that_gave_corners;
    visualization_msgs::Marker _gaussian_that_gave_corners2;

    double _resolution;

    std::string _image_file;

   public:
    VisuAutoCompleteGraphBase(ros::NodeHandle nh,
                              const std::string& world_frame_id = "/world")
        : _nb_of_zone(-1), _resolution(0.1) {
        _nh = nh;
        _last_ndtmap_occ =
            _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg_occ", 10);
        _last_ndtmap2_occ =
            _nh.advertise<nav_msgs::OccupancyGrid>("lastgraphmap_acg2_occ", 10);
        _last_ndtmap_full_occ =
            _nh.advertise<nav_msgs::OccupancyGrid>("occ_full", 10);
        _prior_map_occ =
            _nh.advertise<nav_msgs::OccupancyGrid>("occ_prior", 10);
        _marker_pub =
            _nh.advertise<visualization_msgs::Marker>("prior_marker", 10);
        _ndt_node_pub =
            _nh.advertise<visualization_msgs::Marker>("ndt_nodes_marker", 10);
        _prior_node_pub =
            _nh.advertise<visualization_msgs::Marker>("prior_nodes_marker", 10);
        _corner_ndt_node_pub =
            _nh.advertise<visualization_msgs::Marker>("corner_ndt_marker", 10);
        _angles_pub = _nh.advertise<visualization_msgs::Marker>("angles", 10);
        _angles_prior_pub =
            _nh.advertise<visualization_msgs::Marker>("angles_prior", 10);
        _anglesw_pub =
            _nh.advertise<visualization_msgs::Marker>("angleswidth", 10);
        _anglesw_prior_pub =
            _nh.advertise<visualization_msgs::Marker>("angleswidth_prior", 10);
        _ndtmap =
            _nh.advertise<ndt_map::NDTVectorMapMsg>("ndt_map_msg_node", 10);
        _gaussian_pub = _nh.advertise<visualization_msgs::Marker>(
            "gaussian_that_gave_corners", 10);
        _gaussian_pub2 = _nh.advertise<visualization_msgs::Marker>(
            "gaussian_that_gave_corners2", 10);
        _observation_edge_pub =
            _nh.advertise<visualization_msgs::Marker>("observation_edge", 10);

        _prior_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
        _prior_edge_markers.header.frame_id = world_frame_id;
        _prior_edge_markers.ns = "acg";
        _prior_edge_markers.id = 0;
        _prior_edge_markers.scale.x = 0.2;
        _prior_edge_markers.scale.y = 0.2;
        _prior_edge_markers.color.b = 0.0f;
        _prior_edge_markers.color.a = 1.0;

        _observation_edge_markers.type = visualization_msgs::Marker::LINE_LIST;
        _observation_edge_markers.header.frame_id = world_frame_id;
        _observation_edge_markers.ns = "acg";
        _observation_edge_markers.id = 0;
        _observation_edge_markers.scale.x = 0.2;
        _observation_edge_markers.scale.y = 0.2;
        _observation_edge_markers.color.g = 1.0f;
        _observation_edge_markers.color.a = 1.0;

        _ndt_node_markers.type = visualization_msgs::Marker::POINTS;
        _ndt_node_markers.header.frame_id = world_frame_id;
        _ndt_node_markers.ns = "acg";
        _ndt_node_markers.id = 1;
        _ndt_node_markers.scale.x = 0.2;
        _ndt_node_markers.scale.y = 0.2;
        _ndt_node_markers.color.r = 1.0f;
        _ndt_node_markers.color.a = 1.0;

        _prior_node_markers.type = visualization_msgs::Marker::POINTS;
        _prior_node_markers.header.frame_id = world_frame_id;
        _prior_node_markers.ns = "acg";
        _prior_node_markers.id = 1;
        _prior_node_markers.scale.x = 0.5;
        _prior_node_markers.scale.y = 0.5;
        _prior_node_markers.color.r = 0.5f;
        _prior_node_markers.color.a = 1.0;

        _corner_ndt_node_markers.type = visualization_msgs::Marker::POINTS;
        _corner_ndt_node_markers.header.frame_id = world_frame_id;
        _corner_ndt_node_markers.ns = "acg";
        _corner_ndt_node_markers.id = 2;
        _corner_ndt_node_markers.scale.x = 0.5;
        _corner_ndt_node_markers.scale.y = 0.5;
        _corner_ndt_node_markers.color.g = 1.0f;
        _corner_ndt_node_markers.color.a = 1.0;

        _angles_markers.type = visualization_msgs::Marker::LINE_LIST;
        _angles_markers.header.frame_id = world_frame_id;
        _angles_markers.ns = "acg";
        _angles_markers.id = 4;
        _angles_markers.scale.x = 0.2;
        _angles_markers.scale.y = 0.2;
        _angles_markers.color.g = 1.0f;
        _angles_markers.color.r = 0.0f;
        _angles_markers.color.a = 1.0;

        _anglesw_markers.type = visualization_msgs::Marker::LINE_LIST;
        _anglesw_markers.header.frame_id = world_frame_id;
        _anglesw_markers.ns = "acg";
        _anglesw_markers.id = 5;
        _anglesw_markers.scale.x = 0.1;
        _anglesw_markers.scale.y = 0.1;
        _anglesw_markers.color.g = 1.0f;
        _anglesw_markers.color.r = 0.2f;
        _anglesw_markers.color.a = 1.0;

        _angles_prior_markers.type = visualization_msgs::Marker::LINE_LIST;
        _angles_prior_markers.header.frame_id = world_frame_id;
        _angles_prior_markers.ns = "acg";
        _angles_prior_markers.id = 6;
        _angles_prior_markers.scale.x = 0.2;
        _angles_prior_markers.scale.y = 0.2;
        _angles_prior_markers.color.g = 0.0f;
        _angles_prior_markers.color.r = 0.5f;
        _angles_prior_markers.color.a = 1.0;

        _anglesw_prior_markers.type = visualization_msgs::Marker::LINE_LIST;
        _anglesw_prior_markers.header.frame_id = world_frame_id;
        _anglesw_prior_markers.ns = "acg";
        _anglesw_prior_markers.id = 6;
        _anglesw_prior_markers.scale.x = 0.1;
        _anglesw_prior_markers.scale.y = 0.1;
        _anglesw_prior_markers.color.g = 0.1f;
        _anglesw_prior_markers.color.r = 0.5f;
        _anglesw_prior_markers.color.a = 1.0;

        _gaussian_that_gave_corners.type =
            visualization_msgs::Marker::LINE_LIST;
        _gaussian_that_gave_corners.header.frame_id = world_frame_id;
        _gaussian_that_gave_corners.ns = "acg";
        _gaussian_that_gave_corners.id = 7;
        _gaussian_that_gave_corners.scale.x = 0.2;
        _gaussian_that_gave_corners.scale.y = 0.2;
        _gaussian_that_gave_corners.color.g = 1.0f;
        _gaussian_that_gave_corners.color.r = 0.5f;
        _gaussian_that_gave_corners.color.a = 1.0;

        _gaussian_that_gave_corners2.type =
            visualization_msgs::Marker::LINE_LIST;
        _gaussian_that_gave_corners2.header.frame_id = world_frame_id;
        _gaussian_that_gave_corners2.ns = "acg";
        _gaussian_that_gave_corners2.id = 7;
        _gaussian_that_gave_corners2.scale.x = 0.2;
        _gaussian_that_gave_corners2.scale.y = 0.2;
        _gaussian_that_gave_corners2.color.r = 1.0f;
        _gaussian_that_gave_corners2.color.b = 1.0f;
        _gaussian_that_gave_corners2.color.a = 1.0;
    }
    void setImageFileNameOut(const std::string& f) { _image_file = f; }

    void updateRvizStepByStep(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        drawPrior(acg);
        drawCornersNdt(acg);

        if (_nb_of_zone != acg.getRobotNodes().size()) {
            nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
            nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
            ACGtoOccupancyGrid(acg, occ_outt, acg.getRobotNodes().size() - 2);

            ROS_DEBUG_STREAM("Going to publish");

            ROS_DEBUG_STREAM("WELLL HERE IT IS : "
                             << occ_outt->info.origin.position << " ori "
                             << occ_outt->info.origin.orientation << std::endl);

            ROS_DEBUG_STREAM("Pub");
            _last_ndtmap_full_occ.publish<nav_msgs::OccupancyGrid>(*occ_outt);

            ndt_map::NDTVectorMapMsg msg;
            ACGToVectorMaps(acg, msg);
            _ndtmap.publish(msg);
        }
    }

    void toOcc(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        drawPrior(acg);
        drawCornersNdt(acg);
        drawAngles(acg);

        if (_nb_of_zone != acg.getRobotNodes().size()) {
            nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
            nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
            ACGtoOccupancyGrid(acg, occ_outt);
            _last_ndtmap_full_occ.publish<nav_msgs::OccupancyGrid>(*occ_outt);
        }
    }

    void updateRviz(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        drawPrior(acg);
        drawCornersNdt(acg);
        if (_nb_of_zone != acg.getRobotNodes().size()) {
            drawObservations(acg);
            drawGaussiansThatGaveCorners(acg);
            drawRobotPoses(acg);

            ndt_map::NDTVectorMapMsg msg;
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            ACGToVectorMaps(acg, msg);
            _ndtmap.publish(msg);

            _nb_of_zone = acg.getRobotNodes().size();
            ROS_DEBUG_STREAM("Done");
        }
    }

    void updateRvizNoNDT(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        drawPrior(acg);
        drawCornersNdt(acg);
        drawAngles(acg);
    }

    void updateRvizV2(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        drawPrior(acg);
        drawCornersNdt(acg);

        if (_nb_of_zone != acg.getRobotNodes().size()) {
            grids.clear();
            ROS_DEBUG_STREAM("update the zones");

            for (size_t i = 0; i < acg.getRobotNodes().size(); ++i) {
                nav_msgs::OccupancyGrid* omap_tmp =
                    new nav_msgs::OccupancyGrid();
                perception_oru::toOccupancyGrid(
                    acg.getRobotNodes()[i]->getMap().get(), *omap_tmp,
                    _resolution, "/world");
                auto node = acg.getRobotNodes()[i];
                auto vertex = node->estimate().toIsometry();
                moveOccupancyMap(*omap_tmp, vertex);
                omap_tmp->header.frame_id = "/world";
                omap_tmp->header.stamp = ros::Time::now();
                nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
                grids.push_back(ptr);

                geometry_msgs::Point p;
                auto vertex2 = node->estimate().toVector();
                // Getting the translation out of the transform :
                // https://en.wikipedia.org/wiki/Transformation_matrix
                p.x = vertex2(0);
                p.y = vertex2(1);
                p.z = 0;

                _ndt_node_markers.points.push_back(p);
            }
        }
        nav_msgs::OccupancyGrid::Ptr final;
        if (grids.size() > 0) {
            ROS_DEBUG_STREAM("Combining " << grids.size());
            final = occupancy_grid_utils::combineGrids(grids);
            final->header.frame_id = "/world";
            final->header.stamp = ros::Time::now();
            _last_ndtmap_occ.publish<nav_msgs::OccupancyGrid>(*final);

            _ndt_node_pub.publish(_ndt_node_markers);
        }

        _nb_of_zone = acg.getRobotNodes().size();
    }

    void updateRvizOriginalSLAM(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        drawPrior(acg);
        if (_nb_of_zone != acg.getRobotNodes().size()) {
            std::cout << "update the zones" << std::endl;

            for (size_t i = _nb_of_zone; i < acg.getRobotNodes().size(); ++i) {
                nav_msgs::OccupancyGrid* omap_tmp =
                    new nav_msgs::OccupancyGrid();
                perception_oru::toOccupancyGrid(
                    acg.getRobotNodes()[i]->getMap().get(), *omap_tmp,
                    _resolution, "/world");
                auto pose = acg.getRobotNodes()[i]->getPose();
                ROS_DEBUG_STREAM("Move : " << pose.matrix());
                moveOccupancyMap(*omap_tmp, pose);
                omap_tmp->header.frame_id = "/world";
                omap_tmp->header.stamp = ros::Time::now();
                nav_msgs::OccupancyGrid::ConstPtr ptr(omap_tmp);
                grids.push_back(ptr);
            }
        }
        _nb_of_zone = acg.getRobotNodes().size();
        nav_msgs::OccupancyGrid::Ptr final;
        if (grids.size() > 0) {
            final = occupancy_grid_utils::combineGrids(grids);
            final->header.frame_id = "/world";
            final->header.stamp = ros::Time::now();
            _last_ndtmap_occ.publish<nav_msgs::OccupancyGrid>(*final);
        }
    }

   protected:
    void moveOccupancyMap(nav_msgs::OccupancyGrid& occ_grid,
                          const Eigen::Affine3d& pose_vec);
    void moveOccupancyMap(nav_msgs::OccupancyGrid& occ_grid,
                          const Eigen::Affine2d& pose_vec);
    Eigen::Affine3d vector3dToAffine3d(const Eigen::Vector3d& vec) {
        Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1, 1, 2)));
        return t;
    }

    virtual void drawPrior(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
        std::cout << "Not implemented" << std::endl;
    }
    void drawCornersNdt(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg);
    void drawGaussiansThatGaveCorners(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg);
    void drawAngles(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg);
    void drawPriorAngles(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
        const VertexPrior& vertex_in,
        const Eigen::Vector2d& vertex_pose);
    void drawRobotPoses(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg);
    void drawObservations(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg);

    void saveImage(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
        nav_msgs::OccupancyGrid::Ptr& msg);
};

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::
    VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::moveOccupancyMap(
        nav_msgs::OccupancyGrid& occ_grid,
        const Eigen::Affine3d& pose_vec) {
    Eigen::Affine3d map_origin;
    tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
    Eigen::Affine3d new_map_origin = pose_vec * map_origin;
    tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::
    VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::moveOccupancyMap(
        nav_msgs::OccupancyGrid& occ_grid,
        const Eigen::Affine2d& a2d) {
    // Affine 2d to 3d
    double angle = atan2(a2d.rotation()(1, 0), a2d.rotation()(0, 0));
    Eigen::Affine3d pose_effi =
        Eigen::Translation3d(a2d.translation()(0), a2d.translation()(1), 0.) *
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

    Eigen::Affine3d map_origin;
    tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
    Eigen::Affine3d new_map_origin = pose_effi * map_origin;
    tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::
    VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::drawObservations(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
    _observation_edge_markers.header.stamp = ros::Time::now();
    auto edges = acg.getLandmarkEdges();
    if (edges.size() != _observation_edge_markers.points.size()) {
        _observation_edge_markers.points.clear();

        auto it = edges.begin();
        for (it; it != edges.end(); ++it) {
            for (auto ite2 = (*it)->vertices().begin();
                 ite2 != (*it)->vertices().end(); ++ite2) {
                geometry_msgs::Point p;
                g2o::VertexSE2ACG* ptr =
                    dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
                g2o::VertexPointXYACG* ptr2 =
                    dynamic_cast<g2o::VertexPointXYACG*>((*ite2));
                if (ptr != NULL) {
                    auto vertex = ptr->estimate().toVector();
                    // Getting the translation out of the transform :
                    // https://en.wikipedia.org/wiki/Transformation_matrix
                    p.x = vertex(0);
                    p.y = vertex(1);
                    p.z = acg.getZElevation();
                    ROS_DEBUG_STREAM("Found Some pose at " << p.x << " "
                                                           << p.y);
                } else if (ptr2 != NULL) {
                    auto vertex = ptr2->estimate();
                    // Getting the translation out of the transform :
                    // https://en.wikipedia.org/wiki/Transformation_matrix
                    p.x = vertex(0);
                    p.y = vertex(1);
                    p.z = acg.getZElevation();
                    ROS_DEBUG_STREAM("Found landmark at " << p.x << " " << p.y);
                } else {
                    ROS_ERROR_STREAM("Not found ");
                    throw std::runtime_error("Vertex type not found :O");
                }
                _observation_edge_markers.points.push_back(p);
            }
        }
    }
    _observation_edge_pub.publish(_observation_edge_markers);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::
    VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::drawCornersNdt(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
    _corner_ndt_node_markers.header.stamp = ros::Time::now();
    auto edges = acg.getLandmarkNodes();
    if (edges.size() != _corner_ndt_node_markers.points.size()) {
        _corner_ndt_node_markers.points.clear();
        auto it = edges.begin();
        for (it; it != edges.end(); ++it) {
            geometry_msgs::Point p;
            g2o::VertexLandmarkNDT* ptr =
                dynamic_cast<g2o::VertexLandmarkNDT*>((*it));
            auto vertex = ptr->estimate();
            // Getting the translation out of the transform :
            // https://en.wikipedia.org/wiki/Transformation_matrix
            p.x = vertex(0);
            p.y = vertex(1);
            p.z = acg.getZElevation();

            _corner_ndt_node_markers.points.push_back(p);
        }
    }

    drawAngles(acg);

    _corner_ndt_node_pub.publish(_corner_ndt_node_markers);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::
    drawGaussiansThatGaveCorners(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
    _gaussian_that_gave_corners.points.clear();

    _gaussian_that_gave_corners.header.stamp = ros::Time::now();
    _gaussian_that_gave_corners.header.stamp = ros::Time::now();

    auto landmark = acg.getLandmarkNodes();

    for (auto it = landmark.begin(); it != landmark.end(); ++it) {
        auto vertex = (*it)->first_seen_from->estimate().toVector();

        for (auto it_ndt = (*it)->cells_that_gave_it_1.begin();
             it_ndt != (*it)->cells_that_gave_it_1.end(); ++it_ndt) {
            Eigen::Vector3d mean = (*it_ndt)->getMean();
            auto angle =
                perception_oru::ndt_feature_finder::NDTCellAngle(**it_ndt);
            mean(2) = angle;
            Eigen::Vector3d robotframe_eigen;
            translateFromRobotFrameToGlobalFrame(mean, vertex,
                                                 robotframe_eigen);

            geometry_msgs::Point p2;
            p2.x = robotframe_eigen(0) + (0.5 * std::cos(robotframe_eigen(2)));
            p2.y = robotframe_eigen(1) + (0.5 * std::sin(robotframe_eigen(2)));
            p2.z = acg.getZElevation();
            geometry_msgs::Point p3;
            p3.x = robotframe_eigen(0) - (0.5 * std::cos(robotframe_eigen(2)));
            p3.y = robotframe_eigen(1) - (0.5 * std::sin(robotframe_eigen(2)));
            p3.z = acg.getZElevation();

            _gaussian_that_gave_corners.points.push_back(p2);
            _gaussian_that_gave_corners.points.push_back(p3);
        }

        for (auto it_ndt = (*it)->cells_that_gave_it_2.begin();
             it_ndt != (*it)->cells_that_gave_it_2.end(); ++it_ndt) {
            Eigen::Vector3d mean = (*it_ndt)->getMean();
            auto angle =
                perception_oru::ndt_feature_finder::NDTCellAngle(**it_ndt);
            mean(2) = angle;
            Eigen::Vector3d robotframe_eigen;
            translateFromRobotFrameToGlobalFrame(mean, vertex,
                                                 robotframe_eigen);

            geometry_msgs::Point p2;
            p2.x = robotframe_eigen(0) + (0.5 * std::cos(robotframe_eigen(2)));
            p2.y = robotframe_eigen(1) + (0.5 * std::sin(robotframe_eigen(2)));
            p2.z = acg.getZElevation();
            geometry_msgs::Point p3;
            p3.x = robotframe_eigen(0) - (0.5 * std::cos(robotframe_eigen(2)));
            p3.y = robotframe_eigen(1) - (0.5 * std::sin(robotframe_eigen(2)));
            p3.z = acg.getZElevation();

            _gaussian_that_gave_corners2.points.push_back(p2);
            _gaussian_that_gave_corners2.points.push_back(p3);
        }
    }

    _gaussian_pub.publish(_gaussian_that_gave_corners);
    _gaussian_pub2.publish(_gaussian_that_gave_corners2);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::saveImage(
    const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    nav_msgs::OccupancyGrid::Ptr& msg) {
    assert(msg->info.height * msg->info.width == msg->data.size());

    std::cout << "Creating the mat" << std::endl;
    cv::Mat img = cv::Mat(msg->data, true);
    img.rows = msg->info.height;
    img.cols = msg->info.width;

    std::string file_out = _image_file;
    std::ostringstream convert;  // stream used for the conversion
    convert << acg.getGraph().vertices().size();
    file_out = file_out + convert.str();
    file_out = file_out + "nodes.png";

    std::cout << "IMwrite to " << file_out << std::endl;
    cv::imwrite(file_out, img);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void
AASS::acg::VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::drawAngles(
    const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
    _angles_markers.header.stamp = ros::Time::now();
    _anglesw_markers.header.stamp = ros::Time::now();
    auto landmark = acg.getLandmarkNodes();
    if (landmark.size() != _angles_markers.points.size()) {
        _angles_markers.points.clear();
        _anglesw_markers.points.clear();
        auto it = landmark.begin();
        for (it; it != landmark.end(); ++it) {
            for (int i = 0; i < (*it)->getAnglesAndOrientations().size(); ++i) {
                geometry_msgs::Point p;
                auto vertex = (*it)->estimate();
                // Getting the translation out of the transform :
                // https://en.wikipedia.org/wiki/Transformation_matrix
                p.x = vertex(0);
                p.y = vertex(1);
                p.z = acg.getZElevation();
                _angles_markers.points.push_back(p);

                double angle = (*it)->getOrientationGlobal(i);
                double anglew = (*it)->getAngleWidth(i);

                geometry_msgs::Point p2;
                p2.x = p.x + (2 * std::cos(angle));
                p2.y = p.y + (2 * std::sin(angle));
                p2.z = acg.getZElevation();
                _angles_markers.points.push_back(p2);

                p2.x = p.x + (2 * std::cos(angle - (anglew / 2)));
                p2.y = p.y + (2 * std::sin(angle - (anglew / 2)));
                p2.z = acg.getZElevation();
                _anglesw_markers.points.push_back(p);
                _anglesw_markers.points.push_back(p2);

                p2.x = p.x + (2 * std::cos(angle + (anglew / 2)));
                p2.y = p.y + (2 * std::sin(angle + (anglew / 2)));
                p2.z = acg.getZElevation();
                _anglesw_markers.points.push_back(p);
                _anglesw_markers.points.push_back(p2);
            }
        }
    }
    _angles_pub.publish(_angles_markers);
    _anglesw_pub.publish(_anglesw_markers);
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::
    VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::drawPriorAngles(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
        const VertexPrior& vertex_in,
        const Eigen::Vector2d& vertex_pose) {
    geometry_msgs::Point p;
    auto vertex = vertex_pose;
    // Getting the translation out of the transform :
    // https://en.wikipedia.org/wiki/Transformation_matrix
    p.x = vertex(0);
    p.y = vertex(1);
    p.z = acg.getZElevation();

    auto angles = vertex_in.getAnglesAndOrientations();

    for (auto it = angles.begin(); it != angles.end(); ++it) {
        _angles_prior_markers.points.push_back(p);

        double angle = it->second;
        double anglew = it->first;
        geometry_msgs::Point p2;
        p2.x = p.x + (2 * std::cos(angle));
        p2.y = p.y + (2 * std::sin(angle));
        p2.z = acg.getZElevation();
        _angles_prior_markers.points.push_back(p2);

        p2.x = p.x + (2 * std::cos(angle - (anglew / 2)));
        p2.y = p.y + (2 * std::sin(angle - (anglew / 2)));
        p2.z = acg.getZElevation();
        _anglesw_prior_markers.points.push_back(p);
        _anglesw_prior_markers.points.push_back(p2);

        p2.x = p.x + (2 * std::cos(angle + (anglew / 2)));
        p2.y = p.y + (2 * std::sin(angle + (anglew / 2)));
        p2.z = acg.getZElevation();
        _anglesw_prior_markers.points.push_back(p);
        _anglesw_prior_markers.points.push_back(p2);
    }
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void AASS::acg::
    VisuAutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>::drawRobotPoses(
        const AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg) {
    _ndt_node_markers.points.clear();
    _ndt_node_markers.header.stamp = ros::Time::now();

    for (size_t i = 0; i < acg.getRobotNodes().size(); ++i) {
        auto node = acg.getRobotNodes()[i];
        geometry_msgs::Point p;
        auto vertex2 = node->estimate().toVector();
        // Getting the translation out of the transform :
        // https://en.wikipedia.org/wiki/Transformation_matrix
        p.x = vertex2(0);
        p.y = vertex2(1);
        p.z = acg.getZElevation();

        _ndt_node_markers.points.push_back(p);
    }

    _ndt_node_pub.publish(_ndt_node_markers);
}

template <>
inline void AASS::acg::VisuAutoCompleteGraphBase<AutoCompleteGraphPriorSE2,
                                                 g2o::VertexSE2Prior,
                                                 g2o::EdgeSE2Prior_malcolm>::
    drawPrior(const AASS::acg::AutoCompleteGraphBase<AutoCompleteGraphPriorSE2,
                                                     g2o::VertexSE2Prior,
                                                     g2o::EdgeSE2Prior_malcolm>&
                  acg) {
    _prior_edge_markers.header.stamp = ros::Time::now();
    auto edges = acg.getPrior()->getEdges();
    if (edges.size() != _prior_edge_markers.points.size()) {
        _prior_edge_markers.points.clear();

        auto it = edges.begin();
        for (it; it != edges.end(); ++it) {
            for (auto ite2 = (*it)->vertices().begin();
                 ite2 != (*it)->vertices().end(); ++ite2) {
                geometry_msgs::Point p;
                g2o::VertexSE2ACG* ptr =
                    dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
                auto vertex = ptr->estimate().toVector();
                // Getting the translation out of the transform :
                // https://en.wikipedia.org/wiki/Transformation_matrix
                p.x = vertex(0);
                p.y = vertex(1);
                p.z = acg.getZElevation();
                _prior_edge_markers.points.push_back(p);
            }
        }

        auto prior_node = acg.getPrior()->getNodes();
        _prior_node_markers.points.clear();
        _angles_prior_markers.points.clear();
        _anglesw_prior_markers.points.clear();
        _angles_prior_markers.header.stamp = ros::Time::now();
        _anglesw_prior_markers.header.stamp = ros::Time::now();
        auto itt = prior_node.begin();
        for (itt; itt != prior_node.end(); ++itt) {
            geometry_msgs::Point p;
            g2o::VertexSE2Prior* ptr =
                dynamic_cast<g2o::VertexSE2Prior*>((*itt));
            auto vertex = ptr->estimate().toVector();
            // Getting the translation out of the transform :
            // https://en.wikipedia.org/wiki/Transformation_matrix
            p.x = vertex(0);
            p.y = vertex(1);
            p.z = acg.getZElevation();
            _prior_node_markers.points.push_back(p);

            drawPriorAngles(acg, *ptr, vertex.head(2));
        }
    }
    _marker_pub.publish(_prior_edge_markers);
    _prior_node_pub.publish(_prior_node_markers);
    _angles_prior_pub.publish(_angles_prior_markers);
    _anglesw_prior_pub.publish(_anglesw_prior_markers);
}

class VisuAutoCompleteGraph
    : public VisuAutoCompleteGraphBase<AutoCompleteGraphPriorSE2,
                                       g2o::VertexSE2Prior,
                                       g2o::EdgeSE2Prior_malcolm> {
   public:
    VisuAutoCompleteGraph(ros::NodeHandle nh)
        : VisuAutoCompleteGraphBase<AutoCompleteGraphPriorSE2,
                                    g2o::VertexSE2Prior,
                                    g2o::EdgeSE2Prior_malcolm>(nh) {}
};

}  // namespace acg
}  // namespace AASS

#endif
