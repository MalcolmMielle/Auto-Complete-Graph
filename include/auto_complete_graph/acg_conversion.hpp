#ifndef AUTOCOMPLETEGRAPH_ACG_CONVERSION_25112016
#define AUTOCOMPLETEGRAPH_ACG_CONVERSION_25112016

#include <grid_map_msgs/GridMap.h>
#include <tf/transform_datatypes.h>

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "ACGBase.hpp"
#include "auto_complete_graph/ACGMaps.h"
#include "auto_complete_graph/ACGMapsOM.h"
#include "auto_complete_graph/Localization/ACG_localization.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "ndt_map/NDTVectorMapMsg.h"
#include "occupancy_grid_utils/combine_grids.h"

namespace AASS {
namespace acg {

inline bool toGridMap(perception_oru::NDTMap* ndt_map,
                      grid_map::GridMap& map,
                      double resolution,
                      std::string frame_id,
                      std::string layer_name);

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGPriortoGridMap(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    grid_map::GridMap& gridMap,
    double resolution);

template <>
inline void ACGPriortoGridMap(
    const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorSE2,
                                           g2o::VertexSE2RobotPose,
                                           g2o::EdgeSE2Prior_malcolm>& acg,
    grid_map::GridMap& gridMap,
    double resolution);

template <>
inline void ACGPriortoGridMap(
    const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorXY,
                                           g2o::VertexXYPrior,
                                           g2o::EdgeXYPriorACG>& acg,
    grid_map::GridMap& gridMap,
    double resolution);

inline void fuseGridMap(const grid_map::GridMap& src,
                        grid_map::GridMap& target,
                        grid_map::GridMap& out_grid,
                        const std::string& layer = "combined",
                        const std::string& layer2 = "combined",
                        const std::string& final_layer = "combined");

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGNdtNodetoVecGrids(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    std::vector<g2o::VertexSE2RobotPose*>::const_iterator& it_start,
    std::vector<g2o::VertexSE2RobotPose*>::const_iterator& it_end,
    std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids_out);

inline void moveOccupancyMap(nav_msgs::OccupancyGrid& occ_grid,
                             const Eigen::Affine3d& pose_vec) {
    Eigen::Affine3d map_origin;
    tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
    Eigen::Affine3d new_map_origin = pose_vec * map_origin;
    tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}

inline void moveOccupancyMap(nav_msgs::OccupancyGrid& occ_grid,
                             const Eigen::Affine2d& a2d) {
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
inline nav_msgs::OccupancyGrid::Ptr ACGNDTtoOcc(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    nav_msgs::OccupancyGrid::ConstPtr& ptr_prior_occ,
    double resol) {
    std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
    if (acg.getRobotNodes().size() != 0) {
        grids.push_back(ptr_prior_occ);

        std::cout << "update the zones" << acg.getRobotNodes().size()
                  << std::endl;

        for (size_t i = 0; i < acg.getRobotNodes().size(); ++i) {
            // Grid map test
            std::cout << "Node" << std::endl;
            nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();
            perception_oru::toOccupancyGrid(
                acg.getRobotNodes()[i]->getMap().get(), *omap, resol, "/world");
            auto node = acg.getRobotNodes()[i];
            auto vertex = node->estimate().toIsometry();

            std::cout << "Move" << std::endl;
            moveOccupancyMap(*omap, vertex);
            omap->header.frame_id = "/world";
            omap->header.stamp = ros::Time::now();

            nav_msgs::OccupancyGrid::ConstPtr ptr(omap);
            grids.push_back(ptr);
        }
    }

    nav_msgs::OccupancyGrid::Ptr occ_out;
    std::cout << "Building the final thingy " << grids.size() << std::endl;
    if (grids.size() > 0) {
        std::cout << "Combine " << grids.size() << std::endl;
        occ_out = occupancy_grid_utils::combineGrids(grids);
        occ_out->header.frame_id = "/world";
        occ_out->header.stamp = ros::Time::now();
    }
    std::cout << "Out" << std::endl;
    return occ_out;
}

/**
 * @brief This method exports the ACG to an occupancy grid map where the prior
 * and ndt map are one occupancy grid
 *
 * @tparam Prior: the prior map type
 * @tparam VertexPrior: the type of vertex in the prior map
 * @tparam EdgePrior: the type of edges in the prior map
 * @param acg: the ACG graph to convert
 * @param occ_out: the output occupancy map
 * @param start: the first ndt map to export. Defaults to 0.
 * @param end: the last occupancy map to export. Defaults the
 * acg.getRobotNodes().size()
 */
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGtoOccupancyGrid(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    nav_msgs::OccupancyGrid::Ptr& occ_out,
    int start = 0,
    int end = -1) {
    if (end < start && end != -1)
        || (start >= acg.getRobotNodes().size()) {
            throw std::runtime_error(
                "End pointer is before the start. Can draw zones backward in "
                "ACG_CONVERSION.hpp");
        }

    //************* TODO : shorten this code !
    // Get max size of prior
    grid_map::GridMap map;
    auto edges = acg.getPrior()->getEdges();

    double max_x, min_x, max_y, min_y;
    bool flag_init = false;
    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            geometry_msgs::Point p;
            g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
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

    double size_x, size_y;
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

    /***********************************/

    map.setFrameId("/world");
    map.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), 0.1,
                    grid_map::Position(0.0, 0.0));

    map.add("prior");
    map.add("ndt");
    map.add("all");
    map["ndt"].setZero();
    map["all"].setZero();

    ACGPriortoGridMap<Prior, VertexPrior, EdgePrior>(acg, map, 0.1);

    nav_msgs::OccupancyGrid* prior_occ = new nav_msgs::OccupancyGrid();
    nav_msgs::OccupancyGrid::ConstPtr ptr_prior_occ(prior_occ);
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "prior", 0, 1.,
                                                   *prior_occ);

    std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids;
    grids.push_back(ptr_prior_occ);

    if (acg.getRobotNodes().size() != 0) {
        std::vector<g2o::VertexSE2RobotPose*>::const_iterator it =
            acg.getRobotNodes().begin() + start;
        std::vector<g2o::VertexSE2RobotPose*>::const_iterator it_end =
            acg.getRobotNodes().end();

        if (end != -1) {
            it_end = acg.getRobotNodes().begin() + end;
        }

        ACGNdtNodetoVecGrids(acg, it, it_end, grids);
    }

    if (grids.size() > 0) {
        occ_out = occupancy_grid_utils::combineGrids(grids);
        occ_out->header.frame_id = "/world";
        occ_out->header.stamp = ros::Time::now();
    }
}

/**
 * @brief Write all the submap in `acg` into a vector of occupancy grids
 *
 * @tparam Prior: the prior map type
 * @tparam VertexPrior: the type of vertex in the prior map
 * @tparam EdgePrior: the type of edges in the prior map
 * @param acg: the ACG graph to convert
 * @param it_start: the first ndt map to export.
 * @param it_end: the last occupancy map to export.
 * @param grids_out: the output vector of grids
 */
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGNdtNodetoVecGrids(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    std::vector<g2o::VertexSE2RobotPose*>::const_iterator& it_start,
    std::vector<g2o::VertexSE2RobotPose*>::const_iterator& it_end,
    std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids_out) {
    for (it_start; it_start != it_end; ++it_start) {
        nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();

        assert(
            (*it_start)->getMap()->getAllInitializedCellsShared().size() ==
            (*it_start)->getMap().get()->getAllInitializedCellsShared().size());

        assert((*it_start)->getMap()->getAllCellsShared().size() ==
               (*it_start)->getMap().get()->getAllCellsShared().size());

        perception_oru::toOccupancyGrid((*it_start)->getMap().get(), *omap, 0.3,
                                        "/world");
        auto node = *it_start;
        auto vertex = node->estimate().toIsometry();

        moveOccupancyMap(*omap, vertex);
        omap->header.frame_id = "/world";
        omap->header.stamp = ros::Time::now();

        nav_msgs::OccupancyGrid::ConstPtr ptr(omap);
        grids_out.push_back(ptr);
    }
}

template <typename Prior>
inline void ACGPriorToNDTMap(const Prior& acg,
                             perception_oru::NDTMap& map_out,
                             double z_elevation,
                             double pt_cloud_resolution) {
    auto pcl_prior = acg.toPointCloud(pt_cloud_resolution, z_elevation,
                                      pt_cloud_resolution / 4);

    map_out.loadPointCloud(*pcl_prior);
    map_out.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    auto allcells = map_out.getAllCellsShared();
    auto allcellsinit = map_out.getAllInitializedCellsShared();
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGPriortoGridMap(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    grid_map::GridMap& gridMap,
    double resolution) {
    throw std::runtime_error("DO NOT USE TEMPLATE of prior to grid map");
}

template <>
inline void ACGPriortoGridMap(
    const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorSE2,
                                           g2o::VertexSE2RobotPose,
                                           g2o::EdgeSE2Prior_malcolm>& acg,
    grid_map::GridMap& gridMap,
    double resolution) {
    auto edges = acg.getPrior()->getEdges();

    gridMap.add("prior");
    gridMap["prior"].setZero();

    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        std::vector<Eigen::Vector2d> points;

        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            geometry_msgs::Point p;
            g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
            auto vertex = ptr->estimate().toVector();
            // Getting the translation out of the transform :
            // https://en.wikipedia.org/wiki/Transformation_matrix
            Eigen::Vector2d veve;
            veve << vertex(0), vertex(1);
            points.push_back(veve);
        }

        assert(points.size() == 2);

        for (grid_map::LineIterator iterator(gridMap, points[0], points[1]);
             !iterator.isPastEnd(); ++iterator) {
            gridMap.at("prior", *iterator) = 100;
        }
    }
}

template <>
inline void ACGPriortoGridMap(
    const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorXY,
                                           g2o::VertexXYPrior,
                                           g2o::EdgeXYPriorACG>& acg,
    grid_map::GridMap& gridMap,
    double resolution) {
    auto edges = acg.getPrior()->getEdges();

    gridMap.add("prior");
    gridMap["prior"].setZero();

    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        std::vector<Eigen::Vector2d> points;

        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            geometry_msgs::Point p;
            g2o::VertexXYPrior* ptr =
                dynamic_cast<g2o::VertexXYPrior*>((*ite2));
            auto vertex = ptr->estimate();
            // Getting the translation out of the transform :
            // https://en.wikipedia.org/wiki/Transformation_matrix
            Eigen::Vector2d veve;
            veve << vertex(0), vertex(1);
            points.push_back(veve);
        }

        assert(points.size() == 2);

        for (grid_map::LineIterator iterator(gridMap, points[0], points[1]);
             !iterator.isPastEnd(); ++iterator) {
            gridMap.at("prior", *iterator) = 100;
        }
    }
}

///@brief return the biggest absolute value along x and y for the prior.
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void getPriorSizes(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    double& size_x,
    double& size_y) {
    throw std::runtime_error("do not use templated version of prior sizes");
}

template <>
inline void getPriorSizes(
    const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorXY,
                                           g2o::VertexXYPrior,
                                           g2o::EdgeXYPriorACG>& acg,
    double& size_x,
    double& size_y) {
    auto edges = acg.getPrior()->getEdges();

    double max_x, min_x, max_y, min_y;
    bool flag_init = false;
    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            geometry_msgs::Point p;
            g2o::VertexXYPrior* ptr =
                dynamic_cast<g2o::VertexXYPrior*>((*ite2));
            assert(ptr != NULL);
            auto vertex = ptr->estimate();
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

template <>
inline void getPriorSizes(
    const AASS::acg::AutoCompleteGraphBase<AASS::acg::AutoCompleteGraphPriorSE2,
                                           g2o::VertexSE2RobotPose,
                                           g2o::EdgeSE2Prior_malcolm>& acg,
    double& size_x,
    double& size_y) {
    auto edges = acg.getPrior()->getEdges();

    double max_x, min_x, max_y, min_y;
    bool flag_init = false;
    auto it = edges.begin();
    for (it; it != edges.end(); ++it) {
        for (auto ite2 = (*it)->vertices().begin();
             ite2 != (*it)->vertices().end(); ++ite2) {
            geometry_msgs::Point p;
            g2o::VertexSE2ACG* ptr = dynamic_cast<g2o::VertexSE2ACG*>((*ite2));
            assert(ptr != NULL);
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

/**
 *
 * \brief builds ocuupancy grid message
 * \details Builds 2D occupancy grid map based on 2D NDTMap
 * @param[in] ndt_map 2D ndt map to conversion
 * @param[out] occ_grid 2D cost map
 * @param[in] resolution desired resolution of occupancy map
 * @param[in] name of cooridnation frame for the map (same as the NDT map has)
 *
 */
inline bool toGridMap(perception_oru::NDTMap* ndt_map,
                      grid_map::GridMap& map,
                      double resolution,
                      std::string frame_id,
                      std::string layer_name) {  // works only for 2D case
    double size_x, size_y, size_z;
    int size_x_cell_count, size_y_cell_count;
    double cen_x, cen_y, cen_z;
    double orig_x, orig_y;
    ndt_map->getGridSizeInMeters(size_x, size_y, size_z);
    ndt_map->getCentroid(cen_x, cen_y, cen_z);
    orig_x = cen_x - size_x / 2.0;
    orig_y = cen_y - size_y / 2.0;
    size_x_cell_count = int(size_x / resolution);
    size_y_cell_count = int(size_y / resolution);

    map.setFrameId(frame_id);
    if (map.getSize()(0) == 0 && map.getSize()(1) == 0) {
        std::cerr << "Init the grid map" << std::endl;
        map.setGeometry(grid_map::Length(size_x_cell_count, size_y_cell_count),
                        resolution, grid_map::Position(orig_x, orig_y));
    }
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        map.getPosition(*it, position);

        pcl::PointXYZ pt(position(0), position(1), 0);
        perception_oru::NDTCell* cell;
        if (!ndt_map->getCellAtPoint(pt, cell)) {
            map.at(layer_name, *it) = 0;
        } else if (cell == NULL) {
            map.at(layer_name, *it) = 0;
        } else {
            Eigen::Vector3d vec(pt.x, pt.y, pt.z);
            vec = vec - cell->getMean();
            double likelihood = vec.dot(cell->getInverseCov() * vec);
            char s_likelihood;
            if (cell->getOccupancy() != 0.0) {
                if (cell->getOccupancy() > 0.0) {
                    // 						if(std::isnan(likelihood))
                    // s_likelihood = -1;
                    if (std::isnan(likelihood))
                        s_likelihood = 0;
                    likelihood = exp(-likelihood / 2.0) + 0.1;
                    likelihood = (0.5 + 0.5 * likelihood);
                    s_likelihood = char(likelihood * 100.0);
                    if (likelihood > 1.0)
                        s_likelihood = 100;
                    map.at(layer_name, *it) = s_likelihood;
                } else {
                    map.at(layer_name, *it) = 0;
                }
            } else {
                map.at(layer_name, *it) = 0;
            }
        }
    }

    return true;
}

/**
 * @brief fuse src in target
 * For now only the best value of of both map is kept in the final map.
 * TODO : The grid map need their centers to be at the exact same physical spot.
 * As trump would say : _BAD_
 */
inline void fuseGridMap(const grid_map::GridMap& src,
                        grid_map::GridMap& target,
                        grid_map::GridMap& out_grid,
                        const std::string& layer,
                        const std::string& layer2,
                        const std::string& final_layer) {
    assert(target.exists(layer2) == true);
    assert(src.exists(layer) == true);

    grid_map::GridMap modifiedMap;

    modifiedMap = src;
    grid_map::Matrix& data_targ = target[layer2];
    grid_map::Matrix& data_src = modifiedMap[layer];
    auto t_size = target.getSize();
    auto s_size = modifiedMap.getSize();

    auto pos_targ = target.getPosition();
    auto pos_src = src.getPosition();

    int max_x, max_y;
    max_x = std::max(t_size(0), s_size(0));
    max_y = std::max(t_size(1), s_size(1));

    out_grid.setFrameId(target.getFrameId());
    // +1 because of non even numbers.
    double res = target.getResolution();
    out_grid.setGeometry(grid_map::Length((max_x * res) + 1, (max_y * res) + 1),
                         target.getResolution(), target.getPosition());
    out_grid.add(final_layer);
    out_grid[final_layer].setZero();

    cv::Mat originalImageP;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
        out_grid, final_layer, CV_16UC1, 0.0, 1, originalImageP);
    cv::imwrite("/home/malcolm/outgrid_tmp.png", originalImageP);

    grid_map::Matrix& out = out_grid[final_layer];
    out.setZero();

    int t1 = t_size(0);
    int t2 = t_size(1);
    int s1 = s_size(0);
    int s2 = s_size(1);

    out.block((max_x - t_size(0)) / 2, (max_y - t_size(1)) / 2, t1, t2) =
        data_targ;

    cv::Mat originalImageP1;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
        out_grid, final_layer, CV_16UC1, 0.0, 1, originalImageP1);
    cv::imwrite("/home/malcolm/outgrid_prior.png", originalImageP1);

    out.block((max_x - s_size(0)) / 2, (max_y - s_size(1)) / 2, s1, s2) =
        out.block((max_x - s_size(0)) / 2, (max_y - s_size(1)) / 2, s1, s2)
            .cwiseMax(data_src);

    cv::Mat originalImageP2;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
        out_grid, final_layer, CV_16UC1, 0.0, 1, originalImageP2);
    cv::imwrite("/home/malcolm/outgrid_all_tog.png", originalImageP2);
}

inline void fuseGridMap(std::vector<grid_map::GridMap>& maps,
                        std::vector<std::string>& layers,
                        grid_map::GridMap& combinedGrid,
                        const std::string& frame_id,
                        double resolution) {
    double size_x = -1, size_y = -1;
    for (auto it = maps.begin(); it != maps.end(); ++it) {
        if (size_x < it->getLength()(0))
            size_x = it->getLength()(0);
        if (size_y < it->getLength()(1))
            size_y = it->getLength()(1);
    }

    combinedGrid.setFrameId(frame_id);
    combinedGrid.setGeometry(grid_map::Length(size_x, size_y), resolution);
    combinedGrid.add("combined");
    grid_map::Matrix& data_targ = combinedGrid["combined"];
    data_targ.setZero();

    int i = 0;
    for (auto it = maps.begin(); it != maps.end(); ++it) {
        fuseGridMap(*it, combinedGrid, combinedGrid, layers[i]);
        ++i;
    }
    assert(combinedGrid.getSize()(0) == combinedGrid["combined"].rows());
    assert(combinedGrid.getSize()(1) == combinedGrid["combined"].cols());
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGToVectorMaps(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    ndt_map::NDTVectorMapMsg& maps) {
    if (acg.getRobotNodes().size() != 0) {
        for (auto it = acg.getRobotNodes().begin();
             it != acg.getRobotNodes().end(); ++it) {
            ndt_map::NDTMapMsg msg;
            bool good =
                perception_oru::toMessage((*it)->getMap().get(), msg, "/world");

            maps.maps.push_back(msg);

            auto pose = (*it)->estimate().toVector();
            geometry_msgs::Pose geo_pose;
            geo_pose.position.x = pose(0);
            geo_pose.position.y = pose(1);
            geo_pose.position.z = acg.getZElevation();

            auto quat = tf::createQuaternionFromRPY(0, 0, pose(2));
            geo_pose.orientation.x = quat.getX();
            geo_pose.orientation.y = quat.getY();
            geo_pose.orientation.z = quat.getZ();
            geo_pose.orientation.w = quat.getW();

            maps.poses.push_back(geo_pose);
        }
    }
    if (acg.getOdometryEdges().size() != 0 && acg.getRobotNodes().size() >= 2) {
        geometry_msgs::Transform trans;
        trans.translation.x = 0;
        trans.translation.y = 0;
        trans.translation.z = 0;

        auto quat = tf::createQuaternionFromRPY(0, 0, 0);
        trans.rotation.x = quat.getX();
        trans.rotation.y = quat.getY();
        trans.rotation.z = quat.getZ();
        trans.rotation.w = quat.getW();

        maps.transformations.push_back(trans);

        for (auto odom : acg.getOdometryEdges()) {
            geometry_msgs::Transform trans;
            trans.translation.x = odom->measurement().toVector()(0);
            trans.translation.y = odom->measurement().toVector()(1);
            trans.translation.z = 0;

            auto quat = tf::createQuaternionFromRPY(
                0, 0, odom->measurement().toVector()(2));
            trans.rotation.x = quat.getX();
            trans.rotation.y = quat.getY();
            trans.rotation.z = quat.getZ();
            trans.rotation.w = quat.getW();

            maps.transformations.push_back(trans);
        }
    }
}

///@brief transform the ACG into a message including a NDTVectorMapMsg
/// representing all submaps and the transof between them AND the prior
/// represented by grid centered on the origin frame
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGToACGMapsMsg(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    auto_complete_graph::ACGMaps& mapmsg,
    const std::string& frame_id = "/world") {
    mapmsg.header.stamp = ros::Time::now();
    mapmsg.header.frame_id = frame_id;

    ndt_map::NDTVectorMapMsg maps;
    ACGToVectorMaps(acg, mapmsg.ndt_maps);

    perception_oru::NDTMap* ndt_prior =
        new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
    AASS::acg::ACGPriorToNDTMap<AASS::acg::AutoCompleteGraphPriorXY>(
        *acg.getPrior(), *ndt_prior, acg.getZElevation(), 0.1);

    ndt_map::NDTMapMsg mapmsgndt;
    perception_oru::toMessage(ndt_prior, mapmsgndt, frame_id);

    mapmsg.prior = mapmsgndt;
}

template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGToOccMaps(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    auto_complete_graph::ACGMapsOM& mapmsg,
    double resolution = 0.1,
    const std::string& frame_id = "/world",
    double ndt_cell_gaussian_scaling = 0.1,
    bool euclidean_dist_occ_map = false) {
    for (auto it = acg.getRobotNodes().begin(); it != acg.getRobotNodes().end();
         ++it) {
        nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();
        perception_oru::toOccupancyGrid(
            (*it)->getMap().get(), *omap, resolution, frame_id,
            ndt_cell_gaussian_scaling, euclidean_dist_occ_map);

        grid_map::GridMap mapNDT;
        // THis ruin prior because they are of different sizes ! Need my custom
        // fuse function :)
        grid_map::GridMapRosConverter::fromOccupancyGrid(*omap, "ndt", mapNDT);
        delete omap;

        grid_map::GridMapRosConverter converter;
        grid_map_msgs::GridMap gridmapmsg;
        converter.toMessage(mapNDT, gridmapmsg);

        mapmsg.ndt_maps_om.push_back(gridmapmsg);

        auto pose = (*it)->estimate().toVector();
        geometry_msgs::Transform transform;
        transform.translation.x = pose(0);
        transform.translation.y = pose(1);
        transform.translation.z = 0;

        auto quat = tf::createQuaternionFromRPY(0, 0, pose(2));
        transform.rotation.x = quat.getX();
        transform.rotation.y = quat.getY();
        transform.rotation.z = quat.getZ();
        transform.rotation.w = quat.getW();

        mapmsg.robot_poses.push_back(transform);
    }
}

///@brief transform the ACG into a message including a NDTVectorMapMsg
/// representing all submaps and the transof between them AND the prior
/// represented by grid centered on the origin frame
template <typename Prior, typename VertexPrior, typename EdgePrior>
inline void ACGToACGMapsOMMsg(
    const AASS::acg::AutoCompleteGraphBase<Prior, VertexPrior, EdgePrior>& acg,
    auto_complete_graph::ACGMapsOM& mapmsg,
    const std::string& frame_id = "/world",
    double resolution = 0.1,
    double ndt_cell_gaussian_scaling = 0.05,
    bool euclidean_dist_occ_map = false) {
    mapmsg.header.stamp = ros::Time::now();
    mapmsg.header.frame_id = frame_id;

    ACGToOccMaps(acg, mapmsg, resolution, frame_id, ndt_cell_gaussian_scaling,
                 euclidean_dist_occ_map);

    grid_map::GridMap gridMap;
    gridMap.setFrameId(frame_id);
    double size_x, size_y;
    getPriorSizes(acg, size_x, size_y);
    gridMap.setGeometry(grid_map::Length(4 * size_x, 4 * size_y), resolution,
                        grid_map::Position(0.0, 0.0));
    gridMap.add("prior");
    gridMap["prior"].setZero();
    ACGPriortoGridMap(acg, gridMap, resolution);
    grid_map::GridMapRosConverter converter;
    grid_map_msgs::GridMap gridmapmsg;
    converter.toMessage(gridMap, mapmsg.prior);
}

}  // namespace acg
}  // namespace AASS

#endif
