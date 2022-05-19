#ifndef AUTOCOMPLETEGRAPH_ACGLOGGER_01062017
#define AUTOCOMPLETEGRAPH_ACGLOGGER_01062017

#include "ACGBase.hpp"

namespace AASS {
namespace acg {

class AutoCompleteGraphLogger : public AutoCompleteGraph {
   protected:
    std::string _file;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AutoCompleteGraphLogger(const std::string& logfile,
                            const g2o::SE2& sensoffset,
                            const std::string& load_file)
        : AutoCompleteGraph(sensoffset, load_file), _file(logfile) {}

    AutoCompleteGraphLogger(const std::string& logfile,
                            const g2o::SE2& sensoffset,
                            const Eigen::Vector2d& tn,
                            double rn,
                            const Eigen::Vector2d& ln,
                            const Eigen::Vector2d& pn,
                            double rp,
                            const Eigen::Vector2d& linkn,
                            ndt_feature::NDTFeatureGraph* ndt_graph)
        : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn, ndt_graph),
          _file(logfile) {}

    AutoCompleteGraphLogger(const std::string& logfile,
                            const g2o::SE2& sensoffset,
                            const Eigen::Vector2d& tn,
                            double rn,
                            const Eigen::Vector2d& ln,
                            const Eigen::Vector2d& pn,
                            double rp,
                            const Eigen::Vector2d& linkn)
        : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn),
          _file(logfile) {}

    void log() {
        std::ofstream out(_file.c_str(),
                          std::ios::in | std::ios::out | std::ios::ate);
        out << "New step"
            << "\n\n";
        out.close();

        for (auto it = _nodes_ndt.begin(); it != _nodes_ndt.end(); ++it) {
            auto vec2d = (*it)->estimate().toVector();
            ros::Time time((*it)->getTime());
            logT(vec2d, time);
        }
    }

    void logT(const Eigen::Vector3d& T_out, ros::Time time) {
        std::cout << "Log T " << _file << std::endl;
        std::ofstream out(_file.c_str(),
                          std::ios::in | std::ios::out | std::ios::ate);

        std::cout << T_out(0) << " " << T_out(1) << " " << T_out(2) << " "
                  << time << std::endl;
        out << T_out(0) << " " << T_out(1) << " " << T_out(2) << " " << time
            << "\n";
        out.close();
    }
};
}  // namespace acg
}  // namespace AASS

#endif