#ifndef AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018
#define AUTOCOMPLETEGRAPH_VERTEXNDTCELL_02072018

#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeXYPrior.hpp"
#include "VertexPointXYACG.hpp"
#include "ndt_map/ndt_cell.h"

namespace g2o {

class VertexNDTCell : public g2o::VertexPointXYACG {
   protected:
    boost::shared_ptr<perception_oru::NDTCell> _cell;
    std::set<VertexNDTCell*> _equivalent_cells;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexNDTCell() : g2o::VertexPointXYACG() {}

    boost::shared_ptr<perception_oru::NDTCell>& getCell() { return _cell; }
    const boost::shared_ptr<perception_oru::NDTCell>& getCell() const {
        return _cell;
    }
    std::set<VertexNDTCell*>& getEquivalentNDTCells() {
        return _equivalent_cells;
    }
    const std::set<VertexNDTCell*>& getEquivalentNDTCells() const {
        return _equivalent_cells;
    }
    void setCell(const boost::shared_ptr<perception_oru::NDTCell>& cell) {
        _cell = cell;
    }
};
}  // namespace g2o

#endif