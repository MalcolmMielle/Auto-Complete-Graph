#ifndef NDTFEATURE_OPTIMIZER_07102016
#define NDTFEATURE_OPTIMIZER_07102016

#include "ACG.hpp"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

namespace AASS {

namespace acg{	
	
	
	class Optimizer{
	public :
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	protected:
		g2o::SparseOptimizer _optimizer;
		SlamLinearSolver* _linearSolver;
		SlamBlockSolver* _blockSolver;
		g2o::OptimizationAlgorithmGaussNewton* _solver;
		g2o::SE2 _sensorOffsetTransf;
		g2o::ParameterSE2Offset* _sensorOffset;
		
		AutoCompleteGraph* _graph;
		
	public:
		
		Optimizer(const g2o::SE2& sensoffset,
				const Eigen::Vector2d& tn, 
				double rn,
				const Eigen::Vector2d& ln,
				const Eigen::Vector2d& pn,
				double rp,
				const Eigen::Vector2d& linkn) : _sensorOffsetTransf(sensoffset){
			
			_linearSolver = new SlamLinearSolver();
			_linearSolver->setBlockOrdering(false);
			_blockSolver = new SlamBlockSolver(_linearSolver);
			_solver = new g2o::OptimizationAlgorithmGaussNewton(_blockSolver);
			_linearSolver->setBlockOrdering(false);
			_optimizer.setAlgorithm(_solver);

			// add the parameter representing the sensor offset ATTENTION was ist das ?
			_sensorOffset = new g2o::ParameterSE2Offset;
			_sensorOffset->setOffset(_sensorOffsetTransf);
			_sensorOffset->setId(0);
			_optimizer.addParameter(_sensorOffset);
			
			_graph = new AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn);
			
			_optimizer.addGraph(_graph);
			
		};
		
		///@brief Does not copy pointer :)!
		Optimizer(const Optimizer& opt){
			_optimizer = opt.getoptimizer;
			_linearSolver = opt.getLinearSolver();
			_blockSolver = opt.getBlockSolver;
			_solver = opt.getSolver;
			_sensorOffsetTransf = opt.getSensorOffsetTransfo();
			_sensorOffset = opt.getSensorOffset();
			_graph = opt.getGraph();
		}
		
		~Optimizer(){
			delete _graph;
			delete _sensorOffset;
			delete _blockSolver;
			delete _solver;
			delete _linearSolver;
			
			//TODO : not sure about that
			Factory::destroy();
			OptimizationAlgorithmFactory::destroy();
			HyperGraphActionLibrary::destroy();
		}
		
		AutoCompleteGraph* getGraph(){return _graph;}
		g2o::ParameterSE2Offset* getSensorOffset(){return _sensorOffset;}
		SlamLinearSolver* getLinearSolver(){ return _linearSolver;}
		SlamBlockSolver* getBlockSolver(){ return _blockSolver;}
		const g2o::SparseOptimizer& getoptimizer(){return _optimizer;}
		g2o::OptimizationAlgorithmGaussNewton* getSolver() {return _solver;}
		const g2o::SE2& getSensorOffsetTransfo(){return _sensorOffsetTransf;}
		
		optimize(){
			_optimizer.initializeOptimization();
			_optimizer.optimize(10);
		}
		
	};
}
}

#endif