#ifndef NDTFEATURE_OPTIMIZER_07102016
#define NDTFEATURE_OPTIMIZER_07102016


#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel_impl.h"

namespace AASS {

namespace acg{	
	
	
	class OptimizableAutoCompleteGraph : public g2o::SparseOptimizer{
		
	public :
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	protected:
// 		g2o::SparseOptimizer _optimizer;
		SlamLinearSolver* _linearSolver;
		SlamBlockSolver* _blockSolver;
		g2o::OptimizationAlgorithmGaussNewton* _solver;
		g2o::SE2 _sensorOffsetTransf;
		g2o::ParameterSE2Offset* _sensorOffset;
		
// 		AutoCompleteGraph* _graph;
		
	private:
		g2o::RobustKernelPseudoHuber* _huber; //Huber
		g2o::RobustKernelDCS* _dcs; //DCS
		
	public:
		
		OptimizableAutoCompleteGraph(const g2o::SE2& sensoffset
// 				const Eigen::Vector2d& tn, 
// 				double rn,
// 				const Eigen::Vector2d& ln,
// 				const Eigen::Vector2d& pn,
// 				double rp,
// 				const Eigen::Vector2d& linkn
									) : _sensorOffsetTransf(sensoffset){
			
			_linearSolver = new SlamLinearSolver();
			_linearSolver->setBlockOrdering(false);
			_blockSolver = new SlamBlockSolver(_linearSolver);
			_solver = new g2o::OptimizationAlgorithmGaussNewton(_blockSolver);
			_linearSolver->setBlockOrdering(false);
			this->setAlgorithm(_solver);
			
			

			// add the parameter representing the sensor offset ATTENTION was ist das ?
			_sensorOffset = new g2o::ParameterSE2Offset;
			_sensorOffset->setOffset(_sensorOffsetTransf);
			_sensorOffset->setId(0);
			this->addParameter(_sensorOffset);
			
// 			_graph = new AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn);
// 			this->addGraph(_graph);
			_huber = new g2o::RobustKernelPseudoHuber();
			_dcs = new g2o::RobustKernelDCS();
			
		};
		
		//TODO
		///@brief Does not copy pointer :)!
// 		OptimizableAutoCompleteGraph(const OptimizableAutoCompleteGraph& opt){
// 			_optimizer = opt.getoptimizer;
// 			_linearSolver = opt.getLinearSolver();
// 			_blockSolver = opt.getBlockSolver;
// 			_solver = opt.getSolver;
// 			_sensorOffsetTransf = opt.getSensorOffsetTransfo();
// 			_sensorOffset = opt.getSensorOffset();
// // 			_graph = opt.getGraph();
// 		}
		
		~OptimizableAutoCompleteGraph(){
// 			delete _graph;
			delete _sensorOffset;
			delete _blockSolver;
			delete _solver;
			delete _linearSolver;
			delete _huber;
			delete _dcs;
			
			//TODO : not sure about that
			g2o::Factory::destroy();
			g2o::OptimizationAlgorithmFactory::destroy();
			g2o::HyperGraphActionLibrary::destroy();
		}
		
// 		AutoCompleteGraph* getGraph(){return _graph;}
		g2o::ParameterSE2Offset* getSensorOffset(){return _sensorOffset;}
		SlamLinearSolver* getLinearSolver(){ return _linearSolver;}
		SlamBlockSolver* getBlockSolver(){ return _blockSolver;}
// 		const g2o::SparseOptimizer& getoptimizer(){return _optimizer;}
		g2o::OptimizationAlgorithmGaussNewton* getSolver() {return _solver;}
		const g2o::SE2& getSensorOffsetTransfo(){return _sensorOffsetTransf;}
		
		///@brief Init
		void init(){
			this->initializeOptimization();
		}
		
		//TODO : attention -> make sure that the initializeOptimization does not reinitialize the state of the graph as in g2o_viewer
		void initSubset(){
			
		}
		
		///@brief Optimization process
		void optimize(){
			this->setHuberKernel();
			g2o::SparseOptimizer::optimize(40);
			this->setDCSKernel();
			g2o::SparseOptimizer::optimize(5);			
		}
		
		void setHuberKernel(){
			setRobustKernelAllEdges(_huber);
		}
		void setDCSKernel(){
			setRobustKernelAllEdges(_dcs);
		}
		
	private:
		
		void setRobustKernelAllEdges(g2o::RobustKernel* ptr){
			g2o::RobustKernelPseudoHuber huber; //Huber
			auto idmapedges = this->edges();
			for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
				(dynamic_cast<g2o::OptimizableGraph::Edge*>(*ite))->setRobustKernel(ptr);
			}	
		}
		
	};
}
}

#endif