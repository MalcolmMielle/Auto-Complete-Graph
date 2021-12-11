#ifndef AUTOCOMPLETEGRAPH_OPTIMIZABLEGRAPH_07102016
#define AUTOCOMPLETEGRAPH_OPTIMIZABLEGRAPH_07102016

#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel_impl.h"

namespace AASS
{

	namespace acg
	{

		/*
	 * Working directly on g2o graph
	 */
		class OptimizableAutoCompleteGraph : public g2o::SparseOptimizer
		{

		public:
			typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
			typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

		protected:
			//TODO : test with LevenbergMarquard instead (maybe doesn't scale as aggressively)
			g2o::OptimizationAlgorithmGaussNewton *_solver;
			g2o::SE2 _sensorOffsetTransf;
			g2o::ParameterSE2Offset *_sensorOffset;

		private:
			g2o::RobustKernelPseudoHuber *_huber; //Huber
			g2o::RobustKernelDCS *_dcs;			  //DCS

			bool _first;

		public:
			OptimizableAutoCompleteGraph(const g2o::SE2 &sensoffset) : _sensorOffsetTransf(sensoffset), _first(true)
			{

				auto linearSolver = g2o::make_unique<SlamLinearSolver>();
				linearSolver->setBlockOrdering(false);
				auto blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));
				_solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
				this->setAlgorithm(_solver);

				// add the parameter representing the sensor offset
				_sensorOffset = new g2o::ParameterSE2Offset;
				_sensorOffset->setOffset(_sensorOffsetTransf);
				_sensorOffset->setId(0);
				this->addParameter(_sensorOffset);
				_huber = new g2o::RobustKernelPseudoHuber();
				_dcs = new g2o::RobustKernelDCS();
			};

			//Forbid copy
			OptimizableAutoCompleteGraph(const OptimizableAutoCompleteGraph &that) = delete;

			virtual ~OptimizableAutoCompleteGraph()
			{

				delete _huber;
				delete _dcs;

				this->clear();
				g2o::Factory::destroy();
				g2o::OptimizationAlgorithmFactory::destroy();
			}

			g2o::ParameterSE2Offset *getSensorOffset() { return _sensorOffset; }
			g2o::OptimizationAlgorithmGaussNewton *getSolver() { return _solver; }
			const g2o::SE2 &getSensorOffsetTransfo() { return _sensorOffsetTransf; }

			//TODO : attention -> make sure that the initializeOptimization does not reinitialize the state of the graph as in g2o_viewer
			void initSubset(const g2o::HyperGraph::Edge &egde)
			{
				this->initializeOptimization();
			}

			///@brief Set Marginalized to false and do initializeOptimization
			void prepare()
			{
				//Prepare when changing kernels
				for (SparseOptimizer::VertexIDMap::const_iterator it = vertices().begin(); it != vertices().end(); ++it)
				{
					OptimizableGraph::Vertex *v = static_cast<OptimizableGraph::Vertex *>(it->second);
					v->setMarginalized(false);
				}
				initializeOptimization();
			}

			///ATTENTION Call setHuberKernel are other Kernel first ! It's not in the function here just so that I can use a different kernel function in class that inherite this one. I know that design suck. I should inherite from this. But they some stuff are broken :(
			void optimize(int iter_in = 10)
			{

				int iter = g2o::SparseOptimizer::optimize(iter_in);
				if (iter > 0 && !iter)
				{
					std::cerr << "Optimization failed, result might be invalid" << std::endl;
				}
			}

			//TODO better this
			void setHuberKernel()
			{

				auto idmapedges = this->edges();
				for (auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite)
				{
					OptimizableGraph::Edge *e = static_cast<OptimizableGraph::Edge *>(*ite);
					auto huber = new g2o::RobustKernelHuber();
					e->setRobustKernel(huber);
					e->robustKernel()->setDelta(1);
				}
			}
			void setDCSKernel()
			{
				auto idmapedges = this->edges();
				for (auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite)
				{
					OptimizableGraph::Edge *e = static_cast<OptimizableGraph::Edge *>(*ite);
					auto dcs = new g2o::RobustKernelDCS();
					e->setRobustKernel(dcs);
					e->robustKernel()->setDelta(1);
				}
			}
			void removeRobustKernel()
			{

				//Setting robust kernel
				for (SparseOptimizer::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it)
				{
					OptimizableGraph::Edge *e = static_cast<OptimizableGraph::Edge *>(*it);
					e->setRobustKernel(0);
				}
			}

			/**
		 * @brief Fix a node in the graph before the optimization
		 */
			void setFirst(g2o::OptimizableGraph::Vertex *fRobot)
			{
				fRobot->setFixed(true);
			}

		private:
			void setRobustKernelAllEdges(g2o::RobustKernel *ptr = NULL, double width = 1);
		};
	}
}

#endif
