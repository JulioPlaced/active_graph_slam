/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

#include "G2OSolver.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <open_karto/Karto.h>
#include <ros/console.h>
#include <Eigen/StdVector>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;

// typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

G2OSolver::G2OSolver()
{
  // Initialize the SparseOptimizer
   auto linearSolver = g2o::make_unique<SlamLinearSolver>();

  linearSolver->setBlockOrdering(false);

  // auto blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));
  // optimizer_.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver)));

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
     g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
  optimizer_.setAlgorithm(solver);

  latestNodeID_ = 0;

  useRobustKernel_ = true;
}

G2OSolver::~G2OSolver()
{
  // destroy all the singletons
  g2o::Factory::destroy();

  g2o::OptimizationAlgorithmFactory::destroy();

  g2o::HyperGraphActionLibrary::destroy();
}

void G2OSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector& G2OSolver::GetCorrections() const
{
  return corrections_;
}

void G2OSolver::Compute()
{
  corrections_.clear();

  // Fix the first node in the graph to hold the map in place
  g2o::OptimizableGraph::Vertex* first = optimizer_.vertex(0);

  if(!first)
  {
    ROS_ERROR("[g2o] No Node with ID 0 found!");
    return;
  }

  first->setFixed(true);

  // Do the graph optimization
  optimizer_.initializeOptimization();

  optimizer_.setVerbose(true);
  ROS_INFO("[g2o] Optimization started.");
  int iter = optimizer_.optimize(2);

  if (iter > 0)
  {
    ROS_INFO("[g2o] Optimization finished after %d iterations.", iter);
  }
  else
  {
    ROS_ERROR("[g2o] Optimization failed, result might be invalid!");
    return;
  }

  // Write the result so it can be used by the mapper
  g2o::SparseOptimizer::VertexContainer nodes = optimizer_.activeVertices();

  for (g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n != nodes.end(); n++)
  {

    double estimate[3];

    if((*n)->getEstimateData(estimate))
    {
      karto::Pose2 pose(estimate[0], estimate[1], estimate[2]);

      corrections_.push_back(std::make_pair((*n)->id(), pose));
    }
    else
    {
      ROS_ERROR("[g2o] Could not get estimated pose from Optimizer!");
    }
  }
}

void G2OSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{

  karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();

  g2o::VertexSE2* poseVertex = new g2o::VertexSE2;

  poseVertex->setEstimate(g2o::SE2(odom.GetX(), odom.GetY(), odom.GetHeading()));

  poseVertex->setId(pVertex->GetObject()->GetUniqueId());

  optimizer_.addVertex(poseVertex);

  latestNodeID_ = pVertex->GetObject()->GetUniqueId();

  ROS_DEBUG("[g2o] Adding node %d.", pVertex->GetObject()->GetUniqueId());

}

void G2OSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
  // Create a new edge
  g2o::EdgeSE2* odometry = new g2o::EdgeSE2;

  // Set source and target
  int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();

  int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();

  odometry->vertices()[0] = optimizer_.vertex(sourceID);

  odometry->vertices()[1] = optimizer_.vertex(targetID);

  if(odometry->vertices()[0] == NULL)
  {

    ROS_ERROR("[g2o] Source vertex with id %d does not exist!", sourceID);

    delete odometry;

    return;
  }
  if(odometry->vertices()[0] == NULL)
  {

    ROS_ERROR("[g2o] Target vertex with id %d does not exist!", targetID);

    delete odometry;

    return;
  }

  // Set the measurement (odometry distance between vertices)
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();

  g2o::SE2 measurement(diff.GetX(), diff.GetY(), diff.GetHeading());

  odometry->setMeasurement(measurement);

  // Set the covariance of the measurement
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  // std::cout << precisionMatrix << '\n';

  Eigen::Matrix<double,3,3> info;

  info(0,0) = precisionMatrix(0,0);

  info(0,1) = info(1,0) = precisionMatrix(0,1);

  info(0,2) = info(2,0) = precisionMatrix(0,2);

  info(1,1) = precisionMatrix(1,1);

  info(1,2) = info(2,1) = precisionMatrix(1,2);

  info(2,2) = precisionMatrix(2,2);

  odometry->setInformation(info);
  // std::cout << info << '\n';

  if(useRobustKernel_)
  {

    g2o::RobustKernelDCS* rk = new g2o::RobustKernelDCS;

    odometry->setRobustKernel(rk);

  }

  // Add the constraint to the optimizer
  ROS_DEBUG("[g2o] Adding Edge from node %d to node %d.", sourceID, targetID);

  optimizer_.addEdge(odometry);

}

void G2OSolver::getGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges)
{
  using namespace g2o;

  //HyperGraph::VertexIDMap vertexMap = optimizer_.vertices();

  //HyperGraph::EdgeSet edgeSet = optimizer_.edges();

  double *data = new double[3];

  for (SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it)
  {

    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);

    if(v)
    {

      v->getEstimateData(data);

      Eigen::Vector2d pose(data[0], data[1]);

      nodes.push_back(pose);

    }
  }

  double *data1 = new double[3];

  double *data2 = new double[3];

  for (SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it)
  {

    EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);

    if(e)
    {

      VertexSE2* v1 = dynamic_cast<VertexSE2*>(e->vertices()[0]);

      v1->getEstimateData(data1);

      Eigen::Vector2d poseFrom(data1[0], data1[1]);

      VertexSE2* v2 = dynamic_cast<VertexSE2*>(e->vertices()[1]);

      v2->getEstimateData(data2);

      Eigen::Vector2d poseTo(data2[0], data2[1]);

      edges.push_back(std::make_pair(poseFrom, poseTo));

    }

  }

  const char * temp = graphDirectory_.c_str();
  optimizer_.save(temp);

  delete data;

  delete data1;

  delete data2;

}

void G2OSolver::getGraphWithFIM(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges,
                                std::vector<Eigen::VectorXd> &infoMatrices)
{
  using namespace g2o;

  //HyperGraph::VertexIDMap vertexMap = optimizer_.vertices();

  //HyperGraph::EdgeSet edgeSet = optimizer_.edges();

  double *data = new double[3];

  for (SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it)
  {

    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);

    if(v)
    {

      v->getEstimateData(data);

      Eigen::Vector2d pose(data[0], data[1]);

      nodes.push_back(pose);

    }
  }

  double *data1 = new double[3];

  double *data2 = new double[3];

  for (SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it)
  {

    EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
    // std::cout << e->information() << '\n';

    if(e)
    {
      VertexSE2* v1 = dynamic_cast<VertexSE2*>(e->vertices()[0]);

      v1->getEstimateData(data1);

      Eigen::Vector2d poseFrom(data1[0], data1[1]);

      VertexSE2* v2 = dynamic_cast<VertexSE2*>(e->vertices()[1]);

      v2->getEstimateData(data2);

      Eigen::Vector2d poseTo(data2[0], data2[1]);

      edges.push_back(std::make_pair(poseFrom, poseTo));

      // g2o format file :  q_{11}; q_{12}; q_{13}; q_{22}; q_{23}; q_{33}.

      Eigen::VectorXd data3(6);
      data3 << e->information()(0,0),e->information()(0,1),e->information()(0,2),e->information()(1,1),e->information()(1,2),e->information()(2,2);

      // std::cout << "data3: " << data3 << '\n';

      infoMatrices.push_back(data3);

    }

  }

  // if(optimizer_.edges().size()>0){
  //   for(unsigned i = 0; i< infoMatrices.size(); ++i) {
  //       std::cout << "infoMatrices: " << infoMatrices[i] << std::endl;
  //   }
  // }

  delete data;

  delete data1;

  delete data2;

}
