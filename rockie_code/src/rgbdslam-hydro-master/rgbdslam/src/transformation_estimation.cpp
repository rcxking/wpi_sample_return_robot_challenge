#include "node.h"
#include "scoped_timer.h"
#include "transformation_estimation.h"
//#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/icp/types_icp.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "landmark.h" //Only for point_information_matrix. TODO: Move to misc2.h
#include <Eigen/SVD>
//TODO: Move these definitions and includes into a common header, g2o.h
#include "g2o/core/estimate_propagator.h"
//#include "g2o/core/factory.h"
//#include "g2o/core/solver_factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

//!Set parameters for icp optimizer
void optimizerSetup(g2o::SparseOptimizer& optimizer){
  //optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  //g2o::Optimizato
  optimizer.setVerbose(false);

  // variable-size block solver
  g2o::BlockSolverX::LinearSolverType * linearSolver
      = new g2o::LinearSolverCholmod<g2o ::BlockSolverX::PoseMatrixType>();


  g2o::BlockSolverX * solver_ptr
      = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  //optimizer.setSolver(solver_ptr);

  g2o::ParameterCamera* cameraParams = new g2o::ParameterCamera();
  cameraParams->setKcam(521,521,319.5,239.5);
  g2o::SE3Quat offset; // identity
  cameraParams->setOffset(offset);
  cameraParams->setId(0);
  optimizer.addParameter(cameraParams);
}

//Second cam is fixed, therefore the transformation from the second to the first cam will be computed
std::pair<g2o::VertexSE3*, g2o::VertexSE3*>  sensorVerticesSetup(g2o::SparseOptimizer& optimizer, Eigen::Matrix4f& tf_estimate){
    g2o::VertexSE3 *vc2 = new VertexSE3();
    {// set up rotation and translation for the newer node as identity
      Eigen::Quaterniond q(1,0,0,0);
      Eigen::Vector3d t(0,0,0);
      q.setIdentity();
      g2o::SE3Quat cam2(q,t);

      vc2->setEstimate(cam2);
      vc2->setId(1); 
      vc2->setFixed(true);
      optimizer.addVertex(vc2);
    }

    g2o::VertexSE3 *vc1 = new VertexSE3();
    {
      Eigen::Quaterniond q(tf_estimate.topLeftCorner<3,3>().cast<double>());//initialize rotation from estimate 
      Eigen::Vector3d t(tf_estimate.topRightCorner<3,1>().cast<double>());  //initialize translation from estimate
      g2o::SE3Quat cam1(q,t);
      vc1->setEstimate(cam1);
      vc1->setId(0);  
      optimizer.addVertex(vc1);
    }

    // add to optimizer
    return std::make_pair(vc1, vc2);
}

Proj_edge_type* edgeToFeature(const Node* node, 
                              unsigned int feature_id,
                              g2o::VertexSE3* camera_vertex,
                              g2o::VertexPointXYZ* feature_vertex)
{
   Proj_edge_type* edge = new Proj_edge_type();
   cv::KeyPoint kp = node->feature_locations_2d_[feature_id];
   Vector4f position = node->feature_locations_3d_[feature_id];
   float depth = position(2);
   if(!isnan(depth))
   {
     Eigen::Vector3d pix_d(kp.pt.x,kp.pt.y,depth);
     //ROS_INFO_STREAM("Edge from camera to position "<< pix_d.transpose());
     edge->setMeasurement(pix_d);
     Eigen::Matrix3d info_mat = point_information_matrix(depth);
     edge->setInformation(info_mat);
     feature_vertex->setEstimate(position.cast<double>().head<3>());

   } 
   else {//FIXME instead of using an arbitrary depth value and high uncertainty, use the correct vertex type (on the other hand Rainer suggested this proceeding too)
     Eigen::Vector3d pix_d(kp.pt.x,kp.pt.y,10.0);
     //ROS_INFO_STREAM("Edge from camera to position "<< pix_d.transpose());
     edge->setMeasurement(pix_d);
     Eigen::Matrix3d info_mat = point_information_matrix(10);//as uncertain as if it was ...meter away
     edge->setInformation(info_mat);
     feature_vertex->setEstimate(Eigen::Vector3d(position(0)*10, position(1)*10, 10.0));//Move from 1m to 10 m distance. Shouldn't matter much
   }
   edge->setParameterId(0,0);
   //edge->setRobustKernel(true);
   edge->vertices()[0] = camera_vertex;
   edge->vertices()[1] = feature_vertex;

   return edge;
}
//!Compute 
void getTransformFromMatchesG2O(const Node* earlier_node,
                                const Node* newer_node,
                                const std::vector<cv::DMatch> & matches,
                                Eigen::Matrix4f& transformation_estimate, //Input (initial guess) and Output
                                int iterations)
{
  ScopedTimer s(__FUNCTION__);
  //G2O Initialization
  g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();//TODO: Speicherleck
  //Set parameters for icp optimizer
  optimizerSetup(*optimizer);
  //First camera is earlier_node, second camera is newer_node
  //Second cam is set to fixed, therefore the transformation from the second to the first cam will be computed
  std::pair<g2o::VertexSE3*, g2o::VertexSE3*> cams = sensorVerticesSetup(*optimizer, transformation_estimate);

  //std::vector<LM_vertex_type*> feature_vertices;
  int v_id = optimizer->vertices().size(); //0 and 1 are taken by sensor vertices
  //For each match, create a vertex and connect it to the sensor vertices with the measured position
  BOOST_FOREACH(const cv::DMatch& m, matches)
  {
    LM_vertex_type* v = new LM_vertex_type();//TODO: Speicherleck?
    v->setId(v_id++);
    v->setFixed(false);

    optimizer->addVertex(v);
    //feature_vertices.push_back(v);

    Proj_edge_type* e1 = edgeToFeature(earlier_node, m.trainIdx, cams.first, v);
    optimizer->addEdge(e1);

    Proj_edge_type* e2 = edgeToFeature(newer_node, m.queryIdx, cams.second, v);
    optimizer->addEdge(e2);

  }
  optimizer->initializeOptimization();

  ROS_INFO("Optimizer Size: %zu", optimizer->vertices().size());
  optimizer->optimize(iterations);
  optimizer->computeActiveErrors();

  //g2o::SE3Quat final_transformation =  cams.first->estimateAsSE3Quat().inverse();
  //transformation_estimate = final_transformation.to_homogeneous_matrix().cast<float>(); 
  
  transformation_estimate = cams.first->estimate().cast<float>().inverse().matrix();
  /*
  std::cout << "G2O Estimates: \n"; 
  assert(feature_vertices.size() == matches.size() && "Missing Vertices for the Matches");

  for(int i = 0; i < feature_vertices.size(); i++){ 
      LM_vertex_type* v = feature_vertices[i];
      Eigen::Vector3d pos_estimate = v->estimateAsSE3Quat();
      Eigen::Vector4f pos_msr_new = newer_node->feature_locations_3d_[matches[i].queryIdx];
      Eigen::Vector4f pos_msr_old = earlier_node->feature_locations_3d_[matches[i].trainIdx];
      if(v){ 
        std::cout << pos_estimate.transpose() << " | "; 
        std::cout << pos_msr_new.transpose() << " | "; 
        std::cout << pos_msr_old.transpose() << "\n"; 
        //if(pos_estimate(2) > 0.6){ //near-limitation of kinect


      } else {
        ROS_ERROR("non-LM-type vertex in graph!");
      }
  }
  */
  delete optimizer;
}
