/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <vector>
#include <tr1/unordered_set>

#include <ecto/ecto.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Geometry>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <Eigen/StdVector>

#include "g2o/core/block_solver.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/solver.h"
#include "g2o/core/structure_only_solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"

using ecto::tendrils;

namespace g2o
{
  struct SBA
  {
  public:
    /*
     static void
     declare_params(ecto::tendrils& params)
     {
     }
     */

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<Eigen::SparseMatrix<int> >("x", "The n_view by n_points matrix of x.").required(true);
      inputs.declare<Eigen::SparseMatrix<int> >("y", "The n_view by n_points matrix of y.").required(true);
      inputs.declare<Eigen::SparseMatrix<int> >("disparity", "The n_view by n_points matrix of disparity.").required(
          true);
      inputs.declare<std::vector<Eigen::Quaterniond> >("quaternions", "The initial estimates of the camera rotations.").required(
          true);
      inputs.declare<std::vector<Eigen::Vector3d> >("Ts", "The initial estimates of the camera translations.").required(
          true);
      inputs.declare<Eigen::Matrix3d>("K", "The intrinsic parameter matrix.").required(true);
      inputs.declare<std::vector<Eigen::Vector3d> >("points", "The estimated 3d points.").required(true);

      outputs.declare<std::vector<Eigen::Vector3d> >("points", "The optimized positions of the points.");
    }

    void
    configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      x_ = inputs["x"];
      y_ = inputs["y"];
      disparity_ = inputs["disparity"];
      quaternions_ = inputs["quaternions"];
      Ts_ = inputs["Ts"];
      K_ = inputs["K"];
      point_estimates_ = inputs["points"];
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {

      g2o::SparseOptimizer optimizer;
      optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
      optimizer.setVerbose(false);
      g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
      if (0)
      {
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
      }
      else
      {
        linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
      }

      g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&optimizer, linearSolver);

      optimizer.setSolver(solver_ptr);

      double baseline = 0.075; // 7.5 cm baseline

      // set up camera params
      g2o::VertexSCam::setKcam((*K_)(0, 0), (*K_)(1, 1), (*K_)(0, 2), (*K_)(1, 2), baseline);

      // set up the camera vertices
      unsigned int vertex_id = 0;
      for (size_t i = 0; i < quaternions_->size(); ++i)
      {
        g2o::SE3Quat pose((*quaternions_)[i], (*Ts_)[i]);

        g2o::VertexSCam * v_se3 = new g2o::VertexSCam();

        v_se3->setId(vertex_id);
        v_se3->estimate() = pose;
        v_se3->setAll(); // set aux transforms

        optimizer.addVertex(v_se3);
        vertex_id++;
      }

      int point_id = vertex_id;

      cout << endl;
      tr1::unordered_map<int, int> pointid_2_trueid;

      // add point projections to this vertex
      for (size_t i = 0; i < point_estimates_->size(); ++i)
      {
        g2o::VertexPointXYZ * v_p = new g2o::VertexPointXYZ();

        v_p->setId(point_id);
        v_p->setMarginalized(true);
        v_p->estimate() = point_estimates_->at(i);

        for (size_t j = 0; j < quaternions_->size(); ++j)
        {
          // check whether the point is visible in that view
          if ((x_->coeff(j, i) == 0) && (y_->coeff(j, i) == 0) && (disparity_->coeff(j, i) == 0))
            continue;

          g2o::Edge_XYZ_VSC * e = new g2o::Edge_XYZ_VSC();

          e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

          e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(j)->second);

          Vector3d z(x_->coeff(j, i), y_->coeff(j, i), disparity_->coeff(j, i));
          e->measurement() = z;
          e->inverseMeasurement() = -z;
          e->information() = Matrix3d::Identity();

          // TODO
          //e->setRobustKernel(ROBUST_KERNEL);
          e->setHuberWidth(1);

          optimizer.addEdge(e);
        }

        optimizer.addVertex(v_p);

        pointid_2_trueid.insert(make_pair(point_id, i));

        ++point_id;
      }

      cout << endl;
      optimizer.initializeOptimization();

      optimizer.setVerbose(true);

      g2o::StructureOnlySolver<3> structure_only_ba;

      cout << endl;
      cout << "Performing full BA:" << endl;
      optimizer.optimize(1);

      std::vector<Eigen::Vector3d> final_points(point_estimates_->size());
      for (tr1::unordered_map<int, int>::iterator it = pointid_2_trueid.begin(); it != pointid_2_trueid.end(); ++it)
      {
        g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(it->first);

        if (v_it == optimizer.vertices().end())
        {
          cerr << "Vertex " << it->first << " not in graph!" << endl;
          exit(-1);
        }

        g2o::VertexPointXYZ * v_p = dynamic_cast<g2o::VertexPointXYZ *>(v_it->second);

        if (v_p == 0)
        {
          cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
          exit(-1);
        }
        final_points[it->second] = v_p->estimate();
      }

      // Return the right positions of the points
      outputs["points"] << final_points;

      return ecto::OK;
    }
  private:
    ecto::spore<Eigen::SparseMatrix<int> > x_;
    ecto::spore<Eigen::SparseMatrix<int> > y_;
    ecto::spore<Eigen::SparseMatrix<int> > disparity_;
    ecto::spore<std::vector<Eigen::Quaterniond> > quaternions_;
    ecto::spore<std::vector<Eigen::Vector3d> > Ts_;
    ecto::spore<Eigen::Matrix3d> K_;
    ecto::spore<std::vector<Eigen::Vector3d> > point_estimates_;
  };
}

ECTO_CELL(g2o, g2o::SBA, "SBA", "Stack 3d points and descriptors but by cleaning them")
