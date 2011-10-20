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

#ifndef SBA_H_
#define SBA_H_

#include <vector>
#include <Eigen/Geometry>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <Eigen/StdVector>

namespace g2o
{
  typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > VectorQuaterniond;
  typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VectorVector3d;

  void
  sba_process_impl(const Eigen::SparseMatrix<int> &x, const Eigen::SparseMatrix<int> & y,
                   const Eigen::SparseMatrix<int> & disparity, const Eigen::Matrix3d & K,
                   const std::vector<Eigen::Quaterniond> & quaternions, const std::vector<Eigen::Vector3d> & Ts,
                   const std::vector<Eigen::Vector3d> & in_point_estimates,
                   std::vector<Eigen::Vector3d> & out_point_estimates);
  void
  sba_process_impl(const Eigen::SparseMatrix<int> &x, const Eigen::SparseMatrix<int> & y, const Eigen::Matrix3d & K,
                   const std::vector<Eigen::Quaterniond> & quaternions, const std::vector<Eigen::Vector3d> & Ts,
                   const std::vector<Eigen::Vector3d> & in_point_estimates,
                   std::vector<Eigen::Vector3d> & out_point_estimates);
}

#endif /* SBA_H_ */
