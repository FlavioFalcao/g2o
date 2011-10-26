/*********************************************************************
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


#include <g2o/pe.h>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>


using namespace Eigen;
using namespace std;

namespace pe
{

  // constructor
  PoseEstimator::PoseEstimator(int NRansac, bool LMpolish, double maxrange,
                               double maxidx, double maxidd)
  {
    numRansac = NRansac;
    maxMatchRange = maxrange;
    maxInlierXDist = maxidx;
    maxInlierDDist = maxidd;
  }


  //
  // find the best estimate for a geometrically-consistent match
  // assumes stereo points have been calculated, and matches filtered,
  //   and frames set up
  // uses the SVD procedure for aligning point clouds
  //   SEE: Arun, Huang, Blostein: Least-Squares Fitting of Two 3D Point Sets
  // final SBA polishing step
  //

  int PoseEstimator::estimate(const matches_t &matches, 
                              const points_t &train_kpts, const points_t &query_kpts,
                              const points3d_t &train_pts, const points3d_t &query_pts,
                              image_pipeline::PinholeCameraModel &pm, double baseline)
  {
    int nmatch = matches.size();

    // set up data structures for fast processing
    // indices to good matches
    vector<int> m0, m1;
    for (int i=0; i<nmatch; i++)
      {
        if (query_pts[matches[i].queryIdx].z < maxMatchRange && 
            train_pts[matches[i].trainIdx].z > maxMatchRange)
          {
            m0.push_back(matches[i].queryIdx);
            m1.push_back(matches[i].trainIdx);
          }
      }

    nmatch = m0.size();
    if (nmatch < 3) return 0;   // can't do it...

    int bestinl = 0;

    // RANSAC loop
    for (int i=0; i<numRansac; i++) 
      {
        // find a candidate
        int a=rand()%nmatch;
        int b = a;
        while (a==b)
          b=rand()%nmatch;
        int c = a;
        while (a==c || b==c)
          c=rand()%nmatch;

        int i0a = m0[a];
        int i0b = m0[b];
        int i0c = m0[c];
        int i1a = m1[a];
        int i1b = m1[b];
        int i1c = m1[c];

        if (i0a == i0b || i0a == i0c || i0b == i0c ||
            i1a == i1b || i1a == i1c || i1b == i1c)
          continue;

        // get centroids
        const cv::Point3f &qpa = query_pts[i0a];
        Vector3d p0a(qpa.x,qpa.y,qpa.z); 
        const cv::Point3f &qpb = query_pts[i0b];
        Vector3d p0b(qpb.x,qpb.y,qpb.z); 
        const cv::Point3f &qpc = query_pts[i0c];
        Vector3d p0c(qpc.x,qpc.y,qpc.z);
        const cv::Point3f &tpa = train_pts[i1a];
        Vector3d p1a(tpa.x,tpa.y,tpa.z);
        const cv::Point3f &tpb = train_pts[i1b];
        Vector3d p1b(tpb.x,tpb.y,tpb.z);
        const cv::Point3f &tpc = train_pts[i1c];
        Vector3d p1c(tpc.x,tpc.y,tpc.z);

        Vector3d c0 = (p0a+p0b+p0c)*(1.0/3.0);
        Vector3d c1 = (p1a+p1b+p1c)*(1.0/3.0);

        // subtract out
        p0a -= c0;
        p0b -= c0;
        p0c -= c0;
        p1a -= c1;
        p1b -= c1;
        p1c -= c1;

        Matrix3d H = p1a*p0a.transpose() + p1b*p0b.transpose() +
                     p1c*p0c.transpose();

#if 0
        cout << p0a.transpose() << endl;
        cout << p0b.transpose() << endl;
        cout << p0c.transpose() << endl;
#endif

        // do the SVD thang
        JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
        Matrix3d V = svd.matrixV();
        Matrix3d R = V * svd.matrixU().transpose();          
        double det = R.determinant();
        //ntot++;
        if (det < 0.0)
          {
            //nneg++;
            V.col(2) = V.col(2)*-1.0;
            R = V * svd.matrixU().transpose();
          }
        Vector3d tr = c0-R*c1;    // translation 

        //        cout << "[PE test] R: " << endl << R << endl;
        //        cout << "[PE test] t: " << tr.transpose() << endl;

#if 0
        R << 0.9997, 0.002515, 0.02418,
          -0.002474, .9999, -0.001698,
          -0.02418, 0.001638, 0.9997;
        tr << -0.005697, -0.01753, 0.05666;
        R = R.transpose();
        tr = -R*tr;
#endif

#if 0
        // transformation matrix, 3x4
        Matrix<double,3,4> tfm;
        //        tfm.block<3,3>(0,0) = R.transpose();
        //        tfm.col(3) = -R.transpose()*tr;
        tfm.block<3,3>(0,0) = R;
        tfm.col(3) = tr;
#endif
        
        //        cout << "[PE test] T: " << endl << tfm << endl;

        Vector3f trf = tr.cast<float>();      // convert to floats
        Matrix3f Rf = R.cast<float>();
        Matrix3f Kf = pm.getK().cast<float>(); // camera matrix
        float fb = Kf(0,0)*baseline; // focal length times baseline
        float maxInlierXDist2 = maxInlierXDist*maxInlierXDist;
        float maxInlierDDist2 = maxInlierDDist*maxInlierDDist;

        // find inliers, based on image reprojection
        int inl = 0;
        for (int i=0; i<nmatch; i++)
          {
            cv::Point3f cvp = train_pts[m1[i]];
            Vector3f pt1(cvp.x,cvp.y,cvp.z);
            Vector3f pt = Rf*pt1+trf; // transform query point
            Vector3f ipt = Kf*pt;
            float iz = 1.0/ipt.z();
            const cv::Point2f &kp = query_kpts[m0[i]];
            float dx = kp.x - ipt.x()*iz;
            float dy = kp.y - ipt.y()*iz;
            float dd = fb/(query_pts[m0[i]].z - ipt.z());
            if (dx*dx < maxInlierXDist2 && dy*dy < maxInlierXDist2 && 
                dd*dd < maxInlierDDist2)
               //              inl+=(int)fsqrt(ipt.z()); // clever way to weight closer points
              inl++;
          }
        
        if (inl > bestinl) 
          {
            bestinl = inl;
            rot = R.cast<float>();
            trans = tr.cast<float>();
          }
      }

    printf("Best inliers: %d\n", bestinl);
//    printf("Total ransac: %d  Neg det: %d\n", ntot, nneg);

    // reduce matches to inliers
    matches_t inls;    // temporary for current inliers
    inliers.clear();
    Matrix3f Kf = pm.getK().cast<float>(); // camera matrix
    float fb = Kf(0,0)*baseline; // focal length times baseline
    float maxInlierXDist2 = maxInlierXDist*maxInlierXDist;
    float maxInlierDDist2 = maxInlierDDist*maxInlierDDist;

    nmatch = matches.size();
    for (int i=0; i<nmatch; i++)
      {
        cv::Point3f cvp = train_pts[m1[i]];
        Vector3f pt1(cvp.x,cvp.y,cvp.z);
        Vector3f pt = rot*pt1+trans; // transform query point
        Vector3f ipt = Kf*pt;
        float iz = 1.0/ipt.z();
        const cv::Point2f &kp = query_kpts[m0[i]];
        float dx = kp.x - ipt.x()*iz;
        float dy = kp.y - ipt.y()*iz;
        float dd = fb/(query_pts[m0[i]].z - ipt.z());
        if (dx*dx < maxInlierXDist2 && dy*dy < maxInlierXDist2 && 
            dd*dd < maxInlierDDist2)
          inls.push_back(matches[i]); 
      }

#if 0
    cout << endl << trans.transpose().head(3) << endl << endl;
    cout << rot << endl;
#endif

    // polish with stereo sba
    if (polish)
      {
#if 0
        // system
        SysSBA sba;
        sba.verbose = 0;

        // set up nodes
        // should have a frame => node function        
        Vector4d v0 = Vector4d(0,0,0,1);
        Quaterniond q0 = Quaternion<double>(Vector4d(0,0,0,1));
        sba.addNode(v0, q0, f0.cam, true);
        
        Quaterniond qr1(rot);   // from rotation matrix
        Vector4d temptrans = Vector4d(trans(0), trans(1), trans(2), 1.0);

        //        sba.addNode(temptrans, qr1.normalized(), f1.cam, false);
        qr1.normalize();
        sba.addNode(temptrans, qr1, f1.cam, false);

        int in = 3;
        if (in > (int)inls.size())
          in = inls.size();

        // set up projections
        for (int i=0; i<(int)inls.size(); i++)
          {
            // add point
            int i0 = inls[i].queryIdx;
            int i1 = inls[i].trainIdx;
            Vector4d pt = query_pts[i0];
            sba.addPoint(pt);
            
            // projected point, ul,vl,ur
            Vector3d ipt;
            ipt(0) = f0.kpts[i0].pt.x;
            ipt(1) = f0.kpts[i0].pt.y;
            ipt(2) = ipt(0)-f0.disps[i0];
            sba.addStereoProj(0, i, ipt);

            // projected point, ul,vl,ur
            ipt(0) = f1.kpts[i1].pt.x;
            ipt(1) = f1.kpts[i1].pt.y;
            ipt(2) = ipt(0)-f1.disps[i1];
            sba.addStereoProj(1, i, ipt);
          }

        sba.huber = 2.0;
        sba.doSBA(5,10e-4,SBA_DENSE_CHOLESKY);
        int nbad = sba.removeBad(2.0);
//        cout << endl << "Removed " << nbad << " projections > 2 pixels error" << endl;
        sba.doSBA(5,10e-5,SBA_DENSE_CHOLESKY);

//        cout << endl << sba.nodes[1].trans.transpose().head(3) << endl;

        // get the updated transform
	      trans = sba.nodes[1].trans.head(3);
	      Quaterniond q1;
	      q1 = sba.nodes[1].qrot;
	      rot = q1.toRotationMatrix();

        // set up inliers
        inliers.clear();
        for (int i=0; i<(int)inls.size(); i++)
          {
            ProjMap &prjs = sba.tracks[i].projections;
            if (prjs[0].isValid && prjs[1].isValid) // valid track
              inliers.push_back(inls[i]);
          }
#if 0
        printf("Inliers: %d   After polish: %d\n", (int)inls.size(), (int)inliers.size());
#endif

#endif

      }

    return (int)inliers.size();
  }
} // ends namespace pe
