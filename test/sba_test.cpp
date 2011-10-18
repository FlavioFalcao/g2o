#include <gtest/gtest.h>

#include "g2o/sba.h"

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
#include <tr1/unordered_set>

#include "g2o/types/icp/types_icp.h"

class Sample
{

  static std::tr1::ranlux_base_01 gen_real;
  static std::tr1::mt19937 gen_int;
public:
  static int
  uniform(int from, int to);

  static double
  uniform();

  static double
  gaussian(double sigma);
};

std::tr1::ranlux_base_01 Sample::gen_real;
std::tr1::mt19937 Sample::gen_int;

int
Sample::uniform(int from, int to)
{
  std::tr1::uniform_int<int> unif(from, to);
  int sam = unif(gen_int);
  return sam;
}

double
Sample::uniform()
{
  std::tr1::uniform_real<double> unif(0.0, 1.0);
  double sam = unif(gen_real);
  return sam;
}

double
Sample::gaussian(double sigma)
{
  std::tr1::normal_distribution<double> gauss(0.0, sigma);
  double sam = gauss(gen_real);
  return sam;
}

TEST(g2o, SBA)
{
  double PIXEL_NOISE = 1;

  double OUTLIER_RATIO = 0.0;

  bool ROBUST_KERNEL = false;

  bool STRUCTURE_ONLY = false;

  bool DENSE = false;

  std::cout << "PIXEL_NOISE: " << PIXEL_NOISE << std::endl<<"OUTLIER_RATIO: " << OUTLIER_RATIO << std::endl<< "ROBUST_KERNEL: " << ROBUST_KERNEL << std::endl<< "STRUCTURE_ONLY: " << STRUCTURE_ONLY << std::endl<< "DENSE: " << DENSE << std::endl;

// set up 500 points
  vector<Vector3d> true_points;
  for (size_t i = 0; i < 500; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform() - 0.5) * 3, Sample::uniform() - 0.5, Sample::uniform() + 10));
  }

  Vector2d focal_length(500, 500); // pixels
  Vector2d principal_point(320, 240);// 640x480 image
  double baseline = 0.075;// 7.5 cm baseline

  vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;

// set up camera params
  Eigen::Matrix3d K;
  K(0,0)=focal_length[0];
  K(1,1)= focal_length[1];
  K(0,2)=principal_point[0];
  K(1,2)= principal_point[1];

// set up 5 vertices, first 2 fixed
  int vertex_id = 0;
  std::vector<Eigen::Quaterniond> quaternions;
  std::vector<Eigen::Vector3d> Ts;
  for (size_t i = 0; i < 5; ++i)
  {
    Vector3d trans(i * 0.04 - 1., 0, 0);

    Eigen::Quaterniond q;
    q.setIdentity();
    quaternions.push_back(q);
    Ts.push_back(trans);

    g2o::SE3Quat pose(q, trans);
    true_poses.push_back(pose);
  }

  int point_id = vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

  std::tr1::unordered_map<int, int> pointid_2_trueid;
  std::tr1::unordered_set<int> inliers;

// add point projections to this vertex
  for (size_t i = 0; i < true_points.size(); ++i)
  {
    g2o::VertexPointXYZ * v_p = new g2o::VertexPointXYZ();

    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->estimate() = true_points.at(i) + Vector3d(Sample::gaussian(1), Sample::gaussian(1), Sample::gaussian(1));

    int num_obs = 0;

    for (size_t j = 0; j < true_poses.size(); ++j)
    {
      Vector3d z;
      true_poses[j].mapPoint(z, true_points.at(i));

      if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480)
      {
        Vector3d z;
        double sam = Sample::uniform();
        if (sam < OUTLIER_RATIO)
        {
          z = Vector3d(Sample::uniform(64, 640), Sample::uniform(0, 480), Sample::uniform(0, 64)); // disparity
          z(2) = z(0) - z(2);// px' now

          inlier = false;
        }

        z += Vector3d(Sample::gaussian(PIXEL_NOISE), Sample::gaussian(PIXEL_NOISE),
            Sample::gaussian(PIXEL_NOISE / 16.0));
      }

      if (inlier)
      {
        inliers.insert(point_id);
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      // else
      //   cout << "Point: " << point_id <<  "has at least one spurious observation" <<endl;

      pointid_2_trueid.insert(make_pair(point_id, i));

      ++point_id;
      ++point_num;
    }

  }

  sba_process_impl(const Eigen::SparseMatrix<int> &x, const Eigen::SparseMatrix<int> & y,
      const Eigen::SparseMatrix<int> & disparity, K,
      quaternions, Ts,
      const std::vector<Eigen::Vector3d> & in_point_estimates,
      std::vector<Eigen::Vector3d> & out_point_estimates);

  cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2 / point_num) << endl;

  point_num = 0;
  sum_diff2 = 0;

  for (std::tr1::unordered_map<int, int>::iterator it = pointid_2_trueid.begin(); it != pointid_2_trueid.end(); ++it)
  {

    g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(it->first);

    g2o::VertexPointXYZ * v_p = dynamic_cast<g2o::VertexPointXYZ *>(v_it->second);

    Vector3d diff = v_p->estimate() - true_points[it->second];

    if (inliers.find(it->first) == inliers.end())
    continue;

    sum_diff2 += diff.dot(diff);

    ++point_num;
  }

  cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2 / point_num) << endl;
  cout << endl;

}
