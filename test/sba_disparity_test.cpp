#include <iostream>
#include <stdint.h>
#include <vector>

#include <tr1/random>
#include <tr1/unordered_set>

#include <gtest/gtest.h>

#include "g2o/sba.h"

#include <Eigen/StdVector>

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

  std::cout << "PIXEL_NOISE: " << PIXEL_NOISE << std::endl << "OUTLIER_RATIO: " << OUTLIER_RATIO << std::endl
            << "ROBUST_KERNEL: " << ROBUST_KERNEL << std::endl << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << std::endl
            << "DENSE: " << DENSE << std::endl;

// set up 500 points
  std::vector<Eigen::Vector3d> true_points;
  for (size_t i = 0; i < 500; ++i)
  {
    true_points.push_back(
        Eigen::Vector3d((Sample::uniform() - 0.5) * 3, Sample::uniform() - 0.5, Sample::uniform() + 10));
  }

  Eigen::Vector2d focal_length(500, 500); // pixels
  Eigen::Vector2d principal_point(320, 240); // 640x480 image

  std::vector<g2o::VertexSCam> true_cameras;

// set up camera params
  Eigen::Matrix3d K;
  K(0, 0) = focal_length[0];
  K(1, 1) = focal_length[1];
  K(0, 2) = principal_point[0];
  K(1, 2) = principal_point[1];

// set up 5 vertices, first 2 fixed
  double baseline = 7.5;
  g2o::VertexSCam::setKcam(K(0, 0), K(1, 1), K(0, 2), K(1, 2), baseline);
  int vertex_id = 0;
  std::vector<Eigen::Quaterniond> quaternions;
  std::vector<Eigen::Vector3d> Ts;
  for (size_t i = 0; i < 5; ++i)
  {
    Eigen::Vector3d trans(i * 0.04 - 1., 0, 0);

    Eigen::Quaterniond q;
    q.setIdentity();
    quaternions.push_back(q);
    Ts.push_back(trans);

    g2o::VertexSCam v_se3;

    g2o::SE3Quat pose(q, trans);

    v_se3.setId(vertex_id);
    v_se3.estimate() = pose;
    v_se3.setAll(); // set aux transforms

    true_cameras.push_back(v_se3);
  }

  double sum_diff1 = 0;

  // add point projections to this vertex
  unsigned int n_points = true_points.size();
  unsigned int n_views = true_cameras.size();
  std::cout << "n: " << n_points << " " << n_views << std::endl;
  Eigen::SparseMatrix<int> x(n_views, n_points);
  Eigen::SparseMatrix<int> y(n_views, n_points);
  Eigen::SparseMatrix<int> disparity(n_views, n_points);
  std::vector<Eigen::Vector3d> in_point_estimates(n_points);
  for (size_t i = 0; i < n_points; ++i)
  {
    Eigen::Vector3d estimate = true_points.at(i)
                               + Eigen::Vector3d(Sample::gaussian(1), Sample::gaussian(1), Sample::gaussian(1));
    in_point_estimates[i] = estimate;

    for (size_t j = 0; j < n_views; ++j)
    {
      Eigen::Vector3d z;
      true_cameras[j].mapPoint(z, true_points.at(i));

      if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480)
      {
        Eigen::Vector3d z;

        z += Eigen::Vector3d(Sample::gaussian(PIXEL_NOISE), Sample::gaussian(PIXEL_NOISE),
                             Sample::gaussian(PIXEL_NOISE / 16.0));
        x.insert(j, i) = z[0];
        y.insert(j, i) = z[1];
        disparity.insert(j, i) = z[2];
      }
    }

    Eigen::Vector3d diff = estimate - true_points[i];
    sum_diff1 += diff.dot(diff);
  }

  std::vector<Eigen::Vector3d> out_point_estimates;
  g2o::sba_process_impl(x, y, disparity, K, quaternions, Ts, in_point_estimates, out_point_estimates);

  std::cout << "Point error before optimization : " << sqrt(sum_diff1 / n_points) << std::endl;

  double sum_diff2 = 0;

  for (size_t i = 0; i < n_points; ++i)
  {
    Eigen::Vector3d diff = out_point_estimates[i] - true_points[i];
    sum_diff2 += diff.dot(diff);
  }

  std::cout << "Point error after optimization : " << sqrt(sum_diff2 / n_points) << std::endl << std::endl;
  EXPECT_LE(sum_diff2, sum_diff1);
}
