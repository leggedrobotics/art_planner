#include "art_planner/utils.h"

#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <opencv2/photo/photo.hpp>



using namespace art_planner;



grid_map::Matrix art_planner::inpaintMatrix(const grid_map::Matrix& mat) {

  // Swap rows and cols because Eigen is column major, opencv is row major.
  cv::Mat mat_cv(mat.cols(),
                 mat.rows(),
                 cv::DataType<grid_map::Matrix::Scalar>::type,
                 const_cast<void*>(static_cast<const void*>(mat.data())));

  const auto min = mat.minCoeffOfFinites();
  const auto max = mat.maxCoeffOfFinites();

  const auto mask = (mat_cv!=mat_cv);

//  mat_cv = (mat_cv-min)/(max-min)*255;

  cv::Mat mat_8u_cv;
  mat_cv.convertTo(mat_8u_cv, CV_8U, 255/(max-min), -min*255/(max-min));

  cv::Mat mat_8u_cv_inpainted;

  cv::inpaint(mat_8u_cv, mask, mat_8u_cv_inpainted, 3, cv::INPAINT_TELEA);
  cv::Mat mat_cv_inpainted;
  mat_8u_cv_inpainted.convertTo(mat_cv_inpainted, CV_32F);

  Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
                           Eigen::Dynamic,
                           Eigen::Dynamic> > mat_inpainted(reinterpret_cast<grid_map::Matrix::Scalar*>(mat_cv_inpainted.data),
                                                           mat.rows(),
                                                           mat.cols());

  mat_inpainted *= (max-min)/255;
  mat_inpainted.array() += min;

  // Inpainting fails at zero indices for some reason.
  mat_inpainted.col(0) = mat_inpainted.col(1);
  mat_inpainted.row(0) = mat_inpainted.row(1);

  return mat_inpainted;
}



grid_map::Matrix art_planner::blurMatrix(const grid_map::Matrix &mat, int size) {
  cv::Mat mat_cv(mat.cols(),
                 mat.rows(),
                 cv::DataType<grid_map::Matrix::Scalar>::type,
                 const_cast<void*>(static_cast<const void*>(mat.data())));

  cv::Mat mat_cv_blurred;

  cv::blur(mat_cv, mat_cv_blurred, cv::Size(size, size));


  Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
                           Eigen::Dynamic,
                           Eigen::Dynamic> > mat_blurred(reinterpret_cast<grid_map::Matrix::Scalar*>(mat_cv_blurred.data),
                                                           mat.rows(),
                                                           mat.cols());

  return mat_blurred;
}



grid_map::Matrix art_planner::gaussianBlurMatrix(const grid_map::Matrix &mat,
                                                 int size,
                                                 double std_dev) {
  cv::Mat mat_cv(mat.cols(),
                 mat.rows(),
                 cv::DataType<grid_map::Matrix::Scalar>::type,
                 const_cast<void*>(static_cast<const void*>(mat.data())));

  cv::Mat mat_cv_blurred;

  cv::GaussianBlur(mat_cv, mat_cv_blurred, cv::Size(size, size), std_dev);


  Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
                           Eigen::Dynamic,
                           Eigen::Dynamic> > mat_blurred(reinterpret_cast<grid_map::Matrix::Scalar*>(mat_cv_blurred.data),
                                                           mat.rows(),
                                                           mat.cols());

  return mat_blurred;
}



cv::Mat getCircularKernel(int size) {
  cv::Mat kernel = cv::Mat::zeros(size, size, CV_8U);
  const auto radius = size/2;
  cv::circle(kernel, cv::Point(radius, radius), radius, {255, 255, 255}, -1);
  return kernel;
}



grid_map::Matrix art_planner::dilateAndErodeMatrix(const grid_map::Matrix& mat, int size) {
  cv::Mat mat_cv(mat.cols(),
                 mat.rows(),
                 cv::DataType<grid_map::Matrix::Scalar>::type,
                 const_cast<void*>(static_cast<const void*>(mat.data())));

  cv::Mat kernel = getCircularKernel(size);
  cv::Mat mat_cv_filtered;

  cv::dilate(mat_cv, mat_cv_filtered, kernel);
  cv::erode(mat_cv_filtered, mat_cv_filtered, kernel);

  Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
                           Eigen::Dynamic,
                           Eigen::Dynamic> > mat_filtered(reinterpret_cast<grid_map::Matrix::Scalar*>(mat_cv_filtered.data),
                                                          mat.rows(),
                                                          mat.cols());

  return mat_filtered;
}



grid_map::Matrix art_planner::erodeMatrix(const grid_map::Matrix& mat, int size) {
  cv::Mat mat_cv(mat.cols(),
                 mat.rows(),
                 cv::DataType<grid_map::Matrix::Scalar>::type,
                 const_cast<void*>(static_cast<const void*>(mat.data())));

  cv::Mat kernel = getCircularKernel(size);
  cv::Mat mat_cv_filtered;

  cv::erode(mat_cv, mat_cv_filtered, kernel);

  Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
      Eigen::Dynamic,
      Eigen::Dynamic> > mat_filtered(reinterpret_cast<grid_map::Matrix::Scalar*>(mat_cv_filtered.data),
                                     mat.rows(),
                                     mat.cols());

  return mat_filtered;
}



grid_map::Matrix art_planner::dilateMatrix(const grid_map::Matrix& mat, int size) {
  cv::Mat mat_cv(mat.cols(),
                 mat.rows(),
                 cv::DataType<grid_map::Matrix::Scalar>::type,
                 const_cast<void*>(static_cast<const void*>(mat.data())));

  cv::Mat kernel = getCircularKernel(size);
  cv::Mat mat_cv_filtered;

  cv::dilate(mat_cv, mat_cv_filtered, kernel);

  Eigen::Map<Eigen::Matrix<grid_map::Matrix::Scalar,
      Eigen::Dynamic,
      Eigen::Dynamic> > mat_filtered(reinterpret_cast<grid_map::Matrix::Scalar*>(mat_cv_filtered.data),
                                     mat.rows(),
                                     mat.cols());

  return mat_filtered;
}



void art_planner::estimateNormals(grid_map::GridMap& map,
                                  double estimation_radius,
                                  const std::string& input_layer_name,
                                  const std::string& output_layer_prefix) {
  const std::string normal_x = output_layer_prefix + "_x";
  const std::string normal_y = output_layer_prefix + "_y";
  const std::string normal_z = output_layer_prefix + "_z";
  const std::string plane_fit_std_dev = "plane_fit_std_dev";
  map.add(normal_x);
  map.add(normal_y);
  map.add(normal_z);
  map.add(plane_fit_std_dev);
  const int estimation_radius_cells = estimation_radius / map.getResolution();
  const int estimation_radius_diag_cells = estimation_radius * 0.70710678118 / map.getResolution();

  auto& input_layer = map.get(input_layer_name);
  auto& normal_x_layer = map.get(normal_x);
  auto& normal_y_layer = map.get(normal_y);
  auto& normal_z_layer = map.get(normal_z);
  auto& std_layer = map.get(plane_fit_std_dev);

  // Build matrix of cell positions.
  grid_map::Position pos;
  Eigen::Matrix<grid_map::DataType, 3, Eigen::Dynamic> map_3d(3, input_layer.size());
  std::vector<std::pair<Eigen::Index, Eigen::Index> > lin_to_2d_index(input_layer.size());
  std::vector<std::vector<Eigen::Index> > lin_from_2d_index(input_layer.rows(), std::vector<Eigen::Index>(input_layer.cols()));
  Eigen::Index lin_index = 0;
  for (Eigen::Index i = 0; i < input_layer.rows(); ++i) {
    for (Eigen::Index j = 0; j < input_layer.cols(); ++j) {
      map.getPosition(grid_map::Index(i,j), pos);
      map_3d(0, lin_index) = pos.x();
      map_3d(1, lin_index) = pos.y();
      map_3d(2, lin_index) = input_layer(i, j);
      lin_from_2d_index[i][j] = lin_index;
      lin_to_2d_index[lin_index++] = std::make_pair(i, j);
    }
  }

  unsigned int n_vec;
  Eigen::Matrix<grid_map::DataType, 3, 1> vec_sum;
  float max_z_diff;
  for (Eigen::Index i = 0; i < input_layer.rows(); ++i) {
    for (Eigen::Index j = 0; j < input_layer.cols(); ++j) {
      vec_sum.setZero();
      n_vec = 0;
      max_z_diff = 0;

      const auto center = map_3d.col(lin_from_2d_index[i][j]);

      for (int offset = 1; offset < estimation_radius_cells; ++offset) {
        const Eigen::Index i_offset = i + offset;
        const Eigen::Index j_offset = j + offset;
        if (i_offset >= input_layer.rows() || j_offset >= input_layer.cols()) continue;
        const auto vec_x = map_3d.col(lin_from_2d_index[i_offset][j]) - center;
        const auto vec_y = map_3d.col(lin_from_2d_index[i][j_offset]) - center;
        if (std::fabs(vec_x.z()) > max_z_diff) max_z_diff = std::fabs(vec_x.z());
        if (std::fabs(vec_y.z()) > max_z_diff) max_z_diff = std::fabs(vec_y.z());
        vec_sum += vec_x.cross(vec_y).normalized();
        ++n_vec;
      }
      for (int offset = 1; offset < estimation_radius_cells; ++offset) {
        const Eigen::Index i_offset = i - offset;
        const Eigen::Index j_offset = j - offset;
        if (i_offset < 0 || j_offset < 0) continue;
        const auto vec_x = map_3d.col(lin_from_2d_index[i_offset][j]) - center;
        const auto vec_y = map_3d.col(lin_from_2d_index[i][j_offset]) - center;
        if (std::fabs(vec_x.z()) > max_z_diff) max_z_diff = std::fabs(vec_x.z());
        if (std::fabs(vec_y.z()) > max_z_diff) max_z_diff = std::fabs(vec_y.z());
        vec_sum += vec_x.cross(vec_y).normalized();
        ++n_vec;
      }
      for (int offset = 1; offset < estimation_radius_diag_cells; ++offset) {
        const Eigen::Index i_offset_1 = i + offset;
        const Eigen::Index j_offset_1 = j + offset;
        const Eigen::Index i_offset_2 = i - offset;
        const Eigen::Index j_offset_2 = j + offset;
        if (i_offset_1 >= input_layer.rows() || j_offset_1 >= input_layer.cols()
            || i_offset_2 < 0 || j_offset_2 >= input_layer.cols()) continue;
        const auto vec_x = map_3d.col(lin_from_2d_index[i_offset_1][j_offset_1]) - center;
        const auto vec_y = map_3d.col(lin_from_2d_index[i_offset_2][j_offset_2]) - center;
        if (std::fabs(vec_x.z()) > max_z_diff) max_z_diff = std::fabs(vec_x.z());
        if (std::fabs(vec_y.z()) > max_z_diff) max_z_diff = std::fabs(vec_y.z());
        vec_sum += vec_x.cross(vec_y).normalized();
        ++n_vec;
      }
      for (int offset = 1; offset < estimation_radius_diag_cells; ++offset) {
        const Eigen::Index i_offset_1 = i - offset;
        const Eigen::Index j_offset_1 = j - offset;
        const Eigen::Index i_offset_2 = i + offset;
        const Eigen::Index j_offset_2 = j - offset;
        if (i_offset_1 < 0 || j_offset_1 < 0
            || i_offset_2 >= input_layer.rows() || j_offset_2 < 0) continue;
        const auto vec_x = map_3d.col(lin_from_2d_index[i_offset_1][j_offset_1]) - center;
        const auto vec_y = map_3d.col(lin_from_2d_index[i_offset_2][j_offset_2]) - center;
        if (std::fabs(vec_x.z()) > max_z_diff) max_z_diff = std::fabs(vec_x.z());
        if (std::fabs(vec_y.z()) > max_z_diff) max_z_diff = std::fabs(vec_y.z());
        vec_sum += vec_x.cross(vec_y).normalized();
        ++n_vec;
      }

      // Average over all vectors.
      if (n_vec > 0) {
        vec_sum.array() /= n_vec;
      }
      std_layer(i, j) = max_z_diff;
      // Set normal layers.
      vec_sum.normalize();
      normal_x_layer(i, j) = vec_sum.x();
      normal_y_layer(i, j) = vec_sum.y();
      normal_z_layer(i, j) = vec_sum.z();
    }
  }

}
