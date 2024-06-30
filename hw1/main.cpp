#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "rasterizer.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0],
               0, 1, 0, -eye_pos[1],
               0, 0, 1, -eye_pos[2],
               0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
  Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

  float radians = rotation_angle * MY_PI / 180.0f;

  model(0, 0) = cos(radians);
  model(0, 1) = -sin(radians);
  model(1, 0) = sin(radians);
  model(1, 1) = cos(radians);

  return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

  float fov_rad = eye_fov * MY_PI / 180.0f;
  float t = tan(fov_rad / 2) * zNear;
  float r = t * aspect_ratio;
  float l = -r;
  float b = -t;

  projection(0, 0) = 2 * zNear / (r - l);
  projection(1, 1) = 2 * zNear / (t - b);
  projection(2, 2) = -(zFar + zNear) / (zFar - zNear);
  projection(2, 3) = -2 * zFar * zNear / (zFar - zNear);
  projection(3, 2) = -1;
  projection(3, 3) = 0;

  return projection;
}

Eigen::Matrix4f get_rotation_matrix(Vector3f axis, float angle) {
  Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

  axis.normalize();

  float radians = angle * MY_PI / 180.0f;

  Eigen::Matrix3f tmp = Eigen::AngleAxisf(radians, axis).toRotationMatrix();

  rotation.block<3, 3>(0, 0) = tmp;

  return rotation;
}

int main(int argc, const char **argv) {
  float angle = 0;
  bool command_line = false;
  std::string filename = "output.png";

  if (argc >= 3) {
    command_line = true;
    angle = std::stof(argv[2]);  // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    }
  }

  rst::rasterizer r(700, 700);

  Eigen::Vector3f eye_pos = {0, 0, 5};

  std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

  std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

  auto pos_id = r.load_positions(pos);
  auto ind_id = r.load_indices(ind);

  int key = 0;
  int frame_count = 0;

  if (command_line) {
    std::cout << argv[2] << std::endl;

    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite(filename, image);

    return 0;
  }

  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);

    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << '\n';

    if (key == 'a') {
      angle += 10;
    } else if (key == 'd') {
      angle -= 10;
    }
  }

  return 0;
}
