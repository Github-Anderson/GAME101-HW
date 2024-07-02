#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << '\n';
    control_points.emplace_back(x, y);
  }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
  auto &p_0 = points[0];
  auto &p_1 = points[1];
  auto &p_2 = points[2];
  auto &p_3 = points[3];

  for (double t = 0.0; t <= 1.0; t += 0.001) {
    auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

    // Apply anti-aliasing
    int x = cv::borderInterpolate(static_cast<int>(point.x), window.cols, cv::BORDER_REPLICATE);
    int y = cv::borderInterpolate(static_cast<int>(point.y), window.rows, cv::BORDER_REPLICATE);
    window.at<cv::Vec3b>(y, x)[2] = 255;
  }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) {
  std::vector<cv::Point2f> new_control_points;
  for (auto it = control_points.begin(); it != control_points.end() - 1; ++it) {
    new_control_points.emplace_back(t * (*it) + (1 - t) * (*(it + 1)));
  }
  if (new_control_points.size() >= 2) return recursive_bezier(new_control_points, t);
  return new_control_points[0];
}

void draw_aa_point(cv::Mat &window, cv::Point2f point, cv::Vec3b color) {
  int x = static_cast<int>(point.x);
  int y = static_cast<int>(point.y);
  float alpha_x = point.x - x;
  float alpha_y = point.y - y;

  auto blend = [](cv::Vec3b &dst, cv::Vec3b src, float alpha) {
    for (int i = 0; i < 3; ++i) {
      dst[i] = static_cast<uchar>(dst[i] * (1 - alpha) + src[i] * alpha);
    }
  };

  blend(window.at<cv::Vec3b>(y, x), color, (1 - alpha_x) * (1 - alpha_y));
  blend(window.at<cv::Vec3b>(y, x + 1), color, alpha_x * (1 - alpha_y));
  blend(window.at<cv::Vec3b>(y + 1, x), color, (1 - alpha_x) * alpha_y);
  blend(window.at<cv::Vec3b>(y + 1, x + 1), color, alpha_x * alpha_y);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window, bool antialiasing) {
  for (double t = 0.0; t <= 1.0; t += 0.001) {
    auto point = recursive_bezier(control_points, t);
    if (antialiasing) {
      draw_aa_point(window, point, {0, 255, 0});
    } else {
      window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
  }
}

int main(int argc, char **argv) {
  bool antialiasing = (argc > 1 && std::string(argv[1]) == "aa");

  cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
  cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
  cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

  cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

  int key = -1;
  while (key != 27) {
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    for (auto &point : control_points) {
      cv::circle(window, point, 3, {255, 255, 255}, 3);
    }

    if (control_points.size() >= 3) {
      bezier(control_points, window, antialiasing);
      cv::imwrite("my_bezier_curve.png", window);
    }

    if (control_points.size() == 4 && !antialiasing) {
      naive_bezier(control_points, window);
      cv::imwrite("my_bezier_curve.png", window);
    }
    
    cv::imshow("Bezier Curve", window);
    key = cv::waitKey(20);
  }

  return 0;
}