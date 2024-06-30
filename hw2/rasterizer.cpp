//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"

#include <cmath>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <vector>

rst::pos_buf_id rst::rasterizer::load_positions(
    const std::vector<Eigen::Vector3f>& positions) {
  auto id = get_next_id();
  pos_buf.emplace(id, positions);

  return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(
    const std::vector<Eigen::Vector3i>& indices) {
  auto id = get_next_id();
  ind_buf.emplace(id, indices);

  return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(
    const std::vector<Eigen::Vector3f>& cols) {
  auto id = get_next_id();
  col_buf.emplace(id, cols);

  return {id};
}

Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
  return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v) {
  Eigen::Vector3f P(x, y, 0);

  Eigen::Vector3f PA = _v[0] - P;
  Eigen::Vector3f PB = _v[1] - P;
  Eigen::Vector3f PC = _v[2] - P;
  
  Eigen::Vector3f AB = _v[1] - _v[0];
  Eigen::Vector3f BC = _v[2] - _v[1];
  Eigen::Vector3f CA = _v[0] - _v[2];

  float cross0 = PA.cross(AB).z();
  float cross1 = PB.cross(BC).z();
  float cross2 = PC.cross(CA).z();

  bool result = (cross0 >= 0 && cross1 >= 0 && cross2 >= 0) ||
                (cross0 <= 0 && cross1 <= 0 && cross2 <= 0);

  return result;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y,
                                                            const Vector3f* v) {
  float c1 =
      (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y +
       v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
      (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() +
       v[1].x() * v[2].y() - v[2].x() * v[1].y());
  float c2 =
      (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y +
       v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
      (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() +
       v[2].x() * v[0].y() - v[0].x() * v[2].y());
  float c3 =
      (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y +
       v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
      (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() +
       v[0].x() * v[1].y() - v[1].x() * v[0].y());
  return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer,
                           col_buf_id col_buffer, Primitive type) {
  auto& buf = pos_buf[pos_buffer.pos_id];
  auto& ind = ind_buf[ind_buffer.ind_id];
  auto& col = col_buf[col_buffer.col_id];

  float f1 = (50 - 0.1) / 2.0;
  float f2 = (50 + 0.1) / 2.0;

  Eigen::Matrix4f mvp = projection * view * model;
  for (auto& i : ind) {
    Triangle t;
    Eigen::Vector4f v[] = {mvp * to_vec4(buf[i[0]], 1.0f),
                           mvp * to_vec4(buf[i[1]], 1.0f),
                           mvp * to_vec4(buf[i[2]], 1.0f)};
    // Homogeneous division
    for (auto& vec : v) {
      vec /= vec.w();
    }
    // Viewport transformation
    for (auto& vert : v) {
      vert.x() = 0.5 * width * (vert.x() + 1.0);
      vert.y() = 0.5 * height * (vert.y() + 1.0);
      vert.z() = vert.z() * f1 + f2;
    }

    for (int i = 0; i < 3; ++i) {
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
    }

    auto col_x = col[i[0]];
    auto col_y = col[i[1]];
    auto col_z = col[i[2]];

    t.setColor(0, col_x[0], col_x[1], col_x[2]);
    t.setColor(1, col_y[0], col_y[1], col_y[2]);
    t.setColor(2, col_z[0], col_z[1], col_z[2]);

    rasterize_triangle(t);
  }
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
  auto v = t.toVector4();

  // Find out the bounding box of current triangle.
  float min_x = std::floor(std::min({v[0].x(), v[1].x(), v[2].x()}));
  float max_x = std::ceil(std::max({v[0].x(), v[1].x(), v[2].x()}));
  float min_y = std::floor(std::min({v[0].y(), v[1].y(), v[2].y()}));
  float max_y = std::ceil(std::max({v[0].y(), v[1].y(), v[2].y()}));

  // Iterate through the bounding box
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      std::vector<float> sample_depths(4, std::numeric_limits<float>::infinity());
      int covered_samples = 0;
      Eigen::Vector3f sample_color = {0, 0, 0};

      // Check each sub-pixel (2x2 super-sampling)
      for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
          float sub_x = x + 0.25 + i * 0.5;
          float sub_y = y + 0.25 + j * 0.5;

          if (insideTriangle(sub_x, sub_y, t.v)) {
            auto [alpha, beta, gamma] = computeBarycentric2D(sub_x, sub_y, t.v);
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() +
                                   beta  * v[1].z() / v[1].w() +
                                   gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            int sample_index = i * 2 + j;
            if (z_interpolated < sample_depths[sample_index]) {
              sample_depths[sample_index] = z_interpolated;
              covered_samples++;
              sample_color += t.getColor() / 4.0;  // Average color accumulation
            }
          }
        }
      }

      // If any sub-pixel is covered, we color the pixel
      if (covered_samples > 0) {
        int index = get_index(x, y);
        if (*min_element(sample_depths.begin(), sample_depths.end()) <
            depth_buf[index]) {
          depth_buf[index] =
              *min_element(sample_depths.begin(), sample_depths.end());
          set_pixel(Eigen::Vector3f(x, y, 1), sample_color);
        }
      }
    }
  }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) { model = m; }

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) { view = v; }

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {
  projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
  if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
    std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
  }
  if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
    std::fill(depth_buf.begin(), depth_buf.end(),
              std::numeric_limits<float>::infinity());
  }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y) {
  return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point,
                                const Eigen::Vector3f& color) {
  // old index: auto ind = point.y() + point.x() * width;
  auto ind = (height - 1 - point.y()) * width + point.x();
  frame_buf[ind] = color;
}

Eigen::Vector3f rst::rasterizer::get_pixel(const Eigen::Vector3f& point) {
  auto ind = (height - 1 - point.y()) * width + point.x();
  return frame_buf[ind];
}