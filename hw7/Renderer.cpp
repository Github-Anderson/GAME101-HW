//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"

#include <fstream>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

#include "Scene.hpp"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001f;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene, int spp) {
  std::vector<Vector3f> framebuffer(scene.width * scene.height);
  float scale = tan(deg2rad(scene.fov * 0.5));
  float imageAspectRatio = scene.width / (float)scene.height;
  Vector3f eye_pos(278, 273, -800);
  
  // Change the spp value to change sample amount
  std::cout << "SPP: " << spp << "\n";

  int num_threads = std::thread::hardware_concurrency();
  std::vector<std::thread> threads;
  std::atomic<int> current_line(0);
  std::mutex framebuffer_mutex;

  auto render_task = [&](int thread_id) {
    while (true) {
      int j = current_line.fetch_add(1);
      if (j >= scene.height) break;

      for (uint32_t i = 0; i < scene.width; ++i) {
        float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
        float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
        Vector3f dir = normalize(Vector3f(-x, y, 1));
        Vector3f pixel_color(0, 0, 0);
        for (int k = 0; k < spp; k++) {
          pixel_color += scene.castRay(Ray(eye_pos, dir), 0) / spp;
        }

        {
          std::lock_guard<std::mutex> lock(framebuffer_mutex);
          framebuffer[j * scene.width + i] = pixel_color;
        }
      }
    }
  };

  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(render_task, i);
  }

  while (current_line < scene.height) {
    UpdateProgress(current_line / (float)scene.height);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  for (auto& thread : threads) {
    thread.join();
  }

  UpdateProgress(1.f);

  // Save framebuffer to file
  FILE* fp = fopen("binary.ppm", "wb");
  (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
  for (auto i = 0; i < scene.height * scene.width; ++i) {
    static unsigned char color[3];
    color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
    color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
    color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
    fwrite(color, 1, 3, fp);
  }
  fclose(fp);
}