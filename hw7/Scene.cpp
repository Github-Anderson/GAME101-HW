//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
  printf(" - Generating BVH...\n\n");
  this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
  return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
  float emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
    }
  }
  float p = get_random_float() * emit_area_sum;
  emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
      if (p <= emit_area_sum) {
        objects[k]->Sample(pos, pdf);
        break;
      }
    }
  }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) {
  *hitObject = nullptr;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    float tNearK = kInfinity;
    uint32_t indexK;
    Vector2f uvK;
    if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
      *hitObject = objects[k];
      tNear = tNearK;
      index = indexK;
    }
  }

  return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  // TODO: Implement Path Tracing Algorithm here
  if (depth > maxDepth) {
    return Vector3f(0.0f);
  }

  Intersection inter = intersect(ray);
  if (!inter.happened) {
    return Vector3f(0.0f);
  }

  if (inter.m->hasEmission()) {
    return inter.m->getEmission();
  }

  Vector3f hitPoint = inter.coords;
  Vector3f N = inter.normal;

  Vector3f L_dir(0.0f);

  Intersection lightInter;
  float pdf_light = 0.0f;
  sampleLight(lightInter, pdf_light);

  Vector3f x = lightInter.coords;
  Vector3f ws = (x - hitPoint).normalized();
  Vector3f NN = lightInter.normal;

  Ray shadowRay(hitPoint, ws);

  Intersection shadowInter = intersect(shadowRay);
  if (shadowInter.distance - (x - hitPoint).norm() > -0.005f) {
    L_dir = lightInter.emit * inter.m->eval(ray.direction, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / dotProduct(x - hitPoint, x - hitPoint) / pdf_light;
  }

  Vector3f L_indir(0.0f);

  if (get_random_float() < RussianRoulette) {
    Vector3f wi = inter.m->sample(ray.direction, N).normalized();
    Ray reflectionRay(hitPoint, wi);
    Intersection reflectionInter = intersect(reflectionRay);

    if (reflectionInter.happened && !reflectionInter.m->hasEmission()) {
      float pdf_bsdf = inter.m->pdf(ray.direction, wi, N);
      if (pdf_bsdf > 0) {
        L_indir = castRay(reflectionRay, depth + 1) * inter.m->eval(ray.direction, wi, N) * dotProduct(wi, N) / pdf_bsdf / RussianRoulette;
      }
    }
  }

  return L_dir + L_indir;
}