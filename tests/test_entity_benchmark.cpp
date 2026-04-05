#include "doctest.h"
#include <chrono>
#include <iostream>

#include "entity.hpp"
#include <cutil/pool.hpp>

using namespace cutil;

// ベンチマーク用ヘルパー
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

public:
  Timer() : start_(std::chrono::high_resolution_clock::now()) {}

  long long elapsed_ms() {
    auto now   = std::chrono::high_resolution_clock::now();
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_);
    return delta.count();
  }

  long long elapsed_us() {
    auto now   = std::chrono::high_resolution_clock::now();
    auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);
    return delta.count();
  }
};

TEST_SUITE("Entity System Benchmark") {
  TEST_CASE("ObjectPool<Model> vs new/delete - 100000 objects") {
    constexpr size_t COUNT = 100000;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Model, 256> model_pool;
      {
        std::vector<Ref<Model>> models;
        for(size_t i = 0; i < COUNT; ++i) {
          auto ref = Model::Create("model_" + std::to_string(i), &model_pool);
          if(ref) models.push_back(ref);
        }
        pool_time = t.elapsed_us();
        // models がスコープを抜ける時に全てデストラクト
      }
    }

    // 通常のnew/delete測定
    long long new_time = 0;
    {
      Timer t;
      std::vector<Ref<Model>> models;
      for(size_t i = 0; i < COUNT; ++i) {
        auto ref  = make_ref<Model>();
        ref->name = "model_" + std::to_string(i);
        models.push_back(ref);
      }
      new_time = t.elapsed_us();
    }

    std::cout << "\n=== Model Creation (100000 objects) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "new/delete time: " << new_time << " us\n"
              << "Ratio (pool/new): " << (double)pool_time / new_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(new_time > 0);
  }

  TEST_CASE("ObjectPool<Mesh> vs new/delete - 500000 objects") {
    constexpr size_t COUNT = 500000;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Mesh, 512> mesh_pool;
      {
        std::vector<Ref<Mesh>> meshes;
        for(size_t i = 0; i < COUNT; ++i) {
          auto ref = Mesh::Create(i % 100 + 10, &mesh_pool);
          if(ref) meshes.push_back(ref);
        }
        pool_time = t.elapsed_us();
      }
    }

    // 通常のnew/delete測定
    long long new_time = 0;
    {
      Timer t;
      std::vector<Ref<Mesh>> meshes;
      for(size_t i = 0; i < COUNT; ++i) {
        auto ref          = make_ref<Mesh>();
        ref->vertex_count = i % 100 + 10;
        meshes.push_back(ref);
      }
      new_time = t.elapsed_us();
    }

    std::cout << "\n=== Mesh Creation (500000 objects) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "new/delete time: " << new_time << " us\n"
              << "Ratio (pool/new): " << (double)pool_time / new_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(new_time > 0);
  }

  TEST_CASE("ObjectPool<Scene> vs new/delete - 50000 objects") {
    constexpr size_t COUNT = 50000;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Scene, 128> scene_pool;
      {
        std::vector<Ref<Scene>> scenes;
        for(size_t i = 0; i < COUNT; ++i) {
          auto ref = Scene::Create("scene_" + std::to_string(i), &scene_pool);
          if(ref) scenes.push_back(ref);
        }
        pool_time = t.elapsed_us();
      }
    }

    // 通常のnew/delete測定
    long long new_time = 0;
    {
      Timer t;
      std::vector<Ref<Scene>> scenes;
      for(size_t i = 0; i < COUNT; ++i) {
        auto ref  = make_ref<Scene>();
        ref->name = "scene_" + std::to_string(i);
        scenes.push_back(ref);
      }
      new_time = t.elapsed_us();
    }

    std::cout << "\n=== Scene Creation (50000 objects) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "new/delete time: " << new_time << " us\n"
              << "Ratio (pool/new): " << (double)pool_time / new_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(new_time > 0);
  }

  TEST_CASE("Parent-Child hierarchy - ObjectPool vs new/delete - 10000 parent + 10 children") {
    constexpr size_t PARENT_COUNT = 10000;
    constexpr size_t CHILD_COUNT  = 10;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Model, 256> model_pool;
      {
        std::vector<Ref<Model>> roots;
        for(size_t i = 0; i < PARENT_COUNT; ++i) {
          auto parent = Model::Create("parent_" + std::to_string(i), &model_pool);
          if(parent) {
            for(size_t j = 0; j < CHILD_COUNT; ++j) {
              auto child = Model::Create("child_" + std::to_string(j), &model_pool);
              if(child) parent->add_child(child);
            }
            roots.push_back(parent);
          }
        }
        pool_time = t.elapsed_us();
      }
    }

    // 通常のnew/delete測定
    long long new_time = 0;
    {
      Timer t;
      std::vector<Ref<Model>> roots;
      for(size_t i = 0; i < PARENT_COUNT; ++i) {
        auto parent  = make_ref<Model>();
        parent->name = "parent_" + std::to_string(i);
        for(size_t j = 0; j < CHILD_COUNT; ++j) {
          auto child  = make_ref<Model>();
          child->name = "child_" + std::to_string(j);
          parent->add_child(child);
        }
        roots.push_back(parent);
      }
      new_time = t.elapsed_us();
    }

    std::cout << "\n=== Hierarchy Creation (10000 parents + 10 children each) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "new/delete time: " << new_time << " us\n"
              << "Ratio (pool/new): " << (double)pool_time / new_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(new_time > 0);
  }

  TEST_CASE("Rapid allocation/deallocation - 100000 cycles") {
    constexpr size_t CYCLES = 100000;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Model, 128> model_pool;
      for(size_t i = 0; i < CYCLES; ++i) {
        auto ref = Model::Create("temp", &model_pool);
        (void)ref; // 生成直後に削除
      }
      pool_time = t.elapsed_us();
    }

    // 通常のnew/delete測定
    long long new_time = 0;
    {
      Timer t;
      for(size_t i = 0; i < CYCLES; ++i) {
        auto ref = make_ref<Model>();
        (void)ref; // 生成直後に削除
      }
      new_time = t.elapsed_us();
    }

    std::cout << "\n=== Rapid Allocation/Deallocation (100000 cycles) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "new/delete time: " << new_time << " us\n"
              << "Ratio (pool/new): " << (double)pool_time / new_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(new_time > 0);
  }

  TEST_CASE("Iterator update - position increment (100000 models)") {
    constexpr size_t COUNT = 100000;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Model, 256> model_pool;
      {
        std::vector<Ref<Model>> models;
        for(size_t i = 0; i < COUNT; ++i) {
          auto ref                = Model::Create("model_" + std::to_string(i), &model_pool);
          ref->position[0]        = i * 0.1f;
          ref->position[1]        = i * 0.2f;
          ref->position[2]        = i * 0.3f;
          if(ref) models.push_back(ref);
        }

        // イテレータで要素を更新
        for(auto& model : model_pool) {
          model.position[0] += 1.0f;
          model.position[1] += 1.0f;
          model.position[2] += 1.0f;
        }
        pool_time = t.elapsed_us();
      }
    }

    // 通常のvector測定
    long long vec_time = 0;
    {
      Timer t;
      std::vector<Ref<Model>> models;
      for(size_t i = 0; i < COUNT; ++i) {
        auto ref           = make_ref<Model>();
        ref->name          = "model_" + std::to_string(i);
        ref->position[0]   = i * 0.1f;
        ref->position[1]   = i * 0.2f;
        ref->position[2]   = i * 0.3f;
        models.push_back(ref);
      }

      // イテレータで要素を更新
      for(auto& ref : models) {
        ref->position[0] += 1.0f;
        ref->position[1] += 1.0f;
        ref->position[2] += 1.0f;
      }
      vec_time = t.elapsed_us();
    }

    std::cout << "\n=== Iterator Update - Position Increment (100000 models) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "Vector time:     " << vec_time << " us\n"
              << "Ratio (pool/vec): " << (double)pool_time / vec_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(vec_time > 0);
  }

  TEST_CASE("Iterator update on sparse hierarchy - ObjectPool (50000 parent, iterate children)") {
    constexpr size_t PARENT_COUNT = 50000;
    constexpr size_t CHILD_COUNT  = 5;

    // ObjectPool測定
    long long pool_time = 0;
    {
      Timer t;
      ObjectPool<Model, 256> model_pool;
      {
        std::vector<Ref<Model>> roots;
        for(size_t i = 0; i < PARENT_COUNT; ++i) {
          auto parent = Model::Create("parent_" + std::to_string(i), &model_pool);
          if(parent) {
            for(size_t j = 0; j < CHILD_COUNT; ++j) {
              auto child = Model::Create("child_" + std::to_string(j), &model_pool);
              if(child) parent->add_child(child);
            }
            roots.push_back(parent);
          }
        }

        // イテレータで親の position を更新
        for(auto& model : model_pool) {
          model.position[0] += 0.5f;
          model.position[1] += 0.5f;
          model.position[2] += 0.5f;
        }
        pool_time = t.elapsed_us();
      }
    }

    // Vector測定
    long long vec_time = 0;
    {
      Timer t;
      std::vector<Ref<Model>> roots;
      for(size_t i = 0; i < PARENT_COUNT; ++i) {
        auto parent = make_ref<Model>();
        parent->name = "parent_" + std::to_string(i);
        for(size_t j = 0; j < CHILD_COUNT; ++j) {
          auto child  = make_ref<Model>();
          child->name = "child_" + std::to_string(j);
          parent->add_child(child);
        }
        roots.push_back(parent);
      }

      // Vector のすべての要素を更新（親+子すべてを vector に入れる必要がある）
      std::vector<Ref<Model>> all_models;
      for(auto& root : roots) {
        all_models.push_back(root);
        for(auto& child : root->children) {
          all_models.push_back(child);
        }
      }
      for(auto& model : all_models) {
        model->position[0] += 0.5f;
        model->position[1] += 0.5f;
        model->position[2] += 0.5f;
      }
      vec_time = t.elapsed_us();
    }

    std::cout << "\n=== Sparse Hierarchy Iterator Update (50000 parents + 5 children) ===\n"
              << "ObjectPool time: " << pool_time << " us\n"
              << "Vector time:     " << vec_time << " us\n"
              << "Ratio (pool/vec): " << (double)pool_time / vec_time << "x\n";

    CHECK(pool_time > 0);
    CHECK(vec_time > 0);
  }
}
