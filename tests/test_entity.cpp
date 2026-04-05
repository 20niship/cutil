#include "doctest.h"
#include "entity.hpp"

using namespace cutil;

TEST_SUITE("Entity System") {
  TEST_CASE("Mesh::Create returns valid Ref") {
    auto ref = Mesh::Create(100);
    CHECK(ref);
    CHECK(ref->vertex_count == 100);
  }

  TEST_CASE("Model::Create returns valid Ref") {
    auto ref = Model::Create("test_model");
    CHECK(ref);
    CHECK(ref->name == "test_model");
  }

  TEST_CASE("Scene::Create returns valid Ref") {
    auto ref = Scene::Create("test_scene");
    CHECK(ref);
    CHECK(ref->name == "test_scene");
  }

  TEST_CASE("Model parent-child relationship") {
    auto parent = Model::Create("parent");
    CHECK(parent);

    auto child = Model::Create("child");
    CHECK(child);

    parent->add_child(child);
    CHECK(parent->children.size() == 1);
    CHECK(parent->children[0] == child);
    CHECK(child->parent == parent);
  }

  TEST_CASE("Model add_mesh") {
    auto model = Model::Create("model");
    CHECK(model);

    auto mesh = Mesh::Create(50);
    CHECK(mesh);

    model->add_mesh(mesh);
    CHECK(model->meshes.size() == 1);
    CHECK(model->meshes[0] == mesh);
  }

  TEST_CASE("Model remove_child") {
    auto parent = Model::Create("parent");
    auto child  = Model::Create("child");

    parent->add_child(child);
    CHECK(parent->children.size() == 1);

    parent->remove_child(child);
    CHECK(parent->children.size() == 0);
    CHECK(child->parent == nullptr);
  }

  TEST_CASE("Scene add_model") {
    auto scene = Scene::Create("scene");
    CHECK(scene);

    auto model = Model::Create("model");
    CHECK(model);

    scene->add_model(model);
    CHECK(scene->root_models.size() == 1);
    CHECK(scene->root_models[0] == model);
  }

  TEST_CASE("Complex hierarchy") {
    auto scene      = Scene::Create("main_scene");
    auto root1      = Model::Create("root1");
    auto child1     = Model::Create("child1");
    auto child2     = Model::Create("child2");
    auto grandchild = Model::Create("grandchild");

    scene->add_model(root1);
    root1->add_child(child1);
    root1->add_child(child2);
    child1->add_child(grandchild);

    CHECK(scene->root_models.size() == 1);
    CHECK(root1->children.size() == 2);
    CHECK(child1->children.size() == 1);

    // Check references
    CHECK(child1->parent == root1);
    CHECK(child2->parent == root1);
    CHECK(grandchild->parent == child1);
  }

  TEST_CASE("WeakPtr expiration") {
    WeakPtr<Model> weak;
    {
      auto ref  = make_ref<Model>();
      ref->name = "temp";
      weak      = ref->weak_from_this();
      CHECK(!weak.expired());
    }
    CHECK(weak.expired());
  }
}
