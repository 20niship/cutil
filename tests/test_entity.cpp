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

  TEST_CASE("Model parent-child relationship with WeakPtr") {
    auto parent = Model::Create("parent");
    CHECK(parent);

    auto child = Model::Create("child");
    CHECK(child);

    parent->add_child(child);
    CHECK(parent->children.size() == 1);
    CHECK(parent->children[0] == child);

    // parent は WeakPtr なのでロックして比較
    auto parent_locked = child->parent.lock();
    CHECK(parent_locked);
    CHECK(parent_locked == parent);
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
    CHECK(child->parent.expired());  // WeakPtr が expired か確認
  }

  TEST_CASE("Scene add_model") {
    auto scene = Scene::Create("scene");
    CHECK(scene);

    auto model = Model::Create("model");
    CHECK(model);

    scene->add_model(model);
    CHECK(scene->root_models.size() == 1);
    CHECK(scene->root_models[0] == model);
    CHECK(model->parent.expired());  // root なので parent は expired
  }

  TEST_CASE("Complex hierarchy with WeakPtr parent") {
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

    // Check references with WeakPtr locked
    auto child1_parent = child1->parent.lock();
    CHECK(child1_parent == root1);

    auto child2_parent = child2->parent.lock();
    CHECK(child2_parent == root1);

    auto grandchild_parent = grandchild->parent.lock();
    CHECK(grandchild_parent == child1);
  }

  TEST_CASE("Parent deletion affects WeakPtr expiration") {
    WeakPtr<Model> child_parent_weak;
    {
      auto parent = Model::Create("parent");
      auto child  = Model::Create("child");
      parent->add_child(child);

      child_parent_weak = child->parent;

      // parent がまだ存在する間は expired ではない
      CHECK(!child_parent_weak.expired());
      CHECK(child_parent_weak.lock());
    }
    // parent がスコープを抜けて削除されたので expired
    CHECK(child_parent_weak.expired());
  }

  TEST_CASE("Circular reference prevention with WeakPtr") {
    // Ref<Model> parent と WeakPtr<Model> parent を組み合わせることで
    // 親→子の強参照と子→親の弱参照となり、循環参照を防ぐ
    Ref<Model> parent;
    Ref<Model> child;
    {
      parent = Model::Create("parent");
      child  = Model::Create("child");

      // add_child 前の状態
      // parent: 1 (親変数), child: 1 (子変数)
      CHECK(parent.use_count() == 1);
      CHECK(child.use_count() == 1);

      parent->add_child(child);

      // この時点での参照カウント：
      // - parent: 1 (parent変数のみ、parent->children に保存されていない)
      // - child: 2 (child変数 + parent->children[0])
      CHECK(parent.use_count() == 1);  // parent変数のみ
      CHECK(child.use_count() == 2);   // child変数 + parent->children

      // child->parent は WeakPtr なので親の参照カウントを増やさない
      auto parent_locked = child->get_parent();
      CHECK(parent_locked);        // WeakPtr がロック可能
      CHECK(parent_locked == parent);
    }

    // ブロック内の変数は削除されても、外側の parent・child 変数はまだ存在
    CHECK(parent);
    CHECK(child);

    // 子を削除すると、子の参照カウントが減る
    parent->remove_child(child);
    CHECK(child.use_count() == 1);   // parent->children からも削除されたので1に戻る
    CHECK(child->parent.expired());  // WeakPtr が expired
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

  TEST_CASE("get_parent() convenience method") {
    auto parent = Model::Create("parent");
    auto child  = Model::Create("child");

    parent->add_child(child);

    // get_parent() で直接 Ref<Model> を取得できる
    auto parent_ref = child->get_parent();
    CHECK(parent_ref);
    CHECK(parent_ref == parent);
    CHECK(parent_ref->name == "parent");
  }
}
