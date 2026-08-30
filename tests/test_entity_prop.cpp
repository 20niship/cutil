#include "doctest.h"
#include "entity.hpp"
#include <cutil/prop.hpp>

using namespace cutil;

TEST_SUITE("Entity - Prop dump/load_to via get_propinfo") {
  TEST_CASE("Mesh::get_propinfo round-trips vertex_count") {
    auto mesh_a = Mesh::Create(42);
    auto mesh_b = Mesh::Create(0);

    Prop p;
    CHECK(p.dump(mesh_a.get(), Mesh::get_propinfo()));
    CHECK(p.load_to(mesh_b.get(), Mesh::get_propinfo()));

    CHECK(mesh_b->vertex_count == 42);
  }

  TEST_CASE("Model::get_propinfo round-trips name/position (POD/Str fields)") {
    auto a         = Model::Create("original");
    a->position[0] = 1.0f;
    a->position[1] = 2.0f;
    a->position[2] = 3.0f;

    Prop p;
    CHECK(p.dump(a.get(), Model::get_propinfo()));

    auto b = Model::Create("to be overwritten");
    CHECK(p.load_to(b.get(), Model::get_propinfo()));

    CHECK(b->name == "original");
    CHECK(b->position[0] == doctest::Approx(1.0f));
    CHECK(b->position[1] == doctest::Approx(2.0f));
    CHECK(b->position[2] == doctest::Approx(3.0f));
  }

  TEST_CASE("Model::get_propinfo round-trips parent (WeakPtr) as a live reference") {
    auto parent = Model::Create("parent");
    auto child  = Model::Create("child");
    parent->add_child(child);

    Prop p;
    CHECK(p.dump(child.get(), Model::get_propinfo()));

    auto child_copy = Model::Create("child_copy");
    CHECK(p.load_to(child_copy.get(), Model::get_propinfo()));

    auto locked = child_copy->parent.lock();
    REQUIRE(locked);
    CHECK(locked.get() == parent.get()); // 実体は共有され、生きたオブジェクトを指す
  }

  TEST_CASE("Model::get_propinfo round-trips meshes/children (RefList) as live references") {
    auto model = Model::Create("model");
    auto m1    = Mesh::Create(10);
    auto m2    = Mesh::Create(20);
    model->add_mesh(m1);
    model->add_mesh(m2);

    auto child1 = Model::Create("child1");
    auto child2 = Model::Create("child2");
    model->add_child(child1);
    model->add_child(child2);

    Prop p;
    CHECK(p.dump(model.get(), Model::get_propinfo()));

    auto model_copy = Model::Create("model_copy");
    CHECK(p.load_to(model_copy.get(), Model::get_propinfo()));

    REQUIRE(model_copy->meshes.size() == 2);
    CHECK(model_copy->meshes[0].get() == m1.get());
    CHECK(model_copy->meshes[1].get() == m2.get());

    REQUIRE(model_copy->children.size() == 2);
    CHECK(model_copy->children[0].get() == child1.get());
    CHECK(model_copy->children[1].get() == child2.get());
  }

  TEST_CASE("parent reference becomes null after load_to when the original parent is not set") {
    auto orphan = Model::Create("orphan");

    Prop p;
    CHECK(p.dump(orphan.get(), Model::get_propinfo()));

    auto other = Model::Create("other");
    other->add_child(Model::Create("someone_elses_child")); // otherに元々何か入っていても上書きされることを確認
    CHECK(p.load_to(other.get(), Model::get_propinfo()));

    CHECK(!other->parent.lock());
  }
}

TEST_SUITE("Entity - Model merged Model3D fields (pos/scale/quat/path/bbox/color)") {
  TEST_CASE("round-trip preserves all POD fields") {
    auto a       = Model::Create("a");
    a->pos       = Vec3f(1, 2, 3);
    a->scale     = Vec3f(4, 5, 6);
    a->quat      = Quat<float>(0, 0, 0, 1);
    a->visible   = true;
    a->animate   = false;
    a->bbox      = Rect3D(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
    a->color     = Vec4f(0.1f, 0.2f, 0.3f, 1.0f);

    Prop p;
    CHECK(p.dump(a.get(), Model::get_propinfo()));

    auto b = Model::Create("b");
    CHECK(p.load_to(b.get(), Model::get_propinfo()));

    CHECK(b->pos.data[0] == doctest::Approx(1.0f));
    CHECK(b->pos.data[2] == doctest::Approx(3.0f));
    CHECK(b->scale.data[1] == doctest::Approx(5.0f));
    CHECK(b->quat.w == doctest::Approx(1.0f));
    CHECK(b->visible == true);
    CHECK(b->animate == false);
    CHECK(b->bbox.x.max == doctest::Approx(1.0f));
    CHECK(b->color.data[3] == doctest::Approx(1.0f));
  }

  TEST_CASE("round-trip preserves Path and Str (pointer fields)") {
    auto a  = Model::Create("model name that will exceed sso capacity for sure");
    a->path = Path("models/character.fbx");

    Prop p;
    CHECK(p.dump(a.get(), Model::get_propinfo()));

    auto b = Model::Create("placeholder");
    CHECK(p.load_to(b.get(), Model::get_propinfo()));

    CHECK(b->path.str() == "models/character.fbx");
    CHECK(b->name == "model name that will exceed sso capacity for sure");
  }

  TEST_CASE("dump then modify Prop does not affect original struct") {
    auto a = Model::Create("original");

    Prop p;
    p.dump(a.get(), Model::get_propinfo());
    p.get<Str>("name") = Str("modified in prop");

    CHECK(a->name == "original");
  }

  TEST_CASE("load_to into an already-populated struct overwrites pointer fields correctly") {
    auto a = Model::Create("first value that is definitely long enough to be heap allocated");
    Prop p;
    p.dump(a.get(), Model::get_propinfo());

    auto b = Model::Create("pre-existing value in b that should be replaced without leaking");
    CHECK(p.load_to(b.get(), Model::get_propinfo()));
    CHECK(b->name == "first value that is definitely long enough to be heap allocated");
  }

  TEST_CASE("re-dump same struct into same Prop (already has fields) updates values") {
    auto a = Model::Create("v1");
    Prop p;
    p.dump(a.get(), Model::get_propinfo());
    CHECK(p.field_count() == Model::get_propinfo()->fields.size());

    a->name = Str("v2, now long enough to require heap allocation for this string");
    p.dump(a.get(), Model::get_propinfo());
    CHECK(p.field_count() == Model::get_propinfo()->fields.size()); // フィールド数は増えない(上書き)

    auto b = Model::Create("b");
    CHECK(p.load_to(b.get(), Model::get_propinfo()));
    CHECK(b->name == "v2, now long enough to require heap allocation for this string");
  }

  TEST_CASE("load_to reports incomplete when rule references a missing Prop field") {
    Prop p; // 何もdumpしていない空のProp
    auto b       = Model::Create("b");
    bool complete = p.load_to(b.get(), Model::get_propinfo());
    CHECK(!complete);
  }
}

TEST_SUITE("Entity - Mesh/Model::dump()/load() wrappers") {
  TEST_CASE("Mesh::dump()/load() round-trips vertex_count") {
    auto mesh_a = Mesh::Create(7);
    Prop p      = mesh_a->dump();

    auto mesh_b = Mesh::Create(0);
    CHECK(mesh_b->load(p));
    CHECK(mesh_b->vertex_count == 7);
  }

  TEST_CASE("Model::dump()/load() round-trips POD/Str fields and references") {
    auto parent        = Model::Create("parent");
    auto model         = Model::Create("model");
    auto mesh          = Mesh::Create(5);
    model->position[0] = 9.0f;
    model->add_mesh(mesh);
    parent->add_child(model);

    Prop p = model->dump();

    auto restored = Model::Create("placeholder");
    CHECK(restored->load(p));

    CHECK(restored->name == "model");
    CHECK(restored->position[0] == doctest::Approx(9.0f));
    REQUIRE(restored->meshes.size() == 1);
    CHECK(restored->meshes[0].get() == mesh.get());
    auto locked = restored->parent.lock();
    REQUIRE(locked);
    CHECK(locked.get() == parent.get());
  }
}
