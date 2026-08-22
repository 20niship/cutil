#include "doctest.h"
#include <cutil/prop.hpp>

using cutil::align_up;
using cutil::has_flag;
using cutil::PropClass;
using cutil::PropFlags;
using cutil::PropInfo;
using cutil::validate;
using Field = cutil::PropInfo::Field;

TEST_SUITE("PropInfo - Basic Operations") {
  TEST_CASE("Field sizes do not overflow") {
    Field f;
    CHECK(sizeof(f.name) == 32);
    CHECK(sizeof(f.label) == 64);
    CHECK(sizeof(f.desc) == 256);
  }

  TEST_CASE("Construct with name/offset/type") {
    Field f("pos", 0, cutil::prop_info_of<cutil::Vec3f>());
    CHECK(std::string(f.name) == "pos");
    CHECK(f.type == cutil::prop_info_of<cutil::Vec3f>());
    CHECK(f.offset == 0);
  }

  TEST_CASE("set_name/set_label/set_desc truncate safely") {
    Field f;
    std::string long_name(100, 'x');
    f.set_name(long_name.c_str());
    CHECK(std::strlen(f.name) == sizeof(f.name) - 1);
  }

  TEST_CASE("PropInfo::find_field by name (linear search)") {
    PropInfo info = {
      {"a", 0, cutil::prop_info_of<int32_t>()},
      {"b", 4, cutil::prop_info_of<float>()},
      {"c", 8, cutil::prop_info_of<cutil::Str>()},
    };

    const Field* found = info.find_field("b");
    REQUIRE(found != nullptr);
    CHECK(found->type == cutil::prop_info_of<float>());

    CHECK(info.find_field("nonexistent") == nullptr);
  }

  TEST_CASE("PropFlags bit operations") {
    PropFlags f = PropFlags::EditOnly | PropFlags::Hidden;
    CHECK(has_flag(f, PropFlags::EditOnly));
    CHECK(has_flag(f, PropFlags::Hidden));
    CHECK(!has_flag(f, PropFlags::ReadOnly));
  }

  TEST_CASE("validate min/max range") {
    Field f("t", 0, cutil::prop_info_of<float>());
    f.min_value = 0.0f;
    f.max_value = 1.0f;
    CHECK(validate(f, 0.5f));
    CHECK(!validate(f, 1.5f));
    CHECK(!validate(f, -0.5f));
  }

  TEST_CASE("validate with unset range (0,0) always passes") {
    Field f("t", 0, cutil::prop_info_of<float>());
    CHECK(validate(f, 12345.0f));
  }

  TEST_CASE("align_up rounds up to alignment") {
    CHECK(align_up(0, 4) == 0);
    CHECK(align_up(1, 4) == 4);
    CHECK(align_up(4, 4) == 4);
    CHECK(align_up(5, 8) == 8);
    CHECK(align_up(9, 8) == 16);
  }

  TEST_CASE("PropClass classifies Trivial vs Indirect vs Dynamic") {
    CHECK(cutil::prop_info_of<bool>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<int32_t>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<float>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Vec3f>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Vec4f>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Quat<float>>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Range>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Rect>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Rect3D>()->klass == PropClass::Trivial);
    CHECK(cutil::prop_info_of<cutil::Str>()->klass == PropClass::Indirect);
    CHECK(cutil::prop_info_of<cutil::Path>()->klass == PropClass::Indirect);
    CHECK(cutil::prop_info_of<std::vector<uint8_t>>()->klass == PropClass::Indirect);
    CHECK(cutil::prop_info_of<cutil::CustomSlot>()->klass == PropClass::Dynamic);
  }
}
