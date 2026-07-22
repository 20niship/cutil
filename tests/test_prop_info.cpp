#include "doctest.h"
#include <cutil/prop.hpp>

using cutil::align_up;
using cutil::find_info;
using cutil::has_flag;
using cutil::PropFlags;
using cutil::PropInfo;
using cutil::PropType;
using cutil::PropWidget;
using cutil::validate;
using Data = cutil::PropInfo::Data;

TEST_SUITE("PropInfo - Basic Operations") {
  TEST_CASE("Field sizes do not overflow") {
    Data info;
    CHECK(sizeof(info.name) == 32);
    CHECK(sizeof(info.label) == 64);
    CHECK(sizeof(info.desc) == 256);
    CHECK(sizeof(info.custom_type_name) == 32);
    CHECK(sizeof(info.default_value) == 32);
  }

  TEST_CASE("Construct with name/type/offset/size/is_pointer") {
    Data info("pos", PropType::Vec3, 0, 12, false);
    CHECK(std::string(info.name) == "pos");
    CHECK(info.type == PropType::Vec3);
    CHECK(info.offset == 0);
    CHECK(info.size == 12);
    CHECK(!info.is_pointer);
  }

  TEST_CASE("set_name/set_label/set_desc truncate safely") {
    Data info;
    std::string long_name(100, 'x');
    info.set_name(long_name.c_str());
    CHECK(std::strlen(info.name) == sizeof(info.name) - 1);
  }

  TEST_CASE("find_info by name (linear search)") {
    PropInfo list;
    list.emplace_back("a", PropType::Int, 0, 4, false);
    list.emplace_back("b", PropType::Float, 4, 4, false);
    list.emplace_back("c", PropType::Str, 8, sizeof(void*), true);

    const Data* found = find_info(list, "b");
    REQUIRE(found != nullptr);
    CHECK(found->type == PropType::Float);

    CHECK(find_info(list, "nonexistent") == nullptr);
  }

  TEST_CASE("default_value stores POD values") {
    Data info("x", PropType::Float, 0, sizeof(float), false);
    float v = 3.5f;
    std::memcpy(info.default_value, &v, sizeof(v));
    float readback;
    std::memcpy(&readback, info.default_value, sizeof(readback));
    CHECK(readback == doctest::Approx(3.5f));
  }

  TEST_CASE("PropFlags bit operations") {
    PropFlags f = PropFlags::EditOnly | PropFlags::Hidden;
    CHECK(has_flag(f, PropFlags::EditOnly));
    CHECK(has_flag(f, PropFlags::Hidden));
    CHECK(!has_flag(f, PropFlags::ReadOnly));
  }

  TEST_CASE("validate min/max range") {
    Data info("t", PropType::Float, 0, sizeof(float), false);
    info.min_value = 0.0f;
    info.max_value = 1.0f;
    CHECK(validate(info, 0.5f));
    CHECK(!validate(info, 1.5f));
    CHECK(!validate(info, -0.5f));
  }

  TEST_CASE("validate with unset range (0,0) always passes") {
    Data info("t", PropType::Float, 0, sizeof(float), false);
    CHECK(validate(info, 12345.0f));
  }

  TEST_CASE("align_up rounds up to alignment") {
    CHECK(align_up(0, 4) == 0);
    CHECK(align_up(1, 4) == 4);
    CHECK(align_up(4, 4) == 4);
    CHECK(align_up(5, 8) == 8);
    CHECK(align_up(9, 8) == 16);
  }

  TEST_CASE("prop_type_is_pointer classifies POD vs pointer types") {
    CHECK(!cutil::prop_type_is_pointer(PropType::Bool));
    CHECK(!cutil::prop_type_is_pointer(PropType::Int));
    CHECK(!cutil::prop_type_is_pointer(PropType::Float));
    CHECK(!cutil::prop_type_is_pointer(PropType::Vec3));
    CHECK(!cutil::prop_type_is_pointer(PropType::Vec4));
    CHECK(!cutil::prop_type_is_pointer(PropType::Quat));
    CHECK(!cutil::prop_type_is_pointer(PropType::Range));
    CHECK(!cutil::prop_type_is_pointer(PropType::Rect));
    CHECK(!cutil::prop_type_is_pointer(PropType::Rect3D));
    CHECK(cutil::prop_type_is_pointer(PropType::Str));
    CHECK(cutil::prop_type_is_pointer(PropType::Path));
    CHECK(cutil::prop_type_is_pointer(PropType::Binary));
    CHECK(cutil::prop_type_is_pointer(PropType::Nested));
    CHECK(cutil::prop_type_is_pointer(PropType::Custom));
  }
}
