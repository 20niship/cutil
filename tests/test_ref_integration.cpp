#include "doctest.h"
#include <functional>
#include <cutil/ref.hpp>
#include <string>

using namespace cutil;

// Example class with Ref support
class MyObject {
public:
  int id;
  std::string name;

  MyObject(int id, const std::string& name) : id(id), name(name) {}

  ~MyObject() {}

  void print_info() const {}

  std::string get_name() const { return name; }

  void set_name(const std::string& n) { name = n; }
};

// Example with enable_ref_from_this
class MyManagedObject : public enable_ref_from_this<MyManagedObject> {
public:
  int value;

  MyManagedObject(int v) : value(v) {}

  ~MyManagedObject() {}

  Ref<MyManagedObject> get_self() { return ref_from_this(); }

  void do_something() {}
};

// Example with WeakPtr
class Observer {
public:
  WeakPtr<MyObject> target;

  Observer() = default;

  void observe(const Ref<MyObject>& obj) { target = obj; }

  bool is_alive() const { return !target.expired(); }

  void check_target() {
    if(auto obj = target.lock()) {
      // target is alive
    }
  }
};

TEST_SUITE("Ref Integration") {
  TEST_CASE("Basic Ref<T> with class") {
    {
      Ref<MyObject> obj = make_ref<MyObject>(1, "TestObj");
      obj->print_info();
      CHECK(obj.use_count() == 1);
    }
  }

  TEST_CASE("Ref<T> with multiple references") {
    {
      Ref<MyObject> obj1 = make_ref<MyObject>(2, "SharedObj");
      Ref<MyObject> obj2 = obj1;
      Ref<MyObject> obj3 = obj1;

      CHECK(obj1.use_count() == 3);
      CHECK(obj2.use_count() == 3);
      CHECK(obj3.use_count() == 3);
    }
  }

  TEST_CASE("enable_ref_from_this basic") {
    {
      Ref<MyManagedObject> managed = make_ref<MyManagedObject>(42);
      managed->do_something();

      Ref<MyManagedObject> self = managed->get_self();
      CHECK(self.use_count() == 2);
      CHECK(managed.use_count() == 2);
      CHECK(self->value == 42);
    }
  }

  TEST_CASE("enable_ref_from_this multiple get_self calls") {
    {
      Ref<MyManagedObject> obj = make_ref<MyManagedObject>(100);

      Ref<MyManagedObject> self1 = obj->get_self();
      Ref<MyManagedObject> self2 = obj->get_self();
      Ref<MyManagedObject> self3 = self1->get_self();

      CHECK(obj.use_count() == 4);
    }
  }

  TEST_CASE("WeakPtr with Observer pattern - alive") {
    {
      Observer observer;

      {
        Ref<MyObject> observable = make_ref<MyObject>(3, "Observable");
        observer.observe(observable);

        CHECK(observer.is_alive());
        observer.check_target();
      }

      CHECK_FALSE(observer.is_alive());
      observer.check_target();
    }
  }

  TEST_CASE("WeakPtr lock after object destroyed") {
    {
      WeakPtr<MyObject> weak;

      {
        Ref<MyObject> obj = make_ref<MyObject>(4, "Temp");
        weak              = obj;

        CHECK_FALSE(weak.expired());

        Ref<MyObject> locked = weak.lock();
        CHECK(locked != nullptr);
      }

      CHECK(weak.expired());

      Ref<MyObject> locked = weak.lock();
      CHECK(locked == nullptr);
    }
  }

  TEST_CASE("Polymorphic usage (compatible with shared_ptr pattern)") {
    {
      Ref<MyManagedObject> holder = make_ref<MyManagedObject>(200);

      std::function<Ref<MyManagedObject>(const Ref<MyManagedObject>&)> get_self_func = [](const Ref<MyManagedObject>& obj) { return obj->get_self(); };

      Ref<MyManagedObject> result = get_self_func(holder);
      CHECK(result.use_count() == 2);
    }
  }

  TEST_CASE("WeakPtr with enable_ref_from_this") {
    {
      Ref<MyManagedObject> obj      = make_ref<MyManagedObject>(300);
      WeakPtr<MyManagedObject> weak = obj->weak_from_this();

      CHECK_FALSE(weak.expired());

      Ref<MyManagedObject> locked = weak.lock();
      CHECK(locked != nullptr);
      CHECK(locked->value == 300);
    }
  }

  TEST_CASE("Conversion and move semantics (nanobind compatibility)") {
    {
      auto create_object = []() -> Ref<MyObject> { return make_ref<MyObject>(5, "CreatedInFunction"); };

      Ref<MyObject> obj = create_object();
      CHECK(obj.use_count() == 1);
    }
  }

  TEST_CASE("get() method (nanobind holder interface compatibility)") {
    {
      Ref<MyManagedObject> ref = make_ref<MyManagedObject>(400);
      MyManagedObject* ptr     = ref.get();

      CHECK(ptr != nullptr);
      CHECK(ptr->value == 400);
    }
  }

  TEST_CASE("Ref with custom deleter") {
    {
      bool deleted = false;
      auto deleter = [&deleted](MyObject* p) {
        deleted = true;
        delete p;
      };

      {
        Ref<MyObject> ptr(new MyObject(6, "CustomDeleter"), deleter);
        CHECK_FALSE(deleted);
      }
      CHECK(deleted);
    }
  }

  TEST_CASE("WeakPtr from Ref construction") {
    {
      Ref<MyObject> ref = make_ref<MyObject>(7, "WeakTest");
      WeakPtr<MyObject> weak(ref);

      CHECK_FALSE(weak.expired());
      CHECK(weak.use_count() == 1);

      Ref<MyObject> locked = weak.lock();
      CHECK(locked != nullptr);
      CHECK(locked->id == 7);
    }
  }

  TEST_CASE("Multiple weak references from same object") {
    {
      Ref<MyObject> obj = make_ref<MyObject>(8, "MultiWeak");

      WeakPtr<MyObject> weak1 = obj;
      WeakPtr<MyObject> weak2 = obj;
      WeakPtr<MyObject> weak3 = obj;

      CHECK_FALSE(weak1.expired());
      CHECK_FALSE(weak2.expired());
      CHECK_FALSE(weak3.expired());

      auto locked1 = weak1.lock();
      auto locked2 = weak2.lock();
      auto locked3 = weak3.lock();

      CHECK(locked1 != nullptr);
      CHECK(locked2 != nullptr);
      CHECK(locked3 != nullptr);
      CHECK(locked1->id == 8);
    }
  }

  TEST_CASE("enable_ref_from_this after copy assignment") {
    {
      Ref<MyManagedObject> obj1 = make_ref<MyManagedObject>(50);
      Ref<MyManagedObject> obj2 = make_ref<MyManagedObject>(60);

      Ref<MyManagedObject> self1 = obj1->get_self();
      Ref<MyManagedObject> self2 = obj2->get_self();

      CHECK(self1->value == 50);
      CHECK(self2->value == 60);
      CHECK(obj1.use_count() == 2);
      CHECK(obj2.use_count() == 2);
    }
  }

  TEST_CASE("WeakPtr copy and move") {
    {
      Ref<MyObject> obj       = make_ref<MyObject>(9, "WeakCopyMove");
      WeakPtr<MyObject> weak1 = obj;

      // Copy
      WeakPtr<MyObject> weak2 = weak1;
      CHECK_FALSE(weak2.expired());

      // Move
      WeakPtr<MyObject> weak3 = std::move(weak1);
      CHECK_FALSE(weak3.expired());

      auto locked = weak3.lock();
      CHECK(locked != nullptr);
      CHECK(locked->id == 9);
    }
  }

  TEST_CASE("reset() clears reference") {
    {
      Ref<MyObject> obj = make_ref<MyObject>(10, "ResetTest");
      CHECK(obj != nullptr);
      CHECK(obj.use_count() == 1);

      obj.reset();
      CHECK(obj == nullptr);
      CHECK(obj.use_count() == 0);
    }
  }

  TEST_CASE("unique() returns true for single reference") {
    {
      Ref<MyObject> obj = make_ref<MyObject>(11, "UniqueTest");
      CHECK(obj.unique());

      Ref<MyObject> obj2 = obj;
      CHECK_FALSE(obj.unique());
      CHECK_FALSE(obj2.unique());

      obj2.reset();
      CHECK(obj.unique());
    }
  }

  TEST_CASE("swap exchanges pointers") {
    {
      Ref<MyObject> obj1 = make_ref<MyObject>(12, "First");
      Ref<MyObject> obj2 = make_ref<MyObject>(13, "Second");

      int id1 = obj1->id;
      int id2 = obj2->id;

      obj1.swap(obj2);

      CHECK(obj1->id == id2);
      CHECK(obj2->id == id1);
    }
  }
}
