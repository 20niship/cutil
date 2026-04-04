#include "doctest.h"
#include <cutil/ref.hpp>

using namespace cutil;

// Test class
class TestObject {
public:
    int value;
    static int instance_count;

    TestObject(int v = 0) : value(v) {
        instance_count++;
    }

    ~TestObject() {
        instance_count--;
    }

    void print() const {}
};

int TestObject::instance_count = 0;

// Test enable_ref_from_this
class MyClass : public enable_ref_from_this<MyClass> {
public:
    int id;

    MyClass(int i) : id(i) {}

    ~MyClass() {}

    Ref<MyClass> get_self() {
        return ref_from_this();
    }
};

TEST_SUITE("Ref<T>") {
    TEST_CASE("Basic construction and destruction") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(42);
            CHECK(ptr1.use_count() == 1);
            CHECK(ptr1->value == 42);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Copy constructor") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(100);
            CHECK(ptr1.use_count() == 1);

            Ref<TestObject> ptr2 = ptr1;
            CHECK(ptr1.use_count() == 2);
            CHECK(ptr2.use_count() == 2);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Move semantics") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(200);
            CHECK(ptr1.use_count() == 1);

            Ref<TestObject> ptr2 = std::move(ptr1);
            CHECK(ptr1 == nullptr);
            CHECK(ptr2.use_count() == 1);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Copy assignment") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(250);
            Ref<TestObject> ptr2 = make_ref<TestObject>(260);

            ptr1 = ptr2;
            CHECK(ptr1->value == 260);
            CHECK(ptr1.use_count() == 2);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Move assignment") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(270);
            Ref<TestObject> ptr2 = make_ref<TestObject>(280);

            int id1 = ptr1.get()->value;
            ptr1 = std::move(ptr2);
            CHECK(ptr2 == nullptr);
            CHECK(ptr1->value != id1);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Dereference operators") {
        Ref<TestObject> ptr = make_ref<TestObject>(300);
        CHECK((*ptr).value == 300);
        CHECK(ptr->value == 300);
    }

    TEST_CASE("Null comparison") {
        Ref<TestObject> ptr;
        CHECK(ptr == nullptr);
        CHECK(ptr.get() == nullptr);

        Ref<TestObject> ptr2 = make_ref<TestObject>(350);
        CHECK(ptr2 != nullptr);
    }

    TEST_CASE("Explicit bool conversion") {
        Ref<TestObject> empty;
        Ref<TestObject> valid = make_ref<TestObject>(400);

        CHECK_FALSE(static_cast<bool>(empty));
        CHECK(static_cast<bool>(valid));
    }

    TEST_CASE("Reset functionality") {
        {
            Ref<TestObject> ptr = make_ref<TestObject>(700);
            CHECK(ptr != nullptr);

            ptr.reset();
            CHECK(ptr == nullptr);
            CHECK(ptr.use_count() == 0);

            ptr.reset(new TestObject(800));
            CHECK(ptr != nullptr);
            CHECK(ptr->value == 800);
            CHECK(ptr.use_count() == 1);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Unique check") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(900);
            CHECK(ptr1.unique());

            Ref<TestObject> ptr2 = ptr1;
            CHECK_FALSE(ptr1.unique());
            CHECK_FALSE(ptr2.unique());

            ptr2.reset();
            CHECK(ptr1.unique());
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Swap") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(1000);
            Ref<TestObject> ptr2 = make_ref<TestObject>(1100);

            int val1 = ptr1->value;
            int val2 = ptr2->value;

            ptr1.swap(ptr2);

            CHECK(ptr1->value == val2);
            CHECK(ptr2->value == val1);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("Comparison operators") {
        Ref<TestObject> ptr1 = make_ref<TestObject>(500);
        Ref<TestObject> ptr2 = ptr1;
        Ref<TestObject> ptr3 = make_ref<TestObject>(600);

        CHECK(ptr1 == ptr2);
        CHECK(ptr1 != ptr3);
        CHECK((ptr1 < ptr3 || ptr1 > ptr3));
    }

    TEST_CASE("Custom deleter") {
        {
            bool deleted = false;
            auto deleter = [&deleted](TestObject* p) {
                deleted = true;
                delete p;
            };

            {
                Ref<TestObject> ptr(new TestObject(400), deleter);
                CHECK_FALSE(deleted);
            }
            CHECK(deleted);
        }
    }
}

TEST_SUITE("WeakPtr<T>") {
    TEST_CASE("WeakPtr from Ref") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(300);
            WeakPtr<TestObject> weak = ptr1;

            CHECK_FALSE(weak.expired());
            CHECK(weak.use_count() == 1);

            Ref<TestObject> ptr2 = weak.lock();
            CHECK(ptr2 != nullptr);
            CHECK(ptr2->value == 300);
            CHECK(ptr2.use_count() == 2);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("WeakPtr expiration") {
        {
            WeakPtr<TestObject> weak;
            {
                Ref<TestObject> ptr1 = make_ref<TestObject>(300);
                weak = ptr1;

                CHECK_FALSE(weak.expired());
            }

            CHECK(weak.expired());

            Ref<TestObject> ptr3 = weak.lock();
            CHECK(ptr3 == nullptr);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("WeakPtr copy constructor") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(350);
            WeakPtr<TestObject> weak1 = ptr1;
            WeakPtr<TestObject> weak2 = weak1;

            CHECK_FALSE(weak2.expired());
            Ref<TestObject> ptr2 = weak2.lock();
            CHECK(ptr2 != nullptr);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("WeakPtr move constructor") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(360);
            WeakPtr<TestObject> weak1 = ptr1;
            WeakPtr<TestObject> weak2 = std::move(weak1);

            CHECK_FALSE(weak2.expired());
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("WeakPtr reset") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(370);
            WeakPtr<TestObject> weak = ptr1;

            CHECK_FALSE(weak.expired());
            weak.reset();
            CHECK(weak.expired());
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("WeakPtr swap") {
        {
            Ref<TestObject> ptr1 = make_ref<TestObject>(380);
            Ref<TestObject> ptr2 = make_ref<TestObject>(390);

            WeakPtr<TestObject> weak1 = ptr1;
            WeakPtr<TestObject> weak2 = ptr2;

            weak1.swap(weak2);

            Ref<TestObject> locked1 = weak1.lock();
            Ref<TestObject> locked2 = weak2.lock();

            CHECK(locked1->value == 390);
            CHECK(locked2->value == 380);
        }
        CHECK(TestObject::instance_count == 0);
    }
}

TEST_SUITE("enable_ref_from_this<T>") {
    TEST_CASE("ref_from_this basic") {
        {
            Ref<MyClass> obj1 = make_ref<MyClass>(1);
            CHECK(obj1.use_count() == 1);

            Ref<MyClass> obj2 = obj1->get_self();
            CHECK(obj1.use_count() == 2);
            CHECK(obj2.use_count() == 2);
            CHECK(obj1->id == obj2->id);
        }
    }

    TEST_CASE("ref_from_this multiple calls") {
        {
            Ref<MyClass> obj1 = make_ref<MyClass>(2);

            Ref<MyClass> obj2 = obj1->get_self();
            Ref<MyClass> obj3 = obj1->get_self();
            Ref<MyClass> obj4 = obj2->get_self();

            CHECK(obj1.use_count() == 4);
            CHECK(obj2.use_count() == 4);
            CHECK(obj3.use_count() == 4);
            CHECK(obj4.use_count() == 4);
        }
    }

    TEST_CASE("weak_from_this") {
        {
            Ref<MyClass> obj1 = make_ref<MyClass>(3);
            WeakPtr<MyClass> weak = obj1->weak_from_this();

            CHECK_FALSE(weak.expired());
            Ref<MyClass> obj2 = weak.lock();
            CHECK(obj2 != nullptr);
            CHECK(obj2->id == obj1->id);
        }
    }
}

TEST_SUITE("make_ref<T>") {
    TEST_CASE("make_ref with single argument") {
        {
            Ref<TestObject> ptr = make_ref<TestObject>(450);
            CHECK(ptr != nullptr);
            CHECK(ptr->value == 450);
            CHECK(ptr.use_count() == 1);
        }
        CHECK(TestObject::instance_count == 0);
    }

    TEST_CASE("make_ref with default constructor") {
        {
            Ref<TestObject> ptr = make_ref<TestObject>();
            CHECK(ptr != nullptr);
            CHECK(ptr->value == 0);
        }
        CHECK(TestObject::instance_count == 0);
    }
}
