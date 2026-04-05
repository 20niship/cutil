#include "doctest.h"
#include <cutil/pool.hpp>

using namespace cutil;

struct Obj {
  int value;
  static int count;
  Obj(int v = 0) : value(v) { ++count; }
  ~Obj() { --count; }
};
int Obj::count = 0;

TEST_SUITE("ObjectPool") {
  TEST_CASE("create returns valid Ref") {
    ObjectPool<Obj, 4> pool;
    auto r = pool.create(42);
    CHECK(r->value == 42);
    CHECK(Obj::count == 1);
    CHECK(pool.size() == 1);
  }

  TEST_CASE("Ref 解放でスロットがプールへ返却される") {
    ObjectPool<Obj, 4> pool;
    {
      auto r = pool.create(1);
      CHECK(pool.size() == 1);
      CHECK(Obj::count == 1);
    }
    CHECK(pool.size() == 0);
    CHECK(Obj::count == 0);
  }

  TEST_CASE("フリーリスト: 削除済みスロットを再利用する") {
    ObjectPool<Obj, 4> pool;
    Obj* first_ptr = nullptr;
    {
      auto r    = pool.create(10);
      first_ptr = r.get();
    }
    // スロットが返却された後に create すると同じアドレスを再利用する
    auto r2 = pool.create(20);
    CHECK(r2.get() == first_ptr);
    CHECK(r2->value == 20);
  }

  TEST_CASE("チャンク境界を跨いだ生成") {
    ObjectPool<Obj, 2> pool; // 1チャンク = 2スロット
    auto a = pool.create(1);
    auto b = pool.create(2);
    auto c = pool.create(3); // 新チャンクに移行
    CHECK(pool.size() == 3);
    CHECK(a->value == 1);
    CHECK(b->value == 2);
    CHECK(c->value == 3);
  }

  TEST_CASE("イテレーター: alive なオブジェクトのみ列挙") {
    ObjectPool<Obj, 4> pool;
    auto a = pool.create(10);
    auto b = pool.create(20);
    auto c = pool.create(30);

    b.reset(); // b を削除

    int sum = 0;
    int cnt = 0;
    for(auto& obj : pool) {
      sum += obj.value;
      ++cnt;
    }
    CHECK(cnt == 2);
    CHECK(sum == 40); // 10 + 30
  }

  TEST_CASE("イテレーター: チャンク跨ぎ") {
    ObjectPool<Obj, 2> pool;
    auto a = pool.create(1);
    auto b = pool.create(2);
    auto c = pool.create(3);
    auto d = pool.create(4);

    b.reset();
    c.reset();

    int sum = 0;
    for(auto& obj : pool) sum += obj.value;
    CHECK(sum == 5); // 1 + 4
  }

  TEST_CASE("clear() で全オブジェクトを破棄") {
    ObjectPool<Obj, 4> pool;
    auto a = pool.create(1);
    auto b = pool.create(2);
    a.reset();
    b.reset();
    pool.clear();
    CHECK(pool.size() == 0);
    CHECK(Obj::count == 0);
  }

  TEST_CASE("空プールのイテレーター") {
    ObjectPool<Obj, 4> pool;
    int cnt = 0;
    for(auto& _ : pool) ++cnt;
    CHECK(cnt == 0);
  }
}
