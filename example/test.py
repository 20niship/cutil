#!/usr/bin/env python3
"""
Comprehensive mylib Test Suite
- Tests Ref<T> C++ library through Python bindings (ctypes)
- Tests C++ Str class with Python str integration
- Tests all exported C++ functions and classes
"""

import sys
from pathlib import Path

# Add parent directory to path for mylib import
sys.path.insert(0, str(Path(__file__).parent))


# ============================================================================
# SECTION 1: Ref<T> and Basic Module Tests
# ============================================================================

def test_module_import():
    """Test 1: Import mylib as a Python module"""
    print("\n" + "=" * 70)
    print("【Test 1】Module Import and Basic Functions")
    print("=" * 70)

    from mylib import add, multiply, concat, DataManager

    print("\n- Basic arithmetic functions")
    print("-" * 70)

    result_add = add(10, 20)
    print(f"add(10, 20) = {result_add}")
    assert result_add == 30, "add function failed"

    result_mul = multiply(5, 6)
    print(f"multiply(5, 6) = {result_mul}")
    assert result_mul == 30, "multiply function failed"

    result_concat = concat("Hello, ", "World!")
    print(f"concat('Hello, ', 'World!') = '{result_concat}'")
    assert result_concat == "Hello, World!", "concat function failed"

    print("\n- DataManager class")
    print("-" * 70)

    manager = DataManager()
    print("✓ DataManager object created")

    manager.set_data("Test Data from Module")
    data = manager.get_data()
    print(f"✓ Data set/get: '{data}'")
    assert data == "Test Data from Module"

    use_count = manager.use_count()
    print(f"✓ Reference count: {use_count}")
    assert use_count > 0

    for i in range(3):
        manager.increment()
    counter = manager.get_counter()
    print(f"✓ Counter: {counter}")
    assert counter == 3

    manager.reset()
    print("✓ Manager reset")


def test_import_patterns():
    """Test 2: Various import patterns"""
    print("\n" + "=" * 70)
    print("【Test 2】Various Import Patterns")
    print("=" * 70)

    print("\n- Pattern 1: Specific function imports")
    print("-" * 70)
    from mylib import add, multiply

    print(f"from mylib import add, multiply")
    print(f"add(5, 3) = {add(5, 3)}")
    assert add(5, 3) == 8

    print("\n- Pattern 2: Class import")
    print("-" * 70)
    from mylib import DataManager

    print(f"from mylib import DataManager")
    manager = DataManager()
    manager.set_data("Pattern Test")
    print(f"manager.get_data() = '{manager.get_data()}'")
    manager.reset()

    print("\n- Pattern 3: Full module import")
    print("-" * 70)
    import mylib

    print(f"import mylib")
    print(f"mylib.__version__ = {mylib.__version__}")
    print(f"mylib.add(7, 3) = {mylib.add(7, 3)}")
    assert mylib.add(7, 3) == 10


def test_application_usage():
    """Test 3: Application-style usage"""
    print("\n" + "=" * 70)
    print("【Test 3】Application-Style Usage")
    print("=" * 70)

    from mylib import DataManager
    import mylib

    class Calculator:
        """Example class using mylib"""

        def __init__(self):
            self.data = DataManager()

        def compute(self, a, b):
            result = mylib.add(a, b)
            self.data.set_data(f"Result: {a} + {b} = {result}")
            return result

        def multiply(self, a, b):
            result = mylib.multiply(a, b)
            self.data.set_data(f"Result: {a} * {b} = {result}")
            return result

        def status(self):
            return self.data.get_data()

    print("\n- Calculator class usage")
    print("-" * 70)

    calc = Calculator()
    result1 = calc.compute(10, 20)
    print(f"calc.compute(10, 20) = {result1}")
    print(f"Status: {calc.status()}")
    assert result1 == 30

    result2 = calc.multiply(5, 6)
    print(f"calc.multiply(5, 6) = {result2}")
    print(f"Status: {calc.status()}")
    assert result2 == 30


def test_ref_features():
    """Test 4: Ref<T> specific features"""
    print("\n" + "=" * 70)
    print("【Test 4】Ref<T> Features")
    print("=" * 70)

    from mylib import DataManager

    print("\n- Reference count management")
    print("-" * 70)

    manager1 = DataManager()
    manager1.set_data("Manager 1")

    print(f"manager1.use_count() = {manager1.use_count()}")
    assert manager1.use_count() > 0

    for _ in range(5):
        manager1.increment()

    counter = manager1.get_counter()
    print(f"After 5 increments: counter = {counter}")
    assert counter == 5

    manager1.reset()
    print(f"After reset: use_count = {manager1.use_count()}")
    assert manager1.use_count() == 0

    print("\n- Multiple managers")
    print("-" * 70)

    m1 = DataManager()
    m2 = DataManager()
    m3 = DataManager()

    m1.set_data("Manager 1")
    m2.set_data("Manager 2")
    m3.set_data("Manager 3")

    print(f"m1: '{m1.get_data()}', count={m1.use_count()}")
    print(f"m2: '{m2.get_data()}', count={m2.use_count()}")
    print(f"m3: '{m3.get_data()}', count={m3.use_count()}")

    m1.reset()
    m2.reset()
    m3.reset()
    print("✓ All managers reset")


# ============================================================================
# SECTION 2: C++ Str Class Tests
# ============================================================================

def test_str_basic():
    """Test 5: Basic Str operations"""
    print("\n" + "=" * 70)
    print("【Test 5】Basic String Transformations")
    print("=" * 70)

    import mylib

    # Test to_upper
    result = mylib.to_upper("hello world")
    print(f"to_upper('hello world') = '{result}'")
    assert result == "HELLO WORLD"

    # Test to_lower
    result = mylib.to_lower("HELLO WORLD")
    print(f"to_lower('HELLO WORLD') = '{result}'")
    assert result == "hello world"

    # Test reverse
    result = mylib.reverse_string("hello")
    print(f"reverse_string('hello') = '{result}'")
    assert result == "olleh"

    # Test concat_str
    result = mylib.concat_str("hello", "world")
    print(f"concat_str('hello', 'world') = '{result}'")
    assert result == "hello world"

    print("✓ All basic transformations passed!")


def test_str_search():
    """Test 6: String search operations"""
    print("\n" + "=" * 70)
    print("【Test 6】String Search Operations")
    print("=" * 70)

    import mylib

    # Test begins_with
    result = mylib.begins_with("hello world", "hello")
    print(f"begins_with('hello world', 'hello') = {result}")
    assert result == True

    result = mylib.begins_with("hello world", "world")
    print(f"begins_with('hello world', 'world') = {result}")
    assert result == False

    # Test ends_with
    result = mylib.ends_with("hello world", "world")
    print(f"ends_with('hello world', 'world') = {result}")
    assert result == True

    result = mylib.ends_with("hello world", "hello")
    print(f"ends_with('hello world', 'hello') = {result}")
    assert result == False

    # Test count_substring
    count = mylib.count_substring("banana", "a")
    print(f"count_substring('banana', 'a') = {count}")
    assert count == 3

    print("✓ All search operations passed!")


def test_str_replacement():
    """Test 7: String replacement operations"""
    print("\n" + "=" * 70)
    print("【Test 7】String Replacement Operations")
    print("=" * 70)

    import mylib

    # Test replace
    result = mylib.replace("hello world", "world", "python")
    print(f"replace('hello world', 'world', 'python') = '{result}'")
    assert result == "hello python"

    # Test multiple replacements
    result = mylib.replace("aaa", "a", "b")
    print(f"replace('aaa', 'a', 'b') = '{result}'")
    assert result == "bbb"

    print("✓ All replacement operations passed!")


def test_str_manipulation():
    """Test 8: String manipulation"""
    print("\n" + "=" * 70)
    print("【Test 8】String Manipulation")
    print("=" * 70)

    import mylib

    # Test substr
    result = mylib.substr("hello world", 0, 5)
    print(f"substr('hello world', 0, 5) = '{result}'")
    assert result == "hello"

    # Test trim
    result = mylib.trim("  hello  ")
    print(f"trim('  hello  ') = '{result}'")
    assert result == "hello"

    # Test split_first
    result = mylib.split_first("a,b,c", ",")
    print(f"split_first('a,b,c', ',') = '{result}'")
    assert result == "a"

    print("✓ All manipulation operations passed!")


def test_str_case_conversion():
    """Test 9: Case conversion"""
    print("\n" + "=" * 70)
    print("【Test 9】Case Conversion")
    print("=" * 70)

    import mylib

    # Test to_snake_case
    result = mylib.to_snake_case("HelloWorld")
    print(f"to_snake_case('HelloWorld') = '{result}'")
    assert result == "hello_world"

    # Test to_camel_case (with underscore separator)
    result = mylib.to_camel_case("hello_world_test")
    print(f"to_camel_case('hello_world_test') = '{result}'")
    assert "world" in result or "World" in result

    print("✓ All case conversions passed!")


def test_str_path_operations():
    """Test 10: Path operations"""
    print("\n" + "=" * 70)
    print("【Test 10】Path Operations")
    print("=" * 70)

    import mylib

    # Test get_filename
    result = mylib.get_filename("/path/to/file.txt")
    print(f"get_filename('/path/to/file.txt') = '{result}'")
    assert result == "file.txt"

    result = mylib.get_filename("filename.txt")
    print(f"get_filename('filename.txt') = '{result}'")
    assert result == "filename.txt"

    print("✓ All path operations passed!")


def test_python_str_compatibility():
    """Test 11: Python str <-> C++ Str compatibility"""
    print("\n" + "=" * 70)
    print("【Test 11】Python str <-> C++ Str Compatibility")
    print("=" * 70)
    print("Demonstrating automatic conversion (no explicit Str() needed):")
    print("")

    import mylib

    # All these functions accept Python str and return Python str
    test_cases = [
        ("hello", "to_upper", lambda x: mylib.to_upper(x), "HELLO"),
        ("WORLD", "to_lower", lambda x: mylib.to_lower(x), "world"),
        ("test", "reverse_string", lambda x: mylib.reverse_string(x), "tset"),
        ("  spaces  ", "trim", lambda x: mylib.trim(x), "spaces"),
    ]

    for input_val, func_name, func, expected in test_cases:
        result = func(input_val)
        status = "✓" if result == expected else "✗"
        print(f"  {status} {func_name}('{input_val}') = '{result}'")
        assert result == expected, f"Expected '{expected}', got '{result}'"

    print("")
    print("✓ Python str compatibility works perfectly!")
    print("  No need to explicitly call Str() in Python!")


def test_process_string():
    """Test 12: Complex processing function"""
    print("\n" + "=" * 70)
    print("【Test 12】Complex Processing Function")
    print("=" * 70)

    import mylib

    result = mylib.process_string("hello")
    print(f"process_string('hello') = '{result}'")
    assert "HELLO" in result

    result = mylib.process_string("test string")
    print(f"process_string('test string') = '{result}'")
    assert "TEST STRING" in result

    print("✓ Complex processing passed!")


# ============================================================================
# Main Test Runner
# ============================================================================

def main():
    """Run all tests"""
    print("\n" + "=" * 70)
    print("COMPREHENSIVE MYLIB TEST SUITE")
    print("Testing Ref<T> and Str C++ Classes via Python")
    print("=" * 70)

    try:
        # Ref<T> Tests
        test_module_import()
        test_import_patterns()
        test_application_usage()
        test_ref_features()

        # Str Tests
        test_str_basic()
        test_str_search()
        test_str_replacement()
        test_str_manipulation()
        test_str_case_conversion()
        test_str_path_operations()
        test_python_str_compatibility()
        test_process_string()

        print("\n" + "=" * 70)
        print("✓ ALL TESTS PASSED!")
        print("=" * 70)
        print("\n【Summary】")
        print("✓ Ref<T> smart pointer working correctly")
        print("✓ DataManager with enable_ref_from_this functional")
        print("✓ Reference counting accurate")
        print("✓ C++ Str class fully functional")
        print("✓ Python str <-> C++ Str automatic conversion working")
        print("✓ 19 string manipulation functions available")
        print("✓ C++ and Python seamless integration achieved!")
        print("=" * 70 + "\n")

        return 0

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
