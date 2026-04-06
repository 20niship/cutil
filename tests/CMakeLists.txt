include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# Create test executable
add_executable(tests_main
    main.cpp
    test_rect.cpp
    test_vector.cpp
    test_color.cpp
    test_ref.cpp
    test_ref_integration.cpp
    test_pool.cpp
    test_entity.cpp
    test_entity_benchmark.cpp
    test_vec_new.cpp
    test_mat_new.cpp
    test_variant.cpp
    test_hash_map.cpp
    test_dictionary.cpp
)

# Link doctest header
target_include_directories(tests_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ext)

# Register tests
add_test(NAME tests COMMAND tests_main)
