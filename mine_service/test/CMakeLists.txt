find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_mine_service test_mine_service.cpp)
target_link_libraries(test_mine_service ${PROJECT_NAME})