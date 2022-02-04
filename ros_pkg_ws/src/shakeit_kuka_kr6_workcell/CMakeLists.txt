cmake_minimum_required(VERSION 3.5)

project(shakeit_kuka_kr6_workcell)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
)

install(
    DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
