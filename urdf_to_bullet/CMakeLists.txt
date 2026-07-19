cmake_minimum_required(VERSION 3.22)
project(urdf_to_bullet CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(urdf_to_bullet STATIC
    src/urdf_parser.cpp
    src/urdf_to_bullet.cpp
	src/DynamicRobot.cpp
)

target_include_directories(urdf_to_bullet PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # Bullet's CMake predates target-based include propagation, so
    # BulletDynamics/BulletCollision/LinearMath don't carry their headers
    # with them the way tinyxml2 does below.
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/bullet3/src
)

target_link_libraries(urdf_to_bullet PUBLIC
    tinyxml2
    BulletDynamics BulletCollision LinearMath
)
