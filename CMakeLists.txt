cmake_minimum_required(VERSION 3.22)
project(opengl_boilerplate C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)

# GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

# Assimp (OBJ + STL only)
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_OBJ_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_STL_IMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(external/assimp)

# GLM (header-only)
add_subdirectory(external/glm)

# ImGui
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw)

# byte-stuffing (provides: cobs, PPP)
add_subdirectory(external/byte-stuffing)

# dartt-protocol (provides: dartt_protocol, dartt_checksum)
add_subdirectory(external/dartt-protocol)

# Bullet3 (suppress demos/tests)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
add_subdirectory(external/bullet3)

# GLAD (vendored)
add_library(glad STATIC src/glad.c)
target_include_directories(glad PUBLIC src/)

# Copy shaders and assets to build dir at configure time
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(APP_SOURCES
    src/main.cpp
    src/shader.cpp
    src/mesh.cpp
    src/model.cpp
    src/camera.cpp
    src/scene.cpp
    src/collision_box.cpp
    src/ability_beam.cpp
    src/ability_gravity.cpp
)

if(WIN32)
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 iphlpapi)
else()
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})
endif()

target_link_libraries(${APP_TARGET}
    glad OpenGL::GL glfw glm::glm imgui assimp
    cobs PPP dartt_protocol dartt_checksum
    -Wl,--start-group
        LinearMath Bullet3Common BulletInverseDynamics
        BulletCollision BulletDynamics
    -Wl,--end-group
    ${CMAKE_DL_LIBS}
)

target_include_directories(${APP_TARGET} PRIVATE
    src/
    external/bullet3/src
    external/imgui
    external/imgui/backends
)
