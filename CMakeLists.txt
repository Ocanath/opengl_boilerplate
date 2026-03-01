cmake_minimum_required(VERSION 3.22)
project(opengl_boilerplate C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ImGui (no CMakeLists.txt, add manually)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    # ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)


# byte-stuffing (provides: cobs, PPP)
add_subdirectory(external/byte-stuffing)


# dartt-protocol (provides: dartt_protocol, dartt_checksum)
add_subdirectory(external/dartt-protocol)

#add bullet!
add_subdirectory(external/bullet3)

set(APP_SOURCES
    src/main.cpp
)


if(WIN32)
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})

    # tinycsocket socket libraries
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 iphlpapi)

else()
    # Linux
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})
endif()

# Common libraries for all platforms
target_link_libraries(${APP_TARGET}
    imgui
    cobs
    PPP
    dartt_protocol
    dartt_checksum
	LinearMath
	Bullet3Common
	BulletInverseDynamics
	BulletCollision
	BulletDynamics
	BulletSoftBody
)
