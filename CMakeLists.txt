cmake_minimum_required(VERSION 3.23)

project(Ceren)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_subdirectory(vendored/SDL EXCLUDE_FROM_ALL)
add_subdirectory(vendored/box2d EXCLUDE_FROM_ALL)

add_executable(Ceren 
  ${SOURCES}
  vendored/imgui/imgui.cpp
  vendored/imgui/imgui_draw.cpp
  vendored/imgui/imgui_demo.cpp
  vendored/imgui/imgui_tables.cpp
  vendored/imgui/imgui_widgets.cpp
  vendored/imgui/backends/imgui_impl_sdl3.cpp
  vendored/imgui/backends/imgui_impl_sdlrenderer3.cpp
)

target_link_libraries(Ceren PRIVATE SDL3::SDL3)
target_link_libraries(Ceren PRIVATE box2d::box2d)
target_include_directories(Ceren PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/vendored/glm"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/vendored/imgui"
  "${CMAKE_CURRENT_SOURCE_DIR}/vendored/imgui/backends"
)

add_custom_command(TARGET Ceren POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/mufidofufi/runic_weights.bin"
        "$<TARGET_FILE_DIR:Ceren>/runic_weights.bin"
)

target_compile_definitions(Ceren PRIVATE
    RUNIC_WEIGHTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/mufidofufi/runic_weights.bin"
)

