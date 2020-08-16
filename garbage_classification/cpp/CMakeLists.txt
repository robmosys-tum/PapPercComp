cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(garbage-collection)

find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(garbage-collection main.cpp)
target_link_libraries(garbage-collection ${TORCH_LIBRARIES} ${OpenCV_LIBRARIES})
set_property(TARGET garbage-collection PROPERTY CXX_STANDARD 14)

# The following code block is suggested to be used on Windows.
# According to https://github.com/pytorch/pytorch/issues/25457,
# the DLLs need to be copied to avoid memory errors.
if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET garbage-collection
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:garbage-collection>)
endif (MSVC)
