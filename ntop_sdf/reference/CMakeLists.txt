cmake_minimum_required(VERSION 3.20)
project(reference_bench LANGUAGES CXX)

set(REFERENCE_SOURCE "" CACHE PATH "Path to the unmodified sdf-dcd-demo checkout")
if(NOT EXISTS "${REFERENCE_SOURCE}/source/solver.cpp")
  message(FATAL_ERROR "Set REFERENCE_SOURCE to the sdf-dcd-demo checkout")
endif()

add_executable(reference_bench
  main.cpp
  "${REFERENCE_SOURCE}/source/collide.cpp"
  "${REFERENCE_SOURCE}/source/optimize.cpp"
  "${REFERENCE_SOURCE}/source/solver.cpp"
  "${REFERENCE_SOURCE}/source/shapes.cpp"
  "${REFERENCE_SOURCE}/source/mesh.cpp")
target_include_directories(reference_bench PRIVATE "${REFERENCE_SOURCE}/source")
target_compile_features(reference_bench PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(reference_bench PRIVATE /fp:fast)
endif()
