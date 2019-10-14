cmake_minimum_required(VERSION 3.14)
project(RayTracer)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_executable( raytracer raytracer.cpp bvh.h util.h importer/plycpp.h importer/plycpp.cpp)
target_compile_features(raytracer PUBLIC cxx_lambdas)

