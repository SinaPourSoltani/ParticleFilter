cmake_minimum_required(VERSION 3.12)
project(ParticleFilter)

set(CMAKE_CXX_STANDARD 17)

add_executable(ParticleFilter main.cpp ParticleFilter.cpp ParticleFilter.h Particle.cpp Particle.h ParticleLookUp.cpp ParticleLookUp.h)

find_package( OpenCV REQUIRED )
include_directories(OpenCV_INCLUDE_DIRS)
target_link_libraries(ParticleFilter ${OpenCV_LIBS})