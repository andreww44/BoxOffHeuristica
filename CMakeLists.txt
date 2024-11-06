cmake_minimum_required(VERSION 3.29)
project(boxoff)

set(CMAKE_CXX_STANDARD 20)

add_executable(boxoff Board.hpp Board.cpp Algoritmos.cpp Algoritmos.hpp main.cpp)
