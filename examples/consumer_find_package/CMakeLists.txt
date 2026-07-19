cmake_minimum_required(VERSION 3.20)
project(nt_consumer CXX)
set(CMAKE_CXX_STANDARD 17)

# Consume navtracker as an installed package (W6.4). Gives the
# navtracker::-namespaced exported targets.
find_package(navtracker REQUIRED)

add_executable(smoke main.cpp)
target_link_libraries(smoke PRIVATE navtracker::navtracker_core)
