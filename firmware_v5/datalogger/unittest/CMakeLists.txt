cmake_minimum_required(VERSION 3.14)
project(my_project)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

#file(GLOB sqlite_src
#     "/mnt/c/Users/NSieg/OneDrive/Dokumente/PlatformIO/Projects/Freematics/libraries/Esp32-Arduino-Sqlite/src/*.h"
#     "/mnt/c/Users/NSieg/OneDrive/Dokumente/PlatformIO/Projects/Freematics/libraries/Esp32-Arduino-Sqlite/src/*.cpp"
#     "/mnt/c/Users/NSieg/OneDrive/Dokumente/PlatformIO/Projects/Freematics/libraries/Esp32-Arduino-Sqlite/src/*.c"
#)

#INCLUDE_DIRECTORIES(/mnt/c/Users/NSieg/OneDrive/Dokumente/PlatformIO/Projects/Freematics/libraries/Esp32-Arduino-Sqlite/src)
#LINK_DIRECTORIES(/mnt/c/Users/NSieg/OneDrive/Dokumente/PlatformIO/Projects/Freematics/libraries/Esp32-Arduino-Sqlite/src)
#ADD_LIBRARY(sqlite ${sqlite_src})

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  beacon
  beacon.cc
)
target_link_libraries(
  beacon
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(beacon)