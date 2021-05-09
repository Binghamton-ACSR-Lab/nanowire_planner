cmake_minimum_required(VERSION 3.17)
project(NanowirePlanner VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)


include_directories(include include/utility include/planner include/observer include/global_route)
find_package(Boost COMPONENTS system program_options filesystem regex REQUIRED)
find_package(Threads)

file(GLOB source_files
        "include/*.h"
        "include/*.hpp"
        "src/*.cpp"
        "include/utility/*.hpp"
        "include/planner/*.hpp"
        "include/observer/*.hpp"
        "include/global_route/*.hpp"
        )

add_executable(NanowirePlanner main.cpp ${source_files})
target_link_libraries( NanowirePlanner
        acado_toolkit_s
        pthread
        ${CMAKE_THREAD_LIBS_INIT}
        #${OpenCV_LIBS}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_THREAD_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_REGEX_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_DATE_TIME_LIBRARY}
        SQLiteCpp sqlite3
        )