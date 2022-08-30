cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(SOURCE_FILES
        FilePath.cpp
        constants.cpp
        Logging.cpp)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(utility STATIC ${SOURCE_FILES})
target_include_directories(utility PUBLIC
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(utility log4cplus)
target_compile_definitions(utility PUBLIC -D_LINUX_ -Dunix)