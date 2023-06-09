cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(SOURCE_FILES
        CustomMath.cpp
        DMatrix.cpp
        DVector.cpp
        InvalidIndexException.cpp
        RandomGenerator.cpp
        Statistics.cpp)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(math STATIC ${SOURCE_FILES})
target_include_directories(math PUBLIC
        ${LOG4CPLUS_DIRS}
        ${unitslib_INCLUDE_DIRS}
        ${aaesim_INCLUDE_DIRS})

target_link_libraries(math)
target_compile_definitions(math PUBLIC -D_LINUX_ -Dunix)
