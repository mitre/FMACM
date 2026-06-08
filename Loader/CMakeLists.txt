cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(loader STATIC
    DecodedStream.cpp
    FilePath.cpp
    LoadError.cpp
    Loadable.cpp
    LoaderLink.cpp
    LoaderSupport.cpp
    Logging.cpp
    LoggingLoadable.cpp
    RunFileArchiveDirector.cpp
    TokenStream.cpp)
target_link_libraries(loader
        PUBLIC
        log4cplusS
        mitre::cppuom)
target_include_directories(loader PUBLIC
        ${aaesim_INCLUDE_DIRS}
        ${LOG4CPLUS_DIRS}
)
