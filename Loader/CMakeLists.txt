cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(SOURCE_FILES
        DecodedStream.cpp
        HTMLDump.cpp
        LoadError.cpp
        Loadable.cpp
        LoaderLink.cpp
        LoaderSupport.cpp
        RunFileArchiveDirector.cpp
        TokenStream.cpp
)

set (LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(loader ${SOURCE_FILES})
target_link_libraries(loader utility)
target_compile_definitions(loader PUBLIC -D_LINUX_ -Dunix)
