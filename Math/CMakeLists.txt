cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(SOURCE_FILES
      CustomMath.cpp  
      DMatrix.cpp  
      DVector.cpp  
      RandomGenerator.cpp  
      Statistics.cpp
)

set (LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(math ${SOURCE_FILES})
target_link_libraries(math
        pub)
target_compile_definitions(math PUBLIC -D_LINUX_ -Dunix)