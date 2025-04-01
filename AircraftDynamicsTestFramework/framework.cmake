if(NOT ${BUILD_LIBARIES_ONLY})
    add_subdirectory(${FRAMEWORK_DIR})

    SET(FMACM_MAIN_SRC ${FRAMEWORK_DIR}/fmacm.cpp)

    add_executable(FMACM ${FMACM_MAIN_SRC})
    target_link_libraries(FMACM framework)
    target_include_directories(FMACM PUBLIC ${aaesim_INCLUDE_DIRS})
    set_target_properties(FMACM PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
    )
endif()