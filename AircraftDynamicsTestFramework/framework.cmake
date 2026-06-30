add_subdirectory(${FRAMEWORK_DIR})

if(FMACM_BUILD_EXECUTABLE)
    SET(FMACM_MAIN_SRC ${FRAMEWORK_DIR}/fmacm.cpp)

    add_executable(FMACM ${FMACM_MAIN_SRC})
    add_executable(mitre::oss::fmacm ALIAS FMACM)
    target_link_libraries(FMACM framework)
    target_compile_definitions(FMACM PRIVATE "FMACM_VERSION=\"${FMACM_VERSION}\"")
    target_include_directories(FMACM PUBLIC 
        $<BUILD_INTERFACE:${aaesim_INCLUDE_DIRS}>
    )
    set_target_properties(FMACM PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
    )
else()
    # Ensure framework library is a build target even though nothing depends on it
    set_target_properties(framework PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()
