add_subdirectory(${FRAMEWORK_DIR})

if(FMACM_BUILD_EXECUTABLE)
    # CPMAddPackage(
    #     NAME cppmanifest_fmacm
    #     GIT_REPOSITORY https://mustache.mitre.org/scm/idea/cppmanifest.git
    #     GIT_TAG v1.7
    #     DOWNLOAD_ONLY TRUE
    # )
    # if(cppmanifest_fmacm_ADDED AND IS_READABLE "${cppmanifest_fmacm_SOURCE_DIR}")
    #     include(${cppmanifest_fmacm_SOURCE_DIR}/cppmanifest_configure.cmake)
    #     set(PATH_FOR_CPPMANIFEST "${CMAKE_CURRENT_BINARY_DIR}/include/cppmanifest")
    #     cppmanifest_configure(${PATH_FOR_CPPMANIFEST})
    #     set(cppmanifest_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
    # else()
    #     message(FATAL_ERROR "Unable to configure cppmanifest header files: ${cppmanifest_fmacm_SOURCE_DIR}")
    # endif()
    set(cppmanifest_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")

    SET(FMACM_MAIN_SRC ${FRAMEWORK_DIR}/fmacm.cpp)

    add_executable(FMACM ${FMACM_MAIN_SRC})
    add_executable(mitre::oss::fmacm ALIAS FMACM)
    target_link_libraries(FMACM framework)
    target_include_directories(FMACM PUBLIC 
        $<BUILD_INTERFACE:${cppmanifest_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${aaesim_INCLUDE_DIRS}>
    )
    set_target_properties(FMACM PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
    )
else()
    # Ensure framework library is a build target even though nothing depends on it
    set_target_properties(framework PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()
