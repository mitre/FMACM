cmake_minimum_required(VERSION 3.0)

#===================================================================================
# Returns all variables (as a list) that start with prefix.
#===================================================================================
function (getListOfVarsStartingWith _prefix _varResult)
    get_cmake_property(_vars VARIABLES)
    string (REGEX MATCHALL "(^|;)${_prefix}[A-Za-z0-9_]*" _matchedVars "${_vars}")
    set (${_varResult} ${_matchedVars} PARENT_SCOPE)
endfunction()

#===================================================================================
# Returns all variables (as a list) that end with a suffix.
#===================================================================================
function (getListOfVarsEndingWith _suffix _varResult)
    get_cmake_property(_vars VARIABLES)
    string (REGEX MATCHALL "(^|;)[A-Za-z0-9_]+${_suffix}($|;)" _matchedVars "${_vars}")   # start with anything and ends with suffix
    set (${_varResult} ${_matchedVars} PARENT_SCOPE)
endfunction()

#===================================================================================
# Returns all variables (as a list) that end with a suffix.
#===================================================================================
function (createListOfPathsFromSuffix search_suffix result_list)
    getListOfVarsEndingWith(${search_suffix} endingVars)

    set (dir_list "")
    foreach (_var IN LISTS ${endingVars})
            #message("${_var}")
            set (dir_list ${dir_list} ${_var}) 
    endforeach()

    set (${result_list} ${dir_list} PARENT_SCOPE)

endfunction()

#===================================================================================
# Creates a package config/version file for a project or library
#===================================================================================
function(build_config CONFIG_INPUT_FILE VERSION_NUMBER PATH_TO_LIB PATH_TO_INCLUDE)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)

#### This should be a common area to all developers ####
set (CMAKE_INSTALL_PREFIX /devel/cmake/)

set (INCLUDE_DIR ${PATH_TO_INCLUDE})
set (LIB_DIR     ${PATH_TO_LIB})

configure_package_config_file(${CONFIG_INPUT_FILE} ${PATH_TO_LIB}/${CMAKE_PROJECT_NAME}-config.cmake
                              INSTALL_DESTINATION ${PATH_TO_LIB}/${CMAKE_PROJECT_NAME}/cmake
                              PATH_VARS INCLUDE_DIR LIB_DIR)

write_basic_package_version_file( ${PATH_TO_LIB}/${CMAKE_PROJECT_NAME}-config-version.cmake
                                  VERSION ${PROJECT_VERSION}
                                  COMPATIBILITY SameMajorVersion )

install(FILES ${PATH_TO_LIB}/${CMAKE_PROJECT_NAME}-config.cmake ${PATH_TO_LIB}/${CMAKE_PROJECT_NAME}-config-version.cmake
        DESTINATION  ${CMAKE_INSTALL_PREFIX}/${CMAKE_PROJECT_NAME}-${PROJECT_VERSION})

endfunction()
