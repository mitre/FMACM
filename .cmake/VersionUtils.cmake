# VersionUtils.cmake

#
# Modeled heavily after macros for the Log4Cplus project
#

# Get myapp version macro
# first param - path to include folder, we will rip version from version.h
macro(get_version _include_PATH vmajor vminor vpatch vsuffix)
  file(STRINGS "${_include_PATH}/aaesim/version.h" _myapp_VER_STRING_AUX REGEX ".*#define[ ]+AAESIM_VERSION_STR[ ]+")
  # message(${_myapp_VER_STRING_AUX}) debugging message
  string(REGEX MATCHALL "[0-9]+" _myapp_VER_LIST "${_myapp_VER_STRING_AUX}") # version numbers
  list(LENGTH _myapp_VER_LIST _myapp_VER_LIST_LEN)

  # suffix processing
  string(REGEX MATCH "[\"A-Z,a-z\")]*$" _myapp_VER_SUFFIX "${_myapp_VER_STRING_AUX}") # version suffix (string)
  string(LENGTH ${_myapp_VER_SUFFIX} _myapp_SUFFIX_LENGTH)
  if(_myapp_SUFFIX_LENGTH EQUAL 1)
    set(_myapp_VER_SUFFIX " ")
  else()
      string(REPLACE ")" "" _myapp_VER_SUFFIX ${_myapp_VER_SUFFIX}) # trim the suffix
  endif()
  # message(${_myapp_VER_SUFFIX}) # debugging message

# get each version value from the list and return
  set(${vsuffix} ${_myapp_VER_SUFFIX})
  if(_myapp_VER_LIST_LEN EQUAL 3)
    list(GET _myapp_VER_LIST 0 ${vmajor})
    list(GET _myapp_VER_LIST 1 ${vminor})
    list(GET _myapp_VER_LIST 2 ${vpatch})
  endif()
endmacro()