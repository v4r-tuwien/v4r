# Search packages for host system instead of packages for target system
# in case of cross compilation this macro should be defined by toolchain file
if(NOT COMMAND find_host_package)
  macro(find_host_package)
    find_package(${ARGN})
  endmacro()
endif()
if(NOT COMMAND find_host_program)
  macro(find_host_program)
    find_program(${ARGN})
  endmacro()
endif()

# assert macro
# Note: it doesn't support lists in arguments
# Usage samples:
#   v4r_assert(MyLib_FOUND)
#   v4r_assert(DEFINED MyLib_INCLUDE_DIRS)
macro(v4r_assert)
  if(NOT (${ARGN}))
    string(REPLACE ";" " " __assert_msg "${ARGN}")
    message(AUTHOR_WARNING "Assertion failed: ${__assert_msg}")
  endif()
endmacro()

# adds include directories in such way that directories from the V4R source tree go first
function(v4r_include_directories)
  # v4r_print_debug_message("v4r_include_directories( ${ARGN} )")
  set(__add_before "")
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
    if("${__abs_dir}" MATCHES "^${V4R_SOURCE_DIR}" OR "${__abs_dir}" MATCHES "^${V4R_BINARY_DIR}")
      list(APPEND __add_before "${dir}")
    elseif(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0" AND dir MATCHES "/usr/include$")
      # workaround for GCC 6.x bug
    else()
      include_directories(AFTER SYSTEM "${dir}")
    endif()
  endforeach()
  include_directories(BEFORE ${__add_before})
endfunction()

# adds include directories in such way that directories from the V4R source tree go first
function(v4r_target_include_directories target)
  set(__params_private "")
  set(__params_system "")
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
    if("${__abs_dir}" MATCHES "^${V4R_SOURCE_DIR}" OR "${__abs_dir}" MATCHES "^${V4R_BINARY_DIR}")
      list(APPEND __params_private "${__abs_dir}")
    else()
      list(APPEND __params_system "${dir}")
    endif()
  endforeach()
  if(TARGET ${target})
    if(__params_private)
      target_include_directories(${target} PRIVATE ${__params_private})
    endif()
    if(__params_system)
      target_include_directories(${target} SYSTEM PRIVATE ${__params_system})
    endif()
  else()
    v4r_append(V4R_TARGET_SYSTEM_INCLUDE_DIRS_${target} ${__params_system})
    v4r_append(V4R_TARGET_PRIVATE_INCLUDE_DIRS_${target} ${__params_private})
  endif()
endfunction()

# clears all passed variables
macro(v4r_clear_vars)
  foreach(_var ${ARGN})
    unset(${_var})
    unset(${_var} CACHE)
  endforeach()
endmacro()

set(V4R_COMPILER_FAIL_REGEX
    "command line option .* is valid for .* but not for C\\+\\+" # GNU
    "command line option .* is valid for .* but not for C" # GNU
    "unrecognized .*option"                     # GNU
    "unknown .*option"                          # Clang
    "ignoring unknown option"                   # MSVC
    "warning D9002"                             # MSVC, any lang
    "option .*not supported"                    # Intel
    "[Uu]nknown option"                         # HP
    "[Ww]arning: [Oo]ption"                     # SunPro
    "command option .* is not recognized"       # XL
    "not supported in this configuration, ignored"       # AIX (';' is replaced with ',')
    "File with unknown suffix passed to linker" # PGI
    "WARNING: unknown flag:"                    # Open64
  )

MACRO(v4r_check_compiler_flag LANG FLAG RESULT)
  set(_fname "${ARGN}")
  if(NOT DEFINED ${RESULT})
    if(_fname)
      # nothing
    elseif("_${LANG}_" MATCHES "_CXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.cxx")
      if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        FILE(WRITE "${_fname}" "int main() { return 0; }\n")
      else()
        FILE(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      endif()
    elseif("_${LANG}_" MATCHES "_C_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c")
      if("${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        FILE(WRITE "${_fname}" "int main(void) { return 0; }\n")
      else()
        FILE(WRITE "${_fname}" "#pragma\nint main(void) { return 0; }\n")
      endif()
    elseif("_${LANG}_" MATCHES "_OBJCXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.mm")
      if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        FILE(WRITE "${_fname}" "int main() { return 0; }\n")
      else()
        FILE(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      endif()
    else()
      unset(_fname)
    endif()
    if(_fname)
      if(NOT "x${ARGN}" STREQUAL "x")
        file(RELATIVE_PATH __msg "${CMAKE_SOURCE_DIR}" "${ARGN}")
        set(__msg " (check file: ${__msg})")
      else()
        set(__msg "")
      endif()
      MESSAGE(STATUS "Performing Test ${RESULT}${__msg}")
      TRY_COMPILE(${RESULT}
        "${CMAKE_BINARY_DIR}"
        "${_fname}"
        CMAKE_FLAGS "-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"   # CMP0056 do this on new CMake
        COMPILE_DEFINITIONS "${FLAG}"
        OUTPUT_VARIABLE OUTPUT)

      if(${RESULT})
        string(REPLACE ";" "," OUTPUT_LINES "${OUTPUT}")
        string(REPLACE "\n" ";" OUTPUT_LINES "${OUTPUT_LINES}")
        foreach(_regex ${V4R_COMPILER_FAIL_REGEX})
          if(NOT ${RESULT})
            break()
          endif()
          foreach(_line ${OUTPUT_LINES})
            if("${_line}" MATCHES "${_regex}")
              file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
                  "Build output check failed:\n"
                  "    Regex: '${_regex}'\n"
                  "    Output line: '${_line}'\n")
              set(${RESULT} 0)
              break()
            endif()
          endforeach()
        endforeach()
      endif()

      IF(${RESULT})
        SET(${RESULT} 1 CACHE INTERNAL "Test ${RESULT}")
        MESSAGE(STATUS "Performing Test ${RESULT} - Success")
      ELSE(${RESULT})
        MESSAGE(STATUS "Performing Test ${RESULT} - Failed")
        SET(${RESULT} "" CACHE INTERNAL "Test ${RESULT}")
        file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Compilation failed:\n"
            "    source file: '${_fname}'\n"
            "    check option: '${FLAG}'\n"
            "===== BUILD LOG =====\n"
            "${OUTPUT}\n"
            "===== END =====\n\n")
      ENDIF(${RESULT})
    else()
      SET(${RESULT} 0)
    endif()
  endif()
ENDMACRO()


macro(v4r_check_flag_support lang flag varname)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()

  if("_${lang}_" MATCHES "_CXX_")
    set(_lang CXX)
  elseif("_${lang}_" MATCHES "_C_")
    set(_lang C)
  elseif("_${lang}_" MATCHES "_OBJCXX_")
    set(_lang OBJCXX)
  else()
    set(_lang ${lang})
  endif()

  string(TOUPPER "${flag}" ${varname})
  string(REGEX REPLACE "^(/|-)" "HAVE_${_lang}_" ${varname} "${${varname}}")
  string(REGEX REPLACE " -|-|=| |\\." "_" ${varname} "${${varname}}")

  v4r_check_compiler_flag("${_lang}" "${ARGN} ${flag}" ${${varname}})
endmacro()


# Create an option that the user can control.
#
# The macro has two operating modes. First is when only positional arguments are given. In this case a binary switch
# option is created. If additional arguments are provided, then a string cache variable with constrained set of possible
# values is created. The constraits are defined by the _value and consequent arguments.
#
# Positional arguments:
#   _variable : name of the variable
#   _description : option description
#   _value : default value
#
# Usage:
#   v4r_option(<option_variable> "help string describing the option" ON)
#       - create binary option which is enabled by default
#   v4r_option(<option_variable> "help string describing the option" "Release" "Debug" "None")
#       - create an option that is constrained to have one of the three values, and is set by default to "Release"
macro(v4r_option _variable _description _value)
  if("${ARGN}" STREQUAL "")
    if(${_value})
      option(${_variable} "${_description}" ON)
    else()
      option(${_variable} "${_description}" OFF)
    endif()
  else()
    set(_options ${_value})
    list(APPEND _options ${ARGN})
    set(${_variable} ${_value} CACHE STRING "${_description}")
    set_property(CACHE ${_variable} PROPERTY STRINGS ${_options})
  endif()
endmacro()


set(V4R_BUILD_INFO_FILE "${V4R_BINARY_DIR}/version_string.tmp")
file(REMOVE "${V4R_BUILD_INFO_FILE}")
function(v4r_output_status msg)
  message(STATUS "${msg}")
  string(REPLACE "\\" "\\\\" msg "${msg}")
  string(REPLACE "\"" "\\\"" msg "${msg}")
  file(APPEND "${V4R_BUILD_INFO_FILE}" "\"${msg}\\n\"\n")
endfunction()

macro(v4r_finalize_status)
  if(NOT V4R_SKIP_STATUS_FINALIZATION)
    if(DEFINED V4R_MODULE_v4r_core_BINARY_DIR)
      execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${V4R_BUILD_INFO_FILE}" "${V4R_MODULE_v4r_core_BINARY_DIR}/version_string.inc" OUTPUT_QUIET)
    endif()
  endif()
endmacro()


# Status report function.
# Automatically align right column and selects text based on condition.
# Usage:
#   status(<text>)
#   status(<heading> <value1> [<value2> ...])
#   status(<heading> <condition> THEN <text for TRUE> ELSE <text for FALSE> )
function(status text)
  set(status_cond)
  set(status_then)
  set(status_else)

  set(status_current_name "cond")
  foreach(arg ${ARGN})
    if(arg STREQUAL "THEN")
      set(status_current_name "then")
    elseif(arg STREQUAL "ELSE")
      set(status_current_name "else")
    else()
      list(APPEND status_${status_current_name} ${arg})
    endif()
  endforeach()

  if(DEFINED status_cond)
    set(status_placeholder_length 32)
    string(RANDOM LENGTH ${status_placeholder_length} ALPHABET " " status_placeholder)
    string(LENGTH "${text}" status_text_length)
    if(status_text_length LESS status_placeholder_length)
      string(SUBSTRING "${text}${status_placeholder}" 0 ${status_placeholder_length} status_text)
    elseif(DEFINED status_then OR DEFINED status_else)
      v4r_output_status("${text}")
      set(status_text "${status_placeholder}")
    else()
      set(status_text "${text}")
    endif()

    if(DEFINED status_then OR DEFINED status_else)
      if(${status_cond})
        string(REPLACE ";" " " status_then "${status_then}")
        string(REGEX REPLACE "^[ \t]+" "" status_then "${status_then}")
        v4r_output_status("${status_text} ${status_then}")
      else()
        string(REPLACE ";" " " status_else "${status_else}")
        string(REGEX REPLACE "^[ \t]+" "" status_else "${status_else}")
        v4r_output_status("${status_text} ${status_else}")
      endif()
    else()
      string(REPLACE ";" " " status_cond "${status_cond}")
      string(REGEX REPLACE "^[ \t]+" "" status_cond "${status_cond}")
      v4r_output_status("${status_text} ${status_cond}")
    endif()
  else()
    v4r_output_status("${text}")
  endif()
endfunction()


# remove all matching elements from the list
macro(v4r_list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()


# Strip version requirement specifications from the list of dependencies
macro(v4r_list_strip_versions lst)
  set(_out_list "")
  foreach(item ${${lst}})
    string(REGEX REPLACE "([^<>=]+)(=|>|<|>=|<=)[0-9]+(\.[0-9]+)*$" "\\1" _out ${item})
    list(APPEND _out_list ${_out})
  endforeach()
  # For some reason setting lst to _out_list does not work, thus this trick
  foreach(item ${_out_list})
    list(REMOVE_AT ${lst} 0)
    list(APPEND ${lst} ${item})
  endforeach()
endmacro()


# stable & safe duplicates removal macro
macro(v4r_list_unique __lst)
  if(${__lst})
    list(REMOVE_DUPLICATES ${__lst})
  endif()
endmacro()


# safe list reversal macro
macro(v4r_list_reverse __lst)
  if(${__lst})
    list(REVERSE ${__lst})
  endif()
endmacro()


# safe list sorting macro
macro(v4r_list_sort __lst)
  if(${__lst})
    list(SORT ${__lst})
  endif()
endmacro()


# gets and removes the first element from list
macro(v4r_list_pop_front LST VAR)
  if(${LST})
    list(GET ${LST} 0 ${VAR})
    list(REMOVE_AT ${LST} 0)
  else()
    set(${VAR} "")
  endif()
endmacro()


# add install command
function(v4r_install_target)
  install(TARGETS ${ARGN})

  set(isPackage 0)
  unset(__package)
  unset(__target)
  foreach(e ${ARGN})
    if(NOT DEFINED __target)
      set(__target "${e}")
    endif()
    if(isPackage EQUAL 1)
      set(__package "${e}")
      break()
    endif()
    if(e STREQUAL "EXPORT")
      set(isPackage 1)
    endif()
  endforeach()

  if(DEFINED __package)
    list(APPEND ${__package}_TARGETS ${__target})
    set(${__package}_TARGETS "${${__package}_TARGETS}" CACHE INTERNAL "List of ${__package} targets")
  endif()
endfunction()


# read set of version defines from the header file
macro(v4r_parse_header FILENAME FILE_VAR)
  set(vars_regex "")
  set(__parnet_scope OFF)
  set(__add_cache OFF)
  foreach(name ${ARGN})
    if("${name}" STREQUAL "PARENT_SCOPE")
      set(__parnet_scope ON)
    elseif("${name}" STREQUAL "CACHE")
      set(__add_cache ON)
    elseif(vars_regex)
      set(vars_regex "${vars_regex}|${name}")
    else()
      set(vars_regex "${name}")
    endif()
  endforeach()
  if(EXISTS "${FILENAME}")
    file(STRINGS "${FILENAME}" ${FILE_VAR} REGEX "#define[ \t]+(${vars_regex})[ \t]+[0-9]+" )
  else()
    unset(${FILE_VAR})
  endif()
  foreach(name ${ARGN})
    if(NOT "${name}" STREQUAL "PARENT_SCOPE" AND NOT "${name}" STREQUAL "CACHE")
      if(${FILE_VAR})
        if(${FILE_VAR} MATCHES ".+[ \t]${name}[ \t]+([0-9]+).*")
          string(REGEX REPLACE ".+[ \t]${name}[ \t]+([0-9]+).*" "\\1" ${name} "${${FILE_VAR}}")
        else()
          set(${name} "")
        endif()
        if(__add_cache)
          set(${name} ${${name}} CACHE INTERNAL "${name} parsed from ${FILENAME}" FORCE)
        elseif(__parnet_scope)
          set(${name} "${${name}}" PARENT_SCOPE)
        endif()
      else()
        unset(${name} CACHE)
      endif()
    endif()
  endforeach()
endmacro()


# Append elements to an internal cached list (help string is preserved)
# A new list is created if the variable does not exist yet
macro(v4r_append _list)
  if(NOT DEFINED ${_list})
    set(${_list} ${ARGN} CACHE INTERNAL "")
  else()
    get_property(_help_string CACHE "${_list}" PROPERTY HELPSTRING)
    list(APPEND ${_list} ${ARGN})
    set(${_list} ${${_list}} CACHE INTERNAL "${_help_string}")
  endif()
endmacro()


# Prepend elements to an internal cached list (help string is preserved)
# A new list is created if the variable does not exist yet
macro(v4r_prepend _list)
  if(NOT DEFINED ${_list})
    set(${_list} ${ARGN} CACHE INTERNAL "")
  else()
    get_property(_help_string CACHE "${_list}" PROPERTY HELPSTRING)
    list(INSERT ${_list} 0 ${ARGN})
    set(${_list} ${${_list}} CACHE INTERNAL "${_help_string}")
  endif()
endmacro()


# Remove item to an internal cached list (help string is preserved)
# A new empty list is created if the variable does not exist yet
macro(v4r_remove_item _list)
  if(NOT DEFINED ${_list})
    set(${_list} ${ARGN} CACHE INTERNAL "")
  else()
    get_property(_help_string CACHE "${_list}" PROPERTY HELPSTRING)
    list(REMOVE_ITEM ${_list} ${ARGN})
    set(${_list} ${${_list}} CACHE INTERNAL "${_help_string}")
  endif()
endmacro()


# Add a global imported library
# Type is automatically determined from IMPORTED_LOCATION
macro(v4r_add_imported_library _name)
  # Extract (supported) target properties
  set(options)
  set(one_value_args IMPORTED_LOCATION)
  set(multi_value_args INTERFACE_LINK_LIBRARIES INTERFACE_INCLUDE_DIRECTORIES)
  cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  # Determine library type (shared/static/interface)
  if(ARG_IMPORTED_LOCATION)
    if(ARG_IMPORTED_LOCATION MATCHES "\.so$")
      set(_type "SHARED")
    elseif(ARG_IMPORTED_LOCATION MATCHES "\.a$")
      set(_type "STATIC")
    else()
      message(WARNING "Unable to detect the type of imported library ${ARG_IMPORTED_LOCATION}, assuming SHARED")
      set(_type "SHARED")
    endif()
  else()
    set(_type "INTERFACE")
  endif()

  # Filter include directories (remove non-existent)
  # Motivation: on certain systems PCL adds an incorrect path to freetype2 to its include directories list.
  # Adding such a directory to the INTERFACE_INCLUDE_DIRECTORIES property of a target leads to CMake error.
  if(ARG_INTERFACE_INCLUDE_DIRECTORIES)
    set(_filtered "")
    foreach(_include ${ARG_INTERFACE_INCLUDE_DIRECTORIES})
      if(EXISTS ${_include})
        list(APPEND _filtered "${_include}")
      endif()
    endforeach()
    list(REMOVE_DUPLICATES _filtered)
    set(ARG_INTERFACE_INCLUDE_DIRECTORIES ${_filtered})
  endif()

  # Create library and set properties
  add_library(${_name} ${_type} IMPORTED GLOBAL)
  foreach(_m ${one_value_args} ${multi_value_args})
    if(ARG_${_m})
      set_property(TARGET ${_name} PROPERTY ${_m} ${ARG_${_m}})
    endif()
  endforeach()
endmacro()


# Determine the location of an imported library target
# If the target is not an INTERFACE_LIBRARY, then the output is simply the IMPORTED_LOCATION
# Otherwise, if the target has interface include directories, the first one will be output
# Otherwise, if the target has interface link libraries, the location of the first one will be output
function(v4r_get_imported_library_location _target _result)
  if(TARGET ${_target})
    get_property(_type TARGET ${_target} PROPERTY TYPE)
    if(NOT _type STREQUAL "INTERFACE_LIBRARY")
      get_property(_location TARGET ${_target} PROPERTY IMPORTED_LOCATION)
      if(_location)
        set(${_result} "${_location}" PARENT_SCOPE)
        return()
      endif()
    endif()
    get_property(_includes TARGET ${_target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES SET)
    if(_includes)
      get_target_property(_includes ${_target} INTERFACE_INCLUDE_DIRECTORIES)
      if(_includes MATCHES ";")
        list(GET _includes 0 _includes)
      endif()
      set(${_result} "${_includes}" PARENT_SCOPE)
    else()
      get_property(_links TARGET ${_target} PROPERTY INTERFACE_LINK_LIBRARIES SET)
      if(_links)
        get_target_property(_links ${_target} INTERFACE_LINK_LIBRARIES)
        if(_links MATCHES ";")
          list(GET _links 0 _links)
        endif()
        v4r_get_imported_library_location(${_links} _out)
        set(${_result} "${_out}" PARENT_SCOPE)
      endif()
    endif()
  else()
    set(${_result} "" PARENT_SCOPE)
  endif()
endfunction()


# Check if a dependency is available and has an appropriate version.
# Dependency "xyz" is considered available if one of the following holds:
#   * HAVE_xyz cache variable is set to true
#   * HAVE_XYZ cache variable is set to true
#   * xyz is a library
# Positional arguments:
#   dependency : name of the dependency followed by an optional version requirement specification
#   result : variable where to store the result
# Usage:
#   v4r_check_dependency_available(opencv _result) : check that OpenCV is available, version does not matter
#   v4r_check_dependency_available(opencv==2.4.8 _result) : check that OpenCV 2.4.8 is available
#   v4r_check_dependency_available(opencv>=3 _result) : check that OpenCV 3 or above is available
function(v4r_check_dependency_available dependency result)
  # Test if dependency has version requirement specification
  if(${dependency} MATCHES "[<>=]")
    # Extract name and version
    string(REGEX REPLACE "[<>=]" ";" _tokens ${dependency})
    list(GET _tokens 0 _dep_name)
    list(REVERSE _tokens)
    list(GET _tokens 0 _dep_version)
    string(TOUPPER "${_dep_name}" _DEP_NAME)
    # Check if the specification is valid (=, >, <, >=, or <= followed by a digit[.digit[...] version)
    if(${dependency} MATCHES "[^<>=]+(=|>|<|>=|<=)[0-9]+(\.[0-9]+)*$")
      if(DEFINED ${_DEP_NAME}_VERSION)
        set(_version_ok 0)
        if(${dependency} MATCHES "<=")
          if((${_DEP_NAME}_VERSION VERSION_LESS _dep_version) OR (${_DEP_NAME}_VERSION VERSION_EQUAL _dep_version))
            set(_version_ok 1)
          endif()
        elseif(${dependency} MATCHES ">=")
          if((${_DEP_NAME}_VERSION VERSION_GREATER _dep_version) OR (${_DEP_NAME}_VERSION VERSION_EQUAL _dep_version))
            set(_version_ok 1)
          endif()
        elseif(${dependency} MATCHES "=")
          if(${_DEP_NAME}_VERSION VERSION_EQUAL _dep_version)
            set(_version_ok 1)
          endif()
        elseif(${dependency} MATCHES "<")
          if(${_DEP_NAME}_VERSION VERSION_LESS _dep_version)
            set(_version_ok 1)
          endif()
        elseif(${dependency} MATCHES ">")
          if(${_DEP_NAME}_VERSION VERSION_GREATER _dep_version)
            set(_version_ok 1)
          endif()
        endif()
      else()
        message(AUTHOR_WARNING "${dependency}: no version associated with ${_dep_name}, ignoring version requirement")
        set(_version_ok 1)
      endif()
    else()
      message(AUTHOR_WARNING "${dependency}: invalid version requirement, ignoring it")
      set(_version_ok 1)
    endif()
  else()
    # No version specification means version is okay
    set(_dep_name ${dependency})
    set(_version_ok 1)
  endif()
  if(NOT _version_ok)
    set(${result} NO PARENT_SCOPE)
  elseif(TARGET ${_dep_name} OR EXISTS ${_dep_name})
    set(${result} YES PARENT_SCOPE)
  else()
    string(TOUPPER "${_dep_name}" _DEP_NAME)
    if(HAVE_${_dep_name} OR HAVE_${_DEP_NAME})
      set(${result} YES PARENT_SCOPE)
    else()
      set(${result} NO PARENT_SCOPE)
    endif()
  endif()
endfunction()
