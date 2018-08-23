# Global variables:
#
# V4R_MODULE_${the_module}_LOCATION
# V4R_MODULE_${the_module}_BINARY_DIR
# V4R_MODULE_${the_module}_DESCRIPTION
# V4R_MODULE_${the_module}_REASON - reason why module is not built (if so)
# V4R_MODULE_${the_module}_SUBMODULES
# V4R_MODULE_${the_module}_HEADERS
# V4R_MODULE_${the_module}_SOURCES
# V4R_MODULE_${the_module}_DEPS
# V4R_MODULE_${the_module}_DEPS_PRIVATE
# V4R_MODULE_${the_module}_REQ_DEPS
# V4R_MODULE_${the_module}_OPT_DEPS
# V4R_MODULE_${the_module}_PRIVATE_REQ_DEPS
# V4R_MODULE_${the_module}_PRIVATE_OPT_DEPS
# HAVE_${the_module} - for fast check of module availability

# Lists that are populated by the functions in this script
set(V4R_MODULES     "" CACHE INTERNAL "List of V4R modules")
set(V4R_SUBMODULES  "" CACHE INTERNAL "List of V4R submodules")

# Clean flags for modules from the previous cmake run.
# This is necessary to correctly handle modules removal.
foreach(mod ${V4R_MODULES})
  unset(HAVE_${mod} CACHE)
  unset(V4R_MODULE_${mod}_REASON CACHE)
  unset(V4R_MODULE_${mod}_REQ_DEPS CACHE)
  unset(V4R_MODULE_${mod}_OPT_DEPS CACHE)
  unset(V4R_MODULE_${mod}_PRIVATE_REQ_DEPS CACHE)
  unset(V4R_MODULE_${mod}_PRIVATE_OPT_DEPS CACHE)
endforeach()

# Create a new module.
# Module name is derived automatically from the directory name.
#
# Usage:
#
#   v4r_add_module([DISABLED_BY_DEFAULT]
#                  [REQUIRED <list of dependencies>]
#                  [OPTIONAL <list of dependencies>]
#                  [PRIVATE_REQUIRED <list of dependencies>]
#                  [PRIVATE_OPTIONAL <list of dependencies>]
#                  [HEADERS <list of headers>]                 relative to the modules/x/include/v4r/x/ directory
#                  [SOURCES <list of sources>])                relative to the modules/x/src/ directory
#
macro(v4r_add_module)
  # Construct module name
  get_filename_component(_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
  string(TOLOWER "${_name}" name)
  string(TOUPPER "${_name}" NAME)
  set(the_module v4r_${name})

  v4r_print_debug_message("Module ${the_module}")

  if(V4R_INITIAL_PASS)

    # Guard against redefinition
    if(";${V4R_MODULES};" MATCHES ";${the_module};")
      message(FATAL_ERROR "Redefinition of ${the_module} module.
  at:                    ${CMAKE_CURRENT_SOURCE_DIR}
  previously defined at: ${V4R_MODULE_${the_module}_LOCATION}
")
    endif()

    # Parse arguments
    set(options DISABLE_BY_DEFAULT)
    set(one_value_args DESCRIPTION)
    set(multi_value_args REQUIRED OPTIONAL PRIVATE_REQUIRED PRIVATE_OPTIONAL HEADERS SOURCES)
    cmake_parse_arguments(MODULE "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(MODULE_DISABLE_BY_DEFAULT)
      set(_default_state OFF)
    else()
      set(_default_state ON)
    endif()
    # Create option to enable/disable this module
    option(BUILD_${the_module} "Include ${the_module} module into the V4R build" ${_default_state})

    # Module sources
    set(_sources "")
    if(NOT MODULE_SOURCES)
      file(GLOB_RECURSE _sources
        "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/src/*.cc"
      )
    else()
      foreach(_src ${MODULE_SOURCES})
        list(APPEND _sources "${CMAKE_CURRENT_LIST_DIR}/src/${_src}")
      endforeach()
    endif()

    # Module headers
    set(_headers "")
    if(NOT MODULE_HEADERS)
      file(GLOB _headers
        "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/*.h"
        "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/*.hh"
        "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/impl/*.hpp"
      )
    else()
      foreach(_hdr ${MODULE_HEADERS})
        list(APPEND _headers "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/${_hdr}")
      endforeach()
    endif()

    # Remember the module details
    v4r_append(V4R_MODULES ${the_module})
    set(V4R_MODULE_${the_module}_DESCRIPTION "${MODULE_DESCRIPTION}" CACHE INTERNAL "Brief description of ${the_module} module")
    set(V4R_MODULE_${the_module}_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Location of ${the_module} module sources")
    # Create lists
    set(V4R_MODULE_${the_module}_SUBMODULES "" CACHE INTERNAL "List of submodules of ${the_module} module")
    set(V4R_MODULE_${the_module}_REQ_DEPS ${MODULE_REQUIRED} CACHE INTERNAL "List of required public dependencies for ${the_module} module")
    set(V4R_MODULE_${the_module}_OPT_DEPS ${MODULE_OPTIONAL} CACHE INTERNAL "List of optional public dependencies for ${the_module} module")
    set(V4R_MODULE_${the_module}_PRIVATE_REQ_DEPS ${MODULE_PRIVATE_REQUIRED} CACHE INTERNAL "List of required private dependencies for ${the_module} module")
    set(V4R_MODULE_${the_module}_PRIVATE_OPT_DEPS ${MODULE_PRIVATE_OPTIONAL} CACHE INTERNAL "List of optional private dependencies for ${the_module} module")
    set(V4R_MODULE_${the_module}_HEADERS ${_headers} CACHE INTERNAL "List of header files for ${the_module} module")
    set(V4R_MODULE_${the_module}_SOURCES ${_sources} CACHE INTERNAL "List of source files for ${the_module} module")
    set(V4R_MODULE_${the_module}_DEPS ${MODULE_LIBRARIES} CACHE INTERNAL "List of public dependencies for ${the_module} module (final)")
    set(V4R_MODULE_${the_module}_PRIVATE_DEPS ${MODULE_LIBRARIES} CACHE INTERNAL "List of private dependencies for ${the_module} module (final)")

    set(HAVE_${the_module} ${BUILD_${the_module}} CACHE INTERNAL "State of ${the_module} module")
  else()
    # Second pass, create targets
    if(HAVE_${the_module})
      _v4r_add_module_library(${the_module})
      _v4r_set_module_install_rules(${the_module})
    endif()
    # And remember binary dir (because in the first pass it was not convenient to do)
    set(V4R_MODULE_${the_module}_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "Location of ${the_module} module binary directory")
  endif()
endmacro()


# Add a submodule to the current module.
# The current module is the one created with the most recent call to v4r_add_module().
macro(v4r_add_submodule submodule_name)
  # Fully qualified submodule name (e.g. v4r_io_openni2_grabber)
  set(the_submodule ${the_module}_${submodule_name})

  v4r_print_debug_message("Submodule ${the_submodule}")

  if(V4R_INITIAL_PASS)
    # Parse arguments
    set(options DISABLE_BY_DEFAULT)
    set(one_value_args DESCRIPTION EXPORTS)
    set(multi_value_args REQUIRED OPTIONAL PRIVATE_REQUIRED PRIVATE_OPTIONAL HEADERS SOURCES)
    cmake_parse_arguments(SUBMODULE "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(SUBMODULE_DISABLE_BY_DEFAULT)
      set(_default_state OFF)
    else()
      set(_default_state ON)
    endif()
    # Create option to enable/disable this submodule
    option(BUILD_${the_submodule} "Include ${the_submodule} submodule into the V4R build" ${_default_state})

    # Submodule sources
    set(_sources "")
    foreach(_src ${SUBMODULE_SOURCES})
      list(APPEND _sources "${CMAKE_CURRENT_LIST_DIR}/src/${_src}")
    endforeach()

    # Submodule headers
    set(_headers "")
    foreach(_hdr ${SUBMODULE_HEADERS})
      list(APPEND _headers "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/${_hdr}")
    endforeach()

    # Remember the submodule details
    v4r_append(V4R_MODULE_${the_module}_SUBMODULES ${the_submodule})
    v4r_append(V4R_SUBMODULES ${the_submodule})
    set(V4R_SUBMODULE_${the_submodule}_MODULE "${the_module}" CACHE INTERNAL "Module to which ${the_submodule} belongs")
    set(V4R_SUBMODULE_${the_submodule}_NAME "${submodule_name}" CACHE INTERNAL "Short name of ${the_submodule}")
    # Create lists
    set(V4R_SUBMODULE_${the_submodule}_REQ_DEPS ${SUBMODULE_REQUIRED} CACHE INTERNAL "List of required public dependencies for ${the_submodule} submodule")
    set(V4R_SUBMODULE_${the_submodule}_OPT_DEPS ${SUBMODULE_OPTIONAL} CACHE INTERNAL "List of optional public dependencies for ${the_submodule} submodule")
    set(V4R_SUBMODULE_${the_submodule}_PRIVATE_REQ_DEPS ${SUBMODULE_PRIVATE_REQUIRED} CACHE INTERNAL "List of required private dependencies for ${the_submodule} submodule")
    set(V4R_SUBMODULE_${the_submodule}_PRIVATE_OPT_DEPS ${SUBMODULE_PRIVATE_OPTIONAL} CACHE INTERNAL "List of optional private dependencies for ${the_submodule} submodule")
    set(V4R_SUBMODULE_${the_submodule}_HEADERS ${_headers} CACHE INTERNAL "List of header files for ${the_submodule} submodule")
    set(V4R_SUBMODULE_${the_submodule}_SOURCES ${_sources} CACHE INTERNAL "List of source files for ${the_submodule} submodule")
    set(V4R_SUBMODULE_${the_submodule}_EXPORTS ${SUBMODULE_EXPORTS} CACHE INTERNAL "List of classes exported by ${the_submodule} submodule")

    set(HAVE_${the_submodule} ${BUILD_${the_submodule}} CACHE INTERNAL "State of ${the_submodule} submodule")
  else()
    # Second pass, nothing to do
  endif()
endmacro()


# Collect modules from specified directories
#
# This is a multi-step process:
#   1. Discover modules in the specified directories
#   2. Execute add_subdirectory() for each module with V4R_INITIAL_PASS on
#      At this point modules setup options/variables and declare their dependencies
#   3. Resolve dependencies and determine which modules are to be built
#   4. Execute add_subdirectory() for modules that are to be built with V4R_INITIAL_PASS off
#      At this point modules create actual targets and install rules
#
# Note: this must be called only once!
macro(v4r_glob_modules)
  if(DEFINED V4R_INITIAL_PASS)
    message(FATAL_ERROR "V4R has already loaded its modules. Calling v4r_glob_modules second time is not allowed.")
  endif()
  set(__directories_observed "")

  # Initial pass, discover and load modules
  v4r_print_debug_message("v4r_glob_modules: initial pass")
  set(V4R_INITIAL_PASS ON)
  set(V4R_PROCESSING_EXTRA_MODULES 0)
  foreach(__path ${ARGN})
    if("${__path}" STREQUAL "EXTRA")
      set(V4R_PROCESSING_EXTRA_MODULES 1)
    endif()
    get_filename_component(__path "${__path}" ABSOLUTE)

    list(FIND __directories_observed "${__path}" __pathIdx)
    if(__pathIdx GREATER -1)
      message(FATAL_ERROR "The directory ${__path} is globbed for V4R modules second time.")
    endif()
    list(APPEND __directories_observed "${__path}")

    file(GLOB __v4r_modules RELATIVE "${__path}" "${__path}/*")
    if(__v4r_modules)
      list(SORT __v4r_modules)
      foreach(mod ${__v4r_modules})
        get_filename_component(__modpath "${__path}/${mod}" ABSOLUTE)
        if(EXISTS "${__modpath}/CMakeLists.txt")
          list(FIND __directories_observed "${__modpath}" __pathIdx)
          if(__pathIdx GREATER -1)
            message(FATAL_ERROR "The module from ${__modpath} is already loaded.")
          endif()
          list(APPEND __directories_observed "${__modpath}")
          add_subdirectory("${__modpath}" "${CMAKE_CURRENT_BINARY_DIR}/${mod}/.${mod}")
        endif()
      endforeach()
    endif()
  endforeach()
  v4r_clear_vars(__v4r_modules __directories_observed __path __modpath __pathIdx)

  v4r_list_sort(V4R_MODULES)

  # Resolve dependencies
  _v4r_resolve_dependencies()

  # Second pass, create modules
  v4r_print_debug_message("v4r_glob_modules: second pass")
  set(V4R_INITIAL_PASS OFF PARENT_SCOPE)
  set(V4R_INITIAL_PASS OFF)
  foreach(m ${V4R_MODULES_ENABLED})
    if(m MATCHES "^v4r_")
      string(REGEX REPLACE "^v4r_" "" __shortname "${m}")
      add_subdirectory("${V4R_MODULE_${m}_LOCATION}" "${CMAKE_CURRENT_BINARY_DIR}/${__shortname}")
    else()
      message(WARNING "Check module name: ${m}")
      add_subdirectory("${V4R_MODULE_${m}_LOCATION}" "${CMAKE_CURRENT_BINARY_DIR}/${m}")
    endif()
  endforeach()
  unset(__shortname)
endmacro()


# [Internal]
#
# Add a library target for a module and set up its properties.
# Takes care to include headers/sources of enabled submodules.
#
macro(_v4r_add_module_library _module)
  if((NOT DEFINED V4R_MODULE_${_module}_TYPE AND BUILD_SHARED_LIBS) OR
     (DEFINED V4R_MODULE_${_module}_TYPE AND V4R_MODULE_${_module}_TYPE STREQUAL SHARED))
    set(_type SHARED)
  else()
    set(_type STATIC)
  endif()

  # Merge headers/sources form enabled submodules
  foreach(_submodule ${V4R_MODULE_${_module}_SUBMODULES})
    if(HAVE_${_submodule})
      v4r_append(V4R_MODULE_${_module}_HEADERS "${V4R_SUBMODULE_${_submodule}_HEADERS}")
      v4r_append(V4R_MODULE_${_module}_SOURCES "${V4R_SUBMODULE_${_submodule}_SOURCES}")
    endif()
  endforeach()

  v4r_print_debug_message("> add module library")
  v4r_print_debug_message("  type: ${_type}")
  v4r_print_debug_message("  headers: ${V4R_MODULE_${_module}_HEADERS}")
  v4r_print_debug_message("  sources: ${V4R_MODULE_${_module}_SOURCES}")
  v4r_print_debug_message("  dependencies: ${V4R_MODULE_${_module}_DEPS}")
  v4r_print_debug_message("  private dependencies: ${V4R_MODULE_${_module}_PRIVATE_DEPS}")

  add_library(${_module}
    ${_type}
    ${V4R_MODULE_${_module}_HEADERS}
    ${V4R_MODULE_${_module}_SOURCES}
    "${V4R_GENERATED_HEADERS_DIR}/v4r/config.h"
  )

  if(V4R_MODULE_${_module}_DEPS)
    target_link_libraries(${_module} PUBLIC ${V4R_MODULE_${_module}_DEPS})
  endif()

  if(V4R_MODULE_${_module}_PRIVATE_DEPS)
    target_link_libraries(${_module} PRIVATE ${V4R_MODULE_${_module}_PRIVATE_DEPS})
  endif()

  add_dependencies(v4r_modules ${_module})

  set_target_properties(${_module}
    PROPERTIES
      OUTPUT_NAME "${_module}${V4R_DLLVERSION}"
      DEBUG_POSTFIX "${V4R_DEBUG_POSTFIX}"
      ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}
      LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}
      RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
      INSTALL_NAME_DIR lib
  )
  target_include_directories(${_module}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>          # for precompiled headers
  )

  if(_type STREQUAL SHARED)
    set_target_properties(${_module} PROPERTIES
      COMPILE_DEFINITIONS V4RAPI_EXPORTS
      DEFINE_SYMBOL V4RAPI_EXPORTS
      VERSION ${V4R_LIBVERSION}
      SOVERSION ${V4R_SOVERSION}
    )
  endif()
endmacro()


# [Internal]
#
# Set up install rules for the module.
#
macro(_v4r_set_module_install_rules _module)
  v4r_print_debug_message("> set module install rules")
  get_target_property(_target_type ${_module} TYPE)
  if("${_target_type}" STREQUAL "SHARED_LIBRARY" OR (NOT BUILD_SHARED_LIBS OR NOT INSTALL_CREATE_DISTRIB))
    v4r_install_target(${_module} EXPORT V4RModules OPTIONAL
      RUNTIME DESTINATION ${V4R_BIN_INSTALL_PATH} COMPONENT libs
      LIBRARY DESTINATION ${V4R_LIB_INSTALL_PATH} COMPONENT libs NAMELINK_SKIP
      ARCHIVE DESTINATION ${V4R_LIB_INSTALL_PATH} COMPONENT dev
    )
  endif()
  if("${_target_type}" STREQUAL "SHARED_LIBRARY")
    install(TARGETS ${_module} LIBRARY DESTINATION ${V4R_LIB_INSTALL_PATH} COMPONENT dev NAMELINK_ONLY)
  endif()

  if(V4R_MODULE_${_module}_HEADERS)
    foreach(header ${V4R_MODULE_${_module}_HEADERS})
      string(REGEX REPLACE "^.*v4r/" "v4r/" header2 "${header}")
      if(NOT header2 MATCHES "private" AND header2 MATCHES "^(v4r/?.*)/[^/]+.h(..)?$")
        install(FILES ${header} OPTIONAL DESTINATION "${V4R_INCLUDE_INSTALL_PATH}/${CMAKE_MATCH_1}" COMPONENT dev)
      endif()
    endforeach()
  endif()
endmacro()


# [Internal]
#
# Disables a module by updating all relevant cache variables
#
function(_v4r_module_disable the_module reason)
  v4r_append(V4R_MODULES_DISABLED "${the_module}")
  v4r_remove_item(V4R_MODULES_ENABLED "${the_module}")
  set(HAVE_${the_module} OFF CACHE INTERNAL "Module ${the_module} can not be built in current configuration")
  set(V4R_MODULE_${the_module}_REASON ${reason} CACHE INTERNAL "Reason why module ${the_module} is disabled")
endfunction()


# [Internal]
#
# Disable a submodule by updating all relevant cache variables
#
function(_v4r_submodule_disable the_submodule reason)
  string(REGEX REPLACE "<=" "≤" reason ${reason})
  string(REGEX REPLACE ">=" "≥" reason ${reason})
  string(REGEX REPLACE "==" "=" reason ${reason})
  string(REGEX REPLACE "(=|<|>|≤|≥)" " \\1 " reason ${reason})
  v4r_append(V4R_SUBMODULES_DISABLED "${the_submodule}")
  v4r_remove_item(V4R_SUBMODULES_ENABLED "${the_submodule}")
  set(HAVE_${the_submodule} OFF CACHE INTERNAL "Submodule ${the_submodule} can not be built in current configuration")
  set(V4R_SUBMODULE_${the_submodule}_REASON ${reason} CACHE INTERNAL "Reason why submodule ${the_submodule} is disabled")
endfunction()


# [Internal]
#
# Resolve module/submodule dependencies and disable those with unsatisfied dependencies.
# After the call to this function the following variables are populated:
#
#   Globally: V4R_MODULES_ENABLED
#             V4R_MODULES_DISABLED
#             V4R_SUBMODULES_ENABLED
#             V4R_SUBMODULES_DISABLED
#             V4R_MODULES_3P_DEPS
#
#   For each (enabled) module: V4R_MODULE_${m}_DEPS
#                              V4R_MODULE_${m}_PRIVATE_DEPS
#
# See docstrings in the function body for explanation
#
function(_v4r_resolve_dependencies)

  set(V4R_MODULES_ENABLED     "" CACHE INTERNAL "List of V4R modules included into the build")
  set(V4R_MODULES_DISABLED    "" CACHE INTERNAL "List of V4R modules excluded from the build")
  set(V4R_SUBMODULES_ENABLED  "" CACHE INTERNAL "List of V4R submodules included into the build")
  set(V4R_SUBMODULES_DISABLED "" CACHE INTERNAL "List of V4R submodules excluded from the build")
  set(V4R_MODULES_3P_DEPS     "" CACHE INTERNAL "List of public third-party dependencies of V4R modules included into the build")

  foreach(m ${V4R_MODULES})
    if(HAVE_${m})
      v4r_append(V4R_MODULES_ENABLED ${m})
    else()
      v4r_append(V4R_MODULES_DISABLED ${m})
    endif()
  endforeach()

  foreach(s ${V4R_SUBMODULES})
    if(HAVE_${s})
      v4r_append(V4R_SUBMODULES_ENABLED ${s})
    else()
      v4r_append(V4R_SUBMODULES_DISABLED ${s})
    endif()
  endforeach()

  # Disable modules with unresolved dependencies
  set(has_changes ON)
  while(has_changes)
    set(has_changes OFF)
    foreach(m ${V4R_MODULES_ENABLED})
      set(_deps ${V4R_MODULE_${m}_REQ_DEPS} ${V4R_MODULE_${m}_PRIVATE_REQ_DEPS})
      while(_deps)
        v4r_list_pop_front(_deps d)
        v4r_check_dependency_available(${d} _result)
        if(NOT _result)
          _v4r_module_disable(${m} "missing dependency (${d})")
          set(has_changes ON)
          break()
        endif()
      endwhile()
    endforeach()
  endwhile()

  # Disable submodules with unresolved dependencies
  # Note: it is fine to analyze submodules after modules because the state of submodules does not influence the state
  # of the modules
  set(has_changes ON)
  while(has_changes)
    set(has_changes OFF)
    foreach(s ${V4R_SUBMODULES_ENABLED})
      set(_deps ${V4R_SUBMODULE_${s}_REQ_DEPS} ${V4R_SUBMODULE_${s}_PRIVATE_REQ_DEPS})
      while(_deps)
        v4r_list_pop_front(_deps d)
        v4r_check_dependency_available(${d} _result)
        if(NOT _result)
          _v4r_submodule_disable(${s} "missing dependency (${d})")
          set(has_changes ON)
          break()
        endif()
      endwhile()
    endforeach()
  endwhile()

  # Merge submodule dependencies into the module dependencies
  foreach(m ${V4R_MODULES_ENABLED})
    foreach(s ${V4R_MODULE_${m}_SUBMODULES})
      if(HAVE_${s})
        v4r_append(V4R_MODULE_${m}_REQ_DEPS "${V4R_SUBMODULE_${s}_REQ_DEPS}")
        v4r_append(V4R_MODULE_${m}_OPT_DEPS "${V4R_SUBMODULE_${s}_OPT_DEPS}")
        v4r_append(V4R_MODULE_${m}_PRIVATE_REQ_DEPS "${V4R_SUBMODULE_${s}_PRIVATE_REQ_DEPS}")
        v4r_append(V4R_MODULE_${m}_PRIVATE_OPT_DEPS "${V4R_SUBMODULE_${s}_PRIVATE_OPT_DEPS}")
      endif()
    endforeach()
  endforeach()

  # Collect final flattened list of public dependencies for each enabled module
  # Plus a global list of all public dependencies
  set(deps "")
  foreach(m ${V4R_MODULES_ENABLED})
    set(deps_${m} ${V4R_MODULE_${m}_REQ_DEPS})
    list(APPEND deps ${deps_${m}})
    foreach(d ${V4R_MODULE_${m}_OPT_DEPS})
      if(NOT (";${deps_${m}};" MATCHES ";${d};"))
        v4r_check_dependency_available(${d} _result)
        if(_result)
          list(APPEND deps_${m} ${d})
          list(APPEND deps ${d})
        endif()
      endif()
    endforeach()
  endforeach()

  # Collect final flattened list of private dependencies for each enabled module
  foreach(m ${V4R_MODULES_ENABLED})
    foreach(d ${V4R_MODULE_${m}_PRIVATE_REQ_DEPS})
      if(NOT (";${deps_${m}};" MATCHES ";${d};"))
        list(APPEND deps_private_${m} ${d})
      endif()
    endforeach()
    foreach(d ${V4R_MODULE_${m}_PRIVATE_OPT_DEPS})
      if(NOT (";${deps_${m}};" MATCHES ";${d};"))
        v4r_check_dependency_available(${d} _result)
        if(_result)
          list(APPEND deps_private_${m} ${d})
        endif()
      endif()
    endforeach()
  endforeach()

  # Filter, sort, and save lists of dependencies
  foreach(m ${V4R_MODULES_ENABLED})
    v4r_list_strip_versions(deps_${m})
    v4r_list_strip_versions(deps_private_${m})
    v4r_list_unique(deps_${m})
    v4r_list_unique(deps_private_${m})
    v4r_list_sort(deps_${m})
    v4r_list_sort(deps_private_${m})
    set(V4R_MODULE_${m}_DEPS ${deps_${m}} CACHE INTERNAL "Flattened public dependencies of ${m} module")
    set(V4R_MODULE_${m}_PRIVATE_DEPS ${deps_private_${m}} CACHE INTERNAL "Flattened private dependencies of ${m} module")
  endforeach()

  # Global list of public dependencies
  v4r_list_filterout(deps "^v4r_[^ ]+$")
  v4r_list_strip_versions(deps)
  v4r_list_unique(deps)
  v4r_list_sort(deps)
  set(V4R_MODULES_3P_DEPS ${deps} CACHE INTERNAL "Flattened public third-party dependencies of all enabled modules")
endfunction()


# ensures that all passed modules are available
# sets V4R_DEPENDENCIES_FOUND variable to TRUE/FALSE
macro(v4r_check_dependencies)
  set(V4R_DEPENDENCIES_FOUND TRUE)
  foreach(d ${ARGN})
    if(d MATCHES "^v4r_[^ ]+$" AND NOT HAVE_${d})
      set(V4R_DEPENDENCIES_FOUND FALSE)
      break()
    endif()
  endforeach()
endmacro()
