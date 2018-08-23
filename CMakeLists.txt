# ----------------------------------------------------------------------------
#  Root CMake file for V4R
#
#    From the off-tree build directory, invoke:
#      $ cmake <PATH_TO_V4R_ROOT>
#
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(POLICY CMP0022)
  cmake_policy(SET CMP0022 NEW)
endif()

if(POLICY CMP0026)
  # Disallow use of the LOCATION property for build targets
  cmake_policy(SET CMP0026 NEW)
endif()

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

if(POLICY CMP0071)
  # Ignore generated source files in AUTOMOC and AUTOUIC
  cmake_policy(SET CMP0071 OLD)
endif()

project(V4R CXX C)

include(cmake/V4RDebug.cmake)
include(cmake/V4RUtils.cmake)

v4r_clear_vars(V4RModules_TARGETS)

# ----------------------------------------------------------------------------
# Break in case of popular CMake configuration mistakes
# ----------------------------------------------------------------------------
if(NOT CMAKE_SIZEOF_VOID_P GREATER 0)
  message(FATAL_ERROR "CMake fails to deterimine the bitness of target platform.
  Please check your CMake and compiler installation. If you are crosscompiling then ensure that your CMake toolchain file correctly sets the compiler details.")
endif()

# ----------------------------------------------------------------------------
# Detect compiler and target platform architecture
# ----------------------------------------------------------------------------
include(cmake/V4RDetectCXXCompiler.cmake)

# Add these standard paths to the search paths for FIND_LIBRARY
# to find libraries from these locations first
if(EXISTS /lib64)
  list(APPEND CMAKE_LIBRARY_PATH /lib64)
else()
  list(APPEND CMAKE_LIBRARY_PATH /lib)
endif()

if(EXISTS /usr/lib64)
  list(APPEND CMAKE_LIBRARY_PATH /usr/lib64)
else()
  list(APPEND CMAKE_LIBRARY_PATH /usr/lib)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/V4RThirdParty.cmake)


#--------------------------------------------------------------------------------------------------#
#                                         V4R Dependencies                                         #
#--------------------------------------------------------------------------------------------------#
#                                                                                                  #
#                  Name             Default state (ON/OFF)     Prefer from source build (YES/NO)   #
#                                   Leave blank if mandatory   Leave blank if no choice            #
#                                                                                                  #
v4r_add_dependency(PCL                                                                             )
v4r_add_dependency(Boost                                                                           )
v4r_add_dependency(OpenCV                                                                          )
v4r_add_dependency(Glog                                                                            )
v4r_add_dependency(EDT                                                                             )
v4r_add_dependency(PCL_1_8                                                                         )
v4r_add_dependency(Qt               ON                                                             )
v4r_add_dependency(Eigen            ON                                                             )
v4r_add_dependency(GLM              ON                         NO                                  )
v4r_add_dependency(Ceres            ON                         YES                                 )
v4r_add_dependency(OpenGL           ON                                                             )
v4r_add_dependency(GLEW             ON                                                             )
v4r_add_dependency(Assimp           ON                                                             )
v4r_add_dependency(SiftGPU          ON                                                             )
v4r_add_dependency(LibSVM           ON                         NO                                  )
v4r_add_dependency(OpenNURBS        ON                                                             )
v4r_add_dependency(Caffe            ON                                                             )
v4r_add_dependency(OpenMP           ON                                                             )
v4r_add_dependency(Radical          ON                        YES                                  )
v4r_add_dependency(OpenNI2          ON                                                             )
v4r_add_dependency(StructureCore    OFF                                                            )
v4r_add_dependency(RealSense2       ON                                                             )
v4r_add_dependency(MVSTexturing     ON                                                             )

# V4R build components
# ===================================================
v4r_option(BUILD_SHARED_LIBS        "Build shared libraries (.so) instead of static ones (.a)"               ON)
v4r_option(BUILD_v4r_apps           "Build utility applications (used for example to train classifiers)"     ON)
v4r_option(BUILD_DOCS               "Create build rules for V4R Documentation"                               ON)
v4r_option(BUILD_EXAMPLES           "Build all examples"                                                     ON)
v4r_option(BUILD_EVALUATION_TOOLS   "Build all evaluation tools"                                            OFF)
v4r_option(BUILD_UTILITY_TOOLS      "Build all utility tools"                                                ON)
v4r_option(BUILD_TESTS              "Build tests"                                                           OFF)

# V4R installation options
# ===================================================
v4r_option(INSTALL_CREATE_DISTRIB      "Change install rules to build the distribution package"             OFF)
v4r_option(INSTALL_C_EXAMPLES          "Install C examples"                                                  ON)
v4r_option(INSTALL_C_UTILITY_TOOLS     "Install C utility tools"                                             ON)
v4r_option(INSTALL_C_EVALUATION_TOOLS  "Install C evaluation tools"                                         OFF)
v4r_option(INSTALL_TO_MANGLED_PATHS    "Enables mangled install paths (helps with side by side installs)"   OFF)

# V4R build options
# ===================================================
v4r_option(ENABLE_PROFILING            "Enable profiling in the GCC compiler (Add flags: -g -pg)"          OFF)
v4r_option(ENABLE_COVERAGE             "Enable coverage collection with  GCov"                             OFF)
v4r_option(ENABLE_OMIT_FRAME_POINTER   "Enable -fomit-frame-pointer"                                        ON)
v4r_option(ENABLE_FAST_MATH            "Enable -ffast-math"                                                OFF)
v4r_option(ENABLE_NOISY_WARNINGS       "Show all warnings even if they are too noisy"                      OFF)
v4r_option(V4R_WARNINGS_ARE_ERRORS     "Treat warnings as errors"                                          OFF)
v4r_option(V4R_CPU_ARCH_NATIVE         "Enable all capabilities of system's CPU"                   AUTO ON OFF)

# ----------------------------------------------------------------------------
#  Get actual V4R version number from sources
# ----------------------------------------------------------------------------
include(cmake/V4RVersion.cmake)

# ----------------------------------------------------------------------------
#  Build & install layouts
# ----------------------------------------------------------------------------

# Save libs and executables in the same place
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Output directory for applications" )

set(V4R_DOC_INSTALL_PATH share/V4R/doc)
set(V4R_SAMPLES_BIN_INSTALL_PATH "samples")
set(V4R_BIN_INSTALL_PATH "bin")

set(LIBRARY_OUTPUT_PATH         "${V4R_BINARY_DIR}/lib")
set(3P_LIBRARY_OUTPUT_PATH      "${V4R_BINARY_DIR}/3rdparty/lib${LIB_SUFFIX}")

set(V4R_LIB_INSTALL_PATH         lib${LIB_SUFFIX})
set(V4R_3P_LIB_INSTALL_PATH      share/V4R/3rdparty/${V4R_LIB_INSTALL_PATH})
set(V4R_SAMPLES_SRC_INSTALL_PATH share/V4R/samples)
set(V4R_OTHER_INSTALL_PATH       share/V4R)
set(V4R_INCLUDE_INSTALL_PATH     "include")
set(V4R_3P_INCLUDE_INSTALL_PATH  share/V4R/3rdparty/${V4R_INCLUDE_INSTALL_PATH})
set(V4R_3P_INSTALL_PATH          share/V4R/3rdparty)

math(EXPR SIZEOF_VOID_P_BITS "8 * ${CMAKE_SIZEOF_VOID_P}")
if(LIB_SUFFIX AND NOT SIZEOF_VOID_P_BITS EQUAL LIB_SUFFIX)
  set(V4R_CONFIG_INSTALL_PATH lib${LIB_SUFFIX}/cmake/v4r)
else()
  set(V4R_CONFIG_INSTALL_PATH share/V4R)
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${V4R_3P_LIB_INSTALL_PATH};${CMAKE_INSTALL_PREFIX}/${V4R_LIB_INSTALL_PATH}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Postfix of so's:
set(V4R_DLLVERSION "")
set(V4R_DEBUG_POSTFIX "")

if(DEFINED CMAKE_DEBUG_POSTFIX)
  set(V4R_DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
endif()


if(INSTALL_TO_MANGLED_PATHS)
  set(V4R_INCLUDE_INSTALL_PATH ${V4R_INCLUDE_INSTALL_PATH}/v4r-${V4R_VERSION})
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_3P_LIB_INSTALL_PATH "${V4R_3P_LIB_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_SAMPLES_SRC_INSTALL_PATH "${V4R_SAMPLES_SRC_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_CONFIG_INSTALL_PATH "${V4R_CONFIG_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_DOC_INSTALL_PATH "${V4R_DOC_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_JAR_INSTALL_PATH "${V4R_JAR_INSTALL_PATH}")
  string(REPLACE "V4R" "V4R-${V4R_VERSION}" V4R_OTHER_INSTALL_PATH "${V4R_OTHER_INSTALL_PATH}")
endif()

v4r_install_dependencies()

# ----------------------------------------------------------------------------
#  Path for build/platform -specific headers
# ----------------------------------------------------------------------------
set(V4R_GENERATED_HEADERS_DIR "${CMAKE_BINARY_DIR}/generated" CACHE PATH "Where to put generated header files")
v4r_include_directories(${V4R_GENERATED_HEADERS_DIR})

# ----------------------------------------------------------------------------
#  Path for additional modules
# ----------------------------------------------------------------------------
set(V4R_EXTRA_MODULES_PATH "" CACHE PATH "Where to look for additional V4R modules")

# ----------------------------------------------------------------------------
#  Autodetect if we are in a GIT repository
# ----------------------------------------------------------------------------
find_host_package(Git QUIET)

if(GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty --match "[0-9].[0-9].[0-9]*"
    WORKING_DIRECTORY "${V4R_SOURCE_DIR}"
    OUTPUT_VARIABLE V4R_VCSVERSION
    RESULT_VARIABLE GIT_RESULT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT GIT_RESULT EQUAL 0)
    set(V4R_VCSVERSION "unknown")
  endif()
else()
  # We don't have git:
  set(V4R_VCSVERSION "unknown")
endif()


# ----------------------------------------------------------------------------
# V4R compiler and linker options
# ----------------------------------------------------------------------------
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
if(CMAKE_GENERATOR MATCHES "Makefiles|Ninja" AND "${CMAKE_BUILD_TYPE}" STREQUAL "")
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

include(cmake/V4RCompilerOptions.cmake)


# ----------------------------------------------------------------------------
#       CHECK FOR SYSTEM LIBRARIES, OPTIONS, ETC..
# ----------------------------------------------------------------------------
find_package(PkgConfig QUIET)
include(CheckFunctionExists)
include(CheckIncludeFile)

set(V4R_LINKER_LIBS ${V4R_LINKER_LIBS} X11 Xrandr Xinerama Xcursor Xxf86vm)
set(V4R_LINKER_LIBS ${V4R_LINKER_LIBS} dl m pthread rt)

include(cmake/V4RModule.cmake)


# --- Doxygen for documentation ---
unset(DOXYGEN_FOUND CACHE)
if(BUILD_DOCS)
  find_package(Doxygen)
endif(BUILD_DOCS)

# Extra V4R targets: uninstall, package_source, perf, etc.
include(cmake/V4RExtraTargets.cmake)


# ----------------------------------------------------------------------------
# Process subdirectories
# ----------------------------------------------------------------------------

# V4R modules
add_subdirectory(modules)

# TODO
# Generate targets for documentation
#add_subdirectory(doc)

# extra applications
if(BUILD_v4r_apps)
  # TODO
  add_subdirectory(apps)
endif()

# examples
if(BUILD_EXAMPLES)
  add_subdirectory(samples)
endif()

if(BUILD_TESTS)
  v4r_add_dependency(Gtest)
  add_subdirectory(test)
endif()

# ----------------------------------------------------------------------------
# Finalization: generate configuration-based files
# ----------------------------------------------------------------------------

# Generate platform-dependent and configuration-dependent headers
include(cmake/V4RGenerateHeaders.cmake)

# Generate V4RСonfig.cmake and V4RConfig-version.cmake for cmake projects
include(cmake/V4RGenerateConfig.cmake)

include(cmake/V4RStatus.cmake)

#--------------------------------------------------------------------------------------------------#
#                                             Summary                                              #
#--------------------------------------------------------------------------------------------------#

status("")
status("General configuration for V4R ${V4R_VERSION} =====================================")
if(V4R_VCSVERSION)
  status("  Version control:" ${V4R_VCSVERSION})
endif()

# ========================== build platform ==========================
status("")
status("  Platform:")
status("    Host:"             ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR})
if(CMAKE_CROSSCOMPILING)
  status("    Target:"         ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION} ${CMAKE_SYSTEM_PROCESSOR})
endif()
status("    CMake:"            ${CMAKE_VERSION})
status("    CMake generator:"  ${CMAKE_GENERATOR})
status("    CMake build tool:" ${CMAKE_BUILD_TOOL})
status("    Configuration:"    ${CMAKE_BUILD_TYPE})

# ========================== C/C++ options ==========================
if(CMAKE_CXX_COMPILER_VERSION)
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_CXX_COMPILER_VERSION})")
elseif(CMAKE_COMPILER_IS_CLANGCXX)
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_CLANG_REGEX_VERSION})")
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_GCC_REGEX_VERSION})")
else()
  set(V4R_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1}")
endif()
string(STRIP "${V4R_COMPILER_STR}" V4R_COMPILER_STR)
string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

status("")
status("  C/C++:")
status("    Built as dynamic libs:"  BUILD_SHARED_LIBS THEN YES ELSE NO)
status("    C++ Compiler:"           ${V4R_COMPILER_STR})
status("    C++ flags:"              ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${BUILD_TYPE}})
status("    Linker flags:"           ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_${BUILD_TYPE}})

# ========================== Third-party libraries ==========================
v4r_status("")
v4r_status("  Third-party libraries:")
v4r_status("")
v4r_status("  # Mandatory:")
v4r_status("")
foreach(_NAME ${V4R_3P_MANDATORY})
  v4r_dependency_status(${_NAME})
endforeach()
v4r_status("")
v4r_status("  # Optional:")
v4r_status("")
foreach(_NAME ${V4R_3P_OPTIONAL})
  v4r_dependency_status(${_NAME})
endforeach()
v4r_status("")
v4r_status("  V4R modules:")
v4r_status("")
foreach(_NAME ${V4R_MODULES})
  v4r_module_status(${_NAME})
endforeach()

# ========================== documentation ==========================
if(BUILD_DOCS)
  status("")
  status("  Documentation:")
  status("    Doxygen:"             DOXYGEN_FOUND             THEN "${DOXYGEN_EXECUTABLE} (ver ${DOXYGEN_VERSION})" ELSE NO)
endif()

# ========================== samples and tests ==========================
status("")
status("  Tests and samples:")
status("    Unit tests:"        BUILD_TESTS                   THEN YES ELSE NO)
status("    C/C++ Examples:"    BUILD_EXAMPLES                THEN YES ELSE NO)

# ========================== auxiliary ==========================
status("")
status("  Install path:" "${CMAKE_INSTALL_PREFIX}")
status("")
status("  v4r/config.h is in:" "${V4R_GENERATED_HEADERS_DIR}")
status("-----------------------------------------------------------------")
status("")

v4r_finalize_status()

# ----------------------------------------------------------------------------
# Warn in the case of in-source build
# ----------------------------------------------------------------------------
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  message(WARNING "The source directory is the same as binary directory. \"make clean\" may damage the source tree")
endif()


#--------------------------------------------------------------------------------------------------#
#                                              Debug                                               #
#--------------------------------------------------------------------------------------------------#
#
# * Debug message printing in V4R scripts:
#
#     -DPRINT_DEBUG=ON
#
# * Print all cache variables matching a certain string:
#
#     -DPRINT_CACHE=<STRING_TO_MATCH>
#
# * Clear all cache variables matching a certain string:
#
#     -DCLEAR_CACHE=<STRING_TO_MATCH>
#
# * Print all properties of a target:
#
#     -DPRINT_TARGET=<TARGET_NAME>
#
v4r_print_cache_variables("${PRINT_CACHE}")
v4r_clear_cache_variables("${CLEAR_CACHE}")
v4r_print_target_properties("${PRINT_TARGET}")
v4r_clear_vars(PRINT_DEBUG PRINT_CACHE CLEAR_CACHE PRINT_TARGET)
