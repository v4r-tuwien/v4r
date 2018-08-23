if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_GNUCXX 1)
  set(CMAKE_COMPILER_IS_CLANGCXX 1)
endif()
if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_GNUCC 1)
  set(CMAKE_COMPILER_IS_CLANGCC 1)
endif()

# ----------------------------------------------------------------------------
# Detect Intel ICC compiler -- for -fPIC in 3rdparty ( UNIX ONLY ):
#  see  include/v4r/cxtypes.h file for related   ICC & CV_ICC defines.
# NOTE: The system needs to determine if the '-fPIC' option needs to be added
#  for the 3rdparty static libs being compiled.  The CMakeLists.txt files
#  in 3rdparty use the CV_ICC definition being set here to determine if
#  the -fPIC flag should be used.
# ----------------------------------------------------------------------------
if  (__ICL)
  set(CV_ICC   __ICL)
elseif(__ICC)
  set(CV_ICC   __ICC)
elseif(__ECL)
  set(CV_ICC   __ECL)
elseif(__ECC)
  set(CV_ICC   __ECC)
elseif(__INTEL_COMPILER)
  set(CV_ICC   __INTEL_COMPILER)
elseif(CMAKE_C_COMPILER MATCHES "icc")
  set(CV_ICC   icc_matches_c_compiler)
endif()

# Similar code exists in V4RConfig.cmake
if(NOT DEFINED V4R_STATIC)
  # look for global setting
  if(NOT DEFINED BUILD_SHARED_LIBS OR BUILD_SHARED_LIBS)
    set(V4R_STATIC OFF)
  else()
    set(V4R_STATIC ON)
  endif()
endif()
