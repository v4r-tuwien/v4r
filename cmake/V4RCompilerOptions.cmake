set(V4R_EXTRA_CXX_FLAGS "")
set(V4R_EXTRA_CXX_FLAGS_RELEASE "")
set(V4R_EXTRA_CXX_FLAGS_DEBUG "")
set(V4R_EXTRA_EXE_LINKER_FLAGS "")
set(V4R_EXTRA_EXE_LINKER_FLAGS_RELEASE "")
set(V4R_EXTRA_EXE_LINKER_FLAGS_DEBUG "")

macro(add_extra_compiler_option option)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()
  v4r_check_flag_support(CXX "${option}" _varname "${V4R_EXTRA_CXX_FLAGS} ${ARGN}")
  if(${_varname})
    set(V4R_EXTRA_CXX_FLAGS "${V4R_EXTRA_CXX_FLAGS} ${option}")
  endif()
endmacro()

# High level of warnings.
add_extra_compiler_option(-W)
add_extra_compiler_option(-Wall)
add_extra_compiler_option(-Werror=return-type)
add_extra_compiler_option(-Werror=address)
add_extra_compiler_option(-Werror=sequence-point)
add_extra_compiler_option(-Wformat)
add_extra_compiler_option(-Werror=format-security -Wformat)
add_extra_compiler_option(-Wmissing-declarations)
add_extra_compiler_option(-Wmissing-prototypes)
add_extra_compiler_option(-Wstrict-prototypes)
add_extra_compiler_option(-Winit-self)
add_extra_compiler_option(-Wpointer-arith)
add_extra_compiler_option(-Wsign-promo)

if(ENABLE_NOISY_WARNINGS)
  add_extra_compiler_option(-Wcast-align)
  add_extra_compiler_option(-Wstrict-aliasing=2)
  add_extra_compiler_option(-Werror=non-virtual-dtor)
  add_extra_compiler_option(-Wundef)
  add_extra_compiler_option(-Wshadow)
else()
  add_extra_compiler_option(-Wno-narrowing)
  add_extra_compiler_option(-Wno-delete-non-virtual-dtor)
  add_extra_compiler_option(-Wno-unnamed-type-template-args)
endif()
add_extra_compiler_option(-fdiagnostics-show-option)

# The -Wno-long-long is required in 64bit systems when including system headers.
add_extra_compiler_option(-Wno-long-long)

# We need pthread's
add_extra_compiler_option(-pthread)

if(CMAKE_COMPILER_IS_CLANGCXX)
  add_extra_compiler_option(-Qunused-arguments)
endif()

if(V4R_WARNINGS_ARE_ERRORS)
  add_extra_compiler_option(-Werror)
endif()

# Other optimizations
if(ENABLE_OMIT_FRAME_POINTER)
  add_extra_compiler_option(-fomit-frame-pointer)
else()
  add_extra_compiler_option(-fno-omit-frame-pointer)
  if(CMAKE_COMPILER_IS_CLANGCXX)
    # Clang requires this additional option (https://bugs.llvm.org/show_bug.cgi?id=9825)
    add_extra_compiler_option(-mno-omit-leaf-frame-pointer)
  endif()
endif()

if(ENABLE_FAST_MATH)
  add_extra_compiler_option(-ffast-math)
endif()

if(V4R_CPU_ARCH_NATIVE STREQUAL "AUTO")
  # Auto means we use the same setting as PCL
  if(PCL_DEFINITIONS MATCHES "-march=native")
    add_extra_compiler_option(-march=native)
  endif()
elseif(V4R_CPU_ARCH_NATIVE STREQUAL "ON")
  # Force on, however warn if PCL has different setting
  add_extra_compiler_option(-march=native)
  if(NOT PCL_DEFINITIONS MATCHES "-march=native")
    message(WARNING "By setting V4R_CPU_ARCH_NATIVE=ON you are forcing V4R to be built with -march=native flag. \
    However, PCL was built without it. This may lead to subtle alignment issues and segfaults. Use at your own risk!")
  endif()
elseif(V4R_CPU_ARCH_NATIVE STREQUAL "OFF")
  # Force off, however warn if PCL has different setting
  if(PCL_DEFINITIONS MATCHES "-march=native")
    message(WARNING "By setting V4R_CPU_ARCH_NATIVE=OFF you are forcing V4R to be built without -march=native flag. \
    However, PCL was built with it. This may lead to subtle alignment issues and segfaults. Use at your own risk!")
  endif()
endif()

# Profiling?
if(ENABLE_PROFILING)
  add_extra_compiler_option("-pg -g")
  # turn off incompatible options
  foreach(flags CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_DEBUG V4R_EXTRA_CXX_FLAGS_RELEASE V4R_EXTRA_CXX_FLAGS_DEBUG V4R_EXTRA_CXX_FLAGS)
    string(REPLACE "-fomit-frame-pointer" "" ${flags} "${${flags}}")
    string(REPLACE "-ffunction-sections" "" ${flags} "${${flags}}")
  endforeach()
else()
  # Remove unreferenced functions: function level linking
  add_extra_compiler_option(-ffunction-sections)
endif()

if(ENABLE_COVERAGE)
  set(V4R_EXTRA_CXX_FLAGS "${V4R_EXTRA_CXX_FLAGS} --coverage")
endif()

set(V4R_EXTRA_CXX_FLAGS_DEBUG "${V4R_EXTRA_CXX_FLAGS_DEBUG} -O0 -DDEBUG -D_DEBUG")

# Extra link libs if the user selects building static libs:
if(NOT BUILD_SHARED_LIBS AND CMAKE_COMPILER_IS_GNUCXX)
  set(V4R_LINKER_LIBS ${V4R_LINKER_LIBS} stdc++)
  set(V4R_EXTRA_CXX_FLAGS "-fPIC ${V4R_EXTRA_CXX_FLAGS}")
endif()

# Add user supplied extra options (optimization, etc...)
# ==========================================================
set(V4R_EXTRA_CXX_FLAGS     "${V4R_EXTRA_CXX_FLAGS}"     CACHE INTERNAL "Extra compiler options for C++ sources")
set(V4R_EXTRA_CXX_FLAGS_RELEASE "${V4R_EXTRA_CXX_FLAGS_RELEASE}" CACHE INTERNAL "Extra compiler options for Release build")
set(V4R_EXTRA_CXX_FLAGS_DEBUG   "${V4R_EXTRA_CXX_FLAGS_DEBUG}"   CACHE INTERNAL "Extra compiler options for Debug build")
set(V4R_EXTRA_EXE_LINKER_FLAGS         "${V4R_EXTRA_EXE_LINKER_FLAGS}"         CACHE INTERNAL "Extra linker flags")
set(V4R_EXTRA_EXE_LINKER_FLAGS_RELEASE "${V4R_EXTRA_EXE_LINKER_FLAGS_RELEASE}" CACHE INTERNAL "Extra linker flags for Release build")
set(V4R_EXTRA_EXE_LINKER_FLAGS_DEBUG   "${V4R_EXTRA_EXE_LINKER_FLAGS_DEBUG}"   CACHE INTERNAL "Extra linker flags for Debug build")

# Set default visibility to hidden
if(CMAKE_COMPILER_IS_GNUCXX)
  add_extra_compiler_option(-fvisibility=hidden)
  add_extra_compiler_option(-fvisibility-inlines-hidden)
endif()

# Append "extra" options, but only if build type is not None.
# Reason: None is used by Debian packaging tools to indicate that CMake should not add any flags and only use the ones
# that they provided explicitly with -DCMAKE_CXX_FLAGS.
if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "None")
  set(CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS} ${V4R_EXTRA_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} ${V4R_EXTRA_CXX_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} ${V4R_EXTRA_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS         "${CMAKE_EXE_LINKER_FLAGS} ${V4R_EXTRA_EXE_LINKER_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${V4R_EXTRA_EXE_LINKER_FLAGS_RELEASE}")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG   "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${V4R_EXTRA_EXE_LINKER_FLAGS_DEBUG}")
endif()
