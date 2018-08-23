if(HAVE_GLFW3)
  v4r_include_directories(${GLFW3_INCLUDE_DIRS})
  list(APPEND DEP_LIBS ${GLFW3_LIBRARIES})
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT ENABLE_NOISY_WARNINGS)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
endif()

  # ---------------------------------------------
  #      Define executable targets
  # ---------------------------------------------
MACRO(V4R_DEFINE_CPP_EXAMPLE name)
    set(sample_subfolder "cpp")
    set(the_target "${sample_kind}_${name}")

    v4r_check_dependencies(${V4R_DEPS})
    if(NOT V4R_DEPENDENCIES_FOUND)
      message("*** Cannot build ${the_target} because required dependencies (${V4R_DEPS}) are not found")
    else()
      add_executable(${the_target} "${name}.cpp")
      target_link_libraries(${the_target} ${V4R_LINKER_LIBS} ${V4R_DEPS} ${DEP_LIBS})

      set_target_properties(${the_target} PROPERTIES
        OUTPUT_NAME "${sample_kind}-${name}"
        PROJECT_LABEL "(${sample_KIND}) ${name}")

      if(ENABLE_SOLUTION_FOLDERS)
        set_target_properties(${the_target} PROPERTIES FOLDER "samples/${sample_subfolder}")
      endif()

      if (INSTALL_C_EXAMPLES)
        install(TARGETS ${the_target}
                RUNTIME DESTINATION bin
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
                )
      endif()
    endif()


  ENDMACRO()

project(cpp_samples)

# Detect if we want to build samples with library binaries or not
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_LIST_DIR)
#
# BUILD CASE 1: Build samples with library sources
#

#add_subdirectory(c)
add_subdirectory(examples)
add_subdirectory(eval)
add_subdirectory(tools)

#
# END OF BUILD CASE 1: Build samples with library sources
#
else()
#
# BUILD CASE 2: Build samples with library binaries
#
cmake_minimum_required(VERSION 2.8)

project(samples C CXX)
option(BUILD_EXAMPLES "Build samples" ON)
option(BUILD_EVALUATION_TOOLS "Build evaluation tools" ON)
option(BUILD_UTILITY_TOOLS "Build utility tools" ON)

find_package(V4R REQUIRED)

#add_subdirectory(c)
add_subdirectory(examples)
add_subdirectory(eval)
add_subdirectory(tools)

endif()
