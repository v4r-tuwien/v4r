
if(DEFINED OpenNI2_DIR AND EXISTS ${OpenNI2_DIR})
  # If OpenNI2_DIR is supplied (and exists), search there first.
  # Note that we force clear paths associated with OpenNI2 because they may already be populated by finder scripts of
  # other dependency libraries (e.g. PCL). Clearing makes sure find_* commans are re-run.
  v4r_clear_vars(OPENNI2_INCLUDE_DIR OPENNI2_LIBRARY)
  find_path(OPENNI2_INCLUDE_DIR OpenNI.h PATHS ${OpenNI2_DIR}
            PATH_SUFFIXES openni2 Include
            DOC "The path to OpenNI2 headers"
            NO_DEFAULT_PATH)
  find_library(OPENNI2_LIBRARY OpenNI2
               PATHS ${OpenNI2_DIR}
               PATH_SUFFIXES Redist
               DOC "The OpenNI2 library"
               NO_DEFAULT_PATH)
endif()

find_path(OPENNI2_INCLUDE_DIR OpenNI.h
          PATHS /usr/local /opt /usr
          PATH_SUFFIXES openni2
          DOC "The path to OpenNI2 headers"
          CMAKE_FIND_ROOT_PATH_BOTH)
find_library(OPENNI2_LIBRARY
             NAMES OpenNI2
             PATHS /usr/local /opt /usr
             DOC "The OpenNI2 library")

if(EXISTS ${OPENNI2_INCLUDE_DIR} AND EXISTS ${OPENNI2_LIBRARY})
  # Create imported target
  v4r_add_imported_library(openni2
    IMPORTED_LOCATION ${OPENNI2_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${OPENNI2_INCLUDE_DIR}
  )
  # Find version number
  v4r_parse_header("${OPENNI2_INCLUDE_DIR}/OniVersion.h" ONI_VERSION_LINE ONI_VERSION_MAJOR ONI_VERSION_MINOR ONI_VERSION_MAINTENANCE ONI_VERSION_BUILD)
  if(ONI_VERSION_MAJOR)
    set(OPENNI2_VERSION ${ONI_VERSION_MAJOR}.${ONI_VERSION_MINOR}.${ONI_VERSION_MAINTENANCE} CACHE INTERNAL "OpenNI2 version")
    set(OPENNI2_VERSION_BUILD ${ONI_VERSION_BUILD} CACHE INTERNAL "OpenNI2 build version")
  endif()
  v4r_clear_vars(ONI_VERSION_LINE ONI_VERSION_MAJOR ONI_VERSION_MINOR ONI_VERSION_MAINTENANCE ONI_VERSION_BUILD)
  set(HAVE_OPENNI2 TRUE)
else()
  set(HAVE_OPENNI2 FALSE)
endif()

mark_as_advanced(OPENNI2_INCLUDE_DIR OPENNI2_LIBRARY)

