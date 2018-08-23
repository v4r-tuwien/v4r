if(DEFINED StructureCore_DIR AND EXISTS ${StructureCore_DIR})
  # If StructureCore_DIR is supplied (and exists), search there first.
  # Note that we force clear paths associated with StructureCore because they may already be populated by finder scripts
  # on previous runs. Clearing makes sure find_* commans are re-run.
  v4r_clear_vars(STRUCTURECORE_INCLUDE_DIR STRUCTURECORE_LIBRARY)
  find_path(STRUCTURECORE_INCLUDE_DIR StructureCore.h PATHS ${StructureCore_DIR}
            PATH_SUFFIXES inc
            DOC "The path to StructureCore headers"
            NO_DEFAULT_PATH)
  find_library(STRUCTURECORE_LIBRARY StructureCore
               PATHS ${StructureCore_DIR}
               PATH_SUFFIXES lib
               DOC "The StructureCore library"
               NO_DEFAULT_PATH)
endif()

find_path(STRUCTURECORE_INCLUDE_DIR StructureCore.h
          PATHS /usr/local /opt /usr
          PATH_SUFFIXES inc
          DOC "The path to StructureCore headers"
          CMAKE_FIND_ROOT_PATH_BOTH)
find_library(STRUCTURECORE_LIBRARY
             NAMES StructureCore
             PATHS /usr/local /opt /usr
             DOC "The StructureCore library")

if(EXISTS ${STRUCTURECORE_INCLUDE_DIR} AND EXISTS ${STRUCTURECORE_LIBRARY})
  # Create imported target
  # TODO(sergey): the link libraries should be discovered using CMake
  v4r_add_imported_library(structurecore
    IMPORTED_LOCATION ${STRUCTURECORE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${STRUCTURECORE_INCLUDE_DIR}
    INTERFACE_LINK_LIBRARIES -lz -lrt -lm -lusb-1.0
  )
  set(HAVE_STRUCTURECORE TRUE)
else()
  set(HAVE_STRUCTURECORE FALSE)
endif()

mark_as_advanced(STRUCTURECORE_INCLUDE_DIR STRUCTURECORE_LIBRARY)

