^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package v4r
^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2017-09-27)
------------------
  update version in package.xml
* update version in package.xml
  Master
* bug fix: removed cam.center_x ... it's not available in pcl-stable
* bug work-fix: removed OdmTexturing::loadMesh (not available in pcl)
* New RTMT2 version including a modified version of OpenDroneMap texture mapping
* test odm texturing
* Contributors: Johann Prankl, Markus Bajones, Thomas Faeulhammer, Thomas Fäulhammer

2.0.2 (2017-10-03)
------------------
  add ODM and indicate existing libraries inside V4R
* add brief description on Readme and some styling
* update readme
* fix typo
* add ODM and indicate existing libraries inside V4R
  revert readme overwrite
* revert readme overwrite
  Remove all mentions of Qt4 and always use Qt5
* Remove all mentions of Qt4 and always use Qt5
  update author list
  update contribution page
  Update readme
  Remove explicit VTK dependency
* remove VTK and OpenNI
* fix style
* Remove explicit VTK dependency
* update license URL
* split issue tracker URL
* fix URL
* fix URLs
  Remove all references to OpenCL
* update contribution page
  Textured rendering
* update author list
* update Readme
  fix license
* Remove all references to OpenCL
  Was inherited from OpenCV build system, but not used in V4R.
* fix license
* update deps
* fix conflicts
  Strands to v4r in readme
* update dependencies description
* As we build ceres within v4r this is not needed anymore
* Update readme to get rid of Strands links
  OpenNURBS does not need to depend on PCL
* OpenNURBS does not need to depend on PCL
  The only usage of PCL in the code is PCL_EXPORTS macro, which anyway evaluates to nothing on Unix platform.
  Build deb packages on all repos when tagged
  Do not process some RTMT headers with Qt MOC
  # Conflicts:
  #   .gitlab-ci.yml
  cleanup gitlab_CI and build fixes
* Build deb packages on all repos when tagged
* Merge remote-tracking branch 'v4r_origin/master'
* Added rendering of textured meshes.
* Do not process some RTMT headers with Qt MOC
  This is to avoid classical Qt MOC/Boost error:
  usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
  apps/RTMT2/CMakeFiles/RTMT2.dir/build.make:87: recipe for target 'apps/RTMT2/moc_OctreeVoxelCentroidContainerXYZRGB.cxx' failed
* Remove "-dev" string from version
* Remove duplicate job entry
* Merge remote-tracking branch 'upstream/master'
  Merge back the released state from v4r
* 2.0.1
* update changelog before releasing V4R
* Merge remote-tracking branch 'upstream/master'
  update version in package.xml
* update version in package.xml
* Merge remote-tracking branch 'upstream/master'
  Master
* bug fix: removed cam.center_x ... it's not available in pcl-stable
* bug work-fix: removed OdmTexturing::loadMesh (not available in pcl)
* New RTMT2 version including a modified version of OpenDroneMap texture mapping
* test odm texturing
* do not compile multiview recognizer
* fixed: we did not compile on xenial
* Add missing headers an source files.
  Arrange them in lexicographical order
* Contributors: Johann Prankl, Markus 'Bajo' Bajones, Markus Bajones, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer, Unknown, V4R Release Manager

2.0.3 (2017-11-17)
------------------
  Cmake updates
* Remove unused submodules in `_CHILDREN` variables
* Backport various updates for utils from OpenCV master
* Remove unused functions/macros
* Remove Windows-only branch in CMake scripts
  Fix install rules for third-party dependencies
  Fix error ‘boost::Q_FOREACH’ has not been declared
* Do not clear variables after debug cache print
* Fix install rules for third-party dependencies
  Install only if enabled and built.
* Fix error ‘boost::Q_FOREACH’ has not been declared
  Solution from: https://stackoverflow.com/a/17610731/1525865
  use our own server, as people reported issues with the keyservers
  use nullptr instead of NULL
* use nullptr instead of NULL
  Add radical
* Add radical 3rd-party dependency
* Improve dependency status printing
  Detect targets and print their location
* use our own server, as people reported issues with the keyservers
  Rendering of Normals
  Fix warnings
* Fixed one stupid blank space.
* fix some more warnings
* fix warnings
* Reduced the amount of violation of our Coding Style Guideline.
  Master
* add glog dependency to io
* use more bf paths
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Ceres 1.13 clean
  Remove unneeded CMakeLists.txt files
  Closes #36
* Merge remote-tracking branch 'origin/master'
* Added a normal rendering, in case no normal is provided by the mesh it has a fallback to a per triangle estimation of normals.
* Updating Readme with Ceres 1.13
* Delete 3rdpartie's cmakelists
* Moving 2 Ceres 1.13
  Update cmake/3rdparty/BuildCeres.cmake
* add glog dependency to io
* Remove unneeded CMakeLists.txt files
* Merge remote-tracking branch 'v4r_origin/master'
* use more bf paths
* remove unused stuff
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Fix warnings
* remove comments
* fix some warnings
* fix typos
  use bf::path instead of std::string for filepaths
* use bf::path instead of std::string for filepaths
  Format style
* fix errors
  remove redundant if
* update style guide documentation
  fix warnings
* fix remaining style in modules, apps and samples
* reformat core
* reformat change_detection, recognition, segmentation and ml
* reformat features
* reformat keypoints
* reformat 'io'
* add clang-format file
* remove redundant if
* remove unused normal estimation with pre-processing
* fix another bunch of warnings
* fix warnings
  Textured rendering
  Disable Gtest by default and add a download timeout
  Set universal imported location for source built dependencies
* Disable Gtest by default and add a download timeout
  At the moment V4R has no tests, so Gtest is not needed.
* Set universal imported location for source built dependencies
  Remove support for using git-apply for patching
* small fixes in depth map renderer
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Remove support for using git-apply for patching
  Revamp third-party dependency system
* Disable -Wshadow which floods the logs
* Remove unused CMake functions/macros
* Better SYSTEM/PRIVATE include handling
* Disable annoying debug print in CMake
* New framework for managing 3rd-party dependencies
* fix yet another typo in CONTRIBUTING.md
* fix typo in CONTRIBUTING.md
* Update CONTRIBUTING.md
* 2.0.2
* update changelog and version.h before releasing V4R
  add ODM and indicate existing libraries inside V4R
* add brief description on Readme and some styling
* update readme
* fix typo
* add ODM and indicate existing libraries inside V4R
  revert readme overwrite
* revert readme overwrite
  Remove all mentions of Qt4 and always use Qt5
* Remove all mentions of Qt4 and always use Qt5
  update author list
  update contribution page
  Update readme
  Remove explicit VTK dependency
* remove VTK and OpenNI
* fix style
* Remove explicit VTK dependency
* update license URL
* split issue tracker URL
* fix URL
* fix URLs
  Remove all references to OpenCL
* update contribution page
  Textured rendering
* update author list
* update Readme
  fix license
* Remove all references to OpenCL
  Was inherited from OpenCV build system, but not used in V4R.
* fix license
* update deps
* fix conflicts
  Strands to v4r in readme
* update dependencies description
* As we build ceres within v4r this is not needed anymore
* Update readme to get rid of Strands links
  OpenNURBS does not need to depend on PCL
* OpenNURBS does not need to depend on PCL
  The only usage of PCL in the code is PCL_EXPORTS macro, which anyway evaluates to nothing on Unix platform.
  Build deb packages on all repos when tagged
  Do not process some RTMT headers with Qt MOC
  # Conflicts:
  #   .gitlab-ci.yml
  cleanup gitlab_CI and build fixes
* Build deb packages on all repos when tagged
* Merge remote-tracking branch 'v4r_origin/master'
* Added rendering of textured meshes.
* Do not process some RTMT headers with Qt MOC
  This is to avoid classical Qt MOC/Boost error:
  usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
  apps/RTMT2/CMakeFiles/RTMT2.dir/build.make:87: recipe for target 'apps/RTMT2/moc_OctreeVoxelCentroidContainerXYZRGB.cxx' failed
* Remove "-dev" string from version
* Remove duplicate job entry
* Merge remote-tracking branch 'upstream/master'
  Merge back the released state from v4r
* 2.0.1
* update changelog before releasing V4R
* Merge remote-tracking branch 'upstream/master'
  update version in package.xml
* update version in package.xml
* Merge remote-tracking branch 'upstream/master'
  Master
* bug fix: removed cam.center_x ... it's not available in pcl-stable
* bug work-fix: removed OdmTexturing::loadMesh (not available in pcl)
* New RTMT2 version including a modified version of OpenDroneMap texture mapping
* test odm texturing
* do not compile multiview recognizer
* fixed: we did not compile on xenial
* Add missing headers an source files.
  Arrange them in lexicographical order
* Contributors: Georg, Johann Prankl, Markus 'Bajo' Bajones, Markus Bajones, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer, Unknown, V4R Release Manager

2.0.4 (2017-11-17)
------------------
  Use Ninja on CI server
* 2.0.3
* update changelog and version.h before releasing V4R
* Automatically set 30 second download timeout for all external projects
* Use CMAKE_GENERATOR when configuring external projects
* Use Ninja generator in CMake
* Strip trailing spaces in GitLab CI config
  Cmake updates
* Remove unused submodules in `_CHILDREN` variables
* Backport various updates for utils from OpenCV master
* Remove unused functions/macros
* Remove Windows-only branch in CMake scripts
  Fix install rules for third-party dependencies
  Fix error ‘boost::Q_FOREACH’ has not been declared
* Do not clear variables after debug cache print
* Fix install rules for third-party dependencies
  Install only if enabled and built.
* Fix error ‘boost::Q_FOREACH’ has not been declared
  Solution from: https://stackoverflow.com/a/17610731/1525865
  use our own server, as people reported issues with the keyservers
  use nullptr instead of NULL
* use nullptr instead of NULL
  Add radical
* Add radical 3rd-party dependency
* Improve dependency status printing
  Detect targets and print their location
* use our own server, as people reported issues with the keyservers
  Rendering of Normals
  Fix warnings
* Fixed one stupid blank space.
* fix some more warnings
* fix warnings
* Reduced the amount of violation of our Coding Style Guideline.
  Master
* add glog dependency to io
* use more bf paths
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Ceres 1.13 clean
  Remove unneeded CMakeLists.txt files
  Closes #36
* Merge remote-tracking branch 'origin/master'
* Added a normal rendering, in case no normal is provided by the mesh it has a fallback to a per triangle estimation of normals.
* Updating Readme with Ceres 1.13
* Delete 3rdpartie's cmakelists
* Moving 2 Ceres 1.13
  Update cmake/3rdparty/BuildCeres.cmake
* add glog dependency to io
* Remove unneeded CMakeLists.txt files
* Merge remote-tracking branch 'v4r_origin/master'
* use more bf paths
* remove unused stuff
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Fix warnings
* remove comments
* fix some warnings
* fix typos
  use bf::path instead of std::string for filepaths
* use bf::path instead of std::string for filepaths
  Format style
* fix errors
  remove redundant if
* update style guide documentation
  fix warnings
* fix remaining style in modules, apps and samples
* reformat core
* reformat change_detection, recognition, segmentation and ml
* reformat features
* reformat keypoints
* reformat 'io'
* add clang-format file
* remove redundant if
* remove unused normal estimation with pre-processing
* fix another bunch of warnings
* fix warnings
  Textured rendering
  Disable Gtest by default and add a download timeout
  Set universal imported location for source built dependencies
* Disable Gtest by default and add a download timeout
  At the moment V4R has no tests, so Gtest is not needed.
* Set universal imported location for source built dependencies
  Remove support for using git-apply for patching
* small fixes in depth map renderer
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Remove support for using git-apply for patching
  Revamp third-party dependency system
* Disable -Wshadow which floods the logs
* Remove unused CMake functions/macros
* Better SYSTEM/PRIVATE include handling
* Disable annoying debug print in CMake
* New framework for managing 3rd-party dependencies
* fix yet another typo in CONTRIBUTING.md
* fix typo in CONTRIBUTING.md
* Update CONTRIBUTING.md
* 2.0.2
* update changelog and version.h before releasing V4R
  add ODM and indicate existing libraries inside V4R
* add brief description on Readme and some styling
* update readme
* fix typo
* add ODM and indicate existing libraries inside V4R
  revert readme overwrite
* revert readme overwrite
  Remove all mentions of Qt4 and always use Qt5
* Remove all mentions of Qt4 and always use Qt5
  update author list
  update contribution page
  Update readme
  Remove explicit VTK dependency
* remove VTK and OpenNI
* fix style
* Remove explicit VTK dependency
* update license URL
* split issue tracker URL
* fix URL
* fix URLs
  Remove all references to OpenCL
* update contribution page
  Textured rendering
* update author list
* update Readme
  fix license
* Remove all references to OpenCL
  Was inherited from OpenCV build system, but not used in V4R.
* fix license
* update deps
* fix conflicts
  Strands to v4r in readme
* update dependencies description
* As we build ceres within v4r this is not needed anymore
* Update readme to get rid of Strands links
  OpenNURBS does not need to depend on PCL
* OpenNURBS does not need to depend on PCL
  The only usage of PCL in the code is PCL_EXPORTS macro, which anyway evaluates to nothing on Unix platform.
  Build deb packages on all repos when tagged
  Do not process some RTMT headers with Qt MOC
  # Conflicts:
  #   .gitlab-ci.yml
  cleanup gitlab_CI and build fixes
* Build deb packages on all repos when tagged
* Merge remote-tracking branch 'v4r_origin/master'
* Added rendering of textured meshes.
* Do not process some RTMT headers with Qt MOC
  This is to avoid classical Qt MOC/Boost error:
  usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
  apps/RTMT2/CMakeFiles/RTMT2.dir/build.make:87: recipe for target 'apps/RTMT2/moc_OctreeVoxelCentroidContainerXYZRGB.cxx' failed
* Remove "-dev" string from version
* Remove duplicate job entry
* Merge remote-tracking branch 'upstream/master'
  Merge back the released state from v4r
* 2.0.1
* update changelog before releasing V4R
* Merge remote-tracking branch 'upstream/master'
  update version in package.xml
* update version in package.xml
* Merge remote-tracking branch 'upstream/master'
  Master
* bug fix: removed cam.center_x ... it's not available in pcl-stable
* bug work-fix: removed OdmTexturing::loadMesh (not available in pcl)
* New RTMT2 version including a modified version of OpenDroneMap texture mapping
* test odm texturing
* do not compile multiview recognizer
* fixed: we did not compile on xenial
* Add missing headers an source files.
  Arrange them in lexicographical order
* Contributors: Georg, Johann Prankl, Markus 'Bajo' Bajones, Markus Bajones, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer, Unknown, V4R Release Manager

2.0.5 (2018-01-19)
------------------
  Make sure the same OpenCV version is used to build both radical and V4R
  Clang format
* Make sure the same OpenCV version is used to build both radical and V4R
* clang-format
* allow to set a subset of object models to load
  avoid overwrite of existing rendered files
* avoid overwrite of existing rendered files
* Update v4r_style_guide.md
* Update v4r_style_guide.md
  remove pcl serialization from header files
  remove glog includes from header files
* remove glog includes from header files
  this solves potential conflicts with multiple LOG definitions
  fix some warnings
  remove dead code
* fix some warnings
* remove dead code
* remove pcl serialization from header files
  avoids potential multiple definitions when linking to external libraries
  remove unused files
  improve readability for boost program options
  optimize includes
  fix seg fault
* remove unused files
* improve readability for boost program options
* optimize includes
* fix seg fault
  make HV single templated
  add bound for generated hypotheses per object
* Update ObjectRecognizer.cpp
  # Conflicts:
  #   modules/recognition/include/v4r/recognition/local_recognition_pipeline.h
  # Conflicts:
  #   modules/recognition/src/hypotheses_verification.cpp
  allow to change visualization layout by parameter
  Use bf path and glog more extensively
* Update local_pipeline.xml
* Update hv_config.xml
* Update hv_config.xml
* fix typo
  remove outlier variable
  add xml parameters for local recognition pipeline
  update xml to fix warnings output and improve readability
* update xml to fix warnings output and improve readability
* make HV single templated
* add bound for generated hypotheses per object
* use bf_path and glog more extensively
* remove outlier variable
* add xml parameters for local recognition pipeline
  fix bug when color is ignored in HV
  fix confidence bug
* fix bug when color is ignored in HV
* fix confidence bug
* Use enum class
* use enums
* allow to change visualization layout by parameter
  remove dead code
  Code simplification
* Update organized_edge_detection.h
  Turn third-party dependencies into proper CMake targets
  Closes #40
* remove unused files
* :lipstick:
* simplify stuff
* fix conflicts
  Fix some warnings
* fix some warnings
* Switch to NEW behavior on CMP0025 policy
* Improve the code responsible for extracting imported library location
  Move to a separate function in V4RUtils.cmake.
* Add protection from multiple inclusion of V4R config
* Use built-in CMAKE_CXX_STANDARD instead of directly manipulating flags
  Also set CMAKE_CXX_STANDARD in exported V4R config
* Upgrade CMake debugging facilities
  * Add ability to print target properties
  * Clear debug flags after each CMake run
  * Move debugging functions to a separate file
* Generate a config file for third-party dependencies
* Remove convoluted external linker dependency handling code
  Not necessary anymore since all third-party dependencies are proper imported targets.
* Update SiftGPU finder script to create an imported library
* Do not define unnecessary variables in Radical finder script
* Update Qt finder script to create an imported library
* Update PCL_1_8 finder to not define unnecessary variables
* Update PCL finder script to create an imported library
* Update OpenNURBS finder script to create an imported library
* Do not define unnecessary variables in OpenMP finder script
* Fix dependencies of visualize_hypothesis tool
* Update OpenGL finder script to create an imported library
* Update OpenCV finder script to create an imported library
* Update GLM finder script to create an imported library
* Update Eigen finder script to create an imported library
* Update METSlib finder script to create an imported library
* Update LibSVM finder script to create an imported library
* Update Gtest finder script to not define unnecessary variables
* Update Glog finder script to create an imported library
* Update GLEW finder script to create an imported library
* Update EDT finder script to create an imported library
* Update Ceres finder script to not define unnecessary variables
* Caffe finder script already creates imported target, don't define unnecessary variables
* Update boost finder script to create an imported library
* Update Assimp finder script to create an imported library
* Remove BINDINGS module class support (inherited from OpenCV)
* Remove "wrapper" modules (inherited from OpenCV and not used)
* Temporary fixup on v4r_get_all_libs
* Setup include directories on V4R module targets
* Fix resolving of external dependencies (priority for targets)
* Always make link libraries public to ensure dependency propagation
* Update v4r_dependency_status function to handle target-only dependencies
* Add v4r_add_imported_library utility function
* Fix condition in v4r_install_dependencies macro
* Abolish V4RMinDepVersions.cmake and embed version information into finder scripts
  Force cmake 3.10 (hackish) and fix parallel debian package build
* Update .gitlab-ci.yml
* fix parallel build, force newer cmake version
  make change detection optional in Object Reco
* fix typo
* Update ObjectClassification.md
  fix some warnings
* fix some warnings
  Fix compatibility with OpenCV 3.3.1
* Fix compatibility with OpenCV 3.3.1
  Avoid argument type ambiguity in cv::goodFeaturesToTrack() calls.
  Lift minimum CMake required version to 3.5.1
* Add CMake to the list of required dependencies in README
* Raise minimum required CMake version to 3.5.1
* Install CMake 3.10.0 on Trusty in setup.sh
  On Xenial default apt-get package is used. This makes sure that on both
  supported systems we have at least CMake 3.5.1.
  EuclideanClusterComparator does not actually use normals
* EuclideanClusterComparator does not actually use normals
  Associated template parameters and functions have been deprecated in
  current PCL master. This conditional compilation is to avoid warnings
  and prepare for future complete removal of these functions.
  Master
* make change detection optional in Object Reco
* changed order of qt/ pcl includes
* changed params: dist thr. for colour opti. (to avoid zero correspondences)
* bug fix 14.04: removed pcl::TextureMesh
* support radical for RTMT2
  add pcl texturing structure to odm (compatibility to old pcl)
  Fix installation of headers
  Closes #39
* Fix installation of headers
  Was introduced by c8511da6
* 2.0.4
* update changelog and version.h before releasing V4R
  Use Ninja on CI server
* 2.0.3
* update changelog and version.h before releasing V4R
* Automatically set 30 second download timeout for all external projects
* Use CMAKE_GENERATOR when configuring external projects
* Use Ninja generator in CMake
* Strip trailing spaces in GitLab CI config
  Cmake updates
* Remove unused submodules in `_CHILDREN` variables
* Backport various updates for utils from OpenCV master
* Remove unused functions/macros
* Remove Windows-only branch in CMake scripts
  Fix install rules for third-party dependencies
  Fix error ‘boost::Q_FOREACH’ has not been declared
* Do not clear variables after debug cache print
* Fix install rules for third-party dependencies
  Install only if enabled and built.
* Fix error ‘boost::Q_FOREACH’ has not been declared
  Solution from: https://stackoverflow.com/a/17610731/1525865
  use our own server, as people reported issues with the keyservers
  use nullptr instead of NULL
* use nullptr instead of NULL
  Add radical
* Add radical 3rd-party dependency
* Improve dependency status printing
  Detect targets and print their location
* use our own server, as people reported issues with the keyservers
  Rendering of Normals
  Fix warnings
* Fixed one stupid blank space.
* fix some more warnings
* fix warnings
* Reduced the amount of violation of our Coding Style Guideline.
  Master
* add glog dependency to io
* use more bf paths
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Ceres 1.13 clean
  Remove unneeded CMakeLists.txt files
  Closes #36
* Merge remote-tracking branch 'origin/master'
* Added a normal rendering, in case no normal is provided by the mesh it has a fallback to a per triangle estimation of normals.
* Updating Readme with Ceres 1.13
* Delete 3rdpartie's cmakelists
* Moving 2 Ceres 1.13
  Update cmake/3rdparty/BuildCeres.cmake
* add glog dependency to io
* Remove unneeded CMakeLists.txt files
* Merge remote-tracking branch 'v4r_origin/master'
* use more bf paths
* remove unused stuff
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Fix warnings
* remove comments
* fix some warnings
* fix typos
  use bf::path instead of std::string for filepaths
* use bf::path instead of std::string for filepaths
  Format style
* fix errors
  remove redundant if
* update style guide documentation
  fix warnings
* fix remaining style in modules, apps and samples
* reformat core
* reformat change_detection, recognition, segmentation and ml
* reformat features
* reformat keypoints
* reformat 'io'
* add clang-format file
* remove redundant if
* remove unused normal estimation with pre-processing
* fix another bunch of warnings
* fix warnings
  Textured rendering
  Disable Gtest by default and add a download timeout
  Set universal imported location for source built dependencies
* Disable Gtest by default and add a download timeout
  At the moment V4R has no tests, so Gtest is not needed.
* Set universal imported location for source built dependencies
  Remove support for using git-apply for patching
* small fixes in depth map renderer
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Remove support for using git-apply for patching
  Revamp third-party dependency system
* Disable -Wshadow which floods the logs
* Remove unused CMake functions/macros
* Better SYSTEM/PRIVATE include handling
* Disable annoying debug print in CMake
* New framework for managing 3rd-party dependencies
* fix yet another typo in CONTRIBUTING.md
* fix typo in CONTRIBUTING.md
* Update CONTRIBUTING.md
* 2.0.2
* update changelog and version.h before releasing V4R
  add ODM and indicate existing libraries inside V4R
* add brief description on Readme and some styling
* update readme
* fix typo
* add ODM and indicate existing libraries inside V4R
  revert readme overwrite
* revert readme overwrite
  Remove all mentions of Qt4 and always use Qt5
* Remove all mentions of Qt4 and always use Qt5
  update author list
  update contribution page
  Update readme
  Remove explicit VTK dependency
* remove VTK and OpenNI
* fix style
* Remove explicit VTK dependency
* update license URL
* split issue tracker URL
* fix URL
* fix URLs
  Remove all references to OpenCL
* update contribution page
  Textured rendering
* update author list
* update Readme
  fix license
* Remove all references to OpenCL
  Was inherited from OpenCV build system, but not used in V4R.
* fix license
* update deps
* fix conflicts
  Strands to v4r in readme
* update dependencies description
* As we build ceres within v4r this is not needed anymore
* Update readme to get rid of Strands links
  OpenNURBS does not need to depend on PCL
* OpenNURBS does not need to depend on PCL
  The only usage of PCL in the code is PCL_EXPORTS macro, which anyway evaluates to nothing on Unix platform.
  Build deb packages on all repos when tagged
  Do not process some RTMT headers with Qt MOC
  # Conflicts:
  #   .gitlab-ci.yml
  cleanup gitlab_CI and build fixes
* Build deb packages on all repos when tagged
* Merge remote-tracking branch 'v4r_origin/master'
* Added rendering of textured meshes.
* Do not process some RTMT headers with Qt MOC
  This is to avoid classical Qt MOC/Boost error:
  usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
  apps/RTMT2/CMakeFiles/RTMT2.dir/build.make:87: recipe for target 'apps/RTMT2/moc_OctreeVoxelCentroidContainerXYZRGB.cxx' failed
* Remove "-dev" string from version
* Remove duplicate job entry
* Merge remote-tracking branch 'upstream/master'
  Merge back the released state from v4r
* 2.0.1
* update changelog before releasing V4R
* Merge remote-tracking branch 'upstream/master'
  update version in package.xml
* update version in package.xml
* Merge remote-tracking branch 'upstream/master'
  Master
* bug fix: removed cam.center_x ... it's not available in pcl-stable
* bug work-fix: removed OdmTexturing::loadMesh (not available in pcl)
* New RTMT2 version including a modified version of OpenDroneMap texture mapping
* test odm texturing
* do not compile multiview recognizer
* fixed: we did not compile on xenial
* Add missing headers an source files.
  Arrange them in lexicographical order
* Contributors: Georg, Johann Prankl, Markus 'Bajo' Bajones, Markus Bajones, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer, Unknown, V4R Release Manager

2.0.8 (2018-08-23)
------------------
  some updates
  use V4R namespace for intrinsics >> operator
  fix bug in HV when smooth clustering is disabled
* use V4R namespace for intrinsics >> operator
* use struct instead of public class members
* remove unneeded EIGEN_MAKE_ALIGNED_OPERATOR_NEW
* use template name PointT
* use default destructor
* use glog
* fix bug in HV when smooth clustering is disabled
* fix doc
* :lipstick:
* remove redundant code
* fix clang compilation errors
  remove redundant camera class
  some updates
* :lipstick:
* fix Feature based view registration
  fix clang errors
* fix fast_icp compilation errors
* remove redundant camera class
* fix clang errors
  :lipstick:
* :lipstick:
  Fix eigen alignment issues
  Use new ceres version
  fix object reco tutorial
  add feature type stream operator
  remove dead metrics file
  fix spelling
* use new ceres version 1.14
  use git lfs for SIFTGPU source files
* fix spelling
* add feature type stream operator
* fix object reco tutorial
* remove dead metrics file
* fix eigen alignment issues
* use git lfs for SIFTGPU source files
  Add libvtk-qt and freenect and sort package.xml entries
* Add libvtk-qt and freenect and sort package.xml entries
  fix spell checking errors
* fix spell checking errors
* Contributors: Markus Bajones, Sergey Alexandrov, Thomas Faeulhammer, Thomas Fäulhammer

2.0.7 (2018-06-21)
------------------
  Introduce CI_ORIGIN_NAMESPACE variable in CI config
* Introduce CI_ORIGIN_NAMESPACE variable in CI config
  hv: remove dead params, fix bug for disabled smooth clustering
* hv: remove dead params, fix bug for disabled smooth clustering
  add camera parameter defaults to boost program options
* add camera parameter defaults to boost program options
  Thomasf/update object gt annotator
* update Object Ground Truth Annotator
  fix bug when depth_registration_mask  is empty
* fix bug when depth_registration_mask  is empty
  add V4R updates
  use ${OPENGL_LIBRARIES}  instead of GL for cmake OpenGL library
* use ${OPENGL_LIBRARIES}  instead of GL for cmake OpenGL library
* add V4R updates
  - removes METSLIB dependency
  - implements own conditional clustering
  - updates z-buffering
  - some more HV updates
  add const and override expressions
  fixed cv::Mat segfault
* add const and override expressions
* fixed cv::Mat segfault
  Master
* bug fixes: alignment
  Master
  Remove repo.acin.tuwien.ac.at from v4r
  fix bug when scene_explained_weights vector is uninitialized
* fix bug when scene_explained_weights vector is uninitialized
  <PointT> ereased to fix the color layer in the visualization
  Enhance hv visualization
* <PointT> ereased to fix the color layer in the visualization
* fix bug in hv visualization due to using auto for Eigen bool vectors
* some hv visualization improvement for smooth clusters
* Finally remove last references to repo.acin.tuwien.ac.at
* add missing variable
* Fix download links
* Remove old links from nyu preparation scripts.
  Update gitlab-ci config
* add missing name change
* Update gitlab-ci config
  Enable V4R_WARNINGS_ARE_ERRORS on bionic
  Rename build script to emphasize that xenial needs to be treated in a special manner and bionic (and later releases hopefully) need not.
* formatting...
* 17.10 eigen stuff (memcpy instead of = operator)
* style ...
* on 17.10 eigen crashes. changed setIdentity to identity constructor and = operator to memcpy
  remove unused parameter init functions
* remove unused parameter init functions
  fix bug in RTMT2
  remove supervoxel code that generates warnings for newer PCL versions
  Closes #62
* remove unused variables in semantic segmenation which also fix warnings
* remove supervoxel code that generates warnings for newer PCL versions
  code was only used for visualization of supervoxels
* fix bug in RTMT2
  remove dead code (updated file already exists in samples/tools)
* remove dead code (updated file already exists in samples/tools)
  remove dead code
* remove dead multiview object reco app code
  remove deprecated typename for ObjectHypothesis object
* remove deprecated typename for ObjectHypothesis object
  Require at least PCL 1.7.2
* Require at least PCL 1.7.2 and remove fallback code for earlier versions
  Enable release builds using the new docker CI infrastructure
* move to Dockerfile to reducetime spent in release builds
* fix type
  More constant conversions
  update lsh parameters
* Follow-up to !425, forgot to commit most of the changes.
  Rename CV\_* color conversion constants to cv::COLOR\_*
  enable checking for NaN values in extracted feature descriptors
* update lsh parameters
  speeds up matching
* enable checking for NaN values in extracted feature descriptors
  this can now handle both float and bool signature types
* Rename CV\_* color conversion constants to cv::COLOR\_*
  CV\_* style of constants is deprecated and will be removed from OpenCV 4.
  The new style is supported by OpenCV 2.4, so not breaking anything with
  this change.
  fix wrong resize of indices vector
* fix wrong resize of indices vector
* Install debhelper for release builds
* fix variable name
  remove packages that we already installed during container creation
* Reuse docker containers for release builds
  Reduce warnings on Ubuntu 18.04
* Fix some warnings with pcl 1.8 and gcc 7
  use constructor depending on pcl version
  do not create iterator if we don't use it
  allow v4r object recognition to be tested on specific files also (rather than
* allow v4r object recognition to be tested on specific files also (rather than
* Set download file name for siftgpu
* 3DEF new server url
* move siftgpu to new server
* Update 3d-net scripts.
* We moved all data away from repo.acin.tuwien.ac.at. First step to do so in the code as well.
  Make v4r compile on Ubuntu 18.04
* switch back to v4r/mvs-texturing
* Add Ubuntu 18.04 and ROS Melodic support to CI
  add pointer typedef for depthmap renderer class
* add pointer typedef for depthmap renderer class
  fix bug in reading parameter for textured mesh tool
* fix bug in reading parameter for textured mesh tool
  Reorganize test data
  Closes #60
  Apply calibration
* Minor Changes.
  Remove hack, remove pcl 1.8 from CMakeLists.txt and add comments
* Code Cleanup
* Apply Calibration, Initial Commit
* Reorganize test data directory
  * Move data files under their respective module directory;
  * Track data files with Git LFS.
* Warn the user if test data file is not available
  Fix compute recognition rate
* output warning instead of error if input is empty for pcd2image convert
* fix save images to disk when visualization is off
* quit program if parameter input is not okay
* move load models to avoid bug when externally setting object properties
  :lipstick:
* :lipstick:
* update parameter initialization
  simplify visualization for object reco
* use correct view port
* use correct point type for V4R models
* fix eigen alignment issues
* simplify visualization for object reco
  add FREAK
  :lipstick:
* :lipstick:
* update config so it runs for default libopencv-dev package
* add FREAK
  Motion Blur Detection.
* Minor Changes and Updates
  Doxygen
  Minor fixes
* Merge definition and declaration of MiddlePoint ctor to avoid multiple definition error
* Fix a typo and remove trailing spaces in the style guide
  add sample for mesh2pointcloud
  Remove unneeded glog initialization
* Remove unneeded glog initialization
  This will be called inside ProgramOptions constructor.
* Update v4r_style_guide.md
* add sample for mesh2pointcloud
  remove storing elapsed time from stopwatch destructor to fix seg fault
* avoid redundant start time init
* :lipstick:
* use default argument for ScopeTime title
* move StopWatch/ScopeTime to common
* remove storing elapsed time from stopwatch destructor to fix seg fault
* Motion Blur Detection.
  Motion Blur Detection using the Variance of Laplacian (VL) and the Variance of a Modified Laplacian (VML)
  use bf::path for filepaths in object model
  Refactor feature stuff
  remove deprecated image_local_estimator header
* use bf::path for filepaths in object model
* remove deprecated image_local_estimator header
* fix compilation error for OpenCV 2.4
* add SIFT GPU to object reco pipeline
* set descriptor name by the combination of keypoint detector and feat est
* make gray image a class member variable again
* add boost options for ORB
* group feature parameters into separate struct
* use detectAndCompute for same keypoint detector and feat estimator
* add ORB to object reco pipeline
* use relative paths for class pointer typedefs
* update names in error message
* reduce scope of variable
* make gray image a local variable as it is not used otherwise
* add BRISK as iostream
* fix bug in object local reco pipeline setup (confused feat detector and extractor)
* add object mask to harris
* add some featDet to object local reco pipeline setup and fix sift compilation error
* use enum class for feature detectors
* fix brisk compilation for opencv 2.4
* use enum class for feature detectors
* use right includes for opencv features
* return by const reference
* include sift only if xfeatures opencv module exists
* add some modularity
* clang format
* make recognition pipeline setup easier by allowing only a single local 2D feature estimator
* add brisk and mser to reco params
* use relative namespace
* add MSER
* allow to split feature detection and description
* return descriptor name
* add const expression
* return descriptor name
* add brisk
* fix FeatureDetector init in AKAZE and SURF
* rename surf and akaze
* convert AKAZE and SURF to FeautureDetector API
* replace sift_local_estimator with FeatureDetector_KD_SIFTGPU
* use FeatureDetector SIFT implementation for Feature Based registration
* add static siftgpu matching function
* use structs instead of classes for sift parameters
  Add brute force matching possibility
* add brute force matching possibility
* fix compile error on OpenCV 2.4 with knnmatch
* use flann-based matcher
  Img loader
* Minor Changes, remove opencv.hpp
  make models dir required and some :lipstick:
* ImgLoader
  Adding Class to load all images from given folder
  Refactor feature stuff
* add include for cv::keypoint
  Update v4r_style_guide
* Update v4r_style_guide
  Adding Required CLANG version (5.0), and  Update QtCreator's Beautifier Setup to be compliant with our coding style.
* use American English for meters
* :lipstick:
* use CV_MAJOR_VERSION macro directly
* add field for descriptor name
* rename feature detector functions to match opencv names
* add computation of keypoint indices
* :lipstick:
* add object mask as optional parameter
* add override final specifiers
* remove unused include
* clang-format
* include cv sift from hannes to cmakelists
* allow to use OpenCV 3.x for SIFT
* tidy up
* add brief description
* add brief description
* merge autors
* use pragma once
* make models dir required and some :lipstick:
  Master
* formatting
* missing include (Ubuntu 17.10)
* added iostream.h (Ubuntu 17.10)
* added type specification to std::min (Ubuntu 17.10, gcc 7.2.0 compatibility)
  Filter out non-existent include directories when creating imported library
* Filter out non-existent include directories when creating imported library
  Remove code that detects compiler version
* Remove code that detects compiler version
  * It is not used anyway;
  * It does not work with GCC 7.
  Add StructureCore grabber
* Add --overlay option to tool-grab
* Support saving unprojected point clouds to disk in tool-grab
* Add StructureCoreGrabber
* Add StructureCore as a third-party dependency
  Support auto-loading of calibrated intrinsics in OpenNI2 grabber
* Update OpenNI2 grabber to auto-load intrinsics
* Add Grabber::tryLoadIntrinsics()
  This function attempts to load intrinsics from standard locations.
* Add getDataDir/getConfigDir functions to filesystem.h
* Initialize Glog in ProgramOptionsBase helper class
* Log debug information in createGrabber() function
* Add clear_cache_variables debug command
  Update OpenNI2 finder script to search in OpenNI2_DIR first
* Update OpenNI2 finder script to search in OpenNI2_DIR first
  Builsystem updates
* Move disabled warnings under "Noise warnings" section and enable
* Minor cleanup in CMakeLists.txt
* Rename V4RGenConfig -> V4RGenerateConfig
* V4R does not support MSVC, no need to condition on GNUCXX
* Fix no omit frame pointer option for clang
* Update V4R_CPU_ARCH_NATIVE option
  Now accepts ON/OFF/AUTO values. ON means force -march=native, OFF means
  force no -march=native, and AUTO means use the same setting as PCL
  (default).
* Update v4r_option() macro
  Remove support for "IF <condition>", added support for constrained value
  list.
* Set CMake policy 22 to NEW and 71 to OLD
* Not necessary to append -DNDEBUG because CMake already adds it
* Add v4r_prepend utility CMake function
  Add MVSTexturing and remove ODM
  Closes #32
* Rename module surface_texturing into surface
  This matches PCL naming.
* Remove ODM texturing
* Replace ODMTexturing with MVSTexturing in camera_tracking_and_mapping example
* Replace ODMTexturing with MVSTexturing in RTMT2
* Add a CLI tool to texture meshes
* Add MVSTexturing class
* Add MVSTexturing third-party dependency
* Initialize Glog in RTMT2 to avoid logs spilling to console
* Small clean-up in RTMT2 CMakeLists
* Update tools/program_options.h
  Rename addOptions() -> addGeneral(), add addOther().
  RTMT2 refactorings and fixes
* Fix a bug in RTMT2::Sensor
  A local variable was passed over update_boundingbox() signal by
  reference. By the time receiving thread processes the signal, the
  variable goes out of scope, causing segfaults.
  The approach taken in this fix is to store current bounding box
  transform in a member field.
* Use memcpy instead of operator= to avoid sporadical segfaults
* Add EIGEN_MAKE_ALIGNED_OPERATOR_NEW to some classes
  This is absolutely needed when a class contains fixed-size vectorizable
  Eigen members (Matrix4f in this case) and is allocated on the heap
  (which is the case for these classes).
* Pass camera parameters using v4r::Intrinsics
  In near future, in order to perform texturing, ObjectSegmentation needs
  to know image dimensions. This information is contained in
  v4r::Intrinsics structure. Therefore, it is more convenient to exchange
  camera parameters through this struct rather than a combination of
  cv::Mat and width/height.
* Add const qualifiers to several methods
* Add Intrinsics::fromCameraMatrixAndResolution() factory constructor
* Rename Intrinsics::Kinect() into Intrinsics::PrimeSense()
* Refactor triple into a struct to improve code readability
  Additionally fixes Eigen alignment issues (a list containing Matrix4f
  objects should use aligned allocator).
  Modernize and tidy code in IO module
* Modernize and tidy code in IO module
  Add a new CI stage with spelling check
* Add a new CI stage with spelling check
  Fix spelling mistakes in code, comments, and filenames
* Rename files with spelling mistakes
* Get rid of local variables with names suspicious for codespell
* Rename local object such that codespell does not complain
* Fix spelling mistakes in code and comments
  Fix a bug when a 3p dependency can appear multiple times
* Fix a bug when a 3p dependency can appear multiple times
  add surf and akaze
* add some parameter
* remove unused/redundant includes
* make AKAZE and SURF compile conditionally on installed OpenCV library
* add surf and akaze
  Fix submodule enabled/disabled macro
* Fix submodule enabled/disabled macro
  Add information about submodule state to config.h
* Add information about submodule state to config.h
* Remove obsolete v4r_config.h.in template
  Add support of version requirement specification for dependencies
* Add support of version requirement specification for dependencies
  With this patch it is possible to request a particular version of dependency, e.g. opencv>=3.2.
  tidy up after updating parameter IO
* tidy up after updating parameter IO
  update parameter IO
  Fix log message in local feature matching
* Fix log message in local feature matching
  Do not allow clang-format job to fail
* clang format
* Do not allow clang-format job to fail
  allow to set KeypointType via String
* allow multiview object reco params to be set via boost program options
* use new parameter IO
* :lipstick:
* change multiview param max_views name
* allow to set KeypointType via String
* allow to set KeypointType via String
* allow shot params to be set via boost program options
* fix doxy comment
* allow global recognizer params to be set via boost program options
* allow gcg params to be set via boost program options
* allow local feat match params to be set via boost program options
* allow local recognition pipel params to be set via boost program options
* allow pcl visualization params to be set via boost program options
* allow camera params to be set via boost program options
* allow rendering params to be set via boost program options
* allow HV params to be set via boost program options
  Use std pointers
  Implement new docker-based CI infrastructure
* Implement new docker-based CI infrastructure
* fix comments
* serialize object instead of pointer to fix compile error
* replace boost with std shared_ptr
  Drop support for Ubuntu 14.04
* Drop support for Ubuntu 14.04
  * remove CI jobs
  * update README
  * update setup.sh
  Also cleans up a bit package.xml file.
  fix_typo
* fix_typo
  Only use CMAKE_CXX_FLAGS when CMAKE_BUILD_TYPE is None
  Do not run check/build stages on tags
* Only add V4R extra flags for not None build types
* Abolish all C flags related CMake variables
  V4R does not have C code.
* Do not run check/build stages on tags
  remove comments and unused code
  save SIFT keypoint ange and -size same as Hannes
  Add a CMake option to enable "-march=native"
* Replace ENABLE_xxx options with a single switch for -march=native
* In the configuration summary only report active C++ and Linker flags
* save SIFT keypoint ange and -size same as Hannes
* remove comments and unused code
  optimize by using SSE instruction and accessing elements with []
  add eigen aligned allocator
* use const auto references instead of Eigen::Vector4f
  :lipstick:
* add eigen aligned allocator
* :lipstick:
  fix bug of kept coordinate axis in object reco visualization
  remove deprecated files
* fix bug of kept coordinate axis in object reco visualization
* optimize by using SSE instruction and accessing elements with []
* remove deprecated files
  Lipstick
* remove redundant cast
* remove dead includes
* use nullptr
  save all point clouds in a binary compressed format
* save all point clouds in a binary compressed format
  Fix a bug introduced by !353
* Fix a bug introduced by !353
  Export only public third-party dependencies
* Export only public third-party dependencies
  Third-party dependencies that are private for V4R or are not used
  (because modules that need them are disabled) should not be exported to
  V4RDependencies.cmake.
  Change order of paths in RPATH
* Change order of paths in RPATH
  Without this change the V4R library installation path is the first in
  RPATH of the installed binaries. Therefore, if V4R is installed to a
  standard system location "/usr/local/lib", other custom installed
  libraries may be picked up from there, overriding 3rdparty libraries
  provided by V4R.
  Example: there is a Ceres installed in "/usr/local/lib", but V4R is
  configured to build it's own version of Ceres. This own version is
  installed to "/usr/local/share/V4R/3rdparty/lib" and has lower priority
  than the one in "/usr/local/lib".
  Remove flann class
* update checks\_ variable description
* include distance check
* use different parameters for knn test
* fix warning in knn test for unimplented distance metrics
* add test for nearest neighbor classifier
* fix bug
* add virtual destructor for classifier
* remove flann
* :lipstick:
* move distance metric to own cpp file
  add kdtrees check param as explicit parameter
* add kdtrees check param as explicit parameter
  also increases default value to get better matching accuracy
  Add RealSense2 grabber
  Add unit testing framework
  Closes #18
* Run tests in the end of CI build tasks
* Add example test for v4r::Intrinsics
* Add simple unit testing framework
* Remove explicit installation of some dependencies in Xenial CI task
  These dependencies are istalled through rosdep.
* Add librealsense2 to the Xenial CI task
* Add io::RealSense2 grabber
* Add RealSense2 third-party dependency
  Attention segmentation refactor (2)
* attention segmentation refactor:
  Refactoring attention segmentation related code parts. Main refactorings done in the list below:
  - substitute exit() and return statements with exception
  - adapted SVM training code to use random parameter search
  - handle negative curvatures in surface modeling
  - clang-format new version
  - remove orphaned files
  - added pretrained svm models for applications
  - adapted install for apps
  - include google logging and deleting printf statements
  Minor updates for the grabber framework
* Output metric depth maps from Grabber::grabFrame()
  Different grabbers may have different depth units, thus 16-bit integer maps are not a good format.
* Use cv::cvtColor() in OpenNI2Grabber for conversion between RGB and BGR
* Remove unused member fields in OpenNI2Grabber
  Update RTMT2 to use v4r::Intrinsics
* Use Intrinsics::getCameraMatrix() in RTMT2 where appropriate
* Add Intrinsics::getCameraMatrix()
* Replace custom RGBDCameraParameter with Intrinsics from v4r_common
* Change the source of RGBDCameraParameter
  Formerly the camera parameters came from the Preferences dialog. If changed by the user, the parameters propagated to GLViewer and Sensor via slots/signals.
  This commit removes camare parameters section from the dialog. The source of the camera intrinsics is now io::Grabber. The retrieved parameters are stored in the Sensor object and signaled to the other parts of RTMT.
  Make sure that EDT is built with the same PCL as the rest of V4R
* Improve feature information printing in Grabber
* Refactor mode selection in OpenNI2 grabber
* Add a helper function selectStreamMode()
* Add initializers and constructors to Grabber::StreamMode
  replace V4R smartPtr with boost::shared_ptr
  Closes #50
* Make sure that EDT is built with the same PCL as the rest of V4R
* replace V4R smartPtr with boost::shared_ptr
  Make CI jobs fail on compiler warnings
  Closes #44
* Make CI jobs fail on compiler warnings
  remove unused (copyrighted) SLICO
  Closes #42
* remove unused (copyrighted) SLICO
  fix warnings
* fix warnings
  remove duplicated friend entry
  use cv mat signatures instead of float vectors and use opencv flann
* remove duplicated friend entry
  RTMT2 refactoring
* Use modern style of `Qt::connect` without `SIGNAL/SLOT` macros
* Abolish "Apply" button in preferences dialog
* Remove unused includes
* Rename a number of member fields
* Remove unused member fields
  * cos_min_delta_angle
  * sqr_min_cam_distance
  * log_clouds
  * cameras
  * prev_voxel_size
  * prev_filter_z
  * indices
  * pass
* Rename members related to camera tracking
* Remove inv_pose member field and rename pose into camera_pose\_
* Rename camera matrix and normal/plane estimation member fields
* Rename member fields pertaining to bounding box filtering
* Remove unused functions and typedefs
  * sqr()
  * AlignedPointXYZRGBVector
  * cam_id
  * conf
  * cloud_mutex
* Use pcl::isFinite() instead of custom isNaN()
* Remove empty documentation comments
* use cv mat signatures instead of float vectors and use opencv flann
  allow to select subset of object models to be detected
  add tool for computing image registration mask
  use enum classes
* allow to select subset of object models to be detected
* use enum classes
  :lipstick:
* add tool for computing image registration mask
* :lipstick:
  Update RTMT to use io::Grabber
* Add "stream uri" parameter to RTMT
  Stream URI can be chosen in the "Preferences" dialog.
* Update RTMT to use io::Grabber
  This removes cloud callback, queue, and mutexes.
  Add unproject() function
  Update createGrabber() to automatically work with whichever grabbers are built
* Add --3d visualization option to tool-grab
* Add v4r::unproject()
* Update createGrabber() to automatically work with whichever grabbers are built
  Implement submodule concept in CMake
  Master
  Closes #51
* clang stuff
* bug fix: wrong index in TSFilterCloudsXYZRGB::getCameraParameter
* Remove large number of unused CMake functions/macros
* Reimplement module system
  * Single "right" way to create a module (v4r_add_module)
  * Support private dependencies
  * Submodules for optional components with dependencies
  on.cpp
* Replace v4r_target_link_libraries with plain target_link_libraries
  They are the same.
* Remove unused v4r_add_perf_tests()
* Remove unused v4r_add_accuracy_test()
* Make v4r_add_dependencies internal macro and rename
* Remove v4r_include_modules/recurse functions
  Nowadays target_link_libraries takes care of includes automatically.
* Remove Windows-specific branches in CMake scripts
* Abolish V4R_MODULE_xxx_LINK_DEPS variable (not used anymore)
  First part of grabber framework
* Add a tool to grab and visualize RGB-D stream from a camera or a file
* Add an utility class ProgramOptionsBase to simplify writing CLI tool apps
* Add io::createGrabber() function
  Empty so far
* Add io::PCDGrabber
* Add io::OpenNI2Grabber
  remove vedaldi
* remove unused vedaldi SIFT implementation
* Add abstract base class io::Grabber
* Add simple Intrinsics struct to v4r_common
  remove deprecated multi-view object recognition files
  Closes #49
* Enable JAVADOC_AUTOBRIEF option in Doxygen config
* remove deprecated multi-view object recognition files
  Cmake simplifications
  Use awk to avoid grep -P (not available in alpine image)
* Use awk to avoid grep -P (not available in alpine image)
  List misformatted files when clang-format check fails on CI
  Reduce debconf dialog warnings on fresh docker image
* Fix "debconf: delaying package configuration, since apt-utils is not installed" warnings
* List misformatted files when clang-format check fails on CI
  fix ACIN repo server download error
  some minor fixes
* Remove some commented out lines
* Remove remaining variables pertaining to CUDA in module scripts
* Better docstring for v4r_glob_modules
* Remove unused function v4r_module_disable()
  Also removed associated variable V4R_MODULES_DISABLED_FORCE.
* Abolish the concept of module class (PUBLIC/INTERNAL)
  There is no need in internal modules in V4R.
* :lipstick:
* fix ACIN repo server download error
  Rework module listing in the CMake summary
* Rework module listing in the CMake summary
  add doc
  Improve configuration header files
* add doc
* Reduce debconf dialog warnings on fresh docker image
* Upgrade v4r/config.h
  * A macro for every module and third-party dependency (even disabled) is
  now defined.
  * A consexpr function is provided to check module/dependency state at
  runtime
  See cmake/templates/config.h.in for complete instructions on how to use.
* Update generated config/modules header files
  * Merge v4r_config.h and v4r_modules.h
  * Rename and move v4r_config.h to v4r/config.h
* Introduce V4R_MODULES_ALL list in CMake scripts
  Add OpenNI2 as optional third-party dependency
  Module v4r_common should not depend on v4r_io
  Improve formatting tools
* Add OpenNI2 as optional dependency
* Small tidy-up in CMakeLists
  * Remove unused options
  * Remove commented out code
  * Sort policies
  * Enable compile commands exporting
* Module v4r_common should not depend on v4r_io
* Improve pre-commit hook
  Gracefully handle situation when some of the files to be committed is
  only partially staged.
* Make sure clang-format 5.0 is used in clang-format-all
  Silence last v4r warnings in xenial build
  put debian build into deploy stage
* put debian build into deploy stage
* Silence warnings in xenial build
  Fix formatting after the last MR
* Fix formatting after the last MR
  master
  Check coding style on CI server
* removed PSiftGPU and directly added SiftGPU to FeatureDetector_KD_SIFTGPU
* Add pre-commit hook that runs code through clang-format
  To activate the hook, run from the repository root:
  $ rm -rf .git/hooks && ln -s ../scripts/dev/hooks/ .git
* Create a CI job that checks style
* Add a script that applies clang-format to a directory
  Reformat V4R using clang-format-5.0
* Reformat V4R using clang-format-5.0
  fix warnings
* fix warnings
  Cleaning up attention segmentation.
* Cleaning up attention segmentation.
  Cleanup Attention Segmentation review changes
  - use namespace for function in standalone programs
  - remove some unnecessary comments
  Attention segmentation cleanup:  changed copies to const reference pointers
  Formatting source code files.
  Reordering of includes
  Fix a bug in ON_Curve::EvaluatePoint
* Fix a bug in ON_Curve::EvaluatePoint
  Backported from upstream PCL (https://github.com/PointCloudLibrary/pcl/pull/2185)
  Add libusb to the build dependencies
  Closes #43
* Add libusb-1.0-dev as a build requirement
  This is needed to avoid endless
  <command-line>:0:15: warning: ISO C++11 requires whitespace after the macro name
  warnings in the CI server output.
* Remove unused CMake code/variables inherited from OpenCV
  Explicitly set build type to Release for all 3rd-party libraries
* Explicitly set build type to Release for all 3rd-party libraries
* 2.0.5
* update changelog and version.h before releasing V4R
  Make sure the same OpenCV version is used to build both radical and V4R
  Clang format
* Make sure the same OpenCV version is used to build both radical and V4R
* clang-format
* allow to set a subset of object models to load
  avoid overwrite of existing rendered files
* avoid overwrite of existing rendered files
* Update v4r_style_guide.md
* Update v4r_style_guide.md
  remove pcl serialization from header files
  remove glog includes from header files
* remove glog includes from header files
  this solves potential conflicts with multiple LOG definitions
  fix some warnings
  remove dead code
* fix some warnings
* remove dead code
* remove pcl serialization from header files
  avoids potential multiple definitions when linking to external libraries
  remove unused files
  improve readability for boost program options
  optimize includes
  fix seg fault
* remove unused files
* improve readability for boost program options
* optimize includes
* fix seg fault
  make HV single templated
  add bound for generated hypotheses per object
* Update ObjectRecognizer.cpp
  # Conflicts:
  #   modules/recognition/include/v4r/recognition/local_recognition_pipeline.h
  # Conflicts:
  #   modules/recognition/src/hypotheses_verification.cpp
  allow to change visualization layout by parameter
  Use bf path and glog more extensively
* Update local_pipeline.xml
* Update hv_config.xml
* Update hv_config.xml
* fix typo
  remove outlier variable
  add xml parameters for local recognition pipeline
  update xml to fix warnings output and improve readability
* update xml to fix warnings output and improve readability
* make HV single templated
* add bound for generated hypotheses per object
* use bf_path and glog more extensively
* remove outlier variable
* add xml parameters for local recognition pipeline
  fix bug when color is ignored in HV
  fix confidence bug
* fix bug when color is ignored in HV
* fix confidence bug
* Use enum class
* use enums
* allow to change visualization layout by parameter
* bug fix: changed cmake to new style
  remove dead code
* bug fix dbg viz
  zoom param gl viewer
  Code simplification
* temporal filter demo
* Update organized_edge_detection.h
  Turn third-party dependencies into proper CMake targets
  Closes #40
* remove unused files
* :lipstick:
* simplify stuff
* fix conflicts
  Fix some warnings
* fix some warnings
* Switch to NEW behavior on CMP0025 policy
* Improve the code responsible for extracting imported library location
  Move to a separate function in V4RUtils.cmake.
* Add protection from multiple inclusion of V4R config
* Use built-in CMAKE_CXX_STANDARD instead of directly manipulating flags
  Also set CMAKE_CXX_STANDARD in exported V4R config
* Upgrade CMake debugging facilities
  * Add ability to print target properties
  * Clear debug flags after each CMake run
  * Move debugging functions to a separate file
* Generate a config file for third-party dependencies
* Remove convoluted external linker dependency handling code
  Not necessary anymore since all third-party dependencies are proper imported targets.
* Update SiftGPU finder script to create an imported library
* Do not define unnecessary variables in Radical finder script
* Update Qt finder script to create an imported library
* Update PCL_1_8 finder to not define unnecessary variables
* Update PCL finder script to create an imported library
* Update OpenNURBS finder script to create an imported library
* Do not define unnecessary variables in OpenMP finder script
* Fix dependencies of visualize_hypothesis tool
* Update OpenGL finder script to create an imported library
* Update OpenCV finder script to create an imported library
* Update GLM finder script to create an imported library
* Update Eigen finder script to create an imported library
* Update METSlib finder script to create an imported library
* Update LibSVM finder script to create an imported library
* Update Gtest finder script to not define unnecessary variables
* Update Glog finder script to create an imported library
* Update GLEW finder script to create an imported library
* Update EDT finder script to create an imported library
* Update Ceres finder script to not define unnecessary variables
* Caffe finder script already creates imported target, don't define unnecessary variables
* Update boost finder script to create an imported library
* Update Assimp finder script to create an imported library
* Remove BINDINGS module class support (inherited from OpenCV)
* Remove "wrapper" modules (inherited from OpenCV and not used)
* Temporary fixup on v4r_get_all_libs
* Setup include directories on V4R module targets
* Fix resolving of external dependencies (priority for targets)
* Always make link libraries public to ensure dependency propagation
* Update v4r_dependency_status function to handle target-only dependencies
* Add v4r_add_imported_library utility function
* Fix condition in v4r_install_dependencies macro
* Abolish V4RMinDepVersions.cmake and embed version information into finder scripts
  Force cmake 3.10 (hackish) and fix parallel debian package build
* Update .gitlab-ci.yml
* fix parallel build, force newer cmake version
  make change detection optional in Object Reco
* fix typo
* Update ObjectClassification.md
  fix some warnings
* fix some warnings
  Fix compatibility with OpenCV 3.3.1
* Fix compatibility with OpenCV 3.3.1
  Avoid argument type ambiguity in cv::goodFeaturesToTrack() calls.
  Lift minimum CMake required version to 3.5.1
* Add CMake to the list of required dependencies in README
* Raise minimum required CMake version to 3.5.1
* Install CMake 3.10.0 on Trusty in setup.sh
  On Xenial default apt-get package is used. This makes sure that on both
  supported systems we have at least CMake 3.5.1.
  EuclideanClusterComparator does not actually use normals
* EuclideanClusterComparator does not actually use normals
  Associated template parameters and functions have been deprecated in
  current PCL master. This conditional compilation is to avoid warnings
  and prepare for future complete removal of these functions.
  Master
* make change detection optional in Object Reco
* changed order of qt/ pcl includes
* changed params: dist thr. for colour opti. (to avoid zero correspondences)
* bug fix 14.04: removed pcl::TextureMesh
* support radical for RTMT2
  add pcl texturing structure to odm (compatibility to old pcl)
  Fix installation of headers
  Closes #39
* Fix installation of headers
  Was introduced by c8511da6
* 2.0.4
* update changelog and version.h before releasing V4R
  Use Ninja on CI server
* 2.0.3
* update changelog and version.h before releasing V4R
* Automatically set 30 second download timeout for all external projects
* Use CMAKE_GENERATOR when configuring external projects
* Use Ninja generator in CMake
* Strip trailing spaces in GitLab CI config
  Cmake updates
* Remove unused submodules in `_CHILDREN` variables
* Backport various updates for utils from OpenCV master
* Remove unused functions/macros
* Remove Windows-only branch in CMake scripts
  Fix install rules for third-party dependencies
  Fix error ‘boost::Q_FOREACH’ has not been declared
* Do not clear variables after debug cache print
* Fix install rules for third-party dependencies
  Install only if enabled and built.
* Fix error ‘boost::Q_FOREACH’ has not been declared
  Solution from: https://stackoverflow.com/a/17610731/1525865
  use our own server, as people reported issues with the keyservers
  use nullptr instead of NULL
* use nullptr instead of NULL
  Add radical
* Add radical 3rd-party dependency
* Improve dependency status printing
  Detect targets and print their location
* use our own server, as people reported issues with the keyservers
  Rendering of Normals
  Fix warnings
* Fixed one stupid blank space.
* fix some more warnings
* fix warnings
* Reduced the amount of violation of our Coding Style Guideline.
  Master
* add glog dependency to io
* use more bf paths
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Ceres 1.13 clean
  Remove unneeded CMakeLists.txt files
  Closes #36
* Merge remote-tracking branch 'origin/master'
* Added a normal rendering, in case no normal is provided by the mesh it has a fallback to a per triangle estimation of normals.
* Updating Readme with Ceres 1.13
* Delete 3rdpartie's cmakelists
* Moving 2 Ceres 1.13
  Update cmake/3rdparty/BuildCeres.cmake
* add glog dependency to io
* Remove unneeded CMakeLists.txt files
* Merge remote-tracking branch 'v4r_origin/master'
* use more bf paths
* remove unused stuff
* conditionally render based on v4r_rendering available
* add view rendering to object recognizer
* allow prefix names of model database input to be changed
* use bf::path instead of std::string for filepaths
  Fix warnings
* remove comments
* fix some warnings
* fix typos
  use bf::path instead of std::string for filepaths
* use bf::path instead of std::string for filepaths
  Format style
* fix errors
  remove redundant if
* update style guide documentation
  fix warnings
* fix remaining style in modules, apps and samples
* reformat core
* reformat change_detection, recognition, segmentation and ml
* reformat features
* reformat keypoints
* reformat 'io'
* add clang-format file
* remove redundant if
* remove unused normal estimation with pre-processing
* fix another bunch of warnings
* fix warnings
  Textured rendering
  Disable Gtest by default and add a download timeout
  Set universal imported location for source built dependencies
* Disable Gtest by default and add a download timeout
  At the moment V4R has no tests, so Gtest is not needed.
* Set universal imported location for source built dependencies
  Remove support for using git-apply for patching
* small fixes in depth map renderer
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Loading and NOT SCALING the model is default option now. Added option to only render NORTH HEMISPHERE views.
* Remove support for using git-apply for patching
  Revamp third-party dependency system
* Disable -Wshadow which floods the logs
* Remove unused CMake functions/macros
* Better SYSTEM/PRIVATE include handling
* Disable annoying debug print in CMake
* New framework for managing 3rd-party dependencies
* fix yet another typo in CONTRIBUTING.md
* fix typo in CONTRIBUTING.md
* Update CONTRIBUTING.md
* 2.0.2
* update changelog and version.h before releasing V4R
  add ODM and indicate existing libraries inside V4R
* add brief description on Readme and some styling
* update readme
* fix typo
* add ODM and indicate existing libraries inside V4R
  revert readme overwrite
* revert readme overwrite
  Remove all mentions of Qt4 and always use Qt5
* Remove all mentions of Qt4 and always use Qt5
  update author list
  update contribution page
  Update readme
  Remove explicit VTK dependency
* remove VTK and OpenNI
* fix style
* Remove explicit VTK dependency
* update license URL
* split issue tracker URL
* fix URL
* fix URLs
  Remove all references to OpenCL
* update contribution page
  Textured rendering
* update author list
* update Readme
  fix license
* Remove all references to OpenCL
  Was inherited from OpenCV build system, but not used in V4R.
* fix license
* update deps
* fix conflicts
  Strands to v4r in readme
* update dependencies description
* As we build ceres within v4r this is not needed anymore
* Update readme to get rid of Strands links
  OpenNURBS does not need to depend on PCL
* OpenNURBS does not need to depend on PCL
  The only usage of PCL in the code is PCL_EXPORTS macro, which anyway evaluates to nothing on Unix platform.
  Build deb packages on all repos when tagged
  Do not process some RTMT headers with Qt MOC
  # Conflicts:
  #   .gitlab-ci.yml
  cleanup gitlab_CI and build fixes
* Build deb packages on all repos when tagged
* Merge remote-tracking branch 'v4r_origin/master'
* Added rendering of textured meshes.
* Do not process some RTMT headers with Qt MOC
  This is to avoid classical Qt MOC/Boost error:
  usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
  apps/RTMT2/CMakeFiles/RTMT2.dir/build.make:87: recipe for target 'apps/RTMT2/moc_OctreeVoxelCentroidContainerXYZRGB.cxx' failed
* Remove "-dev" string from version
* Remove duplicate job entry
* Merge remote-tracking branch 'upstream/master'
  Merge back the released state from v4r
* 2.0.1
* update changelog before releasing V4R
* Merge remote-tracking branch 'upstream/master'
  update version in package.xml
* update version in package.xml
* Merge remote-tracking branch 'upstream/master'
  Master
* bug fix: removed cam.center_x ... it's not available in pcl-stable
* bug work-fix: removed OdmTexturing::loadMesh (not available in pcl)
* New RTMT2 version including a modified version of OpenDroneMap texture mapping
  Master
* remove remnants from Hannes' commit
* Test commit
* test odm texturing
  Master
* stop threads
* GNUv3 license header for the attention-module
  replace license
* update license info
* bug fix: removed TextureMesh tex_mesh
* bug fix: pcl-stable has no texturing -> removed it
* changed license to GNUv3
* Merge remote-tracking branch 'v4r-internal/license'
* bug fix: store inv pose, test pcl-texturing
* continue license update
* replace license
  update gitignore file to ignore clion stuff and cmake-build-* folders
* update gitignore file to ignore clion stuff and cmake-build-* folders
  Split focal length
* bug fix: outlier detection in mapping
* bug fix activate/ deactivate optimization of cam params
* default config (no upsampling)
* changed license header to MIT
* rtmt2 with Nguyen noise model, part of the license headers changed to mit
* split focal length into x and y components
  add license and update readme
* add license and update readme
* Allow to set camera from outside
  add camera getter function
* add camera getter function
  define config folder rather than individual files
* change back to system wide config install
* install config files into v4r home directory
* install config files
* define config folder rather than individual files
  Fix semantic segmentation opencv bug
  Closes #27
* fix semantic segmentation compile error due to opencv bug
  Remove def/undef of PCL_NO_PRECOMPILE
* batch keyframe optimization: added bilin. colour transfere to keyframe
* rtmt2 (not tested, missig ui for parameter settings)
* do not compile multiview recognizer
* fixed: we did not compile on xenial
* Add missing headers an source files.
  Arrange them in lexicographical order
* Remove def/undef of PCL_NO_PRECOMPILE
* create and store object model for recognition (RTMT2)
* rtmt create cloud and mesh model (ui)
* removed warning (shadow event)
* bilin col (no thr as for depth), upscaling option
* rtmt2 with TSFVisualSLAM
* changed to batch version for TSFVisualSLAM
* Contributors: Georg, Johann Prankl, Marcin Bodych, Markus 'Bajo' Bajones, Markus Bajones, Markus Suchi, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer, Unknown, V4R Release Manager, adam_chudas

1.4.9 (2017-07-28)
------------------
* merge master into release
* fix changelog, package.xml
  Fix installation of 3rd party shared libs
  Closes #29
  Include and install Ceres as 3rd party
* Add CMake command to install libsvm.so
* Add 3rd party library directory to the CMAKE_INSTALL_RPATH
* fixed old naming issue
* install ceres as 3rd party library
  Because of Ubuntu not fixing a bug in their ceres package [1]  we need to add it as a 3rd party lib.
  [1] https://bugs.launchpad.net/ubuntu/+source/ceres-solver/+bug/1596296
  Remove useless include
* Remove useless include
* Update .gitlab-ci.yml
* Update .gitlab-ci.yml
* update changelog
* Test: Enable deb generation on Ubuntu 16.04
* merge
  RGB-D cloud filtering using a sequence of clouds, (batch version, average of depth and colour)
  add semantic segmentation module + apps and scripts for NYU Datasets
* add semantic segmentation module + apps and scripts for NYU Datasets
  bugfix: renamed one function
  adapted command line arguments
  adapted scripts, moved some files from git to repo server
  Update Readme.md
  Update Readme.md
  removed unused include files (esp. openni2_grabber.h)
  add readme file for semantic segmentation apps
  update scripts and readme
  add matlab scripts for nyu depth dataset conversion
  add apps for semantic_segmentation
  add module semantic_segmentation
* changed name of tsf and added example
* tsf batch filtering
  Fix global pipeline param bug
  Forward declare caffe net
* fix bug when global pipeline parameters are not completely defined
  Fix caffe
  Closes #25
* comment pcl conflict
* comment definitions
* forward declare caffe net
  avoid caffe include
* fix missing include paths
  Fix OpenCV Bug for unordered sets
* Add Caffe Path hints
* Fix OpenCV Bug for unordered sets
* bug fix: turn off mapping -> tsf-filtering still collected frames
  Master
* bug fix:
  - TSFGlobalCloudFilteringSimple: set width of the point cloud (if not ec-filtered)
  - example: parameter setting: do not ec-filter
* added modul: camera tracking and mapping (using s temporal smoothing filter - TSF)
* add multiview keypoint correspondence visualization
  Pcl mesh renderer
* removed loading of pcl files with pcl::io::loadPLYFile. This method 1. tends to fail and 2. does not exist on the continuous integration system.
* Cleanup of the changes. Finally got rid of the flipped coordinate system bug.
* Added a lot of debug code but principially found the bugs causing the projection to be inverted. TODO: remove debug code!!
* no idea (must be minor)
* Added new constructor accepting pcl meshes.
* update doc
* update doc
  Add change detection
* update doc
* update CMakeLists
* remove Eval app
* fix error when reading floating point occlusion values
* avoid running into pcl bug because corr_rejector ransac is not reset
* remove unused method
* make aligned
* :lipstick:
* read training data despite loading model from file
* add more verbosity
* allow to load SVM model from file
* update default param
* update default svm parameter range
* fix seg fault when saving model
* save trained svm to current working directory
* add svm scaling
* allow setting kp_transfer
* allow to sub-sample views to speed up evaluations
* allow to set knn from command line
* allow to do icp outside HV
* write refined pose
* remove condition
* remove opencv sift as it is integrated in sift local estimator
* weight feature vector so they approximately scale to 1
* add some boost program options
* remove unused icp parameter
* update min dist for cluster factor default parameter value
* remove debug
* update parameter
* temp debug info
* remove debugging information again
* remove exit
* bf
* add more debug info
* temporary for debugging
* update
* more info
* :lipstick:
* fix compile error
* up
* temporary verbose logging
* :lipstick:
* up
* update visualization
* update params
* use openmp for zbuffering
* fix min_dist_for_cluster parameter for gc
* add multiview visualization for kp transfer
* avoid redundant recognition rate computations
* use full model for z-buffering in HV
* add timing and do some common pre-checks
* update author order
* :lipstick:
* pre-compute search radius
* set rendering default background-color to white
* add missing pieces for remove non-upright objects
* serialize hypotheses
* add option to remove hypotheses not standing upright on a support plane
* do not output confusion matrix twice
* allow subsampling during rendering
* remove unused normal computation for model
  should be done outside
  fix nasty bug in normals computation (flip always towards viewpoint)
* use auto
* use auto
* revert normal referencing
* add missing declarations
* revert normal referencing
* fix visualization bug when skip verification is on
* use reference
* use reference
* fix nasty bug in normals computation (flip always towards viewpoint)
* fix doxygen comments
* visualize curvature information
* fix background color bug
* allow to save images to disk
* remove leftover for visualiization
* add option in xml
* just to make sure
* add ICRA version of multi-view classifier
* add missing piece for param gc grouping
* get recognition pipelines
* allow to disable correspondence clustering
* make deep copy
* fix bug when views are empty
* add try catch
* :lipstick:
* update params
* add keypoint redundancy check
* use tanh function for xyz score and re-name parameter
* temporarily add some hack to avoid running into bug for outlier cloud visualization
* add some more information for timing
* fix some timing measurement bug
* make shuffle views a parameter
* add reading of computation time
* :lipstick:
* fix bug in recognition rate compuation when dot product becomes 1+eps
* add multiview evaluation
* fix nasty bug in normals computation (flip always towards viewpoint)
* add normals visualization
* fix multiview and icp stuff, add max views parameter
* backup of some old code
* remove some deprecated stuff
* partially fix multi-view registration
* remove unused template parameter
* add xtion depth mask
* add some logs
* add global config
* update params
* :lipstick:
* update rec error computation and visualize errors
* fix visualization
* fix recognition rate bug
* add missing files
* update dependency list
* update color and normals comparison
* make HV param private
* add dependencies
* fix bug when hypothesis does not explain any scene point
* :lipstick:
* remove empty appearance module
* adapt L channel
* make fitness threshold dependent on visibility
* :lipstick:
* put params in separate file
* :lipstick:
* Merge remote-tracking branch 'root/master' into add_change_detection
* :lipstick:
* update url for siftgpu
* update timings log
* re-arrangechecks to hopefully speed-up a bit
* fix rec file output and add timings
* fix visualization
* reset mv
* improve performance and brevity of noise model based cloud integration
* add copyrights
* integrate change detection again
* add param for mutliview hv
* use separate parameter class for noise model based registration + fix remaining const madness
* add noise model based cloud integration
  not quite working yet
* :lipstick:
* const madness
* update multiview
  save verification status intrinsically
  correctly transfer hypotheses across views
* add verified member
* make objectrecognizer mv compatible
* update param
* allow online mv rec
* opti model - structure (no impl)
* 1.4.5
* update changelog
* set ros distro
* update changelog
* Contributors: Daniel Wolf, Johann Prankl, Markus Bajones, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer, bajo

1.4.8 (2017-04-11)
------------------
* Merge remote-tracking branch 'upstream/master' into release
  update url for siftgpu
  Closes #24
* update url for siftgpu
  Add eval
* up
* update params
* just to make sure
* :lipstick:
* use inner product for normal comparison
* up
* use pcl_version macro
* update param
* update z-buffering
* use dotproduct for normals comparison
* use dist_l2 enum from opencv 2.4
* remove highgui include
* update params
* use vis params
* use logarithm
* :lipstick:
* use param
* update pose refinement and visible cloud computation
* remove color normalization
* use pcl stopwatch to allow verbosity setting
* use tanh for color comparison
* use right color comparison function
* fix missing header
* reset viewpoint by default to fix visual
* small hack to have consistent number of parameters
* hopefully fixes compile error
* allow inconsistent xml file and just warn
* exit when param not okay
* use planar filter
* use integer for verbosity
* initialize depth registration mask
* add accidentally deleted header declaration
* improve readability
* fix wrong sort direction
* fix some warnings
* use surface normals for z-buffering
* rename parameter
* update hv parameter
* add verbosity
* fix
* fix compile error
* :lipstick:
* add confusion matrix computation to evaluate global pipeline
* reset sensor orientation before segmentation
* use distance instead of z component for cut-off
* updae eval to allow setting multitoken elements
* make remove planes parameter accessible from outside
* use distance instead of z component for cut-off
* save parameters to file
* add initialization
* use RGB transform histogram for global color estimator and add parameter class
* make histogram row major
* use built-in svm cross validation
* fix compile error
* update global recognizer (mainly add visualization)
* update alexnet integration
* update global recognizer to allow for multiple feature descriptors per cluster
  + add visualization option
* add to previous commit
* use eigen vector instead of PointT type as min max point
* add ourcvfh as an option for global concat estimator
* use normal estimator from outside
* make camera a class variable
* remove virtual where not necessary
* add ourcvfh estimator
* update global pose estimation and add visualization
  TODO: check if everything works correctly
* update desc
* compute discrepancy between centroids of view and centroid of full object model
* compute model centroid
* fix taking wrong norms (should be sqrt)
* use visualization parameter
* add parameter for coordinate axis scale
* add utility function for aligning two vectors
* Merge remote-tracking branch 'root/master' into add_eval
  Add GTest
* add pose refinement to app
  Add eval
  fix throwing runtime_error
  change include order to fix opencv bug
  Closes #23
* fix throwing runtime_error
* fix throwing runtime_error
* update default config (elongation can now be part of the feature vector)
* update header information
* :lipstick:
* awesome global concate descriptor
* remove parameter remnant
* :lipstick:
* fix hist range check
* add color to concat descriptor
* fix type
* add global color descriptor
* add concatenation of global descriptor
* :lipstick:
* add a simple global shape estimator
* fix visualization bug
* remove log copy as glog does not support this apparently
* write function to query boost xml value
* use glog more frequently
* remove volatile remnant
* reset normals
* :lipstick:
* make svm param gamma init to 1/num_featurse by default if not otherwise set
* add unfiltered global results to vis
* make global rec visualization a parameter
* fix coordinate system vis issue
* fix global recognizers plane alignment
* allow to return normals from segmentation
* update opencv sift
* use l1 norm
* output recognition rate
* revert accidental commit
* fix wrong endif for rops
* fix sift
* fix rops comile error on pcl < 1.7.2
* allow to visualize keypoints
* update params
* allow having multiple keypoint extractor + change default params
* update xml
* comment try catch
* fix bug
* tmp commit
* :performance:
* use organized normal computation by default
* remove redundant object mask check
* fix sift opengl issue
  TODO: use master thread for sift to use openmp
* allow having multiple support radii
* :lipstick:
* remove keypoint cloud and processed cloud from local estimator
* Merge remote-tracking branch 'root/master' into add_eval
  Add rotation check
* change include order to fix opencv bug
* change include order to fix opencv bug
* remove points not belonging to object during initialization
* visualize normals in processed cloud
* add rops as features
  not tested yet
* make shot support radius a boost parameter
* remove nan points before  shot computation
* remove keypoint indices if the normals are not finite and estimator needs normals
* use omp
* fix segfault
* change default params
* allow setting boundary type
* avoid keypoint cloud copy if not necessary
* add narf parameters
* fix bug
* remove config files as they are copied all the time anyway
* add parameter for shot and normal estimation
* increase normal densitiy in visualization
* remove check
* change default param
* :lipstick:
* add check if file exists when reading xml files
* use stl vector and :lipstick:
* :lipstick:
* :lipstick:
* :lipstick:
* make normal computation method a parameter
* another try with rootsift
* add missing piece
* add rootsift as parameter
* add l2 normalization
* fix
* try root sift
* revert test
* test
* fix eval stopping criteria
* fix compile error
* fix hash
* fix compile error on pcl 1.8 (take 2)
* fix compile error on pcl 1.8 (take 2)
* fix compile error on pcl 1.8
* fix compile error on pcl 1.8
* add eval
* remove eval
* add missing executable in cmake
* add point types include to gcg to hopefully fix compile error on trusty
* add inidividiual hypotheses check with ground-truth data + :lipstick:
  add angular threshold parameter
* temp commit
* compute normals when init features
* allow graph-based correspondence grouping
* use cv::Vec3b
* also use stl vectors for thresholds
* fix not removing already used parameter
* make gcg input const
* make retrain a parameter for outside
* also use 2d stl vector instead of mat for center
  + fix some warnings
* use 2d stl vector instead of cv mat to fix memory leak
* fix compile error
* Add GTest as a third-party library
* add angular threshold parameter
* increase model resolution for visualization
* make compute recognition rate an apps library
* remove plane extractor from global and use segmentation app
* :wrench:
* :sparkles:
* add angular threshold parameter
* Add protection from calling target_include_directories with empty list
* Remove INSTALL_TESTS option from CMake
* Install only modules marked as PUBLIC
* Merge remote-tracking branch 'refs/remotes/root/master'
  Conflicts:
  modules/segmentation/src/plane_utils.cpp
  modules/segmentation/src/segmentation_utils.cpp
  fix opening multiple pcl visualization windows
  add simons plane extractor
  add hellinger kernel
* init params
* add boost parameter
* remove redundant param init
* init params
* init params
* add boost parameter
* add boost parameter
* :lipstick:
* fix compile error
* cleanup
* fix wrong index
* flip normals always towards viewpoint
* only vector4f now
* temp commit
* temp commit
* temp commit
* tmp commit
* temp commit
* :lipstick:
* remove redundancy
* reduce redundancy
* reduce redundancy
* use plane utility functions
* use global functions
* use const
* remove cam
* add some plane utils
* :lipstick:
* fix compile error
* cleanup
* fix wrong index
* flip normals always towards viewpoint
* only vector4f now
* temp commit
* temp commit
* temp commit
* tmp commit
* temp commit
* :lipstick:
* remove redundancy
* reduce redundancy
* use plane utility functions
* use global functions
* rename config variable name
* use const
* remove cam
* add some plane utils
* add seg params
* add processed cloud to visualization and use original cloud for hv
* update cloud segmenter
* allow title change
* use boost paths
* use boost filesystems paths instead of string
  will be casted anyway
* remove visualization output
* extract plane indices and increase performance
* fix opening multiple pcl visualization windows
  define visualizer static
* enable plane removal by default
* use separate cloud for processing input
* fix opening multiple pcl visualization windows
  define visualizer static
* extract plane indices and increase performance
* add hellinger kernel
* add simons plane extractor
* update recognizer to use new segmentation with explicit plane extraction
* add hellinger kernel
* add simons plane extractor
* Merged branch master into add_evaluation_for_diss
  ec filter of small clusters - some beautification
* ec filter of small clusters - some beautification
  Update io
  Master
* RTMT: remove small clusters before storing the global model
* use random search for hyperparameter
* prepare for local search
* fix performance bug
* put changes in separate file and automatically add pairwise changes
* add functionality to set output dir
* add eval in cmake
* do not visualize by default
* fix missing occlusion threshold init
* fix bug in visualization
* init commit
* add xml config for multipipeline
* :lipstick:
* add parameter class for multipipeline recognizer
* add remove folder
* update folder copy
  Up recognition rate computation
* add chi-square
* add parameter for num trees of kdtree in feature matching
* use object centroid for translation error computation
* make eval compatible to new annotation format
* object models are read directly from folder instead from init source
  remove deprecated functions and make paths windows compatible
  revert disabling of some apps and examples
* revert disabling of some apps and examples
* minor: config/ test ImGD-Descriptor
* remove deprecated functions and make paths windows compatible
  bug fix in random forest training
  Rec eval
* fix pcl version madness
* avoid not available getkeypointsindices in PCL < 1.7.2
* remove conditional pcl instantiation
* lm optimization of the poses (proj. + depth) -- tested
* allow to skip verification
* Merged branch master into normal_estimation
* update keypoint example demo
* fix bug when indices are empty
* update default param
* :lipstick:
* update normal computations
* add z adaptive
* init normal class
* fix noise model based cloud integration when object mask is not available
* fix missing init
* update segmentation
  - use vector<int> instead of pcl::pointindices
  - separate plane extraction and segmentation
  - rename files
  - move some definitions into .cpp
* tmp commit
* rename variables
* remove siftgpu dependency
* remove pcl 1_8 dependency
* adds example code for keypoint extraction and filtering
* add dependencies
  add dependencies
* make segment pcd an app
* fix seg fault when input contains nan points
* fix wrong if conditions
* remove siftgpu dependency
* pnp and/or depth ransac version (not tested)
  fix noise model based cloud integration when object mask is not available
* add dependencies
* fix noise model based cloud integration when object mask is not available
* fix redundant typename in keypoint init
* Merged branch master into rename_params
* live version (capture with opencv)
* bug fix in random forest training
  boost random generator is not thread safe, do not access it from within openmp parallel section
* update segmentation
  - use vector<int> instead of pcl::pointindices
  - separate plane extraction and segmentation
  - rename files
  - move some definitions into .cpp
* tmp commit
* Merged branch master into rename_params
* rename variables
* :lipstick:
* Contributors: Daniel Wolf, Johann Prankl, Markus Bajones, Sergey Alexandrov, Thomas Faeulhammer, Thomas Fäulhammer

1.4.7 (2017-02-23)
------------------
  fix missing init
* fix missing init
  added additional parameter for imkRecognizer to specify bin filename
* changed imkRecognizer example to handle user filename
* added additional parameter to set file name for imk-bin-file
  param change: more accurate flann configuration
* param change: more accurate flann
* Contributors: Edith Langer, Johann Prankl, Markus Bajones, Thomas Faeulhammer

1.4.6 (2017-02-14)
------------------
* Merged branch master into release
* remove auto keyword as this causes undesired behavior on some machines
* Contributors: Thomas Fäulhammer, Unknown

1.4.5 (2017-02-13)
------------------
  Release 1.4.4-internal
* Contributors: Markus Bajones

1.4.4 (2017-02-13)
------------------
  fix initialize parameters
  Make sure that found Ceres library is a shared object
* Make sure that found Ceres library is a shared object
  Otherwise will get relocation error at linker stage.
* fix initialize parameters
  Segmentation, ML and Keypoint extractor updates
  update segmentation
  Build Ceres in shared mode on CI server
* Build Ceres in shared mode on CI server
* :lipstick:
* update classifiers
  - write a global iniitialization method that allows to initialize any classifier by its type
  - move parameters outside class to not make them templates
* update keypoint extractors
  - write a global iniitialization method that allows to initialize any keypoint extractor by its type
  - move parameters outside class to not make them templates
* move parameter class
* remove segmenter.cpp
* update segmentation
  - write a global iniitialization method that allows to initialize any segmenter by its type
  - use visualization from utils
  - move parameters outside class to not make them templates
  New recognizer
* put files into v4r module apps to allow to use it as a library
  fixes alignment issues with potentially wrong library versions of PCL, OpenCV, Eigen, Boost etc
* add pointer check
* add missing header file to cmake
* make classes aligned for fixed-sized vectorizable eigen objects
* Merged branch master into new_recognizer
  fixes #19 (duplicate base type for cv::Feature2D)
  Closes #19
* fix warnings
* Merged branch fix_opencv3_issue into new_recognizer
* fixes #19 (duplicate base type for cv::Feature2D)
* update readme
* fix some warnings
* do smooth cluster check in global optimization, move visualization option away from param, cleanup
* fix visualization
* move hv visualization in separate file/class, update hv evaluation
  + cleanup
* add is_outlier visualization
  + :lipstick:
* fix visualization bug
* :lipstick:
* use auto and fix indentation
* make model resolution consistent with scene resolution
* fix model color in vis
* fix filename
* fix compile error
* initial commit for mv recognition
* fix visualization issue for ghv
  + fix warning
* fix missing camera depth registration mask init for xml constructor
* fix hv crash and wrong filename input
* :lipstick:
* read camera parameters from xml file
* remove unused recognition files
* make path windows compatible
* update compute recognition rate
* fix error on PCL 1.7.2
* update annotation format
* fix pcl 1.7 error
* use conditional clustering from 3rdParty folder
* new recognizer
  this commit is too huge to comment... sorry a few months work that I can't split up anymore :-P
* create class for zBuffering, PCL-2-OpenCV converter and Occlusion Reasoning
* use pcl correspondence grouping instead of a copy
  Look for DevIL library and fail if not found when building SiftGPU
  Closes #20
* Look for DevIL library and fail if not found when building SiftGPU
  IMKRecognizer: create model from indices
  Cmake cleanup
* Remove some occurrences of IOS, WINRT, and ANDROID in CMakeLists
* Remove options and config variables inherited from OpenCV
* Remove unused CHECK_MODULE CMake macro
* Fix defaulting to Release mode
* load indices files (additional to to masks) for modelling
* added temporal filtering to RTMT-modelling
  Update z buff and occ reasoning
  Feat
  IMKRecognizer: keypoint based, monocular object recognizer
  fixes shadowed auto variable warning
  creates a recognition database from ply files in folder
* removed debug cout
* merged
  fix some warnings (shadowed and unused variables)
  make svm class better encapsulated
* create class for zBuffering, PCL-2-OpenCV converter and Occlusion Reasoning
* fixes shadowed auto variable warning
* creates a recognition database from ply files in folder
  (used for new recongition database format)
* add pcl visualization parameter class
* Merged branch update_ml into master
* Merged branch feat into master
* Merged branch fix_some_shadow_warnings into master
* add shift histogramm function
  + some :lipstick:
* update pcl serialization
* update camera class to include depth registration mask
* update point cloud properties computation
* use new indices
* add image crop utility function
* some sift updates
* fix some warnings (shadowed variables)
* Merged branch cleanup_segmentation into master
* remove files
* Merged branch cleanup_segmentation into master
* clean up segmentation and add plane utils
  removes specific segmenation example from modules
* make svm class better encapsulated
  put files into utility file, add const
* add cmake file to find vtk (copied from opencv)
* clean up segmentation and add plane utils
  removes specific segmenation example from modules
* Update ObjectDetection.md
* make svm class better encapsulated
  put files into utility file, add const
  put pcl files only available in PCL 1.8 into 3rdparty PCL folder
  remove docs from blacklist
  Closes #16
  fix some warnings
* remove docs from blacklist
* MIT license, colour confidence value, documentation
* MIT license header
* remove unused pcl trunk files
  fix compile error on OpenCV3
* Fix compile error
* remove comment
* fix some warnings
* fix some more missing pcl_1_8 bits
* up
* fix compile error on OpenCV3
* use uniform sampling from PCL 1.7 since 1.8 doesn't seem to provide indices
* getting mad
* up
* namespace driving me crazy
* add 2d
* some more dashes
* fix some namespace problems
* c++ does not seem to like dashes too much
* put pcl files only available in PCL 1.8 into 3rdparty PCL folder
* keypoint based monocular object recognizer
  add fov to camera
* remove redundant vertical field of view member variable
  create debian packages if commit was tagged.
* add fov to camera
  Doxy and some minor beauty
  use constptr where appropriate
  make uniform sampling work on PCL versions 1.8+
* some doxygen fixes and :lipstick:
* use constptr when necessary
* make uniform sampling work on PCL versions 1.8+
  Some minor fixes
  fix ceres version issue with renamed parameter
* add function for principal component analysis (elongation / centroid)
* neglect .autosave files
* fix  quaternion computation in case not normalized
* add focal length to boost program options
* remove openni include
* fix ceres version issue with renamed parameter
  Feat1
  This adds some functions used for recognition / hypotheses verification
  Add boost serialization for common pcl types
  add doxygen config file to be able to generate Doxygen based documentation
  remove redundant copy of PCL file
  cleans up v4r repository (redundant PCL copy - PCL is a required dependency anyway)
* Merged branch master into debian_packages
* add computeMaskFromImageMap function
* minor code reduction
* add initial histogram equalizer
  not tested yet
* add colorcomparison method enum
* remove unused std_msgs serialization
* add boost serialization for common pcl point cloud types
* some more redundant pcl copies replaced
* add doxygen config file to be able to generate Doxygen based documentation
* fix namespace issue
* replace v4r/common/eigen.h with pcl/eigen.h
  Fixed warnings in PCL
  So, this should fix every warning from external libraries. I did this by adding the necessary SYSTEM parameter to the according "(target_)include_directories" calls. As far as i see it it does not suppress any warnings in our own headers (as intended) but due to my lack of understanding of CMAKE and our V4R structure i suggest somebody might want to test it.
* Fixed another mistake... credits goes to Sergey.
* Fixed missing }
* removed comments.
* Being more selective at suppressing warnings.
  Being more selective at suppressing warnings as now only warnings from external header files will be suppressed.
  Fixed some Warnings in Keypoints
  As the title says. I hope i broke no real functionality.
* Update V4RUtils.cmake
* Suppress warnings in external headers.
  Added the SYSTEM property to V4RUtils.cmake where it is needed to suppress pcl warnings of external include files.
* fix to lowercase path name
* rename header files to be included in the packaging process
* update changelog
* Merged branch master into debian_packages
* Merge remote-tracking branch 'v4r-master/master'
* Update .gitlab-ci.yml
  Install sed, use it to compile on 8 cores
* Update .gitlab-ci.yml
  Make sure v4r is installed in /usr
* Merged branch master into master
  Integrate attention based segmentation Now works on Ubuntu 14.04 and 16.04
  @michael-zillich-1 @msuchi Can I get some feedback before I merge this?
* disable deb packages on xenial for now. issue with libceres to blame
* Adaptions for v4r source code for compiling under Ubuntu 16.04.
  + changed include from "cvmath" to <cvmath>
  + changed namespace of isnan to std::isnan
  this is tested for opencv 2.4 and pcl 1.7.2 which have to be set when launchi9ng cmake:
  cmake -DPCL_DIR=<path to pcl> -DOpenCV_DIR=<oath to opencv 2.4>
* Merged origin/attention_segmentation into master
  Update contributing.md
  Added usage of  setup.sh to CONTRIBUTING.md
* need to install devscripts
* need to install python-bloom before we can use it
* need to install python-bloom before we can use it
* Update CONTRIBUTING.md
* update gitlab-ci file
* Added dependency installation "how to" to Contributing.md
* create debian packages if commit was tagged.
* Merged branch master into master
  Ubuntu16.04+opencv3
  @ghalmets
* pass 2 parameters to setup. ubuntu and ros codename
* Removed some warnings in keypoints.
* escape variables
* update setup.sh and gitlab-ci.yml to be more generic
* gitlab's lint checker says it is OK now. Let's see.
* fix gitlab syntax after strange merge issue
* Merge remote-tracking branch 'refs/remotes/upstream/master'
  Conflicts:
  .gitlab-ci.yml
* Merged branch master into master
  Changed camera matrix input and output of the pointcloud generation class. Now i…
  Adding Setup.sh
  Added Setup.sh to v4r for a more convenient dependency installation.
  .gitlab-ci.yml was edited to use setup.sh to keep the script in the CI loop.
  Workflow:
  `git clone git@my-awesome-v4r-repo`
  `cd v4r`
  `./setup.sh`
  `mkdir build && cd build`
  `cmake ..`
  `make -j8`
* Update Readme.md
* Update Readme.md to rgit and added usage of ./setup.sh
* Changed camera matrix input and output of the pointcloud generation class. Now it is not transposed, or does not has to be transposed anymore.
* Update .gitlab-ci.yml
* Added Setup.sh
* Add script for first build
  Installing rosdep and dependencies, building v4r.
* update apps to include all programs for attention based segmentation
* update attention_segmentation module
* Add first sample app for attention based segmentation! Yes it works.
* fix header files
* fix cmake mistake
* add and activate opennurbs and on_nurbs
* shifted around attention_* files
* Merged branch master into master
* fix cmake file
* add opennurbs as build option for V4R
* reflect change of opennurbs directory
* add cmake find file for openNurbs
* moved opennurbs to 3rdparty
* delete autosave file
* change dependencies
* bring in all the files for attention segmentation -HACK
* Update examples after eputils merge
* Small changes because of the eputils merge into attention_segmentation
* We no longer need/have a v4r_eputils module
* move eputils into attention_segmentation
* Add examples for attention based segmentation
* Adapt to new v4r structure for attention based segmentation
  mainly namespace changes, V4R_EXPORTS, etc.
* Add missing files for eputils
* Adapt to new v4r structure
  namespaces, V4R_EXPORTS, etc.
* small changes to bring attention_segmentation into the new v4r structure
* small changes to bring eputils into the new v4r structure
* Initial copy of attention_segmentation from v4r svn
* Initial copy of eputils from v4r svn
  v4r now compiles with OpenCV 2.x and 3.1 on Ubuntu 14.04 and 16.04
* v4r now compiles with OpenCV 2.x and 3.1 on Ubuntu 14.04 and 16.04
  temporal filter (tracks pose and integrates several rgb-d frames)
  incl. bug fix: default param in common/occlusion_reasoning.cpp
  Update of Contribution.md with results from Structure Workshop.
  I have merged the minutes of V4R structure workshop into the Contribution.md
* Update CONTRIBUTING.md
* Fixed some Typos
* Update CONTRIBUTING.md
* temporal filter (tracks pose and integrates several rgb-d frames)
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md Update description v4r exports
* Update CONTRIBUTING.md minor changes
* Update CONTRIBUTING.md: Formatting
* Update CONTRIBUTING.md: added sections: "Structure", "Documentation", and "How to Build V4R?".
* Manually set PCL_APPS_LIBRARY
* Merged branch ubuntu16.04+opencv3 into ubuntu16.04+opencv3
* I think this is it.
* say yes to apt-get. all the time
* No debug symbols installed
* build ceres from source
* We can force the dpkg installation
* handle install with apt-get force
* Next Ubuntu hack
* fix stupid Ubuntu typo
* Introduce hack because Ubuntu
* rosdep really needs sudo. install it.
* No sudo in xenial image
* No sudo in xenial image
* fix ubuntu version. and again.
* fix ubuntu version
* Also build on Ubuntu 16.04
* fix: no default values
* I think this is it.
* say yes to apt-get. all the time
* No debug symbols installed
* build ceres from source
* We can force the dpkg installation
* handle install with apt-get force
* Next Ubuntu hack
* fix stupid Ubuntu typo
* Introduce hack because Ubuntu
* rosdep really needs sudo. install it.
* No sudo in xenial image
* No sudo in xenial image
* fix ubuntu version. and again.
* fix ubuntu version
* Also build on Ubuntu 16.04
  Update AUTHORS
  fix libsvm dependency in package.xml
* fix libsvm dependency in package.xml
* Update AUTHORS
  Update hypotheses_verification.cpp (wtf? I just committed this change)
* Update hypotheses_verification.cpp
* Update hypotheses_verification.cpp (wtf? I just committed this change)
  Master
  add missing boost dependency
* add missing boost dependency
* Go back to use standard ubuntu trusty docker image
  This is easier to support in the future.
* Merged branch master into master
* Merged branch master into master
  Fix vector type
* Update hypotheses_verification.cpp
* Update hypotheses_verification.cpp
* fix vector type
* fix vector type for new histogram interface
* Update Readme.md
  Use docker image that has those dependencies already installed
* Use docker image that has those dependencies already installed
  Update .gitlab-ci.yml
* Update .gitlab-ci.yml
* Merged branch master into master
* Update .gitlab-ci.yml
* Update .gitlab-ci.yml
  Some fixes
* Update .gitlab-ci.yml
* Merged branch master into master
* Update .gitlab-ci.yml
  yet again. ;-)
* Update .gitlab-ci.yml
* put some header definitions into cpp files and remove .hpp files
* Update .gitlab-ci.yml
* put miscellaneous functions into more specific files
* Update v4r_style_guide.md
* fix merge conflict
* Merged branch master into master
* added: only small inline functions
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Merged branch master into master
* added: keep pull requests short
* fixed typo
* fixed typo
* clean up .gitlab-ci.yml
* add dependencies description
* Update .gitlab-ci.yml
* add contributing and style_format doc files
* Update package.xml
  test if this compiles now
* Update .gitlab-ci.yml
  Continue on rosdep errors. Arrrrrr
* Update .gitlab-ci.yml
  fix syntax
* Update .gitlab-ci.yml
  specify the ROS version (needed to resolve packages from package.xml)
* Update .gitlab-ci.yml
  We need wget as well.
* Update .gitlab-ci.yml
* Update .gitlab-ci.yml
  Seems like we need cmake after all
* Add .gitlab-ci.yml
  First try
* update color transformation and comparison
* use boost::dynamic_bitset instead of vector<bool>, add camera class, put some definitions into header files + some code refactoring
* fix roi when reaching boundary
  Ubuntu 16.04 compatibility
* Merge pull request #67 from strands-project/ubuntu1604_compatibility
  Ubuntu1604 compatibility
* Merge remote-tracking branch 'hannes/master'
* add pcl time header
* Merge remote-tracking branch 'v4r-master/master'
* add bounding box function
* separate definitions from some header files to reduce compile time
* fix some warnings
* Merge pull request #66 from strands-project/sync_from_gitlab
  Sync from gitlab
  New try
* Merge pull request #64 from strands-project/new_try
  [WIP] New try
* add timing for pose refinement
* update citation file
* put pcl_opencv functions from header into implementation file
* some code optimization
* some changes for compiling with Ubuntu 16.04
* some beauty
* add script for obtaining alexNet CNN
* use const
* make destructors virtual for virtual classes
* remove empty file
* remove broken files
* put test data into directory
* fix existing directory warning
* add docs for recognition
  update get_TUW script
* add script for downloading 3dnet test data
* add doc for RTMT
* include scripts to obtain training data from TUW and 3dNet
* include missing mean subtraction in alexnet feature estimation
* update for shape cnn classifier to work
* fix wrong model assembly resolution
* fix compilation errors for eval and app pieces
* fix missing clear of indices when no keypoints are detected
* remove voxelgriddistancetransform method call
* remove default typename in createIndicesFromMask method to allow usage without c++11
* add global hypotheses non-maxima surpression to rejection method
* group hypotheses by segmentation cluster
* add online averaging function
* add hyp stuff (should have been staged earlier)
* remove EDT stuff
* check if all views are trained during initialization (not just if directory exist)
* put boost program options directly into parameter classes, merge ghv with hypotheses verification class
* make separate table class
* minor fixes for save pose into pcd
* update some visualization functions in recognition
* remove sift based alignment in incremental object learning class
* use new segmentation class and provide combined feature, keypoint and segmentation layer
* hopefully fixing Caffe optional dependency
* up pcl version
* fix compilation error caused by addcoordinatesystem if used with PCL < 1.7.2
* add esf classifier again
* fix typo in openmp call
* fix some warnings
* fix bug in optional depending on caffe and rendering
* change default params and do not instantiate harris and iss keypoint extractor on PCL versions < 1.7.2 (as keypoint indices is not available for these versions)
* make recognition library dependency to rendering and Caffe optional
* move some hpps into cpps
* skip recongition rate computation if output file already exists
* add nicer visualization for paper
* add todo comment
* add eval for rec rate over occlusion
* fix crop bug in pcl opencv conversion
* fix min fitness threshold scaling
* flip table plane towards viewpoint and make parameter for min points accessible
* make resolution as an integer in mm
* add coordinate system for visualizing recognition results
* fix bug in color conversion
* change default parameter for svm cross validation
* make smooth segmentation parameter scale with depth
* avoid table plane filtering for initialization in local recognizer
* add parameter options for smooth clustering
* add dense SIFT option (not tested yet and only available for SIFTGPU)
* add smooth clustering and linear thresholding of model fitness threshold (with visibliity)
* use multi-plane segmentation for local recognizer to find *highest* table plane
* fix visualization for recognition rate computation when cloud sensor header is changed
* temporary remove parallel call of recognizer
  QH6205 qhull error (qh_initqhull_start): qh_qh already defined.  Call qh_save_qhull() first
* fix bug in compute recognition rate
* ignore multiview and esf object classifier for now
* make model fitness threshold adaptive to visible ratio (TODO: write a proper function)
* use bigger rendering points in model cues visualization
* fix wrong sigma for AB color components
* remove table plane debug visualization
* rename some recognition init parameters
* reset view to do not mess up visualization in evaluation recognition example
* add option to just compute dominant plane (without clustering)
* fix bug with multiple call to recognize if recognizer is local
* add all the options for initialization
* make local recognizer more modular
* fix bug in knn_sift initialization
* add missing iostream include in classifier.h
* add opencv sift option again (NOTE: Not tested yet)
* remove keypoint rounding stuff in sift
* rewrite local estimator interfaces
* remove redundant files, take into acccount sign ambiguity of eigen vectors for global recognizer
* fix bug with missing normal computation
* migrated feature estimator changes (except eigen matrix). kinda working but only for first test view it seems
* add global recognizer
* add ourcvfh pcl trunk version, fix view all point clouds in folder
* merging svmwrapper, classifier classes, keypoint extractors... still working
* merged many things from rec_optimization_isolated branch (hyp generation still working - verificaiton not)
* add point cloud to eigen matrix conversion with indices
* add ptr for gcg
* remove old comments
* add ptr for gcg
* fix merge conflict
* add vector sort with indices return
* add some histogram functions
* add cielab2rgb transformation
* some code polish in graph based geometric consistency grouping
* avoid some warnings
* add visualization of model keypoints
* fix visualization of correspondences
* remove global requirement for samples to have all modules enabled
  it now only looks for the individual dependency of each sample and compiles just the ones which meet their dependencies
* using parameter class for gcg when gcg is used... small code polish
* addition to previous commit
* fix color conversion compilation error in case of PointXYZ instantiation
* make ghv compile for PointXYZ type instantation as well
* fix error with color retrieval in verification code
* optimize speed
* fix bug in model assembly
* remove parameter output
* speed up verification evaluation by compressing scene fitting matrix
* add recognition rate evaluation
* make it compile for PointXYZ as well
* compute visible model cloud on multiple views
* merge hv_go3d into ghv (not ready yet)
  optimize visible model cloud computation in verification
* add depth image computation in zBuffering class
  (remove XYZRGBA template instantition)
* split code into more functions, add omp sections again, and some minor beauty
* add replace moves again by checking pairwise intersection matrix
* enhance pairwise intersection computation by fixing smoothing, speeding up computation and adding erosion
* remove some more obsolete code
* use new verification cost function and remove obsolete code pieces
* add smoothing function to zbuffering (does not work properly though)
* add function to remove column or row from eigen matrix
* fix compiler error in change detection module
* implement pairwise intersection computation in verification algorithm
* add rendering function in zbuffering (explicit)
* use local point color to compare color
* delete obsolet member variables
* delete count active hypotheses function in verificitation (as it is not used anyway)
* make update function use member variables instead of having to pass them as an argument
* do not use weights for outliers - just ratio of number of outliers compared to visible points
* clip noise model based radius for inliers search
* rename variable and do label check earlier to avoid redundant processing
* fix seg fault when not using icp for pose refinement
* reset camera view point in object recognizer to avoid messing up visualization
* clip max viewing angle in noise model to 80 degrees to avoid huge noise terms (was 85)
* use noise model for model explained points
* fix ignore color even if exists check
* fix wrong use of row and column counter in self zbuffering module
* do incremental smooth clustering via noise model (not finished yet)
* make visualize go cues a switch parameter
* add a static function to query noise level for a single point
* temp commit
* Integration of change detection into recognition module
* Annotation of changes in GT data
* Change detection module added
* Compilation fix: duplicated pragma
* add merge for multiview
* normalize optimization variables
* fix multipipeline merging of hypotheses when disabled. Also skip merging of ident hypothesis
* fixed self occlusion reasoning
* add pose refinement
  fix noise model based cloud integration for just one input cloud as well as for no indices given
* fixed points on plane side variable in ghv
* working again
* explained and unexplained points seem okay
* fix merge conflict
* fix merge conflict
* Merge pull request #63 from taketwo/remove-x86
  Remove all mentions of x86 and x86_64 in CMake scripts
* Remove all mentions of x86 and x86_64 in CMake scripts
* use object indices also for unfiltered registered model cloud and only save filtered input clouds if debug option is set
  added quick fixed to handle some range check exceptions
  needs proper handling soon
* 1.3.3
* 1.3.2
* Merge remote-tracking branch 'upstream/master'
* add missing Xxf86vm lib
* Merge remote-tracking branch 'remotes/upstream/recognition_update'
* Contributors: Georg, Georg Halmetschlager-Funek, Johann Prankl, Markus Bajones, Markus Suchi, Martin Velas, Michael Zillich, Sergey Alexandrov, Simon Schreiberhuber, Thomas Faeulhammer, Thomas Fäulhammer

  Integrate attention based segmentation Now works on Ubuntu 14.04 and 16.04
  @michael-zillich-1 @msuchi Can I get some feedback before I merge this?
* Adaptions for v4r source code for compiling under Ubuntu 16.04.
  + changed include from "cvmath" to <cvmath>
  + changed namespace of isnan to std::isnan
  this is tested for opencv 2.4 and pcl 1.7.2 which have to be set when launchi9ng cmake:
  cmake -DPCL_DIR=<path to pcl> -DOpenCV_DIR=<oath to opencv 2.4>
* Merged origin/attention_segmentation into master
  Update contributing.md
  Added usage of  setup.sh to CONTRIBUTING.md
* Update CONTRIBUTING.md
* Added dependency installation "how to" to Contributing.md
* Merged branch master into master
  Ubuntu16.04+opencv3
  @ghalmets
* pass 2 parameters to setup. ubuntu and ros codename
* escape variables
* update setup.sh and gitlab-ci.yml to be more generic
* gitlab's lint checker says it is OK now. Let's see.
* fix gitlab syntax after strange merge issue
* Merge remote-tracking branch 'refs/remotes/upstream/master'
  Conflicts:
  .gitlab-ci.yml
  Changed camera matrix input and output of the pointcloud generation class. Now i…
  Adding Setup.sh
  Added Setup.sh to v4r for a more convenient dependency installation.
  .gitlab-ci.yml was edited to use setup.sh to keep the script in the CI loop.
  Workflow:
  `git clone git@my-awesome-v4r-repo`
  `cd v4r`
  `./setup.sh`
  `mkdir build && cd build`
  `cmake ..`
  `make -j8`
* Update Readme.md
* Update Readme.md to rgit and added usage of ./setup.sh
* Changed camera matrix input and output of the pointcloud generation class. Now it is not transposed, or does not has to be transposed anymore.
* Update .gitlab-ci.yml
* Added Setup.sh
* Add script for first build
  Installing rosdep and dependencies, building v4r.
* update apps to include all programs for attention based segmentation
* update attention_segmentation module
* Add first sample app for attention based segmentation! Yes it works.
* fix header files
* fix cmake mistake
* add and activate opennurbs and on_nurbs
* shifted around attention\_* files
* Merged branch master into master
* fix cmake file
* add opennurbs as build option for V4R
* reflect change of opennurbs directory
* add cmake find file for openNurbs
* moved opennurbs to 3rdparty
* delete autosave file
* change dependencies
* bring in all the files for attention segmentation -HACK
* Update examples after eputils merge
* Small changes because of the eputils merge into attention_segmentation
* We no longer need/have a v4r_eputils module
* move eputils into attention_segmentation
* Add examples for attention based segmentation
* Adapt to new v4r structure for attention based segmentation
  mainly namespace changes, V4R_EXPORTS, etc.
* Add missing files for eputils
* Adapt to new v4r structure
  namespaces, V4R_EXPORTS, etc.
* small changes to bring attention_segmentation into the new v4r structure
* small changes to bring eputils into the new v4r structure
* Initial copy of attention_segmentation from v4r svn
* Initial copy of eputils from v4r svn
  v4r now compiles with OpenCV 2.x and 3.1 on Ubuntu 14.04 and 16.04
* v4r now compiles with OpenCV 2.x and 3.1 on Ubuntu 14.04 and 16.04
  temporal filter (tracks pose and integrates several rgb-d frames)
  incl. bug fix: default param in common/occlusion_reasoning.cpp
  Update of Contribution.md with results from Structure Workshop.
  I have merged the minutes of V4R structure workshop into the Contribution.md
* Update CONTRIBUTING.md
* Fixed some Typos
* Update CONTRIBUTING.md
* temporal filter (tracks pose and integrates several rgb-d frames)
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md Update description v4r exports
* Update CONTRIBUTING.md minor changes
* Update CONTRIBUTING.md: Formatting
* Update CONTRIBUTING.md: added sections: "Structure", "Documentation", and "How to Build V4R?".
* Manually set PCL_APPS_LIBRARY
* Merged branch ubuntu16.04+opencv3 into ubuntu16.04+opencv3
* I think this is it.
* say yes to apt-get. all the time
* No debug symbols installed
* build ceres from source
* We can force the dpkg installation
* handle install with apt-get force
* Next Ubuntu hack
* fix stupid Ubuntu typo
* Introduce hack because Ubuntu
* rosdep really needs sudo. install it.
* No sudo in xenial image
* No sudo in xenial image
* fix ubuntu version. and again.
* fix ubuntu version
* Also build on Ubuntu 16.04
* fix: no default values
* I think this is it.
* say yes to apt-get. all the time
* No debug symbols installed
* build ceres from source
* We can force the dpkg installation
* handle install with apt-get force
* Next Ubuntu hack
* fix stupid Ubuntu typo
* Introduce hack because Ubuntu
* rosdep really needs sudo. install it.
* No sudo in xenial image
* No sudo in xenial image
* fix ubuntu version. and again.
* fix ubuntu version
* Also build on Ubuntu 16.04
  Update AUTHORS
  fix libsvm dependency in package.xml
* fix libsvm dependency in package.xml
* Update AUTHORS
  Update hypotheses_verification.cpp (wtf? I just committed this change)
* Update hypotheses_verification.cpp
* Update hypotheses_verification.cpp (wtf? I just committed this change)
  Master
  add missing boost dependency
* add missing boost dependency
* Go back to use standard ubuntu trusty docker image
  This is easier to support in the future.
* Merged branch master into master
* Merged branch master into master
  Fix vector type
* Update hypotheses_verification.cpp
* Update hypotheses_verification.cpp
* fix vector type
* fix vector type for new histogram interface
* Update Readme.md
  Use docker image that has those dependencies already installed
* Use docker image that has those dependencies already installed
  Update .gitlab-ci.yml
* Update .gitlab-ci.yml
* Merged branch master into master
* Update .gitlab-ci.yml
* Update .gitlab-ci.yml
  Some fixes
* Update .gitlab-ci.yml
* Merged branch master into master
* Update .gitlab-ci.yml
  yet again. ;-)
* Update .gitlab-ci.yml
* put some header definitions into cpp files and remove .hpp files
* Update .gitlab-ci.yml
* put miscellaneous functions into more specific files
* Update v4r_style_guide.md
* fix merge conflict
* Merged branch master into master
* added: only small inline functions
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Update CONTRIBUTING.md
* Merged branch master into master
* added: keep pull requests short
* fixed typo
* fixed typo
* clean up .gitlab-ci.yml
* add dependencies description
* Update .gitlab-ci.yml
* add contributing and style_format doc files
* Update package.xml
  test if this compiles now
* Update .gitlab-ci.yml
  Continue on rosdep errors. Arrrrrr
* Update .gitlab-ci.yml
  fix syntax
* Update .gitlab-ci.yml
  specify the ROS version (needed to resolve packages from package.xml)
* Update .gitlab-ci.yml
  We need wget as well.
* Update .gitlab-ci.yml
* Update .gitlab-ci.yml
  Seems like we need cmake after all
* Add .gitlab-ci.yml
  First try
* update color transformation and comparison
* use boost::dynamic_bitset instead of vector<bool>, add camera class, put some definitions into header files + some code refactoring
* fix roi when reaching boundary
  Ubuntu 16.04 compatibility
* Merge pull request #67 from strands-project/ubuntu1604_compatibility
  Ubuntu1604 compatibility
* Merge remote-tracking branch 'hannes/master'
* add pcl time header
* Merge remote-tracking branch 'v4r-master/master'
* add bounding box function
* separate definitions from some header files to reduce compile time
* fix some warnings
* Merge pull request #66 from strands-project/sync_from_gitlab
  Sync from gitlab
  New try
* Merge pull request #64 from strands-project/new_try
  [WIP] New try
* add timing for pose refinement
* update citation file
* put pcl_opencv functions from header into implementation file
* some code optimization
* some changes for compiling with Ubuntu 16.04
* some beauty
* add script for obtaining alexNet CNN
* use const
* make destructors virtual for virtual classes
* remove empty file
* remove broken files
* put test data into directory
* fix existing directory warning
* add docs for recognition
  update get_TUW script
* add script for downloading 3dnet test data
* add doc for RTMT
* include scripts to obtain training data from TUW and 3dNet
* include missing mean subtraction in alexnet feature estimation
* update for shape cnn classifier to work
* fix wrong model assembly resolution
* fix compilation errors for eval and app pieces
* fix missing clear of indices when no keypoints are detected
* remove voxelgriddistancetransform method call
* remove default typename in createIndicesFromMask method to allow usage without c++11
* add global hypotheses non-maxima surpression to rejection method
* group hypotheses by segmentation cluster
* add online averaging function
* add hyp stuff (should have been staged earlier)
* remove EDT stuff
* check if all views are trained during initialization (not just if directory exist)
* put boost program options directly into parameter classes, merge ghv with hypotheses verification class
* make separate table class
* minor fixes for save pose into pcd
* update some visualization functions in recognition
* remove sift based alignment in incremental object learning class
* use new segmentation class and provide combined feature, keypoint and segmentation layer
* hopefully fixing Caffe optional dependency
* up pcl version
* fix compilation error caused by addcoordinatesystem if used with PCL < 1.7.2
* add esf classifier again
* fix typo in openmp call
* fix some warnings
* fix bug in optional depending on caffe and rendering
* change default params and do not instantiate harris and iss keypoint extractor on PCL versions < 1.7.2 (as keypoint indices is not available for these versions)
* make recognition library dependency to rendering and Caffe optional
* move some hpps into cpps
* skip recongition rate computation if output file already exists
* add nicer visualization for paper
* add todo comment
* add eval for rec rate over occlusion
* fix crop bug in pcl opencv conversion
* fix min fitness threshold scaling
* flip table plane towards viewpoint and make parameter for min points accessible
* make resolution as an integer in mm
* add coordinate system for visualizing recognition results
* fix bug in color conversion
* change default parameter for svm cross validation
* make smooth segmentation parameter scale with depth
* avoid table plane filtering for initialization in local recognizer
* add parameter options for smooth clustering
* add dense SIFT option (not tested yet and only available for SIFTGPU)
* add smooth clustering and linear thresholding of model fitness threshold (with visibliity)
* use multi-plane segmentation for local recognizer to find *highest* table plane
* fix visualization for recognition rate computation when cloud sensor header is changed
* temporary remove parallel call of recognizer
  QH6205 qhull error (qh_initqhull_start): qh_qh already defined.  Call qh_save_qhull() first
* fix bug in compute recognition rate
* ignore multiview and esf object classifier for now
* make model fitness threshold adaptive to visible ratio (TODO: write a proper function)
* use bigger rendering points in model cues visualization
* fix wrong sigma for AB color components
* remove table plane debug visualization
* rename some recognition init parameters
* reset view to do not mess up visualization in evaluation recognition example
* add option to just compute dominant plane (without clustering)
* fix bug with multiple call to recognize if recognizer is local
* add all the options for initialization
* make local recognizer more modular
* fix bug in knn_sift initialization
* add missing iostream include in classifier.h
* add opencv sift option again (NOTE: Not tested yet)
* remove keypoint rounding stuff in sift
* rewrite local estimator interfaces
* remove redundant files, take into acccount sign ambiguity of eigen vectors for global recognizer
* fix bug with missing normal computation
* migrated feature estimator changes (except eigen matrix). kinda working but only for first test view it seems
* add global recognizer
* add ourcvfh pcl trunk version, fix view all point clouds in folder
* merging svmwrapper, classifier classes, keypoint extractors... still working
* merged many things from rec_optimization_isolated branch (hyp generation still working - verificaiton not)
* add point cloud to eigen matrix conversion with indices
* add ptr for gcg
* remove old comments
* add ptr for gcg
* fix merge conflict
* add vector sort with indices return
* add some histogram functions
* add cielab2rgb transformation
* some code polish in graph based geometric consistency grouping
* avoid some warnings
* add visualization of model keypoints
* fix visualization of correspondences
* remove global requirement for samples to have all modules enabled
  it now only looks for the individual dependency of each sample and compiles just the ones which meet their dependencies
* using parameter class for gcg when gcg is used... small code polish
* addition to previous commit
* fix color conversion compilation error in case of PointXYZ instantiation
* make ghv compile for PointXYZ type instantation as well
* fix error with color retrieval in verification code
* optimize speed
* fix bug in model assembly
* remove parameter output
* speed up verification evaluation by compressing scene fitting matrix
* add recognition rate evaluation
* make it compile for PointXYZ as well
* compute visible model cloud on multiple views
* merge hv_go3d into ghv (not ready yet)
  optimize visible model cloud computation in verification
* add depth image computation in zBuffering class
  (remove XYZRGBA template instantition)
* split code into more functions, add omp sections again, and some minor beauty
* add replace moves again by checking pairwise intersection matrix
* enhance pairwise intersection computation by fixing smoothing, speeding up computation and adding erosion
* remove some more obsolete code
* use new verification cost function and remove obsolete code pieces
* add smoothing function to zbuffering (does not work properly though)
* add function to remove column or row from eigen matrix
* fix compiler error in change detection module
* implement pairwise intersection computation in verification algorithm
* add rendering function in zbuffering (explicit)
* use local point color to compare color
* delete obsolet member variables
* delete count active hypotheses function in verificitation (as it is not used anyway)
* make update function use member variables instead of having to pass them as an argument
* do not use weights for outliers - just ratio of number of outliers compared to visible points
* clip noise model based radius for inliers search
* rename variable and do label check earlier to avoid redundant processing
* fix seg fault when not using icp for pose refinement
* reset camera view point in object recognizer to avoid messing up visualization
* clip max viewing angle in noise model to 80 degrees to avoid huge noise terms (was 85)
* use noise model for model explained points
* fix ignore color even if exists check
* fix wrong use of row and column counter in self zbuffering module
* do incremental smooth clustering via noise model (not finished yet)
* make visualize go cues a switch parameter
* add a static function to query noise level for a single point
* temp commit
* Integration of change detection into recognition module
* Annotation of changes in GT data
* Change detection module added
* Compilation fix: duplicated pragma
* add merge for multiview
* normalize optimization variables
* fix multipipeline merging of hypotheses when disabled. Also skip merging of ident hypothesis
* fixed self occlusion reasoning
* add pose refinement
  fix noise model based cloud integration for just one input cloud as well as for no indices given
* fixed points on plane side variable in ghv
* working again
* explained and unexplained points seem okay
* fix merge conflict
* fix merge conflict
* Merge pull request #63 from taketwo/remove-x86
  Remove all mentions of x86 and x86_64 in CMake scripts
* Remove all mentions of x86 and x86_64 in CMake scripts
* use object indices also for unfiltered registered model cloud and only save filtered input clouds if debug option is set
  added quick fixed to handle some range check exceptions
  needs proper handling soon
* 1.3.3
* 1.3.2
* Merge remote-tracking branch 'upstream/master'
* add missing Xxf86vm lib
* Merge remote-tracking branch 'remotes/upstream/recognition_update'
* Contributors: Georg, Georg Halmetschlager-Funek, Johann Prankl, Markus Bajones, Markus Suchi, Martin Velas, Michael Zillich, Sergey Alexandrov, Simon Schreiberhuber, Thomas Fäulhammer

1.4.3 (2016-02-26)
------------------

1.4.2 (2016-02-26)
------------------
* Merge pull request `#60 <https://github.com/strands-project/v4r/issues/60>`_ from strands-project/strands
  some quick fixes regarding range check exceptions, need proper fix eventually
* added quick fixed to handle some range check exceptions
  needs proper handling soon
* Merge pull request `#59 <https://github.com/strands-project/v4r/issues/59>`_ from strands-project/fix_range_error_when_using_hv_use_histogram_specification
  Update ghv.h
* Update ghv.h
* Contributors: Michael Zillich, Thomas Fäulhammer, mzillich

1.4.1 (2016-02-01)
------------------
* Merge pull request `#58 <https://github.com/strands-project/v4r/issues/58>`_ from strands-project/fix1
  initialize counter variable
* initialize counter variable
* Merge pull request `#57 <https://github.com/strands-project/v4r/issues/57>`_ from strands-project/remove_c+11_from_header
  remove c++11 construct in header file
* remove c++11 construct in header file
* Merge pull request `#56 <https://github.com/strands-project/v4r/issues/56>`_ from strands-project/fix1
  Fix1
* add siftgpu as optional dependency in RTMT
* copy uniform_sampling files from PCL 1.7.2 to make V4R also compile on PCL 1.8
* updated RTMT noise model parameters
* Merge remote-tracking branch 'v4r_root/master'
  Dynamic object learning
  Master
* Contributors: Thomas Fäulhammer

1.4.0 (2016-01-27)
------------------
* Merge pull request `#55 <https://github.com/strands-project/v4r/issues/55>`_ from strands-project/new_recognition_resolved_merge_conflict
  New recognition resolved merge conflict
* Merge remote-tracking branch 'strands/master'
* change default values
* fix noise model based cloud integration
* make opencv sift instantiation conditional on siftgpu presence
* integrate parse console arguments into library
* Merge pull request `#54 <https://github.com/strands-project/v4r/issues/54>`_ from taketwo/speed-up
  Speed-up info collection in NMBasedCloudIntegration
* uses more parallelization
* Merge remote-tracking branch 'sergey_strands/speed-up' into new_recognition
* Speed-up info collection in NMBasedCloudIntegration
  Pre-compute the number of points and resize big_cloud_info\_ only once.
  This achieves > 2x speed-up in Debug mode.
* tmp commit to test siftgpu
* some beauty
* add present of model in view variable for go3d
  change default noise model param
* parallelize add models function in go3d
* some beauty
* normalize all components of LAB colors in range -1 to 1
* put color transform into separate class
* remove a few pointers and add parallel block
  refactor code for merging feature correspondences in multiview recognizer
* fix conditional compilation with -DWITH_SIFTGPU=OFF
* remove hough_3d as it is not used and within PCL (maybe other version though)
* remove accidentally added build folder
* remove template parameters FeatureT and DistT for local recognizer/estimator
  save descriptors as binary text file on disk
* getting rid of some pointers
  move duplicated functions in a common file
* make multipipeline recognizer code parallel
* parallelize correspondence grouping
* make converttoflann create its flann data internally (to make interfacing simpler)
* hopefully solves range_check_error during correspondence grouping
  refactored some code
* add missing ifdef HAVE_SIFTGPU
* fix interface problem in IOL and avoid deprecated interface
* Merge pull request `#52 <https://github.com/strands-project/v4r/issues/52>`_ from strands-project/add_citation_license_file
  add citation, license and authors file
* add citation, license and authors file
* Merge pull request `#51 <https://github.com/strands-project/v4r/issues/51>`_ from strands-project/build-fixes
  Build fixes
* Merge remote-tracking branch 'severin/build-fixes'
  Conflicts:
  samples/examples/object_recognizer_new.cpp
* Merge pull request `#49 <https://github.com/strands-project/v4r/issues/49>`_ from strands-project/fix_siftgpu_problem_in_IOL
  Fix siftgpu problem in iol
* use HAVE_SIFTGPU to check if siftgpu is available on system in object modelling module
* rename dynamic object learning to incremental object learning
* Added missing header 'boost/format.hpp' in a few examples
* [cmake] ObjectGroundTruthAnnotator requires VTK
* [cmake] Ensure v4r compiles without ceres at CMake level
  Note that V4R *does not yet* compile without ceres due to
  modules/reconstruction/include/v4r/reconstruction/impl/ReprojectionError.hpp
  requiring ceres.h
* Properly guards omp.h includes so that the project compile without OpenMP support
* [cmake] Cosmetic in CMakeLists
* [cmake] Use pkg-config to find OpenNI2
  The Debian package for libopenni2 provides a .pc but no
  FindOpenNI2.cmake
* [cmake] FindOpenGL does not return the version
* [cmake] Added support for compiling with Qt5
  Note that CMake option WITH_QT needs to be enabled,
  and *WIT_QT4* needs to be disabled.
* [cmake] Enable WITH_QT by default
* Merge pull request `#44 <https://github.com/strands-project/v4r/issues/44>`_ from strands-project/dynamic_object_learning
  Dynamic object learning
* make compatible to new v4r interfaces
* fix deprecated warning
* remnants from RAL paper
* fixed some bugs
* fix of fix
* fixed bug in evaluation - don't test on same set as object was trained
* write eval recognition for uncontrolled scenes to test on controlled ones
  added visualization of all learnt models
* fix wrong parameter type
* added file to test model coverage
* skip patrol run for which no object has been modelled
* recognition evaluation with respect to coverage for controlled runs
  Conflicts:
  modules/recognition/include/v4r/recognition/impl/local_recognizer.hpp
* taken view file is now correct
* forgot to undo temporary change
* added evaluation tool for recognition performance measure of partial model coverage
* add FindX11 and FindXRandR
* fixed error when training views do not begin with 0
* recognition evaluation for partial model and for offline data more or less ready
* added first evaluation tool to compute recognition performance with respect to percentage of visible model
* added visualize_masked_pcd again
  Conflicts:
  samples/examples/dynamic_object_learning.cpp
  samples/icra16/eval_dol_gt.cpp
  samples/icra16/eval_dynamic_object_learning_with_mask_pertubation.cpp
* add eval
  use boost program options
* adapt code to make rebase compile
* rebase commit
* added noise level evaluation for initial mask ICRA16
* added eval for initial mask evaluation
  added for icra16 singleview
* fixed sv eval when test_dir is not present
* fixed bug in icra sv eval, when csv file has only 2 columns
* eval almost ready
* added icra vis
* separate post-processing and save to disk in object learning
* fixed wrong output file if name of mask is mask.txt only
* removed overhead computation when sift based camera pose estimation is disabled
* fixed ground truth labelling
* fixed color in add text
* just added a const
* removing nan points in initial mask - otherwise seg fault when after erosion not enough points
* included plane merge
  moved logical stuff to common module
  added plane visualization
  added plane properties
* added function to write images to disk for intermediate steps
* make ratio parameter accessible from outside for occluded and object supported points
* sort files before evaluation and output debug info
* added some V4R_EXPORTS in registration module
  removed redundant fast_icp in common module
  added app for 3D reconstruction based on SIFT and MST
  fixed CERES version conflict
  fixed some dependency issues
* up
* add Willow Dataset definition for save_pose_into_pcd sample
* set sensor pose to identity in eval to show right visiualization
* parameters can now also be set in constructor
  initial eval code now in samples (should be moved somewhere else later on)
* moved mask<->indices conversion function into v4r common module
* added object_modelling again
* adapt code to make rebase compile
* rebase commit
* added noise level evaluation for initial mask ICRA16
* added eval for initial mask evaluation
  added for icra16 singleview
* fixed sv eval when test_dir is not present
* fixed bug in icra sv eval, when csv file has only 2 columns
* eval almost ready
* added icra vis
* separate post-processing and save to disk in object learning
* fixed wrong output file if name of mask is mask.txt only
* removed overhead computation when sift based camera pose estimation is disabled
* fixed ground truth labelling
* fixed color in add text
* just added a const
* removing nan points in initial mask - otherwise seg fault when after erosion not enough points
* included plane merge
  moved logical stuff to common module
  added plane visualization
  added plane properties
* added function to write images to disk for intermediate steps
* make ratio parameter accessible from outside for occluded and object supported points
* sort files before evaluation and output debug info
* added some V4R_EXPORTS in registration module
  removed redundant fast_icp in common module
  added app for 3D reconstruction based on SIFT and MST
  fixed CERES version conflict
  fixed some dependency issues
* up
* add Willow Dataset definition for save_pose_into_pcd sample
* set sensor pose to identity in eval to show right visiualization
* parameters can now also be set in constructor
  initial eval code now in samples (should be moved somewhere else later on)
* moved mask<->indices conversion function into v4r common module
* added object_modelling again
* Contributors: Sergey Alexandrov, Séverin Lemaignan, Thomas Fäulhammer

1.3.1 (2016-01-13)
------------------
* Merge pull request `#43 <https://github.com/strands-project/v4r/issues/43>`_ from strands-project/fix_classifier
  Fix classifier
* fix global classifier error when reading from new model database file structure
* build utility tools by default
* Merge remote-tracking branch 'simon/master'
* Merge pull request `#42 <https://github.com/strands-project/v4r/issues/42>`_ from strands-project/remove_glfw3_dependency
  Remove glfw3 dependency
* remove some output messages
* remove glfw3 dependency and use X* libraries only
  fixed some deprecated interfaces warnings
  added some build /run dependency for openni
* Added code for a proper destructor
* cleaned up some code
* Merge remote-tracking branch 'simon/master'
* Merge remote-tracking branch 'simon/change_glfw_to_old'
* removed the need for glfw and changed everything to work with only x11 dependencies
* Merge remote-tracking branch 'origin/master'
* Merge remote-tracking branch 'v4r_root/master'
* Merge remote-tracking branch 'strands/master'
* merged
* use openni instead of openni2
* Merge remote-tracking branch 'v4r_root/recognition_dev'
  Recognition update
  Recognition update
* Contributors: Simon Schreiberhuber, Thomas Fäulhammer

1.3.0 (2016-01-08)
------------------

1.2.0 (2016-01-08)
------------------
* Merge pull request `#40 <https://github.com/strands-project/v4r/issues/40>`_ from strands-project/recognition_dev
  Recognition dev
* use openni instead of openni2
* rename object tracker
* updated object tracker and RTMT saves tracking model correctly
* separated normal computation methods into new file
  using using namespace v4r in samples for (i) brevity, (ii) conformity with ROS wrappers
  changed some deprecated interfaces
  split header files into impl (.hpp) files mainly to avoid c++11 neccessity when including with ROS
* temporary backup commit
* noise model based cloud integration update (also moved to registration module)
  uses properties (1) lateral noise, (2) axial noise, (3) distance in px to depth discontinuity
* backup commit
* first try for new noise modelling
* fixed problem when training views do not start with cloud_000000.pcd
  TODO: re-initialize problem still exists (if training database is altered, flann matrix will be wrong - have to remove *_flann.idx manually right now)
  fixed trigraph warnings
* use absolute value when checking reprojected poitns in ground truth annotation
  added parameters for noise model based integration demo program
* since image2 only takes integer values, we do not need to interpolate (checked by Hannes)
* add zero padding in interpolationfunction to avoid assertion error in Debug mode
  add fix from Hannes
* fix seg fault when dist_coeffs is 2x4 matrix instead of 1x8
* small fix (avoid ourcvfh)
* rewrite noise model based integration so that it uses really equation from Nguyen et al paper.
* tmp commit
* tmp commit
* replaced a few integer and long unsigned integer by size_t to hopefully make it working on 32bit machines
  added visualization functions for hypotheses verification
* some more changes in pcl2opencv
* change pcl2opencv interfaces
* added tools again
* Merge remote-tracking branch 'strands/master' into add_v4r_exports
* added a few more V4R_EXPORTS (visibility attribute) for classes
  added cmake_minimum_required version (cmake 2.8.8 apparently can not handle url hash tags)
* add v4r_export for tomita
* Contributors: Thomas Fäulhammer

1.1.1 (2015-11-23)
------------------
* Merge pull request `#37 <https://github.com/strands-project/v4r/issues/37>`_ from strands-project/add_glm_run_dependency
  add glm also as run dependency
* add glm also as run dependency
* Contributors: Thomas Fäulhammer

1.1.0 (2015-11-20)
------------------
* Merge pull request `#35 <https://github.com/strands-project/v4r/issues/35>`_ from strands-project/recognition_update
  Recognition update
* Merge remote-tracking branch 'v4r_root/recognition_update' into recognition_update1
  Fix glfw3 and undefined references to X*
* add multiple X11 linker libs
* Fix variable names in examples CMakeLists
* Merge remote-tracking branch 'sergey/fix-glfw3' into recognition_update1
* Export 3rdparty include directories into config file
* undo insert HAVE_V4R_RENDERING
* add some x*libraries in package.xml to hopefully solve undefined references
* Merge remote-tracking branch 'sergey/fix-glfw3' into recognition_update1
* added description for go3d parameter
* Properly add GLFW3 third-party library
  Fixes for recognition update
  This fixes a few compilation problems in the current recognition update branch.
* Fix "invalid suffix 'd' on floating constant" error
* Add missing dependency (rendering depends on GLM)
* added glog as dependency (otherwise linking potentially fails)
* updated parameters for sv recognition
* added conversion function from point cloud to fixed sized image
  removed unused parameters in global estimator
  changed namespace of pclopencv to v4r
* computing histogram size by sizeof to make esf estimator compatible with PCL 1.7.1
* remove template parameter Feature from global classifier and make it a vector instead
  added esf object classifier again
* tmp commit
* Merge remote-tracking branch 'simon/recognition_update' into recognition_update1
  Conflicts:
  modules/rendering/src/depthmapRenderer.cpp
* tmp commit (conditional saving of pcd as xyz or xyzrgb) before merging simons update
* Cleaned up the code and sorted out some culprits.
* fixed datatype for colormap
  fixed some warnings
  added program options for radius, subdivision, camera intrinsics,...
* added glGetError queries.
* added rendering + example
  added glew, glfw find package
* updated some more parameter descriptions
* renamed occlusion reasoning into zbuffering and removed second layer namespace
  separated classes into separate files
  renamed boost graph extension into multi-view representation
  fixed shot recognizer (removed indices), parameters are now written to file
* added GLOG dependency
  using boost program option for object recognizer examples and Ground-truth annotator
* use integer key for model assembly (instead of float) - resolution for it is now a parameter
  temporary included visualization for pose refinement
* parameters are now double (instead of float) to be easily accessible from outside via ros getparam
  default parameters change
  updated ground truth annotator and evaluations for recognizer to new framework
* added clear multiview data
* Properly export template instantiations in EDT
* Fix METSlib third-party library
* removed visualization reminiscent in single-view recognizer
* fixed wrong index computation in 3D occupancy grid
  removed siftgpu library from necessary dependency in reconstruction app
* fixed wrong angle difference calculation when clustering object hypotheses [TODO: make parameter accessible from outside]
* (hopefully) fixes crash when no valid recognition model is found
  merging close hypotheses is now possible (generate less hypotheses, refines pose by these larger correspondence set)
* using mask instead of indices in go3d addModels function
  increased default occlusion threshold
  can be compiled with clang again
* fixed multiplane segmentation in unorganized point clouds (TODO: downsample cloud)
  replaced USE_SIFT_GPU definitions with HAVE_SIFTGPU
  v4r_config.h now presents status of HAVE_SIFTGPU
* added pcl version of ClusterNormalsToPlane (works for unorganized point clouds now)
  TODO: fix multiplane segmentation method
* install metslib header files
  fixed go3d
  createVoxelGridAndDistanceTransforms is now called inside generate (for registered views source) TODO: Check influence of resolution parameter!
  added some description and licenses
* temporary commit with GO3D visualization
* fixed wrong transformation of keypoints when using -transfer_feature_matches 1
* added mising tracking dependency
* recognizer:
  - added license
  - removed unused variables
  - moved internally used public methods to protected
* go3d implemented but results not satisfying (parameter not good?)
* pruningGrap in Multiview Object Recognizer is working
  [TODO: Finish Go3D Hypothesis Verification Integration]
* failed try of point cloud rendering with vtk
* when using compute_mst, it crashes after using pruneGraph
* absolute pose computation seems to work
* absolute pose computation seems to work
* added merging of feat correspondences
* tmp commit
* temporary commit (single-view recognizer correspondence grouping seems broken)
* adding parameter classes
  remove redundant variables
  getting rid of singleview_object_recognizer class
  local estimator uses normal estimator from v4r common now
  Reimplementation of multiview recognizer just started (NOT WORKING / COMPILING)
* single view object recognizer almost ready
* tmp commit
  getting rid of redundnant single_view object recognizer class
* correspondences in recognizer are now stored as indexes to original cloud
  this should reduce memory requirement
  New parameter class for Hypotheses Verification methods (different results to before - TODO: double check default parameters!)
* only training dir parameter necessary any more
  improved code readability
* temporary commit (signatures not initialized) otherwise it seems to work
* overall update of training procedure
* recognizer structure sift parameter was named inconsistently
  fixed some warnings
* this includes the changes from gitlab v4r version made by @alexandrox88
  - fixes assimp in tomgine
  - remove ipp
  adds object tracking
  fixes a few warnings
* SOMETHING SEEMS TO BE WRONG WITH THE TRANSFORMS
  namespace update
  polishing multiview recognizer
  add libsvm as system dependency
* merged remove_tomgine
  Remove all mentions of IPP (Intel Performance Primitives)
  Remove all mentions of IPP (Intel Performance Primitives). This remained from OpenCV scripts.
* Remove all mentions of IPP (Intel Performance Primitives)
  Fix Assimp dependency
* Update Assimp finder script
* Add missing AssImp include in tomgine
* Fix a few warnings in tomgine
  Master
  created a tracking module and added the monocular object tracker from RTMT
  Add CMake commands to detect system installation of LibSVM
  The possibility to build LibSVM from source is preserved, but has to be enabled by setting BUILD_LIBSVM option (which is now off by default).
* added monocular camera pose tracker (lk/ keypoint based) from RTMT
* test
* test
* mv test
* just a test file
* Contributors: Johann Prankl, Markus Bajones, Sergey Alexandrov, Thomas Fäulhammer, simon.schreiberhuber@gmx.net

1.0.11 (2015-10-14)
-------------------
* Merge pull request `#34 <https://github.com/strands-project/v4r/issues/34>`_ from strands-project/remove_tomgine
  temporary remove Tomgine and everything related to it (i.e. object cl…
* also comment computeCentroid in single-view object recognizer
* comment computeCentroid to silence error
* temporary remove Tomgine and everything related to it (i.e. object classification)
* Contributors: Thomas Fäulhammer

1.0.10 (2015-09-21)
-------------------
* Merge pull request `#31 <https://github.com/strands-project/v4r/issues/31>`_ from strands-project/namespace_update
  Namespace update
* namespace update
  polishing multiview recognizer
  add libsvm as system dependency
* Merge remote-tracking branch 'sergey/find-system-libsvm' into namespace_update
* Add CMake commands to detect system installation of LibSVM
  The possibility to build LibSVM from source is preserved, but has to be
  enabled by setting BUILD_LIBSVM option (which is now off by default).
* rename multiview_object_recognizer
  silence unused variable warning
  removed unnecessary point cloud copy
  normal method now a parameter
  Master
  Master
* Contributors: Sergey Alexandrov, Thomas Fäulhammer

1.0.9 (2015-09-17)
------------------
* fix Bloom issue with umlauts
* Merge remote-tracking branch 'strands/master'
* Contributors: Thomas Fäulhammer

1.0.8 (2015-09-17)
------------------
* Merge pull request `#28 <https://github.com/strands-project/v4r/issues/28>`_ from strands-project/remove_c++11_flags_and_common_namespace
  remove C++11 flags
* remove C++11 flags
  remove common namespace
  remove duplicated files
  divide samples in examples, evaluation and utility tools (enable examples by default in cmake)
  add Qt Cache files in .gitignore list
* Contributors: Thomas Fäulhammer

1.0.7 (2015-09-16)
------------------
* Merge pull request `#27 <https://github.com/strands-project/v4r/issues/27>`_ from strands-project/new_samples_structure
  New samples structure
* Merge pull request `#26 <https://github.com/strands-project/v4r/issues/26>`_ from strands-project/add-tomgine
  Add tomgine
* new samples structure
* divide samples into examples, tools and evals
* adds ESF classifier using new point cloud rendering based on TomGine (camera pose is not extracted right now)
* Merge pull request `#24 <https://github.com/strands-project/v4r/issues/24>`_ from strands-project/sift_gpu_solution
  Sift gpu solution
* added initial segmentation example
* updated usage output
* added tomgine
* added Random Forest and SVM
* Merge remote-tracking branch 'sergey/add-libsvm' into add-libsvm
* added RandomForest
  fixed some warnings
* Add libsvm 3rd-party library
  Master
* reverted sv recognizer header file because otherwise cg pointer cast caused seg fault
  fixed some warnings
* make SIFT_GPU optional by setting BUILD_SIFTGPU in cmake
* added segmentation dependency to samples
* added binary vector increment
  changed parameter name to avoid confusion in range image computation
* merged
  Master
  this hopefully includes all the changes from LaMoR Summer School + fixes for the Recognizer
* Contributors: Sergey Alexandrov, Thomas Fäulhammer

1.0.6 (2015-09-07)
------------------
* Merge pull request `#23 <https://github.com/strands-project/v4r/issues/23>`_ from strands-project/mergeLAMOR
  Merge lamor
* merged lamor STRANDS
* added default param for printParams in MV recognizer
  other minor changes
* Update Readme.md
* hopefully fixes bug in ourcvfh with different pcl versions
  view_all_point_clouds_in_folder can now also save images to disk
  Master
* catch SIFT FLANN exception when updating model database
* flann idx now configurable
  Master
  Master
* Contributors: Marc Hanheide, Thomas Fäulhammer

1.0.5 (2015-08-30)
------------------

1.0.4 (2015-08-29)
------------------
* Merge pull request `#22 <https://github.com/strands-project/v4r/issues/22>`_ from strands-project/marc-hanheide-patch-1
  disable C++11
* disable C++11
  see https://github.com/strands-project/v4r_ros_wrappers/commit/0f008ac162ef2319d5685054023ce2c6f0c8db55
* disable C++11
  see https://github.com/strands-project/v4r_ros_wrappers/commit/0f008ac162ef2319d5685054023ce2c6f0c8db55
* Contributors: Marc Hanheide

1.0.3 (2015-08-29)
------------------
* Merge pull request `#21 <https://github.com/strands-project/v4r/issues/21>`_ from strands-project/added_install_commands
  added install targets for apps
* added install targets for apps
* Contributors: Marc Hanheide

1.0.2 (2015-08-29)
------------------
* Merge pull request `#20 <https://github.com/strands-project/v4r/issues/20>`_ from strands-project/marc-hanheide-patch-1
  don't include FREAK headers
* don't include FRAK headers
  as this seems to fail in non-free opencv... see https://github.com/strands-project/v4r_ros_wrappers/pull/3
* Contributors: Marc Hanheide, Michael Zillich

1.0.1 (2015-08-28)
------------------
* fixed some compiler warnings
  fixed out of range bug in GHV RGB2CIELAB when RGB color is white (255,255,255)
  fixed typo in parameter for eval sv
* removed comments in sv recognizer,
  save parameter file in sv recognizer eval
* removed linemod and debug build for recognition
* fixed bug in sv_recognizer
* added EDT include path
* added ground truth annotator as app
  removed unused files in recognition
* added sv recognition sample
  fixed missing chop_z behaviour in single view recognizer
* added sample eval for single view object recognizer
* updated ReadMe
* added libglm-dev as build dependency
  Add GLM dependency
* Add GLM dependency
  Master
* added cmake files for openni2
  Master
  Fix undefined reference errors (with Octree and 1.7.1)
* added qt-opengl-dev as dependency
* added openni in package.xml
* linked openni libraries to RTMT
  added octree_impl to hopefully solve pcl conflicts with versions <= 1.7.1
* Hopefully fix undefined reference errors (with Octree)
  Add missing 'template' keyword (causes clang compilation error)
* added RTMT GL libraries again
* Add missing 'template' keyword (causes clang compilation error)
* added binary operations to common
  changed dist calculation for planes available from outside
* fixed QT4 CMake file
  fixed QT4 CMake file
  added RTMT
* added RTMT
  Master
* added possibility to crop image when converting PCD to image
  createDirIfNotExists should now create all directories recursively
  added initial version for pcl segmentation (used in STRANDS in Year1) - not finished
* make parameters double (instead of float) to make it directly accessible via ros getparam function
  Master
* fixed error with Willow Poses
  removed object modelling dependency which is not yet present
* added const specifier for get function
  Master
  Conflicts:
  samples/cpp/save_pose_into_pcd.cpp
* added some V4R_EXPORTS in registration module
  removed redundant fast_icp in common module
  added app for 3D reconstruction based on SIFT and MST
  fixed CERES version conflict
  fixed some dependency issues
* fix of last push
* fix of last push
* added definitions for willow_dataset in save_pose_into_pcd sample
* added mask<->indices converter function
  ground truth annotator now also outputs mask for object in first frame
* added initial version for ground truth labelling tool
* del
* added samples folder
* fixed some ns
* fixes some namespace issues
* added object learning again
* fixed pcl version conflict with vector of eigen
* fixed vector conflict with different PCL versions
* fixed some ns
* changed ns
* fixed wrong macro names for detect CUDA cmake
* added object learning again
* fixes some namespace issues
* added object learning again
* fixed wrong cmake macro name
* added object learning again
* del
  del
  Master
* remnoved second layer namespace "rec_3d_framework"
  added some V4R_EXPORTS
  changed some include paths
  removed redundant faat_3d_rec_framework.h file
* Print OpenCV and Ceres statuses
* Update find Ceres to export variables
* Implement dependency propagation
* Split filesystem_utils into parts
* Remove duplicate find eigen call
* Properly set variables in FindEDT
* Properly set variables in FindOpenCV
* Properly set variables in FindEigen
* SiftGPU fixup
* Boost fixup
* Change SIFTGPU_INCLUDE_DIR -> SIFTGPU_INCLUDE_DIRS
* Update io module
* Find Boost globally
  Master
* added camera tracker - uff, so many changes!
* updated recognition cmakefile to have correct link to opencv
  fixed some shadow warnings
* fixed some warning and added V4R_EXPORTS
  added openmp in cmake
  fixed some warning and added V4R_EXPORTS
  added openmp in cmake
  Build EDT library with -fPIC option
* Build EDT library with -fPIC option
* fixed some warnings
  changed default parameter value of sor
  Master
* added object_modelling cmakelists.txt
* added OpenCV as cmake dependency
  added some V4R_EXPORTS
  re-inserted computeOccludedPoints (why was this not merged?? Check other files!)
  added OpenMP cmake c/cxx flags
* fixed warnings of shadowed variables
  using new v4r namespaces
  Conflicts:
  modules/object_modelling/include/v4r/object_modelling/do_learning.h
  modules/object_modelling/include/v4r/object_modelling/model_view.h
  modules/object_modelling/src/do_learning.cpp
  modules/object_modelling/src/visualization.cpp
* updated EDT include path
* Merge remote-tracking branch 'sergey/cmake_updates'
* Create core module, moved macros.h and version.h here
* All modules now explicitly depend on PCL
* Fix EDT
* added missing segmentation dependency
  added missing segmentation dependency
* adapted to new cmake system
  Master
* Merge pull request `#19 <https://github.com/strands-project/v4r/issues/19>`_ from strands-project/new_cmake
  New cmake
  Conflicts:
  modules/CMakeLists.txt
* Fix 3rd party header handling for the case of no-install usage of V4R
  New cmake
* changed required PCL version to less restrictive 1.7.
  Otherwise, there is a conflict on Jenkins because it only provides package for 1.7.1
* hide recognition module for the time being
* added package.xml again - Jenkins needs it to build the library
  added sergey to maintainer list
* Merge remote-tracking branch 'sergey/master' into new_cmake
  Conflicts:
  modules/recognition/CMakeLists.txt
  modules/registration/CMakeLists.txt
* Fix V4RConfig.cmake for use without installation
* fixed some warnings with redundant typenames and wrong derived signature (& missing) in Recognition
  fixed missing EDT dependency in Registration
  Master
* updated supervoxel clustering method
  added some function docs
  optional parameter for pairwise transform refinement
* filtering smooth clusters works -- without visualization
* smooth clusters work now --- with visualization for debug
* Miscellaneous should not depend on keypoints
* Revert "(Temporarily) move miscellaneous to keypoints because it depends on them"
  This reverts commit 8b4bf90048750c95bae136b9b65dbb890c8c900e.
* Add V4R_EXPORTS here and there
* pcl::copyPointCloud now also accepts binary obj mask
* beautify code - moved from indices to mask
  added parameter filter_planes_only (not working for value false yet)
* (Temporarily) move miscellaneous to keypoints because it depends on them
* Solve undefined reference problem
* Export 3rdparty include directories
* Remove compatibility stuff
* Finalize SiftGPU support
* table filtering working now as expected...
  removed some unnecessary includes
* temporary commit for visualizing table planes supported by object mask
* Another fix for SiftGPU
  This reverts commit 87d034a1a8c8763657ca59ff08f9ec95a5d1c4be, reversing
  changes made to d183d5143b68e70de0e678a3d0659fae2a85a731.
  This reverts commit 87d034a1a8c8763657ca59ff08f9ec95a5d1c4be, reversing
  changes made to d183d5143b68e70de0e678a3d0659fae2a85a731.
* Trying to add SiftGPU
* Fix EDT
* Remove SiftGPU sources
* Fix EDT third-party dependency
* fixed some warnings
  added occlusion reasoning for two clouds (optional tf) which return occlusion mask
  Dynamic object learning
* added parameter for statistical outlier removal (mean=50 and stddevmul=1 didn't work well on asus_box)
  fixed bug in CreateMaskFromVecIndices
  there seems to be still a problem in occlusion reasoning
* Add new build system, migrate common and segmentation modules
* Get rid of legacy build system stuff
  fixed warning of unused variable in SLICO
  fixed visualization issue when called multiple times
* fixed warning of unused variable in SLICO
  fixed visualization issue when called multiple times
* updated region growing such that it does not use points already neglected by plane extractor
  fixed visualizition issue when calling the visualization service more than once
  Master
* added ceres version check
  updated McLMIcp.cpp to use new fixes from aitor
* include devil dependency
* changed to right rosdep key for glew
* added some dependencies
  Master
* removed aitor from maintainer list
* Merge remote-tracking branch 'strands/package_xml'
* added parameter class for noise model based integration
  changed Eigen::Vector4f vector for correct allocation
* indices are now stored in a more generic way
  visualization now also includes noise model
  added Statistical Outlier Removal for initial indices
  added logical operator for binary masks
  TODO: visualization does only work for first service call
* added opencv dependency
* fixed dependencies to the correct rosdep keys
* added a first package.xml
* MST is now a parameter
  plane indices are stored as a vector of a vector now - (otherwise high cost occurred in callgrind profiler)
  updated clear function
* createDirIfNotExist function is now in common
* fixed problem with nan values (recursive absolute pose computation based on spanning tree implementation was not correct)
* minimum spanning tree is working now... there are nan values transferred to nearest neighbor search -> still needs to be fixed!
* bug fix - should be back to STRANDS review demo state
  Master
* fixed some linking problems... fixed bug in setCloudPose (last element was not set to 1)
  made code clang compatible...
* tmp commit
* fixed linking error, updated some namespaces
* tmp commit
* changed some recognition files to use new filesystem namespace
* tmp commit
  Master
* temporary commit of dynamic object learning. not compiling yet!
* deleted remaining temp(~) files
* added keypoint files needed for object learning
* added clustertonormals from keypointTools
* add initial version of keypoints
  Master
* some fixes to merge to master
* Merge remote-tracking branch 'v4r_root/master'
  Conflicts:
  3rdparty/metslib/CMakeLists.txt
  CMakeLists.txt
  cmake/v4rutils.cmake
  cmake/v4rutils.cmake~
  modules/common/CMakeLists.txt
  modules/common/include/v4r/common/noise_model_based_cloud_integration.h
  modules/common/include/v4r/common/noise_models.h
  modules/common/src/noise_model_based_cloud_integration.cpp
  modules/common/src/noise_models.cpp
  modules/recognition/include/v4r/recognition/boost_graph_extension.h
  modules/recognition/include/v4r/recognition/ghv.h
  modules/recognition/include/v4r/recognition/multiview_object_recognizer_service.h
  modules/recognition/src/boost_graph_extension.cpp
  modules/recognition/src/boost_graph_visualization_extension.cpp
  modules/recognition/src/multiview_object_recognizer_service.cpp
  modules/segmentation/CMakeLists.txt
* remove ~
* .
* .
* tmp commit
  Added multiview recognizer. renamed some namespaces.
* Added multiview recognizer. renamed some namespaces.
  Master
* Fixed merge conflict
* Initial commit. For some reason if segmentation app is compiled - there is a linking problem with pcl. Namespaces are a mess!
* initial commit
* upd
  update readme
* update readme
* Add new file
* Init commit
* Contributors: Marc Hanheide, Markus Bajones, Sergey Alexandrov, Thomas Faeulhammer, Thomas Fäulhammer
