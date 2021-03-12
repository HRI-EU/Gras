################################################################################
#
#  Copyright (c) 2020, Honda Research Institute Europe GmbH.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################


################################################################################
#
# Generate package config files so that the build tree can be referenced by
# other projects.
#
################################################################################

# Settings
SET(PACKAGE_NAME Gras)
IF(WIN32)
  SET(EXPORT_INSTALL_DEST ${PACKAGE_NAME}/CMake)
ELSE() # Unix etc
  SET(EXPORT_INSTALL_DEST share/cmake/${PACKAGE_NAME})
ENDIF()

# Write version file
INCLUDE(CMakePackageConfigHelpers)
WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
  VERSION 2.0
  COMPATIBILITY AnyNewerVersion
)

# Write targets file
EXPORT(TARGETS ${GRAS_EXPORT_LIBRARIES}
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Targets.cmake"
#  NAMESPACE Gras:: # Not really needed, since the targets are prefixed with Gras anyways
)
# TODO Maybe write to separate target files and use as components?

# Write config file
set(CONFIG_INSTALL_DIR "config")
CONFIGURE_PACKAGE_CONFIG_FILE(
  "cmake/GrasConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake"
  # Hack to ensure the package root is set to the current directory. 
  # Using a custom INSTALL_PREFIX would be better, but that option doesn't
  # exist in CMake 2.8
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}
  PATH_VARS CONFIG_INSTALL_DIR
)

# Add to user package registry if desired
OPTION(WRITE_PACKAGE_REGISTRY "Add build tree to user package registry" OFF)
IF(${WRITE_PACKAGE_REGISTRY})
  EXPORT(PACKAGE ${PACKAGE_NAME})
ENDIF()

################################################################################
#
# Installation of exports
#
################################################################################

# Install exports file
INSTALL(
  EXPORT GrasExport 
  FILE ${PACKAGE_NAME}Targets.cmake
  DESTINATION ${EXPORT_INSTALL_DEST}
)

# Write separate install config to ensure the config dir path is set properly
CONFIGURE_PACKAGE_CONFIG_FILE(
  "cmake/GrasConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/install_files/${PACKAGE_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/${EXPORT_INSTALL_DEST}"
  PATH_VARS CONFIG_INSTALL_DIR
)
# Install configs and dependency finders
INSTALL(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/install_files/${PACKAGE_NAME}Config.cmake"
  DESTINATION ${EXPORT_INSTALL_DEST}
)
