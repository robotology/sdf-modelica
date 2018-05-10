#.rst:
# FMIUtils
# ----------
#
# A set of macros for creating and manipulating FMUs.
#

# Copyright: (C) 2018 Silvio Traversaro <silvio@traversaro.it>
# Authors: Silvio Traversaro <silvio@traversaro.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

################################################################################
#.rst:
# .. command:: omc_compile_mo()
#
# Create a FMU from a .mo Modelica model using the OpenModelica Compiler
#
#   omc_compile_mo_to_fmu(INPUT_MO <modelica model file>
#                         MODEL_NAME <model name>
#                         OUTPUT_DIRECTORY <output directory>)
#
# This macro converts a .mo Modelica model to a FMU using the OpenModelica Compiler
macro(OMC_COMPILE_MO_TO_FMU)
  set(_options "")
  set(_oneValueArgs INPUT_MO MODEL_NAME OUTPUT_DIRECTORY)
  set(_multiValueArgs "")
  cmake_parse_arguments(_OCM "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN} )

  if(NOT DEFINED _OCM_INPUT_MO)
    message(SEND_ERROR "Missing INPUT_MO option.")
    return()
  endif()

  if(NOT DEFINED _OCM_MODEL_NAME)
    message(SEND_ERROR "Missing MODEL_NAME option.")
    return()
  endif()

  if(NOT DEFINED _OCM_OUTPUT_DIRECTORY)
    message(SEND_ERROR "Missing OUTPUT_DIRECTORY option.")
    return()
  endif()

  # Find the openmodelica compiler
  find_program(OMC_COMPILER NAMES omc)
  if(NOT OMC_COMPILER)
    message(SEND_ERROR "omc program not found by find_program")
    return()
  endif()

  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ocm_compile_fmu_${_OCM_MODEL_NAME}.mos"
"/**
 * This file was generated by the OMC_COMPILE_MO CMake command.
 */
loadModel(Modelica);
loadFile(\"${_OCM_INPUT_MO}\");
buildModelFMU(${_OCM_MODEL_NAME}, platforms = {\"static\"});
")

  if(NOT TARGET compile-fmus)
    add_custom_target(compile-fmus)
  endif()
  add_custom_command(TARGET compile-fmus
                     COMMAND ${OMC_COMPILER} ${CMAKE_CURRENT_BINARY_DIR}/ocm_compile_fmu_${_OCM_MODEL_NAME}.mos
                     COMMENT "Generating FMU for model ${_OCM_MODEL_NAME}"
                     WORKING_DIRECTORY  ${_OCM_OUTPUT_DIRECTORY})

endmacro()
