# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or of the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

foreach(MODEL_FILE ${SDF_MODELICA_TEST_MODELS})
  get_filename_component(MODEL_NAME "${MODEL_FILE}" NAME_WE)
  # Generate test scripts
  configure_file(mo2fmu.mos.in ${CMAKE_CURRENT_BINARY_DIR}/mo2fmu_${MODEL_NAME}.mos @ONLY)
  configure_file(sdf2fmu.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/sdf2fmu_${MODEL_NAME}.cmake @ONLY)
  add_test(NAME openmodelica_sdf2fmu_${MODEL_NAME}
           COMMAND ${CMAKE_COMMAND}
                  -DSDF2MO=$<TARGET_FILE:sdf2modelica>
                  -P ${CMAKE_CURRENT_BINARY_DIR}/sdf2fmu_${MODEL_NAME}.cmake)
endforeach()

