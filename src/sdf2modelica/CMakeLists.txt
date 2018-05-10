# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or of the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

set(EXE_TARGET_NAME sdf2modelica)

add_executable(${EXE_TARGET_NAME} sdf2modelica.cpp)

target_link_libraries(${EXE_TARGET_NAME} sdf-modelica-lib)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
