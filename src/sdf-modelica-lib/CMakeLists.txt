# This is the "physical" (i.e. real) name of the library.
# The actual file name of the library built is constructed based on conventions
# of the native platform, such as lib<LIBRARY_TARGET_NAME>.a,
set(LIBRARY_TARGET_NAME sdf-modelica-lib)

# List of CPP (source) library files.
set(${LIBRARY_TARGET_NAME}_SRC
        src/sdf_modelica.cpp
        src/sdf_modelica_diagram_layout.cpp
        src/nlohmann_json_to_kainjow_mustache.cpp
)

# List of public headers file
set(${LIBRARY_TARGET_NAME}_HDR
        include/sdf_modelica/sdf_modelica.h
)

# List of private headers file
set(${LIBRARY_TARGET_NAME}_PRIVATE_HDR
        include/sdf_modelica/nlohmann_json_to_kainjow_mustache.h
        include/sdf_modelica/sdf_modelica_diagram_layout.h)

# TODO(traveraro): Re-enable the graphviz diagram layout generation
# src/sdf_modelica_diagram_layout_graphviz.cpp
# include/sdf_modelica/sdf_modelica_diagram_layout_graphviz.h



add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC ${SDFormat_INCLUDE_DIRS})
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${SDFormat_LIBRARIES})
# TODO(traversaro) : this will not work for the installed library and is not relocatable
set(SDF_MODELICA_TEMPLATE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/template)
configure_file(src/template_path.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/template_path.hpp)
# Include json and inja
target_include_directories(${LIBRARY_TARGET_NAME} PRIVATE ${PROJECT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR})
# Include and link graphviz
target_include_directories(${LIBRARY_TARGET_NAME} PRIVATE ${Graphviz_INCLUDE_DIRS})
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${Graphviz_LIBRARIES})


install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})

if(BUILD_TESTING)
  add_subdirectory(test)
  set(SDF_MODELICA_TEST_MODELS ${SDF_MODELICA_TEST_MODELS} PARENT_SCOPE)
endif()

