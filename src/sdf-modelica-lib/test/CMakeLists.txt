add_executable(sdf_modelica_test sdf_modelica_test.cpp)
target_link_libraries(sdf_modelica_test PRIVATE sdf-modelica-lib)

set(SDF_MODELICA_TEST_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/oneLinkFixedToWorld.urdf
                             ${CMAKE_CURRENT_SOURCE_DIR}/oneLinkRevoluteToWorld.urdf
                             ${CMAKE_CURRENT_SOURCE_DIR}/oneLinkPrismaticToWorld.urdf
                             ${CMAKE_CURRENT_SOURCE_DIR}/twoLinks.urdf
                             ${CMAKE_CURRENT_SOURCE_DIR}/kr16_2.urdf)

foreach(MODEL_FILE ${SDF_MODELICA_TEST_MODELS})
  file(COPY ${MODEL_FILE} DESTINATION ${PROJECT_BINARY_DIR}/bin)
  get_filename_component(MODEL_NAME "${MODEL_FILE}" NAME_WE)
  add_test(NAME sdf_modelica_test_${MODEL_NAME}
           COMMAND sdf_modelica_test ${MODEL_FILE}
           WORKING_DIRECTORY $<TARGET_FILE_DIR:sdf_modelica_test>)
endforeach()

set(SDF_MODELICA_TEST_MODELS ${SDF_MODELICA_TEST_MODELS} PARENT_SCOPE)

add_executable(nlohmann_json_to_kainjow_mustache_test nlohmann_json_to_kainjow_mustache_test.cpp)
target_link_libraries(nlohmann_json_to_kainjow_mustache_test PRIVATE sdf-modelica-lib)
target_include_directories(nlohmann_json_to_kainjow_mustache_test PRIVATE ${PROJECT_BINARY_DIR})
add_test(NAME nlohmann_json_to_kainjow_mustache_test COMMAND nlohmann_json_to_kainjow_mustache_test)

# add_executable(diagram_layout_graphviz_test diagram_layout_graphviz_test.cpp)
# target_link_libraries(diagram_layout_graphviz_test PRIVATE sdf-modelica-lib)
# target_include_directories(diagram_layout_graphviz_test PRIVATE ${PROJECT_BINARY_DIR})
# add_test(NAME diagram_layout_graphviz_test COMMAND diagram_layout_graphviz_test)


