/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v3 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or of the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <sdf_modelica/sdf_modelica_diagram_layout_graphviz.h>

void dummyAssert(bool condition)
{
    if (!condition)
    {
        exit(EXIT_FAILURE);
    }
}

void testOneLinkLayout()
{
    nlohmann::json modelData;
    nlohmann::json links = nlohmann::json::array();
    sdf_modelica::DiagramGraph modelGraph;
    std::map<std::string, ignition::math::graph::VertexId> modelicaComponent2ignGraphId;
    modelicaComponent2ignGraphId["world"] = modelGraph.AddVertex("world", sdf_modelica::ModelicaGraphComponentInfo("world")).Id();

    // Add links
    nlohmann::json link;
    std::string linkName = link["name"] = "dummyLink";
    links.push_back(link);
    modelicaComponent2ignGraphId[linkName] = modelGraph.AddVertex(linkName, sdf_modelica::ModelicaGraphComponentInfo(linkName)).Id();
    modelData["links"] = links;

    // Add joints
    std::string jointName = "dummyJoint";
    std::string parentLink = "world";
    std::string childLink = "dummyLink";
    modelicaComponent2ignGraphId[jointName] = modelGraph.AddVertex(jointName, jointName).Id();
    modelGraph.AddEdge(ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[jointName+"_fixedRotation"], modelicaComponent2ignGraphId[jointName]),
                       sdf_modelica::ModelicaGraphConnectionInfo(sdf_modelica::EAST, sdf_modelica::WEST));
    modelGraph.AddEdge(ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[jointName], modelicaComponent2ignGraphId[childLink]),
                       sdf_modelica::ModelicaGraphConnectionInfo(sdf_modelica::EAST, sdf_modelica::EAST));

    // Generate layout for the graph
    bool ok = add_diagram_layout_graphviz(modelGraph, modelData);
    dummyAssert(ok);

    return;
}


int main(int argc, char** argv)
{
    testOneLinkLayout();
}
