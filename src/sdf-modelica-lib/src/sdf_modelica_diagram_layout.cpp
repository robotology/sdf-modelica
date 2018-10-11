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

#include <sdf_modelica/sdf_modelica_diagram_layout.h>

namespace sdf_modelica {

bool add_icon_layout(
    DiagramGraph& modelGraph,
    nlohmann::json& modelData,
    std::map<std::string, ignition::math::graph::VertexId>& modelicaComponent2ignGraphId)
{
    nlohmann::json& joints = modelData["joints"];
    int dofIndex = 0;
    std::stringstream ssIconFlangeLabels;
    for (nlohmann::json::iterator it = joints.begin(); it != joints.end(); ++it) {
        nlohmann::json& joint = *it;

        bool isTypeFixed = (joint.find("isType_fixed") != joint.end());
        if (!isTypeFixed) {
            std::stringstream ss;
            std::string jointName = joint["name"];
            int modelicaFlangeX = -210;
            int modelicaFlangeY = -170 + 40 * dofIndex;
            ss << "Placement(transformation(origin = {" << modelicaFlangeX << ", "
               << modelicaFlangeY << "}, extent={{-10, -10}, {10, 10}}))";
            joint["flangeGraphicsAnnotation"] = ss.str();
            ssIconFlangeLabels << ",";
            ssIconFlangeLabels << "Text(extent={ {-200, " << -150 + 40 * dofIndex << "},{-140, "
                               << -190 + 40 * dofIndex << " } }, textString=\"" << jointName
                               << "\", lineColor={0,0,255})";
            std::string componentName = "axis_" + jointName;
            modelicaComponent2ignGraphId[componentName]
                = modelGraph
                      .AddVertex(componentName,
                                 ModelicaGraphComponentInfo(
                                     componentName, true, modelicaFlangeX, modelicaFlangeY))
                      .Id();

            modelGraph.AddEdge(
                ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[componentName],
                                                  modelicaComponent2ignGraphId[jointName]),
                ModelicaGraphConnectionInfo(ConnectionSide::EAST, ConnectionSide::NORTH));

            dofIndex++;
        }
    }
    ssIconFlangeLabels << "}";
    modelData["iconFlangeLabels"] = ssIconFlangeLabels.str();
    return true;
}

bool add_diagram_layout_handtuned(const DiagramGraph& modelGraph, nlohmann::json& modelData)
{
    std::map<std::string, int> link2index;
    // Graphics annotation for the Modelica.Mechanics.MultiBody.World element included in the world
    link2index["world"] = -1;
    std::stringstream ssWorld;
    ssWorld << "Placement(visible = true, transformation(origin = { " << -30 << ", "
            << -70 + 40 * (-1) << "}, extent = {{-10, -10}, {10, 10}}, rotation = 180))";
    modelData["worldGraphicsAnnotation"] = ssWorld.str();

    // Set all the annotations to null
    nlohmann::json& links = modelData["links"];
    int linkIndex = 0;
    for (nlohmann::json::iterator it = links.begin(); it != links.end(); ++it) {
        nlohmann::json& link = *it;

        link2index[link["name"]] = linkIndex;
        std::stringstream ss;
        ss << "Placement(visible = true, transformation(origin = { " << -30 << ", "
           << -70 + 40 * linkIndex << "}, extent = {{-10, -10}, {10, 10}}, rotation = 180))";
        link["graphicsAnnotation"] = ss.str();
        linkIndex++;
    }

    nlohmann::json& joints = modelData["joints"];
    int jointIndex = 0;
    for (nlohmann::json::iterator it = joints.begin(); it != joints.end(); ++it) {
        nlohmann::json& joint = *it;

        std::string parentLink = joint["loparentLink"];
        std::string childLink = joint["childLink"];

        std::stringstream ss;
        ss << "Placement(visible = true, transformation(origin = { " << 10 << ", "
           << -70 + 40 * jointIndex << "}, extent = {{-10, -10}, {10, 10}}, rotation = 0))";
        joint["fixedTranformGraphicsAnnotation"] = ss.str();
        ss.str("");
        ss << "Placement(visible = true, transformation(origin = { " << 50 << ", "
           << -70 + 40 * jointIndex << "}, extent = {{-10, -10}, {10, 10}}, rotation = 0))";
        joint["jointGraphicsAnnotation"] = ss.str();
        ss.str("");
        ss << "Line(points = {{-20, " << -70 + 40 * link2index[parentLink] << "}, {0, "
           << -70 + 40 * jointIndex << "}}, color = {95, 95, 95})";
        joint["parent2fixedRotationGraphicsAnnotation"] = ss.str();
        ss.str("");
        ss << "Line(points = {{20, " << -70 + 40 * jointIndex << "}, {40, " << -70 + 40 * jointIndex
           << "}}, color = {95, 95, 95})";
        joint["fixedRotation2jointGraphicsAnnotation"] = ss.str();
        ss.str("");
        ss << "Line(points = {{60, " << -70 + 40 * jointIndex << "}, {-20, "
           << -70 + 40 * link2index[childLink] << " }}, color = {95, 95, 95})";
        joint["joint2childGraphicsAnnotation"] = ss.str();
        ss.str("");
        ss << "Line(points = {{20, " << -70 + 40 * jointIndex << "}, {-20, "
           << -70 + 40 * link2index[childLink] << " }}, color = {95, 95, 95})";
        joint["fixedRotation2childGraphicsAnnotation"] = ss.str();

        jointIndex++;
    }

    return true;
}

bool add_diagram_layout_dummy(const DiagramGraph& modelGraph, nlohmann::json& modelData)
{
    // Graphics annotation for the Modelica.Mechanics.MultiBody.World element included in the world
    modelData["worldGraphicsAnnotation"] = "";

    // Set all the annotations to null
    nlohmann::json& links = modelData["links"];

    for (nlohmann::json::iterator it = links.begin(); it != links.end(); ++it) {
        nlohmann::json& link = *it;
        link["graphicsAnnotation"] = "";
    }

    nlohmann::json& joints = modelData["joints"];

    for (nlohmann::json::iterator it = joints.begin(); it != joints.end(); ++it) {
        nlohmann::json& joint = *it;
        joint["fixedTranformGraphicsAnnotation"] = "";
        joint["jointGraphicsAnnotation"] = "";
        joint["parent2fixedRotationGraphicsAnnotation"] = "";
        joint["fixedRotation2jointGraphicsAnnotation"] = "";
        joint["joint2childGraphicsAnnotation"] = "";
        joint["fixedRotation2childGraphicsAnnotation"] = "";
    }

    return true;
}

} // namespace sdf_modelica
