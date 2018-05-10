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

#ifndef SDF_MODELICA_DIAGRAM_LAYOUT_H
#define SDF_MODELICA_DIAGRAM_LAYOUT_H

#include <string>

#include <ignition/math/graph/Graph.hh>
#include <nlohmann/json.hpp>

namespace sdf_modelica
{

enum ConnectionSide
{
    WEST,
    EAST,
    NORTH
};

/**
 * Data for each component in the Modelica diagram.
 */
struct ModelicaGraphComponentInfo
{
    std::string name;
    bool fixedLocation;
    int fixedLocationModelicaUnitsX;
    int fixedLocationModelicaUnitsY;
    ModelicaGraphComponentInfo(): name(""), fixedLocation(false),
                                  fixedLocationModelicaUnitsX(0), fixedLocationModelicaUnitsY(0) {};
    ModelicaGraphComponentInfo(std::string arg_name, bool a_fixedLocation=false,
                                int a_fixedLocationModelicaUnitsX=0, int a_fixedLocationModelicaUnitsY=0):
      name(arg_name), fixedLocation(a_fixedLocation),
      fixedLocationModelicaUnitsX(a_fixedLocationModelicaUnitsX), fixedLocationModelicaUnitsY(a_fixedLocationModelicaUnitsY) {};
};

/**
 * Data for each connection in the Modelica diagram.
 */
struct ModelicaGraphConnectionInfo
{
    ConnectionSide firstSide;
    ConnectionSide secondSide;
    ModelicaGraphConnectionInfo(): firstSide(EAST), secondSide(WEST) {};
    ModelicaGraphConnectionInfo(ConnectionSide first, ConnectionSide second):
      firstSide(first), secondSide(second) {};
};

using DiagramGraph = ignition::math::graph::DirectedGraph<ModelicaGraphComponentInfo, ModelicaGraphConnectionInfo>;


/**
 * Add diagram layout annotations using a simple hand-written model.
 *
 * This works well only for chain-like robots.
 *
 * @param[in]      graph    graph of Modelica-component connections
 * @param[in,out] modelData data used the for the mustache template generation
 * @return        true if all went well, false otherwise
 */
bool add_diagram_layout_handtuned(const DiagramGraph& modelGraph,
                                  nlohmann::json& modelData);


/**
 * Add empty diagram layout annotations.
 *
 * This is only useful to make sure that all the annotations used by the mustache
 * template are actually inserted in the model, but with an empty value.
 *
 * @param[in]      graph    graph of Modelica-component connections
 * @param[in,out] modelData data used the for the mustache template generation
 * @return        true if all went well, false otherwise
 */
bool add_diagram_layout_dummy(const DiagramGraph& modelGraph,
                              nlohmann::json& modelData);

/**
 * Add annotation for the icon rappresentation of the model.
 *
 * Furthermore, add to the graph the flange components and the flange joint connections,
 * for subsequent use in the diagram layout.
 *
 * @param[in,out]      graph    graph of Modelica-component connections
 * @param[in,out] modelData data used the for the mustache template generation
 * @param[in,out] modelicaComponent2ignGraphId map between modelica component names and ign graph VertexId
 * @return        true if all went well, false otherwise
 */
bool add_icon_layout(DiagramGraph& modelGraph,
                     nlohmann::json& modelData,
                     std::map<std::string, ignition::math::graph::VertexId>& modelicaComponent2ignGraphId);
}

#endif
