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

#ifndef SDF_MODELICA_DIAGRAM_LAYOUT_GRAPHVIZ_H
#define SDF_MODELICA_DIAGRAM_LAYOUT_GRAPHVIZ_H

#include <string>

#include <ignition/math/graph/Graph.hh>
#include <nlohmann/json.hpp>

#include <sdf_modelica/sdf_modelica_diagram_layout.h>


namespace sdf_modelica
{


/**
 * Add diagram layout annotations using graphviz.
 *
 * @param[in]      graph    graph of Modelica-component connections
 * @param[in,out] modelData data used the for the mustache template generation
 * @return        true if all went well, false otherwise
 */
bool add_diagram_layout_graphviz(const sdf_modelica::DiagramGraph& modelGraph,
                                 nlohmann::json& modelData);

}

#endif
