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

#include <graphviz/cgraph.h>
#include <graphviz/gvc.h>

#include <iostream>

namespace sdf_modelica
{

// Ratio between graphviz inches and modelica units
const double graphvizInches2modelica = 20.0;

// Ratio between graphviz dots and modelica units
const double graphvizDots2modelica = graphvizInches2modelica/72.0;

// Center of modelica diagram in dots
const double modelicaCenter = 50.0;

/**
 * Convert a graphviz point (in dots) to a modelica point.
 */
std::string toModelica(const pointf gv_pos)
{
    std::stringstream ss;
    ss << "{ " << std::ceil(graphvizDots2modelica*gv_pos.x-modelicaCenter)
       << ", " << std::ceil(graphvizDots2modelica*gv_pos.y-modelicaCenter)  << "}";
    return ss.str();
}

struct ModelicaPoint
{
    int x;
    int y;
};

/**
 * Convert a modelica point in a graphviz point (in inches).
 */
float toGraphvizInches(const int point)
{
    return (point + modelicaCenter)/graphvizInches2modelica;
}

/**
 * Convert a modelica point in a graphviz point (in inches).
 */
pointf toGraphvizInches(const ModelicaPoint& point)
{
    pointf ret;
    ret.x = toGraphvizInches(point.x);
    ret.y = toGraphvizInches(point.y);
    return ret;
}

/**
 * Convert a graphviz spline to a modelica polyline
 * TODO(traversaro): check
 */
std::string toModelicaLine(const splines* spl)
{
    std::stringstream ss;
    ss << "{";
    if((spl->list != 0))
    {
        bezier bez = spl->list[0];

        //If there is a starting point, add it to the line
        /* TODO(traversaro): check this code
        if(bez.sflag == 1)
        {
            ss << toModelica(bez.sp) << ", ";
        }*/

        //Loop over the curve points
        for (int i=0; i<bez.size; i++)
        {
            ss << toModelica(bez.list[i]);

            // If it is the last point of the spline and there is not ending point,
            // do not add the last comma
            if (!(i == bez.size-1))
            {
                ss << ", ";
            }
        }

        //If there is an ending point, draw a line to it
        /* TODO(traversaro): check this code
        if(bez.eflag == 1)
        {
            ss << toModelica(bez.sp);
        }*/
        ss << "}";
    }
    return ss.str();
}

ignition::math::graph::EdgeId edgeConnectingTwoVertices(const DiagramGraph& modelGraph,
                                                        const ignition::math::graph::VertexId parentId,
                                                        const ignition::math::graph::VertexId childId)
{
    // Check all outboarding connections
    auto childEdges = modelGraph.IncidentsFrom(parentId);
    for(auto& edge: childEdges)
    {
        if (edge.second.get().Tail() == childId)
        {
            return edge.first;
        }

        if (edge.second.get().Head() == childId)
        {
            return edge.first;
        }
    }

    return ignition::math::graph::kNullId;
}

Agedge_t* gvEdgeConnectingTwoComponents(std::map<ignition::math::graph::EdgeId, Agedge_t *>& id2edge,
                                        std::map<std::string, ignition::math::graph::VertexId>& modelicaComponentName2id,
                                        const DiagramGraph& modelGraph,
                                        const std::string& parentComponent,
                                        const std::string& childComponent)
{
   ignition::math::graph::EdgeId edgeId = edgeConnectingTwoVertices(modelGraph,
                                          modelicaComponentName2id[parentComponent],
                                          modelicaComponentName2id[childComponent]);
   assert(id2edge[edgeId]);
   return id2edge[edgeId];
}


bool add_diagram_layout_graphviz(const DiagramGraph& modelGraph,
                                 nlohmann::json& modelData)
{
    // Create cgraph graph from the DiagramGraph
    Agraph_t *G;
    char* graphName = "g";
    G = agopen(graphName, Agdirected, NULL);
    agsafeset(G, "splines", "polyline", "");

    std::map<ignition::math::graph::VertexId, Agnode_t *> id2node;
    std::map<std::string, Agnode_t *> modelicaComponentName2node;
    std::map<std::string, ignition::math::graph::VertexId> modelicaComponentName2id;
    auto vertices = modelGraph.Vertices();
    for(auto& vertex: vertices)
    {
        Agnode_t* node = agidnode(G, vertex.first, 1);
        std::string nodeName = vertex.second.get().Name();
        std::vector<char> nodeNameCstr(nodeName.c_str(), nodeName.c_str() + nodeName.size() + 1);
        node = agnode(G, nodeNameCstr.data(), 1);
        if (!node)
        {
            std::cerr << "Error in creading graphviz node for " << vertex.second.get().Name() << ", exiting." << std::endl;
            return false;
        }

        agsafeset(node, "shape", "box", " ");
        agsafeset(node, "fixedsize", "true", "");
        agsafeset(node, "height", "1", "");
        agsafeset(node, "width", "1", "");

        if (vertex.second.get().Data().fixedLocation)
        {
            std::stringstream ss;
            pointf gvPointInches;
            gvPointInches.x = toGraphvizInches(vertex.second.get().Data().fixedLocationModelicaUnitsX);
            gvPointInches.y = toGraphvizInches(vertex.second.get().Data().fixedLocationModelicaUnitsY);
            ss << gvPointInches.x << "," << gvPointInches.y;
            std::string ss_str = ss.str();
            std::vector<char> charvect(ss_str.begin(), ss_str.end());
            charvect.push_back('\0');
            agsafeset(node, "pos", charvect.data(), "");
            agsafeset(node, "pin", "true", "");
        }


        id2node[vertex.first] = node;
        modelicaComponentName2node[vertex.second.get().Name()] = node;
        modelicaComponentName2id[vertex.second.get().Name()] = vertex.first;
    }

    std::map<ignition::math::graph::EdgeId, Agedge_t *> id2edge;
    auto edges = modelGraph.Edges();
    for(auto& edge: edges)
    {
        ignition::math::graph::VertexId parentEdge = edge.second.get().Vertices().first;
        ignition::math::graph::VertexId childEdge = edge.second.get().Vertices().second;
        std::string edgeName = modelGraph.VertexFromId(parentEdge).Name() + "_" + modelGraph.VertexFromId(childEdge).Name();
        std::vector<char> edgeNameCstr(edgeName.c_str(), edgeName.c_str() + edgeName.size() + 1);
        Agedge_t* gvedge = agedge(G,
                                  id2node[parentEdge],
                                  id2node[childEdge],
                                  edgeNameCstr.data(), 1);
        if(edge.second.get().Data().firstSide == EAST)
        {
            agsafeset(gvedge, "tailport", "e", "");
        }
        else if(edge.second.get().Data().firstSide == WEST)
        {
            agsafeset(gvedge, "tailport", "w", "");
        }
        else
        {
            assert(edge.second.get().Data().firstSide == NORTH);
            agsafeset(gvedge, "tailport", "n", "");
        }

        if(edge.second.get().Data().secondSide == EAST)
        {
            agsafeset(gvedge, "headport", "e", "");
        }
        else if(edge.second.get().Data().secondSide == WEST)
        {
            agsafeset(gvedge, "headport", "w", "");
        }
        else
        {
            assert(edge.second.get().Data().secondSide == NORTH);
            agsafeset(gvedge, "headport", "n", "");
        }

        id2edge[edge.first] = gvedge;
    }

    // Compute layout
    GVC_t *gvc;
    gvc = gvContext();
    int status = gvLayout (gvc, G, "fdp");
    if (status != 0)
    {
        std::cerr << "sdf_modelica: error in calling gvLayout "
                  << agerrors() << " " << aglasterr() << std::endl;
        return false;
    }

    // Get node position for the world node
    pointf gv_pos  = ND_coord(modelicaComponentName2node["world"]);

    std::stringstream ssWorld;
        ssWorld << "Placement(visible = true, transformation(origin = " << toModelica(gv_pos)
           << ", extent = {{-10, -10}, {10, 10}}))";
    modelData["worldGraphicsAnnotation"] = ssWorld.str();

    // Get node positions for link nodes
    nlohmann::json& links = modelData["links"];
    for (nlohmann::json::iterator it = links.begin(); it != links.end(); ++it)
    {
        nlohmann::json& link = *it;
        pointf gv_pos  = ND_coord(modelicaComponentName2node[link["name"]]);

        std::stringstream ss;
        ss << "Placement(visible = true, transformation(origin = " << toModelica(gv_pos)
           << ", extent = {{-10, -10}, {10, 10}}, rotation = 180))";
        link["graphicsAnnotation"] = ss.str();
    }

    // Get node and edge positions for joint nodes and edges
    nlohmann::json& joints = modelData["joints"];
    for (nlohmann::json::iterator it = joints.begin(); it != joints.end(); ++it)
    {
        nlohmann::json& joint = *it;

        // Nodes
        std::string jointName = joint["name"];
        pointf gv_pos_fixedRotation = ND_coord(modelicaComponentName2node[jointName+"_fixedRotation"]);

        std::stringstream ss;
        ss << "Placement(visible = true, transformation(origin = " << toModelica(gv_pos_fixedRotation)
           << ", extent = {{-10, -10}, {10, 10}}, rotation = 0))";
        joint["fixedTranformGraphicsAnnotation"] = ss.str();

        bool isTypeFixed = (joint.find("isType_fixed") != joint.end());
        if (!isTypeFixed)
        {
            pointf gv_pos_joint  = ND_coord(modelicaComponentName2node[joint["name"]]);
            ss.str("");
            ss << "Placement(visible = true, transformation(origin = " << toModelica(gv_pos_joint)
               << ", extent = {{-10, -10}, {10, 10}}, rotation = 0))";
            joint["jointGraphicsAnnotation"] = ss.str();
        }

        // Edges
        std::string parentLink = joint["parentLink"];
        std::string childLink  = joint["childLink"];

        Agedge_t *edge = gvEdgeConnectingTwoComponents(id2edge, modelicaComponentName2id, modelGraph,
                                                       parentLink, jointName+"_fixedRotation");
        const splines* spl = ED_spl(edge);
        ss.str("");
        ss << "Line(points = " << toModelicaLine(spl) << ", color = {95, 95, 95})";
        joint["parent2fixedRotationGraphicsAnnotation"] = ss.str();
        if (isTypeFixed)
        {
            edge = gvEdgeConnectingTwoComponents(id2edge, modelicaComponentName2id, modelGraph,
                                                  jointName+"_fixedRotation", childLink);
            spl = ED_spl(edge);
            ss.str("");
            ss << "Line(points = " << toModelicaLine(spl) << ", color = {95, 95, 95})";
            joint["fixedRotation2childGraphicsAnnotation"] = ss.str();
        }
        else
        {
            edge = gvEdgeConnectingTwoComponents(id2edge, modelicaComponentName2id, modelGraph,
                                                  jointName+"_fixedRotation", jointName);
            spl = ED_spl(edge);
            ss.str("");
            ss << "Line(points = " << toModelicaLine(spl) << ", color = {95, 95, 95})";
            joint["fixedRotation2jointGraphicsAnnotation"] = ss.str();
            edge = gvEdgeConnectingTwoComponents(id2edge, modelicaComponentName2id, modelGraph,
                                                  jointName, childLink);
            spl = ED_spl(edge);
            ss.str("");
            ss << "Line(points = " << toModelicaLine(spl) << ", color = {95, 95, 95})";
            joint["joint2childGraphicsAnnotation"] = ss.str();

            // Add connection between flange and joint
            edge = gvEdgeConnectingTwoComponents(id2edge, modelicaComponentName2id, modelGraph,
                                                 "axis_"+jointName, jointName);
            spl = ED_spl(edge);
            ss.str("");
            ss << "Line(points = " << toModelicaLine(spl) << ", color = {95, 95, 95})";
            joint["flange2jointGraphicsAnnotation"] = ss.str();
        }
    }

    // Dump to file for debug
    gvRender(gvc, G, "dot", stdout);

    // Free resources
    gvFreeLayout(gvc, G);
    agclose (G);
    gvFreeContext(gvc);

    return true;
}

}
