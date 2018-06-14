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

#include <sdf_modelica/sdf_modelica.h>
#include <sdf_modelica/sdf_modelica_diagram_layout.h>
#include <sdf_modelica/sdf_modelica_diagram_layout_graphviz.h>
#include <sdf_modelica/nlohmann_json_to_kainjow_mustache.h>

#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/math/graph/Graph.hh>

#include <nlohmann/json.hpp>

#include <mustache.hpp>


#include "template_path.hpp"
#include <cassert>
#include <map>
#include <sstream>
#include <regex>

namespace sdf_modelica
{


bool modelicaFromSDFFile(const std::string& sdf_filename,
                         std::string& modelica_model,
                         const SDFModelicaOptions options)
{
  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  if (!sdf::readFile(sdf_filename, sdfElement))
  {
    std::cerr << "sdf_modelica: " << sdf_filename << " is not a valid SDF file." << std::endl;
    return false;
  }

  SDFModelicaOptions modifiableOptions(options);
  modifiableOptions.originalFilename = sdf_filename;

  return modelicaFromSDF(sdfElement, modelica_model, modifiableOptions);
}

bool modelicaFromSDFString(const std::string& sdf_string,
                           std::string& modelica_model,
                           const SDFModelicaOptions options)
{
  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  if (!sdf::readString(sdf_string, sdfElement))
  {
    std::cerr << "sdf_modelica: " << sdf_string << " is not a valid SDF string." << std::endl;
    return false;
  }

  return modelicaFromSDF(sdfElement, modelica_model, options);
}

bool isIdentity(const ignition::math::Quaterniond& q, const double tol = 1e-7)
{
    return (std::abs((q.W()-1)) < tol) &&
               (std::abs(q.X()) < tol) &&
               (std::abs(q.Y()) < tol) &&
               (std::abs(q.Z()) < tol);
}

// Unfortunatly, the overloading of operator* is extremly counterintuitive,
// especially if you are used to work with 4x4 homogeneous matrices
// See https://bitbucket.org/osrf/gazebo/issues/216/pose-addition-and-subtraction-needs-work#comment-18702360
ignition::math::Pose3d SE3Multiplication(const ignition::math::Pose3d& firstTerm, const ignition::math::Pose3d& secondTerm)
{
  return (firstTerm*secondTerm).Inverse();
}

// Convert a ign vector to a string in modelica format
std::string toModelicaVector(const ignition::math::Vector3d& vec)
{
  // TODO(traversaro): find a way to express a floating point number without
  // losing precision, but also using the minimal number of chars
  std::stringstream ss;
  ss << "{" << vec.X() << "," << vec.Y() << "," << vec.Z() << "}";
  return ss.str();
}

bool modelicaFromSDF(sdf::SDFPtr sdf,
                     std::string& modelica_model,
                     const SDFModelicaOptions options)
{
  // We build a ign-math graph of the Modelica components
  // to simplify generating graphical annotations via graphviz
  DiagramGraph modelGraph;

  const sdf::ElementPtr rootElement = sdf->Root();
  if (!rootElement->HasElement("model"))
  {
    std::cerr << "sdf_modelica: Passed SDF does not contain a model." << std::endl;
    return false;
  }
  const sdf::ElementPtr modelElement = rootElement->GetElement("model");

  // Populate json-like data structure that will be consumed by the template
  // nlohmann::json is used for the internal representation of the data due to
  // its maturity and documentation, but it is in the end converted to
  // kainjow::mustache::data for using the kainjow::mustache mustache template library
  nlohmann::json data;

  // Original filename
  data["fileName"] = options.originalFilename;

  // modelName
  const std::string tmpModelName=modelElement->Get<std::string>("name");
  data["modelName"] = tmpModelName;
  
  // Modelica Language specification Version 3.3 Appendix B
  if (!std::regex_match (tmpModelName, std::regex("\\w+") ))
  {
    std::cerr << "sdf_modelica: Passed SDF modelName contains not allowed char:"<<tmpModelName << std::endl;
    return false;
  }
      
  std::map<std::string, ignition::math::Pose3d> linkPoses;

  // parse model links
  nlohmann::json links = nlohmann::json::array();
  sdf::ElementPtr linkElement = modelElement->GetElement("link");


  // Handle world
  std::map<std::string, ignition::math::graph::VertexId> modelicaComponent2ignGraphId;
  modelicaComponent2ignGraphId["world"] = modelGraph.AddVertex("world", ModelicaGraphComponentInfo("world")).Id();

  while (linkElement)
  {
    nlohmann::json link;

    const std::string linkName = linkElement->Get<std::string>("name");
    link["name"] = linkName;
    // Modelica Language specification Version 3.3 Appendix B
    if (!std::regex_match (linkName, std::regex("\\w+") ))
    {
      std::cerr << "sdf_modelica: Passed SDF link name contains not allowed char:"<<linkName << std::endl;
      return false;
    }

    // Set default values
    ignition::math::Pose3d inertiaPose = ignition::math::Pose3d::Zero;
    ignition::math::MassMatrix3d massMatrix = ignition::math::MassMatrix3d(1.0,
                                                                           ignition::math::Vector3d::One,
                                                                           ignition::math::Vector3d::Zero);

    if (linkElement->HasElement("inertial"))
    {
      sdf::ElementPtr inertialElem = linkElement->GetElement("inertial");
      if (inertialElem->HasElement("pose"))
      {
        inertiaPose =
          inertialElem->GetElement("pose")->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero).first;
      }

      // Get the mass.
      massMatrix.Mass(inertialElem->Get<double>("mass", 1.0).first);

      if (inertialElem->HasElement("inertia"))
      {
        sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
        massMatrix.IXX(inertiaElem->Get<double>("ixx", 1.0).first);
        massMatrix.IYY(inertiaElem->Get<double>("iyy", 1.0).first);
        massMatrix.IZZ(inertiaElem->Get<double>("izz", 1.0).first);
        massMatrix.IXY(inertiaElem->Get<double>("ixy", 0.0).first);
        massMatrix.IXZ(inertiaElem->Get<double>("ixz", 0.0).first);
        massMatrix.IYZ(inertiaElem->Get<double>("iyz", 0.0).first);
      }
    }

    if (!isIdentity(inertiaPose.Rot()))
    {
        ignition::math::Matrix3d link_H_inertial = ignition::math::Matrix3d(inertiaPose.Rot());
        massMatrix.MOI(link_H_inertial*massMatrix.MOI()*link_H_inertial.Transposed());
    }

    link["mass"] = std::to_string(massMatrix.Mass());
    link["centerOfMass"] = toModelicaVector(inertiaPose.Pos());
    link["I_11"] = std::to_string(massMatrix.IXX());
    link["I_22"] = std::to_string(massMatrix.IYY());
    link["I_33"] = std::to_string(massMatrix.IZZ());
    link["I_21"] = std::to_string(massMatrix.IXY());
    link["I_31"] = std::to_string(massMatrix.IXZ());
    link["I_23"] = std::to_string(massMatrix.IYZ());

    // Store the initial pose of each link, it is necessary for computing the joint transforms
    linkPoses[linkName] = linkElement->GetElement("pose")->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero).first;

    links.push_back(link);
    modelicaComponent2ignGraphId[linkName] = modelGraph.AddVertex(linkName, linkName).Id();

    linkElement = linkElement->GetNextElement("link");
  }
  data["links"] = links;

  // parse model joints
  nlohmann::json joints = nlohmann::json::array();
  sdf::ElementPtr jointElement = modelElement->GetElement("joint");
  while (jointElement)
  {
    nlohmann::json joint;

    const std::string jointName = jointElement->Get<std::string>("name");
    joint["name"] = jointName;
   // Modelica Language specification Version 3.3 Appendix B
    if (!std::regex_match (jointName, std::regex("\\w+") ))
    {
      std::cerr << "sdf_modelica: Passed SDF joint name contains not allowed char:"<<jointName << std::endl;
      return false;
    }

    std::string jointType = jointElement->Get<std::string>("type");
    joint["type"] = jointType;

    if ( !( (jointType == "revolute") ||
            (jointType == "prismatic") ||
            (jointType == "fixed") ) )
    {
      std::cerr << "sdf_modelica: type " << jointType
                << " of joint " << jointName << " not supported, parsing failed."
                << std::endl;
      return false;
    }

    joint["isType_"+jointType] = true;

    std::string parentLink, childLink;
    joint["parentLink"] = parentLink = jointElement->GetElement("parent")->Get<std::string>();
    joint["childLink"] =  childLink  = jointElement->GetElement("child")->Get<std::string>();
    if (parentLink == "world")
    {
      joint["parentLinkIsWorld"] = true;
    }

    // The joint transformation is encoded in the SDF format in the
    // initial pose of each link
    ignition::math::Pose3d model_T_parentLink = linkPoses[jointElement->GetElement("parent")->Get<std::string>()];
    ignition::math::Pose3d model_T_childLink = linkPoses[jointElement->GetElement("child")->Get<std::string>()];
    ignition::math::Pose3d parentLink_T_childLink = SE3Multiplication(model_T_parentLink.Inverse(), model_T_childLink);

    // Extract the fixed parent <---> child transform
    joint["fixedTransformTrans"] = toModelicaVector(parentLink_T_childLink.Pos());
    ignition::math::Vector3d rotAxis;
    double rotAngle;
    parentLink_T_childLink.Rot().ToAxis(rotAxis, rotAngle);
    joint["fixedTransformAxis"] = toModelicaVector(rotAxis);
    // TODO(traversaro) : handle precision and locale issues
    joint["fixedTransformAngle"] = std::to_string(rotAngle);

    if ( (jointType == "revolute") || (jointType == "prismatic") )
    {
      // In sdf, the axis of the joint is expressed in the child frame, unless use_parent_model_frame is set to true

      // TODO(traversaro) : handle use_parent_model_frame
      ignition::math::Vector3d axisInSDF= jointElement->GetElement("axis")->GetElement("xyz")->Get<ignition::math::Vector3d>();
      bool use_parent_model_frame = jointElement->GetElement("axis")->GetElement("use_parent_model_frame")->Get<bool>();
      ignition::math::Vector3d axisInChildFrame;

      if (use_parent_model_frame)
      {
        axisInChildFrame = model_T_childLink.Rot().Inverse()*(axisInSDF);
      }
      else
      {
        axisInChildFrame = axisInSDF;
      }

      joint["modelicaAxis"] = toModelicaVector(axisInChildFrame);
    }

    // For each joint we have one modelica component: the fixed transformation,
    // and for joints with non-zero DOF we also have the joint component
    modelicaComponent2ignGraphId[jointName+"_fixedRotation"] = modelGraph.AddVertex(jointName+"_fixedRotation", jointName+"_fixedRotation").Id();

    modelGraph.AddEdge(ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[parentLink], modelicaComponent2ignGraphId[jointName+"_fixedRotation"]),
                           ModelicaGraphConnectionInfo(ConnectionSide::EAST, ConnectionSide::WEST));

    if ( (jointType == "revolute") || (jointType == "prismatic") )
    {
        modelicaComponent2ignGraphId[jointName] = modelGraph.AddVertex(jointName, jointName).Id();

        modelGraph.AddEdge(ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[jointName+"_fixedRotation"], modelicaComponent2ignGraphId[jointName]),
                           ModelicaGraphConnectionInfo(EAST, WEST));
        modelGraph.AddEdge(ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[jointName], modelicaComponent2ignGraphId[childLink]),
                           ModelicaGraphConnectionInfo(EAST, EAST));
    }
    else
    {
        modelGraph.AddEdge(ignition::math::graph::VertexId_P(modelicaComponent2ignGraphId[jointName+"_fixedRotation"], modelicaComponent2ignGraphId[childLink]),
                          ModelicaGraphConnectionInfo(EAST, EAST));
    }

    joints.push_back(joint);
    jointElement = jointElement->GetNextElement("joint");
  }
  data["joints"] = joints;

  // Add graphical annotations
  bool ok = add_icon_layout(modelGraph, data, modelicaComponent2ignGraphId);
  if (!ok)
  {
     std::cerr << "sdf_modelica: error in function add_icon_layout, exiting." << std::endl;
     return false;
  }

  // For now, do not try to add diagram annotation as the results are still not ready
  ok = add_diagram_layout_dummy(modelGraph, data);
  if (!ok)
  {
    std::cerr << "sdf_modelica: error in function add_diagram_layout_handtuned, exiting." << std::endl;
    return false;
  }

  // Load mustache template for the Modelica model
  std::string mustache_template = SDF_MODELICA_TEMPLATE_PATH + "/mustache-modelica.txt";

  std::ifstream t(mustache_template);
  std::string mustache_template_str((std::istreambuf_iterator<char>(t)),
                                     std::istreambuf_iterator<char>());

  kainjow::mustache::mustache tmpl{mustache_template_str};

  kainjow::mustache::data mustache_data = nlohmann_json_to_kainjow_mustache(data);
  modelica_model = tmpl.render(mustache_data);
  if (!tmpl.is_valid())
  {
    std::cerr << "sdf_modelica: mustache template not valid, error message: " << tmpl.error_message() << std::endl;
    return false;
  }

  return true;
}

}
