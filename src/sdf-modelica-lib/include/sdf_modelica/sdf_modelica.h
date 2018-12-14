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

#ifndef SDF_MODELICA_H
#define SDF_MODELICA_H

#include <string>

#include <sdf/sdf.hh>

namespace sdf_modelica {

/**
 * \brief Options for the SDF Modelica converter
 */
struct SDFModelicaOptions
{

    /**
     * Original filename of the converted SDF.
     *
     * This filename is inserted in the generate modelica model.
     *
     * Default: empty string.
     */
    std::string originalFilename{""};

    /**
     * Frame connectors rendered on the bottom of the component.
     *
     * Default: the base frame of the model.
     */
    // std::vector<std::string> baseFrameConnectors;

    /**
     * Frame connectors rendered on the top of the component.
     *
     * Default: empty
     */
    // std::vector<std::string> topFrameConnectors;

    /**
     * Constructor, containing default values.
     */
    SDFModelicaOptions()
    {
    }
};

/**
 * \brief Create a Modelica model object from a SDF file.
 *
 * Load a SDF file and convert it to a Modelica model.
 *
 * @param[in] options options passed to the parser
 * @return true if all went ok, false otherwise.
 */
bool modelicaFromSDFFile(const std::string& sdf_filename,
                         std::string& modelica_model,
                         const SDFModelicaOptions options = SDFModelicaOptions());

/**
 * \brief Create a Modelica model object from a SDF string.
 *
 * Load a SDF string and convert it to a Modelica model.
 *
 * @param[in] options options passed to the parser
 * @return true if all went ok, false otherwise.
 */
bool modelicaFromSDFString(const std::string& sdf_string,
                           std::string& modelica_model,
                           const SDFModelicaOptions options = SDFModelicaOptions());

/**
 * \brief Create a Modelica model object from a SDF object.
 *
 * Load a SDF object and convert it to a Modelica model.
 *
 * @param[in] options options passed to the parser
 * @return true if all went ok, false otherwise.
 */
bool modelicaFromSDF(sdf::SDFPtr sdf,
                     std::string& modelica_model,
                     const SDFModelicaOptions options = SDFModelicaOptions());

} // namespace sdf_modelica

#endif
