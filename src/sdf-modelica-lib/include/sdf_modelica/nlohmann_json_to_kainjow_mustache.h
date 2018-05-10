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

#ifndef NLOHMANN_JSON_TO_KAINJOW_MUSTACHE_H
#define NLOHMANN_JSON_TO_KAINJOW_MUSTACHE_H

#include <string>

#include <nlohmann/json.hpp>
#include <mustache.hpp>

namespace sdf_modelica
{

/**
 * @brief Convert a  nlohmann::json to a kainjow::mustache::data
 *
 * @param[in] inputData Input data in nlohmann::json format.
 * @param[out] outputData Output data in kainjow::mustache::data format.
 * @return true if the conversion was successful, false otherwise.
 */
bool nlohmann_json_to_kainjow_mustache(const nlohmann::json& inputData,
                                       kainjow::mustache::data& outputData);

/**
 * @brief Convert a  nlohmann::json to a kainjow::mustache::data
 *
 * @param[in] inputData Input data in nlohmann::json format.
 * @return Output data in kainjow::mustache::data format.
 */
kainjow::mustache::data nlohmann_json_to_kainjow_mustache(const nlohmann::json& inputData);

}

#endif
