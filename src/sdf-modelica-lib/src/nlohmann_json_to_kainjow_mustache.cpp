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

#include <sdf_modelica/nlohmann_json_to_kainjow_mustache.h>

namespace sdf_modelica
{

bool nlohmann_json_to_kainjow_mustache(const nlohmann::json& inputData, kainjow::mustache::data& outputData)
{
    outputData = nlohmann_json_to_kainjow_mustache(inputData);
    return true;
}

kainjow::mustache::data nj_to_jm_object_helper(const nlohmann::json& inputData)
{
    assert(inputData.type() == nlohmann::json::value_t::object);
    kainjow::mustache::data km_data;
    for (nlohmann::json::const_iterator it = inputData.begin(); it != inputData.end(); ++it)
    {
        km_data[it.key()] = nlohmann_json_to_kainjow_mustache(it.value());
    }
    return km_data;
}

kainjow::mustache::data nj_to_jm_array_helper(const nlohmann::json& inputData)
{
    assert(inputData.type() == nlohmann::json::value_t::array);
    kainjow::mustache::data km_list{kainjow::mustache::data::type::list};
    for (nlohmann::json::const_iterator it = inputData.begin(); it != inputData.end(); ++it)
    {
        km_list.push_back(nlohmann_json_to_kainjow_mustache(*it));
    }
    return km_list;
}

kainjow::mustache::data nlohmann_json_to_kainjow_mustache(const nlohmann::json& inputData)
{
    nlohmann::json::value_t type = inputData.type();

    switch(type)
    {
        case nlohmann::json::value_t::null:
            return kainjow::mustache::data();
            break;
        case nlohmann::json::value_t::object:
            return nj_to_jm_object_helper(inputData);
            break;
        case nlohmann::json::value_t::array:
            return nj_to_jm_array_helper(inputData);
            break;
        case nlohmann::json::value_t::string:
            return kainjow::mustache::data(inputData.get<std::string>());
            break;
        case nlohmann::json::value_t::boolean:
            return kainjow::mustache::data(inputData.get<bool>());
            break;
        case nlohmann::json::value_t::number_integer:
            return kainjow::mustache::data(std::to_string(inputData.get<int>()));
            break;
        case nlohmann::json::value_t::number_unsigned:
            return kainjow::mustache::data(std::to_string(inputData.get<unsigned int>()));
            break;
        case nlohmann::json::value_t::number_float:
            return kainjow::mustache::data(std::to_string(inputData.get<double>()));
            break;
        case nlohmann::json::value_t::discarded:
            return kainjow::mustache::data();
            break;
        default:
            return kainjow::mustache::data();
            break;
    }
}




}
