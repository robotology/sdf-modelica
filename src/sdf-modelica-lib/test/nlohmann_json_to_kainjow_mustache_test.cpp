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

void dummyAssert(bool condition)
{
    if (!condition)
    {
        exit(EXIT_FAILURE);
    }
}

void testArrayConversions()
{
    nlohmann::json nj_array;
    nj_array.push_back("1");
    nj_array.push_back("2");
    nj_array.push_back("3");
    nj_array.push_back(4);

    kainjow::mustache::data km_array = sdf_modelica::nlohmann_json_to_kainjow_mustache(nj_array);
    dummyAssert(km_array.is_list());
    kainjow::mustache::list km_list = km_array.list_value();
    dummyAssert(nj_array.size() == km_list.size());
    for (int i=0; i < nj_array.size(); i++)
    {
        std::string nj_val;
        if (nj_array[i].is_string())
        {
            nj_val = nj_array[i];
        }
        if (nj_array[i].is_number_integer())
        {
            int nj_val_int = nj_array[i];
            nj_val = std::to_string(nj_val_int);
        }
        if (nj_array[i].is_number_float())
        {
            double nj_val_dbl = nj_array[i];
            nj_val = std::to_string(nj_val_dbl);
        }
        dummyAssert(nj_val == km_list[i].string_value());
    }
}


int main(int argc, char** argv)
{
    testArrayConversions();
}
