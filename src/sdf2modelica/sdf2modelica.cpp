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

#include <cstdlib>
#include <cmath>
#include <iostream>

int main(int argc, char *argv[])
{
  // TODO(traversaro) : implement actual flag parsing
  if (argc != 3)
  {
    std::cerr << "Usage: sdf2modelica inputModel.(sdf|urdf) outModel.mo" << std::endl;
    return EXIT_SUCCESS;
  }

  std::string modelicaModelString;
  bool ok = sdf_modelica::modelicaFromSDFFile(argv[1], modelicaModelString);

  if (!ok)
  {
    std::cerr << "sdf2modelica: failure in parsing SDF model from " << argv[1] << std::endl;
  }

  std::string outputFileName = argv[2];
  std::ofstream ofs (outputFileName.c_str(), std::ofstream::out);

  ofs << modelicaModelString;

  return EXIT_SUCCESS;
}
