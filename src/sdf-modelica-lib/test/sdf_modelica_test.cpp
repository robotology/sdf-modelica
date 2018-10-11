#include <cmath>
#include <cstdlib>
#include <iostream>

#include <sdf_modelica/sdf_modelica.h>

void dummyAssert(bool condition)
{
    if (!condition) {
        exit(EXIT_FAILURE);
    }
}

void check_file(std::string file)
{
    std::string modelica_model;
    bool ok = sdf_modelica::modelicaFromSDFFile(file, modelica_model);
    std::cerr << "~~~~~~> Start generated modelica model <~~~~~" << std::endl;
    std::cerr << modelica_model << std::endl;
    std::cerr << "~~~~~~> End generated modelica model <~~~~~" << std::endl;
    dummyAssert(ok);
}

int main(int argc, char** argv)
{
    check_file(argv[1]);
    return EXIT_SUCCESS;
}
