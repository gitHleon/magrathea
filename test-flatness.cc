/*
 * test.flatness.cc
 */

#include "flatness.h"

int main(int argc, char **argv)
{
    if (argv[1] == nullptr)
    {
        std::cout << "I need a data file" << std::endl;
        return 1;
    }
    alglib::real_2d_array CC;
    alglib::read_csv(argv[1], '\t', 0, CC);
    double F = flatness_lin(CC);
    std::cout << "Flatness: " << F << std::endl;
    return 0;
}

