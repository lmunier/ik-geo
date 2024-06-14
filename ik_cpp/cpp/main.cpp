#include <iostream>
#include "robot.hpp"
using namespace ik_geo;

int main(int argc, char const *argv[])
{
    Robot robot("ur5");
    // Create identity matrix
    double rotation_matrix[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    double position_vector[3] = {0.0, 0.0, 0.0};

    double q_out[6];
    bool is_ls_out;
    robot.get_ik(rotation_matrix, position_vector, q_out, &is_ls_out);

    std::cout << "q: ";
    for (int i = 0; i < 6; i++) {
        std::cout << q_out[i] << " ";
    }

    return 0;
}
