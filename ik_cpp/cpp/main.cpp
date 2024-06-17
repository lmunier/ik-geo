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

    double * q_out;
    bool * is_ls_out;
    size_t num_outputs;

    robot.get_ik(rotation_matrix, position_vector, &q_out, &is_ls_out, &num_outputs);

    for (size_t i = 0; i < num_outputs; i++) {
        std::cout << "Solution " << i << ": ";
        for (size_t j = 0; j < 6; j++) {
            std::cout << q_out[i * 6 + j] << " ";
        }

        std::cout << "Is LS: " << is_ls_out[i] << std::endl;
        
        std::cout << std::endl;
    }

    return 0;
}
