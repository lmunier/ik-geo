#include <iostream>
#include "include/ik_geo.h"


using namespace ik_geo;
void run_ik_hardcoded() {
    std::cout << "\nRunning hardcoded inverse kinematics:\n-----------------------------" << std::endl;

    std::cout << "UR5:" << std::endl;
    
    // Create the robot from a string
    Robot ur5("ur5");

    double rotation_matrix[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double position_vector[3] = {0.0, 0.0, 0.0};

    double q[6];
    bool is_ls;
    // Run ik, outputs to q and is_ls
    ur5.get_ik(rotation_matrix, position_vector, q, &is_ls);

    std::cout << "q: ";
    for (int i = 0; i < 6; i++) {
        std::cout << q[i] << " ";
    }

    std::cout << "\nis_ls: " << is_ls << std::endl;    
}

void run_ik_general() {
    std::cout << "\nRunning general inverse kinematics:\n-----------------------------" << std::endl;

    Robot robot("SphericalTwoIntersecting");

    // Must set the kinematics for a general robot
    double zv[3] = {0.0, 0.0, 0.0};
    double ez[3] = {0.0, 0.0, 1.0};
    double ey[3] = {0.0, 1.0, 0.0};
    double ex[3] = {1.0, 0.0, 0.0};

    double hMat[6][3] = {
        {ez[0], ez[1], ez[2]},
        {ey[0], ey[1], ey[2]},
        {ey[0], ey[1], ey[2]},
        {ex[0], ex[1], ex[2]},
        {ey[0], ey[1], ey[2]},
        {ex[0], ex[1], ex[2]}
    };
    double pMat[7][3] = {
        {zv[0], zv[1], zv[2]},
        {0.32, 0.0, 0.78},
        {0.0, 0.0, 1.075},
        {1.1425, 0.0, 0.2},
        {ez[0], ez[1], ez[2]},
        {ez[0], ez[1], ez[2]},
        {0.2, 0.0, 0.0}
    };

    std::cout << "Setting kinematics for SphericalTwoIntersecting" << std::endl;
    robot.set_kinematics(hMat, pMat);



    double rotation_matrix[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double position_vector[3] = {0, 0, 0};

    double q[6];
    bool is_ls;

    std::cout << "Running inverse kinematics for SphericalTwoIntersecting" << std::endl;
    robot.get_ik(rotation_matrix, position_vector, q, &is_ls);

    std::cout << "q: ";
    for (int i = 0; i < 6; i++) {
        std::cout << q[i] << " ";
    }

    std::cout << "\nis_ls: " << is_ls << std::endl;


}

int main() {
    run_ik_hardcoded();
    run_ik_general();
    return 0;
}