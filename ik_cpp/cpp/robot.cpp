#include "robot.hpp"

using namespace ik_geo;
Robot::Robot(std::string robot_type) {
    this->robot = new_robot(robot_type.c_str(), robot_type.size());
}

void Robot::set_kinematics(double h_matrix[6][3], double p_matrix[7][3]) {
    set_robot_kinematics(this->robot, h_matrix, p_matrix);
}

void Robot::get_ik(double rotation_matrix[3][3], double position_vector[3], double * q_out[6], bool** is_ls_out, size_t * num_outputs) {
    get_robot_ik(this->robot, rotation_matrix, position_vector, q_out, is_ls_out, num_outputs);
}

void Robot::get_fk(double q[6], double rot_matrix_out[3][3], double pos_vector_out[3]) {
    forward_kinematics(this->robot, q, rot_matrix_out, pos_vector_out);
}

Robot Robot::irb6640() {
    return Robot("irb6640");
}

Robot Robot::kuka_r800_fixed_q3() {
    return Robot("kuka_r800_fixed_q3");
}

Robot Robot::ur5() {
    return Robot("ur5");
}

Robot Robot::three_parallel_bot() {
    return Robot("three_parallel_bot");
}

Robot Robot::two_parallel_bot() {
    return Robot("two_parallel_bot");
}

Robot Robot::rrc_fixed_q6() {
    return Robot("rrc_fixed_q6");
}

Robot Robot::spherical_bot() {
    return Robot("spherical_bot");
}

Robot Robot::yumi_fixed_q3() {
    return Robot("yumi_fixed_q3");
}

Robot Robot::spherical_two_parallel() {
    return Robot("spherical_two_parallel");
}

Robot Robot::spherical_two_intersecting() {
    return Robot("spherical_two_intersecting");
}

Robot Robot::spherical() {
    return Robot("spherical");
}

Robot Robot::three_parallel_two_intersecting() {
    return Robot("three_parallel_two_intersecting");
}

Robot Robot::three_parallel() {
    return Robot("three_parallel");
}

Robot Robot::two_parallel() {
    return Robot("two_parallel");
}

Robot Robot::two_intersecting() {
    return Robot("two_intersecting");
}

Robot Robot::gen_six_dof() {
    return Robot("gen_six_dof");
}

