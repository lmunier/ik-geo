#include <string>

extern "C" {
    // Use the robot struct from the rust library
    struct _RustRobot;

    // Create a new robot
    _RustRobot* new_robot(const char* robot_type, size_t robot_type_len);

    // Set kinematics for the robot
    void set_robot_kinematics(_RustRobot* robot, double h_matrix[6][3], double p_matrix[7][3]);

    // Get the inverse kinematics
    void get_robot_ik(_RustRobot* robot, double rotation_matrix[3][3], double position_vector[3], double ** q_out, bool** is_ls_out, size_t * num_outputs);

    // Forward kinematics
    void forward_kinematics(_RustRobot* robot, double q[6], double rot_matrix_out[3][3], double pos_vector_out[3]);

    // All the robot type factory functions
    _RustRobot* new_irb6640();
    _RustRobot* new_kuka_r800_fixed_q3();
    _RustRobot* new_ur5();
    _RustRobot* new_three_parallel_bot();
    _RustRobot* new_two_parallel_bot();
    _RustRobot* new_rrc_fixed_q6();
    _RustRobot* new_spherical_bot();
    _RustRobot* new_yumi_fixed_q3();
    _RustRobot* new_spherical_two_parallel();
    _RustRobot* new_spherical_two_intersecting();
    _RustRobot* new_spherical();
    _RustRobot* new_three_parallel_two_intersecting();
    _RustRobot* new_three_parallel();
    _RustRobot* new_two_parallel();
    _RustRobot* new_two_intersecting();
    _RustRobot* new_gen_six_dof();
}
namespace ik_geo {
    // Do not use the _RustRobot in your own code, the struct Robot below is what is intended to be used
    


    /**
     * Class that represents a certain robot
     */
    class Robot {
        public:
            /**
             * Create a new robot
             * @param robot_type The type of robot to create, must be one of:
             * irb6640, kuka_r800_fixed_q3, ur5, three_parallel_bot, two_parallel_bot, rrc_fixed_q6, spherical_bot, yumi_fixed_q3, spherical_two_parallel, spherical_two_intersecting, spherical, three_parallel_two_intersecting, three_parallel, two_parallel, two_intersecting, gen_six_dof
             * @return The robot struct
             */
            Robot(std::string robot_type);
            ~Robot() {
                // delete robot;
            }

            /**
             * Set the kinematics for the robot
             * @param h_matrix The h matrix (6x3, contains the axes of rotation for each joint)
             * @param p_matrix The p matrix (7x3, contains the displacement between any two joints)
             */
            void set_kinematics(double h_matrix[6][3], double p_matrix[7][3]);

            /**
             * Get the inverse kinematics
             * @param rotation_matrix The rotation matrix
             * @param position_vector The position vector
             * @param q_out Array of 6-vectors representing output joint angles
             * @param is_ls_out Array of bools for whether each solution is least squares
             * @param num_outputs The number of output solutions
             */
            void get_ik(double rotation_matrix[3][3], double position_vector[3], double * q_out[6], bool** is_ls_out, size_t * num_outputs);


            /**
             * Get the forward kinematics
             * @param q The joint angles
             * @param rot_matrix_out The rotation matrix output
             * @param pos_vector_out The position vector output
             */
            void get_fk(double q[6], double rot_matrix_out[3][3], double pos_vector_out[3]);

            // Factory functions

            // Create irb6640 robot
            static Robot irb6640();
            // Create kuka_r800_fixed_q3 robot
            static Robot kuka_r800_fixed_q3();
            // Create ur5 robot
            static Robot ur5();
            // Create three_parallel_bot robot
            static Robot three_parallel_bot();
            // Create two_parallel_bot robot
            static Robot two_parallel_bot();
            // Create rrc_fixed_q6 robot
            static Robot rrc_fixed_q6();
            // Create spherical_bot robot
            static Robot spherical_bot();
            // Create yumi_fixed_q3 robot
            static Robot yumi_fixed_q3();
            // Create spherical_two_parallel robot
            static Robot spherical_two_parallel();
            // Create spherical_two_intersecting robot
            static Robot spherical_two_intersecting();
            // Create spherical robot
            static Robot spherical();
            // Create three_parallel_two_intersecting robot
            static Robot three_parallel_two_intersecting();
            // Create three_parallel robot
            static Robot three_parallel();
            // Create two_parallel robot
            static Robot two_parallel();
            // Create two_intersecting robot
            static Robot two_intersecting();
            // Create gen_six_dof robot
            static Robot gen_six_dof();

        private:
            _RustRobot* robot;
    };

}