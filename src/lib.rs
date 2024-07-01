use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

// use ::ik_geo::inverse_kinematics::auxiliary::Kinematics;
// use ::ik_geo::inverse_kinematics::hardcoded::*;
// use ::ik_geo::inverse_kinematics::{
//     gen_six_dof, spherical, spherical_two_intersecting, spherical_two_parallel, three_parallel,
//     three_parallel_two_intersecting, two_intersecting, two_parallel,
// };

// use ik_geo::inverse_kinematics::setups::calculate_ik_error;

use ik_geo::nalgebra::{Matrix3, Vector3, Vector6};

use ik_geo::robot::IKSolver;
use ik_geo::robot::Robot as IKGeoRobot;
use ik_geo::robot::{irb6640, spherical_bot, three_parallel_bot, two_parallel_bot, ur5};

fn pack(rotation: [[f64; 3]; 3], translation: [f64; 3]) -> (Matrix3<f64>, Vector3<f64>) {
    let mut new_matrix = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            new_matrix[(i, j)] = rotation[j][i];
        }
    }
    (new_matrix, Vector3::from_row_slice(&translation))
}

fn unpack(rotation: Matrix3<f64>, translation: Vector3<f64>) -> ([[f64; 3]; 3], [f64; 3]) {
    let mut new_matrix = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            new_matrix[j][i] = rotation[(i, j)];
        }
    }
    (
        new_matrix,
        translation.as_slice().to_vec().try_into().unwrap(),
    )
}

// Create a class for the robot
#[pyclass()]
struct Robot {
    robot: IKGeoRobot,
}

// Implement the Robot class
#[pymethods]
impl Robot {
    // Get the inverse kinematics for the robot
    // 2d array for the rotation matrix (row major), 3 values for translation vector
    pub fn get_ik(
        &mut self,
        rot_matrix: [[f64; 3]; 3],
        trans_vec: [f64; 3],
    ) -> PyResult<Vec<([f64; 6], bool)>> {
        let mut rotation = Matrix3::zeros();
        for i in 0..3 {
            for j in 0..3 {
                rotation[(i, j)] = rot_matrix[j][i];
            }
        }

        Ok(self
            .robot
            .ik(rotation, Vector3::from_row_slice(&trans_vec))
            .into_iter()
            .map(|(q, is_ls)| {
                let mut q_vals = [0.0; 6];
                for j in 0..6 {
                    q_vals[j] = q[j];
                }
                (q_vals, is_ls)
            })
            .collect())
    }

    // Get inverse kinematics and errors, sorted by error
    pub fn get_ik_sorted(
        &mut self,
        rot_matrix: [[f64; 3]; 3],
        trans_vec: [f64; 3],
    ) -> PyResult<Vec<([f64; 6], f64, bool)>> {
        let (rotation, translation) = pack(rot_matrix, trans_vec);
        Ok(self.robot.get_ik_sorted(rotation, translation))
    }

    pub fn forward_kinematics(&self, q: [f64; 6]) -> PyResult<([[f64; 3]; 3], [f64; 3])> {
        let (rotation, translation) = self.robot.fk(&q);
        let (rot_matrix, trans_vec) = unpack(rotation, translation);

        Ok((rot_matrix, trans_vec))
    }

    // Factory methods for each robot type
    #[staticmethod]
    fn irb6640() -> PyResult<Self> {
        Ok(Robot { robot: irb6640() })
    }

    // #[staticmethod]
    // fn kuka_r800_fixed_q3() -> PyResult<Self> {
    //     Ok(Robot { robot: irb6640() })
    // }

    #[staticmethod]
    fn ur5() -> PyResult<Self> {
        Ok(Robot { robot: ur5() })
    }

    #[staticmethod]
    fn three_parallel_bot() -> PyResult<Self> {
        Ok(Robot {
            robot: three_parallel_bot(),
        })
    }

    #[staticmethod]
    fn two_parallel_bot() -> PyResult<Self> {
        Ok(Robot {
            robot: two_parallel_bot(),
        })
    }

    // #[staticmethod]
    // fn rrc_fixed_q6() -> PyResult<Self> {
    //     Self::new("rrcfixedq6")
    // }

    #[staticmethod]
    fn spherical_bot() -> PyResult<Self> {
        Ok(Robot {
            robot: spherical_bot(),
        })
    }

    // #[staticmethod]
    // fn yumi_fixed_q3() -> PyResult<Self> {
    //     Self::new("yumifixedq3")
    // }

    #[staticmethod]
    fn spherical_two_parallel() -> PyResult<Self> {
        // TODO: Get kinematics and handle these.
    }

    #[staticmethod]
    fn spherical_two_intersecting() -> PyResult<Self> {
        Self::new("sphericaltwointersecting")
    }

    #[staticmethod]
    fn spherical() -> PyResult<Self> {
        Self::new("spherical")
    }

    #[staticmethod]
    fn three_parallel_two_intersecting() -> PyResult<Self> {
        Self::new("threeparalleltwointersecting")
    }

    #[staticmethod]
    fn three_parallel() -> PyResult<Self> {
        Self::new("threeparallel")
    }

    #[staticmethod]
    fn two_parallel() -> PyResult<Self> {
        Self::new("twoparallel")
    }

    #[staticmethod]
    fn two_intersecting() -> PyResult<Self> {
        Self::new("twointersecting")
    }

    #[staticmethod]
    fn gen_six_dof() -> PyResult<Self> {
        Self::new("gensixdof")
    }
}

// fn dummy_solver_hardcoded(_: &Matrix3<f64>, _: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
//     panic!("This function should never be called");
// }

// fn dummy_solver_general(
//     _: &Matrix3<f64>,
//     _: &Vector3<f64>,
//     _: &Kinematics<6, 7>,
// ) -> (Vec<Vector6<f64>>, Vec<bool>) {
//     panic!("This function should never be called");
// }

// // Unexposed method to call the correct ik solver
// fn call_ik_solver(
//     robot: &mut Robot,
//     rot_matrix: Matrix3<f64>,
//     trans_vec: Vector3<f64>,
// ) -> (Vec<Vector6<f64>>, Vec<bool>) {
//     if robot.is_hardcoded {
//         (robot.hardcoded_solver)(&rot_matrix, &trans_vec)
//     } else {
//         // Make sure kinematics are set before calling the general solver
//         if !robot.kin_set {
//             panic!("Kinematics must be set before calling the general solver");
//         }
//         (robot.general_solver)(&rot_matrix, &trans_vec, &robot.kin)
//     }
// }

// // Kinematics wrapper class
// #[pyclass]
// #[derive(Clone)]
// struct KinematicsObject {
//     pub kin: Kinematics<6, 7>,
// }

// // Implement the Kinematics wrapper class
// #[pymethods]
// impl KinematicsObject {
//     const H_ROWS: usize = 3;
//     const P_ROWS: usize = 3;
//     const H_COLS: usize = 6;
//     const P_COLS: usize = 7;

//     // Create a new Kinematics object from
//     // h_vals: array of vals in the h matrix, column major
//     // p_vals: array of vals in the p matrix, column major
//     // Basically, both are of format [[1,0,0], [0,1,0]...] where these are the vectors
//     #[new]
//     fn new(
//         h_vals: [[f64; Self::H_ROWS]; Self::H_COLS],
//         p_vals: [[f64; Self::P_ROWS]; Self::P_COLS],
//     ) -> Self {
//         let mut kin = Kinematics::<6, 7>::new();
//         for i in 0..Self::H_ROWS {
//             for j in 0..Self::H_COLS {
//                 kin.h[(i, j)] = h_vals[j][i];
//             }
//         }
//         for i in 0..Self::P_ROWS {
//             for j in 0..Self::P_COLS {
//                 kin.p[(i, j)] = p_vals[j][i];
//             }
//         }
//         KinematicsObject { kin }
//     }
// }

// /// A Python module implemented in Rust.
// #[pymodule]
// fn ik_geo(_py: Python, m: &PyModule) -> PyResult<()> {
//     m.add_class::<Robot>()?;
//     m.add_class::<KinematicsObject>()?;
//     Ok(())
// }
