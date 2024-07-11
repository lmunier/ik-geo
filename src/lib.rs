use ::ik_geo::inverse_kinematics::auxiliary::Matrix3x7;
use pyo3::prelude::*;

use ::ik_geo::nalgebra::{Matrix3, Matrix3x6, Vector3};

use ::ik_geo::robot::IKSolver;
use ::ik_geo::robot::Robot as IKGeoRobot;
use ::ik_geo::robot;

fn pack(rotation: [[f64; 3]; 3], translation: [f64; 3]) -> (Matrix3<f64>, Vector3<f64>) {
    let mut new_matrix: Matrix3<f64> = Matrix3::zeros();
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

fn pack_kinematics(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> (Matrix3x6<f64>, Matrix3x7<f64>) {
    let mut new_h = Matrix3x6::zeros();
    let mut new_p = Matrix3x7::zeros();
    for i in 0..3 {
        for j in 0..6 {
            new_h[(i, j)] = h[j][i];
        }
    }
    for i in 0..3 {
        for j in 0..7 {
            new_p[(i, j)] = p[j][i];
        }
    }
    (new_h, new_p)
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
        Ok(Robot { robot: robot::irb6640() })
    }

    // #[staticmethod]
    // fn kuka_r800_fixed_q3() -> PyResult<Self> {
    //     Ok(Robot { robot: irb6640() })
    // }

    #[staticmethod]
    fn ur5() -> PyResult<Self> {
        Ok(Robot { robot: robot::ur5() })
    }

    #[staticmethod]
    fn three_parallel_bot() -> PyResult<Self> {
        Ok(Robot {
            robot: robot::three_parallel_bot(),
        })
    }

    #[staticmethod]
    fn two_parallel_bot() -> PyResult<Self> {
        Ok(Robot {
            robot: robot::two_parallel_bot(),
        })
    }

    // #[staticmethod]
    // fn rrc_fixed_q6() -> PyResult<Self> {
    //     Self::new("rrcfixedq6")
    // }

    #[staticmethod]
    fn spherical_bot() -> PyResult<Self> {
        Ok(Robot {
            robot: robot::spherical_bot(),
        })
    }

    // #[staticmethod]
    // fn yumi_fixed_q3() -> PyResult<Self> {
    //     Self::new("yumifixedq3")
    // }

    

    #[staticmethod]
    fn spherical_two_parallel(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::spherical_two_parallel(new_h, new_p),
        })
    }
    

    #[staticmethod]
    fn spherical_two_intersecting(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::spherical_two_intersecting(new_h, new_p),
        })
    }

    #[staticmethod]
    fn spherical(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::spherical(new_h, new_p),
        })
    }

    #[staticmethod]
    fn three_parallel_two_intersecting(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::three_parallel_two_intersecting(new_h, new_p),
        })
    }


    #[staticmethod]
    fn three_parallel(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::three_parallel(new_h, new_p),
        })
    }

    #[staticmethod]
    fn two_parallel(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::two_parallel(new_h, new_p),
        })
    }

    #[staticmethod]
    fn two_intersecting(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::two_intersecting(new_h, new_p),
        })
    }

    #[staticmethod]
    fn gen_six_dof(h: [[f64; 3]; 6], p: [[f64; 3]; 7]) -> PyResult<Self> {
        let (new_h, new_p) = pack_kinematics(h, p);
        Ok(Robot {
            robot: robot::gen_six_dof(new_h, new_p),
        })
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

/// A Python module implemented in Rust.
#[pymodule]
fn ik_geo(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<Robot>()?;
    Ok(())
}
