use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;


use linear_subproblem_solutions_rust::inverse_kinematics::hardcoded::*;
use linear_subproblem_solutions_rust::inverse_kinematics::{ spherical_two_parallel, spherical_two_intersecting, three_parallel_two_intersecting, three_parallel, two_parallel, two_intersecting, spherical, gen_six_dof };
use linear_subproblem_solutions_rust::inverse_kinematics::auxiliary::Kinematics;

use linear_subproblem_solutions_rust::nalgebra::{ Matrix3, Vector3, Vector6 };

// Create a class for the robot
#[pyclass()]
struct Robot {
    // Function pointer to the correct ik solver function for setup
    hardcoded_solver: fn(&Matrix3<f64>, &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>),
    general_solver: fn(&Matrix3<f64>, &Vector3<f64>, &Kinematics<6,7>) -> (Vec<Vector6<f64>>, Vec<bool>),
    is_hardcoded: bool,
    kin: Kinematics<6,7>,
    kin_set: bool,
}

// Implement the Robot class
#[pymethods]
impl Robot {
    // Create a new robot
    #[new]
    fn new(robot_type: &str) -> PyResult<Self> {
        let hardcoded_solver: fn(&Matrix3<f64>, &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>);
        let general_solver: fn(&Matrix3<f64>, &Vector3<f64>, &Kinematics<6,7>) -> (Vec<Vector6<f64>>, Vec<bool>);
        let is_hardcoded: bool;
        match robot_type.to_lowercase().replace("_", "").replace("-","").as_str() {
            "irb6640" => {
                hardcoded_solver = irb6640;
                general_solver = dummy_solver_general;

                is_hardcoded = true;
            },
            "kukar800fixedq3" => {
                hardcoded_solver = kuka_r800_fixed_q3;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "ur5" => {
                hardcoded_solver = ur5;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "threeparallelbot" => {
                hardcoded_solver = three_parallel_bot;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "twoparallelbot" => {
                hardcoded_solver = two_parallel_bot;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "rrcfixedq6" => {
                hardcoded_solver = rrc_fixed_q6;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "sphericalbot" => {
                hardcoded_solver = spherical_bot;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "yumifixedq3" => {
                hardcoded_solver = yumi_fixed_q3;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            },
            "sphericaltwoparallel" => {
                general_solver = spherical_two_parallel;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "sphericaltwointersecting" => {
                general_solver = spherical_two_intersecting;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "spherical" => {
                general_solver = spherical;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "threeparalleltwointersecting" => {
                general_solver = three_parallel_two_intersecting;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "threeparallel" => {
                general_solver = three_parallel;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "twoparallel" => {
                general_solver = two_parallel;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "twointersecting" => {
                general_solver = two_intersecting;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            "gensixdof" => {
                general_solver = gen_six_dof;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            },
            _ => {
                return Err(PyErr::new::<PyValueError, _>("Invalid robot type, must be one of:\n
                       Irb6640, KukaR800FixedQ3, Ur5, ThreeParallelBot, TwoParallelBot, RrcFixedQ6, SphericalBot, YumiFixedQ3, \n
                       SphericalTwoParallel, SphericalTwoIntersecting, Spherical, ThreeParallelTwoIntersecting, ThreeParallel, TwoParallel, TwoIntersecting, GenSixDof"));
            }
        }

        Ok(Robot {
            hardcoded_solver,
            general_solver,
            is_hardcoded,
            kin_set: false,
            kin: Kinematics::<6,7>::new()
        })
    }

    // Set the kinematics for the robot
    pub fn set_kinematics(&mut self, kin: KinematicsObject) -> PyResult<()> {
        self.kin = kin.kin;
        self.kin_set = true;
        Ok(())
    }

    // Get the inverse kinematics for the robot
    // 2d array for the rotation matrix (row major), 3 values for translation vector
    pub fn get_ik(&mut self, rot_matrix: [[f64; 3]; 3], trans_matrix: [f64; 3]) -> PyResult<([f64; 6], bool)> {
        // Convert the input to the correct types
        let mut rotation = Matrix3::zeros();
        // Fill rotation matrix
        for i in 0..3 {
            for j in 0..3 {
                rotation[(i,j)] = rot_matrix[j][i];
            }
        }

        let translation = Vector3::from_row_slice(&trans_matrix);
        let q : Vec<Vector6<f64>>;
        let is_ls : Vec<bool>;
        if self.is_hardcoded {
            (q, is_ls) = (self.hardcoded_solver)(&rotation, &translation);
        } else {
            // Make sure kinematics are set before calling the general solver
            if !self.kin_set {
                return Err(PyValueError::new_err("Kinematics must be set before calling the general solver"));
            }
            (q, is_ls) = (self.general_solver)(&rotation, &translation, &self.kin)
        }
        let mut ret_vals = [0.0; 6];
        let mut is_least_squares = true;
        if q.len() > 0 {
            for i in 0..6 {
                ret_vals[i] = q[0][i];
            }
            is_least_squares = is_ls[0];
        }
        Ok((ret_vals, is_least_squares))
    }

    pub fn forward_kinematics(&self, q: [f64; 6]) -> PyResult<([[f64; 3];3], [f64; 3])> {
        if !self.kin_set && !self.is_hardcoded {
            return Err(PyValueError::new_err("Kinematics must be set before calling forward kinematics"));
        } else if self.is_hardcoded {
            return Err(PyValueError::new_err("Forward kinematics not implemented for hardcoded solvers"));
        }
        let mut q_vec = Vector6::zeros();
        for i in 0..6 {
            q_vec[i] = q[i];
        }
        let (r, p) = self.kin.forward_kinematics(&q_vec);
        let mut r_vals = [[0.0; 3]; 3];
        let mut p_vals = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                r_vals[j][i] = r[(i,j)];
            }
            p_vals[i] = p[i];
        }
        Ok((r_vals, p_vals))
        
    }
}

fn dummy_solver_hardcoded(_ :  &Matrix3<f64>, _ : &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    panic!("This function should never be called");
}

fn dummy_solver_general(_:  &Matrix3<f64>, _ : &Vector3<f64>, _ : &Kinematics<6,7>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    panic!("This function should never be called");
}

// Kinematics wrapper class
#[pyclass]
#[derive(Clone)]
struct KinematicsObject {
    pub kin: Kinematics<6,7>,
}

// Implement the Kinematics wrapper class
#[pymethods]
impl KinematicsObject {
    const H_ROWS: usize = 3;
    const P_ROWS: usize = 3;
    const H_COLS: usize = 6;
    const P_COLS: usize = 7;
    
    // Create a new Kinematics object from
    // h_vals: array of vals in the h matrix, column major
    // p_vals: array of vals in the p matrix, column major
    // Basically, both are of format [[1,0,0], [0,1,0]...] where these are the vectors
    #[new]
    fn new (h_vals : [[f64; Self::H_ROWS]; Self::H_COLS], p_vals : [[f64; Self::P_ROWS]; Self::P_COLS]) -> Self {
        let mut kin = Kinematics::<6,7>::new();
        for i in 0..Self::H_ROWS {
            for j in 0..Self::H_COLS {
                kin.h[(i,j)] = h_vals[j][i];
            }
        }
        for i in 0..Self::P_ROWS {
            for j in 0..Self::P_COLS {
                kin.p[(i,j)] = p_vals[j][i];
            }
        }
        KinematicsObject {
            kin
        }
    }
    
}


/// A Python module implemented in Rust.
#[pymodule]
fn ik_python(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<Robot>()?;
    m.add_class::<KinematicsObject>()?;
    Ok(())
}
