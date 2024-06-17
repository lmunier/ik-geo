
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use linear_subproblem_solutions_rust::inverse_kinematics::auxiliary::Kinematics;
use linear_subproblem_solutions_rust::inverse_kinematics::hardcoded::*;
use linear_subproblem_solutions_rust::inverse_kinematics::{
    gen_six_dof, spherical, spherical_two_intersecting, spherical_two_parallel, three_parallel,
    three_parallel_two_intersecting, two_intersecting, two_parallel,
};


use linear_subproblem_solutions_rust::inverse_kinematics::setups::calculate_ik_error;

use linear_subproblem_solutions_rust::nalgebra::{Matrix3, Vector3, Vector6};

// Create a class for the robot
#[pyclass()]
struct Robot {
    // Function pointer to the correct ik solver function for setup
    hardcoded_solver: fn(&Matrix3<f64>, &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>),
    general_solver:
        fn(&Matrix3<f64>, &Vector3<f64>, &Kinematics<6, 7>) -> (Vec<Vector6<f64>>, Vec<bool>),
    is_hardcoded: bool,
    kin: Kinematics<6, 7>,
    kin_set: bool,
}

// Implement the Robot class
#[pymethods]
impl Robot {
    // Create a new robot
    #[new]
    fn new(robot_type: &str) -> PyResult<Self> {
        let hardcoded_solver: fn(&Matrix3<f64>, &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>);
        let general_solver: fn(
            &Matrix3<f64>,
            &Vector3<f64>,
            &Kinematics<6, 7>,
        ) -> (Vec<Vector6<f64>>, Vec<bool>);
        let is_hardcoded: bool;

        match robot_type
            .to_lowercase()
            .replace("_", "")
            .replace("-", "")
            .as_str()
        {
            "irb6640" => {
                hardcoded_solver = irb6640;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "kukar800fixedq3" => {
                hardcoded_solver = kuka_r800_fixed_q3;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "ur5" => {
                hardcoded_solver = ur5;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "threeparallelbot" => {
                hardcoded_solver = three_parallel_bot;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "twoparallelbot" => {
                hardcoded_solver = two_parallel_bot;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "rrcfixedq6" => {
                hardcoded_solver = rrc_fixed_q6;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "sphericalbot" => {
                hardcoded_solver = spherical_bot;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "yumifixedq3" => {
                hardcoded_solver = yumi_fixed_q3;
                general_solver = dummy_solver_general;
                is_hardcoded = true;
            }
            "sphericaltwoparallel" => {
                general_solver = spherical_two_parallel;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "sphericaltwointersecting" => {
                general_solver = spherical_two_intersecting;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "spherical" => {
                general_solver = spherical;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "threeparalleltwointersecting" => {
                general_solver = three_parallel_two_intersecting;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "threeparallel" => {
                general_solver = three_parallel;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "twoparallel" => {
                general_solver = two_parallel;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "twointersecting" => {
                general_solver = two_intersecting;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
            "gensixdof" => {
                general_solver = gen_six_dof;
                hardcoded_solver = dummy_solver_hardcoded;
                is_hardcoded = false;
            }
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
            kin: Kinematics::new(),
            kin_set: false,
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
    pub fn get_ik(
        &mut self,
        rot_matrix: [[f64; 3]; 3],
        trans_vec: [f64; 3],
    ) -> PyResult<Vec<([f64; 6], bool)>> {
        // Convert the input to the correct types
        let mut rotation = Matrix3::zeros();
        // Fill rotation matrix
        for i in 0..3 {
            for j in 0..3 {
                rotation[(i, j)] = rot_matrix[j][i];
            }
        }

        let translation = Vector3::from_row_slice(&trans_vec);
        let q: Vec<Vector6<f64>>;
        let is_ls: Vec<bool>;

        // Start a timer

        (q, is_ls) = call_ik_solver(self, rotation, translation);

        let mut ret_vals = Vec::new();
        for i in 0..q.len() {
            let mut q_vals = [0.0; 6];
            for j in 0..6 {
                q_vals[j] = q[i][j];
            }
            ret_vals.push((q_vals, is_ls[i]));
        }
        Ok(ret_vals)
    }

    // Get inverse kinematics and errors, sorted by error
    pub fn get_ik_sorted(
        &mut self,
        rot_matrix: [[f64; 3]; 3],
        trans_vec: [f64; 3],
    ) -> PyResult<Vec<([f64; 6], f64, bool)>> {
        let mut rotation = Matrix3::zeros();
        // Fill rotation matrix
        for i in 0..3 {
            for j in 0..3 {
                rotation[(i, j)] = rot_matrix[j][i];
            }
        }

        let translation = Vector3::from_row_slice(&trans_vec);

        // First, get all the solutions
        let solutions = call_ik_solver(self, rotation, translation);

        // Get the errors of each solution
        let mut solutions_with_errors: Vec<([f64; 6], f64, bool)> =
            Vec::<([f64; 6], f64, bool)>::new();
        for i in 0..solutions.0.len() {
            let q = solutions.0[i];
            let is_ls = solutions.1[i];

            // Calculate the error, assuming it's 0 if it's not a least squares solution
            let mut error: f64 = 0.0;
            if is_ls {
                error = calculate_ik_error(&self.kin, &rotation, &translation, &q);
            }

            // Convert q to a 6 element array
            let mut q_vals = [0.0; 6];
            for j in 0..6 {
                q_vals[j] = q[j];
            }
            solutions_with_errors.push((q_vals, error, is_ls));
        }

        // Sort the solutions by error, asecending
        solutions_with_errors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        Ok(solutions_with_errors)
    }

    pub fn forward_kinematics(&self, q: [f64; 6]) -> PyResult<([[f64; 3]; 3], [f64; 3])> {
        if !self.kin_set && !self.is_hardcoded {
            return Err(PyValueError::new_err(
                "Kinematics must be set before calling forward kinematics",
            ));
        } else if self.is_hardcoded {
            return Err(PyValueError::new_err(
                "Forward kinematics is not supported for hardcoded robots",
            ));
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
                r_vals[j][i] = r[(i, j)];
            }
            p_vals[i] = p[i];
        }
        Ok((r_vals, p_vals))
    }

    // Factory methods for each robot type
    #[staticmethod]
    fn irb6640() -> PyResult<Self> {
        Self::new("irb6640")
    }

    #[staticmethod]
    fn kuka_r800_fixed_q3() -> PyResult<Self> {
        Self::new("kukar800fixedq3")
    }

    #[staticmethod]
    fn ur5() -> PyResult<Self> {
        Self::new("ur5")
    }

    #[staticmethod]
    fn three_parallel_bot() -> PyResult<Self> {
        Self::new("threeparallelbot")
    }

    #[staticmethod]
    fn two_parallel_bot() -> PyResult<Self> {
        Self::new("twoparallelbot")
    }

    #[staticmethod]
    fn rrc_fixed_q6() -> PyResult<Self> {
        Self::new("rrcfixedq6")
    }

    #[staticmethod]
    fn spherical_bot() -> PyResult<Self> {
        Self::new("sphericalbot")
    }

    #[staticmethod]
    fn yumi_fixed_q3() -> PyResult<Self> {
        Self::new("yumifixedq3")
    }

    #[staticmethod]
    fn spherical_two_parallel() -> PyResult<Self> {
        Self::new("sphericaltwoparallel")
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

fn dummy_solver_hardcoded(_: &Matrix3<f64>, _: &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>) {
    panic!("This function should never be called");
}

fn dummy_solver_general(
    _: &Matrix3<f64>,
    _: &Vector3<f64>,
    _: &Kinematics<6, 7>,
) -> (Vec<Vector6<f64>>, Vec<bool>) {
    panic!("This function should never be called");
}

// Unexposed method to call the correct ik solver
fn call_ik_solver(
    robot: &mut Robot,
    rot_matrix: Matrix3<f64>,
    trans_vec: Vector3<f64>,
) -> (Vec<Vector6<f64>>, Vec<bool>) {
    if robot.is_hardcoded {
        (robot.hardcoded_solver)(&rot_matrix, &trans_vec)
    } else {
        // Make sure kinematics are set before calling the general solver
        if !robot.kin_set {
            panic!("Kinematics must be set before calling the general solver");
        }
        (robot.general_solver)(&rot_matrix, &trans_vec, &robot.kin)
    }
}

// Kinematics wrapper class
#[pyclass]
#[derive(Clone)]
struct KinematicsObject {
    pub kin: Kinematics<6, 7>,
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
    fn new(
        h_vals: [[f64; Self::H_ROWS]; Self::H_COLS],
        p_vals: [[f64; Self::P_ROWS]; Self::P_COLS],
    ) -> Self {
        let mut kin = Kinematics::<6, 7>::new();
        for i in 0..Self::H_ROWS {
            for j in 0..Self::H_COLS {
                kin.h[(i, j)] = h_vals[j][i];
            }
        }
        for i in 0..Self::P_ROWS {
            for j in 0..Self::P_COLS {
                kin.p[(i, j)] = p_vals[j][i];
            }
        }
        KinematicsObject { kin }
    }
}

/// A Python module implemented in Rust.
#[pymodule]
fn ik_geo(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<Robot>()?;
    m.add_class::<KinematicsObject>()?;
    Ok(())
}
