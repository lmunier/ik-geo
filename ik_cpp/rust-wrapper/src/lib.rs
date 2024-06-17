use linear_subproblem_solutions_rust::inverse_kinematics::auxiliary::Kinematics;
use linear_subproblem_solutions_rust::inverse_kinematics::hardcoded::*;
use linear_subproblem_solutions_rust::inverse_kinematics::{
    gen_six_dof, spherical, spherical_two_intersecting, spherical_two_parallel, three_parallel,
    three_parallel_two_intersecting, two_intersecting, two_parallel,
};

use linear_subproblem_solutions_rust::nalgebra::{Matrix3, Vector3, Vector6};



// Create a class for the robot
// Not using #[repr(C)] because it messes with the bool types
pub struct _RustRobot {
    // Function pointer to the correct ik solver function for setup
    is_hardcoded: bool,
    kin_set: bool,
    hardcoded_solver: fn(&Matrix3<f64>, &Vector3<f64>) -> (Vec<Vector6<f64>>, Vec<bool>),
    general_solver:
        fn(&Matrix3<f64>, &Vector3<f64>, &Kinematics<6, 7>) -> (Vec<Vector6<f64>>, Vec<bool>),
    kin: Kinematics<6, 7>,    
}

impl _RustRobot {
    // Create a new robot
    #[no_mangle]
    fn new(robot_type: &str) -> Self {
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
                panic!("Invalid robot type, must be one of:\n
                       Irb6640, KukaR800FixedQ3, Ur5, ThreeParallelBot, TwoParallelBot, RrcFixedQ6, SphericalBot, YumiFixedQ3, \n
                       SphericalTwoParallel, SphericalTwoIntersecting, Spherical, ThreeParallelTwoIntersecting, ThreeParallel, TwoParallel, TwoIntersecting, GenSixDof");
            }
        }

        _RustRobot {
            hardcoded_solver,
            general_solver,
            is_hardcoded,
            kin_set: false,
            kin: Kinematics::<6, 7>::new(),
        }
    }

    // Set the kinematics for the robot
    fn set_kinematics(&mut self, kin: KinematicsObject) -> () {
        self.kin = kin.kin;
        self.kin_set = true;
    }

    // Get the inverse kinematics for the robot
    // 2d array for the rotation matrix (row major), 3 values for translation vector
    fn get_ik(
        &mut self,
        rot_matrix: [[f64; 3]; 3],
        trans_matrix: [f64; 3],
    ) -> (Vec<Vector6<f64>>, Vec<bool>) {
        // Convert the input to the correct types
        let mut rotation = Matrix3::zeros();
        // Fill rotation matrix
        for i in 0..3 {
            for j in 0..3 {
                rotation[(i, j)] = rot_matrix[j][i];
            }
        }

        let translation = Vector3::from_row_slice(&trans_matrix);
        let q: Vec<Vector6<f64>>;
        let is_ls: Vec<bool>;
        if self.is_hardcoded {
            (self.hardcoded_solver)(&rotation, &translation)
        } else {
            // Make sure kinematics are set before calling the general solver
            if !self.kin_set {
                panic!(
                    "Kinematics must be set before calling the general solver",
                );
            }
            (self.general_solver)(&rotation, &translation, &self.kin)
        }
        
    }

    fn forward_kinematics(&self, q: [f64; 6]) -> ([[f64; 3]; 3], [f64; 3]) {
        if !self.kin_set && !self.is_hardcoded {
            panic!(
                "Kinematics must be set before calling forward kinematics",
            );
        } else if self.is_hardcoded {
            panic!(
                "Forward kinematics not implemented for hardcoded solvers",
            );
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
        (r_vals, p_vals)
    }

    // Factory methods for each robot type
    
    fn irb6640() -> Self {
        Self::new("irb6640")
    }

    
    fn kuka_r800_fixed_q3() -> Self {
        Self::new("kukar800fixedq3")
    }

    
    fn ur5() -> Self {
        Self::new("ur5")
    }

    
    fn three_parallel_bot() -> Self {
        Self::new("threeparallelbot")
    }

    
    fn two_parallel_bot() -> Self {
        Self::new("twoparallelbot")
    }

    
    fn rrc_fixed_q6() -> Self {
        Self::new("rrcfixedq6")
    }

    
    fn spherical_bot() -> Self {
        Self::new("sphericalbot")
    }

    
    fn yumi_fixed_q3() -> Self {
        Self::new("yumifixedq3")
    }

    
    fn spherical_two_parallel() -> Self {
        Self::new("sphericaltwoparallel")
    }

    
    fn spherical_two_intersecting() -> Self {
        Self::new("sphericaltwointersecting")
    }

    
    fn spherical() -> Self {
        Self::new("spherical")
    }

    
    fn three_parallel_two_intersecting() -> Self {
        Self::new("threeparalleltwointersecting")
    }

    
    fn three_parallel() -> Self {
        Self::new("threeparallel")
    }

    
    fn two_parallel() -> Self {
        Self::new("twoparallel")
    }

    
    fn two_intersecting() -> Self {
        Self::new("twointersecting")
    }

    
    fn gen_six_dof() -> Self {
        Self::new("gensixdof")
    }
}



// Referenced robot functions: ----------------------------------
// Implement functions for the robot class, can't use an impl block for this
#[no_mangle]
pub extern "C" fn new_robot(robot_type_in : *const u8, robot_type_len : usize) -> *mut _RustRobot {
    // Get t
    let robot_type = unsafe {
        assert!(!robot_type_in.is_null());
        std::str::from_utf8(std::slice::from_raw_parts(robot_type_in, robot_type_len))
            .expect("Invalid robot type")
    };
    Box::into_raw(Box::new(_RustRobot::new(robot_type)))
}

// Set the kinematics for the robot
#[no_mangle]
pub extern "C" fn set_robot_kinematics(robot_in: *mut _RustRobot, h_vals: *const f64, p_vals: *const f64) {
    let mut robot = unsafe {
        Box::from_raw(robot_in)
    };

    // Convert the h_vals and p_vals to 2d arrays
    let mut h_mat = [[0.0; KinematicsObject::H_ROWS]; KinematicsObject::H_COLS];
    let mut p_mat = [[0.0; KinematicsObject::P_ROWS]; KinematicsObject::P_COLS];
    for i in 0..KinematicsObject::H_ROWS {
        for j in 0..KinematicsObject::H_COLS {
            h_mat[j][i] = unsafe { *h_vals.add(j * KinematicsObject::H_ROWS + i) };
        }
    }
    for i in 0..KinematicsObject::P_ROWS {
        for j in 0..KinematicsObject::P_COLS {
            p_mat[j][i] = unsafe { *p_vals.add(j * KinematicsObject::P_ROWS + i) };
        }
    }
    let kin = KinematicsObject::new(h_mat, p_mat);
    robot.set_kinematics(kin);
    // Prevent double free
    std::mem::forget(robot);
}


#[no_mangle]
// return_q is a pointer to an array of 6 element vectors of f64
// is_ls is a pointer to an array of bools that will be set to true if the solution is a least squares solution
pub extern "C" fn get_robot_ik(robot_in: *mut _RustRobot, rot_matrix_ptr: *const f64, t_vec_ptr: *const f64, return_q: *mut *mut f64, return_is_ls: *mut *mut bool, return_len: *mut usize) {
    let mut robot = unsafe {
        Box::from_raw(robot_in)
    };
    let rot_matrix = unsafe {
        let mut rot_matrix = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                rot_matrix[j][i] = *rot_matrix_ptr.add(i * 3 + j);
            }
        }
        rot_matrix
    };
    let t_vec = unsafe {
        let mut t_vec = [0.0; 3];
        for i in 0..3 {
            t_vec[i] = *t_vec_ptr.add(i);
        }
        t_vec
    };
    let (q, is_ls) = robot.get_ik(rot_matrix, t_vec);
    // Prevent double free
    std::mem::forget(robot);

    // Output into the return pointers
    let mut q_out = Vec::with_capacity(q.len() * 6);
    let mut is_ls_out = Vec::with_capacity(is_ls.len());
    for i in 0..q.len() {
        for j in 0..6 {
            q_out.push(q[i][j]);
        }
        is_ls_out.push(is_ls[i]);
    }
    let q_ptr = q_out.as_mut_ptr();
    let is_ls_ptr = is_ls_out.as_mut_ptr();
    let len = q.len();
    std::mem::forget(q_out);
    std::mem::forget(is_ls_out);
    unsafe {
        *return_q = q_ptr;
        *return_is_ls = is_ls_ptr;
        *return_len = len;
    }
}

#[no_mangle]
pub extern "C" fn forward_kinematics(robot_in: *mut _RustRobot, q_in: *const f64, rot_matrix_out : *mut f64, t_vec_out : *mut f64) {
    let robot = unsafe {
        Box::from_raw(robot_in)
    };
    let mut q = [0.0; 6];
    for i in 0..6 {
        q[i] = unsafe { *q_in.add(i) };
    }
    let (r, p) = robot.forward_kinematics(q);
    for i in 0..3 {
        for j in 0..3 {
            unsafe {
                *rot_matrix_out.add(i * 3 + j) = r[j][i];
            }
        }
        unsafe {
            *t_vec_out.add(i) = p[i];
        }
    }
    // Prevent double free
    std::mem::forget(robot);
}

// Factory methods for each robot type

#[no_mangle]
pub extern "C" fn new_irb6640() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::irb6640()))
}

#[no_mangle]
pub extern "C" fn new_kuka_r800_fixed_q3() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::kuka_r800_fixed_q3()))
}

#[no_mangle]
pub extern "C" fn new_ur5() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::ur5()))
}

#[no_mangle]
pub extern "C" fn new_three_parallel_bot() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::three_parallel_bot()))
}

#[no_mangle]
pub extern "C" fn new_two_parallel_bot() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::two_parallel_bot()))
}

#[no_mangle]
pub extern "C" fn new_rrc_fixed_q6() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::rrc_fixed_q6()))
}

#[no_mangle]
pub extern "C" fn new_spherical_bot() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::spherical_bot()))
}

#[no_mangle]
pub extern "C" fn new_yumi_fixed_q3() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::yumi_fixed_q3()))
}

#[no_mangle]
pub extern "C" fn new_spherical_two_parallel() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::spherical_two_parallel()))
}

#[no_mangle]
pub extern "C" fn new_spherical_two_intersecting() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::spherical_two_intersecting()))
}

#[no_mangle]
pub extern "C" fn new_spherical() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::spherical()))
}

#[no_mangle]
pub extern "C" fn new_three_parallel_two_intersecting() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::three_parallel_two_intersecting()))
}

#[no_mangle]
pub extern "C" fn new_three_parallel() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::three_parallel()))
}

#[no_mangle]
pub extern "C" fn new_two_parallel() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::two_parallel()))
}

#[no_mangle]
pub extern "C" fn new_two_intersecting() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::two_intersecting()))
}

#[no_mangle]
pub extern "C" fn new_gen_six_dof() -> *mut _RustRobot {
    Box::into_raw(Box::new(_RustRobot::gen_six_dof()))
}

// ----------------------------------------------------------------



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

// Kinematics wrapper class
struct KinematicsObject {
    pub kin: Kinematics<6, 7>,
}

// Implement the Kinematics wrapper class
impl KinematicsObject {
    const H_ROWS: usize = 3;
    const P_ROWS: usize = 3;
    const H_COLS: usize = 6;
    const P_COLS: usize = 7;

    // Create a new Kinematics object from
    // h_vals: array of vals in the h matrix, column major
    // p_vals: array of vals in the p matrix, column major
    // Basically, both are of format [[1,0,0], [0,1,0]...] where these are the vectors
    
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

