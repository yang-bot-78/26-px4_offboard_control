#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__AuxCommand() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__AuxCommand__init(msg: *mut AuxCommand) -> bool;
    fn quadrotor_msgs__msg__AuxCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AuxCommand>, size: usize) -> bool;
    fn quadrotor_msgs__msg__AuxCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AuxCommand>);
    fn quadrotor_msgs__msg__AuxCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AuxCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<AuxCommand>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__AuxCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AuxCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kf_correction: f64,

    /// Trims for roll, pitch
    pub angle_corrections: [f64; 2],


    // This member is not documented.
    #[allow(missing_docs)]
    pub enable_motors: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub use_external_yaw: bool,

}



impl Default for AuxCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__AuxCommand__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__AuxCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AuxCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__AuxCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__AuxCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__AuxCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AuxCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AuxCommand where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/AuxCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__AuxCommand() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Corrections() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__Corrections__init(msg: *mut Corrections) -> bool;
    fn quadrotor_msgs__msg__Corrections__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Corrections>, size: usize) -> bool;
    fn quadrotor_msgs__msg__Corrections__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Corrections>);
    fn quadrotor_msgs__msg__Corrections__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Corrections>, out_seq: *mut rosidl_runtime_rs::Sequence<Corrections>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__Corrections
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Corrections {

    // This member is not documented.
    #[allow(missing_docs)]
    pub kf_correction: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angle_corrections: [f64; 2],

}



impl Default for Corrections {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__Corrections__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__Corrections__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Corrections {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Corrections__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Corrections__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Corrections__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Corrections {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Corrections where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/Corrections";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Corrections() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Gains() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__Gains__init(msg: *mut Gains) -> bool;
    fn quadrotor_msgs__msg__Gains__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Gains>, size: usize) -> bool;
    fn quadrotor_msgs__msg__Gains__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Gains>);
    fn quadrotor_msgs__msg__Gains__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Gains>, out_seq: *mut rosidl_runtime_rs::Sequence<Gains>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__Gains
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Gains {

    // This member is not documented.
    #[allow(missing_docs)]
    pub kp: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kd: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kp_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kd_yaw: f64,

}



impl Default for Gains {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__Gains__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__Gains__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Gains {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Gains__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Gains__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Gains__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Gains {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Gains where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/Gains";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Gains() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__LQRTrajectory() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__LQRTrajectory__init(msg: *mut LQRTrajectory) -> bool;
    fn quadrotor_msgs__msg__LQRTrajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LQRTrajectory>, size: usize) -> bool;
    fn quadrotor_msgs__msg__LQRTrajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LQRTrajectory>);
    fn quadrotor_msgs__msg__LQRTrajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LQRTrajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<LQRTrajectory>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__LQRTrajectory
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LQRTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// the trajectory id, starts from "1".
    pub trajectory_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub action: u32,

    /// the weight coefficient of the control effort
    pub r: f64,

    /// the yaw command
    pub start_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_yaw: f64,

    /// the initial and final state
    pub s0: [f64; 6],


    // This member is not documented.
    #[allow(missing_docs)]
    pub ut: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub sf: [f64; 6],

    /// the optimal arrival time
    pub t_f: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub debug_info: rosidl_runtime_rs::String,

}

impl LQRTrajectory {
    /// the action command for trajectory server.
    pub const ACTION_ADD: u32 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_ABORT: u32 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_WARN_START: u32 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_WARN_FINAL: u32 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_WARN_IMPOSSIBLE: u32 = 5;

}


impl Default for LQRTrajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__LQRTrajectory__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__LQRTrajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LQRTrajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__LQRTrajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__LQRTrajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__LQRTrajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LQRTrajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LQRTrajectory where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/LQRTrajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__LQRTrajectory() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__OutputData() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__OutputData__init(msg: *mut OutputData) -> bool;
    fn quadrotor_msgs__msg__OutputData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OutputData>, size: usize) -> bool;
    fn quadrotor_msgs__msg__OutputData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OutputData>);
    fn quadrotor_msgs__msg__OutputData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OutputData>, out_seq: *mut rosidl_runtime_rs::Sequence<OutputData>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__OutputData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OutputData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub loop_rate: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub voltage: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation: geometry_msgs::msg::rmw::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub linear_acceleration: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pressure_dheight: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pressure_height: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub magnetic_field: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub radio_channel: [u8; 8],

    /// uint8 motor_rpm
    pub seq: u8,

}



impl Default for OutputData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__OutputData__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__OutputData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OutputData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__OutputData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__OutputData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__OutputData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OutputData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OutputData where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/OutputData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__OutputData() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__PositionCommand() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__PositionCommand__init(msg: *mut PositionCommand) -> bool;
    fn quadrotor_msgs__msg__PositionCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PositionCommand>, size: usize) -> bool;
    fn quadrotor_msgs__msg__PositionCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PositionCommand>);
    fn quadrotor_msgs__msg__PositionCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PositionCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<PositionCommand>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__PositionCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PositionCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_dot: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kx: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub kv: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub trajectory_id: u32,

    /// Its ID number will start from 1, allowing you comparing it with 0.
    pub trajectory_flag: u8,

}

impl PositionCommand {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTORY_STATUS_EMPTY: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTORY_STATUS_READY: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTORY_STATUS_COMPLETED: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTROY_STATUS_ABORT: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTORY_STATUS_ILLEGAL_START: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTORY_STATUS_ILLEGAL_FINAL: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAJECTORY_STATUS_IMPOSSIBLE: u8 = 7;

}


impl Default for PositionCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__PositionCommand__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__PositionCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PositionCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PositionCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PositionCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PositionCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PositionCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PositionCommand where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/PositionCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__PositionCommand() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__PPROutputData() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__PPROutputData__init(msg: *mut PPROutputData) -> bool;
    fn quadrotor_msgs__msg__PPROutputData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PPROutputData>, size: usize) -> bool;
    fn quadrotor_msgs__msg__PPROutputData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PPROutputData>);
    fn quadrotor_msgs__msg__PPROutputData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PPROutputData>, out_seq: *mut rosidl_runtime_rs::Sequence<PPROutputData>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__PPROutputData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PPROutputData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub quad_time: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub des_thrust: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub des_roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub des_pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub des_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_angvel_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_angvel_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_angvel_z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_acc_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_acc_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub est_acc_z: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pwm: [u16; 4],

}



impl Default for PPROutputData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__PPROutputData__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__PPROutputData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PPROutputData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PPROutputData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PPROutputData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PPROutputData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PPROutputData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PPROutputData where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/PPROutputData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__PPROutputData() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Serial() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__Serial__init(msg: *mut Serial) -> bool;
    fn quadrotor_msgs__msg__Serial__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Serial>, size: usize) -> bool;
    fn quadrotor_msgs__msg__Serial__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Serial>);
    fn quadrotor_msgs__msg__Serial__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Serial>, out_seq: *mut rosidl_runtime_rs::Sequence<Serial>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__Serial
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Note: These constants need to be kept in sync with the types
/// defined in include/quadrotor_msgs/comm_types.h

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Serial {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub channel: u8,

    /// One of the types listed above
    pub type_: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::Sequence<u8>,

}

impl Serial {
    /// 's' in base 10
    pub const SO3_CMD: u8 = 115;

    /// 'p' in base 10
    pub const TRPY_CMD: u8 = 112;

    /// 'c' in base 10
    pub const STATUS_DATA: u8 = 99;

    /// 'd' in base 10
    pub const OUTPUT_DATA: u8 = 100;

    /// 't' in base 10
    pub const PPR_OUTPUT_DATA: u8 = 116;

    /// 'g'
    pub const PPR_GAINS: u8 = 103;

}


impl Default for Serial {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__Serial__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__Serial__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Serial {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Serial__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Serial__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Serial__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Serial {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Serial where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/Serial";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Serial() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__SO3Command() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__SO3Command__init(msg: *mut SO3Command) -> bool;
    fn quadrotor_msgs__msg__SO3Command__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SO3Command>, size: usize) -> bool;
    fn quadrotor_msgs__msg__SO3Command__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SO3Command>);
    fn quadrotor_msgs__msg__SO3Command__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SO3Command>, out_seq: *mut rosidl_runtime_rs::Sequence<SO3Command>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__SO3Command
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SO3Command {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub force: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation: geometry_msgs::msg::rmw::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kr: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub kom: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub aux: super::super::msg::rmw::AuxCommand,

}



impl Default for SO3Command {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__SO3Command__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__SO3Command__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SO3Command {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__SO3Command__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__SO3Command__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__SO3Command__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SO3Command {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SO3Command where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/SO3Command";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__SO3Command() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__StatusData() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__StatusData__init(msg: *mut StatusData) -> bool;
    fn quadrotor_msgs__msg__StatusData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StatusData>, size: usize) -> bool;
    fn quadrotor_msgs__msg__StatusData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StatusData>);
    fn quadrotor_msgs__msg__StatusData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StatusData>, out_seq: *mut rosidl_runtime_rs::Sequence<StatusData>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__StatusData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StatusData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub loop_rate: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub voltage: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub seq: u8,

}



impl Default for StatusData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__StatusData__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__StatusData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StatusData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__StatusData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__StatusData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__StatusData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StatusData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StatusData where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/StatusData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__StatusData() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__TRPYCommand() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__TRPYCommand__init(msg: *mut TRPYCommand) -> bool;
    fn quadrotor_msgs__msg__TRPYCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TRPYCommand>, size: usize) -> bool;
    fn quadrotor_msgs__msg__TRPYCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TRPYCommand>);
    fn quadrotor_msgs__msg__TRPYCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TRPYCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<TRPYCommand>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__TRPYCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TRPYCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub thrust: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub aux: super::super::msg::rmw::AuxCommand,

}



impl Default for TRPYCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__TRPYCommand__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__TRPYCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TRPYCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__TRPYCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__TRPYCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__TRPYCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TRPYCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TRPYCommand where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/TRPYCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__TRPYCommand() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Odometry() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__Odometry__init(msg: *mut Odometry) -> bool;
    fn quadrotor_msgs__msg__Odometry__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Odometry>, size: usize) -> bool;
    fn quadrotor_msgs__msg__Odometry__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Odometry>);
    fn quadrotor_msgs__msg__Odometry__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Odometry>, out_seq: *mut rosidl_runtime_rs::Sequence<Odometry>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__Odometry
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Odometry {

    // This member is not documented.
    #[allow(missing_docs)]
    pub curodom: nav_msgs::msg::rmw::Odometry,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kfodom: nav_msgs::msg::rmw::Odometry,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kfid: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u8,

}

impl Odometry {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATUS_ODOM_VALID: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATUS_ODOM_INVALID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STATUS_ODOM_LOOPCLOSURE: u8 = 2;

}


impl Default for Odometry {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__Odometry__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__Odometry__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Odometry {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Odometry__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Odometry__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__Odometry__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Odometry {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Odometry where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/Odometry";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__Odometry() }
  }
}


#[link(name = "quadrotor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__PolynomialTrajectory() -> *const std::ffi::c_void;
}

#[link(name = "quadrotor_msgs__rosidl_generator_c")]
extern "C" {
    fn quadrotor_msgs__msg__PolynomialTrajectory__init(msg: *mut PolynomialTrajectory) -> bool;
    fn quadrotor_msgs__msg__PolynomialTrajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PolynomialTrajectory>, size: usize) -> bool;
    fn quadrotor_msgs__msg__PolynomialTrajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PolynomialTrajectory>);
    fn quadrotor_msgs__msg__PolynomialTrajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PolynomialTrajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<PolynomialTrajectory>) -> bool;
}

// Corresponds to quadrotor_msgs__msg__PolynomialTrajectory
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PolynomialTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// the trajectory id, starts from "1".
    pub trajectory_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub action: u32,

    /// the order of trajectory.
    pub num_order: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub num_segment: u32,

    /// the polynomial coecfficients of the trajectory.
    pub start_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_x: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_y: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_z: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mag_coeff: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub order: rosidl_runtime_rs::Sequence<u32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub debug_info: rosidl_runtime_rs::String,

}

impl PolynomialTrajectory {
    /// the action command for trajectory server.
    pub const ACTION_ADD: u32 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_ABORT: u32 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_WARN_START: u32 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_WARN_FINAL: u32 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTION_WARN_IMPOSSIBLE: u32 = 5;

}


impl Default for PolynomialTrajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !quadrotor_msgs__msg__PolynomialTrajectory__init(&mut msg as *mut _) {
        panic!("Call to quadrotor_msgs__msg__PolynomialTrajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PolynomialTrajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PolynomialTrajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PolynomialTrajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { quadrotor_msgs__msg__PolynomialTrajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PolynomialTrajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PolynomialTrajectory where Self: Sized {
  const TYPE_NAME: &'static str = "quadrotor_msgs/msg/PolynomialTrajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__quadrotor_msgs__msg__PolynomialTrajectory() }
  }
}


