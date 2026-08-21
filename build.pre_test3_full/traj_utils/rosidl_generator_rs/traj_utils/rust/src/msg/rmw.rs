#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "traj_utils__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__traj_utils__msg__Bspline() -> *const std::ffi::c_void;
}

#[link(name = "traj_utils__rosidl_generator_c")]
extern "C" {
    fn traj_utils__msg__Bspline__init(msg: *mut Bspline) -> bool;
    fn traj_utils__msg__Bspline__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Bspline>, size: usize) -> bool;
    fn traj_utils__msg__Bspline__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Bspline>);
    fn traj_utils__msg__Bspline__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Bspline>, out_seq: *mut rosidl_runtime_rs::Sequence<Bspline>) -> bool;
}

// Corresponds to traj_utils__msg__Bspline
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Bspline {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drone_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub order: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub traj_id: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_time: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub knots: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos_pts: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_pts: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_dt: f64,

    /// A constrained trajectory is produced only after normal-clearance planning
    /// fails. Consumers must validate the marker and clearance before accepting it.
    pub constrained_clearance: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub required_clearance: f64,

}



impl Default for Bspline {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !traj_utils__msg__Bspline__init(&mut msg as *mut _) {
        panic!("Call to traj_utils__msg__Bspline__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Bspline {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__Bspline__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__Bspline__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__Bspline__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Bspline {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Bspline where Self: Sized {
  const TYPE_NAME: &'static str = "traj_utils/msg/Bspline";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__traj_utils__msg__Bspline() }
  }
}


#[link(name = "traj_utils__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__traj_utils__msg__DataDisp() -> *const std::ffi::c_void;
}

#[link(name = "traj_utils__rosidl_generator_c")]
extern "C" {
    fn traj_utils__msg__DataDisp__init(msg: *mut DataDisp) -> bool;
    fn traj_utils__msg__DataDisp__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DataDisp>, size: usize) -> bool;
    fn traj_utils__msg__DataDisp__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DataDisp>);
    fn traj_utils__msg__DataDisp__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DataDisp>, out_seq: *mut rosidl_runtime_rs::Sequence<DataDisp>) -> bool;
}

// Corresponds to traj_utils__msg__DataDisp
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DataDisp {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub a: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub b: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub c: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub d: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub e: f64,

}



impl Default for DataDisp {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !traj_utils__msg__DataDisp__init(&mut msg as *mut _) {
        panic!("Call to traj_utils__msg__DataDisp__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DataDisp {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__DataDisp__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__DataDisp__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__DataDisp__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DataDisp {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DataDisp where Self: Sized {
  const TYPE_NAME: &'static str = "traj_utils/msg/DataDisp";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__traj_utils__msg__DataDisp() }
  }
}


#[link(name = "traj_utils__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__traj_utils__msg__MultiBsplines() -> *const std::ffi::c_void;
}

#[link(name = "traj_utils__rosidl_generator_c")]
extern "C" {
    fn traj_utils__msg__MultiBsplines__init(msg: *mut MultiBsplines) -> bool;
    fn traj_utils__msg__MultiBsplines__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MultiBsplines>, size: usize) -> bool;
    fn traj_utils__msg__MultiBsplines__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MultiBsplines>);
    fn traj_utils__msg__MultiBsplines__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MultiBsplines>, out_seq: *mut rosidl_runtime_rs::Sequence<MultiBsplines>) -> bool;
}

// Corresponds to traj_utils__msg__MultiBsplines
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiBsplines {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drone_id_from: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub traj: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Bspline>,

}



impl Default for MultiBsplines {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !traj_utils__msg__MultiBsplines__init(&mut msg as *mut _) {
        panic!("Call to traj_utils__msg__MultiBsplines__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MultiBsplines {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__MultiBsplines__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__MultiBsplines__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { traj_utils__msg__MultiBsplines__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MultiBsplines {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MultiBsplines where Self: Sized {
  const TYPE_NAME: &'static str = "traj_utils/msg/MultiBsplines";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__traj_utils__msg__MultiBsplines() }
  }
}


