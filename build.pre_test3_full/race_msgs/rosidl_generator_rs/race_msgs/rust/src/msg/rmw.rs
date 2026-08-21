#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "race_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__FlightAltitudeReference() -> *const std::ffi::c_void;
}

#[link(name = "race_msgs__rosidl_generator_c")]
extern "C" {
    fn race_msgs__msg__FlightAltitudeReference__init(msg: *mut FlightAltitudeReference) -> bool;
    fn race_msgs__msg__FlightAltitudeReference__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FlightAltitudeReference>, size: usize) -> bool;
    fn race_msgs__msg__FlightAltitudeReference__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FlightAltitudeReference>);
    fn race_msgs__msg__FlightAltitudeReference__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FlightAltitudeReference>, out_seq: *mut rosidl_runtime_rs::Sequence<FlightAltitudeReference>) -> bool;
}

// Corresponds to race_msgs__msg__FlightAltitudeReference
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FlightAltitudeReference {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Monotonic identifier assigned by the Offboard authority for each takeoff.
    pub flight_id: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,

    /// All configured vertical limits are heights above the locked takeoff ground.
    pub target_agl_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub min_agl_m: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_agl_m: f64,

    /// PX4 local position uses NED (positive down).
    pub ground_z_local_ned: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_z_local_ned: f64,

    /// Navigation and EGO use map ENU (positive up).
    pub ground_z_map: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_z_map: f64,

}



impl Default for FlightAltitudeReference {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !race_msgs__msg__FlightAltitudeReference__init(&mut msg as *mut _) {
        panic!("Call to race_msgs__msg__FlightAltitudeReference__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FlightAltitudeReference {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__FlightAltitudeReference__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__FlightAltitudeReference__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__FlightAltitudeReference__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FlightAltitudeReference {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FlightAltitudeReference where Self: Sized {
  const TYPE_NAME: &'static str = "race_msgs/msg/FlightAltitudeReference";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__FlightAltitudeReference() }
  }
}


#[link(name = "race_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__GlobalPlannerStatus() -> *const std::ffi::c_void;
}

#[link(name = "race_msgs__rosidl_generator_c")]
extern "C" {
    fn race_msgs__msg__GlobalPlannerStatus__init(msg: *mut GlobalPlannerStatus) -> bool;
    fn race_msgs__msg__GlobalPlannerStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GlobalPlannerStatus>, size: usize) -> bool;
    fn race_msgs__msg__GlobalPlannerStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GlobalPlannerStatus>);
    fn race_msgs__msg__GlobalPlannerStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GlobalPlannerStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<GlobalPlannerStatus>) -> bool;
}

// Corresponds to race_msgs__msg__GlobalPlannerStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Structured final-goal contract from the global planner to Offboard.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GlobalPlannerStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub global_goal_id: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub global_path_id: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub local_goal_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_active: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_goal_reached: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_to_final: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub horizontal_speed: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: rosidl_runtime_rs::String,

}



impl Default for GlobalPlannerStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !race_msgs__msg__GlobalPlannerStatus__init(&mut msg as *mut _) {
        panic!("Call to race_msgs__msg__GlobalPlannerStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GlobalPlannerStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__GlobalPlannerStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__GlobalPlannerStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__GlobalPlannerStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GlobalPlannerStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GlobalPlannerStatus where Self: Sized {
  const TYPE_NAME: &'static str = "race_msgs/msg/GlobalPlannerStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__GlobalPlannerStatus() }
  }
}


#[link(name = "race_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__LocalPathReference() -> *const std::ffi::c_void;
}

#[link(name = "race_msgs__rosidl_generator_c")]
extern "C" {
    fn race_msgs__msg__LocalPathReference__init(msg: *mut LocalPathReference) -> bool;
    fn race_msgs__msg__LocalPathReference__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LocalPathReference>, size: usize) -> bool;
    fn race_msgs__msg__LocalPathReference__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LocalPathReference>);
    fn race_msgs__msg__LocalPathReference__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LocalPathReference>, out_seq: *mut rosidl_runtime_rs::Sequence<LocalPathReference>) -> bool;
}

// Corresponds to race_msgs__msg__LocalPathReference
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Super's exact, safety-checked polyline from the current path projection to
/// the rolling local goal.  The two counters bind this reference to one global
/// path transaction and one local-goal update.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LocalPathReference {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub global_path_id: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub local_goal_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub local_goal: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,

    /// Safety-checked continuation of the same global polyline after local_goal.
    /// Normal EGO planning uses only points; recovery uses continuation_points to
    /// choose a rejoin target without extrapolating a single yaw through a corner.
    pub continuation_points: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub arc_length: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub minimum_clearance: f64,

}



impl Default for LocalPathReference {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !race_msgs__msg__LocalPathReference__init(&mut msg as *mut _) {
        panic!("Call to race_msgs__msg__LocalPathReference__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LocalPathReference {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__LocalPathReference__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__LocalPathReference__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__LocalPathReference__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LocalPathReference {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LocalPathReference where Self: Sized {
  const TYPE_NAME: &'static str = "race_msgs/msg/LocalPathReference";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__LocalPathReference() }
  }
}


#[link(name = "race_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__NavigationSetpoint() -> *const std::ffi::c_void;
}

#[link(name = "race_msgs__rosidl_generator_c")]
extern "C" {
    fn race_msgs__msg__NavigationSetpoint__init(msg: *mut NavigationSetpoint) -> bool;
    fn race_msgs__msg__NavigationSetpoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigationSetpoint>, size: usize) -> bool;
    fn race_msgs__msg__NavigationSetpoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigationSetpoint>);
    fn race_msgs__msg__NavigationSetpoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigationSetpoint>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigationSetpoint>) -> bool;
}

// Corresponds to race_msgs__msg__NavigationSetpoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Planner-to-controller contract. All values use header.frame_id.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigationSetpoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,

    /// Optional path-tangent velocity feed-forward.
    pub velocity_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Vector3,

    /// Heading and optional heading-rate feed-forward in radians and radians/second.
    pub yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_rate_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_rate: f64,

}



impl Default for NavigationSetpoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !race_msgs__msg__NavigationSetpoint__init(&mut msg as *mut _) {
        panic!("Call to race_msgs__msg__NavigationSetpoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigationSetpoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__NavigationSetpoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__NavigationSetpoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { race_msgs__msg__NavigationSetpoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigationSetpoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigationSetpoint where Self: Sized {
  const TYPE_NAME: &'static str = "race_msgs/msg/NavigationSetpoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__race_msgs__msg__NavigationSetpoint() }
  }
}


