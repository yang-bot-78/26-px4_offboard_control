#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to race_msgs__msg__FlightAltitudeReference

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FlightAltitudeReference {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FlightAltitudeReference::default())
  }
}

impl rosidl_runtime_rs::Message for FlightAltitudeReference {
  type RmwMsg = super::msg::rmw::FlightAltitudeReference;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        flight_id: msg.flight_id,
        valid: msg.valid,
        target_agl_m: msg.target_agl_m,
        min_agl_m: msg.min_agl_m,
        max_agl_m: msg.max_agl_m,
        ground_z_local_ned: msg.ground_z_local_ned,
        target_z_local_ned: msg.target_z_local_ned,
        ground_z_map: msg.ground_z_map,
        target_z_map: msg.target_z_map,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      flight_id: msg.flight_id,
      valid: msg.valid,
      target_agl_m: msg.target_agl_m,
      min_agl_m: msg.min_agl_m,
      max_agl_m: msg.max_agl_m,
      ground_z_local_ned: msg.ground_z_local_ned,
      target_z_local_ned: msg.target_z_local_ned,
      ground_z_map: msg.ground_z_map,
      target_z_map: msg.target_z_map,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      flight_id: msg.flight_id,
      valid: msg.valid,
      target_agl_m: msg.target_agl_m,
      min_agl_m: msg.min_agl_m,
      max_agl_m: msg.max_agl_m,
      ground_z_local_ned: msg.ground_z_local_ned,
      target_z_local_ned: msg.target_z_local_ned,
      ground_z_map: msg.ground_z_map,
      target_z_map: msg.target_z_map,
    }
  }
}


// Corresponds to race_msgs__msg__GlobalPlannerStatus
/// Structured final-goal contract from the global planner to Offboard.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GlobalPlannerStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    pub mode: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,

}



impl Default for GlobalPlannerStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GlobalPlannerStatus::default())
  }
}

impl rosidl_runtime_rs::Message for GlobalPlannerStatus {
  type RmwMsg = super::msg::rmw::GlobalPlannerStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        global_goal_id: msg.global_goal_id,
        global_path_id: msg.global_path_id,
        local_goal_seq: msg.local_goal_seq,
        goal_active: msg.goal_active,
        final_goal_reached: msg.final_goal_reached,
        distance_to_final: msg.distance_to_final,
        horizontal_speed: msg.horizontal_speed,
        mode: msg.mode.as_str().into(),
        reason: msg.reason.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      global_goal_id: msg.global_goal_id,
      global_path_id: msg.global_path_id,
      local_goal_seq: msg.local_goal_seq,
      goal_active: msg.goal_active,
      final_goal_reached: msg.final_goal_reached,
      distance_to_final: msg.distance_to_final,
      horizontal_speed: msg.horizontal_speed,
        mode: msg.mode.as_str().into(),
        reason: msg.reason.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      global_goal_id: msg.global_goal_id,
      global_path_id: msg.global_path_id,
      local_goal_seq: msg.local_goal_seq,
      goal_active: msg.goal_active,
      final_goal_reached: msg.final_goal_reached,
      distance_to_final: msg.distance_to_final,
      horizontal_speed: msg.horizontal_speed,
      mode: msg.mode.to_string(),
      reason: msg.reason.to_string(),
    }
  }
}


// Corresponds to race_msgs__msg__LocalPathReference
/// Super's exact, safety-checked polyline from the current path projection to
/// the rolling local goal.  The two counters bind this reference to one global
/// path transaction and one local-goal update.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LocalPathReference {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub global_path_id: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub local_goal_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub local_goal: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: Vec<geometry_msgs::msg::Point>,

    /// Safety-checked continuation of the same global polyline after local_goal.
    /// Normal EGO planning uses only points; recovery uses continuation_points to
    /// choose a rejoin target without extrapolating a single yaw through a corner.
    pub continuation_points: Vec<geometry_msgs::msg::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub arc_length: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub minimum_clearance: f64,

}



impl Default for LocalPathReference {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LocalPathReference::default())
  }
}

impl rosidl_runtime_rs::Message for LocalPathReference {
  type RmwMsg = super::msg::rmw::LocalPathReference;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        global_path_id: msg.global_path_id,
        local_goal_seq: msg.local_goal_seq,
        local_goal: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.local_goal)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        continuation_points: msg.continuation_points
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        arc_length: msg.arc_length,
        minimum_clearance: msg.minimum_clearance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      global_path_id: msg.global_path_id,
      local_goal_seq: msg.local_goal_seq,
        local_goal: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.local_goal)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        continuation_points: msg.continuation_points
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      arc_length: msg.arc_length,
      minimum_clearance: msg.minimum_clearance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      global_path_id: msg.global_path_id,
      local_goal_seq: msg.local_goal_seq,
      local_goal: geometry_msgs::msg::Point::from_rmw_message(msg.local_goal),
      points: msg.points
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
      continuation_points: msg.continuation_points
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
      arc_length: msg.arc_length,
      minimum_clearance: msg.minimum_clearance,
    }
  }
}


// Corresponds to race_msgs__msg__NavigationSetpoint
/// Planner-to-controller contract. All values use header.frame_id.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigationSetpoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,

    /// Optional path-tangent velocity feed-forward.
    pub velocity_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::Vector3,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavigationSetpoint::default())
  }
}

impl rosidl_runtime_rs::Message for NavigationSetpoint {
  type RmwMsg = super::msg::rmw::NavigationSetpoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        velocity_valid: msg.velocity_valid,
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        yaw: msg.yaw,
        yaw_rate_valid: msg.yaw_rate_valid,
        yaw_rate: msg.yaw_rate,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
      velocity_valid: msg.velocity_valid,
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
      yaw: msg.yaw,
      yaw_rate_valid: msg.yaw_rate_valid,
      yaw_rate: msg.yaw_rate,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      velocity_valid: msg.velocity_valid,
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      yaw: msg.yaw,
      yaw_rate_valid: msg.yaw_rate_valid,
      yaw_rate: msg.yaw_rate,
    }
  }
}


