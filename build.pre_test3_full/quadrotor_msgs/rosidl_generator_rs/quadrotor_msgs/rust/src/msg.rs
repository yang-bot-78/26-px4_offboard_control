#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to quadrotor_msgs__msg__AuxCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AuxCommand::default())
  }
}

impl rosidl_runtime_rs::Message for AuxCommand {
  type RmwMsg = super::msg::rmw::AuxCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_yaw: msg.current_yaw,
        kf_correction: msg.kf_correction,
        angle_corrections: msg.angle_corrections,
        enable_motors: msg.enable_motors,
        use_external_yaw: msg.use_external_yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      current_yaw: msg.current_yaw,
      kf_correction: msg.kf_correction,
        angle_corrections: msg.angle_corrections,
      enable_motors: msg.enable_motors,
      use_external_yaw: msg.use_external_yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_yaw: msg.current_yaw,
      kf_correction: msg.kf_correction,
      angle_corrections: msg.angle_corrections,
      enable_motors: msg.enable_motors,
      use_external_yaw: msg.use_external_yaw,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__Corrections

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Corrections::default())
  }
}

impl rosidl_runtime_rs::Message for Corrections {
  type RmwMsg = super::msg::rmw::Corrections;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        kf_correction: msg.kf_correction,
        angle_corrections: msg.angle_corrections,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      kf_correction: msg.kf_correction,
        angle_corrections: msg.angle_corrections,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      kf_correction: msg.kf_correction,
      angle_corrections: msg.angle_corrections,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__Gains

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Gains::default())
  }
}

impl rosidl_runtime_rs::Message for Gains {
  type RmwMsg = super::msg::rmw::Gains;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        kp: msg.kp,
        kd: msg.kd,
        kp_yaw: msg.kp_yaw,
        kd_yaw: msg.kd_yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      kp: msg.kp,
      kd: msg.kd,
      kp_yaw: msg.kp_yaw,
      kd_yaw: msg.kd_yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      kp: msg.kp,
      kd: msg.kd,
      kp_yaw: msg.kp_yaw,
      kd_yaw: msg.kd_yaw,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__LQRTrajectory

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LQRTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    pub debug_info: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LQRTrajectory::default())
  }
}

impl rosidl_runtime_rs::Message for LQRTrajectory {
  type RmwMsg = super::msg::rmw::LQRTrajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        trajectory_id: msg.trajectory_id,
        action: msg.action,
        r: msg.r,
        start_yaw: msg.start_yaw,
        final_yaw: msg.final_yaw,
        s0: msg.s0,
        ut: msg.ut,
        sf: msg.sf,
        t_f: msg.t_f,
        debug_info: msg.debug_info.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      trajectory_id: msg.trajectory_id,
      action: msg.action,
      r: msg.r,
      start_yaw: msg.start_yaw,
      final_yaw: msg.final_yaw,
        s0: msg.s0,
        ut: msg.ut,
        sf: msg.sf,
      t_f: msg.t_f,
        debug_info: msg.debug_info.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      trajectory_id: msg.trajectory_id,
      action: msg.action,
      r: msg.r,
      start_yaw: msg.start_yaw,
      final_yaw: msg.final_yaw,
      s0: msg.s0,
      ut: msg.ut,
      sf: msg.sf,
      t_f: msg.t_f,
      debug_info: msg.debug_info.to_string(),
    }
  }
}


// Corresponds to quadrotor_msgs__msg__OutputData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OutputData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub loop_rate: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub voltage: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation: geometry_msgs::msg::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_velocity: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub linear_acceleration: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pressure_dheight: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pressure_height: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub magnetic_field: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub radio_channel: [u8; 8],

    /// uint8 motor_rpm
    pub seq: u8,

}



impl Default for OutputData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::OutputData::default())
  }
}

impl rosidl_runtime_rs::Message for OutputData {
  type RmwMsg = super::msg::rmw::OutputData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        loop_rate: msg.loop_rate,
        voltage: msg.voltage,
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Owned(msg.orientation)).into_owned(),
        angular_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.angular_velocity)).into_owned(),
        linear_acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.linear_acceleration)).into_owned(),
        pressure_dheight: msg.pressure_dheight,
        pressure_height: msg.pressure_height,
        magnetic_field: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.magnetic_field)).into_owned(),
        radio_channel: msg.radio_channel,
        seq: msg.seq,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      loop_rate: msg.loop_rate,
      voltage: msg.voltage,
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Borrowed(&msg.orientation)).into_owned(),
        angular_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.angular_velocity)).into_owned(),
        linear_acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.linear_acceleration)).into_owned(),
      pressure_dheight: msg.pressure_dheight,
      pressure_height: msg.pressure_height,
        magnetic_field: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.magnetic_field)).into_owned(),
        radio_channel: msg.radio_channel,
      seq: msg.seq,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      loop_rate: msg.loop_rate,
      voltage: msg.voltage,
      orientation: geometry_msgs::msg::Quaternion::from_rmw_message(msg.orientation),
      angular_velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.angular_velocity),
      linear_acceleration: geometry_msgs::msg::Vector3::from_rmw_message(msg.linear_acceleration),
      pressure_dheight: msg.pressure_dheight,
      pressure_height: msg.pressure_height,
      magnetic_field: geometry_msgs::msg::Vector3::from_rmw_message(msg.magnetic_field),
      radio_channel: msg.radio_channel,
      seq: msg.seq,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__PositionCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PositionCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration: geometry_msgs::msg::Vector3,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PositionCommand::default())
  }
}

impl rosidl_runtime_rs::Message for PositionCommand {
  type RmwMsg = super::msg::rmw::PositionCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.acceleration)).into_owned(),
        yaw: msg.yaw,
        yaw_dot: msg.yaw_dot,
        kx: msg.kx,
        kv: msg.kv,
        trajectory_id: msg.trajectory_id,
        trajectory_flag: msg.trajectory_flag,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.acceleration)).into_owned(),
      yaw: msg.yaw,
      yaw_dot: msg.yaw_dot,
        kx: msg.kx,
        kv: msg.kv,
      trajectory_id: msg.trajectory_id,
      trajectory_flag: msg.trajectory_flag,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      acceleration: geometry_msgs::msg::Vector3::from_rmw_message(msg.acceleration),
      yaw: msg.yaw,
      yaw_dot: msg.yaw_dot,
      kx: msg.kx,
      kv: msg.kv,
      trajectory_id: msg.trajectory_id,
      trajectory_flag: msg.trajectory_flag,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__PPROutputData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PPROutputData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PPROutputData::default())
  }
}

impl rosidl_runtime_rs::Message for PPROutputData {
  type RmwMsg = super::msg::rmw::PPROutputData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        quad_time: msg.quad_time,
        des_thrust: msg.des_thrust,
        des_roll: msg.des_roll,
        des_pitch: msg.des_pitch,
        des_yaw: msg.des_yaw,
        est_roll: msg.est_roll,
        est_pitch: msg.est_pitch,
        est_yaw: msg.est_yaw,
        est_angvel_x: msg.est_angvel_x,
        est_angvel_y: msg.est_angvel_y,
        est_angvel_z: msg.est_angvel_z,
        est_acc_x: msg.est_acc_x,
        est_acc_y: msg.est_acc_y,
        est_acc_z: msg.est_acc_z,
        pwm: msg.pwm,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      quad_time: msg.quad_time,
      des_thrust: msg.des_thrust,
      des_roll: msg.des_roll,
      des_pitch: msg.des_pitch,
      des_yaw: msg.des_yaw,
      est_roll: msg.est_roll,
      est_pitch: msg.est_pitch,
      est_yaw: msg.est_yaw,
      est_angvel_x: msg.est_angvel_x,
      est_angvel_y: msg.est_angvel_y,
      est_angvel_z: msg.est_angvel_z,
      est_acc_x: msg.est_acc_x,
      est_acc_y: msg.est_acc_y,
      est_acc_z: msg.est_acc_z,
        pwm: msg.pwm,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      quad_time: msg.quad_time,
      des_thrust: msg.des_thrust,
      des_roll: msg.des_roll,
      des_pitch: msg.des_pitch,
      des_yaw: msg.des_yaw,
      est_roll: msg.est_roll,
      est_pitch: msg.est_pitch,
      est_yaw: msg.est_yaw,
      est_angvel_x: msg.est_angvel_x,
      est_angvel_y: msg.est_angvel_y,
      est_angvel_z: msg.est_angvel_z,
      est_acc_x: msg.est_acc_x,
      est_acc_y: msg.est_acc_y,
      est_acc_z: msg.est_acc_z,
      pwm: msg.pwm,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__Serial
/// Note: These constants need to be kept in sync with the types
/// defined in include/quadrotor_msgs/comm_types.h

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Serial {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub channel: u8,

    /// One of the types listed above
    pub type_: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: Vec<u8>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Serial::default())
  }
}

impl rosidl_runtime_rs::Message for Serial {
  type RmwMsg = super::msg::rmw::Serial;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        channel: msg.channel,
        type_: msg.type_,
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      channel: msg.channel,
      type_: msg.type_,
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      channel: msg.channel,
      type_: msg.type_,
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to quadrotor_msgs__msg__SO3Command

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SO3Command {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub force: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation: geometry_msgs::msg::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kr: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub kom: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub aux: super::msg::AuxCommand,

}



impl Default for SO3Command {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SO3Command::default())
  }
}

impl rosidl_runtime_rs::Message for SO3Command {
  type RmwMsg = super::msg::rmw::SO3Command;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        force: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.force)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Owned(msg.orientation)).into_owned(),
        kr: msg.kr,
        kom: msg.kom,
        aux: super::msg::AuxCommand::into_rmw_message(std::borrow::Cow::Owned(msg.aux)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        force: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.force)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Borrowed(&msg.orientation)).into_owned(),
        kr: msg.kr,
        kom: msg.kom,
        aux: super::msg::AuxCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.aux)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      force: geometry_msgs::msg::Vector3::from_rmw_message(msg.force),
      orientation: geometry_msgs::msg::Quaternion::from_rmw_message(msg.orientation),
      kr: msg.kr,
      kom: msg.kom,
      aux: super::msg::AuxCommand::from_rmw_message(msg.aux),
    }
  }
}


// Corresponds to quadrotor_msgs__msg__StatusData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StatusData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StatusData::default())
  }
}

impl rosidl_runtime_rs::Message for StatusData {
  type RmwMsg = super::msg::rmw::StatusData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        loop_rate: msg.loop_rate,
        voltage: msg.voltage,
        seq: msg.seq,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      loop_rate: msg.loop_rate,
      voltage: msg.voltage,
      seq: msg.seq,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      loop_rate: msg.loop_rate,
      voltage: msg.voltage,
      seq: msg.seq,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__TRPYCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TRPYCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    pub aux: super::msg::AuxCommand,

}



impl Default for TRPYCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TRPYCommand::default())
  }
}

impl rosidl_runtime_rs::Message for TRPYCommand {
  type RmwMsg = super::msg::rmw::TRPYCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        thrust: msg.thrust,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
        aux: super::msg::AuxCommand::into_rmw_message(std::borrow::Cow::Owned(msg.aux)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      thrust: msg.thrust,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
        aux: super::msg::AuxCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.aux)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      thrust: msg.thrust,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      aux: super::msg::AuxCommand::from_rmw_message(msg.aux),
    }
  }
}


// Corresponds to quadrotor_msgs__msg__Odometry

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Odometry {

    // This member is not documented.
    #[allow(missing_docs)]
    pub curodom: nav_msgs::msg::Odometry,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kfodom: nav_msgs::msg::Odometry,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Odometry::default())
  }
}

impl rosidl_runtime_rs::Message for Odometry {
  type RmwMsg = super::msg::rmw::Odometry;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        curodom: nav_msgs::msg::Odometry::into_rmw_message(std::borrow::Cow::Owned(msg.curodom)).into_owned(),
        kfodom: nav_msgs::msg::Odometry::into_rmw_message(std::borrow::Cow::Owned(msg.kfodom)).into_owned(),
        kfid: msg.kfid,
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        curodom: nav_msgs::msg::Odometry::into_rmw_message(std::borrow::Cow::Borrowed(&msg.curodom)).into_owned(),
        kfodom: nav_msgs::msg::Odometry::into_rmw_message(std::borrow::Cow::Borrowed(&msg.kfodom)).into_owned(),
      kfid: msg.kfid,
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      curodom: nav_msgs::msg::Odometry::from_rmw_message(msg.curodom),
      kfodom: nav_msgs::msg::Odometry::from_rmw_message(msg.kfodom),
      kfid: msg.kfid,
      status: msg.status,
    }
  }
}


// Corresponds to quadrotor_msgs__msg__PolynomialTrajectory

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PolynomialTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    pub coef_x: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_y: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_z: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mag_coeff: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub order: Vec<u32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub debug_info: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PolynomialTrajectory::default())
  }
}

impl rosidl_runtime_rs::Message for PolynomialTrajectory {
  type RmwMsg = super::msg::rmw::PolynomialTrajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        trajectory_id: msg.trajectory_id,
        action: msg.action,
        num_order: msg.num_order,
        num_segment: msg.num_segment,
        start_yaw: msg.start_yaw,
        final_yaw: msg.final_yaw,
        coef_x: msg.coef_x.into(),
        coef_y: msg.coef_y.into(),
        coef_z: msg.coef_z.into(),
        time: msg.time.into(),
        mag_coeff: msg.mag_coeff,
        order: msg.order.into(),
        debug_info: msg.debug_info.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      trajectory_id: msg.trajectory_id,
      action: msg.action,
      num_order: msg.num_order,
      num_segment: msg.num_segment,
      start_yaw: msg.start_yaw,
      final_yaw: msg.final_yaw,
        coef_x: msg.coef_x.as_slice().into(),
        coef_y: msg.coef_y.as_slice().into(),
        coef_z: msg.coef_z.as_slice().into(),
        time: msg.time.as_slice().into(),
      mag_coeff: msg.mag_coeff,
        order: msg.order.as_slice().into(),
        debug_info: msg.debug_info.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      trajectory_id: msg.trajectory_id,
      action: msg.action,
      num_order: msg.num_order,
      num_segment: msg.num_segment,
      start_yaw: msg.start_yaw,
      final_yaw: msg.final_yaw,
      coef_x: msg.coef_x
          .into_iter()
          .collect(),
      coef_y: msg.coef_y
          .into_iter()
          .collect(),
      coef_z: msg.coef_z
          .into_iter()
          .collect(),
      time: msg.time
          .into_iter()
          .collect(),
      mag_coeff: msg.mag_coeff,
      order: msg.order
          .into_iter()
          .collect(),
      debug_info: msg.debug_info.to_string(),
    }
  }
}


