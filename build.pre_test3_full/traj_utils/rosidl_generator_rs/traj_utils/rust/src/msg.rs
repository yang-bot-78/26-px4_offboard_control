#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to traj_utils__msg__Bspline

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub start_time: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub knots: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos_pts: Vec<geometry_msgs::msg::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_pts: Vec<f64>,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Bspline::default())
  }
}

impl rosidl_runtime_rs::Message for Bspline {
  type RmwMsg = super::msg::rmw::Bspline;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drone_id: msg.drone_id,
        order: msg.order,
        traj_id: msg.traj_id,
        start_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.start_time)).into_owned(),
        knots: msg.knots.into(),
        pos_pts: msg.pos_pts
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        yaw_pts: msg.yaw_pts.into(),
        yaw_dt: msg.yaw_dt,
        constrained_clearance: msg.constrained_clearance,
        required_clearance: msg.required_clearance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      drone_id: msg.drone_id,
      order: msg.order,
      traj_id: msg.traj_id,
        start_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start_time)).into_owned(),
        knots: msg.knots.as_slice().into(),
        pos_pts: msg.pos_pts
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        yaw_pts: msg.yaw_pts.as_slice().into(),
      yaw_dt: msg.yaw_dt,
      constrained_clearance: msg.constrained_clearance,
      required_clearance: msg.required_clearance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      drone_id: msg.drone_id,
      order: msg.order,
      traj_id: msg.traj_id,
      start_time: builtin_interfaces::msg::Time::from_rmw_message(msg.start_time),
      knots: msg.knots
          .into_iter()
          .collect(),
      pos_pts: msg.pos_pts
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
      yaw_pts: msg.yaw_pts
          .into_iter()
          .collect(),
      yaw_dt: msg.yaw_dt,
      constrained_clearance: msg.constrained_clearance,
      required_clearance: msg.required_clearance,
    }
  }
}


// Corresponds to traj_utils__msg__DataDisp

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DataDisp {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DataDisp::default())
  }
}

impl rosidl_runtime_rs::Message for DataDisp {
  type RmwMsg = super::msg::rmw::DataDisp;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        a: msg.a,
        b: msg.b,
        c: msg.c,
        d: msg.d,
        e: msg.e,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      a: msg.a,
      b: msg.b,
      c: msg.c,
      d: msg.d,
      e: msg.e,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      a: msg.a,
      b: msg.b,
      c: msg.c,
      d: msg.d,
      e: msg.e,
    }
  }
}


// Corresponds to traj_utils__msg__MultiBsplines

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiBsplines {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drone_id_from: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub traj: Vec<super::msg::Bspline>,

}



impl Default for MultiBsplines {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MultiBsplines::default())
  }
}

impl rosidl_runtime_rs::Message for MultiBsplines {
  type RmwMsg = super::msg::rmw::MultiBsplines;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drone_id_from: msg.drone_id_from,
        traj: msg.traj
          .into_iter()
          .map(|elem| super::msg::Bspline::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      drone_id_from: msg.drone_id_from,
        traj: msg.traj
          .iter()
          .map(|elem| super::msg::Bspline::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      drone_id_from: msg.drone_id_from,
      traj: msg.traj
          .into_iter()
          .map(super::msg::Bspline::from_rmw_message)
          .collect(),
    }
  }
}


