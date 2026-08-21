#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__LoadMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "fastlio_global_slam__rosidl_generator_c")]
extern "C" {
    fn fastlio_global_slam__srv__LoadMap_Request__init(msg: *mut LoadMap_Request) -> bool;
    fn fastlio_global_slam__srv__LoadMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>, size: usize) -> bool;
    fn fastlio_global_slam__srv__LoadMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>);
    fn fastlio_global_slam__srv__LoadMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>) -> bool;
}

// Corresponds to fastlio_global_slam__srv__LoadMap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub directory: rosidl_runtime_rs::String,

}



impl Default for LoadMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fastlio_global_slam__srv__LoadMap_Request__init(&mut msg as *mut _) {
        panic!("Call to fastlio_global_slam__srv__LoadMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__LoadMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__LoadMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__LoadMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "fastlio_global_slam/srv/LoadMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__LoadMap_Request() }
  }
}


#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__LoadMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "fastlio_global_slam__rosidl_generator_c")]
extern "C" {
    fn fastlio_global_slam__srv__LoadMap_Response__init(msg: *mut LoadMap_Response) -> bool;
    fn fastlio_global_slam__srv__LoadMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>, size: usize) -> bool;
    fn fastlio_global_slam__srv__LoadMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>);
    fn fastlio_global_slam__srv__LoadMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>) -> bool;
}

// Corresponds to fastlio_global_slam__srv__LoadMap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for LoadMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fastlio_global_slam__srv__LoadMap_Response__init(&mut msg as *mut _) {
        panic!("Call to fastlio_global_slam__srv__LoadMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__LoadMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__LoadMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__LoadMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "fastlio_global_slam/srv/LoadMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__LoadMap_Response() }
  }
}


#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__Relocalize_Request() -> *const std::ffi::c_void;
}

#[link(name = "fastlio_global_slam__rosidl_generator_c")]
extern "C" {
    fn fastlio_global_slam__srv__Relocalize_Request__init(msg: *mut Relocalize_Request) -> bool;
    fn fastlio_global_slam__srv__Relocalize_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Relocalize_Request>, size: usize) -> bool;
    fn fastlio_global_slam__srv__Relocalize_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Relocalize_Request>);
    fn fastlio_global_slam__srv__Relocalize_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Relocalize_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Relocalize_Request>) -> bool;
}

// Corresponds to fastlio_global_slam__srv__Relocalize_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Relocalize_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub use_latest_scan: bool,

}



impl Default for Relocalize_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fastlio_global_slam__srv__Relocalize_Request__init(&mut msg as *mut _) {
        panic!("Call to fastlio_global_slam__srv__Relocalize_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Relocalize_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__Relocalize_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__Relocalize_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__Relocalize_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Relocalize_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Relocalize_Request where Self: Sized {
  const TYPE_NAME: &'static str = "fastlio_global_slam/srv/Relocalize_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__Relocalize_Request() }
  }
}


#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__Relocalize_Response() -> *const std::ffi::c_void;
}

#[link(name = "fastlio_global_slam__rosidl_generator_c")]
extern "C" {
    fn fastlio_global_slam__srv__Relocalize_Response__init(msg: *mut Relocalize_Response) -> bool;
    fn fastlio_global_slam__srv__Relocalize_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Relocalize_Response>, size: usize) -> bool;
    fn fastlio_global_slam__srv__Relocalize_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Relocalize_Response>);
    fn fastlio_global_slam__srv__Relocalize_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Relocalize_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Relocalize_Response>) -> bool;
}

// Corresponds to fastlio_global_slam__srv__Relocalize_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Relocalize_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub estimated_pose: geometry_msgs::msg::rmw::Pose,

}



impl Default for Relocalize_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fastlio_global_slam__srv__Relocalize_Response__init(&mut msg as *mut _) {
        panic!("Call to fastlio_global_slam__srv__Relocalize_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Relocalize_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__Relocalize_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__Relocalize_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__Relocalize_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Relocalize_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Relocalize_Response where Self: Sized {
  const TYPE_NAME: &'static str = "fastlio_global_slam/srv/Relocalize_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__Relocalize_Response() }
  }
}


#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__SaveMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "fastlio_global_slam__rosidl_generator_c")]
extern "C" {
    fn fastlio_global_slam__srv__SaveMap_Request__init(msg: *mut SaveMap_Request) -> bool;
    fn fastlio_global_slam__srv__SaveMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>, size: usize) -> bool;
    fn fastlio_global_slam__srv__SaveMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>);
    fn fastlio_global_slam__srv__SaveMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SaveMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>) -> bool;
}

// Corresponds to fastlio_global_slam__srv__SaveMap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub directory: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub resolution: f32,

}



impl Default for SaveMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fastlio_global_slam__srv__SaveMap_Request__init(&mut msg as *mut _) {
        panic!("Call to fastlio_global_slam__srv__SaveMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SaveMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__SaveMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__SaveMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__SaveMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SaveMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "fastlio_global_slam/srv/SaveMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__SaveMap_Request() }
  }
}


#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__SaveMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "fastlio_global_slam__rosidl_generator_c")]
extern "C" {
    fn fastlio_global_slam__srv__SaveMap_Response__init(msg: *mut SaveMap_Response) -> bool;
    fn fastlio_global_slam__srv__SaveMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>, size: usize) -> bool;
    fn fastlio_global_slam__srv__SaveMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>);
    fn fastlio_global_slam__srv__SaveMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SaveMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>) -> bool;
}

// Corresponds to fastlio_global_slam__srv__SaveMap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SaveMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fastlio_global_slam__srv__SaveMap_Response__init(&mut msg as *mut _) {
        panic!("Call to fastlio_global_slam__srv__SaveMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SaveMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__SaveMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__SaveMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fastlio_global_slam__srv__SaveMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SaveMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "fastlio_global_slam/srv/SaveMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fastlio_global_slam__srv__SaveMap_Response() }
  }
}






#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fastlio_global_slam__srv__LoadMap() -> *const std::ffi::c_void;
}

// Corresponds to fastlio_global_slam__srv__LoadMap
#[allow(missing_docs, non_camel_case_types)]
pub struct LoadMap;

impl rosidl_runtime_rs::Service for LoadMap {
    type Request = LoadMap_Request;
    type Response = LoadMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fastlio_global_slam__srv__LoadMap() }
    }
}




#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fastlio_global_slam__srv__Relocalize() -> *const std::ffi::c_void;
}

// Corresponds to fastlio_global_slam__srv__Relocalize
#[allow(missing_docs, non_camel_case_types)]
pub struct Relocalize;

impl rosidl_runtime_rs::Service for Relocalize {
    type Request = Relocalize_Request;
    type Response = Relocalize_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fastlio_global_slam__srv__Relocalize() }
    }
}




#[link(name = "fastlio_global_slam__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fastlio_global_slam__srv__SaveMap() -> *const std::ffi::c_void;
}

// Corresponds to fastlio_global_slam__srv__SaveMap
#[allow(missing_docs, non_camel_case_types)]
pub struct SaveMap;

impl rosidl_runtime_rs::Service for SaveMap {
    type Request = SaveMap_Request;
    type Response = SaveMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fastlio_global_slam__srv__SaveMap() }
    }
}


