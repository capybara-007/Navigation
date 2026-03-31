#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__Imu() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__Imu__init(msg: *mut Imu) -> bool;
    fn autorccar_interfaces__msg__Imu__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Imu>, size: usize) -> bool;
    fn autorccar_interfaces__msg__Imu__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Imu>);
    fn autorccar_interfaces__msg__Imu__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Imu>, out_seq: *mut rosidl_runtime_rs::Sequence<Imu>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__Imu
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Imu {

    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub linear_acceleration: geometry_msgs::msg::rmw::Vector3,

}



impl Default for Imu {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__Imu__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__Imu__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Imu {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Imu__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Imu__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Imu__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Imu {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Imu where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/Imu";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__Imu() }
  }
}


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__Gnss() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__Gnss__init(msg: *mut Gnss) -> bool;
    fn autorccar_interfaces__msg__Gnss__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Gnss>, size: usize) -> bool;
    fn autorccar_interfaces__msg__Gnss__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Gnss>);
    fn autorccar_interfaces__msg__Gnss__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Gnss>, out_seq: *mut rosidl_runtime_rs::Sequence<Gnss>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__Gnss
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Gnss {

    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position_ecef: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity_ecef: geometry_msgs::msg::rmw::Vector3,

}



impl Default for Gnss {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__Gnss__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__Gnss__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Gnss {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Gnss__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Gnss__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Gnss__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Gnss {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Gnss where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/Gnss";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__Gnss() }
  }
}


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__NavState() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__NavState__init(msg: *mut NavState) -> bool;
    fn autorccar_interfaces__msg__NavState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavState>, size: usize) -> bool;
    fn autorccar_interfaces__msg__NavState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavState>);
    fn autorccar_interfaces__msg__NavState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavState>, out_seq: *mut rosidl_runtime_rs::Sequence<NavState>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__NavState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub origin: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub quaternion: geometry_msgs::msg::rmw::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_velocity: geometry_msgs::msg::rmw::Vector3,

}



impl Default for NavState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__NavState__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__NavState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__NavState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__NavState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__NavState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavState where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/NavState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__NavState() }
  }
}


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__ControlCommand() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__ControlCommand__init(msg: *mut ControlCommand) -> bool;
    fn autorccar_interfaces__msg__ControlCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlCommand>, size: usize) -> bool;
    fn autorccar_interfaces__msg__ControlCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlCommand>);
    fn autorccar_interfaces__msg__ControlCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlCommand>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__ControlCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_angle: f64,

}



impl Default for ControlCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__ControlCommand__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__ControlCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__ControlCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__ControlCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__ControlCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/ControlCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__ControlCommand() }
  }
}


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__Path() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__Path__init(msg: *mut Path) -> bool;
    fn autorccar_interfaces__msg__Path__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Path>, size: usize) -> bool;
    fn autorccar_interfaces__msg__Path__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Path>);
    fn autorccar_interfaces__msg__Path__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Path>, out_seq: *mut rosidl_runtime_rs::Sequence<Path>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__Path
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Path {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path_points: rosidl_runtime_rs::Sequence<super::super::msg::rmw::PathPoint>,

}



impl Default for Path {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__Path__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__Path__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Path {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Path__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Path__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__Path__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Path {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Path where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/Path";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__Path() }
  }
}


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__PathPoint() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__PathPoint__init(msg: *mut PathPoint) -> bool;
    fn autorccar_interfaces__msg__PathPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PathPoint>, size: usize) -> bool;
    fn autorccar_interfaces__msg__PathPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PathPoint>);
    fn autorccar_interfaces__msg__PathPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PathPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<PathPoint>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__PathPoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PathPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f64,

}



impl Default for PathPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__PathPoint__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__PathPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PathPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__PathPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__PathPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__PathPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PathPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PathPoint where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/PathPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__PathPoint() }
  }
}


#[link(name = "autorccar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__BoundingBoxes() -> *const std::ffi::c_void;
}

#[link(name = "autorccar_interfaces__rosidl_generator_c")]
extern "C" {
    fn autorccar_interfaces__msg__BoundingBoxes__init(msg: *mut BoundingBoxes) -> bool;
    fn autorccar_interfaces__msg__BoundingBoxes__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BoundingBoxes>, size: usize) -> bool;
    fn autorccar_interfaces__msg__BoundingBoxes__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BoundingBoxes>);
    fn autorccar_interfaces__msg__BoundingBoxes__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BoundingBoxes>, out_seq: *mut rosidl_runtime_rs::Sequence<BoundingBoxes>) -> bool;
}

// Corresponds to autorccar_interfaces__msg__BoundingBoxes
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundingBoxes {

    // This member is not documented.
    #[allow(missing_docs)]
    pub num: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bounding_boxes: rosidl_runtime_rs::Sequence<vision_msgs::msg::rmw::BoundingBox2D>,

}



impl Default for BoundingBoxes {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autorccar_interfaces__msg__BoundingBoxes__init(&mut msg as *mut _) {
        panic!("Call to autorccar_interfaces__msg__BoundingBoxes__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BoundingBoxes {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__BoundingBoxes__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__BoundingBoxes__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autorccar_interfaces__msg__BoundingBoxes__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BoundingBoxes {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BoundingBoxes where Self: Sized {
  const TYPE_NAME: &'static str = "autorccar_interfaces/msg/BoundingBoxes";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autorccar_interfaces__msg__BoundingBoxes() }
  }
}


