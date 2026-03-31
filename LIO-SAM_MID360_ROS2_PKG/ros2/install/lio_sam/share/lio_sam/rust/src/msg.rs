#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to lio_sam__msg__CloudInfo
/// Cloud Info

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CloudInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_ring_index: Vec<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub end_ring_index: Vec<i32>,

    /// point column index in range image
    pub point_col_ind: Vec<i32>,

    /// point range
    pub point_range: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu_available: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub odom_available: i64,

    /// Attitude for LOAM initialization
    pub imu_roll_init: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu_pitch_init: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu_yaw_init: f32,

    /// Initial guess from imu pre-integration
    pub initial_guess_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub initial_guess_y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub initial_guess_z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub initial_guess_roll: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub initial_guess_pitch: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub initial_guess_yaw: f32,

    /// Point cloud messages
    /// original cloud deskewed
    pub cloud_deskewed: sensor_msgs::msg::PointCloud2,

    /// extracted corner feature
    pub cloud_corner: sensor_msgs::msg::PointCloud2,

    /// extracted surface feature
    pub cloud_surface: sensor_msgs::msg::PointCloud2,

}



impl Default for CloudInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CloudInfo::default())
  }
}

impl rosidl_runtime_rs::Message for CloudInfo {
  type RmwMsg = super::msg::rmw::CloudInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        start_ring_index: msg.start_ring_index.into(),
        end_ring_index: msg.end_ring_index.into(),
        point_col_ind: msg.point_col_ind.into(),
        point_range: msg.point_range.into(),
        imu_available: msg.imu_available,
        odom_available: msg.odom_available,
        imu_roll_init: msg.imu_roll_init,
        imu_pitch_init: msg.imu_pitch_init,
        imu_yaw_init: msg.imu_yaw_init,
        initial_guess_x: msg.initial_guess_x,
        initial_guess_y: msg.initial_guess_y,
        initial_guess_z: msg.initial_guess_z,
        initial_guess_roll: msg.initial_guess_roll,
        initial_guess_pitch: msg.initial_guess_pitch,
        initial_guess_yaw: msg.initial_guess_yaw,
        cloud_deskewed: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Owned(msg.cloud_deskewed)).into_owned(),
        cloud_corner: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Owned(msg.cloud_corner)).into_owned(),
        cloud_surface: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Owned(msg.cloud_surface)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        start_ring_index: msg.start_ring_index.as_slice().into(),
        end_ring_index: msg.end_ring_index.as_slice().into(),
        point_col_ind: msg.point_col_ind.as_slice().into(),
        point_range: msg.point_range.as_slice().into(),
      imu_available: msg.imu_available,
      odom_available: msg.odom_available,
      imu_roll_init: msg.imu_roll_init,
      imu_pitch_init: msg.imu_pitch_init,
      imu_yaw_init: msg.imu_yaw_init,
      initial_guess_x: msg.initial_guess_x,
      initial_guess_y: msg.initial_guess_y,
      initial_guess_z: msg.initial_guess_z,
      initial_guess_roll: msg.initial_guess_roll,
      initial_guess_pitch: msg.initial_guess_pitch,
      initial_guess_yaw: msg.initial_guess_yaw,
        cloud_deskewed: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Borrowed(&msg.cloud_deskewed)).into_owned(),
        cloud_corner: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Borrowed(&msg.cloud_corner)).into_owned(),
        cloud_surface: sensor_msgs::msg::PointCloud2::into_rmw_message(std::borrow::Cow::Borrowed(&msg.cloud_surface)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      start_ring_index: msg.start_ring_index
          .into_iter()
          .collect(),
      end_ring_index: msg.end_ring_index
          .into_iter()
          .collect(),
      point_col_ind: msg.point_col_ind
          .into_iter()
          .collect(),
      point_range: msg.point_range
          .into_iter()
          .collect(),
      imu_available: msg.imu_available,
      odom_available: msg.odom_available,
      imu_roll_init: msg.imu_roll_init,
      imu_pitch_init: msg.imu_pitch_init,
      imu_yaw_init: msg.imu_yaw_init,
      initial_guess_x: msg.initial_guess_x,
      initial_guess_y: msg.initial_guess_y,
      initial_guess_z: msg.initial_guess_z,
      initial_guess_roll: msg.initial_guess_roll,
      initial_guess_pitch: msg.initial_guess_pitch,
      initial_guess_yaw: msg.initial_guess_yaw,
      cloud_deskewed: sensor_msgs::msg::PointCloud2::from_rmw_message(msg.cloud_deskewed),
      cloud_corner: sensor_msgs::msg::PointCloud2::from_rmw_message(msg.cloud_corner),
      cloud_surface: sensor_msgs::msg::PointCloud2::from_rmw_message(msg.cloud_surface),
    }
  }
}


