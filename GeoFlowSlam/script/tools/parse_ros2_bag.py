import os
import cv2
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore  # 修改这里：引入新的类型系统
from cv_bridge import CvBridge
import numpy as np
import argparse

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.state = None

    def filter(self, value):
        if self.state is None:
            self.state = value
        else:
            self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state

def extract_images_and_imu(bag_file, color_dir, depth_dir, imu_dir):
    # 创建文件夹
    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)

    bridge = CvBridge()
    color_num = 0
    depth_num = 0
    imu_num = 0
    odom_num = 0
    
    # 初始化 typestore，加载标准的 ROS2 消息类型（如 sensor_msgs, nav_msgs）
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(bag_file) as reader, open(os.path.join(imu_dir, "imu.txt"), 'w') as imu_f, open(os.path.join(imu_dir, "odom.txt"), 'w') as odom_f:
            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic
                
                # 1. 过滤掉不需要的话题，直接跳过，避免后面的 header 读取报错
                target_topics = [
                    '/camera/camera/color/image_raw',
                    '/camera/camera/aligned_depth_to_color/image_raw',
                    '/camera/camera/imu',
                    '/robot_odom'
                ]
                if topic not in target_topics:
                    continue

                # 2. 反序列化解码
                try:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                except Exception as e:
                    print(f"Skipping message on topic {topic} due to error: {e}")
                    continue

                # 3. 从消息中获取时间戳（此时留下来的话题都肯定包含 header 属性）
                timestamp = float(msg.header.stamp.sec) * 1e3 + float(msg.header.stamp.nanosec) * 1e-6

                # 4. 提取数据并保存
                if topic == '/camera/camera/color/image_raw':
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    image_filename = os.path.join(color_dir, f"{timestamp:.4f}.png")
                    cv2.imwrite(image_filename, cv_image)
                    print("timestamp: ", timestamp)
                    color_num += 1

                elif topic == '/camera/camera/aligned_depth_to_color/image_raw':
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    depth_filename = os.path.join(depth_dir, f"{timestamp:.4f}.png")
                    cv2.imwrite(depth_filename, cv_image)
                    depth_num += 1

                elif topic == '/camera/camera/imu':
                    accel = msg.linear_acceleration
                    gyro = msg.angular_velocity
                    imu_f.write(f"{timestamp:.4f},{accel.x},{accel.y},{accel.z},{gyro.x},{gyro.y},{gyro.z}\n")
                    imu_num += 1
                    
                elif topic == '/robot_odom':
                    twist = msg.twist.twist
                    linear = twist.linear
                    angular = twist.angular
                    odom_f.write(f"{timestamp:.4f},{linear.x},{linear.y},{linear.z},{angular.x},{angular.y},{angular.z}\n")
                    odom_num += 1

    print(f"Extracted {color_num} color images, {depth_num} depth images, {imu_num} IMU data, and {odom_num} Odom data.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_root", required=True, help="Path to the data root")
    parser.add_argument("--data_list", required=True, nargs='+', help="List of data directories")
    args = parser.parse_args()
    
    data_root = args.data_root
    data_list = args.data_list
    
    for data_dir in data_list:
        bag_file = os.path.join(data_root, data_dir)
        color_dir = os.path.join(bag_file, "color")
        depth_dir = os.path.join(bag_file, "depth")
        imu_dir = os.path.join(bag_file, "imu")

        print(f"Processing bag: {bag_file}")
        extract_images_and_imu(bag_file, color_dir, depth_dir, imu_dir)