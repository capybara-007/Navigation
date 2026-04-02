import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import struct
import time
import threading
import math
import numpy as np

SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

# 协议常量
HEADER = b'\x41\x54'
TARGET_ID = b'\x0A\x20\x00\x00'
TAIL = b'\x0D\x0A'
CMD_SET_500K = b'AT+CG=04\r\n'

# 轴距 (单位: 米) - Wheeltec 0.55m
WHEELBASE = 0.55
# 速度死区 (m/s)
STOP_THRESHOLD = 0.02
# 速度校准系数 
CORRECTION_FACTOR = 1.0 

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')
        self.odom_pub = self.create_publisher(Odometry, '/robot_odom', 10)
        self.port = SERIAL_PORT
        self.baud = BAUD_RATE
        self.ser = None
        self.running = False
        
        if self.connect_serial():
            self.running = True
            self.read_thread = threading.Thread(target=self.read_loop)
            self.read_thread.start()
            self.get_logger().info(f"Odom 节点启动 (位操作解析版, 轴距={WHEELBASE}m)")
        else:
            self.get_logger().error(f"无法打开串口 {self.port}")

    def init_can_module(self):
        self.get_logger().info("初始化 CAN 模块...")
        try:
            self.ser.write(b'AT+AT\r\n')
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.ser.write(CMD_SET_500K)
            time.sleep(0.5)
            response = self.ser.read_all()
            if b'OK' in response:
                self.get_logger().info("✅ CAN 初始化成功")
                return True
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 初始化失败: {e}")
            return False

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            if self.init_can_module():
                return True
            else:
                self.ser.close()
                return False
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            return False

    def parse_packet(self, data_bytes):
        if len(data_bytes) != 8: return

        try:
            # 1. 解析逻辑 
            
            # 使用 Little-Endian 解析 64位 无符号整数
            raw_val = struct.unpack('<Q', data_bytes)[0]

            # 解析速度 
            # 移位 20 bits，取 9 bits (0x1FF)
            speed_raw_val = (raw_val >> 20) & 0x1FF
            
            # 转换单位: raw * 0.1 = km/h
            speed_kmh = speed_raw_val * 0.1
            
            # 转换单位: km/h -> m/s
            speed_mps = speed_kmh / 3.6
            
            # 应用死区和校准
            if abs(speed_mps) < STOP_THRESHOLD:
                speed_mps = 0.0
            
            final_speed_mps = speed_mps * CORRECTION_FACTOR

            # 解析转向角度 
        
            # 用字节方式读取转向 
            steering_raw = struct.unpack('>h', data_bytes[2:4])[0] 
            steering_angle_deg = steering_raw * 0.01
            
            # [安全措施] 如果转向角大得离谱(>45度)，强制归零，防止角速度计算爆炸
            if abs(steering_angle_deg) > 60.0:
                steering_angle_deg = 0.0

            steering_angle_rad = math.radians(steering_angle_deg)

            # 2. 运动学计算 (四轮转向)
            # 前后轮反向偏转模式：角速度 = 2 * (v / L) * tan(delta)
            if abs(WHEELBASE) > 0.001:
                calculated_angular_z = 2.0 * (final_speed_mps / WHEELBASE) * math.tan(steering_angle_rad)
            else:
                calculated_angular_z = 0.0

            # 3. 坐标系旋转与发布 (Base -> Camera)
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            
            # 1. 线速度: 机器人 X -> 相机 Z
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = float(final_speed_mps)
            
            # 2. 角速度: 机器人 Z (Yaw) -> 相机 -Y (Pan)
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = float(calculated_angular_z) 
            msg.twist.twist.angular.z = 0.0
            
            self.odom_pub.publish(msg)

            # [调试打印] 
            print(f"Raw: {raw_val:016X} | Spd: {final_speed_mps:.2f} m/s | Steer: {steering_angle_deg:.2f}° | W: {calculated_angular_z:.2f}")

        except Exception as e:
            self.get_logger().warn(f"解析异常: {e}")

    def read_loop(self):
        buffer = b''
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)
                while len(buffer) >= 16:
                    if buffer[0:2] != HEADER:
                        buffer = buffer[1:]
                        continue
                    if buffer[2:6] != TARGET_ID:
                        buffer = buffer[1:] 
                        continue
                    data_len = buffer[6]
                    pkg_len = 9 + data_len
                    if len(buffer) < pkg_len:
                        break
                    if buffer[pkg_len - 2:pkg_len] == TAIL:
                        payload = buffer[7:7 + data_len]
                        self.parse_packet(payload)
                        buffer = buffer[pkg_len:]
                    else:
                        buffer = buffer[1:]
                
                time.sleep(0.002)
            except Exception as e:
                self.get_logger().error(f"读取异常: {e}")
                time.sleep(1)

    def destroy_node(self):
        self.running = False
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
