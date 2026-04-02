import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import serial
import struct
import time
import threading
import math
import numpy as np

# ================= 配置参数 =================
SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

# 协议常量
HEADER = b'\x41\x54'
TAIL = b'\x0D\x0A'
CMD_SET_500K = b'AT+CG=04\r\n'

# 发送ID (控制) - 保留您测试成功的模块特定ID
CMD_CAN_ID = b'\x24\x20\x00\x00'  
# 接收ID (反馈)
ODOM_TARGET_ID = b'\x0A\x20\x00\x00'  

# 轴距 (单位: 米)
WHEELBASE = 0.55
# 速度死区保护
STOP_THRESHOLD = 0.05
CORRECTION_FACTOR = 1.0 

class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver_node')
        
        # ROS 2 话题接口
        self.odom_pub = self.create_publisher(Odometry, '/robot_odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 串口及线程变量
        self.port = SERIAL_PORT
        self.baud = BAUD_RATE
        self.ser = None
        self.running = False
        self.ser_lock = threading.Lock()
        
        # 控制状态保存
        self.last_gear = 1  # 默认D档

        if self.connect_serial():
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop)
            self.read_thread.start()
            self.get_logger().info(f"✅ 底盘驱动已启动 (已融合实车验证逻辑, 轴距={WHEELBASE}m)")
        else:
            self.get_logger().error(f"❌ 无法打开串口 {self.port}")

    # ==========================================
    #                发送逻辑 (Tx)
    # ==========================================
    def _calculate_steering(self, angle):
        """采用您的安全转向字节计算方法"""
        angle = max(-27.0, min(27.0, angle))
        val = int(abs(angle) * (120 / 27))
        if angle >= 0:
            return val & 0xFF
        else:
            return (256 - val) & 0xFF

    def cmd_vel_callback(self, msg):
        """接收 Nav2 控制并转换下发"""
        if not self.running or self.ser is None: 
            return

        # 1. 提取 Nav2 指令
        target_speed_mps = msg.linear.x
        target_angular_z = msg.angular.z

        # 2. 阿克曼逆解，计算目标方向盘转角 (度)
        if abs(target_speed_mps) > 0.01:
            steer_rad = math.atan((target_angular_z * WHEELBASE) / abs(target_speed_mps))
        else:
            steer_rad = 0.0
        target_angle_deg = math.degrees(steer_rad)

        # 3. 档位判断
        target_gear = 1
        if target_speed_mps < -0.01:
            target_gear = 3  # R档
        elif target_speed_mps > 0.01:
            target_gear = 1  # D档
        else:
            target_gear = self.last_gear
        
        self.last_gear = target_gear
        byte0 = (1 << 6) | target_gear

        # 4. 转向角装填 (直接复用您的逻辑)
        byte1_front = self._calculate_steering(target_angle_deg)
        byte2_rear = self._calculate_steering(-target_angle_deg) 

        # 5. 速度转换与刹车逻辑
        speed_abs_mps = abs(target_speed_mps)
        brake_enable = False

        if speed_abs_mps < STOP_THRESHOLD:
            speed_val = 0
            brake_enable = True  # 速度为0时主动拉起刹车
        else:
            speed_kmh = speed_abs_mps * 3.6
            speed_val = int(speed_kmh * 10)

        speed_val = min(speed_val, 100) # 限制最大值

        byte3 = speed_val & 0xFF
        byte4 = (speed_val >> 8) & 0x01

        if brake_enable:
            byte4 |= (1 << 1)

        # 6. 打包与发送
        payload = bytes([byte0, byte1_front, byte2_rear, byte3, byte4, 0x00, 0x00, 0x00])
        packet = HEADER + CMD_CAN_ID + b'\x08' + payload + TAIL

        with self.ser_lock:
            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().error(f"指令下发错误: {e}")

    # ==========================================
    #                接收逻辑 (Rx)
    # ==========================================
    def _parse_0x51_payload(self, data_bytes):
        """解析里程计数据并发布"""
        if len(data_bytes) != 8: return
        try:
            raw_val = struct.unpack('<Q', data_bytes)[0]

            # 1. 档位
            shift_level = raw_val & 0x03

            # 2. 转向 (依据协议 Bit 位)
            steer_dir = (raw_val >> 7) & 0x01
            steer_raw = (raw_val >> 8) & 0xFFF
            steer_angle_deg = steer_raw * 0.1
            if steer_dir == 0:
                steer_angle_deg = -steer_angle_deg

            # 3. 速度解析 (复用您的换算逻辑)
            speed_raw = (raw_val >> 20) & 0x1FF
            speed_kmh = speed_raw * 0.1
            speed_mps = speed_kmh / 3.6

            if abs(speed_mps) < 0.02:
                speed_mps = 0.0
            
            # 若处于 R 档，赋予负速度以保证 SLAM 正常运作
            if shift_level == 3: 
                speed_mps = -speed_mps

            final_speed_mps = speed_mps * CORRECTION_FACTOR

            # 4. 阿克曼正解，计算底盘实际角速度
            steering_angle_rad = math.radians(steer_angle_deg)
            if abs(WHEELBASE) > 0.001:
                calculated_angular_z = 2.0 * (final_speed_mps / WHEELBASE) * math.tan(steering_angle_rad)
            else:
                calculated_angular_z = 0.0

            # 5. 发布 Odom
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = float(final_speed_mps)
            
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = float(calculated_angular_z) 
            msg.twist.twist.angular.z = 0.0
            
            self.odom_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"解析异常: {e}")

    def _read_loop(self):
        """接收线程循环"""
        buffer = b''
        while self.running and rclpy.ok():
            try:
                # 使用锁保护读操作
                with self.ser_lock:
                    in_waiting = self.ser.in_waiting
                    if in_waiting:
                        buffer += self.ser.read(in_waiting)
                        
                while len(buffer) >= 16:
                    if buffer[0:2] != HEADER:
                        buffer = buffer[1:]
                        continue
                    
                    current_id = buffer[2:6]
                    data_len = buffer[6]
                    packet_len = 2 + 4 + 1 + data_len + 2

                    if len(buffer) < packet_len:
                        break

                    tail = buffer[7 + data_len: 7 + data_len + 2]
                    if tail == TAIL:
                        if current_id == ODOM_TARGET_ID:
                            payload = buffer[7: 7 + data_len]
                            self._parse_0x51_payload(payload)
                        buffer = buffer[packet_len:]
                    else:
                        buffer = buffer[1:]
                
                time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f"读取异常: {e}")
                time.sleep(1)

    # ==========================================
    #                硬件初始化
    # ==========================================
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

    def destroy_node(self):
        self.running = False
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChassisDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
