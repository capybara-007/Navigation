import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist  # [新增] 导入控制指令消息类型
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

class ChassisDriver(Node):  # 建议改名为 ChassisDriver，功能更贴切
    def __init__(self):
        super().__init__('chassis_driver_node')
        
        # 1. 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/robot_odom', 10)
        # 2. [新增] 订阅 Nav2 控制指令
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.port = SERIAL_PORT
        self.baud = BAUD_RATE
        self.ser = None
        self.running = False
        
        # [新增] 串口读写锁，防止发送控制指令和读取底层数据发生冲突
        self.ser_lock = threading.Lock() 
        
        if self.connect_serial():
            self.running = True
            self.read_thread = threading.Thread(target=self.read_loop)
            self.read_thread.start()
            self.get_logger().info(f"✅ 收发一体底盘驱动启动 (自动下发控制, 轴距={WHEELBASE}m)")
        else:
            self.get_logger().error(f"❌ 无法打开串口 {self.port}")

    # ==================== 【新增：控制指令下发逻辑】 ====================
    def cmd_vel_callback(self, msg):
        if not self.running or self.ser is None:
            return

        v = msg.linear.x
        w = msg.angular.z

        # 1. 速度换算 (Nav2: m/s -> 协议 0x121: 0.1 km/h)
        speed_abs = abs(v)
        speed_kmh = speed_abs * 3.6
        speed_val = int(speed_kmh * 10.0) 
        if speed_val > 100: speed_val = 100 # 限制在 10km/h 以内安全速度

        brake_en = 0
        if speed_abs < 0.01:
            gear = 2 # N 档
            brake_en = 1 # 速度为0时主动拉起刹车使能
            speed_val = 0
        elif v > 0:
            gear = 1 # D 档
        else:
            gear = 3 # R 档

        # 2. 转向角换算 (阿克曼模型)
        if speed_abs > 0.01:
            steer_rad = math.atan((w * WHEELBASE) / speed_abs)
        else:
            steer_rad = 0.0
            
        steer_deg = math.degrees(steer_rad)
        
        if steer_deg > 27.0: steer_deg = 27.0
        if steer_deg < -27.0: steer_deg = -27.0
        
        # 映射到协议 [-120, 120] 范围
        front_steer_val = int((steer_deg / 27.0) * 120.0)
        front_steer_hex = front_steer_val & 0xFF  # 转补码
        rear_steer_hex = 0x00

        # 3. 严格按 0x121 协议打包数据
        data = bytearray(8)
        data[0] = (1 << 6) | gear                 # 自动模式(Bit 6)+档位(Bit 0-1)
        data[1] = front_steer_hex                 # 前转向
        data[2] = rear_steer_hex                  # 后转向
        data[3] = speed_val & 0xFF                # 速度低8位
        data[4] = ((speed_val >> 8) & 0x01) | (brake_en << 1) # 速度最高位+刹车
        data[5] = 0x00
        data[6] = 0x00
        data[7] = 0x00
        
        # 4. 组装串口帧发送
        ctrl_id = b'\x21\x01\x00\x00' # 小端模式 0x00000121
        frame = HEADER + ctrl_id + bytes([0x08]) + data + TAIL
        
        # 加锁保护写操作
        with self.ser_lock:
            try:
                self.ser.write(frame)
            except Exception as e:
                self.get_logger().error(f"下发控制失败: {e}")
    # ====================================================================

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
            # 使用 Little-Endian 解析 64位 无符号整数
            raw_val = struct.unpack('<Q', data_bytes)[0]

            # 【修复】：严格依据 0x51 协议读取各个字段位
            gear_status = raw_val & 0x03
            steer_dir = (raw_val >> 7) & 0x01       # 转向方向: 0=左边, 1=右边
            steer_raw_val = (raw_val >> 8) & 0xFFF  # 转向角度绝对值
            speed_raw_val = (raw_val >> 20) & 0x1FF
            
            # 解析速度 (协议规定这里就是 m/s, Factor=0.1)
            speed_mps = speed_raw_val * 0.1
            
            # 应用死区和倒车逻辑
            if abs(speed_mps) < STOP_THRESHOLD:
                speed_mps = 0.0
            if gear_status == 3: # 0x3:R档
                speed_mps = -speed_mps
            
            final_speed_mps = speed_mps * CORRECTION_FACTOR

            # 【修复】：解析转向角度 (取代原来错误的 data_bytes[2:4] 切片法)
            steer_deg = steer_raw_val * 0.1
            if steer_dir == 0:  # 左转为负值
                steer_deg = -steer_deg
                
            if abs(steer_deg) > 60.0:
                steer_deg = 0.0

            steering_angle_rad = math.radians(steer_deg)

            # 运动学计算 (四轮转向角速度提取)
            if abs(WHEELBASE) > 0.001:
                calculated_angular_z = 2.0 * (final_speed_mps / WHEELBASE) * math.tan(steering_angle_rad)
            else:
                calculated_angular_z = 0.0

            # 坐标系旋转与发布 (Base -> Camera)
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

        except Exception as e:
            self.get_logger().warn(f"解析异常: {e}")

    def read_loop(self):
        buffer = b''
        while self.running and rclpy.ok():
            try:
                # 【修改】加上读锁，避免与 cmd_vel 发送产生串口抢占冲突
                with self.ser_lock:
                    in_waiting = self.ser.in_waiting
                    if in_waiting:
                        buffer += self.ser.read(in_waiting)
                        
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
