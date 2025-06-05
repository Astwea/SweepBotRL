import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import re
from scipy.spatial.transform import Rotation as R  # 引入scipy.spatial.transform的Rotation类
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import struct
from threading import Thread
import time

class SerialToROS2(Node):
    def __init__(self):
        super().__init__('serial_to_ros2')

        # 创建串口连接
        self.ser = serial.Serial('/dev/control', 460800)  # 替换为你的串口

        # 创建 ROS 2 发布者
        while self.ser.in_waiting == 0: 
            data = f"#0 0 0 0 R#"
            self.ser.write(data.encode('utf-8'))

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transforms()
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.stop = 0
        self.data = None
        self.current_time = time.time()
        self.threads = []
        pub_thread = Thread(target=self.read_serial_data)
        pub_thread.start()
        self.threads.append(pub_thread)
        odom_thread = Thread(target=self.odom_compute)
        odom_thread.start()
        self.threads.append(odom_thread)

        self.buffer = b''
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_offset = None  # 初始偏差
        self.imu_initialized = False
        # 车轮参数
        self.wheel_radius = 0.029  # 假设车轮半径为 0.1 米
        self.wheel_base = 0.2  # 假设两个车轮之间的距离为 0.5 米

        # 设置定时器，每秒读取一次串口数据

        self.timer = self.create_timer(0.2, self.publish_tf)
        self.timer1 = self.create_timer(0.2, self.tf_publisher)
        self.count = 0

    def cmd_vel_callback(self, msg):
        # 从 /cmd_vel 消息中提取速度信息
        linear_x = msg.linear.x  # 线速度 (m/s)
        angular_z = msg.angular.z  # 角速度 (rad/s)
        self.get_logger().info(f"Received /cmd_vel: linear_x={linear_x}, angular_z={angular_z}")
        # 打包并发送给下位机
        self.send_to_serial(linear_x, angular_z)
    
    def imu_callback(self, msg):
        # 从四元数转换为欧拉角
        q = msg.orientation
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        if self.imu_offset is None:
            self.imu_offset = yaw
            self.imu_initialized = True
            self.get_logger().info(f"IMU offset initialized: {self.imu_offset:.4f} rad")
        imu_yaw = yaw - self.imu_offset
        # 确保角度在 -pi 到 pi 之间
        if imu_yaw > math.pi:
            imu_yaw -= 2 * math.pi
        elif imu_yaw < -math.pi:
            imu_yaw += 2 * math.pi
        self.theta = imu_yaw
    
    def quaternion_to_euler(self, x, y, z, w):
        # 四元数转欧拉角
        r = R.from_quat([x, y, z, w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        return roll, pitch, yaw
    
    def odom_compute(self):
        while not self.stop:
            now_time = time.time()
            if self.data is not None:
                motor1_speed, motor2_speed, pitch, roll, yaw, ax, ay, az, gx, gy, gz = self.data
                self.data = None
                reduction_ratio = 52  # 假设减速比为 10:1
                wheel1_speed = motor1_speed / reduction_ratio  # 轮子1转速
                wheel2_speed = motor2_speed / reduction_ratio  # 轮子2转速
                # 计算左轮和右轮的线速度 (m/s)
                left_speed = wheel1_speed * 2 * math.pi * self.wheel_radius  # 转速到线速度
                right_speed = wheel2_speed * 2 * math.pi * self.wheel_radius  # 转速到线速度

                self.v = (left_speed + right_speed) / 2  # 平均线速度
                self.w = (right_speed - left_speed) / self.wheel_base  # 角速度

                # 更新位置和角度
                dt = time.time() - now_time
                delta_x = self.v * math.cos(self.theta) * dt
                delta_y = self.v * math.sin(self.theta) * dt
                self.x += delta_x
                self.y += delta_y
    def tf_publisher(self):        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = "base_link"

        # 设置位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # 设置方向 (四元数)
        quat = self.euler_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # 设置速度
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.w

        # 发布里程计数据
        self.odom_pub.publish(odom_msg)

    def broadcast_static_transforms(self):
        transforms = []

        # ------------ base_link → laser ------------------
        laser_tf = TransformStamped()
        laser_tf.header.stamp = self.get_clock().now().to_msg()
        laser_tf.header.frame_id = 'base_link'
        laser_tf.child_frame_id = 'laser'
        laser_tf.transform.translation.x = -0.1
        laser_tf.transform.translation.y = 0.0
        laser_tf.transform.translation.z = 0.15
        quat_laser = self.euler_to_quaternion(0.0)
        laser_tf.transform.rotation.x = quat_laser[0]
        laser_tf.transform.rotation.y = quat_laser[1]
        laser_tf.transform.rotation.z = quat_laser[2]
        laser_tf.transform.rotation.w = quat_laser[3]
        transforms.append(laser_tf)

        # ------------ base_link → camera ------------------
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera'
        camera_tf.transform.translation.x = 0.1
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.10
        quat_cam = self.euler_to_quaternion(0.0)
        camera_tf.transform.rotation.x = quat_cam[0]
        camera_tf.transform.rotation.y = quat_cam[1]
        camera_tf.transform.rotation.z = quat_cam[2]
        camera_tf.transform.rotation.w = quat_cam[3]
        transforms.append(camera_tf)
        # ------------ base_link → imu_link ------------------
        imu_tf = TransformStamped()
        imu_tf.header.stamp = self.get_clock().now().to_msg()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = -0.1  # 向后 10cm
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.05  # 高度 5cm
        quat_imu = self.euler_to_quaternion(0.0)
        imu_tf.transform.rotation.x = quat_imu[0]
        imu_tf.transform.rotation.y = quat_imu[1]
        imu_tf.transform.rotation.z = quat_imu[2]
        imu_tf.transform.rotation.w = quat_imu[3]
        transforms.append(imu_tf)
        # ------------ 统一发布 ------------------
        self.static_tf_broadcaster.sendTransform(transforms)
        
    def send_to_serial(self, linear_x, angular_z):
        # 将线速度和角速度转换为下位机能识别的协议格式
        # 协议格式为: $v w c1 c2#
        try:
            # 假设 c1 和 c2 是占位参数，这里填入默认值 0
            if linear_x != 0 or angular_z != 0:
                c1, c2 = 1, 1
            else:
                c1,c2=0, 0
            data = f"${linear_x:.4f} {angular_z:.4f} {c1} {c2}#"
            self.ser.write(data.encode('utf-8'))
            # self.get_logger().info(f"Sent to serial: {data.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send data to serial: {e}")

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # 设置变换
        t.transform.translation.x = -self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # 设置旋转
        quat = self.euler_to_quaternion(self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # 发布 TF
        self.tf_broadcaster.sendTransform(t)

    def calculate_crc(self, data: bytes) -> int:
        """CRC-16-CCITT 校验计算"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF  # 保证为16位
        return crc

    def read_serial_data(self):
        while not self.stop:
            try:
                if self.ser.in_waiting > 0:
                    self.buffer += self.ser.read(self.ser.in_waiting)
                    #print(self.buffer)
                    while True:
                        start_idx = self.buffer.find(b'$')  # 查找包头
                        end_idx = self.buffer.find(b'#', start_idx)  # 查找包尾
                        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                            packet = self.buffer[start_idx:end_idx + 1]  # 提取完整数据包
                            self.parse_data(packet)  # 解析数据包
                            self.buffer = self.buffer[end_idx + 1:]  # 剩余数据保留在缓冲区
                        else:
                            break
            except Exception as e:
                print(f"\n底层串口报错{e}\n")

    def euler_to_quaternion(self, theta):
        qx = 0.0
        qy = 0.0
        qz = math.sin(theta / 2)
        qw = math.cos(theta / 2)
        return [qx, qy, qz, qw]

    def is_data_complete(self, data):
        if "A->" in data:
            return True
        return False

    def parse_data(self, data: bytes):
        """解析数据包"""
        if len(data) < 4:  # 最小包长度为 4（头 + 尾 + CRC）
            print("Invalid packet length.")
            return

        if data[0] == ord('$') and data[-1] == ord('#'):  # 检查包头和包尾
            payload = data[1:-3]  # 数据部分（去掉包头、CRC和包尾）
            received_crc = (data[-3] << 8) | data[-2]  # 提取CRC
            calculated_crc = self.calculate_crc(data[1:-3])  # 计算本地CRC

            if received_crc == calculated_crc:
                # 解析数据（根据下位机的数据格式）
                try:
                    unpacked_data = struct.unpack('<fffffffffff', payload)
                    self.data = unpacked_data
                    self.current_time = time.time()
                    if self.count > 20 and self.count<22:
                        print("\n*******************初始化完成**********************\n")
                        self.count += 20
                    else:
                        self.count += 1
                except struct.error as e:
                    print(f"Failed to unpack data: {e}")
        else:
            print("Invalid packet format.")

    def cleanup(self):
        self.get_logger().info("Cleaning up threads...")
        for t in self.threads:
            t.join()
        self.get_logger().info("All threads stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("检测到Ctrl+C进行中断,进行线程关闭")
    finally:
        node.ser.close()
        node.stop = 1
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
