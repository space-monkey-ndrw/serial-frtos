import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
from cobs import cobs
from geometry_msgs.msg import Twist

class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.serial_port = '/dev/ttyNucleo'  # Adjust as needed
        self.baud_rate = 500000
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        self.timer = self.create_timer(0.005, self.read_serial_data)  # Read at 100 Hz
        self.buffer = b'' # Persistent buffer to accumulate incoming serial data
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        # 1. Physical Parameters (Update these for your robot!)
        wheel_base = 0.093   # Distance between wheels in meters
        wheel_radius = 0.03 # Radius of your wheels in meters
        max_pwm = 99.0     # Your Nucleo's max CCR value (or 0-255, etc.)

        # 2. Differential Drive Kinematics
        # Linear.x is forward/backward, Angular.z is rotation
        v_left = msg.linear.x - (msg.angular.z * wheel_base / 2.0)
        v_right = msg.linear.x + (msg.angular.z * wheel_base / 2.0)

        # 3. Conversion to PWM
        # Since you don't have encoders, we 'guestimate' that 1.0 m/s = max_pwm
        # You will need to tune this 'multiplier' based on your battery and motors
        multiplier = 262.6  # This is a guess: max_pwm / max_speed (e.g., 99 / 0.377 m/s) - you will need to tune this
        
        left_pwm = v_left * multiplier
        right_pwm = v_right * multiplier

        # 4. Constraints (Clamp to your max PWM range)
        left_pwm = max(min(left_pwm, max_pwm), -max_pwm)
        right_pwm = max(min(right_pwm, max_pwm), -max_pwm)

        # 5. Send to Nucleo
        # This uses the function we already wrote to pack and COBS-encode the data
        self.send_motor_command(left_pwm, right_pwm)
    
    def send_motor_command(self, left_pwm, right_pwm):
        if self.serial_connection.is_open:
            raw_data = struct.pack('<ff', float(left_pwm), float(right_pwm))
            encoded = cobs.encode(raw_data) + b'\x00'
            self.serial_connection.write(encoded)

    def read_serial_data(self):
        if not self.serial_connection.is_open:
            self.get_logger().error(f'Cannot read from serial port {self.serial_port}, it is not open.', throttle_duration_sec=5.0)
            return
        try:
            while self.serial_connection.in_waiting > 0:
                self.buffer += self.serial_connection.read(self.serial_connection.in_waiting)  # Read all available data
                # old way, blocking call 
                # raw_data = self.serial_connection.read_until(b'\x00')  # Read until null byte (COBS delimiter)
                while b'\x00' in self.buffer:
                    packet, self.buffer = self.buffer.split(b'\x00', 1)  # Split at the first null byte
                    if not packet:
                        continue  # Skip empty packets
                    try:
                        decoded_data = cobs.decode(packet)  # Decode the COBS packet
                        imu_msg = self.parse_imu_data(decoded_data)
                        if imu_msg:
                            self.publisher_.publish(imu_msg)
                    except cobs.DecodeError as e:
                        self.get_logger().error(f'Invalid packet received, COBS decode error: {e}')
                        continue  # Skip this packet and continue with the next one
                # old way using old blocking call above:
                # try:
                #    decoded_data = cobs.decode(raw_data[:-1])  # Remove the delimiter before decoding
                #    imu_msg = self.parse_imu_data(decoded_data)
                #    if imu_msg:
                #        self.publisher_.publish(imu_msg)
                #except cobs.DecodeError as e:
                #    self.get_logger().error(f'COBS decode error: {e}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.serial_connection.close()

    def parse_imu_data(self, data):
        if len(data) != 40:  # Expecting 10 floats (3 for accel, 3 for gyro, 4 for quaternion)
            self.get_logger().error(f'Unexpected data length: {len(data)}')
            return None
        
        try:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, quat_w, quat_x, quat_y, quat_z = struct.unpack('<ffffffffff', data)
            imu_msg = Imu()
            imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.orientation.w = quat_w
            imu_msg.orientation.x = quat_x
            imu_msg.orientation.y = quat_y
            imu_msg.orientation.z = quat_z
            return imu_msg
        except struct.error as e:
            self.get_logger().error(f'Struct unpack error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SerialImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
