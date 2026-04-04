import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
from cobs import cobs

class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.serial_port = '/dev/ttyNucleo'  # Adjust as needed
        self.baud_rate = 500000
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        self.timer = self.create_timer(0.005, self.read_serial_data)  # Read at 100 Hz

    def read_serial_data(self):
        while self.serial_connection.in_waiting > 0:
            raw_data = self.serial_connection.read_until(b'\x00')  # Read until null byte (COBS delimiter)
            try:
                decoded_data = cobs.decode(raw_data[:-1])  # Remove the delimiter before decoding
                imu_msg = self.parse_imu_data(decoded_data)
                if imu_msg:
                    self.publisher_.publish(imu_msg)
            except cobs.DecodeError as e:
                self.get_logger().error(f'COBS decode error: {e}')

    def parse_imu_data(self, data):
        if len(data) != 40:  # Expecting 10 floats (3 for accel, 3 for gyro, 4 for quaternion)
            self.get_logger().error(f'Unexpected data length: {len(data)}')
            return None
        
        try:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, quat_w, quat_x, quat_y, quat_z = struct.unpack('<ffffffffff', data)
            imu_msg = Imu()
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
