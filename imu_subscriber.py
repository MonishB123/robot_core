import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import serial
import math
import sys

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_driver')
        self.publisher_ = self.create_publisher(Quaternion, '/imu/data', 10)
        
        # CHANGE PORT HERE
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.serial_port.flush()
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("ROS 2 IMU Driver Started. Connected to Arduino.")

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                data = line.split(',')

                if len(data) == 3:
                    pitch_deg = float(data[0])
                    roll_deg = float(data[1])
                    yaw_deg = float(data[2])

                    roll = math.radians(roll_deg)
                    pitch = math.radians(pitch_deg)
                    yaw = math.radians(yaw_deg)

                    q = get_quaternion_from_euler(roll, pitch, yaw)

                    msg = Quaternion()
                    msg.x = q[0]
                    msg.y = q[1]
                    msg.z = q[2]
                    msg.w = q[3]

                    self.publisher_.publish(msg)
                    
            except ValueError:
                pass
            except Exception as e:
                self.get_logger().error(f"Serial Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuSerialNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.serial_port.close()
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
