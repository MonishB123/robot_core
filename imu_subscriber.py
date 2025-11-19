import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import serial
import math
import sys

def get_quaternion_from_euler(roll, pitch, yaw):
    # This function converts Euler angles (roll, pitch, yaw) in radians to a Quaternion (x, y, z, w).
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_driver')
        # Publisher to the /imu/data topic using geometry_msgs/Quaternion
        self.publisher_ = self.create_publisher(Quaternion, '/imu/data', 10)
        
        # CHANGE PORT HERE
        # Initializes the serial connection
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.serial_port.flush()
        
        # Creates a timer to call timer_callback every 0.02 seconds (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("ROS 2 IMU Driver Started. Connected to Arduino.")

    def timer_callback(self):
        # Checks if there is data waiting on the serial port
        if self.serial_port.in_waiting > 0:
            try:
                # Read a line, decode it from bytes to string, and remove whitespace
                line = self.serial_port.readline().decode('utf-8').strip()
                # Split the string by comma to get individual data points (pitch, roll, yaw)
                data = line.split(',')

                if len(data) == 3:
                    # Convert string data to float degrees
                    pitch_deg = float(data[0])
                    roll_deg = float(data[1])
                    yaw_deg = float(data[2])

                    # Convert degrees to radians
                    roll = math.radians(roll_deg)
                    pitch = math.radians(pitch_deg)
                    yaw = math.radians(yaw_deg)

                    # Convert Euler angles to Quaternion
                    q = get_quaternion_from_euler(roll, pitch, yaw)

                    # Create and populate the ROS 2 message
                    msg = Quaternion()
                    msg.x = q[0]
                    msg.y = q[1]
                    msg.z = q[2]
                    msg.w = q[3]

                    # Publish the message
                    self.publisher_.publish(msg)
                    
            except ValueError:
                # Catches error if data is not a valid float (e.g., incomplete line)
                pass
            except Exception as e:
                # Catches all other exceptions (like serial read errors)
                # **The corrected line using .format()**
                self.get_logger().error("Serial Error: {}".format(e))

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuSerialNode()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup: close serial port and shutdown ROS 2
        imu_node.serial_port.close()
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
