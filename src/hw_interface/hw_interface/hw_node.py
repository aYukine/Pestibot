import rclpy
from rclpy.node import Node
from pestibot_messages.msg import MotorControl
import serial

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        self.motor_sub = self.create_subscription(MotorControl, 'motor_actions', self.motor_cb, 10)
        self.current_motor_cmd = None
        
        self.create_timer(0.05, self.update)
        self.get_logger().info("Hardware Bridge Node Initialized")

    def motor_cb(self, msg: MotorControl):
        """Store the latest motor control command."""
        self.current_motor_cmd = msg

    def update(self):
        """Send motor control commands to Arduino."""
        if self.current_motor_cmd is None:
            return
        
        msg = self.current_motor_cmd
        
        # Extract wheel, servo and pump states.
        left_wheel = msg.wheels[0] if len(msg.wheels) > 0 else 127
        right_wheel = msg.wheels[1] if len(msg.wheels) > 1 else 127
        servo1_pos = msg.servos[0] if len(msg.servos) > 0 else 90
        servo2_pos = msg.servos[1] if len(msg.servos) > 1 else 90
        pump_state = 1 if msg.pump else 0
        
        # Ensure values are in valid ranges
        left_wheel = max(0, min(255, left_wheel))
        right_wheel = max(0, min(255, right_wheel))
        servo1_pos = max(0, min(180, servo1_pos))
        servo2_pos = max(0, min(180, servo2_pos))
        
        # Create message:
        # [left_wheel, right_wheel, servo1, servo2, pump, checksum_low, checksum_high]
        checksum = int(left_wheel) + int(right_wheel) + int(servo1_pos) + int(servo2_pos) + int(pump_state)
        checksum_low = checksum & 0xFF
        checksum_high = (checksum >> 8) & 0xFF
        
        try:
            packet = [
                left_wheel,
                right_wheel,
                servo1_pos,
                servo2_pos,
                pump_state,
                checksum_low,
                checksum_high,
            ]
            self.ser.write(bytes(packet))
        except Exception as e:
            self.get_logger().error(f"Failed to send motor command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()