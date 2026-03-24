import rclpy
from rclpy.node import Node
from pestibot_messages.msg import Detection, MotorControl
import time


class PestibotController(Node):
    """Node that controls servos and pump based on detection feedback."""
    
    def __init__(self):
        super().__init__('pestibot_controller')
        
        # Servo control parameters
        self.declare_parameter('servo1_min', 0)
        self.declare_parameter('servo1_max', 180)
        self.declare_parameter('servo2_min', 70)  # Keep camera ~perpendicular
        self.declare_parameter('servo2_max', 110)
        self.declare_parameter('spray_middle_margin', 0.2)  # 20% margin from center
        self.declare_parameter('spray_delay', 0.3)  # Delay between spray on/off (seconds)
        self.declare_parameter('no_detection_timeout', 2.0)  # Stop spray after 2 sec no detection
        
        self.servo1_min = int(self.get_parameter('servo1_min').value)
        self.servo1_max = int(self.get_parameter('servo1_max').value)
        self.servo2_min = int(self.get_parameter('servo2_min').value)
        self.servo2_max = int(self.get_parameter('servo2_max').value)
        self.spray_middle_margin = float(self.get_parameter('spray_middle_margin').value)
        self.spray_delay = float(self.get_parameter('spray_delay').value)
        self.no_detection_timeout = float(self.get_parameter('no_detection_timeout').value)
        
        # State tracking
        self.last_spray_time = time.time()
        self.last_detection_time = time.time()
        self.current_spray_state = False
        self.servo1_position = (self.servo1_min + self.servo1_max) // 2
        self.servo2_position = (self.servo2_min + self.servo2_max) // 2
        
        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            Detection, 'detect', self.detection_callback, 10
        )
        
        # Publish motor control commands
        self.motor_pub = self.create_publisher(MotorControl, 'motor_actions', 10)
        
        # Timer to handle spray timeout and publish commands
        self.create_timer(0.05, self.update_control)
        
        self.get_logger().info('Pestibot Controller Node Initialized')

    def detection_callback(self, msg: Detection):
        """Process detection messages and calculate servo positions."""
        self.last_detection_time = time.time()
        
        if not msg.class_ids:
            # No detections
            return
        
        # Get the first/primary detection (highest confidence)
        best_idx = msg.confidences.index(max(msg.confidences))
        
        x_min = msg.x_min[best_idx]
        y_min = msg.y_min[best_idx]
        x_max = msg.x_max[best_idx]
        y_max = msg.y_max[best_idx]
        
        image_width = msg.image_width
        image_height = msg.image_height
        
        # Calculate bounding box center
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        
        # Servo 1: Vertical tracking (up/down)
        # Normalize Y position (0 = top, 1 = bottom)
        y_normalized = center_y / image_height
        # Map to servo range
        self.servo1_position = int(
            self.servo1_min + y_normalized * (self.servo1_max - self.servo1_min)
        )
        
        # Servo 2: Keep camera perpendicular to ground (slightly adjust if needed)
        # For now, keep it centered as default position
        self.servo2_position = (self.servo2_min + self.servo2_max) // 2
        
        # Check if object is in the middle of image (for spray activation)
        image_center_x = image_width / 2.0
        margin = image_width * self.spray_middle_margin
        
        if image_center_x - margin <= center_x <= image_center_x + margin:
            self._activate_spray()
        else:
            self._deactivate_spray()

    def _activate_spray(self):
        """Enable spray with delay to prevent flickering."""
        current_time = time.time()
        if not self.current_spray_state:
            if current_time - self.last_spray_time >= self.spray_delay:
                self.current_spray_state = True
                self.last_spray_time = current_time

    def _deactivate_spray(self):
        """Disable spray with delay to prevent flickering."""
        current_time = time.time()
        if self.current_spray_state:
            if current_time - self.last_spray_time >= self.spray_delay:
                self.current_spray_state = False
                self.last_spray_time = current_time

    def update_control(self):
        """Publish motor control messages and handle timeouts."""
        current_time = time.time()
        
        # Stop spray if no detection received for too long
        if current_time - self.last_detection_time > self.no_detection_timeout:
            if self.current_spray_state:
                if current_time - self.last_spray_time >= self.spray_delay:
                    self.current_spray_state = False
                    self.last_spray_time = current_time
        
        # Create and publish motor control message
        msg = MotorControl()
        msg.wheels = [0, 0]  # Wheels not used for the arm
        msg.servos = [self.servo1_position, self.servo2_position]
        msg.pump = self.current_spray_state
        
        self.motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = PestibotController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
