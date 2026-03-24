import rclpy
from rclpy.node import Node

from dc_gamepad_msgs.msg import GamePad
from pestibot_messages.msg import MotorControl


class PestibotManualControl(Node):
	def __init__(self):
		super().__init__('pestibot_manual_control')

		self.declare_parameter('joy_center', 127.0)
		self.declare_parameter('joy_span', 127.0)
		self.declare_parameter('deadzone', 0.08)
		self.declare_parameter('servo1_min', 0)
		self.declare_parameter('servo1_max', 180)
		self.declare_parameter('servo2_center', 90)
		self.declare_parameter('servo2_compensation_sign', -1.0)
		self.declare_parameter('servo2_compensation_gain', 1.0)
		self.declare_parameter('failsafe_timeout', 0.5)

		self.joy_center = float(self.get_parameter('joy_center').value)
		self.joy_span = float(self.get_parameter('joy_span').value)
		self.deadzone = float(self.get_parameter('deadzone').value)
		self.servo1_min = int(self.get_parameter('servo1_min').value)
		self.servo1_max = int(self.get_parameter('servo1_max').value)
		self.servo2_center = int(self.get_parameter('servo2_center').value)
		self.servo2_compensation_sign = float(
			self.get_parameter('servo2_compensation_sign').value
		)
		self.servo2_compensation_gain = float(
			self.get_parameter('servo2_compensation_gain').value
		)
		self.failsafe_timeout = float(self.get_parameter('failsafe_timeout').value)

		self.pump_enabled = False
		self.last_pad_time = self.get_clock().now()
		self.last_msg = None

		self.motor_pub = self.create_publisher(MotorControl, 'motor_actions', 10)
		self.pad_sub = self.create_subscription(GamePad, '/pad', self.pad_cb, 10)
		self.create_timer(0.05, self.publish_loop)

		self.get_logger().info('Pestibot manual control started.')

	def _to_normalized(self, value: int) -> float:
		normalized = (float(value) - self.joy_center) / self.joy_span
		if normalized > 1.0:
			return 1.0
		if normalized < -1.0:
			return -1.0
		return normalized

	def _apply_deadzone(self, value: float) -> float:
		if abs(value) < self.deadzone:
			return 0.0
		return value

	def _to_uint8(self, value: float) -> int:
		# Map [-1.0, 1.0] to [0, 255] where 127-128 is neutral.
		mapped = int(round((value + 1.0) * 127.5))
		return max(0, min(255, mapped))

	def pad_cb(self, msg: GamePad):
		self.last_pad_time = self.get_clock().now()
		self.last_msg = msg

		# Rising edge toggle on dpad_up
		if msg.dpad_up and (not msg.previous_button_up):
			self.pump_enabled = not self.pump_enabled
			state = 'ON' if self.pump_enabled else 'OFF'
			self.get_logger().info(f'Pump toggled: {state}')

	def publish_loop(self):
		control = MotorControl()

		if self.last_msg is None:
			control.wheels = [127, 127]
			control.servos = [90, 90]
			control.pump = False
			self.motor_pub.publish(control)
			return

		now = self.get_clock().now()
		age = (now - self.last_pad_time).nanoseconds / 1e9
		if age > self.failsafe_timeout:
			control.wheels = [127, 127]
			control.servos = [90, 90]
			control.pump = False
			self.motor_pub.publish(control)
			return

		msg = self.last_msg

		left_x = self._apply_deadzone(self._to_normalized(msg.left_analog_x))
		left_y = self._apply_deadzone(self._to_normalized(msg.left_analog_y))

		# Differential drive mix for 2-wheel robot.
		throttle = -left_y
		turn = left_x
		left_wheel = max(-1.0, min(1.0, throttle + turn))
		right_wheel = max(-1.0, min(1.0, throttle - turn))

		servo1_norm = self._to_normalized(msg.right_analog_y)
		servo1_pos = int(
			round(
				self.servo1_min
				+ ((servo1_norm + 1.0) * 0.5) * (self.servo1_max - self.servo1_min)
			)
		)
		servo1_pos = max(self.servo1_min, min(self.servo1_max, servo1_pos))

		# Servo2 compensates servo1 motion to keep camera near perpendicular.
		servo1_center = (self.servo1_min + self.servo1_max) / 2.0
		servo2_pos = int(
			round(
				self.servo2_center
				+ self.servo2_compensation_sign
				* self.servo2_compensation_gain
				* (servo1_pos - servo1_center)
			)
		)
		servo2_pos = max(0, min(180, servo2_pos))

		control.wheels = [self._to_uint8(left_wheel), self._to_uint8(right_wheel)]
		control.servos = [servo1_pos, servo2_pos]
		control.pump = self.pump_enabled

		self.motor_pub.publish(control)


def main(args=None):
	rclpy.init(args=args)
	node = PestibotManualControl()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
