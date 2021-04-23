from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from ds_ros2_msgs.msg import DroneControl

import rclpy
from rclpy.node import Node


class PX4OffboardControl(Node):
	def __init__(self):
		super().__init__("px4_offboard_control")

		# Creating publishers
		self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "OffboardControlMode_PubSubTopic", 10)
		self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", 10)
		self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "VehicleCommand_PubSubTopic", 10)

		# Creating subscribers
		self.timesync_sub_ = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesync, 10)
		self.use_drone_setpoint_sub = self.create_subscription(TrajectorySetpoint, "use_drone_setpoint", self.fetch_trajectory_setpoint, 10)
		self.activate_drone_01 = self.create_subscription(DroneControl, "acticate_topics_msgs", self.drone_control, 10)

		# Setting member and locale variables
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.yaw = 0.0
		self.timestamp = 0
		timer_period = 0.1

		# Getting ready to fly
		#self.arm() # Arming
		#self.publish_offboard_control_mode() # Control modes..
		#self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6) # Control modes..

		# Running
		self.timer = self.create_timer(timer_period, self.timer_callback)

	# Activating topics
	def drone_control(self, activate_msg):
		self.arm = activate_msg.arm
		self.offboard_control = activate_msg.offboard_control

	# Fetch setpoints
	def fetch_trajectory_setpoint(self, use_msg):
		self.x = use_msg.x
		self.y = use_msg.y
		self.z = use_msg.z
		self.yaw = use_msg.yaw

	# Spinning function
	def timer_callback(self):
		#self.publish_offboard_control_mode()
		#self.publish_trajectory_setpoint
		if self.arm == true:
			self.arm()
		else:
			self.disarm()




	# Fetch timestamp
	def timesync(self, px4_time):
		self.timestamp = px4_time.timestamp


	# Send a command to Arm the vehicle
	def arm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
		self.get_logger().info("Arm command send")

	# Send a command to Disarm the vehicle
	def disarm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
		self.get_logger().info("Disarm command send")

	# Publish the offboard control mode.
	def publish_offboard_control_mode(self):
		msg = OffboardControlMode()

		msg.timestamp = self.timestamp
		msg.position = True
		msg.velocity = False
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False

		self.offboard_control_mode_publisher_.publish(msg)

	# Publish trajectory setpoint
	def publish_trajectory_setpoint(self):
		msg = TrajectorySetpoint()

		msg.timestamp = self.timestamp
		msg.x = self.x
		msg.y = self.y
		msg.z = self.z
		msg.yaw = self.yaw

		self.trajectory_setpoint_publisher_.publish(msg)

	# Publish vehicle command
	def publish_vehicle_command(self, command, param1, param2):
		msg = VehicleCommand()

		msg.timestamp = self.timestamp
		msg.param1 = float(param1)
		msg.param2 = float(param2)
		msg.command = command
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True

		self.vehicle_command_publisher_.publish(msg)

	# Spinning function
	def timer_callback(self):
		self.publish_offboard_control_mode()
		self.publish_trajectory_setpoint()



def main(args=None):
	rclpy.init(args=args)

	px4_offboard_control = PX4OffboardControl()

	rclpy.spin(px4_offboard_control)

	px4_offboard_control.destroy_node()

	rclpy.shutdown()


if __name__ == "__main__":
	main()