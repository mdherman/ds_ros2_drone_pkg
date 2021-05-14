from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode

from ds_ros2_msgs.msg import DroneControl
from ds_ros2_msgs.msg import TrajectorySetpoint as TrajectorySetpointDS

import rclpy
from rclpy.node import Node
import os


class PX4OffboardControl(Node):
	def __init__(self):
		# Init node with name
		super().__init__("drone_offboard_control")

		# Creating publishers
		self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "OffboardControlMode_PubSubTopic", 10)
		self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", 10)
		self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "VehicleCommand_PubSubTopic", 10)

		# Creating subscribers
		self.timesync_subcriber_ = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesync, 10)
		self.use_drone_setpoint_subscriber_ = self.create_subscription(TrajectorySetpointDS, "use_drone_setpoint", self.fetch_trajectory_setpoint, 10)
		self.control_subscriber_ = self.create_subscription(DroneControl, "use_drone_control", self.drone_control, 10)

		# Member trajectory setpoint message
		self.trajectory_msg_ = TrajectorySetpoint()

		# Member variables
		self.timestamp_ = 0

		# Flags
		self.arm_ = False
		self.land_ = True
		self.launch_ = False
		self.armed_ = False
		self.switch_px_ = False
		self.px_status_ = False

		# Timer running at 20 Hz
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.timer_callback)

	# Spinning function
	def timer_callback(self):
		# This arms the drone
		if self.arm_ == True and self.armed_ == False:
			self.arm_vehicle()
			self.set_offboard_mode()
			self.armed_ = True

		# This disarms the drone
		if self.arm_ == False and self.armed_ == True:
			self.disarm_vehicle()
			self.armed_ = False
			self.launch_ = False
		# This launches the drone
		if self.launch_ == True self.armed_ == True:
			self.publish_offboard_control_mode()

		self.trajectory_setpoint_publisher_.publish(self.trajectory_msg_)

		# This lands the drone
		if self.land_ == True:
			self.trajectory_msg_.z = float("NaN")
			self.trajectory_msg_.vz = 0.2

		# This switches the pixhawk
		if self.switch_px_ == True and self.px_status_ == False:
			os.system("sudo sh -c 'echo '1' > /sys/class/gpio/gpio27/value'")
			self.px_status_ = True
		elif self.switch_px_ == False and self.px_status_ == True:
			os.system("sudo sh -c 'echo '0' > /sys/class/gpio/gpio27/value'")
			self.px_status_ = False

	# System control
	def drone_control(self, control_msg):
		self.arm_ = control_msg.arm
		self.land_ = control_msg.land
		self.switch_px_ = control_msg.switch_px
		self.launch_ = control_msg.launch

	# Fetch setpoints
	def fetch_trajectory_setpoint(self, use_msg):
		self.trajectory_msg_.x = use_msg.x
		self.trajectory_msg_.y = use_msg.y
		self.trajectory_msg_.z = use_msg.z
		self.trajectory_msg_.yaw = use_msg.yaw
		self.trajectory_msg_.vx = use_msg.vx
		self.trajectory_msg_.vy = use_msg.vy
		self.trajectory_msg_.vz = use_msg.vz
		self.trajectory_msg_.acceleration = use_msg.acceleration
		self.trajectory_msg_.jerk = use_msg.jerk
		self.trajectory_msg_.thrust = use_msg.thrust

	# Fetch timestamp
	def timesync(self, px4_time):
		self.timestamp_ = px4_time.timestamp
		
	# Sets mode to offboard control.
	def set_offboard_mode(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6) # Control modes..
		self.get_logger().info("Drone set to offboard mode")

	# Send a command to Arm the vehicle
	def arm_vehicle(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
		self.get_logger().info("Arm command send")

	# Send a command to Disarm the vehicle
	def disarm_vehicle(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
		self.get_logger().info("Disarm command send")

	# Publish the offboard control mode.
	def publish_offboard_control_mode(self):
		msg = OffboardControlMode()

		msg.timestamp = self.timestamp_
		msg.position = True
		msg.velocity = True
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False

		self.offboard_control_mode_publisher_.publish(msg)

	# Publish vehicle command
	def publish_vehicle_command(self, command, param1, param2):
		msg = VehicleCommand()

		msg.timestamp = self.timestamp_
		msg.param1 = float(param1)
		msg.param2 = float(param2)
		msg.command = command
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True

		self.vehicle_command_publisher_.publish(msg)



def main(args=None):
	rclpy.init(args=args)

	px4_offboard_control = PX4OffboardControl()

	rclpy.spin(px4_offboard_control)

	px4_offboard_control.destroy_node()

	rclpy.shutdown()


if __name__ == "__main__":
	main()
