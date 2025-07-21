# ROS2 packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool, Int64
from rclpy.qos import QoSProfile

# System packages
import numpy as np

# Local packages
from joystick_control.qos_profiles import qos


class JoystickController(Node):
    def __init__(self):
        """
        Map joystick axis reading to linear/angular velocities, then map to thruster values
        """
        super().__init__('joystick_controller')

        # Subscribers
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, qos)
        # Publishers
        self.linear_vel_publisher = self.create_publisher(Float64, '/wamv/linear_velocity', qos)
        self.angular_vel_publisher = self.create_publisher(Float64, '/wamv/angular_velocity', qos)
        self.left_thruster_publisher = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', QoSProfile(depth=10))
        self.right_thruster_publisher = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', QoSProfile(depth=10))
        self.left_rudder_publisher = self.create_publisher(Float64, '/wamv/thrusters/left/pos', qos)
        self.right_rudder_publisher = self.create_publisher(Float64, '/wamv/thrusters/right/pos', qos)
        self.joystick_takingover_publisher = self.create_publisher(Bool, 'joystick_takingover', qos)
        self.joystick_rls_enable_publisher = self.create_publisher(Bool, '/joystick_rls_enable', qos)
        self.joystick_rls_reset_publisher = self.create_publisher(Bool, '/joystick_rls_reset', qos)
        self.save_pose_publisher = self.create_publisher(Bool, 'save_pose', qos)
        self.mode_publisher = self.create_publisher(Int64, 'linc_mode', qos)

        # Parameters
        self.declare_parameter('enable_rudder', False)
        self.declare_parameter('idle_pub_zero_cmd', True)
        self.declare_parameter('linear_axis', 1)  # left vertical axis
        self.declare_parameter('angular_axis', 2)  # right horizontal axis
        self.declare_parameter('takeover_button', 0)  # button that enables joystick control
        self.declare_parameter('save_pose_button', 3)  # button that enables joystick control
        self.declare_parameter('rls_enable_button', 9)     # button that enables rls
        self.declare_parameter('rls_reset_button', 8)      # button that resets rls estimated weights
        self.declare_parameter('max_thrust', 400.0)  # aligned with torqeedo motor command range
        self.declare_parameter('max_rudder', np.pi / 6)  # +-30 deg
        self.max_thrust = 1000
        self.enable_rudder = False
        self.linc_mode = False
        self.enable_thrust = False
        # Constants
        self.log_output_freq = 1.0
        self.max_linear_vel = 3.0
        self.max_angular_vel = 0.5
        self.create_timer(timer_period_sec=1./ 10, callback=self.pub)

        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.linear_velocity =  0.0
        self.angular_velocity = 0.0

        self.rls_enable = False
        self.rls_reset = False

    def pub(self):

        linear_vel_msg = Float64()
        angular_vel_msg = Float64()
        left_thrust_msg = Float64()
        right_thrust_msg = Float64()
        left_rudder_msg = Float64()
        right_rudder_msg = Float64()
        mode_msg = Int64()


        linear_vel_msg.data = self.linear_velocity
        angular_vel_msg.data = self.angular_velocity
        left_thrust_msg.data = self.left_thrust
        right_thrust_msg.data = self.right_thrust
        left_rudder_msg.data = self.left_pos
        right_rudder_msg.data = self.right_pos
        mode_msg.data = self.linc_mode

        self.linear_vel_publisher.publish(linear_vel_msg)
        self.angular_vel_publisher.publish(angular_vel_msg)
        self.mode_publisher.publish(mode_msg)
        if self.enable_thrust:
            self.left_thruster_publisher.publish(left_thrust_msg)
            self.right_thruster_publisher.publish(right_thrust_msg)
            self.left_rudder_publisher.publish(left_rudder_msg)
            self.right_rudder_publisher.publish(right_rudder_msg)


    def joy_callback(self, msg):
        takeover_button: int = self.get_parameter('takeover_button').get_parameter_value().integer_value
        rls_enable_button: int = self.get_parameter('rls_enable_button').get_parameter_value().integer_value
        rls_enable: bool = msg.buttons[rls_enable_button]
        rls_reset_button: int = self.get_parameter('rls_reset_button').get_parameter_value().integer_value
        rls_reset: bool = msg.buttons[rls_reset_button]
        takeover_enabled: bool = msg.buttons[takeover_button]
        save_pose_button: int = self.get_parameter('save_pose_button').get_parameter_value().integer_value
        save_pose: bool = msg.buttons[save_pose_button]
        idle_pub_zero: bool = self.get_parameter('idle_pub_zero_cmd').get_parameter_value().bool_value

        takingover_msg = Bool()
        save_pose_msg = Bool()
        rls_enable_msg = Bool()
        rls_reset_msg = Bool()

        if not takeover_enabled:
            if idle_pub_zero:
                self.pub()  # publish zero commands
                self.get_logger().warn('Joystick inactive, deadman switch is not pulled, publishing zero commands ...',
                                       throttle_duration_sec=5)
            else:
                takingover_msg.data = False
                self.joystick_takingover_publisher.publish(takingover_msg)
                self.get_logger().warn('Joystick inactive, deadman switch is not pulled, publishing no commands ...',
                                       throttle_duration_sec=5)
            self.make_global(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.enable_thrust = False

            return
        else:
            takingover_msg.data = True
            self.joystick_takingover_publisher.publish(takingover_msg)
        if save_pose:
            save_pose_msg.data = True
            self.save_pose_publisher.publish(save_pose_msg)
        else:
            save_pose_msg.data = False
            self.save_pose_publisher.publish(save_pose_msg)

        if msg.buttons[5] :  # or msg.buttons[8]
            self.linc_mode = 1
            self.enable_rudder = 1
        elif msg.buttons[-4] : # or msg.buttons[-3]
            self.linc_mode = 2
            self.enable_rudder = 0
        elif msg.buttons[-5] :  # or msg.buttons[-2]
            self.linc_mode = 3
            self.enable_rudder = 1
        elif msg.buttons[4] :  # or msg.buttons[-7]
            self.linc_mode = 0
            self.enable_rudder = 0
        
        self.enable_thrust: bool = msg.buttons[0]

        # RLS Enable Logic
        if rls_enable == True:
            self.rls_enable = not self.rls_enable
        rls_enable_msg.data = True if self.rls_enable else False
        self.joystick_rls_enable_publisher.publish(rls_enable_msg)
        # RLS Reset Logic
        if rls_reset == True:
            rls_reset_msg.data = True
        else:
            rls_reset_msg.data = False
        self.joystick_rls_reset_publisher.publish(rls_reset_msg)
        #
        #self.get_logger().info(f'[RLS_Enable,RLS_Reset] [{self.rls_enable},{self.rls_reset}]')

        # print("linc_mode",self.linc_mode)
        if self.enable_rudder:
            self.get_logger().info('Rudder is enabled.', once=True)
        else:
            self.get_logger().info('Rudder is not enabled.', once=True)

        # Axis reading are in [-1, 1]
        linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value

        # Thrust range is [-1000, 1000]
        max_velocity = self.max_linear_vel*(msg.axes[3]+1)/2
        max_angular_vel = (msg.axes[3]+1)/2*self.max_angular_vel
        max_thrust = self.max_thrust * (msg.axes[3]+1)/2
        max_rudder = self.get_parameter('max_rudder').get_parameter_value().double_value

        # actually is not the real velocity in m/s, but the percentage of the boat's maximum surge speed
        linear_velocity = msg.axes[linear_axis]
                # still the percentage, but is inherently limited within [-1, 1] rad/s due to axis reading range
        angular_velocity = msg.axes[angular_axis]

        if self.enable_rudder:
            left_thrust = linear_velocity * max_thrust
            right_thrust = linear_velocity * max_thrust
            left_pos = -angular_velocity * max_rudder
            right_pos = -angular_velocity * max_rudder
        else:
            # Assume perpendicular distance from any thruster to boat's central line is 1 meter
            left_thrust = linear_velocity * max_thrust - angular_velocity * max_thrust
            right_thrust = linear_velocity * max_thrust + angular_velocity * max_thrust
            left_pos = 0.0
            right_pos = 0.0

        """ self.get_logger().info(f'Linear: {linear_velocity:.1f}, Angular: {angular_velocity:.1f}, '
                               f'Thrust L: {left_thrust:.1f}, Thrust R: {right_thrust:.1f}, '
                               f'Rudder L: {left_pos:.1f}, Rudder R: {right_pos:.1f}',
                               throttle_duration_sec=1 / self.log_output_freq) """

        self.make_global(left_thrust, right_thrust, left_pos, right_pos, max_velocity * linear_velocity, max_angular_vel * angular_velocity)
    
    def make_global(self, left_thrust, right_thrust, left_pos, right_pos, linear_velocity, angular_velocity):
        self.left_thrust = left_thrust
        self.right_thrust = right_thrust
        self.left_pos = left_pos
        self.right_pos = right_pos
        self.linear_velocity =  linear_velocity
        self.angular_velocity = angular_velocity
        #self.get_logger().info(f'self.linear_velocity,{self.linear_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
