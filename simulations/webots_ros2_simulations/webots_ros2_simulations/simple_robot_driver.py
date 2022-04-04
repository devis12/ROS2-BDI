import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MySimpleRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_id = self.__robot.getName()[len(self.__robot.getName())-1]
        self.__left_motor = self.__robot.getDevice('left_wheel_motor_' + self.__robot_id)
        self.__right_motor = self.__robot.getDevice('right_wheel_motor_' + self.__robot_id)

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__new_target = False
        
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node(self.__robot.getName()+'_driver')
        self.__node.create_subscription(Twist, '/'+self.__robot.getName()+'/cmd_vel', self.__cmd_vel_callback, 1)

        self.__node.get_logger().info("type webots_node {}".format(type(webots_node)))
        self.__node.get_logger().info("type webots_node.robot {}".format(type(webots_node.robot)))

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        self.__new_target = True

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        if self.__new_target:
            self.__new_target = False
            self.__node.get_logger().info("{} target lin.x={}, ang.z={}, setting vel_left={}, vel_right={}".format(
            self.__robot.getName(), 
            self.__target_twist.linear.x, self.__target_twist.angular.z,
            command_motor_left, command_motor_right))
            
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
        