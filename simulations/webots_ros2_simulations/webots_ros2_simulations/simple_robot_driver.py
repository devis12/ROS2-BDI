import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MySimpleRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        '''
        self.__left_motor1 = self.__robot.getDevice('left wheel motor 1')
        self.__right_motor1 = self.__robot.getDevice('right wheel motor 1')

        self.__left_motor1.setPosition(float('inf'))
        self.__left_motor1.setVelocity(0)

        self.__right_motor1.setPosition(float('inf'))
        self.__right_motor1.setVelocity(0)

        self.__target_twist1 = Twist()
        '''
        self.__left_motor2 = self.__robot.getDevice('left wheel motor 2')
        self.__right_motor2 = self.__robot.getDevice('right wheel motor 2')

        self.__left_motor2.setPosition(float('inf'))
        self.__left_motor2.setVelocity(0)

        self.__right_motor2.setPosition(float('inf'))
        self.__right_motor2.setVelocity(0)

        self.__target_twist2 = Twist()
        
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        #self.__node.create_subscription(Twist, 'cmd_vel1', self.__cmd_vel_callback1, 1)
        self.__node.create_subscription(Twist, 'cmd_vel2', self.__cmd_vel_callback2, 1)

        self.__node.get_logger().info("type webots_node {}".format(type(webots_node)))
        self.__node.get_logger().info("type webots_node.robot {}".format(type(webots_node.robot)))
    '''
    def __cmd_vel_callback1(self, twist):
        self.__target_twist1 = twist
    '''
    def __cmd_vel_callback2(self, twist):
        self.__target_twist2 = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        '''
        forward_speed = self.__target_twist1.linear.x
        angular_speed = self.__target_twist1.angular.z

        command_motor_left1 = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right1 = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor1.setVelocity(command_motor_left1)
        self.__right_motor1.setVelocity(command_motor_right1)
        '''
        forward_speed = self.__target_twist2.linear.x
        angular_speed = self.__target_twist2.angular.z

        command_motor_left2 = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right2 = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor2.setVelocity(command_motor_left2)
        self.__right_motor2.setVelocity(command_motor_right2)
        