import math

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from example_interfaces.msg import String
from webots_ros2_simulations_interfaces.msg import MoveStatus

ROBOT_DEFAULT_NAME = 'carrier_x'

ON_TARGET_HIGH_PREC_EPS = 0.06
ON_TARGET_LOW_PREC_EPS = 0.36

MAX_VEL = 12.0
MIN_VEL = 2.0

TARGETS = {
    'base_y': 1.08,
    'deposit_y': 4.28,
    'moving_y' : 4.28 # updated at run time
}

class CarrierRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        id_robot = self.__robot.getName()[len(self.__robot.getName())-1]
        
        self.__motors = [] 
        for i in range(1,5):    
            self.__motors.append(self.__robot.getDevice('c{}_wheel{}'.format(id_robot, i)))
            self.__motors[i-1].setPosition(float('inf'))
            self.__motors[i-1].setVelocity(0)

        # self.__target_twist = Twist()
        self.__start_wp = 'deposit'
        self.__current_wp = 'deposit'
        self.__target_wp = 'deposit'
        self.__current_gps = Point()
        self.__current_vel = 0.0
        
        if not rclpy.ok():
            rclpy.init(args=None)    
        self.__node = rclpy.create_node('my_'+self.__robot.getName()+'_driver')
        # self.__node.create_subscription(Twist, '/'+self.__robot.getName()+'/cmd_vel', self.__cmd_vel_callback, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(String, '/'+self.__robot.getName()+'/cmd_target', self.__cmd_target, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(PointStamped, '/'+self.__robot.getName()+'_driver/gps', self.__callback_carrier_gps, rclpy.qos.QoSProfile(depth=2, reliability=2))
        self.__move_status_publisher_ = self.__node.create_publisher(MoveStatus, '/'+self.__robot.getName()+'/move_status', rclpy.qos.QoSProfile(depth=2, reliability=1))
        
        self.__node.get_logger().info(self.__robot.getName() + ' driver support node set up')

    def __callback_carrier_gps(self, gps_point_stamped):
        self.__current_gps = gps_point_stamped.point
    
    # def __cmd_vel_callback(self, twist):
    #     # self.__node.get_logger().info("Setting velocity to {}".format(self.__target_twist.linear.x))
    #     self.__target_twist = twist
        
    def __cmd_target(self, target):
        new_target = target.data
        if new_target + '_y' in TARGETS and new_target != self.__target_wp: 
               
            if self.__current_wp == 'moving':
                self.__start_wp = 'moving'
                TARGETS['moving' + '_y'] = self.__current_gps.y
                
            self.__node.get_logger().info("Setting target to {}".format(new_target))
            self.__target_wp = new_target

    def move_toward_target(self):
        
        forward_speed = self.__current_vel # default speed value is zero -> carrier stands still in its position
        
        if self.__target_wp + '_y' in TARGETS:
            distance_from_target = self.__current_gps.y - TARGETS[self.__target_wp + '_y'] # check distance from target wp (just on y)
            
            if abs(distance_from_target) > ON_TARGET_HIGH_PREC_EPS: # in case it's greater than eps, move toward it by updating the forward speed
                if forward_speed == 0.0:
                    forward_speed = MIN_VEL
                    
                # different velocities based on the distance (if you're near slow down)
                forward_speed *= 1.2 if abs(distance_from_target) > ON_TARGET_LOW_PREC_EPS else 0.9
                forward_speed = -abs(forward_speed) if distance_from_target > 0 else abs(forward_speed)
                # don't go over vel. thresholds
                if abs(forward_speed) > abs(MAX_VEL):
                    forward_speed = MAX_VEL if forward_speed > 0 else -MAX_VEL
                if abs(forward_speed) < abs(MIN_VEL):
                    forward_speed = MIN_VEL if forward_speed > 0 else -MIN_VEL 
            else:
                forward_speed = 0.0
        
        for i in range(0,4):
            self.__motors[i].setVelocity(forward_speed)
        
        self.__current_vel = forward_speed
        # self.__node.get_logger().info("Current distance from target {} equals {}, setting velocity to {}".format(
        #     self.__target_wp, distance_from_target, self.__current_vel))
    
    def compute_progress(self):
        msg_move_status = MoveStatus()
        msg_move_status.start_name = self.__start_wp
        msg_move_status.current_name = self.__current_wp
        msg_move_status.target_name = self.__target_wp
        msg_move_status.current_pos = self.__current_gps
        msg_move_status.target_pos = Point()
        msg_move_status.target_pos.x = self.__current_gps.x
        msg_move_status.target_pos.y = TARGETS[self.__target_wp + '_y']
        msg_move_status.target_pos.z = self.__current_gps.z
        
        init_distance = abs(TARGETS[self.__target_wp + '_y'] - TARGETS[self.__start_wp + '_y'])
        curr_distance = abs(TARGETS[self.__target_wp + '_y'] - self.__current_gps.y)
        
        progress = (init_distance - curr_distance) / init_distance if init_distance > 0 else 1.0
        progress = progress if curr_distance > ON_TARGET_HIGH_PREC_EPS else 1.0
        
        msg_move_status.dist = init_distance 
        msg_move_status.progress = progress
        
        # self.__node.get_logger().info("Current distance from target {} starting from {} equals {}, tot distance = {} (progress = {})".format(
        #     self.__start_wp, self.__target_wp, curr_distance, init_distance, progress))
        
        if (progress == 1.0):
            self.__start_wp = self.__target_wp
            self.__current_wp = self.__target_wp
            msg_move_status.current_name = self.__current_wp
            
        self.__move_status_publisher_.publish(msg_move_status)
    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        if self.__current_wp != self.__target_wp:
            self.__current_wp = 'moving'
        
        self.move_toward_target()
        self.compute_progress()
        