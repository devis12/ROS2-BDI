import math

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from example_interfaces.msg import String
from webots_ros2_simulations_interfaces.msg import MoveStatus

ROBOT_DEFAULT_NAME = 'carrier_x'

ON_TARGET_SUPER_HIGH_PREC_EPS = 0.02
ON_TARGET_HIGH_PREC_EPS = 0.32
ON_TARGET_LOW_PREC_EPS = 0.64

MAX_VEL = 16.0
MIN_VEL = 1.6

XA = 2.0
XB = -0.5
XC = -3.0

YBASE=1.1
YDEP=4.32

ZGLOB=0.1

TARGETS = {
    'basea': Point(x=XA, y=YBASE, z=ZGLOB),
    'deposita': Point(x=XA, y=YDEP, z=ZGLOB),
    'baseb': Point(x=XB, y=YBASE, z=ZGLOB),
    'depositb': Point(x=XB, y=YDEP, z=ZGLOB),
    'basec': Point(x=XC, y=YBASE, z=ZGLOB),
    'depositc': Point(x=XC, y=YDEP, z=ZGLOB),
    'moving' : Point(x=XA, y=YDEP, z=ZGLOB) # updated at run time
}

def euler_from_quaternion(x, y, z, w):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    # return roll_x, pitch_y, yaw_z # in radians
    return yaw_z
    
class CarrierRobotDriverAdvanced:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        self.__motors = [] 
        for i in range(1,5):    
            self.__motors.append(self.__robot.getDevice('ca_wheel{}'.format(i)))
            self.__motors[i-1].setPosition(float('inf'))
            self.__motors[i-1].setVelocity(0)

        self.__target_twist = Twist()
        self.__start_wp = 'deposita'
        self.__current_wp = 'deposita'
        self.__target_wp = 'deposita'
        self.__current_gps = Point()
        self.__current_vel = 0.0
        self.__current_rad = 3.1416
        
        if not rclpy.ok():
            rclpy.init(args=None)    
        self.__node = rclpy.create_node('my_'+self.__robot.getName()+'_driver')
        self.__node.create_subscription(Twist, '/'+self.__robot.getName()+'/cmd_vel', self.__cmd_vel_callback, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(String, '/'+self.__robot.getName()+'/cmd_target', self.__cmd_target, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(PointStamped, '/'+self.__robot.getName()+'_driver/gps', self.__callback_carrier_gps, rclpy.qos.QoSProfile(depth=2, reliability=2))
        self.__node.create_subscription(Imu, '/'+self.__robot.getName()+'_driver/imu', self.__callback_carrier_imu, rclpy.qos.QoSProfile(depth=2, reliability=2))
        self.__move_status_publisher_ = self.__node.create_publisher(MoveStatus, '/'+self.__robot.getName()+'/move_status', rclpy.qos.QoSProfile(depth=2, reliability=1))
        
        self.__node.get_logger().info(self.__robot.getName() + ' driver support node set up')

    def __callback_carrier_imu(self, imu_feed):
        x = imu_feed.orientation.x
        y = imu_feed.orientation.y
        z = imu_feed.orientation.z
        w = imu_feed.orientation.w
        self.__current_rad = euler_from_quaternion(x, y, z, w)
        #self.__node.get_logger().info('current_angle={}'.format(self.__current_rad))
        
    def __callback_carrier_gps(self, gps_point_stamped):
        self.__current_gps = gps_point_stamped.point
    
    def __cmd_vel_callback(self, twist):
        # self.__node.get_logger().info("Setting velocity to {}".format(self.__target_twist.linear.x))
        self.__target_twist = twist
        
    def __cmd_target(self, target):
        new_target = target.data
        if new_target in TARGETS and new_target != self.__target_wp: 
               
            if self.__current_wp == 'moving':
                self.__start_wp = 'moving'
                TARGETS['moving'] = self.__current_gps
                
            self.__node.get_logger().info("Setting target to {}".format(new_target))
            self.__target_wp = new_target

    def move_toward_target(self):
        forward_speed = 0.0
        ang_speed = 0.0
        
        if self.__target_wp in TARGETS:
            dist_x = self.__current_gps.x - TARGETS[self.__target_wp].x
            dist_y = self.__current_gps.y - TARGETS[self.__target_wp].y
            distance = math.sqrt(dist_x**2 + dist_y**2)
            
            if abs(distance) > ON_TARGET_HIGH_PREC_EPS: 
                forward_speed = self.__current_vel
                
                if forward_speed == 0.0:
                    forward_speed = MIN_VEL
                    
                # different velocities based on the distance (if you're near slow down)
                forward_speed *= 1.6 if abs(distance) > ON_TARGET_LOW_PREC_EPS else 0.8
                # don't go over vel. thresholds
                forward_speed = MAX_VEL if abs(forward_speed) > abs(MAX_VEL) else forward_speed
                forward_speed = MIN_VEL if abs(forward_speed) < abs(MIN_VEL) else forward_speed
                
                if self.__current_gps.x > TARGETS['basea'].x + ON_TARGET_LOW_PREC_EPS or self.__current_gps.x < TARGETS['basec'].x - ON_TARGET_LOW_PREC_EPS:
                    rotating_target = 0.0 if self.__current_gps.x < TARGETS['basec'].x - ON_TARGET_LOW_PREC_EPS else math.pi
                    if abs(abs(self.__current_rad) - rotating_target) > ON_TARGET_HIGH_PREC_EPS:
                        ang_speed = -abs(TARGETS[self.__start_wp].y - TARGETS[self.__target_wp].y) * 1.2
                        forward_speed *= 0.8
                    elif abs(abs(self.__current_rad) - rotating_target) > ON_TARGET_SUPER_HIGH_PREC_EPS:
                        ang_speed = -abs(TARGETS[self.__start_wp].y - TARGETS[self.__target_wp].y) * 0.6
                        forward_speed *= 0.8
            
            elif abs(dist_x) > ON_TARGET_SUPER_HIGH_PREC_EPS:
                forward_speed = MIN_VEL        
                                 
        self.__current_vel_ang = ang_speed
        self.__current_vel = forward_speed  
        self.__target_twist.linear.x = self.__current_vel
        self.__target_twist.angular.z = self.__current_vel_ang
    
    def compute_progress(self):
        msg_move_status = MoveStatus()
        msg_move_status.start_name = self.__start_wp
        msg_move_status.current_name = self.__current_wp
        msg_move_status.target_name = self.__target_wp
        msg_move_status.current_pos = self.__current_gps
        msg_move_status.target_pos = TARGETS[self.__target_wp]
        
        init_dist_x = TARGETS[self.__start_wp].x - TARGETS[self.__target_wp].x
        init_dist_y = TARGETS[self.__start_wp].y - TARGETS[self.__target_wp].y
        init_ed = math.sqrt(init_dist_x**2 + init_dist_y**2)   
        dist_x = self.__current_gps.x - TARGETS[self.__target_wp].x
        dist_y = self.__current_gps.y - TARGETS[self.__target_wp].y
        curr_ed = math.sqrt(dist_x**2 + dist_y**2)   
        progress = (init_ed - curr_ed) / init_ed
        progress = progress if curr_ed > ON_TARGET_HIGH_PREC_EPS or self.__current_vel > 0.0 else 1.0
        
        msg_move_status.euclidean_dist = curr_ed 
        msg_move_status.progress = progress
        
        # self.__node.get_logger().info("Current distance from target {} starting from {} equals {}, tot distance = {} (progress = {})".format(
        #     self.__start_wp, self.__target_wp, curr_distance, init_distance, progress))
        
        if (progress == 1.0):
            self.__start_wp = self.__target_wp
            self.__current_wp = self.__target_wp
            msg_move_status.current_name = self.__current_wp
            self.__target_twist = Twist()
            
        self.__move_status_publisher_.publish(msg_move_status)
    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        
        command_motor_left =  (forward_speed - angular_speed)
        command_motor_right = (forward_speed + angular_speed)
        
        # print("linear.x = {}, angular.z = {}, motor_left = {}, motor_right = {}".format(
        #     round(self.__target_twist.linear.x, 2),round(self.__target_twist.angular.z, 2),
        #     round(command_motor_left, 2), round(command_motor_right, 2)
        # ))
        
        for i in range(0, len(self.__motors)):
            if i % 2 == 0:
                # self.__node.get_logger().info("Motor {} set velocity to {}".format(i, command_motor_right)) 
                self.__motors[i].setVelocity(command_motor_right)
            else:
                # self.__node.get_logger().info("Motor {} set velocity to {}".format(i, command_motor_left)) 
                self.__motors[i].setVelocity(command_motor_left)
        
        if self.__current_wp != self.__target_wp:
            self.__current_wp = 'moving'
            self.move_toward_target()
            self.compute_progress()
        