import math

import rclpy
from example_interfaces.msg import String
from example_interfaces.msg import Float32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from webots_ros2_simulations_interfaces.msg import MoveStatus

def euclidean_distance(p1, p2, _2d = False):
    dx_sq = (p1.x - p2.x)**2
    dy_sq = (p1.y - p2.y)**2
    dz_sq = 0.0 if _2d else (p1.z - p2.z)**2
    return math.sqrt(dx_sq + dy_sq + dz_sq)

ON_TARGET_EPS = 0.032

NUM_MOTORS = 9
NUM_POS_MOTORS = 6 

Z1 = 0.0
Z2 = 5.4
ZA = -25.3
ZB = 6.4
ZC = 37.8
X1 = 0.8
X2 = 0.0
XA = -1.12
XB = -1.12
XC = -1.12
ALPHA1 = +1.5708
ALPHA2 = +1.5708
ALPHAA = +3.1416
ALPHAB = +3.1416
ALPHAC = +3.1416
HI = 0.0
LO = 0.72
OPEN = 0.00
CLOSE = 0.09


ROBOT_NAME = 'gantry'

MOTOR_NAMES = [
    'wheel1_motor',
    'wheel2_motor',
    'wheel3_motor',
    'wheel4_motor',
    'bridge_motor',
    'turret_motor',
    'lift_motor',
    'grip_motor1',
    'grip_motor2'
]

MOTOR_POSES = {
    'start': [Z1, Z1, Z1, Z1, X2, ALPHA1],
    'base1': [Z1, Z1, Z1, Z1, X1, ALPHA1],
    'base2': [Z2, Z2, Z2, Z2, X2, ALPHA2],
    'basea': [ZA, ZA, ZA, ZA, XA, ALPHAA],
    'baseb': [ZB, ZB, ZB, ZB, XB, ALPHAB],
    'basec': [ZC, ZC, ZC, ZC, XC, ALPHAC],
}

GPS_POSES = {
    'start': Point(x=0.0, y=0.0, z=0.0),
    'base1': Point(x=0.0, y=-0.8, z=0.02),
    'base2': Point(x=-0.4, y=0.0, z=0.02),
    'basea': Point(x=2.0, y=0.8, z=0.02),
    'baseb': Point(x=-0.5, y=0.8, z=0.02),
    'basec': Point(x=-3.0, y=0.8, z=0.02),
}

GRIPPER_POSES = {
    'high': HI,
    'low': LO
}

GRIPPER_STATUS = {
    'open': OPEN,
    'close': CLOSE
}

class GantryRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motors = []
        for i in range(0, NUM_MOTORS):
            self.__motors.append(self.__robot.getDevice(MOTOR_NAMES[i]))

        self.__target_pose_name = 'start'
        self.__current_pose_name = 'start'
        self.__gripper_pose = GRIPPER_POSES['high']
        self.__gripper_status = GRIPPER_STATUS['open']

        self.__current_gps_pose = Point()

        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('my_'+ROBOT_NAME+'_driver')
        self.__node.create_subscription(String, '/'+ROBOT_NAME+'/cmd_motors_pose', self.__cmd_motors_pose_callback, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(String, '/'+ROBOT_NAME+'/cmd_gripper_pose', self.__cmd_grippers_pose_callback, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(String, '/'+ROBOT_NAME+'/cmd_gripper_status', self.__cmd_grippers_status_callback, rclpy.qos.QoSProfile(depth=1, reliability=1))
        self.__node.create_subscription(PointStamped, '/'+ROBOT_NAME+'_driver/bridge_motor_gps', self.__callback_bridge_motor_gps, rclpy.qos.QoSProfile(depth=2, reliability=2))

        self.__move_status_publisher_ = self.__node.create_publisher(MoveStatus, '/'+ROBOT_NAME+'/motors_move_status', rclpy.qos.QoSProfile(depth=2, reliability=1))

        self.__node.get_logger().info(self.__robot.getName() + ' driver support node set up')
        
    def __callback_bridge_motor_gps(self, new_gps_point):
        #self.__node.get_logger().info("GPS bridge motor type={}: x={}, y={}, z={}".format(type(new_gps_point.point), new_gps_point.point.x, new_gps_point.point.y, new_gps_point.point.z))
        self.__current_gps_pose = new_gps_point.point
        if(self.__target_pose_name in GPS_POSES):
            msg_move_status = MoveStatus()
            msg_move_status.start_name = self.__current_pose_name
            msg_move_status.target_name = self.__target_pose_name
            msg_move_status.current_pos = self.__current_gps_pose
            msg_move_status.target_pos = GPS_POSES[self.__target_pose_name]
            
            if self.__current_pose_name != self.__target_pose_name:
                ed = euclidean_distance(self.__current_gps_pose, GPS_POSES[self.__target_pose_name], _2d=True)
                init_ed = euclidean_distance(GPS_POSES[self.__current_pose_name], GPS_POSES[self.__target_pose_name], _2d=True)
                msg_move_status.progress = (init_ed-ed)/init_ed
                msg_move_status.progress = msg_move_status.progress if msg_move_status.progress >= 0 and msg_move_status.progress <= 1.0 else 0.0
                msg_move_status.progress = 1.0 if ed < ON_TARGET_EPS else msg_move_status.progress
                msg_move_status.euclidean_dist = ed
            
            else:
                msg_move_status.progress = 1.0
            
            self.__move_status_publisher_.publish(msg_move_status)


    def __cmd_motors_pose_callback(self, new_motors_pose):
        self.__target_pose_name = new_motors_pose.data
    
    def __cmd_grippers_pose_callback(self, new_gripper_pose):
        if(new_gripper_pose.data in GRIPPER_POSES):
            self.__gripper_pose = GRIPPER_POSES[new_gripper_pose.data]

    def __cmd_grippers_status_callback(self, new_gripper_status):
        if(new_gripper_status.data in GRIPPER_STATUS):
            self.__gripper_status = GRIPPER_STATUS[new_gripper_status.data]

    def move_to_target(self):
        if(self.__target_pose_name in MOTOR_POSES):
            target_pose = MOTOR_POSES[self.__target_pose_name]

            for i in range(0, NUM_POS_MOTORS):
                #self.__node.get_logger().info("Moving {} to {}".format(MOTOR_NAMES[i], target_pose[i]))
                self.__motors[i].setPosition(float(target_pose[i]))

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # move wheel motors to target position
        if self.__target_pose_name != self.__current_pose_name:
            self.move_to_target()
        
        #move gripper
        self.__motors[6].setPosition(self.__gripper_pose)
        self.__motors[7].setPosition(self.__gripper_status)
        self.__motors[8].setPosition(self.__gripper_status)

        
