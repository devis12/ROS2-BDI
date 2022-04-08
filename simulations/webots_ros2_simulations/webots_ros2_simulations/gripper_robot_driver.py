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

X1 = 0.11
X2 = 5.16
X3 = 10.22
XA = -25.3
XB = 6.4
XC = 37.8

Y1 = 0.9
Y2 = 0.0
Y3 = Y1
YA = -1.08
YB = -1.08
YC = -1.08

ALPHA1 = +1.5708
ALPHA2 = +1.5708
ALPHA3 = +1.5708
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
    'start': [X1, X1, X1, X1, Y2, ALPHA1],
    'base_1': [X1, X1, X1, X1, Y1, ALPHA1],
    'base_2': [X2, X2, X2, X2, Y2, ALPHA2],
    'base_3': [X3, X3, X3, X3, Y3, ALPHA3],
    'base_a': [XA, XA, XA, XA, YA, ALPHAA],
    'base_b': [XB, XB, XB, XB, YB, ALPHAB],
    'base_c': [XC, XC, XC, XC, YC, ALPHAC],
}

GPS_POSES = {
    'start': Point(x=0.0, y=Y2, z=0.0),
    'base_1': Point(x=0.0, y=-Y1, z=0.02),
    'base_2': Point(x=-0.4, y=Y2, z=0.02),
    'base_3': Point(x=-0.8, y=Y1, z=0.02),
    'base_a': Point(x=2.0, y=-YA, z=0.02),
    'base_b': Point(x=-0.5, y=-YB, z=0.02),
    'base_c': Point(x=-3.0, y=-YC, z=0.02),
    'moving': Point(x=0.0, y=Y2, z=0.0) # updated at run time
}

GRIPPER_POSES = {
    'high': HI,
    'low': LO
}

GRIPPER_STATUS = {
    'open': OPEN,
    'close': CLOSE
}

class GripperRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        ROBOT_NAME = self.__robot.getName()
        
        self.__motors = []
        for i in range(0, NUM_MOTORS):
            self.__motors.append(self.__robot.getDevice(MOTOR_NAMES[i]))

        self.__target_pose_name = 'start'
        self.__start_pose_name = 'start'
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
            msg_move_status.start_name = self.__start_pose_name
            msg_move_status.current_name = self.__current_pose_name
            msg_move_status.target_name = self.__target_pose_name
            msg_move_status.current_pos = self.__current_gps_pose
            msg_move_status.target_pos = GPS_POSES[self.__target_pose_name]
            
            if self.__current_pose_name != self.__target_pose_name:
                ed = euclidean_distance(self.__current_gps_pose, GPS_POSES[self.__target_pose_name], _2d=True)
                init_ed = euclidean_distance(GPS_POSES[self.__start_pose_name], GPS_POSES[self.__target_pose_name], _2d=True)
                msg_move_status.progress = (init_ed-ed)/init_ed
                msg_move_status.progress = msg_move_status.progress if msg_move_status.progress >= 0 and msg_move_status.progress <= 1.0 else 0.0
                msg_move_status.progress = 1.0 if ed < ON_TARGET_EPS else msg_move_status.progress
                msg_move_status.euclidean_dist = ed
            
            else:
                msg_move_status.progress = 1.0
            
            self.__move_status_publisher_.publish(msg_move_status)

            if msg_move_status.progress >= 1.0:
                self.__current_pose_name = self.__target_pose_name

    def __cmd_motors_pose_callback(self, new_motors_pose):
        if(self.__target_pose_name != new_motors_pose.data):#new target
            self.__start_pose_name = self.__current_pose_name
            if self.__start_pose_name == 'moving':
                GPS_POSES['moving'] = self.__current_gps_pose
                
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
            self.__current_pose_name = 'moving'
            self.move_to_target()
        
        #move gripper
        self.__motors[6].setPosition(self.__gripper_pose)
        self.__motors[7].setPosition(self.__gripper_status)
        self.__motors[8].setPosition(self.__gripper_status)

        
