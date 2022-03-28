import rclpy
from example_interfaces.msg import String

NUM_MOTORS = 9
NUM_POS_MOTORS = 6 

Z1 = 0.0
Z2 = 5.0
ZA = 0.0
ZB = 5.0
ZC = 10.0
X1 = 0.8
X2 = 0.0
XA = -0.8
XB = -0.8
XC = -0.8
ALPHA1 = +1.5708
ALPHA2 = +1.5708
ALPHAA = +3.1416
ALPHAB = +3.1416
ALPHAC = +3.1416
HI = 0.0
LO = 0.72
OPEN = 0.00
CLOSE = 0.09

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
    'baseA': [ZA, ZA, ZA, ZA, XA, ALPHAA],
    'baseB': [ZB, ZB, ZB, ZB, XB, ALPHAB],
    'baseC': [ZC, ZC, ZC, ZC, XC, ALPHAC],
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

        self.__target_motors_pose = MOTOR_POSES['start']
        self.__current_motors_pose = MOTOR_POSES['start']
        self.__gripper_pose = GRIPPER_POSES['high']
        self.__gripper_status = GRIPPER_STATUS['open']

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_gantry_driver')
        self.__node.create_subscription(String, 'cmd_motors_pose', self.__cmd_motors_pose_callback, 1)
        self.__node.create_subscription(String, 'cmd_gripper_pose', self.__cmd_grippers_pose_callback, 1)
        self.__node.create_subscription(String, 'cmd_gripper_status', self.__cmd_grippers_status_callback, 1)
        #self.__node.create_subscription(String, 'print_met', self.print_methods, 1)

    def __cmd_motors_pose_callback(self, new_motors_pose):
        self.__target_motors_pose = new_motors_pose.data
    
    def __cmd_grippers_pose_callback(self, new_gripper_pose):
        if(new_gripper_pose.data in GRIPPER_POSES):
            self.__gripper_pose = GRIPPER_POSES[new_gripper_pose.data]

    def __cmd_grippers_status_callback(self, new_gripper_status):
        if(new_gripper_status.data in GRIPPER_STATUS):
            self.__gripper_status = GRIPPER_STATUS[new_gripper_status.data]

    '''
    def print_methods(self, msg):
         for i in range (0, NUM_MOTORS) :
            #print("Motor {} pose: {}".format(MOTOR_NAMES[i], self.__motors[i].getPositionSensorTag()))
            object_methods = [method_name for method_name in dir(self.__motors[i])
                if callable(getattr(self.__motors[i], method_name))]
            print("\nMotor {}".format(MOTOR_NAMES[i]))
            print("methods:")
            for met in object_methods:
                print(met)
    '''
    
    '''
    def open_gripper(self):
        self.__motors[7].setPosition(OPEN)
        self.__motors[8].setPosition(OPEN)
    
    def close_gripper(self):
        self.__motors[7].setPosition(CLOSE)
        self.__motors[8].setPosition(CLOSE)

    def raise_gripper(self):
        self.__motors[6].setPosition(HI)
    
    def lower_gripper(self):
        self.__motors[6].setPosition(LO)
    ''' 

    def move_to_target(self):
        if(self.__target_motors_pose in MOTOR_POSES):
            target_pose = MOTOR_POSES[self.__target_motors_pose]
            for i in range(0, NUM_POS_MOTORS):
                self.__motors[i].setPosition(float(target_pose[i]))

            self.__current_motors_pose = self.__target_motors_pose

            new_pose_msg = String()
            new_pose_msg.data = self.__current_motors_pose
            #self.motors_pose_publisher_.publish(new_pose_msg)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # move wheel motors to target position
        if(self.__target_motors_pose != self.__current_motors_pose):
            self.move_to_target()
        
        #move gripper
        self.__motors[6].setPosition(self.__gripper_pose)
        self.__motors[7].setPosition(self.__gripper_status)
        self.__motors[8].setPosition(self.__gripper_status)

        
