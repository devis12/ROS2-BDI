#!/usr/bin/env python3

import os.path
import json
import rclpy
from rclpy.node import Node

from .tk_litter_world_thread import TkLitterWorldThread

from litter_world_interfaces.msg import CmdPose
from litter_world_interfaces.msg import GridRowStatus
from litter_world_interfaces.msg import GridStatus   

from .cell_types import PLASTIC_AGENT_CELL, PAPER_AGENT_CELL

class LitterWorldROS2Controller(Node):

    def __init__(self):
        super().__init__("litter_world") 
        self.counter_ = 0
        self.declare_parameter("init_world", "{}")
        self.declare_parameter("upd_interval", 3000)
    
    def get_upd_interval(self):
        return self.get_parameter("upd_interval").value
    
    def get_init_world_json(self):
        init_world_filepath = self.get_parameter("init_world").value
        if not os.path.isfile(init_world_filepath):
            self.get_logger().error('Init file {} does not exist.'.format(init_world_filepath))
            self.init_world_ = None
        else:
            self.init_world_ = json.loads(''.join(open(init_world_filepath, "r").readlines()))
        
        return self.init_world_

    def init_ros2_controller(self, tk_litter_world_thread):
        self.tk_litter_world_thread_ = tk_litter_world_thread
        self.litter_world_status_publisher_ = self.create_publisher(GridStatus, "litter_world_status", 10) # create topic to periodically notify about grid map status
        self.plastic_agent_cmd_pose_ = self.create_subscription(CmdPose, "cmd_plastic_agent", self.callback_cmd_plastic_agent, 10)
        self.paper_agent_cmd_pose_ = self.create_subscription(CmdPose, "cmd_paper_agent", self.callback_cmd_paper_agent, 10)
        self.create_timer(0.25, self.litter_world_status_cb) #4Hz
        
        self.get_logger().info("Litter world has been started")
    
    def callback_cmd_plastic_agent(self, msg):
        self.tk_litter_world_thread_.move_agent(PLASTIC_AGENT_CELL, msg)

    def callback_cmd_paper_agent(self, msg):
        self.tk_litter_world_thread_.move_agent(PAPER_AGENT_CELL, msg)

    def litter_world_status_cb(self):
        litter_world_grid = self.tk_litter_world_thread_.get_litter_world_status()
        
        litter_world_grid_msg = GridStatus()
        for i in range(0, len(litter_world_grid)):
            litter_world_row_msg = GridRowStatus()
            litter_world_row_msg.index = i
            for j in range(0, len(litter_world_grid[0])):
                litter_world_row_msg.cells.append(litter_world_grid[i][j])
            litter_world_grid_msg.rows.append(litter_world_row_msg)

        self.litter_world_status_publisher_.publish(litter_world_grid_msg)


def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    
    # Create rclpy node to handle ROS2 "frontend"
    node = LitterWorldROS2Controller()
    upd_interval = node.get_upd_interval()
    init_world_json = node.get_init_world_json()

    if init_world_json != None:
        # Create new thread to handle tk app
        tk_litter_world_thread = TkLitterWorldThread(init_world_json, upd_interval)
        
        node.init_ros2_controller(tk_litter_world_thread)

        # Start new Threads
        tk_litter_world_thread.start()
        rclpy.spin(node) # allow node to continue to be alive

    rclpy.shutdown() # last line of any ROS2 .py node
    tk_litter_world_thread.join()

if __name__ == "__main__":
    main()