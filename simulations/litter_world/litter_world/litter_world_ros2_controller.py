#!/usr/bin/env python3

import os.path
import json
from urllib import response
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer

from .tk_litter_world_thread import TkLitterWorldThread

from litter_world_interfaces.msg import Pose
from litter_world_interfaces.action import CmdPose
from litter_world_interfaces.msg import GridRowStatus
from litter_world_interfaces.msg import GridStatus   
from litter_world_interfaces.action import CmdLoad

from .cell_types import PAPER_LITTER_CELL, PLASTIC_AGENT_CELL, PAPER_AGENT_CELL, PLASTIC_LITTER_CELL

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

        self.plastic_agent_cmd_pose_ = ActionServer(self, CmdPose, 'cmd_plastic_agent_move', self.callback_cmd_plastic_agent_move)
        self.paper_agent_cmd_pose_ = ActionServer(self, CmdPose, 'cmd_paper_agent_move', self.callback_cmd_paper_agent_move)
        self.plastic_agent_cmd_upd_holding_ = ActionServer(self, CmdLoad, "cmd_plastic_agent_hold", self.callback_cmd_plastic_agent_hold)
        self.paper_agent_cmd_upd_holding_ = ActionServer(self, CmdLoad, "cmd_paper_agent_hold", self.callback_cmd_paper_agent_hold)
        self.remove_litter_cmd_pose_ = ActionServer(self, CmdPose, "cmd_remove_litter", self.callback_cmd_remove_litter)
        
        self.create_timer(0.25, self.litter_world_status_cb) #4Hz
        
        self.get_logger().info("Litter world has been started")
    
    def callback_cmd_plastic_agent_move(self, goal_handle):
        performed = self.tk_litter_world_thread_.move_agent(PLASTIC_AGENT_CELL, goal_handle.request.cmd) #TODO feedback 
        goal_handle.succeed()
        result = CmdPose.Result()
        result.performed = performed
        return result

    def callback_cmd_paper_agent_move(self, goal_handle):
        performed = self.tk_litter_world_thread_.move_agent(PAPER_AGENT_CELL, goal_handle.request.cmd) #TODO feedback 
        goal_handle.succeed()
        result = CmdPose.Result()
        result.performed = performed
        return result

    def callback_cmd_plastic_agent_hold(self, goal_handle):
        performed = False
        holding = -1
        if goal_handle.request.cmd == "load" or goal_handle.request.cmd == "unload":
            holding_upd = 1 if goal_handle.request.cmd == "load" else -1
            performed, holding = self.tk_litter_world_thread_.upd_holding_agent(PLASTIC_AGENT_CELL, holding_upd) #TODO feedback 
        
        goal_handle.succeed()
        result = CmdLoad.Result()
        result.performed = performed
        result.holding = holding
        return result

    def callback_cmd_paper_agent_hold(self, goal_handle):
        performed = False
        holding = -1
        if goal_handle.request.cmd == "load" or goal_handle.request.cmd == "unload":
            holding_upd = 1 if goal_handle.request.cmd == "load" else -1
            performed, holding = self.tk_litter_world_thread_.upd_holding_agent(PAPER_AGENT_CELL, holding_upd) #TODO feedback 
        
        goal_handle.succeed()
        result = CmdLoad.Result()
        result.performed = performed
        result.holding = holding
        return result
    
    def callback_cmd_remove_litter(self, goal_handle):
        performed = self.tk_litter_world_thread_.remove_litter(goal_handle.request.cmd.x, goal_handle.request.cmd.y) #TODO feedback 
        goal_handle.succeed()
        result = CmdPose.Result()
        result.performed = performed
        return result

    def litter_world_status_cb(self):
        world_grid, litter_grid = self.tk_litter_world_thread_.get_litter_world_status()
        
        # use world grid to map agent, bins, static and dynamic obstacles
        litter_world_grid_msg = GridStatus()
        for i in range(0, len(world_grid)):
            litter_world_row_msg = GridRowStatus()
            litter_world_row_msg.index = i
            for j in range(0, len(world_grid[0])):
                litter_world_row_msg.cells.append(world_grid[i][j])
            litter_world_grid_msg.rows.append(litter_world_row_msg)
        
        # use litter_grid to map plastic and paper litter poses
        for i in range(0, len(litter_grid)):
            for j in range(0, len(litter_grid[0])):
                pose_msg = Pose()
                pose_msg.x = i
                pose_msg.y = j
                if litter_grid[i][j] == PLASTIC_LITTER_CELL:
                    litter_world_grid_msg.litter_plastic_poses.append(pose_msg)
                elif litter_grid[i][j] == PAPER_LITTER_CELL:
                    litter_world_grid_msg._litter_paper_poses.append(pose_msg)

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

        mt_executor = MultiThreadedExecutor()
        mt_executor.add_node(node)
        mt_executor.spin()# allow node to continue to be alive
        #rclpy.spin(node) 

    rclpy.shutdown() # last line of any ROS2 .py node
    tk_litter_world_thread.join()

if __name__ == "__main__":
    main()