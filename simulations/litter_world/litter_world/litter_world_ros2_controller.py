#!/usr/bin/env python3

import os.path
import json
import random
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer

from .tk_litter_world_thread import TkLitterWorldThread
from .pose import MGPose
from .move_trajectory import MGMoveTrajectory

from ros2_bdi_interfaces.msg import Belief
from ros2_bdi_interfaces.msg import BeliefSet
from ros2_bdi_interfaces.msg import Desire
from ros2_bdi_interfaces.msg import BDIActionExecutionInfoMin
from ros2_bdi_interfaces.msg import BDIPlanExecutionInfoMin

from litter_world_interfaces.msg import Pose
from litter_world_interfaces.action import CmdPose
from litter_world_interfaces.msg import GridRowStatus
from litter_world_interfaces.msg import GridStatus   
from litter_world_interfaces.action import CmdLoad

from .cell_types import EMPTY_CELL, OBSTACLE_CELL, PAPER_BIN_CELL, PAPER_LITTER_CELL, PLASTIC_AGENT_CELL, PAPER_AGENT_CELL, PLASTIC_BIN_CELL, PLASTIC_LITTER_CELL

class LitterWorldROS2Controller(Node):

    def __init__(self):
        super().__init__("litter_world") 
        self.counter_ = 0
        self.declare_parameter("init_world", "{}")
        self.declare_parameter("upd_interval", 3000)
        self.declare_parameter("show_agent_view", '')
        self.declare_parameter("size", 768)
    
    def get_upd_interval(self):
        return self.get_parameter("upd_interval").value
    
    def get_show_agent_view(self):
        return self.get_parameter("show_agent_view").value
    
    def get_size(self):
        return self.get_parameter("size").value
    
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
        self.plastic_agent_patrol_desire = self.create_publisher(Desire, "/plastic_agent/add_desire", 10) #
        
        if self.get_parameter("show_agent_view").value == 'plastic_agent' or self.get_parameter("show_agent_view").value == 'paper_agent':
            monitoring_agent = self.get_parameter("show_agent_view").value
            self.agent_intention_subscriber_ = self.create_subscription(BDIPlanExecutionInfoMin, "/"+monitoring_agent+"/current_intentions", self.callback_pa_agent_intentions, 10)
            self.agent_bset_subscriber_ = self.create_subscription(BeliefSet, "/"+monitoring_agent+"/belief_set", self.callback_pa_agent_bset, 10)

        self.plastic_agent_cmd_pose_ = ActionServer(self, CmdPose, 'cmd_plastic_agent_move', self.callback_cmd_plastic_agent_move)
        self.paper_agent_cmd_pose_ = ActionServer(self, CmdPose, 'cmd_paper_agent_move', self.callback_cmd_paper_agent_move)
        self.plastic_agent_cmd_upd_holding_ = ActionServer(self, CmdLoad, "cmd_plastic_agent_hold", self.callback_cmd_plastic_agent_hold)
        self.paper_agent_cmd_upd_holding_ = ActionServer(self, CmdLoad, "cmd_paper_agent_hold", self.callback_cmd_paper_agent_hold)
        self.remove_litter_cmd_pose_ = ActionServer(self, CmdPose, "cmd_remove_litter", self.callback_cmd_remove_litter)
        
        self.create_timer(0.25, self.litter_world_status_cb) #4Hz
        
        self.get_logger().info("Litter world has been started")
    
    def callback_pa_agent_bset(self, msg:BeliefSet):
        
        plastic_bin_pose = MGPose(-1,-1)
        paper_bin_pose = MGPose(-1,-1)
        plastic_agent_pose = MGPose(-1,-1)
        paper_agent_pose = MGPose(-1,-1)
        obstacles = []
        paper_litter_poses = []
        plastic_litter_poses = []
        detection_depth = 0

        columns = 0
        rows = 0
        for belief in msg.value:
            if belief.type == 'cell':
                cell_pose = self.extract_pose_from_cell_name(belief.name)
                columns = max(columns, cell_pose.y+1)
                rows = max(rows, cell_pose.x+1)
        
        current_map = [[OBSTACLE_CELL for x in range(columns)] for x in range(rows)]
        
        for belief in msg.value:
            if belief.name == 'detection_depth':
                detection_depth = int(belief.value)
            
            if belief.name == 'bin_pose':
                if belief.params[0] == 'plastic_b':
                    plastic_bin_pose = self.extract_pose_from_cell_name(belief.params[1])
                    current_map[plastic_bin_pose.x][plastic_bin_pose.y] = PLASTIC_BIN_CELL
                elif belief.params[0] == 'paper_b':
                    paper_bin_pose = self.extract_pose_from_cell_name(belief.params[1])
                    current_map[paper_bin_pose.x][paper_bin_pose.y] = PAPER_BIN_CELL
            
            if belief.name == 'in':
                if belief.params[0] == 'plastic_agent':
                    plastic_agent_pose = self.extract_pose_from_cell_name(belief.params[1])
                    current_map[plastic_agent_pose.x][plastic_agent_pose.y] = PLASTIC_AGENT_CELL
                elif belief.params[0] == 'paper_agent':
                    print('MAMMA QUANTTTO E\' BELLA TELVE!!! {} {}'.format(belief.name, ' '.join(belief.params)))
                    paper_agent_pose = self.extract_pose_from_cell_name(belief.params[1])
                    current_map[paper_agent_pose.x][paper_agent_pose.y] = PAPER_AGENT_CELL
            
            if belief.name == 'litter_pose':
                if belief.params[0].find('pla') == 0:
                    plastic_litter_poses.append(self.extract_pose_from_cell_name(belief.params[1]))
                elif belief.params[0].find('pap') == 0:
                    paper_litter_poses.append(self.extract_pose_from_cell_name(belief.params[1]))
            
            if belief.name == 'free':
                empty_cell_pose = self.extract_pose_from_cell_name(belief.params[0])
                current_map[empty_cell_pose.x][empty_cell_pose.y] = EMPTY_CELL
        
        for x in range(0, rows):
            for y in range(0, columns):
                if current_map[x][y] == OBSTACLE_CELL:
                    obstacles.append(MGPose(x,y))
        
        init_poses = {
            "plastic_agent": plastic_agent_pose,
            "paper_agent": paper_agent_pose,
            "plastic_bin": plastic_bin_pose,
            "paper_bin": paper_bin_pose,
            "persons": []
        }
        
        self.tk_litter_world_thread_.update_pa_world(obstacles, plastic_litter_poses, paper_litter_poses, init_poses, detection_depth)

    def extract_pose_from_cell_name(self, cell_name:str):
        args = cell_name.split("_")
        if len(args) == 3:
            return MGPose(int(args[1]), int(args[2]))
        else:
            return MGPose(-1, -1)

    def callback_pa_agent_intentions(self, msg:BDIPlanExecutionInfoMin):
        if self.get_parameter("show_agent_view").value == 'plastic_agent' or self.get_parameter("show_agent_view").value == 'paper_agent':
            self.get_logger().info("Current info from /{}/current_intentions:".format(self.get_parameter("show_agent_view").value))
            target_pose = MGPose(-1, -1)
            for target_belief in msg.target_value:
                if target_belief.name == 'in' and target_belief.params[0] == self.get_parameter("show_agent_view").value:
                    target_pose = self.extract_pose_from_cell_name(target_belief.params[1])
                    self.get_logger().info("Trying to get to ({},{})".format(target_pose.x, target_pose.y))
            
            committed_poses = []
            not_committed_poses = []
            self.get_logger().info("Via the following steps:")
            for aex_min in msg.actions_exec_info:
                if aex_min.name == 'move' and aex_min.args[0] == self.get_parameter("show_agent_view").value:
                    if aex_min.status == BDIActionExecutionInfoMin().RUNNING or aex_min.status == BDIActionExecutionInfoMin().WAITING:
                        next_pose = self.extract_pose_from_cell_name(aex_min.args[2])
                        if aex_min.committed:
                            committed_poses.append(next_pose)
                            self.get_logger().info("COMM - {}: ({},{})".format(aex_min.status, next_pose.x, next_pose.y))
                        else:
                            not_committed_poses.append(next_pose)
                            self.get_logger().info("NO_C: - {}: ({},{})".format(aex_min.status, next_pose.x, next_pose.y))

            self.tk_litter_world_thread_.update_pa_trajectory(MGMoveTrajectory(target_pose, committed_poses, not_committed_poses))

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
        
        free_cells = []

        # use world grid to map agent, bins, static and dynamic obstacles
        litter_world_grid_msg = GridStatus()
        for i in range(0, len(world_grid)):
            litter_world_row_msg = GridRowStatus()
            litter_world_row_msg.index = i
            for j in range(0, len(world_grid[0])):
                litter_world_row_msg.cells.append(world_grid[i][j])
                if world_grid[i][j] == EMPTY_CELL:
                    free_cells.append(MGPose(i,j))

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

        if len(free_cells) > 0:
            pla_agent_patrol_msg = Desire()
            pla_agent_patrol_msg.name = 'patrol'
            pla_agent_patrol_msg.priority = 0.4
            pla_agent_patrol_msg.deadline = 36.0
            rnd_pose_index = random.randint(0, len(free_cells)-1)
            rnd_pose = free_cells[rnd_pose_index]
            target_pose_belief = Belief()
            target_pose_belief.name = 'in'
            target_pose_belief.pddl_type = 2
            target_pose_belief.params = ['plastic_agent', 'c_{}_{}'.format(rnd_pose.x, rnd_pose.y)]
            pla_agent_patrol_msg.value = [target_pose_belief]
            self.plastic_agent_patrol_desire.publish(pla_agent_patrol_msg)


def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    
    # Create rclpy node to handle ROS2 "frontend"
    node = LitterWorldROS2Controller()
    upd_interval = node.get_upd_interval()
    init_world_json = node.get_init_world_json()
    show_agent_view = node.get_show_agent_view()
    size = node.get_size()

    if init_world_json != None:
        # Create new thread to handle tk app
        tk_litter_world_thread = TkLitterWorldThread(init_world_json, upd_interval, show_agent_view, size)
        
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