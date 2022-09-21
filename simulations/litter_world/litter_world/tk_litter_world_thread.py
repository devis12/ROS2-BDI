#!/usr/bin/env python3

import threading

from .pose import MGPose
from .move_trajectory import MGMoveTrajectory

from .tk_litter_world import TkLitterWorld

class TkLitterWorldThread (threading.Thread):
    def __init__(self, init_world_json, upd_interval, show_agent_view, size):
        threading.Thread.__init__(self)
        self.init_world_ = self.parse_to_init_world(init_world_json)
        self.upd_interval_ = upd_interval
        self.show_agent_view_ = show_agent_view
        self.size_ = size

    def run(self):
        self.tk_litter_world_ = TkLitterWorld(self.init_world_, self.upd_interval_, self.show_agent_view_, self.size_)
        self.tk_litter_world_.mainloop()  

    def update_pa_trajectory(self, move_trajectory:MGMoveTrajectory):
        self.tk_litter_world_.update_pa_trajectory(move_trajectory)

    def update_pa_world(self, obstacles:list, plastic_litter_poses:list, paper_litter_poses:list, init_poses, detection_depth:int):
        self.tk_litter_world_.update_pa_world(obstacles, plastic_litter_poses, paper_litter_poses, init_poses, detection_depth)

    def update_counter(self, num):
        self.tk_litter_world_.upd_num(num)
    
    def move_agent(self, agent, cmd_move):
        return self.tk_litter_world_.move_agent(agent, cmd_move)
    
    def upd_holding_agent(self, agent, holding_upd):
        return self.tk_litter_world_.upd_holding_agent(agent, holding_upd)
    
    def remove_litter(self, x, y):
        return self.tk_litter_world_.remove_litter(x, y)
    
    def parse_to_init_world(self, init_world_json):
        columns = init_world_json["columns"]
        rows = init_world_json["rows"]
        plastic_agent_pose  = MGPose(init_world_json["plastic_agent"]["x"], init_world_json["plastic_agent"]["y"])
        plastic_bin_pose  = MGPose(init_world_json["plastic_bin"]["x"], init_world_json["plastic_bin"]["y"])
        paper_agent_pose  = MGPose(init_world_json["paper_agent"]["x"], init_world_json["paper_agent"]["y"])
        paper_bin_pose  = MGPose(init_world_json["paper_bin"]["x"], init_world_json["paper_bin"]["y"])

        obstacles = []
        for obs in init_world_json["obstacles"]:
            obstacles.append(MGPose(obs["x"], obs["y"]))
        
        persons = []
        for person in init_world_json["persons"]:
            persons.append(MGPose(person["x"], person["y"]))

        init_world = {
            "columns": columns,
            "rows": rows,
            "obstacles": obstacles,
            "init_poses":{
                "plastic_agent": plastic_agent_pose,
                "paper_agent": paper_agent_pose,
                "plastic_bin": plastic_bin_pose,
                "paper_bin": paper_bin_pose,
                "persons": persons
            }
        }
        return init_world
    
    def get_litter_world_status(self):
        return self.tk_litter_world_.get_litter_world_status()