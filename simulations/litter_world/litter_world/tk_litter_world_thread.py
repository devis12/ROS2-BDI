#!/usr/bin/env python3

import threading

from .pose import MGPose
from .move_trajectory import MGMoveTrajectory

from .tk_litter_world import TkLitterWorld

class TkLitterWorldThread (threading.Thread):
    def __init__(self, init_world_json, upd_interval:int, show_agent_view:str, size:int, people_movement:bool):
        threading.Thread.__init__(self)
        self.init_world_ = self.parse_to_init_world(init_world_json)
        self.upd_interval_ = upd_interval
        self.show_agent_view_ = show_agent_view
        self.size_ = size
        self.people_movement_ = people_movement
        self.tk_litter_world_ = None

    def run(self):
        self.tk_litter_world_ = TkLitterWorld(self.init_world_, self.upd_interval_, self.show_agent_view_, self.size_, self.people_movement_)
        self.tk_litter_world_.mainloop()  

    def update_pa_trajectory(self, move_trajectory:MGMoveTrajectory):
        if self.tk_litter_world_ != None:
            self.tk_litter_world_.update_pa_trajectory(move_trajectory)

    def update_pa_world(self, obstacles:list, plastic_litter_poses:list, paper_litter_poses:list, init_poses, detection_depth:int):
        if self.tk_litter_world_ != None:
            self.tk_litter_world_.update_pa_world(obstacles, plastic_litter_poses, paper_litter_poses, init_poses, detection_depth)

    def update_counter(self, num):
        if self.tk_litter_world_ != None:
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
        plastic_agent_pose  = MGPose(init_world_json["plastic_agent"]["x"], init_world_json["plastic_agent"]["y"]) if 'obstacles'in init_world_json else MGPose(-1, -1)
        plastic_bin_pose  = MGPose(init_world_json["plastic_bin"]["x"], init_world_json["plastic_bin"]["y"]) if 'plastic_bin'in init_world_json else MGPose(-1, -1)
        paper_agent_pose  = MGPose(init_world_json["paper_agent"]["x"], init_world_json["paper_agent"]["y"]) if 'paper_agent'in init_world_json else MGPose(-1, -1)
        paper_bin_pose  = MGPose(init_world_json["paper_bin"]["x"], init_world_json["paper_bin"]["y"]) if 'paper_bin'in init_world_json else MGPose(-1, -1)

        obstacles = []
        if 'obstacles'in init_world_json:
            for obs in init_world_json["obstacles"]:
                obstacles.append(MGPose(obs["x"], obs["y"]))
        
        persons = []
        if 'obstacles'in init_world_json:
            for person in init_world_json["persons"]:
                persons.append(MGPose(person["x"], person["y"]))

        plastic_poses = []
        if 'plastic_poses' in init_world_json:
            for plastic_pose in init_world_json["plastic_poses"]:
                plastic_poses.append(MGPose(plastic_pose["x"], plastic_pose["y"]))

        paper_poses = []
        if 'paper_poses' in init_world_json:
            for paper_pose in init_world_json["paper_poses"]:
                paper_poses.append(MGPose(paper_pose["x"], paper_pose["y"]))

        init_world = {
            "columns": columns,
            "rows": rows,
            "obstacles": obstacles,
            "init_poses":{
                "plastic_agent": plastic_agent_pose,
                "paper_agent": paper_agent_pose,
                "plastic_bin": plastic_bin_pose,
                "paper_bin": paper_bin_pose,
                "persons": persons,
                "plastic_poses": plastic_poses,
                "paper_poses": paper_poses
            }
        }
        return init_world
    
    def get_litter_world_status(self):
        if self.tk_litter_world_ == None:
            return None, None
        else:
            return self.tk_litter_world_.get_litter_world_status()