#!/usr/bin/env python3

import threading

from typing import List
from .person import MGPerson

from .pose import MGPose
from .move_trajectory import MGMoveTrajectory

from .tk_litter_world import TkLitterWorld

class TkLitterWorldThread (threading.Thread):
    def __init__(self, world_setup):
        threading.Thread.__init__(self)
        self.init_world_ = self.parse_to_init_world(world_setup["init_world_json"]) if "init_world_json" in world_setup else None
        self.upd_interval_ = world_setup["upd_interval"] if "upd_interval" in world_setup else None
        self.show_agent_view_ = world_setup["show_agent_view"] if "show_agent_view" in world_setup else None
        self.size_px_ = world_setup["world_size_px"] if "world_size_px" in world_setup else None
        self.tk_litter_world_ = None

    def run(self):
        self.tk_litter_world_ = TkLitterWorld(self.init_world_, self.upd_interval_, self.show_agent_view_, self.size_px_)
        self.tk_litter_world_.mainloop()  

    def update_pa_trajectory(self, move_trajectory:MGMoveTrajectory):
        if self.tk_litter_world_ != None:
            self.tk_litter_world_.update_pa_trajectory(move_trajectory)

    def update_pa_world(self, obstacles:List[MGPose], plastic_litter_poses:List[MGPose], paper_litter_poses:List[MGPose], init_poses, detection_depth:int):
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
        if 'persons'in init_world_json:
            for person in init_world_json["persons"]:
                if 'start' in person:
                    start_pose = MGPose(person["start"]["x"], person["start"]["y"])
                    litter_generation = person["litter_generation"] if "litter_generation" in person and person["litter_generation"] >= 0.0 and person["litter_generation"] <= 1.0 else 0.0
                    move_option = "random"
                    move_value = 0.0
                    if "movement" in person:
                        move_option = person["movement"]["move_option"] if "move_option" in person["movement"] else "random"
                        move_value = person["movement"]["move_value"] if "move_value" in person["movement"] else 0.0
                        if move_option == "random":
                            move_value = move_value if move_value >= 0.0 and move_value <= 1.0 else 0.0
                    mg_person = MGPerson(start_pose,litter_generation, move_option, move_value)
                    persons.append(mg_person)

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
            "persons": persons,
            "init_poses":{
                "plastic_agent": plastic_agent_pose,
                "paper_agent": paper_agent_pose,
                "plastic_bin": plastic_bin_pose,
                "paper_bin": paper_bin_pose,
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