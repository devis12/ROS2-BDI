#!/usr/bin/env python3

import threading

from .pose import Pose

from .tk_litter_world import TkLitterWorld

class TkLitterWorldThread (threading.Thread):
    def __init__(self, init_world_json, upd_interval):
        threading.Thread.__init__(self)
        self.init_world_ = self.parse_to_init_world(init_world_json)
        self.upd_interval_ = upd_interval

    def run(self):
        self.tk_litter_world = TkLitterWorld(self.init_world_, self.upd_interval_)
        self.tk_litter_world.mainloop()  

    def update_counter(self, num):
        self.tk_litter_world.upd_num(num)
    
    def move_agent(self, agent, cmd_move):
        self.tk_litter_world.move_agent(agent, cmd_move)
    
    def parse_to_init_world(self, init_world_json):
        columns = init_world_json["columns"]
        rows = init_world_json["rows"]
        plastic_agent_pose  = Pose(init_world_json["plastic_agent"]["x"], init_world_json["plastic_agent"]["y"])
        plastic_bin_pose  = Pose(init_world_json["plastic_bin"]["x"], init_world_json["plastic_bin"]["y"])
        paper_agent_pose  = Pose(init_world_json["paper_agent"]["x"], init_world_json["paper_agent"]["y"])
        paper_bin_pose  = Pose(init_world_json["paper_bin"]["x"], init_world_json["paper_bin"]["y"])

        obstacles = []
        for obs in init_world_json["obstacles"]:
            obstacles.append(Pose(obs["x"], obs["y"]))
        
        persons = []
        for person in init_world_json["persons"]:
            persons.append(Pose(person["x"], person["y"]))

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
        return self.tk_litter_world.get_litter_world_status()