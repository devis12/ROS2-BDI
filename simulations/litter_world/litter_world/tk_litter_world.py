#!/usr/bin/env python3
from typing import List
from concurrent.futures import thread
import tkinter as tk
import threading
from tkinter import messagebox

from .pose import MGPose

from .move_trajectory import MGMoveTrajectory

from .litter_world import LitterWorld


class TkLitterWorld(tk.Tk):
    def __init__(self, init_world, upd_interval:int, show_agent_view:str = '', width_and_height:int = 768):
        super().__init__()

        self.title("Litter world")
        self.resizable(False, False)

        self.epoch_ = 0
        self.wait_next_epoch_ = threading.Condition()
        self.label_frame_ = tk.Frame(self)
        self.label_frame_.pack(fill=tk.X, expand=True)
        self.epoch_label_ = tk.Label(self.label_frame_, text="Epoch: -", anchor=tk.W, font=('arial bold', 12))
        self.epoch_label_.pack(side=tk.LEFT)

        self.litter_world_canvas_ = tk.Canvas(self, width=width_and_height, height=width_and_height,  bg='white')
        self.litter_world_canvas_.pack(side=tk.LEFT)

        self.pa_litter_world_ = None
        if show_agent_view == 'plastic_agent' or show_agent_view == 'paper_agent':
            self.agent_steps_ = 0
            txt_color = 'blue' if show_agent_view == 'plastic_agent' else 'orange'
            self.steps_label_ = tk.Label(self.label_frame_, text="Steps: -", anchor=tk.E, fg=txt_color, font=('arial bold', 12))
            self.steps_label_.pack()

            self.show_agent_view_ = show_agent_view
            self.agent_view_frame = tk.Frame(self)

            self.pa_litter_world_canvas_ = tk.Canvas(self.agent_view_frame, width=width_and_height, height=width_and_height,  bg='white')
            self.pa_litter_world_canvas_.pack(side=tk.RIGHT)        

            self.pa_litter_world_ = LitterWorld(self.pa_litter_world_canvas_, self, show_agent_view=show_agent_view)
            
            self.divider_canvas_ = tk.Canvas(self.agent_view_frame, width=12, height=width_and_height,  bg='green')
            self.divider_canvas_.pack(side=tk.LEFT)   

            self.agent_view_frame.pack(side=tk.RIGHT)

        self.litter_world_ = LitterWorld(self.litter_world_canvas_, self)
        
        self.upd_interval_ = upd_interval
        self.generate_world(init_world)
    
    def generate_world(self, init_world):
        # initialize world
        self.litter_world_.init_world(init_world["columns"], init_world["rows"], init_world["obstacles"], init_world["persons"], init_world["init_poses"])
        # if not self.litter_world_.valid_world():
        #     messagebox.showerror(master=self, title="Error", message=self.litter_world_.get_error_msg())
        # else:
        self.litter_world_.draw_world()
        self.after(self.upd_interval_, self.upd_sim)
        
        if self.pa_litter_world_ != None:
            self.pa_litter_world_.init_world(init_world["columns"], init_world["rows"], [], [], {}, log_errors=False)
            self.pa_litter_world_.draw_world()
    
    def update_pa_world(self, obstacles:List[MGPose], plastic_litter_poses:List[MGPose], paper_litter_poses:List[MGPose], init_poses, detection_depth:int):
        if self.pa_litter_world_ != None:
            self.pa_litter_world_.update_map(obstacles, [], plastic_litter_poses, paper_litter_poses, init_poses, detection_depth, show_logs=False)

    def update_pa_trajectory(self, move_trajectory:MGMoveTrajectory):
        self.litter_world_.update_pa_trajectory(self.show_agent_view_, move_trajectory)
        
        if self.pa_litter_world_ != None:
            self.pa_litter_world_.update_pa_trajectory(self.show_agent_view_, move_trajectory)

    def move_agent(self, agent, cmd_move):
        print("Thread {} requesting agent to move".format(threading.current_thread().ident ))
        accepted = self.litter_world_.move_agent(agent, cmd_move)
        if accepted:
            with self.wait_next_epoch_: 
                self.wait_next_epoch_.wait()
        return accepted
    
    def upd_holding_agent(self, agent, holding_upd):
        #print("Thread {} requesting hold upd cmd".format(threading.current_thread().ident ))
        if self.pa_litter_world_ != None:
            self.pa_litter_world_.upd_holding_agent(agent, holding_upd)
        performed, holding = self.litter_world_.upd_holding_agent(agent, holding_upd)
        if performed:
            with self.wait_next_epoch_: 
                self.wait_next_epoch_.wait()
        return performed, holding
    
    def remove_litter(self, x, y):
        #print("Thread {} requesting to rm litter".format(threading.current_thread().ident ))
        if self.pa_litter_world_ != None:
            self.pa_litter_world_.remove_litter(x, y)
        accepted = self.litter_world_.remove_litter(x, y)
        if accepted:
            with self.wait_next_epoch_: 
                self.wait_next_epoch_.wait()
        return accepted
    

    def upd_sim(self):
        print("Thread {} updating sim".format(threading.current_thread().ident ))
        moving  = self.litter_world_.draw_next_epoch()
        if self.pa_litter_world_ != None:
            self.pa_litter_world_.draw_next_epoch()
        self.epoch_ += 1
        self.upd_stats()
        with self.wait_next_epoch_:
            self.wait_next_epoch_.notify_all()

        if self.show_agent_view_ in moving:
            self.agent_moved(self.show_agent_view_) #put it here, so steps are going to be shown updated next round 
        
        self.after(self.upd_interval_, self.upd_sim)
    
    def get_litter_world_status(self):
        return self.litter_world_.get_grid(), self.litter_world_.get_litter_grid()

    def agent_moved(self, agent):
        if self.show_agent_view_ == agent:
            self.agent_steps_ += 1
    
    def upd_stats(self):
        self.epoch_label_["text"] = "Epoch: {}".format(self.epoch_)
        self.steps_label_["text"] = "Steps: {}".format(self.agent_steps_)

