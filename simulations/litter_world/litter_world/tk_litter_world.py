#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox

from .litter_world import LitterWorld

class TkLitterWorld(tk.Tk):
    def __init__(self, init_world, upd_interval, width_and_height = 768):
        super().__init__()

        self.title("Litter world")
        self.resizable(False, False)

        self.label_stats_ = tk.Label(self, text="0", anchor=tk.CENTER, font=('arial bold', 12))
        self.label_stats_.pack(fill=tk.X)

        self.litter_world_canvas_ = tk.Canvas(self, width=width_and_height, height=width_and_height,  bg='white')
        self.litter_world_canvas_.pack()

        self.litter_world_ = LitterWorld(self.litter_world_canvas_, self)
        
        self.upd_interval_ = upd_interval
        self.generate_world(init_world)
    
    def generate_world(self, init_world):
        # initialize world
        self.litter_world_.init_world(init_world["columns"], init_world["rows"], init_world["obstacles"], init_world["init_poses"])
        if not self.litter_world_.valid_world():
            messagebox.showerror(master=self, title="Error", message=self.world.get_error_msg())
        else:
            self.litter_world_.reset_stats()
            self.litter_world_.draw_world()
            self.after(self.upd_interval_, self.upd_sim)
    
    def move_agent(self, agent, cmd_move):
        self.litter_world_.move_agent(agent, cmd_move)

    def upd_sim(self):
        self.litter_world_.move_persons()

        self.after(self.upd_interval_, self.upd_sim)
    
    def get_litter_world_status(self):
        return self.litter_world_.get_grid()

    def upd_num(self, num):
        self.label_stats_["text"] = "{}".format(num)

    def upd_stats(self, stats):
        pass

