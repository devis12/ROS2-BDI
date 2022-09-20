#!/usr/bin/env python3

from concurrent.futures import thread
import tkinter as tk
import threading
from tkinter import messagebox

from .pose import Pose

from .litter_world import LitterWorld


class TkLitterWorld(tk.Tk):
    def __init__(self, init_world, upd_interval, width_and_height = 768):
        super().__init__()

        self.title("Litter world")
        self.resizable(False, False)

        self.epoch = 0
        self.wait_next_epoch = threading.Condition()
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
        print("Thread {} requesting agent to move".format(threading.current_thread().ident ))
        accepted = self.litter_world_.move_agent(agent, cmd_move)
        if accepted:
            with self.wait_next_epoch: 
                self.wait_next_epoch.wait()
        return accepted
    
    def upd_holding_agent(self, agent, holding_upd):
        #print("Thread {} requesting hold upd cmd".format(threading.current_thread().ident ))
        performed, holding = self.litter_world_.upd_holding_agent(agent, holding_upd)
        if performed:
            with self.wait_next_epoch: 
                self.wait_next_epoch.wait()
        return performed, holding
    
    def remove_litter(self, x, y):
        #print("Thread {} requesting to rm litter".format(threading.current_thread().ident ))
        accepted = self.litter_world_.remove_litter(x, y)
        if accepted:
            with self.wait_next_epoch: 
                self.wait_next_epoch.wait()
        return accepted
    

    def upd_sim(self):
        print("Thread {} updating sim".format(threading.current_thread().ident ))
        self.litter_world_.move_persons()
        self.epoch += 1
        with self.wait_next_epoch:
            self.wait_next_epoch.notify_all()
        self.after(self.upd_interval_, self.upd_sim)
    
    def get_litter_world_status(self):
        return self.litter_world_.get_grid(), self.litter_world_.get_litter_grid()

    def upd_num(self, num):
        self.label_stats_["text"] = "{}".format(num)

    def upd_stats(self, stats):
        pass

