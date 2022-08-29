import random
import tkinter as tk
from tkinter import messagebox

from .pose import Pose
from .litter_world import LitterWorld

class TkLitterWorld(tk.Tk):
    def __init__(self, width_and_height = 768):
        super().__init__()

        self.title("Litter world")
        self.resizable(False, False)

        self.labelStats = tk.Label(self, text="0", anchor=tk.CENTER, font=('arial bold', 12))
        self.labelStats.pack(fill=tk.X)

        self.litter_world_canvas = tk.Canvas(self, width=width_and_height, height=width_and_height,  bg='white')
        self.litter_world_canvas.pack()

        self.litter_world = LitterWorld(self.litter_world_canvas, self)
        
        cols = 12
        rows = 12
        obstacles = []
        for i in range(0,random.randint(10, 20)):
            obstacles.append(Pose(random.randint(0,11), random.randint(2,11)))
        
        plastic_agent_pose  = Pose(2,1)
        plastic_bin_pose  = Pose(2,0)
        paper_agent_pose  = Pose(4,1)
        paper_bin_pose  = Pose(4,0)

        init_poses = {
            "plastic_agent": plastic_agent_pose,
            "paper_agent": paper_agent_pose,
            "plastic_bin": plastic_bin_pose,
            "paper_bin": paper_bin_pose,
        }
        self.generate_world(cols, rows, obstacles, init_poses)
    
    def generate_world(self, columns: int, rows: int, obstacles: list, init_poses):
        # initialize world
        self.litter_world.init_world(columns, rows, obstacles, init_poses)
        if not self.litter_world.valid_world():
            messagebox.showerror(master=self, title="Error", message=self.world.get_error_msg())
        else:
            self.litter_world.reset_stats()
            self.litter_world.draw_world()
        pass

    def upd_num(self, num):
        self.labelStats["text"] = "{}".format(num)

    def upd_stats(self, stats):
        pass

