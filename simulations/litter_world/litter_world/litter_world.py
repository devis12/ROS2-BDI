#!/usr/bin/env python3

import os
import pathlib
import random

import tkinter as tk
from PIL import Image, ImageTk

from .pose import Pose

from .cell_types import EMPTY_CELL, OBSTACLE_CELL, PAPER_AGENT_CELL, PERSON_CELL, PLASTIC_AGENT_CELL
from .cell_types import PLASTIC_BIN_CELL, PAPER_BIN_CELL, PLASTIC_LITTER_CELL, PAPER_LITTER_CELL

from ament_index_python.packages import get_package_share_directory

MAX_DIM = 32


class RecyclingAgent():
    def __init__(self, pose: Pose):
        self.pose = pose
        self.holding = 0

class PlasticAgent(RecyclingAgent):
    def __init__(self, pose: Pose):
        super().__init__(pose)

class PaperAgent(RecyclingAgent):
    def __init__(self, pose: Pose):
        super().__init__(pose)

class LitterWorld():

    def is_valid_pose(self, pose:Pose):
        return type(pose) == type(Pose(0,0)) and (pose.x >= 0 and pose.x < self.rows) and (pose.y >= 0 and pose.y < self.columns) 

    def is_accessible_pose_value(self, cell_value):
        return cell_value == EMPTY_CELL or cell_value == PLASTIC_LITTER_CELL or cell_value == PAPER_LITTER_CELL

    def is_valid_and_accessible_pose(self, pose:Pose):
        return self.is_valid_pose(pose) and self.is_accessible_pose_value(self.grid[pose.x][pose.y])

    def allowable_moves(self, pose:Pose):
        poses = []
        
        # allow right??
        if pose.x < self.columns -1 and self.grid[pose.x+1][pose.y] == EMPTY_CELL:
            poses.append(Pose(pose.x+1, pose.y))
        
        # allow left??
        if pose.x > 0 and self.grid[pose.x-1][pose.y] == EMPTY_CELL:
            poses.append(Pose(pose.x-1, pose.y))

        # allow down??
        if pose.y < self.rows -1 and self.grid[pose.x][pose.y+1] == EMPTY_CELL:
            poses.append(Pose(pose.x, pose.y+1))
        
        # allow up??
        if pose.y > 0 and self.grid[pose.x][pose.y-1] == EMPTY_CELL:
            poses.append(Pose(pose.x, pose.y-1))
        
        return poses

    def __init__(self, canvas: tk.Canvas, parent_app):
        self.canvas = canvas

        self.package_dir = get_package_share_directory('litter_world')

        # set the data as valide in the beginin
        self.valid_data = True

        self.main_controller = parent_app

        self.interval_handler = None

        self.write_values = False
        self.reset_stats()
            
    def valid_world(self):
        return self.valid_data
    
    def get_error_msg(self):
        return self.error_msg 

    def init_world(self, columns: int, rows: int, obstacles: list, init_poses):
        self.valid_data = True
        # validate num of column and rows
        if(columns > MAX_DIM or rows > MAX_DIM):
            self.error_msg = "Maximum allowed num of columns/rows is {}".format(MAX_DIM)
            self.valid_data = False
        else:
            self.columns = columns
            self.rows = rows

            # verify the initial positions
            if("plastic_agent" in init_poses and self.is_valid_pose(init_poses["plastic_agent"])):
                self.plastic_agent = PlasticAgent(init_poses["plastic_agent"])
                self.plastic_agent_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_agent.png')))
            else:
                self.error_msg = "Invalid initial position for plastic agent {}, {} provided in grid {},{}".format(self.plastic_agent.pose.x, self.plastic_agent.pose.y, self.rows, self.columns)
                self.valid_data = False
            
            if("paper_agent" in init_poses and self.is_valid_pose(init_poses["paper_agent"])):
                self.paper_agent = PaperAgent(init_poses["paper_agent"])
                self.paper_agent_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_agent.png')))
            else:
                self.error_msg = "Invalid initial position for paper agent {}, {} provided in grid {},{}".format(self.paper_agent.pose.x, self.paper_agent.pose.y, self.rows, self.columns)
                self.valid_data = False
            
            if("plastic_bin" in init_poses and self.is_valid_pose(init_poses["plastic_bin"])):
                self.plastic_bin_pose = init_poses["plastic_bin"]
                self.plastic_bin_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_bin.png')))
            else:
                self.error_msg = "Invalid initial position for plastic bin {}, {} provided in grid {},{}".format(self.plastic_bin_pose.x, self.plastic_bin_pose.y, self.rows, self.columns)
                self.valid_data = False
            
            if("paper_bin" in init_poses and self.is_valid_pose(init_poses["paper_bin"])):
                self.paper_bin_pose = init_poses["paper_bin"]
                self.paper_bin_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_bin.png')))
            else:
                self.error_msg = "Invalid initial position for paper bin {}, {} provided in grid {},{}".format(self.paper_bin_pose.x, self.paper_bin_pose.y, self.rows, self.columns)
                self.valid_data = False

            self.obstacles = obstacles
            self.persons = init_poses["persons"] if "persons" in init_poses else []
            self.obstacle_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'person.png')))
        
        if(self.valid_data):
            self.size_factor = (self.canvas.winfo_reqwidth()) / (max(self.columns, self.rows))#size of each square in the maze world
            
            self.person_agent_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'person.png')))
            self.plastic_litter_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_litter.png')))
            self.paper_litter_img = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_litter.png')))
            self.plastic_litter_img_mini = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_litter.png')))
            self.paper_litter_img_mini = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_litter.png')))

            self.plastic_agent_img = self.plastic_agent_img.resize((int(self.size_factor)-8,int(self.size_factor)-8))
            self.plastic_agent_pic = ImageTk.PhotoImage(self.plastic_agent_img)

            self.paper_agent_img = self.paper_agent_img.resize((int(self.size_factor)-8,int(self.size_factor)-8))
            self.paper_agent_pic = ImageTk.PhotoImage(self.paper_agent_img)

            self.plastic_bin_img = self.plastic_bin_img.resize((int(self.size_factor)-8,int(self.size_factor)-8))
            self.plastic_bin_pic = ImageTk.PhotoImage(self.plastic_bin_img)

            self.paper_bin_img = self.paper_bin_img.resize((int(self.size_factor)-8,int(self.size_factor)-8))
            self.paper_bin_pic = ImageTk.PhotoImage(self.paper_bin_img)

            self.plastic_litter_img = self.plastic_litter_img.resize((int(self.size_factor)-24,int(self.size_factor)-24))
            self.plastic_litter_pic = ImageTk.PhotoImage(self.plastic_litter_img)

            self.paper_litter_img = self.paper_litter_img.resize((int(self.size_factor)-24,int(self.size_factor)-24))
            self.paper_litter_pic = ImageTk.PhotoImage(self.paper_litter_img)
            
            self.plastic_litter_img_mini = self.plastic_litter_img_mini.resize((int(self.size_factor)-36,int(self.size_factor)-36))
            self.plastic_litter_pic_mini = ImageTk.PhotoImage(self.plastic_litter_img_mini)

            self.paper_litter_img_mini = self.paper_litter_img_mini.resize((int(self.size_factor)-48,int(self.size_factor)-48))
            self.paper_litter_pic_mini = ImageTk.PhotoImage(self.paper_litter_img_mini)

            self.person_agent_img = self.person_agent_img.resize((int(self.size_factor)-32,int(self.size_factor)-8))
            self.person_agent_pic = ImageTk.PhotoImage(self.person_agent_img)

            # Set up an empty game grid.
            self.grid = [[EMPTY_CELL for x in range(self.columns)] for x in range(self.rows)]
            self.litter_grid = [[EMPTY_CELL for x in range(self.columns)] for x in range(self.rows)]
            print("init_empty", self.grid)

            # Set up obstacles points
            for obs in self.obstacles:
                if(self.is_valid_pose(obs)):
                    self.grid[obs.x][obs.y] = OBSTACLE_CELL
                else:
                    print("Obstacle x:{}, y:{} is not valid, therefore it won't be inserted".format(obs.x, obs.y))

            # Set up persons init points
            for person in self.persons:
                if(self.is_valid_pose(person)):
                    self.grid[person.x][person.y] = PERSON_CELL
                else:
                    print("Person x:{}, y:{} is not valid, therefore it won't be inserted".format(person.x, person.y))
            
            self.grid[self.plastic_agent.pose.x][self.plastic_agent.pose.y] = PLASTIC_AGENT_CELL
            self.grid[self.paper_agent.pose.x][self.paper_agent.pose.y] = PAPER_AGENT_CELL
            self.grid[self.plastic_bin_pose.x][self.plastic_bin_pose.y] = PLASTIC_BIN_CELL
            self.grid[self.paper_bin_pose.x][self.paper_bin_pose.y] = PAPER_BIN_CELL
            print("init_fill", self.grid)

    #Draw
    def draw_world(self):
        # Create the canvas widget and add it to the Tkinter application window.
        if(self.canvas != None):
            self.canvas.delete("all")

             # Draw a square on the game board for every cell in the grid.
            for x in range(0, self.rows):
                for y in range(0, self.columns):
                    realx = x * self.size_factor
                    realy = y * self.size_factor
                    self.canvas.create_rectangle(realy, realx, realy+self.size_factor, realx+self.size_factor, fill='white', outline='black')
                    
            
            # Draw a square on the game board for every cell in the grid.
            for x in range(0, self.rows):
                for y in range(0, self.columns):
                    if(self.grid[x][y] != EMPTY_CELL or self.litter_grid[x][y] != EMPTY_CELL):
                        realx = x * self.size_factor
                        realy = y * self.size_factor
                        if self.litter_grid[x][y] == PLASTIC_LITTER_CELL or self.litter_grid[x][y] == PAPER_LITTER_CELL:
                            self.draw_square(realx, realy, self.size_factor, self.litter_grid[x][y])

                        self.draw_square(realx, realy, self.size_factor, self.grid[x][y])

    
    def draw_square(self, x, y, size, square_type):
        if(self.canvas != None):
            font_size = 12 if size > 50 else 10
            if square_type == PLASTIC_AGENT_CELL:
                self.canvas.create_image(y+4, x+4, anchor=tk.NW, image=self.plastic_agent_pic)
                if self.plastic_agent.holding > 0: #check if loaded
                    self.canvas.create_image(y+18, x+18, anchor=tk.NW, image=self.plastic_litter_pic_mini)
                self.canvas.create_text(y+size-12, x+12, text=("{}").format(self.plastic_agent.holding), fill="black", font=('Helvetica {} bold'.format(font_size)))
            
            elif square_type == PAPER_AGENT_CELL:
                self.canvas.create_image(y+4, x+4, anchor=tk.NW, image=self.paper_agent_pic)
                if self.paper_agent.holding > 0: #check if loaded
                    self.canvas.create_image(y+24, x+24, anchor=tk.NW, image=self.paper_litter_pic_mini)
                self.canvas.create_text(y+size-12, x+12, text=("{}").format(self.paper_agent.holding), fill="black", font=('Helvetica {} bold'.format(font_size)))
            
            elif square_type == PLASTIC_BIN_CELL:
                self.canvas.create_image(y+4, x+4, anchor=tk.NW, image=self.plastic_bin_pic)
            
            elif square_type == PAPER_BIN_CELL:
                self.canvas.create_image(y+4, x+4, anchor=tk.NW, image=self.paper_bin_pic)

            elif square_type == PERSON_CELL: #Draw our bro Paolo on the canvas.
                self.canvas.create_image(y+16, x+4, anchor=tk.NW, image=self.person_agent_pic)

            elif square_type == OBSTACLE_CELL: #Draw a black square on the canvas.
                self.canvas.create_rectangle(y, x, y+size, x+size, fill='black', outline='black')
            
            elif square_type == PAPER_LITTER_CELL:
                self.canvas.create_image(y+12, x+12, anchor=tk.NW, image=self.paper_litter_pic)
            
            elif square_type == PLASTIC_LITTER_CELL:
                self.canvas.create_image(y+12, x+12, anchor=tk.NW, image=self.plastic_litter_pic)
    
    def count_litter(self):
        count = 0
        for x in range(0, self.rows):
            for y in range(0, self.columns):
                count += 1 if self.litter_grid[x][y] == PLASTIC_LITTER_CELL or self.litter_grid[x][y] == PAPER_LITTER_CELL else 0
        return count

    def upd_holding_agent(self, agent, holding_upd):
        if agent == PLASTIC_AGENT_CELL:
            self.plastic_agent.holding += holding_upd
            if self.plastic_agent.holding < 0:
                self.plastic_agent.holding = 0
                return False, 0

            return True, self.plastic_agent.holding

        elif agent == PAPER_AGENT_CELL:
            self.paper_agent.holding += holding_upd
            if self.paper_agent.holding < 0:
                self.paper_agent.holding = 0
                return False, 0

            return True, self.paper_agent.holding
        
        return False, -1

    def remove_litter(self, x, y):
        if x < self.rows and y < self.columns:
            self.litter_grid[x][y] = EMPTY_CELL
            return True
        
        return False
    
    def move_persons(self):
        upd_persons = []
        # Set up persons init points
        for person in self.persons:
            new_poses = self.allowable_moves(person)
            if len(new_poses) > 0 and random.randint(1,10) == 10:
                # 50% chance of movement in new_pose for person if movement is allowed
                new_pose = new_poses[random.randint(0, len(new_poses)-1)]

                if random.randint(1,10) == 10 and self.count_litter() < 10:# 10% chance of not being a polluter this round
                    self.litter_grid[person.x][person.y] = PLASTIC_LITTER_CELL if random.randint(0, 1) == 1 else PAPER_LITTER_CELL
                
                self.grid[person.x][person.y] = EMPTY_CELL 

                person = new_pose 
                self.grid[person.x][person.y] = PERSON_CELL

            upd_persons.append(person)
        
        self.persons = upd_persons

        self.draw_world()
    
    def move_agent(self, agent, cmd_move):

        if agent == PLASTIC_AGENT_CELL:
            new_pose = self.try_upd_agent_pose(self.plastic_agent.pose, cmd_move)
            if new_pose != None:
                self.grid[self.plastic_agent.pose.x][self.plastic_agent.pose.y] = EMPTY_CELL
                self.plastic_agent.pose = new_pose
                self.grid[self.plastic_agent.pose.x][self.plastic_agent.pose.y] = PLASTIC_AGENT_CELL
                return True
                

        elif agent == PAPER_AGENT_CELL:
            new_pose = self.try_upd_agent_pose(self.paper_agent.pose, cmd_move)
            if new_pose != None:
                self.grid[self.paper_agent.pose.x][self.paper_agent.pose.y] = EMPTY_CELL
                self.paper_agent.pose = new_pose
                self.grid[self.paper_agent.pose.x][self.paper_agent.pose.y] = PAPER_AGENT_CELL
                return True
        
        return False
                
    
    def try_upd_agent_pose(self, agent_pose, cmd_move):
        new_pose = Pose(agent_pose.x, agent_pose.y)
        new_pose.x += cmd_move.x
        new_pose.y += cmd_move.y
        
        if self.is_valid_pose(new_pose) and self.grid[new_pose.x][new_pose.y] == EMPTY_CELL:
            return new_pose
        else:
            return None

    def get_grid(self):
        return self.grid

    def get_litter_grid(self):
        return self.litter_grid

    def reset_stats(self):
        self.steps = 0
        self.main_controller.upd_stats({})

    def upd_stats(self):
        self.steps += 1
        self.main_controller.upd_stats({"steps":self.steps})


