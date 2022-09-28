#!/usr/bin/env python3

import os
import pathlib
import random

import tkinter as tk

from typing import List

from PIL import Image, ImageTk

from .pose import MGPose
from .person import MGPerson
from .move_trajectory import MGMoveTrajectory

from .cell_types import EMPTY_CELL, OBSTACLE_CELL, PAPER_AGENT_CELL, PERSON_CELL, PLASTIC_AGENT_CELL
from .cell_types import PLASTIC_BIN_CELL, PAPER_BIN_CELL, PLASTIC_LITTER_CELL, PAPER_LITTER_CELL

from ament_index_python.packages import get_package_share_directory

MAX_DIM = 32


class RecyclingAgent():
    def __init__(self, pose: MGPose):
        self.pose = pose
        self.old_pose = MGPose(-1, -1)
        self.moving = False
        self.holding = 0
        self.move_trajectory = None

class PlasticAgent(RecyclingAgent):
    def __init__(self, pose: MGPose):
        super().__init__(pose)

class PaperAgent(RecyclingAgent):
    def __init__(self, pose: MGPose):
        super().__init__(pose)

class LitterWorld():

    def is_valid_pose(self, pose:MGPose):
        return type(pose) == type(MGPose(0,0)) and (pose.x >= 0 and pose.x < self.rows) and (pose.y >= 0 and pose.y < self.columns) 

    def is_accessible_pose_value(self, cell_value):
        return cell_value == EMPTY_CELL or cell_value == PLASTIC_LITTER_CELL or cell_value == PAPER_LITTER_CELL

    def is_valid_and_accessible_pose(self, pose:MGPose):
        return self.is_valid_pose(pose) and self.is_accessible_pose_value(self.grid_[pose.x][pose.y])

    def allowable_moves(self, pose:MGPose):
        poses = []
        
        # allow right??
        if pose.x < self.columns -1 and self.grid_[pose.x+1][pose.y] == EMPTY_CELL:
            poses.append(MGPose(pose.x+1, pose.y))
        
        # allow left??
        if pose.x > 0 and self.grid_[pose.x-1][pose.y] == EMPTY_CELL:
            poses.append(MGPose(pose.x-1, pose.y))

        # allow down??
        if pose.y < self.rows -1 and self.grid_[pose.x][pose.y+1] == EMPTY_CELL:
            poses.append(MGPose(pose.x, pose.y+1))
        
        # allow up??
        if pose.y > 0 and self.grid_[pose.x][pose.y-1] == EMPTY_CELL:
            poses.append(MGPose(pose.x, pose.y-1))
        
        return poses

    def __init__(self, canvas: tk.Canvas, parent_app, show_agent_view = ''):
        self.canvas_ = canvas
        self.canvas_static_objs_ = []
        self.canvas_dynamic_objs_ = []
        self.canvas_trajectory_points_ = []

        self.package_dir = get_package_share_directory('litter_world')

        # set the data as valide in the beginin
        self.valid_data_ = True
        
        # useful for mirrored vision
        self.show_agent_view_ = show_agent_view #is it the mirrored side? if yes, who am I seeing?
        self.detection_depth_ = 0 #detection depth of the agent

        self.main_controller_ = parent_app

        self.interval_handler = None

        self.write_values = False
            
    def valid_world(self):
        return self.valid_data_
    
    def get_error_msg(self):
        return self.error_msg_ 

    def load_pic(self):
        self.plastic_agent_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_agent.png')))
        self.paper_agent_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_agent.png')))
        self.plastic_bin_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_bin.png')))
        self.paper_bin_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_bin.png')))

        self.person_agent_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'person.png')))
        self.plastic_litter_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_litter.png')))
        self.paper_litter_img_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_litter.png')))
        self.plastic_litter_img_mini_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'plastic_litter.png')))
        self.paper_litter_img_mini_ = Image.open(pathlib.Path(os.path.join(self.package_dir, 'asset', 'paper_litter.png')))

        self.plastic_agent_img_ = self.plastic_agent_img_.resize((int(self.size_factor)-8,int(self.size_factor)-8))
        self.plastic_agent_pic_ = ImageTk.PhotoImage(self.plastic_agent_img_)

        self.paper_agent_img_ = self.paper_agent_img_.resize((int(self.size_factor)-8,int(self.size_factor)-8))
        self.paper_agent_pic_ = ImageTk.PhotoImage(self.paper_agent_img_)

        self.plastic_bin_img_ = self.plastic_bin_img_.resize((int(self.size_factor)-8,int(self.size_factor)-8))
        self.plastic_bin_pic_ = ImageTk.PhotoImage(self.plastic_bin_img_)

        self.paper_bin_img_ = self.paper_bin_img_.resize((int(self.size_factor)-8,int(self.size_factor)-8))
        self.paper_bin_pic_ = ImageTk.PhotoImage(self.paper_bin_img_)

        self.plastic_litter_img_ = self.plastic_litter_img_.resize((int(self.size_factor)-24,int(self.size_factor)-24))
        self.plastic_litter_pic_ = ImageTk.PhotoImage(self.plastic_litter_img_)

        self.paper_litter_img_ = self.paper_litter_img_.resize((int(self.size_factor)-24,int(self.size_factor)-24))
        self.paper_litter_pic_ = ImageTk.PhotoImage(self.paper_litter_img_)
        
        self.plastic_litter_img_mini_ = self.plastic_litter_img_mini_.resize((int(self.size_factor)-36,int(self.size_factor)-36))
        self.plastic_litter_pic_mini_ = ImageTk.PhotoImage(self.plastic_litter_img_mini_)

        self.paper_litter_img_mini_ = self.paper_litter_img_mini_.resize((int(self.size_factor)-48,int(self.size_factor)-48))
        self.paper_litter_pic_mini_ = ImageTk.PhotoImage(self.paper_litter_img_mini_)

        self.person_agent_img_ = self.person_agent_img_.resize((int(self.size_factor)-32,int(self.size_factor)-8))
        self.person_agent_pic_ = ImageTk.PhotoImage(self.person_agent_img_)


    def init_world(self, columns: int, rows: int, obstacles: List[MGPose], persons: List[MGPerson], init_poses, log_errors=True):
        if log_errors:
            self.error_msg_ = ''
        self.valid_data_ = True
        # validate num of column and rows
        if(columns > MAX_DIM or rows > MAX_DIM):
            self.error_msg_ += "\nMaximum allowed num of columns/rows is {}".format(MAX_DIM)
            self.valid_data_ = False
        else:
            self.columns = columns
            self.rows = rows

            self.size_factor = (self.canvas_.winfo_reqwidth()) / (max(self.columns, self.rows))#size of each square in the maze world
            self.load_pic()

            # verify the initial positions
            
            if("plastic_agent" in init_poses and self.is_valid_pose(init_poses["plastic_agent"])):
                self.plastic_agent_ = PlasticAgent(init_poses["plastic_agent"])
            else:
                self.plastic_agent_ = PlasticAgent(MGPose(-1,-1))
                if log_errors:
                    err = "Invalid initial position for plastic agent {}, {} provided in grid {},{}".format(self.plastic_agent_.pose.x, self.plastic_agent_.pose.y, self.rows, self.columns)
                    print(err)
                    self.error_msg_ += "\n" + err
                self.valid_data_ = False
            
            if("paper_agent" in init_poses and self.is_valid_pose(init_poses["paper_agent"])):
                self.paper_agent_ = PaperAgent(init_poses["paper_agent"])
            else:
                self.paper_agent_ = PaperAgent(MGPose(-1,-1))
                if log_errors:
                    err = "Invalid initial position for paper agent {}, {} provided in grid {},{}".format(self.paper_agent_.pose.x, self.paper_agent_.pose.y, self.rows, self.columns)
                    print(err)
                    self.error_msg_ += "\n" + err
                self.valid_data_ = False
            
            
            if("plastic_bin" in init_poses and self.is_valid_pose(init_poses["plastic_bin"])):
                self.plastic_bin_pose = init_poses["plastic_bin"]
            else:
                self.plastic_bin_pose = MGPose(-1,-1)
                if log_errors:
                    err = "Invalid initial position for plastic bin {}, {} provided in grid {},{}".format(self.plastic_bin_pose.x, self.plastic_bin_pose.y, self.rows, self.columns)
                    print(err)
                    self.error_msg_ += "\n" + err
                self.valid_data_ = False
            
            
            if("paper_bin" in init_poses and self.is_valid_pose(init_poses["paper_bin"])):
                self.paper_bin_pose = init_poses["paper_bin"]
            else:
                self.paper_bin_pose = MGPose(-1,-1)
                if log_errors:
                    err = "Invalid initial position for paper bin {}, {} provided in grid {},{}".format(self.paper_bin_pose.x, self.paper_bin_pose.y, self.rows, self.columns)
                    print(err)
                    self.error_msg_ += "\n" + err
                self.valid_data_ = False

            self.obstacles_ = obstacles
            self.persons_ = persons
        
            # Set up an empty game grid.
            self.grid_ = [[EMPTY_CELL for x in range(self.columns)] for x in range(self.rows)]
            self.litter_grid_ = [[EMPTY_CELL for x in range(self.columns)] for x in range(self.rows)]
            
            # Set up obstacles points
            for obs in self.obstacles_:
                if(self.is_valid_pose(obs)):
                    self.grid_[obs.x][obs.y] = OBSTACLE_CELL
                else:
                    print("Obstacle x:{}, y:{} is not valid, therefore it won't be inserted".format(obs.x, obs.y))

            # Set up persons init points
            person_cf = 0 
            for person in self.persons_:
                if(self.is_valid_pose(person.pose)):
                    self.grid_[person.pose.x][person.pose.y] = PERSON_CELL
                    person_cf += 1

                    trajectory = "["
                    if person.move_option == "path-defined":
                        for traj_pt in person.move_path:
                            trajectory += "({},{}),".format(traj_pt.x, traj_pt.y)
                    trajectory += "]"
                    print("Loaded person {} in x:{}, y:{}, litter_gen={}, move_option={}, move_probability={}, move_step={}, path={}".format(person_cf, 
                        person.pose.x, person.pose.y, 
                        person.litter_generation, person.move_option, 
                        person.move_probability,
                        person.move_step, trajectory))
                elif log_errors:
                    print("Person x:{}, y:{} is not valid, therefore it won't be inserted".format(person.pose.x, person.pose.y))
            
            # Set up init litter poses
            if 'plastic_poses' in init_poses:
                for plastic_pose in init_poses['plastic_poses']:
                    if(self.is_valid_pose(plastic_pose)):
                        self.litter_grid_[plastic_pose.x][plastic_pose.y] = PLASTIC_LITTER_CELL
                    elif log_errors:
                        print("Plastic litter x:{}, y:{} is not valid, therefore it won't be inserted".format(plastic_pose.x, plastic_pose.y))
            if 'paper_poses' in init_poses:
                for paper_pose in init_poses['paper_poses']:
                    if(self.is_valid_pose(paper_pose)):
                        self.litter_grid_[paper_pose.x][paper_pose.y] = PAPER_LITTER_CELL
                    elif log_errors:
                        print("Paper litter x:{}, y:{} is not valid, therefore it won't be inserted".format(paper_pose.x, paper_pose.y))
            
            if self.is_valid_pose(self.plastic_agent_.pose):
                self.grid_[self.plastic_agent_.pose.x][self.plastic_agent_.pose.y] = PLASTIC_AGENT_CELL
            if self.is_valid_pose(self.paper_agent_.pose):
                self.grid_[self.paper_agent_.pose.x][self.paper_agent_.pose.y] = PAPER_AGENT_CELL
            if self.is_valid_pose(self.plastic_bin_pose):
                self.grid_[self.plastic_bin_pose.x][self.plastic_bin_pose.y] = PLASTIC_BIN_CELL
            if self.is_valid_pose(self.paper_bin_pose):
                self.grid_[self.paper_bin_pose.x][self.paper_bin_pose.y] = PAPER_BIN_CELL

    def update_map(self, obstacles: List[MGPose], persons: List[MGPerson], plastic_poses: List[MGPose], paper_poses: List[MGPose], init_poses, detection_depth:int, show_logs=False):

        # Set up an empty game grid.
        self.grid_ = [[EMPTY_CELL for x in range(self.columns)] for x in range(self.rows)]
        self.litter_grid_ = [[EMPTY_CELL for x in range(self.columns)] for x in range(self.rows)]
        
        self.detection_depth_ = detection_depth
        # positions of relevant actors/artifacts
        if("plastic_agent" in init_poses and self.is_valid_pose(init_poses["plastic_agent"])):
            self.plastic_agent_.pose = init_poses["plastic_agent"]
            self.grid_[self.plastic_agent_.pose.x][self.plastic_agent_.pose.y] = PLASTIC_AGENT_CELL
        
        if("paper_agent" in init_poses and self.is_valid_pose(init_poses["paper_agent"])):
            self.paper_agent_.pose = init_poses["paper_agent"]
            self.grid_[self.paper_agent_.pose.x][self.paper_agent_.pose.y] = PAPER_AGENT_CELL
            
        if("plastic_bin" in init_poses and self.is_valid_pose(init_poses["plastic_bin"])):
            self.plastic_bin_pose = init_poses["plastic_bin"]
            self.grid_[self.plastic_bin_pose.x][self.plastic_bin_pose.y] = PLASTIC_BIN_CELL
        
        if("paper_bin" in init_poses and self.is_valid_pose(init_poses["paper_bin"])):
            self.paper_bin_pose = init_poses["paper_bin"]
            self.grid_[self.paper_bin_pose.x][self.paper_bin_pose.y] = PAPER_BIN_CELL

        self.obstacles_ = obstacles
        self.persons_ = persons

        # Set up obstacles points
        for obs in self.obstacles_:
            if(self.is_valid_pose(obs)):
                self.grid_[obs.x][obs.y] = OBSTACLE_CELL
            elif show_logs:
                print("Obstacle x:{}, y:{} is not valid, therefore it won't be inserted".format(obs.x, obs.y))

        # Set up persons init points
        for person in self.persons_:
            if(self.is_valid_pose(person.pose)):
                self.grid_[person.pose.x][person.pose.y] = PERSON_CELL
            elif show_logs:
                print("Person x:{}, y:{} is not valid, therefore it won't be inserted".format(person.pose.x, person.pose.y))
        
        # Set up litter_poses init points
        for plastic_pose in plastic_poses:
            if(self.is_valid_pose(plastic_pose)):
                self.litter_grid_[plastic_pose.x][plastic_pose.y] = PLASTIC_LITTER_CELL
            elif show_logs:
                print("Plastic litter pose x:{}, y:{} is not valid, therefore it won't be inserted".format(plastic_pose.x, plastic_pose.y))
        for paper_pose in paper_poses:
            if(self.is_valid_pose(paper_pose)):
                self.litter_grid_[paper_pose.x][paper_pose.y] = PAPER_LITTER_CELL
            elif show_logs:
                print("Paper litter pose x:{}, y:{} is not valid, therefore it won't be inserted".format(paper_pose.x, paper_pose.y))
            
    #Draw
    def draw_world(self):
        # Create the canvas widget and add it to the Tkinter application window.
        if(self.canvas_ != None):
            if self.show_agent_view_ != '':
                for canvas_static_obj in self.canvas_static_objs_:#delete also static obj in show agent view canvas, so that you can redraw detection area grid
                    self.canvas_.delete(canvas_static_obj)

            for canvas_dynamic_obj in self.canvas_dynamic_objs_:#delete all dynamic objs
                self.canvas_.delete(canvas_dynamic_obj)

            if len(self.canvas_static_objs_) == 0 or self.show_agent_view_ != '':
                # Draw a square on the game board for every cell in the grid.
                for x in range(0, self.rows):
                    for y in range(0, self.columns):
                        realx = x * self.size_factor
                        realy = y * self.size_factor
                        drawn = False
                    
                        if self.show_agent_view_ == 'plastic_agent':
                            if x >= self.plastic_agent_.pose.x - self.detection_depth_ and x <= self.plastic_agent_.pose.x + self.detection_depth_:
                                if y >= self.plastic_agent_.pose.y - self.detection_depth_ and y <= self.plastic_agent_.pose.y + self.detection_depth_:
                                    self.canvas_static_objs_.append(self.canvas_.create_rectangle(realy, realx, realy+self.size_factor, realx+self.size_factor, fill='white', outline='blue', width=3))
                                    drawn = True
                        elif self.show_agent_view_ == 'paper_agent':
                            if x >= self.paper_agent_.pose.x - self.detection_depth_ and x <= self.paper_agent_.pose.x + self.detection_depth_:
                                if y >= self.paper_agent_.pose.y - self.detection_depth_ and y <= self.paper_agent_.pose.y + self.detection_depth_:
                                    self.canvas_static_objs_.append(self.canvas_.create_rectangle(realy, realx, realy+self.size_factor, realx+self.size_factor, fill='white', outline='orange', width=3))
                                    drawn = True
                        
                        if not drawn:
                            self.canvas_static_objs_.append(self.canvas_.create_rectangle(realy, realx, realy+self.size_factor, realx+self.size_factor, fill='white', outline='black'))
            
            # Draw a square on the game board for every cell in the grid.
            for x in range(0, self.rows):
                for y in range(0, self.columns):
                    if(self.grid_[x][y] != EMPTY_CELL or self.litter_grid_[x][y] != EMPTY_CELL):
                        realx = x * self.size_factor
                        realy = y * self.size_factor
                        if self.litter_grid_[x][y] == PLASTIC_LITTER_CELL or self.litter_grid_[x][y] == PAPER_LITTER_CELL:
                            self.draw_dynamic_obstacle(realx, realy, self.size_factor, self.litter_grid_[x][y])

                        if self.grid_[x][y] != OBSTACLE_CELL and self.grid_[x][y] != PAPER_BIN_CELL and self.grid_[x][y] != PLASTIC_BIN_CELL:
                            self.draw_dynamic_obstacle(realx, realy, self.size_factor, self.grid_[x][y])
                        else:
                            self.draw_static_obstacle(realx, realy, self.size_factor, self.grid_[x][y])

    def update_pa_trajectory(self, agent:str, move_trajectory:MGMoveTrajectory):
        for canvas_trajectory_point in self.canvas_trajectory_points_:#delete all trajectory points
            self.canvas_.delete(canvas_trajectory_point) 
        if agent == 'plastic_agent':
            self.plastic_agent_.move_trajectory = move_trajectory
        elif agent == 'paper_agent':
            self.paper_agent_.move_trajectory = move_trajectory
        
        if self.plastic_agent_.move_trajectory != None:
            self.draw_trajectory('plastic_agent', self.plastic_agent_.move_trajectory)
            
        if self.paper_agent_.move_trajectory != None:
            self.draw_trajectory('paper_agent', self.paper_agent_.move_trajectory)
    
    def draw_circle(self, canvas, x, y, r, **kwargs):
        return canvas.create_oval(y-r, x-r, y+r, x+r, **kwargs)

    def draw_trajectory(self, agent:str, move_trajectory:MGMoveTrajectory):
        color = 'blue' if agent == 'plastic_agent' else 'orange'
        for commit_pose in move_trajectory.committed:
            if self.is_valid_pose(commit_pose):
                if agent == 'plastic_agent' and self.grid_[commit_pose.x][commit_pose.y] == PLASTIC_AGENT_CELL: # do not draw committed step on top of current agent position (outdated piece of trajectory)
                    pass
                elif agent == 'paper_agent' and self.grid_[commit_pose.x][commit_pose.y] == PAPER_AGENT_CELL: # do not draw committed step on top of current agent position (outdated piece of trajectory)
                    pass
                else:
                    self.canvas_trajectory_points_.append(self.draw_circle(self.canvas_, commit_pose.x*self.size_factor + self.size_factor/2,commit_pose.y*self.size_factor + self.size_factor/2,self.size_factor/8, fill=color, outline="red", width=3))
                    
        for not_commit_pose in move_trajectory.not_committed:
            if self.is_valid_pose(not_commit_pose):
                self.canvas_trajectory_points_.append(self.draw_circle(self.canvas_, not_commit_pose.x*self.size_factor + self.size_factor/2,not_commit_pose.y*self.size_factor + self.size_factor/2,self.size_factor/16, fill=color, outline=''))
            
        if self.is_valid_pose(move_trajectory.target):
            self.canvas_trajectory_points_.append(self.draw_circle(self.canvas_, move_trajectory.target.x*self.size_factor + self.size_factor/2,move_trajectory.target.y*self.size_factor + self.size_factor/2,self.size_factor/4, fill=color, outline=''))
    
    def draw_static_obstacle(self, x, y, size, square_type):
        if(self.canvas_ != None):
            
            if square_type == PLASTIC_BIN_CELL:
                self.canvas_static_objs_.append(self.canvas_.create_image(y+4, x+4, anchor=tk.NW, image=self.plastic_bin_pic_))
            
            elif square_type == PAPER_BIN_CELL:
                self.canvas_static_objs_.append(self.canvas_.create_image(y+4, x+4, anchor=tk.NW, image=self.paper_bin_pic_))

            elif square_type == OBSTACLE_CELL: #Draw a black square on the canvas.
                self.canvas_static_objs_.append(self.canvas_.create_rectangle(y+4, x+4, y+size-2, x+size-2, fill='gray' if self.show_agent_view_ != '' else 'black', outline=''))

    def draw_dynamic_obstacle(self, x, y, size, square_type):
        if(self.canvas_ != None):
            font_size = 12 if size > 50 else 10

            #revise x and y in case drawing agent and they're moving such that they appear to be in between
            if square_type == PLASTIC_AGENT_CELL and self.plastic_agent_.moving:
                x = ((self.plastic_agent_.old_pose.x + self.plastic_agent_.pose.x) / 2) * self.size_factor
                y = ((self.plastic_agent_.old_pose.y + self.plastic_agent_.pose.y) / 2) * self.size_factor
            if square_type == PAPER_AGENT_CELL and self.paper_agent_.moving:
                x = ((self.paper_agent_.old_pose.x + self.paper_agent_.pose.x) / 2) * self.size_factor
                y = ((self.paper_agent_.old_pose.y + self.paper_agent_.pose.y) / 2) * self.size_factor

            if square_type == PLASTIC_AGENT_CELL and self.show_agent_view_ == '':
                self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+4, x+4, anchor=tk.NW, image=self.plastic_agent_pic_))
                if self.plastic_agent_.holding > 0: #check if loaded
                    self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+18, x+18, anchor=tk.NW, image=self.plastic_litter_pic_mini_))
                self.canvas_dynamic_objs_.append(self.canvas_.create_text(y+size-12, x+12, text=("{}").format(self.plastic_agent_.holding), fill="black", font=('Helvetica {} bold'.format(font_size))))
            
            elif square_type == PAPER_AGENT_CELL and self.show_agent_view_ == '':
                self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+4, x+4, anchor=tk.NW, image=self.paper_agent_pic_))
                if self.paper_agent_.holding > 0: #check if loaded
                    self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+24, x+24, anchor=tk.NW, image=self.paper_litter_pic_mini_))
                self.canvas_dynamic_objs_.append(self.canvas_.create_text(y+size-12, x+12, text=("{}").format(self.paper_agent_.holding), fill="black", font=('Helvetica {} bold'.format(font_size))))

            elif square_type == PERSON_CELL: #Draw our bro Paolo on the canvas.
                self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+16, x+4, anchor=tk.NW, image=self.person_agent_pic_))
            
            elif square_type == PAPER_LITTER_CELL:
                self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+12, x+12, anchor=tk.NW, image=self.paper_litter_pic_))
            
            elif square_type == PLASTIC_LITTER_CELL:
                self.canvas_dynamic_objs_.append(self.canvas_.create_image(y+12, x+12, anchor=tk.NW, image=self.plastic_litter_pic_))
    
    def count_litter(self):
        count = 0
        for x in range(0, self.rows):
            for y in range(0, self.columns):
                count += 1 if self.litter_grid_[x][y] == PLASTIC_LITTER_CELL or self.litter_grid_[x][y] == PAPER_LITTER_CELL else 0
        return count

    def upd_holding_agent(self, agent, holding_upd):
        if agent == PLASTIC_AGENT_CELL:
            self.plastic_agent_.holding += holding_upd
            if self.plastic_agent_.holding < 0:
                self.plastic_agent_.holding = 0
                return False, 0

            return True, self.plastic_agent_.holding

        elif agent == PAPER_AGENT_CELL:
            self.paper_agent_.holding += holding_upd
            if self.paper_agent_.holding < 0:
                self.paper_agent_.holding = 0
                return False, 0

            return True, self.paper_agent_.holding
        
        return False, -1

    def remove_litter(self, x, y):
        if x < self.rows and y < self.columns:
            self.litter_grid_[x][y] = EMPTY_CELL
            return True
        
        return False

    def move_persons(self):
        upd_persons = []
        # Set up persons init points
        for person in self.persons_:
            
            new_poses = self.allowable_moves(person.pose)
            new_pose = None

            if person.move_option == "random":
                #person movement probability
                pm_prob = person.move_probability
                if pm_prob > 1:
                    pm_prob = 1.0
                elif pm_prob < 0:
                    pm_prob = 0.0
                people_move_prob_max = (int) (100 * pm_prob)
                if len(new_poses) > 0 and random.randint(1,100) < people_move_prob_max+1:# chance of movement in new_pose for person if movement is allowed
                    new_pose = new_poses[random.randint(0, len(new_poses)-1)]

            elif person.move_option == "path-defined" and person.move_path_index < len(person.move_path):
                # person movement path
                
                # check if next step is allowed
                allowed_move = person.elapsed_epochs_since_last_move + 1 >= person.move_step

                if allowed_move:
                    allowed_move = False
                    for possible_new_pose in new_poses:
                        if possible_new_pose.x == person.move_path[person.move_path_index].x and possible_new_pose.y == person.move_path[person.move_path_index].y:
                            allowed_move = True
                            break
                
                if allowed_move:    
                    new_pose = person.move_path[person.move_path_index] #next step in path
                    person.move_path_index = (person.move_path_index + 1) % len(person.move_path)

            if new_pose !=None:
                person.elapsed_epochs_since_last_move = 0
                #new pose allowed && selected

                #litter_generation probability
                lg_prob = person.litter_generation
                if lg_prob > 1:
                    lg_prob = 1.0
                elif lg_prob < 0:
                    lg_prob = 0.0
                litter_gen_prob_max = (int) (100 * lg_prob)
                if self.litter_grid_[person.pose.x][person.pose.y] == EMPTY_CELL and random.randint(1,100) < litter_gen_prob_max+1:# chance of being a polluter this round iff cell is not already polluted
                    self.litter_grid_[person.pose.x][person.pose.y] = PLASTIC_LITTER_CELL if random.randint(0, 1) == 1 else PAPER_LITTER_CELL
                
                self.grid_[person.pose.x][person.pose.y] = EMPTY_CELL 

                person.pose = new_pose 
                self.grid_[person.pose.x][person.pose.y] = PERSON_CELL
            else:
                person.elapsed_epochs_since_last_move += 1

            upd_persons.append(person)
        
        self.persons_ = upd_persons
    
    def move_agents(self):
        if self.plastic_agent_.moving:
            self.plastic_agent_.moving = False
            self.plastic_agent_.old_pose = self.plastic_agent_.pose
        
        if self.paper_agent_.moving:
            self.paper_agent_.moving = False
            self.paper_agent_.old_pose = self.paper_agent_.pose

    def draw_next_epoch(self):
        self.move_persons()
        
        moving = []
        if self.plastic_agent_.moving:
            moving.append('plastic_agent')
        if self.paper_agent_.moving:
            moving.append('plastic_agent')

        self.draw_world()
        self.move_agents()
        return moving
    
    def move_agent(self, agent, cmd_move):

        if agent == PLASTIC_AGENT_CELL:
            new_pose = self.try_upd_agent_pose(self.plastic_agent_.pose, cmd_move)
            if new_pose != None:
                self.plastic_agent_.old_pose = self.plastic_agent_.pose
                self.plastic_agent_.moving = True
                self.grid_[self.plastic_agent_.pose.x][self.plastic_agent_.pose.y] = EMPTY_CELL
                self.plastic_agent_.pose = new_pose
                self.grid_[self.plastic_agent_.pose.x][self.plastic_agent_.pose.y] = PLASTIC_AGENT_CELL
                return True
                

        elif agent == PAPER_AGENT_CELL:
            new_pose = self.try_upd_agent_pose(self.paper_agent_.pose, cmd_move)
            if new_pose != None:
                self.paper_agent_.old_pose = self.paper_agent_.pose
                self.paper_agent_.moving = True
                self.grid_[self.paper_agent_.pose.x][self.paper_agent_.pose.y] = EMPTY_CELL
                self.paper_agent_.pose = new_pose
                self.grid_[self.paper_agent_.pose.x][self.paper_agent_.pose.y] = PAPER_AGENT_CELL
                return True
        
        return False
                
    
    def try_upd_agent_pose(self, agent_pose, cmd_move):
        new_pose = MGPose(agent_pose.x, agent_pose.y)
        new_pose.x += cmd_move.x
        new_pose.y += cmd_move.y
        
        if self.is_valid_pose(new_pose) and self.grid_[new_pose.x][new_pose.y] == EMPTY_CELL:
            return new_pose
        else:
            return None

    def get_grid(self):
        return self.grid_

    def get_litter_grid(self):
        return self.litter_grid_


