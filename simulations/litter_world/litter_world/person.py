#!/usr/bin/env python3

from math import inf
from .pose import MGPose

class MGPerson:

    def __init__(self, start:MGPose, litter_generation:float, move_option:str, value):
        self.pose = start
        self.litter_generation = litter_generation
        self.elapsed_epochs_since_last_move = 0
        self.move_option = move_option if move_option in ["random", "path-defined"] else "random"
        self.move_probability = 0.0
        self.move_path = []
        self.move_path_index = 0
        self.move_step = inf
        if self.move_option == "random":
            self.move_probability = value if isinstance(value, float) and value >= 0.0 and value <= 1.0 else 0.0
        elif self.move_option == "path-defined":
            if "path" in value and isinstance(value["path"], list):
                for point in value["path"]:
                    if point["x"] >= 0 and point["y"] >= 0:
                        self.move_path.append(MGPose(point["x"], point["y"]))
            self.move_step = value["move_step"] if "move_step" in value else inf

