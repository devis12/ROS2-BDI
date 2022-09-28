#!/usr/bin/env python3
from typing import List
from .pose import MGPose

class MGMoveTrajectory:

    def __init__(self, target:MGPose, committed:List[MGPose], not_committed:List[MGPose]):
        self.target = target
        self.committed = committed
        self.not_committed = not_committed