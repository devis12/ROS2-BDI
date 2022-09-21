#!/usr/bin/env python3

from .pose import MGPose

class MGMoveTrajectory:

    def __init__(self, target:MGPose, committed:list, not_committed:list):
        self.target = target
        self.committed = committed
        self.not_committed = not_committed