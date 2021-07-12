# This is the plan execution message used by the BDI agents in order to express which plan they are executing,
# mentioning the desire it is going to fulfill and the actions needed to do so, alongside the current status of 
# the mentioned execution (i.e. which action is currently executed and at which point it is at)

# @target               -> desire which is going to be fulfilled by the plan
# @actions              -> array of actions which composed the body of the plan in execution
# @executing            -> specific info about the action currently in execution 
# @current_time         -> time elapsed since plan started
# @estimated_deadline   -> estimated deadline in seconds

Desire target
plansys2_msgs/PlanItem[] actions
BDIActionExecutionInfo executing
float32     current_time
float32 estimated_deadline