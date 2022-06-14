# This is the plan execution message used by the BDI agents in order to express which plan they are executing,
# mentioning the desire it is going to fulfill and the actions needed to do so, alongside the current status of 
# the mentioned execution (i.e. which action is currently executed and at which point it is at)

# @target               -> desire which is going to be fulfilled by the plan
# @actions              -> array of actions which composed the body of the plan in execution
# @executing            -> specific info about the action(s) currently in execution 
# @current_time         -> time elapsed since plan started
# @planned_deadline   -> estimated planned deadline in seconds
# @estimated_deadline   -> run time estimated deadline in seconds
# @status               -> plan status

string RUNNING="0"
string ABORT="1"
string SUCCESSFUL="2"
string WAITING="3"

Desire target
BDIActionExecutionInfo[] actions_exec_info
float32     current_time
float32     planned_deadline
float32     estimated_deadline
string      status