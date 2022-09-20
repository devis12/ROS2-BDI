# This is the plan execution minified message used by the BDI agents in order to express which plan they are executing,
# mentioning the desire it is going to fulfill and the actions needed to do so, alongside the current status of 
# the mentioned execution (i.e. which action is currently executed and at which point it is at)

# @target               -> desire's value which is going to be fulfilled by the plan
# @actions              -> array of actions which composed the body of the plan in execution

Belief[]                    target_value
BDIActionExecutionInfoMin[] actions_exec_info