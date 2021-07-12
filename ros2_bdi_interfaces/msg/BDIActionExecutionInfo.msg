# This is the action execution message used by the BDI agents in order to express which action of the current selected
# plan they're executing and at which point the execution is at

# @index        -> index of the current plan item action in execution
# @name         -> name of the action currently in execution
# @args         -> args of the action currently in execution
# @started      -> array of actions which composed the body of the plan in execution
# @status       -> status [0.0-1.0] of the current action execution 

uint16      index
string      name
string[]    args
float32     started
float32     status