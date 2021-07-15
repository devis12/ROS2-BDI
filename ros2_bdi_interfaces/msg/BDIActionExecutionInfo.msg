# This is the action execution message used by the BDI agents in order to express which action of the current selected
# plan they're executing and at which point the execution is at

# @index        -> index of the current plan item action in execution
# @name         -> name of the action currently in execution
# @args         -> args of the action currently in execution
# @exec_time    -> exec_time in sec (prec. up to ms) of the action wrt. start of its execution
# @duration     -> estimated time in sec for its completion (based on domain info)
# @status       -> status percentage in [0.0-1.0] of the current action execution 

uint16      index
string      name
string[]    args
float32     exec_time
float32     duration
float32     status