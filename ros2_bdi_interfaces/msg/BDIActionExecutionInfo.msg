# This is the action execution message used by the BDI agents in order to express which action of the current selected
# plan they're executing and at which point the execution is at

# @index                    -> index of action in the plan
# @wait_action_indexes      -> indexes of the action's predecessors which need to be waited beforee this action can start
# @name                     -> name of the action currently in execution, e.g. "a1"
# @args                     -> args of the action currently in execution, e.g. "[p1, p2, p3]"
# @planned_start            -> planned time of start for this action
# @actual_start             -> actual time of start for this action
# @exec_time                -> exec_time in sec (prec. up to ms) of the action wrt. start of its execution
# @duration                 -> estimated time in sec for its completion (based on domain info)
# @progress                 -> progress status percentage in [0.0-1.0] of the current action execution 
# @status                   -> action exec. status code
# @committed                -> action exec. committed status

int16 RUNNING=0
int16 FAILED=1
int16 SUCCESSFUL=2
int16 WAITING=3
int16 UNKNOWN=4

int16       index
int16[]     wait_action_indexes
string      name
string[]    args
float32     planned_start
float32     actual_start
float32     exec_time
float32     duration
float32     progress
int16       status
bool        committed