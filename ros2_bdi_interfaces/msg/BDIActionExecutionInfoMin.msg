# This is the action execution minified message used by the BDI agents in order to express which action of the current selected
# plan they're executing and at which point the execution is at

# @name                     -> name of the action currently in execution, e.g. "a1"
# @args                     -> args of the action currently in execution, e.g. "[p1, p2, p3]"
# @planned_start            -> planned time of start for this action
# @progress                 -> progress status percentage in [0.0-1.0] of the current action execution 
# @status                   -> action exec. status code
# @committed                -> action exec. committed status

int16 RUNNING=0
int16 FAILED=1
int16 SUCCESSFUL=2
int16 WAITING=3
int16 UNKNOWN=4

string      name
string[]    args
float32     planned_start
float32     progress
int16       status
bool        committed