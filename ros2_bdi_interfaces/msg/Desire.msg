# This is the desire message used by the BDI agents in order to express one of their desirable state
# of the world that they want to reach through intentions 
# Every desire message should be able to be mapped into a PDDL 2.1 goal clause through the Belief array

# @name     -> name of the desire
# @value    -> array of belief expressing the desirable state of the world to be reached
# @priority -> real value between 0 and 1 to express the priority of the desire (higher the value, higher the priority) 
# @deadline -> desirable deadline in seconds

string      name
Belief[]    value
float32     priority
float32     deadline