# This is the desire message used by the BDI agents in order to express one of their desirable state
# of the world that they want to reach through intentions 
# Every desire message should be able to be mapped into a PDDL 2.1 goal clause through the Belief array

# @name         -> name of the desire
# @value        -> array of belief expressing the desirable state of the world to be reached
# @priority     -> real value between 0 and 1 to express the priority of the desire (higher the value, higher the priority) 
# @deadline     -> desirable deadline in seconds
# @precondition -> precondition in order to start an intention satisfying the given desire (DNF expression)
# @context      -> context conditions which has to remain true while satisfying the given desire (DNF expression)
# @rollback_belief_add  -> belief to add in case of fulfilling plan abortion
# @rollback_belief_del  -> belief to del in case of fulfilling plan abortion

string          name
Belief[]        value
float32         priority
float32         deadline
ConditionsDNF   precondition
ConditionsDNF   context
Belief[]        rollback_belief_add
Belief[]        rollback_belief_del