# This is the plan message used by the BDI agents in order to express a list of selected actions
# to be performed in order to reach a specific desire
# Every desire message should be able to be mapped into a PDDL 2.1 goal clause through the Belief array

# @desire       -> desire to be achieved through the plan successful execution
# @actions      -> actions to be performed + when to start each of them + how it is estimated they are going to last
# @precondition -> condition to be checked before the plan exec. is initiated (DNF expression)
# @context      -> conditions to be checked over all the plan exec. (DNF expression)

Desire                      desire
plansys2_msgs/PlanItem[]    actions
ConditionsDNF               precondition
ConditionsDNF               context