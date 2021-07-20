# This is the plansys2 node state used by the PlanSys2 Monitor node of the BDI agents in order to 
# notify the other nodes about the current state (active or not) of the 4 fundamental nodes of PlanSys2
# 
# @domain_expert_active     true iff domain expert node in the current ns appear to be active (check through ns/domain_expert/get_state)
# @problem_expert_active    true iff problem expert node in the current ns appear to be active (check through ns/problem_expert/get_state)
# @planner_active           true iff planner node in the current ns appear to be active (check through ns/planner/get_state)
# @executor_active          true iff executor node in the current ns appear to be active (check through ns/executor/get_state)


bool domain_expert_active
bool problem_expert_active
bool planner_active
bool executor_active