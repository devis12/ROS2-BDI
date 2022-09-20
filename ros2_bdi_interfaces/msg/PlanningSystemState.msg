# This is the plansys2 node state used by the PlanSys2 Monitor node of the BDI agents in order to 
# notify the other nodes about the current state (active or not) of the 4 fundamental nodes of PlanSys2
# 
# @domain_expert_active     true iff domain expert node in the current ns appear to be active (check through ns/domain_expert/get_state)
# @problem_expert_active    true iff problem expert node in the current ns appear to be active (check through ns/problem_expert/get_state)
# @offline_planner_active   true iff planner node within PSYS2 (e.g. POPF, TFD) in the current ns appear to be active (check through ns/planner/get_state)
# @online_planner_active    true iff online planner node (e.g. JavaFF) is up and ready to receive requests
# @executor_active          true iff executor node in the current ns appear to be active (check through ns/executor/get_state)

#TODO improve online_planner_active info (i.e. is it searching, inactive, idling???)

bool domain_expert_active
bool problem_expert_active
bool offline_planner_active
bool online_planner_active
bool executor_active