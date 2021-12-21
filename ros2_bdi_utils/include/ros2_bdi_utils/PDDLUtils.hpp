#ifndef PDDL__UTILS_H_
#define PDDL__UTILS_H_

#include <vector>
#include <string>

namespace PDDLUtils
{
    /*
        Returns all the items within a PlanItem.action string
        E.g. "(dosweep sweeper kitchen)" -> ["dosweep", "sweeper", "kitchen"]
    */
    std::vector<std::string> extractPlanItemActionElements(const std::string& planItemAction); 
    
}  // namespace PDDLUtils

#endif  // PDDL__UTILS_H_