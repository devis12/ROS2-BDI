#ifndef BDIPlanLibrary__UTILS_H_
#define BDIPlanLibrary__UTILS_H_

#include <string>
#include <sqlite3.h>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"

namespace BDIPlanLibrary
{
    /*
        Open connection to plan library, saving reference pointer to it
            @filepath where plan library is going to be stored in the fs
            @DB pointer to conn. object
    */
    bool openConnection(const std::string& filepath, sqlite3* DB);

    /*
        Close connection to plan library via reference pointer to it
            @DB pointer to conn. object
    */
    bool closeConnection(sqlite3* DB);

    /*
        Store new plan, if not already present in the db
        Plan stored with deadline, preconditions and target
    */
    bool insertPlan(const BDIManaged::ManagedPlan& mp, sqlite3* DB);

};  // namespace BDIPlanLibrary

#endif  // BDIPlanLibrary__UTILS_H_