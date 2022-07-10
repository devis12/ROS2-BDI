#ifndef PlanLibrary__UTILS_H_
#define PlanLibrary__UTILS_H_

#include <string>
#include <sqlite3.h>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"

typedef enum {PLANS,SUCCESSORS} PlanLibTable;

std::string PLANS_TABLE = "plans";
std::string SUCCESSORS_TABLE = "successors";

namespace PlanLibrary
{
    class BDIPlanLibrary{
        public:
            BDIPlanLibrary(const std::string& db_filepath): 
            db_filepath_(db_filepath)
            {}

            /*
                Open connection to plan library, saving reference pointer to it
                    @filepath where plan library is going to be stored in the fs
            */
            bool initPlanLibrary();

            /*
                Store new plan, if not already present in the db
                Plan stored with deadline, preconditions and target
                return generated id for stored plan
            */
            int insertPlan(const BDIManaged::ManagedPlan& mp);

            /*
                Store in the db relationship mp1 -> mp2
            */
            bool markSuccessors(const BDIManaged::ManagedPlan& mp1, const BDIManaged::ManagedPlan& mp2);


        private:

            /*
                create table utility function
                select the right query to be performed in order to instantiate the table in the db, returns true if query executed successfully
            */
            bool createTable(sqlite3* DB, const PlanLibTable& table);

            /*
                create table utility function if table is not already defined in the db
                select the right query to be performed in order to instantiate the table in the db, 
                returns true if query executed successfully OR selected table is already defined
            */
            bool tryInitTable(sqlite3* DB, const PlanLibTable& table, const std::string& table_name);

            std::string db_filepath_;
    };
};  // namespace PlanLibrary

#endif  // PlanLibrary__UTILS_H_