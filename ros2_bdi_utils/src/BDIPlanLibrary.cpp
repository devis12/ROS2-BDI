#include "ros2_bdi_utils/BDIPlanLibrary.hpp"

#include <cstring>

using std::string;
using std::vector;

using BDIManaged::ManagedBelief;
using PlanLibrary::BDIPlanLibrary;

/*
    create table utility function
    select the right query to be performed in order to instantiate the table in the db, returns true if query executed successfully
*/
bool BDIPlanLibrary::createTable(sqlite3* DB, const PlanLibTable& table)
{
    string create_query = "";
    switch(table){

        case PLANS:
            create_query = "CREATE TABLE " + PLANS_TABLE + "("
                      "pId              INTEGER     PRIMARY KEY AUTOINCREMENT NOT NULL, "
                      "plan             TEXT    NOT NULL, "
                      "deadline         REAL    NOT NULL, "
                      "preconditions    TEXT, "
                      "target           TEXT    NOT NULL "
                      " );";
            break;
        
        case SUCCESSORS:
            create_query = "CREATE TABLE " + SUCCESSORS_TABLE + "("
                      "pId              INTEGER     NOT NULL, "
                      "pSuccId          INTEGER     NOT NULL, "
                      "PRIMARY KEY(pId, pSuccId), "
                      "FOREIGN KEY (pId) REFERENCES " + PLANS_TABLE + "(pId) ON UPDATE CASCADE ON DELETE CASCADE, "
                      "FOREIGN KEY (pSuccId) REFERENCES " + PLANS_TABLE + "(pId) ON UPDATE CASCADE ON DELETE CASCADE "
                      ");";
            break;
    }

    char* msg_error;
    int res = sqlite3_exec(DB, create_query.c_str(), NULL, 0, &msg_error);
    sqlite3_free(msg_error);

    return res == SQLITE_OK;
}

/*
    create table utility function if table is not already defined in the db
    select the right query to be performed in order to instantiate the table in the db, 
    returns true if query executed successfully OR selected table is already defined
*/
bool BDIPlanLibrary::tryInitTable(sqlite3* DB, const PlanLibTable& table, const string& table_name)
{
    string select_query = "SELECT * FROM " + table_name + ";";
    int select_res = 0;

    select_res = sqlite3_exec(DB, select_query.c_str(), NULL, 0, NULL);
    if (select_res != SQLITE_OK)
    {
        if(!createTable(DB, table))
            return false;//table init has failed
    }
    return true; //either table already existed or creation has been successful

}

bool BDIPlanLibrary::initPlanLibrary()
{
    sqlite3* DB;
    
    int exit = 0;
    exit = sqlite3_open(db_filepath_.c_str(), &DB);
    if (exit)
        return false;
    
    // PLANS TABLE INIT
    if(!tryInitTable(DB, PLANS, PLANS_TABLE))
        return false;

    // SUCCESSORS TABLE INIT
    if(!tryInitTable(DB, SUCCESSORS, SUCCESSORS_TABLE))
        return false;

    // Close connection to DB
    sqlite3_close(DB);

    return true;
}

/*
    Store new plan, if not already present in the db
    Plan stored with deadline, preconditions and target

    return generated id for stored plan
*/
int BDIPlanLibrary::insertPlan(const BDIManaged::ManagedPlan& mp)
{
    int stored_plan_id = -1;
    sqlite3* DB;
    
    int exit = 0;
    exit = sqlite3_open(db_filepath_.c_str(), &DB);
    if (exit)
        return false;

    // retrieve params for the query
    string plan = mp.toPsys2PlanString(); 
    string deadline = std::to_string(mp.getPlannedDeadline());
    string precondition = mp.getPrecondition().toString();
    string target = "";
    vector<ManagedBelief> target_value = mp.getPlanTarget().getValue();
    for(int i=0; i<target_value.size(); i++)
        target += target_value[i].toString() + ((i != target_value.size()-1) ? "&" : "");
    
    string insert_query =  "INSERT INTO " + PLANS_TABLE + " (plan,deadline,preconditions,target) "  
         "VALUES ('"+plan+"',"+deadline+",'"+precondition+"','"+target+"'); ";

    // prevent sql injections attacks (or most of them)
    // why?? is it needed? because it's my code and it has to shine :-) 
    // (and to be less performant as possible)
    if(insert_query.find_first_of(";") != insert_query.find_last_of(";"))
        return false;

    int insert_res = sqlite3_exec(DB, insert_query.c_str(), NULL, 0, NULL);

    if(insert_res == SQLITE_OK)
    {
        string retrieve_id_query = "select seq from sqlite_sequence where name='"+PLANS_TABLE+"'";
        sqlite3_stmt *ppStmt2;
        int r1 = sqlite3_prepare_v2(DB, retrieve_id_query.c_str(), strlen(retrieve_id_query.c_str()), &ppStmt2, 0);
        if(r1 == SQLITE_OK && sqlite3_step(ppStmt2) == SQLITE_ROW)
            stored_plan_id = sqlite3_column_int(ppStmt2,0);
        int r3 = r1 != SQLITE_OK? r1 : sqlite3_finalize(ppStmt2);
    }

    // Close connection to DB
    sqlite3_close(DB);

    return stored_plan_id;
}

/*
    Store in the db relationship mp1 -> mp2
*/
bool BDIPlanLibrary::markSuccessors(const BDIManaged::ManagedPlan& mp1, const BDIManaged::ManagedPlan& mp2)
{
    sqlite3* DB;
    
    int exit = 0;
    exit = sqlite3_open(db_filepath_.c_str(), &DB);
    if (exit)
        return false;

    // retrieve params for the query
    string planID1 = std::to_string(mp1.getPlanLibID());
    string planID2 = std::to_string(mp2.getPlanLibID());

    string insert_query =  "INSERT INTO " + SUCCESSORS_TABLE + " (pId,pSuccId) "  
         "VALUES ('"+planID1+"',"+planID2+"); ";

    // prevent sql injections attacks (or most of them)
    // why?? is it needed? because it's my code and it has to shine :-) 
    // (and to be less performant as possible)
    if(insert_query.find_first_of(";") != insert_query.find_last_of(";"))
        return false;

    int insert_res = sqlite3_exec(DB, insert_query.c_str(), NULL, 0, NULL);
    // Close connection to DB
    sqlite3_close(DB);

    return insert_res == SQLITE_OK;
}