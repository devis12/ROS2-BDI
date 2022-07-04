#include "ros2_bdi_utils/BDIPlanLibrary.hpp"

#include <cstring>

using std::string;
using std::vector;

using BDIManaged::ManagedBelief;

typedef enum {PLANS,SUCCESSORS} PlanLibTable;

#define PLANS_TABLE "plans"
#define SUCCESSORS_TABLE "successors"

/*
    create table utility function
    select the right query to be performed in order to instantiate the table in the db, returns true if query executed successfully
*/
bool createTable(sqlite3* DB, const PlanLibTable& table)
{
    string create_query = "";
    char* create_query_c;
    switch(table){

        case PLANS:
            create_query = "CREATE TABLE %s("
                      "pId              INTEGER     PRIMARY KEY AUTOINCREMENT NOT NULL, "
                      "plan             TEXT    NOT NULL, "
                      "deadline         REAL    NOT NULL, "
                      "preconditions    TEXT, "
                      "target           TEXT    NOT NULL "
                      " );";
            create_query_c = new char[create_query.size() + strlen(PLANS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), PLANS_TABLE);
            break;
        
        case SUCCESSORS:
            create_query = "CREATE TABLE %s("
                      "pId              INTEGER     NOT NULL, "
                      "pSuccId          INTEGER     NOT NULL, "
                      "PRIMARY KEY(pId, pSuccId), "
                      "FOREIGN KEY (pId) REFERENCES %s(pId) ON UPDATE CASCADE ON DELETE CASCADE, "
                      "FOREIGN KEY (pSuccId) REFERENCES %s(pId) ON UPDATE CASCADE ON DELETE CASCADE "
                      ");";
            create_query_c = new char[create_query.size() + strlen(SUCCESSORS_TABLE) + 2*strlen(PLANS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), SUCCESSORS_TABLE, PLANS_TABLE, PLANS_TABLE);
            break;
    }

    char* msg_error;
    int res = sqlite3_exec(DB, create_query_c, NULL, 0, &msg_error);
    sqlite3_free(msg_error);
    delete create_query_c;

    return res == SQLITE_OK;
}

/*
    create table utility function if table is not already defined in the db
    select the right query to be performed in order to instantiate the table in the db, 
    returns true if query executed successfully OR selected table is already defined
*/
bool tryInitTable(sqlite3* DB, const PlanLibTable& table, const char* table_name)
{
    string base_select_query = "SELECT * FROM %s;";
    char* select_query_c;
    int select_res = 0;

    select_query_c = new char[base_select_query.size() + strlen(table_name)];
    sprintf(select_query_c, base_select_query.c_str(), table_name);
    select_res = sqlite3_exec(DB, select_query_c, NULL, 0, NULL);
    delete select_query_c;
    if (select_res != SQLITE_OK)
    {
        if(!createTable(DB, table))
            return false;//table init has failed
    }
    return true; //either table already existed or creation has been successful

}

bool BDIPlanLibrary::openConnection(const string& filepath, sqlite3* DB)
{
    int exit = 0;
    exit = sqlite3_open(filepath.c_str(), &DB);
    if (exit)
        return false;
    
    // PLANS TABLE INIT
    if(!tryInitTable(DB, PLANS, PLANS_TABLE))
        return false;

    // SUCCESSORS TABLE INIT
    if(!tryInitTable(DB, SUCCESSORS, SUCCESSORS_TABLE))
        return false;

    return true;
}

/*
    Store new plan, if not already present in the db
    Plan stored with deadline, preconditions and target
*/
bool BDIPlanLibrary::insertPlan(const BDIManaged::ManagedPlan& mp, sqlite3* DB)
{
    string query =  "INSERT INTO %s (plan,deadline,preconditions,target) "  
         "VALUES ('%s',%s,'%s','%s'); ";
    string plan = mp.toPsys2PlanString(); 
    string deadline = std::to_string(mp.getPlannedDeadline());
    string precondition = mp.getPrecondition().toString();
    string target = "";
    vector<ManagedBelief> target_value = mp.getPlanTarget().getValue();
    for(int i=0; i<target_value.size(); i++)
        target += target_value[i].toString() + ((i != target_value.size()-1) ? "&" : "");

    char* insert_query_c = new char[strlen(PLANS_TABLE)+plan.length()+deadline.length()+precondition.length()+target.length()];
    sprintf(insert_query_c, query.c_str(), PLANS_TABLE, plan.c_str(), deadline.c_str(), precondition.c_str(), target.c_str());
    // prevent sql injections attacks (or most of them)
    // why?? is it needed? because it's my code and it has to shine :-) 
    // (and to be less performant as possible)
    query = string{insert_query_c};
    if(query.find_first_of(";") != query.find_last_of(";"))
        return false;

    std::cout << "Executing query: " << insert_query_c << std::flush << std::endl;
    int insert_res = sqlite3_exec(DB, insert_query_c, NULL, 0, NULL);
    delete insert_query_c;

    return insert_res == SQLITE_OK;
}

bool BDIPlanLibrary::closeConnection(sqlite3* DB)
{
    sqlite3_close(DB);
    return true;
}