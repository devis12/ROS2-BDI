#include "ros2_bdi_utils/BDIPlanLibrary.hpp"

#include <cstring>

using std::string;

typedef enum {
    BELIEFS, CONDITIONS, PLANS, 
    TARGETS, SUCCESSORS,
    PRECONDITIONS, CONDITION_CLAUSES, CLAUSE_LITERALS} PlanLibTable;

#define BELIEFS_TABLE "beliefs"
#define CONDITIONS_TABLE "conditions"
#define PLANS_TABLE "plans"
#define TARGETS_TABLE "targets"
#define PRECONDITIONS_TABLE "preconditions" //p1 e1
#define CONDITION_CLAUSES_TABLE "condition_clauses" //e1 c1
#define CLAUSE_LITERALS_TABLE "clause_literals" //c1 l1, dove l1 is a condition
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
        case BELIEFS:
            create_query = "CREATE TABLE %s("
                      "bId          INT     PRIMARY KEY NOT NULL, "
                      "name         TEXT    NOT NULL, "
                      "pddl_type    INT     NOT NULL, "
                      "params       TEXT    NOT NULL, "
                      "value        REAL );";
            create_query_c = new char[create_query.size() + strlen(BELIEFS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), BELIEFS_TABLE);
            break;

        case CONDITIONS:
            create_query = "CREATE TABLE %s("
                      "cId              INT     PRIMARY KEY NOT NULL, "
                      "check            TEXT    NOT NULL, "
                      "belief_to_check  INT     NOT NULL, "
                      "FOREIGN KEY (belief_to_check) REFERENCES %s(bId) ON UPDATE CASCADE ON DELETE CASCADE);";
            create_query_c = new char[create_query.size() + strlen(CONDITIONS_TABLE) + strlen(BELIEFS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), CONDITIONS_TABLE, BELIEFS_TABLE);
            break;

        case PLANS:
            create_query = "CREATE TABLE %s("
                      "pId          INT     PRIMARY KEY NOT NULL, "
                      "plan         TEXT    NOT NULL, "
                      "deadline     REAL    NOT NULL, "
                      "hashTarget   TEXT    NOT NULL "
                      " );";
            create_query_c = new char[create_query.size() + strlen(PLANS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), PLANS_TABLE);
            break;

        case TARGETS:
            create_query = "CREATE TABLE %s("
                      "pId          INT     NOT NULL, "
                      "bId          INT     NOT NULL, "
                      "PRIMARY KEY(pId, bId), "
                      "FOREIGN KEY (pId) REFERENCES %s(pId), "
                      "FOREIGN KEY (bId) REFERENCES %s(bId) "
                      ");";
            create_query_c = new char[create_query.size() + strlen(TARGETS_TABLE) + strlen(PLANS_TABLE) + strlen(BELIEFS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), TARGETS_TABLE, PLANS_TABLE, BELIEFS_TABLE);
            break;

        case PRECONDITIONS:
            create_query = "CREATE TABLE %s("
                      "pId          INT     NOT NULL, "
                      "expId          INT     NOT NULL, "
                      "PRIMARY KEY(pId, expId), "
                      "FOREIGN KEY (pId) REFERENCES %s(pId) ON UPDATE CASCADE ON DELETE CASCADE"
                      ");";
            create_query_c = new char[create_query.size() + strlen(PRECONDITIONS_TABLE) + strlen(PLANS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), PRECONDITIONS_TABLE, PLANS_TABLE);
            break;

        case CONDITION_CLAUSES:
            create_query = "CREATE TABLE %s("
                      "expId        INT     NOT NULL, "
                      "clauseId     INT     NOT NULL, "
                      "PRIMARY KEY(expId, clauseId) "
                      ");";
            create_query_c = new char[create_query.size() + strlen(CONDITION_CLAUSES_TABLE)];
            sprintf(create_query_c, create_query.c_str(), CONDITION_CLAUSES_TABLE);
            break;

        case CLAUSE_LITERALS:
            create_query = "CREATE TABLE %s("
                      "clauseId     INT     NOT NULL, "
                      "literalId    INT     NOT NULL, "
                      "PRIMARY KEY(clauseId, literalId), "
                      "FOREIGN KEY (literalId) REFERENCES %s(cId) ON UPDATE CASCADE ON DELETE CASCADE"
                      ");";
            create_query_c = new char[create_query.size() + strlen(CLAUSE_LITERALS_TABLE) + strlen(CONDITIONS_TABLE)];
            sprintf(create_query_c, create_query.c_str(), CLAUSE_LITERALS_TABLE, CONDITIONS_TABLE);
            break;

        case SUCCESSORS:
            create_query = "CREATE TABLE %s("
                      "pId              INT     NOT NULL, "
                      "pSuccId          INT     NOT NULL, "
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
        if(!createTable(DB, table));
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
    
    
    // BELIEFS TABLE INIT
    if(!tryInitTable(DB, BELIEFS, BELIEFS_TABLE))
        return false;

    // CONDITIONS TABLE INIT
    if(!tryInitTable(DB, CONDITIONS, CONDITIONS_TABLE))
        return false;

    // PLANS TABLE INIT
    if(!tryInitTable(DB, PLANS, PLANS_TABLE))
        return false;

    // TARGETS TABLE INIT
    if(!tryInitTable(DB, TARGETS, TARGETS_TABLE))
        return false;
    
    // PRECONDITIONS TABLE INIT
    if(!tryInitTable(DB, PRECONDITIONS, PRECONDITIONS_TABLE))
        return false;
    
    // CONDITION_CLAUSES TABLE INIT
    if(!tryInitTable(DB, CONDITION_CLAUSES, CONDITION_CLAUSES_TABLE))
        return false;

    // CLAUSE_LITERALS TABLE INIT
    if(!tryInitTable(DB, CLAUSE_LITERALS, CLAUSE_LITERALS_TABLE))
        return false;

    // SUCCESSORS TABLE INIT
    if(!tryInitTable(DB, SUCCESSORS, SUCCESSORS_TABLE))
        return false;

    return true;
}

// bool BDIPlanLibrary::insertBelief(sqlite3* DB)
// {
//     string insert_query = "INSERT INTO conditions (cId,check ,belief_to_check) VALUES(1,test,1);";
//     char* msg_error;
//     int res = sqlite3_exec(DB, insert_query.c_str(), NULL, 0, &msg_error);

//     return res == SQLITE_OK;
// }

bool BDIPlanLibrary::closeConnection(sqlite3* DB)
{
    sqlite3_close(DB);
    return true;
}