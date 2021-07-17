# This is the belief message used by the BDI agents in order to express an aspect regarding their 
# current knowledge of the world.
# Every belief message should be able to be mapped into a PDDL 2.1 predicate or a PDDL 2.1 function

# @name      -> name of the predicate/fluent
# @pddl_type -> INSTANCE/PREDICATE/FUNCTION (other int values are not supported)
# @params    -> PDDL instances' names making up for the predicate/fluent relation
# @value     -> real value to be considered just in case of type==FUNCTION

uint8 INSTANCE_TYPE=1
uint8 PREDICATE_TYPE=2
uint8 FUNCTION_TYPE=3

string name
uint8 pddl_type
string[] params
float32 value