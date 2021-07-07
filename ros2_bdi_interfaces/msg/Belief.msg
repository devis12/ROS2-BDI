# This is the belief message used by the BDI agents in order to express an aspect regarding their 
# current knowledge of the world.
# Every belief message should be able to be mapped into a PDDL 2.1 predicate or a PDDL 2.1 fluent

# @name     -> name of the predicate/fluent
# @type     -> PREDICATE/FLUENT (other string values are not supported)
# @params   -> PDDL instances' names making up for the predicate/fluent relation
# @value    -> real value to be considered just in case of type==FLUENT

string name
string type
string[] params
float32 value