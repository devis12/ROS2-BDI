# This is the condition message used by the BDI agents in order to express a condition to be checked
# before or during an execution

# @condition -> belief to be checked (e.g. predicate true/false, fluent equals/greater/smaller)
# @check     -> string enum-like type expressing the check to perform

# accepted values for @check = {"EX", "T", "F", "EQ", "GR", "LE", "GREQ", "LEQ"}

# to be used with instances
string EXISTS_CHECK="EX"

# to be used with predicates
string TRUE_CHECK="T"
string FALSE_CHECK="F"

# to be used with fluents
string EQUALS_CHECK="EQ"
string GREATER_CHECK="GR"
string GREATER_OR_EQUALS_CHECK="GREQ"
string SMALLER_CHECK="LE"
string SMALLER_OR_EQUALS_CHECK="LEQ"

Belief  condition_to_check
string  check