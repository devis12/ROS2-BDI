# This is the condition message used by the BDI agents in order to express a condition to be checked
# before or during an execution

# @condition -> belief to be checked (e.g. predicate true/false, fluent equals/greater/smaller)
# @check     -> enum type expressing the check to perform

# accepted values for @check = {"TRUE", "FALSE", "EQUALS", "GREATER", "SMALLER"}

Belief  condition
string  check