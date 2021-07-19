#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters: ./belief_cleaner.sh <agent_id>"
    exit -1
fi

source ~/.bashrc

charge=60
if [ $1 -eq 1 ]; then
    charge=90
fi

echo "I'm going to add some belief to agent$1"


# add belief    (bathroom$1 waypoint) - INSTANCE
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'bathroom$1', params: {'waypoint'}, pddl_type: INSTANCE}" -1
# add belief    (kitchen$1 waypoint) - INSTANCE
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'kitchen$1', params: {'waypoint'}, pddl_type: INSTANCE}" -1
# add belief    (bedroom$1 waypoint) - INSTANCE
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'bedroom$1', params: {'waypoint'}, pddl_type: INSTANCE}" -1

# add belief    (in agent$1 dock$1) - PREDICATE -> forge INSTANCE (agent$1 robot) and INSTANCE (dock$1 waypoint)
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'in', params: {'agent$1', 'dock$1'}, pddl_type: PREDICATE}" -1

# add belief    (workfree agent$1) - PREDICATE 
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'workfree', params: {'agent$1'}, pddl_type: PREDICATE}" -1
# add belief    (recharging_station dock$1) - PREDICATE 
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'recharging_station', params: {'dock$1'}, pddl_type: PREDICATE}" -1
# add belief    (battery_charge=90) - FUNCTION
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'battery_charge', params: {}, pddl_type: FUNCTION, value: $charge}" -1
exit 0