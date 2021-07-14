#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters: ./desire_cleaner.sh <agent_id>"
    exit -1
fi

source ~/.bashrc

echo "I'm going to add a desire to agent$1"

# add desire    (bathroom$1 waypoint) - INSTANCE
ros2 topic pub /agent1/add_desire ros2_bdi_interfaces/msg/Desire "{name: 'clean_rooms', priority: 0.9, deadline: 28.0, value:[{name: 'cleaned', params: {'kitchen$1'}, type: PREDICATE},{name: 'cleaned', params: {'bathroom$1'}, type: PREDICATE}]}" -1
exit 0