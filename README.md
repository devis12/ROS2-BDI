# ROS2-BDI
A MAS (Multi Agent System) framework aiming to run on top of ROS2 Foxy, emulating the BDI architecture for implementing the agents' behaviour. The framework makes usage of an automated PDDL planner for computing plans in the means-end reasoning phase. Specifically it uses [PlanSys2](https://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io/design/index.html).

## Requirements 
In order to compile everything, **make sure [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html#) and [PlanSys2](https://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io/) have been already installed and setup**. Use this [guide](https://docs.google.com/document/d/1vuOYsJIQ1J7aEH1UpamHjjB-0t2JeE9OccRkzy4Tm4o/edit?usp=sharing) to walk you through this process in latest LTS Ubuntu distributions (e.g. 20.04 LTS). 

Moreover, you'll need [Boost libraries](https://www.boost.org/) and [yaml-cpp-0.6.0](https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.6.0). It's suggested to compile both from source, even though Boost should offer a more accessible [script for the installation](https://www.boost.org/doc/libs/1_55_0/doc/html/bbv2/installation.html). The documentation with the installation guide for yaml-cpp can be found [here](https://yaml-cpp.docsforge.com/#problems).

## Building

Once you download the current repository in your `<ros2 workspace>/src/` folder and installed the required libraries, move back to the root of your ROS2 workspace and give the standard `colcon build` command in order to compile all the packages, loading all the launch and additional files in the packages' shared folders. It's suggested to be more specific though (especially if you already have other packages in your workspace and you want to avoid to compile them all every time) building exclusively the packages of ROS2-BDI:
```
colcon build --packages-select ros2_bdi_interfaces ros2_bdi_bringup ros2_bdi_utils ros2_bdi_tests --symlink-install
```
The above command will allow you to compile just the packages of ROS2-BDI, linking all the demo launch files which you can find within `ros2_bdi_tests/launch`, so that you don't need to recompile everything after alterations to the python launch scripts.
Remember to **source your ROS2 setup.bash**, before try to launch any executable (otherwise you won't be able to find them):
```
source /opt/ros/foxy/setup.bash
source ~/<ros2 workspace>/install/setup.bash
```

## Demos
**REMEMBER** to source the installation in every terminal you'll use: does not matter if you use for running executables or ROS2 CLI calls (i.e. `ros2 topic echo, ros2 service call, ...` or similar).

### Launch BDI CLEANER_SIMPLE demo
Use the respective py. launch file:
```
ros2 launch ros2_bdi_tests bdi_cleaner_simple.launch.py
```
Inspect belief set, desire set and plan execution info topics through the following commands given in other terminal(s):
```
ros2 topic echo /agent1/belief_set              # belief set echo
ros2 topic echo /agent1/desire_set              # desire set echo
ros2 topic echo /agent1/plan_execution_info     # plan execution progress echo
```

The initial belief and desire sets are specified as YAML files in `ros2_bdi_tests/init_cleaner_simple/` folder and selected through the launch file. Consult the interface description to understand how to specify Belief and Desire msgs:
```
ros2 interface show ros2_bdi_interfaces/msg/Belief
ros2 interface show ros2_bdi_interfaces/msg/BeliefSet
ros2 interface show ros2_bdi_interfaces/msg/Desire
ros2 interface show ros2_bdi_interfaces/msg/DesireSet

```

To further alter the agent's belief set (or desire set), while it's running is also possible to use a terminal injecting/removing belief (desire) by publishing on the respective topics:

```
# for beliefs
ros2 topic pub /agent1/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'cleaned', params: {'kitchen1'}, pddl_type: 2}"
ros2 topic pub /agent1/del_belief ros2_bdi_interfaces/msg/Belief "{name: 'cleaned', params: {'kitchen1'}, pddl_type: 2}"
```

_Additional note_: The respective launch file can be edit in  `ros2_bdi_tests/launch/bdi_cleaner_simple.launch.py`. E.g. it's easy to notice that the `AGENT_NAME` declared as the first instruction in `generate_launch_description()` function it's the parameter that will affect its ID, thus the agent's namespace definition which makes for the prefix for its topics and services (by default it is set to `agent1` as you can see from the example topic echo cmds put above).

### Launch BDI CLEANER+SWEEPER demo
Use the respective py. launch file to launch the _sweeper_:
```
ros2 launch ros2_bdi_tests bdi_sweeper.launch.py
```
Inspect belief set, desire set and plan execution info topics for the sweeper through the following commands given in other terminal(s):
```
ros2 topic echo /sweeper/belief_set              # sweeper belief set echo
ros2 topic echo /sweeper/desire_set              # sweeper desire set echo
ros2 topic echo /sweeper/plan_execution_info     # sweeper plan execution progress echo
```

Use the respective py. launch file to launch the _cleaner_:
```
ros2 launch ros2_bdi_tests bdi_cleaner.launch.py
```
Inspect belief set, desire set and plan execution info topics for the sweeper through the following commands given in other terminal(s):
```
ros2 topic echo /cleaner/belief_set              # cleaner belief set echo
ros2 topic echo /cleaner/desire_set              # cleaner desire set echo
ros2 topic echo /cleaner/plan_execution_info     # cleaner plan execution progress echo
```
As for the case of the simpler demo above, launch files can be easily found and edited in `ros2_bdi_tests/launch/` folder for triggering different behaviours, as well as it's possible to publish in the respective belief/desire topics of the two agents to lead them through different execution paths. Same apply for belief and desire init. files which can be found in `ros2_bdi_tests/init_cleaner_sweeper/` and are then selected in the py. launch files.
