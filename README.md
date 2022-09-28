# ROS2-BDI
A MAS (Multi Agent System) framework aiming to run on top of ROS2 Foxy, emulating the BDI architecture for implementing the agents' behaviour. The framework makes usage of an automated PDDL planner for computing plans in the means-end reasoning phase. Specifically it uses [PlanSys2](https://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io/design/index.html).

For **user's documentation** follow this [link](https://devis12.github.io/ROS2-BDI.github.io/).

If you want to have a look at one of the provided **demo** example, **click on the image below**
[![PAAMS22_ROS2xBDI_DEMO](https://raw.githubusercontent.com/devis12/ROS2-BDI.github.io/main/img/PAAMS_ROS2xBDI_sim.jpg)](https://www.youtube.com/watch?v=zB2HvCR5H9E)

## DISCLAIMER 
I am currently in the process of integrating a new extended version consisting of a tighter bound between planning and execution to take on the challenge of an improved run time support in more dynamic and evolving environments and this readme as well as the user's docs might be a little bit outdated at the moment, but I plan to fix them as soon as possible. Nonetheless, you should still be able to use the standard version in the main branch following the guide below and none of the changes already merged and integrated within the main should be breaking changes, even considering the development process for new agents. 

NOTE: Just make sure to follow the tips and guidelines below and if anything might be blocking for you, post an issue and/or contact me via the email you'll find in the user docs: I am happy to give you support and assist you!

## Requirements 
In order to compile everything, **make sure [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html#) and [PlanSys2](https://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io/) have been already installed and setup** (highly suggested to use [this forked version of PlanSys2](https://github.com/RTI-BDI/ros2_planning_system)). Use this [guide](https://github.com/devis12/ROS2-BDI/blob/main/ros2_psys2_setup.pdf) to walk you through this process in latest LTS Ubuntu distributions (e.g. 20.04 LTS). With other recent Ubuntu-based distributions (e.g. Mint 20+) you may encounter some issues in the installation of dependencies which can be easily overcome by specifying the option `--os="ubuntu:"` if using rosdep or installing them manually (e.g. `sudo apt install ros-foxy-popf`). 

Moreover, you'll need [Boost libraries](https://www.boost.org/) and [yaml-cpp-0.6.0](https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.6.0). It's suggested to compile both from source, even though Boost should offer a more accessible [script for the installation](https://www.boost.org/doc/libs/1_77_0/tools/build/doc/html/index.html#bbv2.installation). The documentation with the installation guide for building yaml-cpp can be found [here](https://yaml-cpp.docsforge.com/#how-to-build): it's really important to build it as a **shared library** (check the presence of the `-fPIC` option added in compilation in the CMakeLists.txt if the `BUILD_SHARED_LIBS` is set to `ON`). If you want to avoid any struggle, in Ubuntu 20.04 both can be installed via apt:
```
sudo apt install libboost-dev libyaml-cpp0.6
```

To avoid any further unpleasant interaction, it's highly recommended to disable Groot monitoring in PlanSys2. To do that, just go under the `plansys2_executor` package and within the script of the only launch file present ("*executor_launch.py*") add the configuration `'enable_groot_monitoring': False`  to the parameters of the `executor_cmd` **Node** (approximately line 55).

## Building

Once you download the current repository in your `<ros2 workspace>/src/` folder and installed the required libraries, move back to the root of your ROS2 workspace and give the standard `colcon build` command in order to compile all the packages, loading all the launch and additional files in the packages' shared folders. It's suggested to be more specific though (especially if you already have other packages in your workspace and you want to avoid to compile them all every time) building exclusively the packages of ROS2-BDI:
```
colcon build --packages-select ros2_bdi_interfaces  ros2_bdi_utils  ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core && colcon build --symlink-install --packages-ignore ros2_bdi_tests
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
ros2 topic echo /cleaner/belief_set              # belief set echo
ros2 topic echo /cleaner/desire_set              # desire set echo
ros2 topic echo /cleaner/plan_execution_info     # plan execution progress echo
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
ros2 topic pub /cleaner/add_belief ros2_bdi_interfaces/msg/Belief "{name: 'cleaned', params: {'kitchen'}, pddl_type: 2}"
ros2 topic pub /cleaner/del_belief ros2_bdi_interfaces/msg/Belief "{name: 'cleaned', params: {'kitchen'}, pddl_type: 2}"
```

_Additional note_: The respective launch file can be edit in  `ros2_bdi_tests/launch/bdi_cleaner_simple.launch.py`. E.g. it's easy to notice that the `AGENT_NAME` declared as the first instruction in `generate_launch_description()` function it's the parameter that will affect its ID, thus the agent's namespace definition which makes for the prefix for its topics and services (by default it is set to `cleaner` as you can see from the example topic echo cmds put above).

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

### Webots demo

In order to launch and test out the webots demo, [Webots](https://www.cyberbotics.com/) needs to be installed alongside with the `webots_ros2_driver` too. You can install them via apt `sudo apt install ros-foxy-webots-ros2`. Then you can build the following packages supporting the demos which comes within this repo too.
```
colcon build --packages-select webots_ros2_simulations_interfaces webots_ros2_simulations ros2_bdi_on_webots --symlink-install
```
Once building is complete (note that ROS2-BDI core packages have to be already built and sourced!!), you can source again your local workspace and then you're ready to launch the simulation. First, you might want to start the setup scenario in webots with:
```
ros2 launch webots_ros2_simulations blocks_world.launch.py
```
Then, you can launch in **three different terminals** with the environment already sourced, the three carriers agents:
```
ros2 launch ros2_bdi_on_webots carrier_a.launch.py
ros2 launch ros2_bdi_on_webots carrier_b.launch.py
ros2 launch ros2_bdi_on_webots carrier_c.launch.py
```
Finally, you can launch in a new terminal (again with the environment already sourced), the gripper agent:
```
ros2 launch ros2_bdi_on_webots gripper_a.launch.py
```
Feel free to play with the init. sets and init. reactive rules to trigger new scenarios (e.g. you might want to change the `moving_boxes` fluent check from 2 to 1, to trigger the movement of a carrier before the loading of both 2 boxes is complete, causing the replanning of the gripper).  Another option is to update the init. desire set for the gripper agent (as suggested in the file comments), so that just `gripper_a` and `carrier_a` are involved (you can avoid start the other carriers): this is almost necessary in case your pc is not powerful enough to handle the expensive load of having all four agents and the simulator runnning.

Examine the demos to get a better grasp on how the framework should be exploited: hope the materials will inspire you to do great things ;-)
Feel free to contact us for further information
