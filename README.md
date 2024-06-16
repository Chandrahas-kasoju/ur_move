# ur_move
Implentation of PTP an LIN motions on ur10 robot arm
## Pre-requsites
Clone ROS2 UR_robot packages in your ros2_ws 
```
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b humble
```
The moveit_config contain ompl pilpeline for planning tasks. In order to use Pliz motion planner we need to add the following lines to the ur_moveit.launch.py file
 ```
pilz_planning_pipeline_config = {
        "planning_pipelines": ["pilz_industrial_motion_planner"],
        "default_planning_pipeline": "pilz_industrial_motion_planner",
        "pilz_industrial_motion_planner": {},
        "move_group": {},
        "robot_description_planning":{},
    }
    pilz_planning_yaml = load_yaml("ur_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml")
    pilz_planning_pipeline_config["move_group"].update(pilz_planning_yaml)
    pilz_planning_pipeline_config["pilz_industrial_motion_planner"].update(pilz_planning_yaml) 

    pilz_cartesian_limits_yaml = load_yaml("ur_pilz_demo", "config/pilz_cartesian_limits.yaml")
    pilz_planning_pipeline_config["robot_description_planning"].update(pilz_cartesian_limits_yaml)
```
Then add the following line to the parameters of the move_group_node in launch file
```
pilz_planning_pipeline_config,
```
## Launching the robot
First the ros2 UR driver has to be launched
```
ros2 launch ur_robot_driver ur10.launch.py  robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=scaled_joint_trajectory_controller
```
Then launch the moveit_config 
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true
```
Finally run the cpp file to move the robot to prepose and pickpose
```
ros2 run ur_move ur_move
```

