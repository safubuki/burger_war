#!/bin/bash

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node
# 1. randomBot
# roslaunch burger_war sim_robot_run.launch

# 2. sim_level1
roslaunch burger_war sim_level_1_cheese.launch

# 3. sim_level1
# roslaunch burger_war sim_level_2_teriyaki.launch
