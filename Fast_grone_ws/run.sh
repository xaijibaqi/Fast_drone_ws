gnome-terminal -- bash -c "source devel/setup.bash;roslaunch px4ctrl run_ctrl.launch;ecex bash"
gnome-terminal -- bash -c "source devel/setup.bash;roslaunch ego_planner single_run_in_exp.launch;ecex bash"
gnome-terminal -- bash -c "source devel/setup.bash;roslaunch ego_planner rviz.launch;ecex bash"

