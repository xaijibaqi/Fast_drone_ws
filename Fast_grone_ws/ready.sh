gnome-terminal -- bash -c "source devel/setup.bash;roslaunch mavros px4.launch"
sleep 3
gnome-terminal -- bash -c "rosrun mavros mavcmd long   511 105 5000 0 0 0 0 0;echo “Set the frequency of imu/data_raw to 200Hz”;rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0;echo “Set the frequency of imu/data_raw to 200Hz”;rostopic hz /mavros/imu/data_raw;ecex bash"
echo 按任意键继续
read -n 1
echo 继续