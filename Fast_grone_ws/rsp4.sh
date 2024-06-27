gnome-terminal -- bash -c "source devel/setup.bash;sh shfiles/rspx4.sh;exec bash"
sleep 10
gnome-terminal -- bash -c "rostopic echo /vins_fusion/imu_propagate"