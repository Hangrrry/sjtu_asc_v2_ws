gnome-terminal  -- bash -c "source /opt/ros/melodic/setup.bash ;
source /home/amov/catkin_ws/devel/setup.bash;
echo   px4`date  +_%Y_%m_%d_%H_%M_%S` >>  ~/teststart.txt;
roslaunch mavros px4.launch "
