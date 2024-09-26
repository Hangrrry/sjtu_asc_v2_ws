gnome-terminal -- bash -c " source /home/amov/catkin_ws/devel/setup.bash;echo 'timing 20s';sleep 10s;echo   mb`date  +_%Y_%m_%d_%H_%M_%S` >>  ~/teststart.txt ;
rosrun mission_offboard mission_offboard | tee  ./record/recordmo`date  +_%Y_%m_%d_%H_%M_%S`.txt"
