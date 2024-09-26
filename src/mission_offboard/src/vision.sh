gnome-terminal -- bash -c "
source /home/amov/anaconda3/etc/profile.d/conda.sh;
conda activate yolov8; source /home/amov/tst/devel/setup.bash;echo 'timing 10s';sleep 10s;echo   mb`date  +_%Y_%m_%d_%H_%M_%S` >>  ~/teststart.txt ;
cd /home/amov/tst/src/vision_pose/scripts; python new_pos.py | tee  ./record/recordmo`date  +_%Y_%m_%d_%H_%M_%S`.txt "