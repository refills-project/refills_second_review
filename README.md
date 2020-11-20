# refills_second_review
Configuration and launch-files for the use case scenarios of the second REFILLS review. 


## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic```:
```
source /opt/ros/kinetic/setup.bash         # start using ROS Indigo
mkdir -p ~/refills_ws/src                  # create directory for workspace
cd ~/refills_ws                            # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/refills-project/refills_second_review/master/rosinstall/catkin.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/refills_ws/devel/setup.bash       # source new overlay
```


touch universal_robot/ur_kinematics/CATKIN_IGNORE