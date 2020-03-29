
This package provides examples programs to use the standalone TRAC-IK solver and related code.

#### Author: Honghao Lv
#### Email:lvhonghao@zju.edu.cn
#### Date-Version: 2020.03.29 V1.0

## Requirements:
OS: Ubuntu16.04
ROS: Kinetic

Two ros nodes for testing the trac_ik have been finished. 
One is for a very simple robot whose urdf model is created manual, you can just run:
'''
roslaunch test.launch
'''
The second for the yumi ik solution which is very convinient for checking the ik results, you can run:
'''
roslaunch yumitestik.launch
'''
