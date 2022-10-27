#!/usr/bin/env python3

##################
# opmp_test.py
# Sunhong Kim
# tjsghd101@naver.com
# 22. Sep. 2021
##################

from multiprocessing import Process, Manager
import sys
import numpy as np
import matplotlib.pyplot as plt
import os
import rospy
import tf

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import time

from ctypes import *
import socket

from opmp_ctrl.srv import *

#############################################
################## Options ##################
#############################################

command_activate = True

#############################################################################
################## Manager for global data multiprocessing ##################
#############################################################################

manager = Manager()
_q = manager.dict()
_q_d = manager.dict()
_x_d = manager.dict()
_global_flag = manager.dict()

_q['q'] = [0.0]*6; _q['q_dot'] = [0.0]*6;
_q_d['qd'] = [0.0]*6; _q_d['qd_dot'] = [0.0]*6; _q_d['qd_ddot'] = [0.0]*6; 

_x_d['pos'] = [0.0]*3; _x_d['ori'] = [0.0]*3

_global_flag['isHoming'] = False;


#####################################################
################## OpenManipulator ##################
#####################################################


### opmp_cmd_run ###
def opmp_cmd_run():
    cnt2rad1 = 3.141592/501923
    cnt2rad2 = 3.141592/303750

    rad2cnt1 = 501923/3.141592
    rad2cnt2 = 303750/3.141592

    opmp_frq = 240
    traj = Trajectory(6)
    pos_job = [0.0, -0.785, 0.393, 0.0, 0.393, 0.0]
    traj_flag = [0]*6
    traj_res = [0]*6
    
    rospy.init_node('talker_arm', anonymous=True)
    
    rospy.wait_for_service('/sync_get_position')
    try:
        getPos = rospy.ServiceProxy('/sync_get_position', SyncGetPosition)
        getVel = rospy.ServiceProxy('/sync_get_velocity', SyncGetVelocity)
        res_pos = getPos(1,2,3,4,5,6)
        res_vel = getVel(1,2,3,4,5,6)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    pub = rospy.Publisher('/sync_set_position', SyncSetPosition, queue_size=1000)
    
    rate = rospy.Rate(opmp_frq)
    
    opmp_msg = SyncSetPosition()
    opmp_msg.id1 = 1; opmp_msg.id2 = 2; opmp_msg.id3 = 3; opmp_msg.id4 = 4; opmp_msg.id5 = 5; opmp_msg.id6 = 6;
    opmp_msg.position1 = 0; opmp_msg.position2 = 0; opmp_msg.position3 = 0; opmp_msg.position4 = 0; opmp_msg.position5 = 0; opmp_msg.position6 = 0;

    while not rospy.is_shutdown():
        init_time=time.time()
        
        res_pos = getPos(1,2,3,4,5,6)
        res_vel = getVel(1,2,3,4,5,6)
        
        _q['q'] = [res_pos.position1*cnt2rad1, res_pos.position2*cnt2rad1, res_pos.position3*cnt2rad1, res_pos.position4*cnt2rad1, res_pos.position5*cnt2rad2, res_pos.position6*cnt2rad2]
        _q['q_dot'] = [res_vel.velocity1*cnt2rad1, res_vel.velocity2*cnt2rad1, res_vel.velocity3*cnt2rad1, res_vel.velocity4*cnt2rad1, res_vel.velocity5*cnt2rad2, res_vel.velocity6*cnt2rad2]

        for i in range(6):
         
            if traj_flag[i]==0:
                traj.SetPolynomial5th(i,_q['q'][i],pos_job[i],init_time,2.0)
                _q_d['qd'][i]=_q['q'][i]
                traj_res[i]=_q['q'][i]
                traj_flag[i]=1

            elif traj_flag[i]==1:
                tmp_res,tmp_flag=traj.Polynomial5th(i,init_time)
                _q_d['qd'][i] = tmp_res[0]
                traj_res[i] = tmp_res[0]
                if tmp_flag == 0:
                    traj_flag[i]=2
                    _global_flag['isHoming']=True

        _q_d['qd']=[traj_res[0],traj_res[1],traj_res[2],traj_res[3],traj_res[4],traj_res[5]]

        opmp_msg.position1 = int(_q_d['qd'][0] * rad2cnt1)
        opmp_msg.position2 = int(_q_d['qd'][1] * rad2cnt1)
        opmp_msg.position3 = int(_q_d['qd'][2] * rad2cnt1)
        opmp_msg.position4 = int(_q_d['qd'][3] * rad2cnt1)
        opmp_msg.position5 = int(_q_d['qd'][4] * rad2cnt2)
        opmp_msg.position6 = int(_q_d['qd'][5] * rad2cnt2)


        pub.publish(opmp_msg)
        rate.sleep()
           


if __name__ == '__main__': 
    
    opmp_task = Process(target=opmp_cmd_run, args=())
    
    try:
        opmp_task.start()
        
        opmp_task.join()
        
    except KeyboardInterrupt:
    	opmp_task.terminate()
    	
    
    
    
    
