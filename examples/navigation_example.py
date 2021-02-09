import numpy as np 
import pybullet as p
import pybullet_data
import sys, os, time
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
from src.buff_digit import Buff_digit


if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) 

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath('../models')

    floor = p.loadURDF('floor/floor.urdf',useFixedBase=True) 
    robot = Buff_digit(client)
    time.sleep(5)

    #SE2 poses (x, y, theta)
    pose1 = [0.2,-0.25,0]
    pose2 = [-1.0,-1.0,-1.57]
    pose3 = [-4.0,0.5,-3.1415 ]
    pose4 = [-1.0,1.0,1.57]
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    for i in range(10):
        robot.plan_and_drive_to_pose(pose1 , base_limits,obstacles=[]) 
        time.sleep(1)
        robot.plan_and_drive_to_pose(pose2, base_limits,obstacles=[]) 
        time.sleep(1)
        robot.plan_and_drive_to_pose(pose3, base_limits,obstacles=[]) 
        time.sleep(1) 
        robot.plan_and_drive_to_pose(pose4, base_limits,obstacles=[]) 
        time.sleep(1)
    time.sleep(30)
