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
    time.sleep(25)

    table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
    coke = p.loadURDF('coke/coke.urdf', [0.7,-0.3, 0.9]) 

    pick_pose = [0.2,-0.25,0]
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    robot.plan_and_drive_to_pose(pick_pose , base_limits,obstacles=[table, coke]) 
    time.sleep(1)
    robot.pick_up(coke,'right_arm')
    time.sleep(5)
    robot.place_at([0.7, 0, 1.1], coke, 'right_arm')

    time.sleep(30)


