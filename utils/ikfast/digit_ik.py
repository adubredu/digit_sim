import numpy as np 
import sys
import os 
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),'left_arm'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),'right_arm'))
from .left_arm.left_arm_ik import left_ik
from .right_arm.right_arm_ik import right_ik
import pybullet_planning as pyplan 

def solve_ik(position, orientation, armname):
	position = list(position)
	if armname == 'left_arm':
		ik = left_ik
	else:
		ik = right_ik
		position[1]+=0.15
	n_joints = ik.getDOF()
	tf_matrix = pyplan.tform_from_pose((position, orientation))
	ee_pose = tf_matrix[:-1]
	joint_confs = ik.inverse(ee_pose.reshape(-1).tolist())
	n_solutions = int(len(joint_confs)/n_joints) 
	joint_confs = np.asarray(joint_confs).reshape(n_solutions,n_joints)
	if len(joint_confs) == 0: 
		yield None
	for conf in joint_confs: 
		yield conf

def rot_to_quat(R): 
	tr = R[0,0]+R[1,1]+R[2,2]

	if tr > 0.0:
		S = np.sqrt(tr+1.0)*2;
		qw = 0.25*S
		qx = (R[2,1]-R[1,2])/S
		qy = (R[0,2]-R[2,0])/S
		qz = (R[1,0]-R[0,1])/S

	elif ((R[0,0]>R[1,1]) and (R[0,0]>R[2,2])):
		S = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
		qw = (R[2,1]-R[1,2])/S 
		qx = 0.25*S 
		qy = (R[0,1]+R[1,0])/S
		qz = (R[0,2]+R[2,0])/S 

	elif (R[1,1]>R[2,2]):
		S = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2
		qw = (R[0,2]-R[2,0])/S 
		qx = (R[0,1]+R[1,0])/S 
		qy = 0.25*S 
		qz = (R[1,2]+R[2,1])/S 

	else:
		S = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2
		qw = (R[1,0]-R[0,1])/S
		qx = (R[0,2]+R[2,0])/S 
		qy = (R[1,2]+R[2,1])/S
		qz = 0.25*S

	return [qx,qy,qz,qw]


def solve_fk(conf, armname):
	if armname == 'left_arm':
		fk = left_ik
	else:
		fk = right_ik
	pose = fk.forward(conf)
	position = [pose[3], pose[7], pose[11]]
	R = np.asarray(pose).reshape(3,4)[:3,:3]
	orientation = rot_to_quat(R)
	if armname == "right_arm":
		position[1]-=0.15
	return [position, orientation]  

if __name__=='__main__':
	gen = solve_ik((0,0,0), (0,0,0,1),'right_arm')
	a = next(gen)
	f = solve_fk(a, 'right_arm')
	print(f)
