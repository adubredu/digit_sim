import numpy as np 
import time
import pybullet_data
import pybullet as p
import pybullet_planning as pyplan  
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
from utils.ikfast.digit_ik import solve_ik, solve_fk

def ikfunc(pose):
    return solve_ik(pose[0], pose[1], "right_arm")

class Buff_digit:
    def __init__(self, client):
        robot_id = p.loadURDF('../models/buff_digit/digit_hover.urdf',[0,0,0.1])
        self.client = client 
        self.id = robot_id
        self.state = "turn"
        self.max_angular_speed = 1.0
        self.max_linear_speed = 2.0
        self.turn_tol = 0.05
        self.trans_tol = 0.3
        self.final_trans_tol = 0.05
        self.left_wheel_id = 36
        self.right_wheel_id = 37
        self.wheel_width = 0.8
        self.arms_ee = {'left_arm':13, 'right_arm':26}
        self.arms_base = {'left_arm':'left_jlink0', 
                        'right_arm':'right_jlink0'}
        self.arm_joints = {'left_arm':[3,4,8,10,11,12], 
                           'right_arm':[16,17,21,23,24,25]}
        self.elbow_joints={'left_arm':(8,-1.35), 'right_arm':(21,1.35)}
        self.joint_index_ranges = {'left_arm':(0,6), 
                                   'right_arm':(6,12)} 
        self.grasped = {'left_arm':0, 'right_arm':0}
        self.start_up_robot()
        time.sleep(5)
        
    def tuck_arm(self, armname, right_side=False, left_side=True): 
        right_start_conf = [-1.3587102702612153, -0.9894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        self.default_right_conf = right_start_conf
        left_start_conf = [0, 1.1894200000000005, -1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if right_side:
            right_start_conf = [0, -1.1894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if not left_side:
            left_start_conf = [-1.3587102702612153, 0.9894200000000005, -1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if armname == 'left_arm':
            self.drive_arm_joints(self.arm_joints[armname], left_start_conf)
        else:
            self.drive_arm_joints(self.arm_joints[armname], right_start_conf)



    def start_up_robot(self):
        self.lowerLimits,self.upperLimits,self.jointRanges,self.restPoses = self.get_joint_ranges()
        self.tuck_arm('left_arm')
        self.tuck_arm('right_arm') 
        self.get_camera_images()


    def angle_diff(self, angle1, angle2):
        diff = angle2 - angle1
        while diff < -np.pi: diff += 2.0*np.pi
        while diff > np.pi: diff -= 2.0*np.pi
        return diff

    def tf_arm_frame(self, pose, armname): 
        basename = self.arms_base[armname]
        baseid = pyplan.link_from_name(self.id, basename)
        base_to_world = pyplan.invert(pyplan.get_link_pose(self.id, baseid))
        base_to_pose = pyplan.multiply(base_to_world, pose)
        return base_to_pose 

    def tf_world_frame(self, base_to_pose, armname):
        basename = self.arms_base[armname]
        baseid = pyplan.link_from_name(self.id, basename)
        world_to_base = pyplan.get_link_pose(self.id, baseid)
        world_to_pose = pyplan.multiply(world_to_base, base_to_pose)
        return world_to_pose


    def plan_and_drive_to_pose(self, goal_pose, limits=((-12.5, -12.5), (12.5, 12.5)) ,obstacles=[]): 
        path = pyplan.plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        self.drive_along_path(path)

    def plan_to_pose(self, goal_pose, limits=((-12.5, -12.5), (12.5, 12.5)) ,obstacles=[]):
        path = pyplan.plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        return path 


    def drive_along_path(self, path, camera_follow=False):
        for pose in path:
            pyplan.set_base_values(self.id, pose) 
            if camera_follow:
                pose = pyplan.get_point(self.id)
                p.resetDebugVisualizerCamera(3, 90, -30, pose)
            time.sleep(0.05)



    def get_joint_ranges(self, includeFixed=False):
        all_joints = pyplan.get_movable_joints(self.id)
        upper_limits = pyplan.get_max_limits(self.id, all_joints)
        lower_limits = pyplan.get_min_limits(self.id, all_joints)

        jointRanges, restPoses = [], []

        numJoints = p.getNumJoints(self.id)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.id, i)

            if includeFixed or jointInfo[3] > -1:
                rp = p.getJointState(self.id, i)[0]
                jointRanges.append(2)
                restPoses.append(rp)

        return lower_limits, upper_limits, jointRanges, restPoses


    def drive_arm_joints(self, joints, jointPoses):
        assert(len(joints)==len(jointPoses))

        for i,j in enumerate(joints):
            p.setJointMotorControl2(bodyIndex=self.id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i], maxVelocity=0.5)#, positionGain=0.001)
            time.sleep(0.01)
            p.stepSimulation()


    def plan_and_execute_arm_motion(self, position, orientation, armname): 
        pose = self.tf_arm_frame((position, orientation), armname)
        gen = solve_ik(pose[0], pose[1],armname)
        a = next(gen)
        conf = next(gen)

        if conf is not None:
            joints = self.arm_joints[armname] 
            self.drive_arm_joints(joints, conf)
        else:
            print('No IK solution found')

    def compute_ik_to_pose(self, position, orientation, armname):
        pose = self.tf_arm_frame((position, orientation), armname)
        try:
            gen = solve_ik(pose[0], pose[1],armname)
            a = next(gen)
            conf = next(gen)

            if conf is not None:
                return conf
            else:
                return self.default_right_conf
        except:
            return self.default_right_conf
 
    def get_generic_top_grasp(self, pose): 
        height = 0.3
        position, _ = pose
        position = list(position)
        position[2]+=(height/1.8)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        return position, orientation 

    def get_top_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        height = aabb[1][2] - aabb[0][2]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[2]+=(height/1.6)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        return position, orientation 

    def get_side_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        width = aabb[1][0] - aabb[0][0]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[0] -=(width/2.0)
        orientation = p.getQuaternionFromEuler((1.57,0,0))
        return position, orientation 

    def get_put_on_pose(self, topid, bottomid):
        bot_pose = pyplan.get_point(bottomid)
        aabb = p.getAABB(bottomid)
        botheight = aabb[1][2] - aabb[0][2]
        top_pose = list(bot_pose)
        top_pose[2] += botheight/1.8
        aabb = p.getAABB(topid)
        topheight = aabb[1][2] - aabb[0][2]
        top_pose[2] += topheight/1.8
        return top_pose


    def plan_arm_motion(self, pose, armname, obstacles=[]):
        #returns list of joint_confs and the path cost
        pose = robot.tf_arm_frame(pose, armname)
        gen = solve_ik(pose[0], pose[1],armname)
        conf = next(gen) 
        joint_path = pyplan.plan_joint_motion(self.id, self.arm_joints[armname], conf, obstacles=obstacles, self_collisions=False)

        return joint_path


    def forward_kinematics(self, conf, armname):
        pose_in_armbase_frame =  solve_fk(conf, armname)
        pose_in_world_frame = self.tf_world_frame(pose_in_armbase_frame, armname)
        return pose_in_world_frame


    def get_traj_obst_cost(self, trajectory, armname, obstacles=[]):
        cost = 0
        for obs in obstacles:
            cloud = self.get_object_pointcloud(obs)
            for conf in trajectory:
                ee_position = list(self.forward_kinematics(conf, armname)[0])
                for point in cloud:
                    cost += np.linalg.norm(np.array(ee_position) - np.array(point))
        return cost

    def get_traj_goal_cost(self, trajectory, goal_pose, armname):
        conf = trajectory[-1]
        ee_position = self.forward_kinematics(conf, armname)[0]
        cost = np.linalg.norm(np.array(ee_position) - np.array(goal_pose[0]))
        return cost 

    def get_object_pointcloud(self, obid):
        cloud = []
        aabb = p.getAABB(obid)
        minx,miny,minz = aabb[0]
        maxx,maxy,maxz = aabb[1]
        num_x = max(int((maxx - minx)*50), 10)
        xs = np.linspace(minx, maxx, num=num_x)
        for px in xs:
            cloud.append([px, miny, minz])
            cloud.append([px, maxy, maxz]) 
            cloud.append([px, miny, maxz])
            cloud.append([px, maxy, minz])
        num_y = max(int((maxy - miny)*50), 10)
        ys = np.linspace(miny, maxy, num=num_y)
        for py in ys:
            cloud.append([minx, py, minz])
            cloud.append([maxx, py, maxz])
            cloud.append([minx, py, maxz])
            cloud.append([maxx, py, minz])
        num_z = max(int((maxz - minz)*50), 10)
        zs = np.linspace(minz, maxz, num=num_z)
        for pz in zs:
            cloud.append([minx, miny, pz])
            cloud.append([maxx, maxy, pz])
            cloud.append([minx, maxy, pz])
            cloud.append([maxx, miny, pz])
        
        return cloud

    def score_kin(self, conf, pose, grasp):
        ee_position = self.forward_kinematics(conf, armname)[0]
        cost = np.linalg.norm(np.array(ee_position) - np.array(grasp[0]))
        return cost

    def score_grasp(self, grasp):
        return 1

    def score_stable(self, placement):
        return 1


    def hold(self, object_id, armname):
        ee_id = self.arms_ee[armname]
        # self.grasped[armname] = p.createConstraint(self.id, ee_id,object_id,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[0,0,0])
        self.grasped[armname]=pyplan.add_fixed_constraint(object_id, self.id, ee_id)

    def release_hold(self, armname):
        p.removeConstraint(self.grasped[armname])

    def release_specific_hold(self, object_id, armname):
        ee_id = self.arms_ee[armname]
        pyplan.remove_fixed_constraint(object_id, self.id, ee_id)


    def pick_up(self, object_id, armname):
        grasp_position, grasp_orientation = self.get_top_grasp(object_id)
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)
        time.sleep(3) 
        self.hold(object_id, armname) 
        grasp_position[2] += 0.2
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)

    def pick_up_tray(self, object_id, armname):
        grasp_position, grasp_orientation = self.get_top_grasp(object_id)
        grasp_position = list(grasp_position); grasp_position[2]+=0.1
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)
        time.sleep(3) 
        self.hold(object_id, armname) 
        grasp_position[2] += 0.2
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)


    def place_at(self, position, object_id, armname):
        position = list(position)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        intermediate_position = position; intermediate_position[2]+=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        intermediate_position[2]-=0.3
        aabb = p.getAABB(object_id)
        height = aabb[1][2] - aabb[0][2]
        intermediate_position[2] += height/2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        self.release_hold(armname)
        time.sleep(2)
        intermediate_position[2]+=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        # self.tuck_arm(armname)

    def get_camera_images(self): 
        eyeid = pyplan.link_from_name(self.id, 'torso_camera_link')
        fov, aspect, nearplane, farplane = 80, 1.0, 0.01, 100
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
        com_p, com_o, _, _, _, _ = p.getLinkState(self.id, eyeid) 
        rot_matrix = p.getMatrixFromQuaternion(com_o)#p.getQuaternionFromEuler((1.5707,0.866,0)))
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (1, 0, 0) # z-axis
        init_up_vector = (0, 1, 0) # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(com_p+ 0.25 * camera_vector, com_p + 100 * camera_vector, up_vector)
        imgs = p.getCameraImage(640, 640, view_matrix, projection_matrix)
        return imgs 

         