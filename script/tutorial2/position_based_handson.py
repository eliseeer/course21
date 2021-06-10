from ecmIK import *
from ambf_client import Client
import time
from joint_space_trajectory_generator import JointSpaceTrajectory
from ambf_msgs.msg import RigidBodyCmd
import thread

def test_ik():

    js_traj = JointSpaceTrajectory(num_joints=4, num_traj_points=50)
    num_points = js_traj.get_num_traj_points()
    num_joints = 4
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_4_0 = compute_FK(test_q)
        computed_q = compute_IK(convert_mat_to_frame(T_4_0))
        computed_q = round_vec(computed_q, 4)

        errors = [0] * js_traj.get_num_joints()
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]
        print ('--------------------------------------')
        print ('**************************************')
        print ('Test Number:', i)
        print ('Joint Positions used to T_EE_B (EndEffector in Base)')
        print test_q
        print ('IK Output')
        print computed_q
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]



def test_ambf_ecm():

    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toollink', 3

    num_joints = 4
    joint_lims = np.zeros((num_joints, 2))
    joint_lims[0] = [np.deg2rad(-91.96), np.deg2rad(91.96)]
    joint_lims[1] = [np.deg2rad(-60), np.deg2rad(60)]
    joint_lims[2] = [0.0, 0.24]
    joint_lims[3] = [np.deg2rad(-175), np.deg2rad(175)]

    js_traj = JointSpaceTrajectory(num_joints=num_joints, num_traj_points=50, joint_limits=joint_lims)
    num_points = js_traj.get_num_traj_points()
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_4_0 = compute_FK(test_q)

        if target_ik is not None:


        computed_q = compute_IK(convert_mat_to_frame(T_4_0))
        # computed_q = enforce_limits(computed_q, joint_lims)
        
        if target_fk is not None:



        test_q = round_vec(test_q)
        T_4_0 = round_mat(T_4_0, 4, 4, 3)
        errors = [0]*num_joints
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]

        print ('--------------------------------------')
        print ('**************************************')
        print ('Test Number:', i)
        print ('Joint Positions used to T_EE_B (EndEffector in Base)')
        print test_q
        print ('Requested Transform for T_EE_B (EndEffector in Base)')
        print (T_4_0)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]

        time.sleep(1.0)

    time.sleep(3.0)


if __name__ == "__main__":
    test_ambf_ecm()
