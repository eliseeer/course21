from ecmIK import *
from ambf_client import Client
import time
from ambf_msgs.msg import RigidBodyCmd
import thread
import matplotlib.pyplot as plt
from std_msgs.msg import String
import PyKDL
x,y = 0,0


def coordinates_callback(msg):
    x_,y_ = msg.data.split(" ")
    x,y = int(x_), int(y_)

def test_ambf_ecm():
    c = Client('ecm_ik_test')
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('ecm/baselink')
    r = c.get_obj_handle('ecm/remotecenterlink')
    h = c.get_obj_handle('Heart')
    t = c.get_obj_handle('ecm/toollink')
    
    time.sleep(1.0)
    
    i = 0
    ascend = 1
    init_pos = h.get_pos().z
    
    # subscribe
    coord_sub = rospy.Subscriber("target_coordinates", String, coordinates_callback)

    num_joints = 4
    joint_lims = np.zeros((num_joints, 2))
    joint_lims[0] = [np.deg2rad(-91.96), np.deg2rad(91.96)]
    joint_lims[1] = [np.deg2rad(-60), np.deg2rad(60)]
    joint_lims[2] = [0.0, 0.24]
    joint_lims[3] = [np.deg2rad(-175), np.deg2rad(175)]

    upper_center = (h.get_pos().x, h.get_pos().y, init_pos + 0.05)
    lower_center = (h.get_pos().x, h.get_pos().y, init_pos - 0.05)



    while not rospy.is_shutdown():
    
        h.set_pos(h.get_pos().x,h.get_pos().y,init_pos+i)
        if i > 0.05:
            ascend = -1
        if i < 0:
            ascend = +1
        i += ascend * 0.01
        
        #thread.start_new_thread(heart_moving,(h,))

        if y > 123.3:
            target = upper_center
        else: # cuore_sta_in_basso
            target = (h.get_pos().x, h.get_pos().y, h.get_pos().z) #lower_center

        ##questo dovrebbe essere il joint che possiamo controllare direttamente##
        b.set_joint_pos('yawlink-pitchbacklink', b.get_joint_pos('yawlink-pitchbacklink')-i)


        ##primo tentativo###
        H_w = np.array([h.get_pos().x, h.get_pos().y, h.get_pos().z,1])

        P_4_w = Vector(r.get_pos().x, r.get_pos().y, r.get_pos().z)
        R_4_w = Rotation.RPY(r.get_rpy()[0],r.get_rpy()[1], r.get_rpy()[2])
        T_4_w = Frame(R_4_w,P_4_w)

        P_w_desired = np.matmul(convert_frame_to_mat(T_4_w), H_w)     #np.array([r.get_pos().x, r.get_pos().y, r.get_pos().z, 1])

        P_w_desired = np.array(P_w_desired)[0]
        print("new:",P_w_desired)
        print("old:",P_4_w)



        P_0_w = Vector(b.get_pos().x, b.get_pos().y, b.get_pos().z)
        R_0_w = Rotation.RPY(b.get_rpy()[0], b.get_rpy()[1], b.get_rpy()[2])
        T_0_w = Frame(R_0_w, P_0_w)

        P_4_w_new = Vector(P_w_desired[0],P_w_desired[1],P_w_desired[2])
        R_4_w = Rotation.RPY(r.get_rpy()[0],r.get_rpy()[1], r.get_rpy()[2])
        T_4_w = Frame(R_4_w,P_4_w_new)
        T_4_w_new = convert_frame_to_mat(T_4_w)


        T_4_0 = np.matmul(np.linalg.inv(T_4_w_new),convert_frame_to_mat(T_0_w))
        #print(T_4_0)


       # P_h_w = Vector(H_w[0], H_w[1], H_w[2])
       # P_e_w = Vector(P_w[0], P_w[1], P_w[2])
        
       # R_h_w = Rotation.RPY(h.get_rpy()[0], h.get_rpy()[1], h.get_rpy()[2])
       # T_4_0 = Frame(R_h_w, P_e_w)
        
       # print(R_H_W)

        #T_4_0 = T_h_ecm * np.expand_dims(H_p,-1)

        #computed_q = compute_IK(convert_mat_to_frame(T_4_0))
        #computed_q = compute_IK(convert_mat_to_frame(T_h_w))
        

       # b.set_joint_pos('baselink-yawlink', computed_q[0])
       # b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
       # b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
       # b.set_joint_pos('maininsertionlink-toollink', computed_q[3])



        ##secondo tentativo 
        H_w = np.array([h.get_pos().x, h.get_pos().y, h.get_pos().z])
        R_w = np.array([r.get_pos().x, r.get_pos().y, r.get_pos().z])

        z_axis = H_w - R_w 
        z_axis = z_axis/np.linalg.norm(z_axis)

        x_axis = np.array([1,0,0])
        y_axis = z_axis * x_axis
        x_axis =  y_axis * z_axis
        
        x_axis = Vector(x_axis[0],x_axis[1],x_axis[2])
        y_axis = Vector(y_axis[0],y_axis[1],y_axis[2])
        z_axis = Vector(z_axis[0],z_axis[1],z_axis[2])
       

        R = PyKDL.Rotation(x_axis,y_axis,z_axis)
        T_w = np.array([t.get_pos().x, t.get_pos().y, t.get_pos().z])
        R_w = np.array([r.get_pos().x, r.get_pos().y, r.get_pos().z])
        d_axis = R_w - T_w
        d = np.linalg.norm(d_axis)

        z_axis = np.array([z_axis[0],z_axis[1],z_axis[2]])

        pose_desired = T_w + d*z_axis
        print(pose_desired)

        #r.set_pos(pose_desired[0],pose_desired[1],pose_desired[2])
        #r.set_rot(R.GetQuaternion())

        


        
        time.sleep(1.5)


if __name__ == "__main__":
    # test_ik()
    test_ambf_ecm()
