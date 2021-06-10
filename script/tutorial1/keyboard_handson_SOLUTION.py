from ambf_client import Client
import time
from std_msgs.msg import Float64, Float64MultiArray
from ambf_msgs.msg import RigidBodyCmd
import rospy

current_ecm_position = RigidBodyCmd()


def keyboardCallback(msg):
    current_ecm_position.joint_cmds = [msg.data[0], msg.data[1],msg.data[2],msg.data[3]]
    print("pos:",current_ecm_position.joint_cmds)

def ecm():
    rospy.init_node('ecm',anonymous=True)

    sub_key = rospy.Subscriber("/cmd_pose", Float64MultiArray, keyboardCallback)
    joint_pub = rospy.Publisher("/ambf/env/ecm/baselink/Command", RigidBodyCmd, queue_size=10)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        current_ecm_position.joint_cmds_types = [1,1,1,1]
        current_ecm_position.publish_joint_positions = True
        joint_pub.publish(current_ecm_position)
        rate.sleep()
        

if __name__ == '__main__':
    ecm()

  

 



