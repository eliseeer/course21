from ambf_client import Client
import time
from std_msgs.msg import Float64, Float64MultiArray
from ambf_msgs.msg import RigidBodyCmd
import rospy


# In this hands-on your are called at defining a publisher and a subscriber for
# controlling a simulated robotic endoscope using you keyboard

# 1. in the ecm function intialize a subscriber to the topic "/cmd_pose".
#    The received message is a std_msgs.msg.Float64MultiArray containing @Elisa: TODO

# 2. in the ecm function intialize a publisher to the topic "/ambf/env/ecm/baselink/Command".
#    The published message should be a ambf_msgs.msg.RigidBodyCmd
# 2a. before publish the message
#     - set the joint_cmds @Elisa TODO
#     - set the joint_cmds_types @Elisa TODO
#     - set the publish_joint_positions parameters to True

def keyboardCallback(msg):
    # TODO handle the msg
    pass

def ecm():
    rospy.init_node('ecm',anonymous=True)

    # TODO define the subscriber
    ...
    # TODO define the publisher
    ...

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # TODO configure the new message
        current_ecm_position.joint_cmds_types = ...
        current_ecm_position.publish_joint_positions = ...

        # TODO publish the message
        ...

        rate.sleep()
        

if __name__ == '__main__':
    ecm()

  

 



