#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
#from geometry_msgs import Pose,Quaternion

# Design of the code
# goal: - create a working mpc for the BlueROV2 normal configuration
#       - create test
#       - make the code work for the BlueROV2 heavy configuration
#       - implement sonar avoidance
#       - implement leader recognition and tracking
#
# I need to define the body and limitations of the ROV
# I need to implement limitations on the motors
# Implement matrix calculations of the basic equation of motion

class MPCcontroller:

    def __init__(self):
        rospy.init_node('position_check',anonymous=True)
        self.subscriber= rospy.Subscriber("/BlueRov2/state",Odometry,self.show)
        
    
    def show(self,data):
        pub=publi()
        rate = rospy.Rate(0.3) # rate of 1/3 hz ( every 3 seconds)
        while not rospy.is_shutdown():
            
            print(data.pose.pose.position.x)
            rate.sleep()
            pub.thruster_pub()


class publi:

    def __init__(self):

        self.publisher= rospy.Publisher("/BlueRov2/thruster_command", JointState, queue_size=1000)
        self.x=0

    def thruster_pub(self):

        # creation of JointState message

        joint_state_msg = JointState()
        joint_state_msg.header.stamp= rospy.Time.now()
        joint_state_msg.name = ["thr1","thr2","thr3","thr4","thr5","thr6"]

        if (self.x % 2) == 0: #if publisher code calls correctly it should be able to wave back and forth
            effort = 0.2
        else:
            effort = -0.1
        self.x += 1
        efforts= np.array([effort, effort, -effort, -effort, 0, 0 ])
        #publishing of the message
        joint_state_msg.effort= list(efforts)

        self.publisher.publish(joint_state_msg)



if __name__ == '__main__':
    
    MPCcontroller()
    rospy.spin()