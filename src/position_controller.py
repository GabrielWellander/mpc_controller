#!/usr/bin/env python3
import time
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from matplotlib import pyplot as plt

# Goal for the code:
#   - make a working PID controller in X or Y direction.
#
# The code streamlined: 
#   - have code to fetch the data from subscriber position
#   - have code to publish to publisher
#   - calculate the error
#   - calculate input signal1


class controller:
    def __init__(self,k_i,k_p,k_d,arrival):
        # declaration of k values
        self.k_d = k_d
        self.k_i = k_i
        self.k_p = k_p
        # setting value to goal
        self.arrival = (arrival)
        # initiating error and integral for the calculations
        self.init_err = 0
        self.int = 0
        self.err = 0

    def calc(self, curr_pos):
        # calculating error, integral and derivative
        err = self.arrival - curr_pos
        self.err= err
        self.int += err
        
        derivative = err - self.init_err

        # making memory of old error
        
        self.init_err = self.err

        #creating output (has to be negative due to unaligned world and robot (rov pos x is neg x ))

        output = -(self.k_d * derivative + self.int * self.k_i + err * self.k_p)
        #print(f"{output=} {err=} {self.int=} {derivative =}")
           
        return output



class NodeCom:
    def __init__(self):
        #self.tic = time.process_time()
        #print(self.tic)
        #initialize the node
        rospy.init_node('controller_node',anonymous=True)

        #setting the values 
        self.t = []
        self.x = []
        self.limit = []

        k_p = 1.5
        k_i = 0
        k_d = 5000

        arrival = 1
        # Initialize the controller and couple it to the code

        self.pid = controller(k_i,k_p,k_d,arrival)

        self.sub= rospy.Subscriber("/BlueRov2/state", Odometry, self.message_create)
        self.pub = rospy.Publisher("BlueRov2/thruster_command", JointState, queue_size=1000)
    
    def message_create(self,data):

        position =  data.pose.pose.position.y
        effort = self.pid.calc(position)

        self.t.append(data.header.stamp.secs + data.header.stamp.nsecs*0.000000001)
        self.x.append(position)
        self.limit.append(0.99)


        if abs(self.pid.err)< 0.001:
            #self.toc = time.process_time()
            #print(self.toc - self.tic)
            
            plt.plot(self.t,self.x,'b',self.t,self.limit,'r--')
            plt.ylabel('position along x-axis')
            plt.xlabel('time')
            plt.draw()
            plt.grid(True)
            plt.show()

            rate = rospy.Rate(0.0001)
            pass
            rate.sleep()
            
            

        jstate_msg = JointState()
        jstate_msg.header.stamp = rospy.Time.now()
        jstate_msg.name = ["thr1", "thr2", "thr3", "thr4", "thr5", "thr6"]

        effort =  np.array([effort, effort, - effort, - effort, 0, 0])
        jstate_msg.effort = list(effort)
        self.pub.publish(jstate_msg)

if __name__ == '__main__':

    controller_node = NodeCom()
    rospy.spin()
     
    






        

