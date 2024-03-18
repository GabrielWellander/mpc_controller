import numpy as np
from equation_of_motions import *
import opengen as og
import casadi.casadi as cs


class sytem_eq:
    def __init__(self):
        #initialize the constants to be able to put them in the code

        self.const =constants()
        self.inv_M = np.linalg.inv(self.const.M_matrix)
        self.Tau = np.array([[0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0]])
        
        # Initializing N and M, respectively the control signal horizon and the control horizon

        self.N = 6
        self.M = 10
        self.sampling_time = 0.1



        #
    def equations_ct(self,curr_vel,curr_pos):
        #continuous time equations
        # start the quations

        dx_1 = curr_pos
        dx_2 = -self.inv_M * (C_matrix(curr_vel,self.const) + D_matrix(curr_vel, self.const)) * curr_vel + self.inv_M * self.Tau - self.inv_M * G_matrix(self.const,curr_pos)
        return [dx_1, dx_2]
    
    def equations_dt(self,curr_vel,curr_pos):

        # discrete time equations

        dx = equations_ct(self, curr_vel, curr_pos)

        return [curr_pos[i]+ self.sampling_time * curr_vel[i] for i in range(self.N)]




class controller:
    def __init__(self):
        
        # initialize the nodes

        

        # initialize casadi and opengen

        

    

    

