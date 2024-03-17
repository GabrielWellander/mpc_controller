import numpy as np

# define the constants used for the matrix in the equation of motions and 
# the equation of inverse kinematics
class constants:
    def __init__(self):
        # giving values to the robot
        # m = mass nabla = displaced volume
        self.m = 13.5
        self.nabla = 0.0134

        # World values for the ROV
        # g = grav.constant rho= density of water
        self.g = 9.82
        self.rho = 1000

        # center of Buoyancy and gravity of the ROV:
        # {x,y,z}_g = {x,y}_b = 0 due to the symmetry of the rov
        # z_b = distance in z direction from the center of body to the center of Buoyancy
        self.z_b =  -0.01
        
        # Moment of inertia of the ROV:
        # I_{xy, xz, yz} = 0 due to the symmetry
        self.I_x = 0.26
        self.I_y =0.23
        self.I_z = 0.37

        # Hydrodynamics
        # X_u linear dampening
        # X_uu quadratic dampening
        # X_udot 
        self.X_u = 13.7
        self.X_uu = 141.0
        self.X_udot = 6.36

        self.Y_v = 0
        self.Y_vv = 217.0
        self.Y_vdot = 7.12

        self.Z_w = 33.0
        self.Z_ww = 190.0
        self.Z_wdot = 18.68

        self.K_p = 0
        self.K_pp = 1.19
        self.K_pdot = 0.189

        self.M_q = 0.8
        self.M_qq = 0.47
        self.M_qdot = 0.135

        self.N_r = 0
        self.N_rr = 1.5
        self.N_rdot = 0.222

        # M-matrix: the definite inertia matrix and a combination of rigid body and else

        M_rb = np.array([ [self.m , 0, 0, 0, 0, 0 ],
                         [ 0, self.m, 0, 0, 0, 0],
                         [ 0, 0, self.m, 0, 0, 0],
                         [0, 0, 0, self.I_x, 0, 0],
                         [0, 0, 0, 0, self.I_y, 0],
                         [0, 0, 0, 0, 0, self.I_z]])
        
        M_a = - np.diag([self.X_udot, self.Y_vdot, self.Z_wdot, self.K_pdot, self.M_qdot, self.N_rdot])

        self.M_matrix = M_rb + M_a

        # the linear hydrodynamic damping matrix:

        self.D_linear = - np.diag([self.X_u, self.Y_v, self.Z_w, self.K_p, self.M_q, self.N_r])


def D_matrix(curr_vel,constants):
    
    # The hydrodynamic damping matrix D_matrix
    # It is composed of the linear D_linear and the non_linear D_nlinear
    D_1 = constants.X_uu * abs(curr_vel[0])
    D_2 = constants.Y_vv * abs(curr_vel[1])
    D_3 = constants.Z_ww * abs(curr_vel[2])
    D_4 = constants.K_pp * abs(curr_vel[3])
    D_5 = constants.M_qq * abs(curr_vel[4])
    D_6 = constants.N_rr * abs(curr_vel[5])

    D_nlinear =  - np.diag([D_1, D_2, D_3, D_4, D_5, D_6])

    D_matrix = constants.D_linear + D_nlinear
    
    return D_matrix

def C_matrix(curr_vel, constants):

    # The C matrix represents the skew-symmetric Coriolis matrix
    # It is composed of the rigid body and the hydrodynamic effects respectively C_rb and C_a


    C_rb = np.array([[0, 0, 0, 0, constants.m * curr_vel[2], - constants.m * curr_vel[1]],
                     [0, 0, 0, - constants.m * curr_vel[2], 0, constants.m * curr_vel[0]],
                     [0, 0, 0, constants.m * curr_vel[1], - constants.m * curr_vel[0], 0],
                     [0, constants.m * curr_vel[2], - constants.m * curr_vel[1], 0, - constants.I_z * curr_vel[5], - constants.I_y * curr_vel[4]],
                     [- constants.m * curr_vel[2], 0, constants.m * curr_vel[0], constants.I_z * curr_vel[5], 0, constants.I_x * curr_vel[3]],
                     [constants.m * curr_vel[1], - constants.m * curr_vel[0], 0, constants.I_y * curr_vel[4], - constants.I_x * curr_vel[3], 0]])
    
    C_a = np.array([[0, 0, 0, 0, - constants.Z_wdot * curr_vel[2], constants.Y_vdot * curr_vel[1]],
                    [0, 0, 0, constants.Z_wdot * curr_vel[2], 0, - constants.X_udot * curr_vel[0]],
                    [0, 0, 0, - constants.Y_vdot * curr_vel[1], constants.X_udot * curr_vel[0], 0],
                    [0, - constants.Z_wdot * curr_vel[2], constants.Y_vdot * curr_vel[1], 0, - constants.N_rdot * curr_vel[5], constants.M_qdot * curr_vel[4]],
                    [constants.Z_wdot * curr_vel[2], 0, - constants.X_udot * curr_vel[0], constants.N_rdot * curr_vel[5], 0, - constants.K_pdot * curr_vel[3]],
                    [- constants.Y_vdot * curr_vel[1], constants.X_udot * curr_vel[0], 0, - constants.M_qdot * curr_vel[4], constants.K_pdot * curr_vel[3], 0]])
    
    C_mat = C_rb + C_a

    return C_mat

def J_matrix(curr_pos):

    #The matrix for the inverse kinematic to be able to go from velocity b frame to w frame

    # Due to the size of the matrix we have to break up in 2 submatrixes and then for each element
    # submatrix 1 (size 3x3)

    J1_11 = np.cos(curr_pos[5]) * np.cos(curr_pos[4])
    J1_12 = - np.sin(curr_pos[5]) * np.cos(curr_pos[3]) + np.cos(curr_pos[5]) * np.sin(curr_pos[4]) * np.sin(curr_pos[3])
    J1_13 = np.sin(curr_pos[5]) * np.sin(curr_pos[3]) + np.cos(curr_pos[5]) * np.cos(curr_pos[3]) * np.cos(curr_pos[4])
    
    J1_21 = np.sin(curr_pos[5]) * np.cos(curr_pos[4])
    J1_22 = np.cos(curr_pos[5]) * np.cos(curr_pos[3]) + np.sin(curr_pos[3]) * np.sin(curr_pos[4]) * np.sin(curr_pos[5])
    J1_23 = - np.cos(curr_pos[5]) * np.sin(curr_pos[3]) + np.sin(curr_pos[4]) * np.sin(curr_pos[5]) * np.cos(curr_pos[3])

    J1_31 = - np.sin(curr_pos[4])
    J1_32 = np.cos(curr_pos[4]) * np.sin(curr_pos[3])
    J1_33 = np.cos(curr_pos[4]) * np.cos(curr_pos[3])

    # submatrix 2 (size 3x3)

    J2_11 = 1
    J2_12 = np.sin(curr_pos[3]) * np.tan(curr_pos[4])
    J2_13 = np.cos(curr_pos[3]) * np.tan(curr_pos[4])

    J2_21 = 0
    J2_22 = np.cos(curr_pos[3])
    J2_23 = - np.sin(curr_pos[3])
    
    J2_31 = 0
    J2_32 = (np.sin(curr_pos[3]))/(np.cos(curr_pos[4]))
    J2_33 = (np.cos(curr_pos[3]))/(np.cos(curr_pos[4]))


    # Assembling the J_matrix
    J_mat = np.array([[J1_11, J1_12, J1_13, 0, 0, 0],
                      [J1_21, J1_22, J2_23, 0, 0, 0],
                      [J1_31, J1_32, J1_33, 0, 0, 0],
                      [0, 0, 0, J2_11, J2_12, J2_13],
                      [0, 0, 0, J2_21, J2_22, J2_23],
                      [0, 0, 0, J2_31, J2_32, J2_33]])
    
    return J_mat

def G_matrix(const,curr_pos):
    B = const.m * const.g
    # the fact that z_b is the only non null value makes the formulation of g easy
    # the g matrix represents the restorative force on the rov
    # W and B are here equal

    g_mat = np.array([[0],
                      [0],
                      [0],
                      [- curr_pos[2] * B * np.cos(curr_pos[4]) * np.sin(curr_pos[3])],
                      [- curr_pos[2] * B * np.sin(curr_pos[4])],
                      [0]])
    return g_mat