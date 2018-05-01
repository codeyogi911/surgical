'''
Created on 27 Apr 2018

@author: shashwatjain
'''
from math import atan2, cos, sin, acos, pi
import numpy as np
d_base = 0.75
a_1 = 0.35
a_2 = 1.25
a_3 = -0.054
d_3 = 1.5
d_6 = 0.303
# theta1, theta2, theta3, alpha2, beta2, alpha3 = arm_IK(2, 1, 2)
def limit_angle(theta, min, max):
    '''
    Check that angles are within rotation bounds
    :param theta: calculated angle
    :param min: lower bound threshold in degrees
    :param max: upper bound threshold in degrees
    :return: bounded angle
    '''
    if theta > pi * max / 180:
        return pi * max / 180
    elif theta < pi * min / 180:
        return pi * min / 180
    else:
        return theta
def arm_IK(px, py, pz):
    '''
    Calculates the inverse kinematics of the arm section
    :param px: position along the x axis
    :param py: position along the y axis
    :param pz: position along the z axis
    :return: first three arm angles, theta 1, 2 and 3
    '''

    ''' theta 1 '''
    #   ^ y
    #   |
    #   |
    #   |
    #   o ----------> x
    #
    # Calculate joint angles using Geometric IK method
    theta1 = limit_angle(atan2(py, px), 0, 180)

    # remove base offsets from the wrist coords for theta 2 and 3
    a_1_x = a_1 * cos(theta1)  # link a_1 offset in x direction
    a_1_y = a_1 * sin(theta1)
    d_6_x = d_6 * cos(theta1)  # link d_6 offset in x direction
    d_6_y = d_6 * sin(theta1)
    # get the desired end arm x, y, z coordinates
    x_d = px - d_6_x - a_1_x
    y_d = py - a_1_y  # - d_6_y
    z_d = pz - d_base + d_6 / 3

    # x y plane arm projections
    r_xy = np.sqrt(x_d ** 2 + y_d ** 2)
    # x, y, z 3D plane arm length
    r_xyz = np.sqrt(x_d ** 2 + y_d ** 2 + z_d ** 2)

    # calculate link 3 shoulder angle and distance to wrist center
    #         |----- d_3 ------| _
    #         O *               a_3
    #       *   * * * * * * * *  -
    #     *
    #   *
    link_3 = np.sqrt(a_3 ** 2 + d_3 ** 2)
    link_3_theta = atan2(a_3, d_3)

    ''' theta 2 '''
    #   ^ z
    #   |
    #   | a-b   o
    #   |---- *
    #   |   *
    #   | *
    #   o ----------> x
    #
    # link 1 to wrist center angle from z axis / vertical (pi/2)
    beta2 = atan2(r_xy, z_d)

    # law of cosine rule
    D_theta2 = (a_2 ** 2 + r_xyz ** 2 - link_3 ** 2) / (2 * a_2 * r_xyz)
    psi2 = atan2(np.sqrt(np.abs(1 - D_theta2 ** 2)), D_theta2)
    # zero angle is along z axis
    theta2 = limit_angle(beta2 - psi2, -45, 85)

    ''' theta 3 '''
    # law of cosine rule
    D_theta3 = (a_2 ** 2 + link_3 ** 2 - r_xyz ** 2) / (2 * a_2 * link_3)
    psi3 = atan2(np.sqrt(np.abs(1 - D_theta3 ** 2)), D_theta3) + link_3_theta
    # angle perpendicular wrt link 1 but psi is from link 1
    theta3 = limit_angle((psi3 - pi / 2) * -1.0, -210, 155 - 90)

    return theta1, theta2, theta3, psi2, beta2, psi3
    
# if __name__ == '__main__':
#     theta1, theta2, theta3, alpha2, beta2, alpha3 = arm_IK(2, 1, 2)