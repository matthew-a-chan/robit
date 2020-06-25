import numpy as np
import math

import matplotlib
from matplotlib import collections as mc

# + x is left
# + y is forward
# + z is up

hip_x = 30
hip_y = 45
hip_z = 0

leg_u_x = 0
leg_u_y = 0
leg_u_z = -116

leg_l_x = 0
leg_l_y = 0
leg_l_z = -116


def solve(point, leg):

    if leg == 'Front_Left':
        hip = np.array([hip_x, hip_y, hip_z])
        upper_leg = np.array([leg_u_x, leg_u_y, leg_u_z])
        lower_leg = np.array([leg_l_x, leg_l_y, leg_l_z])
    if leg == 'Front_Right':
        hip = np.array([-hip_x, hip_y, hip_z])
        upper_leg = np.array([-leg_u_x, leg_u_y, leg_u_z])
        lower_leg = np.array([-leg_l_x, leg_l_y, leg_l_z])
    if leg == 'Back_Left':
        hip = np.array([hip_x, -hip_y, hip_z])
        upper_leg = np.array([leg_u_x, -leg_u_y, leg_u_z])
        lower_leg = np.array([leg_l_x, -leg_l_y, leg_l_z])
    if leg == 'Back_Right':
        hip = np.array([-hip_x, -hip_y, hip_z])
        upper_leg = np.array([-leg_u_x, -leg_u_y, leg_u_z])
        lower_leg = np.array([-leg_l_x, -leg_l_y, leg_l_z])

    x, y, z = point[0], point[1], point[2]

    length_hip = np.linalg.norm(hip)
    length_upper_leg = np.linalg.norm(upper_leg)
    length_lower_leg = np.linalg.norm(lower_leg)

    # Compute the angle of the hip

    # pre-adjusted for the hip offset
    absolute_angle = math.atan2(z, x)
    length_xz = (x ** 2 + z ** 2) ** (1 / 2)
    length_xz_hip = (hip[0] ** 2 + hip[2] ** 2) ** (1 / 2)
    length_xz_leg = (length_xz ** 2 - length_xz_hip ** 2) ** (1 / 2)

    second_angle = math.atan2(length_xz_leg, length_xz_hip)

    hip_abductor = second_angle + absolute_angle

    # Compute hip/knee joints
    hip_position = math.cos(hip_abductor) * length_xz_hip, hip[1], math.sin(hip_abductor) * length_xz_hip

    height = ((x - hip_position[0]) ** 2 + (y - hip_position[1]) ** 2 + (z - hip_position[2]) ** 2) ** (1 / 2)

    cos_arg = np.clip((height ** 2 - length_upper_leg ** 2 - length_lower_leg ** 2) / (2 * length_upper_leg * length_lower_leg), -.9999999999, .9999999999)
    knee_angle = math.acos(cos_arg)

    hip_angle = math.atan2(hip_position[1] - y, height) + math.atan2(length_lower_leg * math.sin(knee_angle), length_upper_leg + length_lower_leg * math.cos(knee_angle))

    if 'Right' in leg:
        knee_angle = -knee_angle
        hip_angle = -hip_angle
        hip_abductor = -hip_abductor

    if 'Back' in leg:
        hip_abductor = -hip_abductor

    return hip_abductor, hip_angle, knee_angle
