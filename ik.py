import numpy as np
import math

import matplotlib
from matplotlib import collections  as mc


def solve (point, leg):
    

    if leg == 'Front_Left':
        point[1] = - point[1]
        hip = np.array ([20, 0, 0])
        upper_leg = np.array ([0, 0, -115])
        lower_leg = np.array ([0, 0, -115])
    if leg == 'Front_Right':
        point[0] = - point[0]
        hip = np.array ([20, 0, 0])
        upper_leg = np.array ([0, 0, -115])
        lower_leg = np.array ([0, 0, -115])
    if leg == 'Back_Left':
        point[1] = - point[1]
        hip = np.array ([20, 0, 0])
        upper_leg = np.array ([0, 0, -115])
        lower_leg = np.array ([0, 0, -115])
    if leg == 'Back_Right':
        point[0] = - point[0]
        hip = np.array ([20, 0, 0])
        upper_leg = np.array ([0, 0, -115])
        lower_leg = np.array ([0, 0, -115])


    x, y, z = point[0], point[1], point[2]

    length_hip = np.linalg.norm (hip)
    length_upper_leg = np.linalg.norm (upper_leg)
    length_lower_leg = np.linalg.norm (lower_leg)


    total_length = np.linalg.norm (point)

    # Compute the angle of the hip

    # pre-adjusted for the hip offset
    absolute_angle = math.atan2 (z, x)
    length_xz = (x ** 2 + z ** 2) ** (1/2)
    length_xz_leg = (length_xz ** 2 - length_hip ** 2) ** (1/2)

    second_angle = math.atan2 (length_xz_leg, length_hip)

    if 'Right' in leg:
        hip_deflection = - (second_angle + absolute_angle)
    else:
        hip_deflection = second_angle + absolute_angle
    
    if 'Back' in leg:
        hip_deflection = - hip_deflection



    # Compute hip/knee joints
    hip_position = math.cos (hip_deflection) * length_hip, math.sin (hip_deflection) * length_hip
    
    height = ((x - math.cos (hip_deflection) * length_hip) ** 2 + (z - math.sin (hip_deflection) * length_hip) ** 2) ** (1/2)


    cos_arg = np.clip ((y ** 2 + height ** 2 - length_upper_leg ** 2 - length_lower_leg ** 2) / (2 * length_upper_leg * length_lower_leg), -.9999999999, .9999999999)

    if 'Right' in leg:
        knee_angle = math.acos (cos_arg)
    else:
        knee_angle = - math.acos (cos_arg)

    hip_angle = math.atan2 (y, height) - math.atan2 (length_lower_leg * math.sin (knee_angle), length_upper_leg + length_lower_leg * math.cos (knee_angle))




    return hip_deflection, hip_angle, -knee_angle
        
