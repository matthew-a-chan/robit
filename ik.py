import numpy as np
import math

import matplotlib
from matplotlib import collections  as mc


def solve (point, leg):
    

    if True:#leg == 'Front_Left':

        x, y, z = point[0], point[1], point[2]

        hip = np.array ([20, 0, 0])
        length_hip = np.linalg.norm (hip)
        
        upper_leg = np.array ([0, 0, -115])
        length_upper_leg = np.linalg.norm (upper_leg)

        lower_leg = np.array ([0, 0, -115])
        length_lower_leg = np.linalg.norm (lower_leg)

        total_length = np.linalg.norm (point)

        print (length_hip, length_lower_leg, length_upper_leg)




        # Compute the angle of the hip

        # pre-adjusted for the hip offset
        absolute_angle = math.atan2 (z, x)
        length_xz = (x ** 2 + z ** 2) ** (1/2)
        length_xz_leg = (length_xz ** 2 - length_hip ** 2) ** (1/2)
        print (length_xz, length_hip)

        second_angle = math.atan2 (length_xz_leg, length_hip)

        hip_deflection = second_angle + absolute_angle




        # Compute hip/knee joints
        hip_position = math.cos (hip_deflection) * length_hip, math.sin (hip_deflection) * length_hip
        
        height = ((x - math.cos (hip_deflection) * length_hip) ** 2 + (z - math.sin (hip_deflection) * length_hip) ** 2) ** (1/2)

        print ('height:', height)

        print ((y ** 2 + height ** 2 - length_upper_leg ** 2 - length_lower_leg ** 2) / (2 * length_upper_leg * length_lower_leg))

        cos_arg = np.clip ((y ** 2 + height ** 2 - length_upper_leg ** 2 - length_lower_leg ** 2) / (2 * length_upper_leg * length_lower_leg), -.9999999999, 1)

        knee_angle = math.acos (cos_arg)

        print (math.atan2 (y, height))
        print (math.atan2 (length_lower_leg * math.sin (knee_angle), length_upper_leg + length_lower_leg * math.cos (knee_angle)))
        hip_angle = math.atan2 (y, height) - math.atan2 (length_lower_leg * math.sin (knee_angle), length_upper_leg + length_lower_leg * math.cos (knee_angle))




        return hip_deflection, hip_angle, knee_angle

#        cos (theta) * height + sin (theta) * length_hip = y
#        sin (theta) * height + cos (theta) * length_hip = z


        
