'''
Sawyer机械臂的笛卡尔空间移动需要四元数定向

该方法实现旋转矩阵表达转换为四元数表达
'''

from math import sqrt

def rot2q(rot) -> list:
    r11 = rot[0, 0]
    r12 = rot[0, 1]
    r13 = rot[0, 2]
    r21 = rot[1, 0]
    r22 = rot[1, 1]
    r23 = rot[1, 2]
    r31 = rot[2, 0]
    r32 = rot[2, 1]
    r33 = rot[2, 2]

    w = (1 / 2) * sqrt(1 + r11 + r22 + r33)
    x = (r32 - r23) / (4 * w)
    y = (r13 - r31) / (4 * w)
    z = (r21 - r12) / (4 * w)

    q = [x, y, z, w]

    return q
