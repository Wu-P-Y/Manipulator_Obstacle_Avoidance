'''
变换矩阵
|
|-- 旋转矩阵
|
|-- 位移向量

所有角度以弧度计
'''

import numpy as np
from math import sin, cos, pi

def rotx(angle):
    '''
    绕x轴旋转
    '''
    T = np.matrix([[1,0,0,0],[0,cos(angle),-1*sin(angle),0],[0,sin(angle),cos(angle),0],[0,0,0,1]])
    return T

def roty(angle):
    '''
    绕y轴旋转
    '''
    T = np.matrix([[cos(angle),0,sin(angle),0],[0,1,0,0],[-1*sin(angle),0,cos(angle),0],[0,0,0,1]])
    return T

def rotz(angle):
    '''
    绕z轴旋转
    '''
    T = np.matrix([[cos(angle),-1*sin(angle),0,0],[sin(angle),cos(angle),0,0],[0,0,1,0],[0,0,0,1]])
    return T

def displacement(vector:list):
    '''
    坐标原点的位移  vector为位移向量[x, y, z]
    '''
    T=np.matrix([[1,0,0,vector[0]],[0,1,0,vector[1]],[0,0,1,vector[2]],[0,0,0,1]])
    return T

def fixed_angle_trans(gamma, beta, alpha, vector):
    '''
    Fixed angle 推算旋转矩阵

    顺序    转轴    角度

    1       x      gamma

    2       y      beta

    3       z      alpha
    '''
    T = rotz(alpha) * roty(beta) * rotx(gamma) * displacement(vector)
    return T

def euler_angle_trans(gamma, beta, alpha, vector):
    '''
    Euler angle 推算旋转矩阵

    顺序    转轴    角度

    1       z      alpha

    2       y      beta

    3       x      gamma
    '''
    T = rotz(alpha) * roty(beta) * rotx(gamma) * displacement(vector)
    return T