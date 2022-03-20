#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg



#from cinematic import GENFK, rot2quat
import numpy as np
from math import pi, radians
import math
from numpy import *


def signo(valor):
    if valor > 0:
        return 1
    elif valor < 0:
        return -1
    elif valor == 0:
        return 0


def rot2quat(R):

    Sx = R[0][0]
    Sy = R[1][0]
    Sz = R[2][0]

    Nx = R[0][1]
    Ny = R[1][1]
    Nz = R[2][1]

    Ax = R[0][2]
    Ay = R[1][2]
    Az = R[2][2]

    lambdaW = float(0.5*sqrt(Sx+Ny+Az+1))
    lambdax = float(0.5*signo(Nz-Ay)*sqrt(Sx-Ny-Az+1))
    lambday = float(0.5*signo(Ax-Sz)*sqrt(-Sx+Ny-Az+1))
    lambdaz = float(0.5*signo(Sy-Nx)*sqrt(-Sx-Ny+Az+1))

    quat_matrix = np.array([[lambdaW],
                            [lambdax],
                            [lambday],
                            [lambdaz]])
    return quat_matrix

def GENFK(sigma, a, alpha, d, theta, q):

    lenth = len(sigma)
    mat = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    Array = []
    for i in range(lenth):
        # Pa rotacion - theta variable
        if sigma[i] == 0 :
            Ang = theta[i] + q[i]
            dist = d[i]
        # Pa distancia - d varible
        else:
            Ang = theta[i]
            dist = d[i] + q[i]

        cosTheta = math.cos(Ang)
        sinTheta = math.sin(Ang)
        cosAlfa = math.cos(alpha[i])
        sinAlfa = math.sin(alpha[i])

        DH_mat = np.array([[cosTheta, -sinTheta*cosAlfa, sinTheta*sinAlfa , a[i]*cosTheta],
                           [sinTheta,  cosTheta*cosAlfa, -cosTheta*sinAlfa, a[i]*sinTheta],
                           [     0  ,        sinAlfa   ,        cosAlfa   ,     dist     ],
                           [     0  ,          0       ,           0      ,        1     ]])

        for k in range(4):
            for j in range(4):
                if DH_mat[k][j] == -0:
                    DH_mat[k][j] = 0
        print("DH matrix", i)
        print(np.round(DH_mat,decimals = 2))
        mat = dot(mat, DH_mat)

        Array.append(DH_mat)
    print("-------------------------------------------------")
    print("Pose del Efector Final respecto a la base")
    print(np.round(mat,decimals = 2))
    print("Quaternion del Efector Final Respecto a la base")
    print(rot2quat(mat))
    print("---------------------------------------------------")
    return Array



def plot_frame(Frames):
    rospy.init_node('Plot_Rviz', anonymous = True)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)
    Marco = []
    tam = len(Frames)
    for i in range(tam):

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "link"+str(i)
        t.child_frame_id = "link"+str(i+1)
        t.transform.translation.x = Frames[i][0][3]
        t.transform.translation.y = Frames[i][1][3]
        t.transform.translation.z = Frames[i][2][3]
        q = rot2quat(Frames[i])
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        Marco.append(t)
    while not rospy.is_shutdown():
        for i in range(len(Frames)):
            Marco[i].header.stamp = rospy.Time.now()
            br.sendTransform(Marco[i])
            rate.sleep()

if __name__ == '__main__':

    # Ejemplo 6DOF
    sigma = np.array([0,0,1,0,0,0])
    a = np.array([0,0,0,0,0,0])
    alpha = np.array([-pi/2,pi/2,0,-pi/2,-pi/2,0])
    d = np.array([2,2,2,2,0,4])
    theta = np.array([0,0,pi/2,0,-pi/2,0])
    q = np.array([0,0,0,0,0,0])

    #sigma = np.array([0,1])
    #a = np.array([2,0])
    #alpha = np.array([pi/2,0])
    #d = np.array([2,4])
    #theta = np.array([0,pi/2])
    #q = np.array([-pi/2,0])


    Frames = GENFK(sigma, a, alpha, d, theta, q)

    Result = plot_frame(Frames)
