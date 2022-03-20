'''
ITESM
MARCOS EDUARDO GARCIA ORTZ A01276213
IRS 2019
FJ2022
'''

import numpy as np
import math
import matplotlib.pyplot as plt
from math import *
from numpy import *
#from math import sqrt
#Begin Functions


def my_rotx(theta):

    mat = np.array([[1,0,0],[0, math.cos(theta), -math.sin(theta)],[0,math.sin(theta)  ,math.cos(theta)]])
    mat = np.round(mat,decimals = 5)
    return mat

def my_roty(theta):

    mat = np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
    mat = np.round(mat,decimals = 5)
    return mat

def my_rotz(theta):

    mat = np.array([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])
    mat = np.round(mat,decimals = 5)
    return mat

def FK(L,q):
    mat = np.array([[((L[0][1])*math.cos(q[0][0]+q[0][1]))+(L[0][0]*math.cos(q[0][0])),
                     (L[0][1]*math.sin(q[0][0]+q[0][1]))+(L[0][0]*math.sin(q[0][0]))]])
    mat = np.round(mat,decimals = 5)
    print("X: ",mat[0][0])
    print("Y: ",mat[0][1])

def plot(L,q):
    plt.plot([0, math.cos(q[0][0])*L[0][0]],[0, math.sin(q[0][0])*L[0][0]])
    plt.plot([math.cos(q[0][0])*L[0][0],  L[0][1]*math.cos(q[0][0]+q[0][1])+L[0][0]*math.cos(q[0][0]) ],
             [math.sin(q[0][0])*L[0][0], L[0][1]*math.sin(q[0][0]+q[0][1])+L[0][0]*math.sin(q[0][0]) ])

    plt.show()


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

        #mat = np.round(mat,decimals = 5)

        mat = dot(mat, DH_mat)

        for k in range(4):
            for j in range(4):
                if mat[k][j] == -0:
                    mat[k][j] = 0

        print("DH matrix", i)
        print(mat)

        Array.append(mat)
    return Array


def rot2Euler(R):

    yaw = math.atan2(R[1][0],R[0][0])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    roll = math.atan2((-R[1][2]*cos_yaw + R[0][2]*sin_yaw),(R[1][1]*cos_yaw-R[0][1]*sin_yaw))

    pitch = math.atan2(-R[2][0],R[0][0]*cos_yaw + R[1][0]*sin_yaw)

    RPY = np.array([roll, pitch, yaw])

    return RPY

def euler2rot(A):
    matrix_roll = my_rotx(A[0])
    matrix_pitch = my_roty(A[1])
    matrix_yaw = my_rotz(A[2])

    res_mat = reduce(np.dot,[matrix_yaw,matrix_pitch,matrix_roll])

    return res_mat

def signo(valor):
    if valor > 0:
        return 1
    elif valor < 0:
        return -1
    elif valor == 0:
        return 1

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
    #quat_matrix = np.round(quat_matrix,decimals = 5)
    return quat_matrix

def quat2rot(Q):
    lambda0 = Q[0]
    lambda1 = Q[1]
    lambda2 = Q[2]
    lambda3 = Q[3]

    Rot_mat = np.array([[    (2*(lambda0**2+lambda1**2))-1     ,  2*(lambda1*lambda2-lambda0*lambda3) ,  2*(lambda1*lambda3+lambda0*lambda2)   ],
                        [ 2*(lambda1*lambda2+lambda0*lambda3)  ,      2*(lambda0**2+lambda2**2)-1     ,  2*(lambda2*lambda3-lambda0*lambda1)   ],
                        [ 2*(lambda1*lambda3-lambda0*lambda2)  ,  2*(lambda2*lambda3+lambda0*lambda1) ,      2*(lambda0**2+lambda3**2)-1       ]])
    Rot_mat = Rot_mat.astype(float)
    #Rot_mat = np.round(Rot_mat,decimals = 4)
    return Rot_mat

# End Functions


#sigma = np.array([0,1,1,0,0,0])
#a = np.array([0,0,0,0,0,0])
#alpha = np.array([0,-pi/2,0,-pi/2,pi/2,-pi/2])
#d = np.array([2,2,2,2,0,2])
#theta = np.array([0,0,-pi/2,0,0,0])
#q = np.array([0,0,0,0,0,0])

#Matriz = GENFK(sigma,a,alpha,d,theta,q)
#print("Aqui el cuaternion crack")
#print(rot2quat(Matriz[1]))




#Test = GENFK(sigma, a, alpha, d, theta, q)

#matri = np.array([[0,-1,0],[1,0,0],[0,0,-1]])

#result = rot2Euler(matri)
#print("Rotacion a Euler")
#print(result)

#resul2 = euler2rot(result)
#print("\nEuler a Rotacion")
#print(resul2)

#print(Test[2])
#print("\n")
#Resul =
#print(Resul)
#print(Test[2][0][1])
quat = np.array([0.5,-0.5,-0.5,-0.5])

res = np.array([[0,1,0],[0,0,1],[1,0,0]])
res = rot2quat(res)
print("Rotacion a Quaternion\n")

print(res)
print("\n")
res2 = quat2rot(quat)
print("Quaternion a rotacion\n")
print(res2)
#print(Array)

#L = np.array([[4,3]])
#Q = np.array([[pi/4,-pi/2]])

#FK(L,Q)
#plot(L,Q)
