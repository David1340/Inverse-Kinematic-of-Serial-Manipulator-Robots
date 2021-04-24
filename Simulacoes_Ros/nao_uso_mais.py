from math import cos, sin, sqrt, pi, atan2
import numpy as np
import random 

#Calcula os angulos de Euler a partir de uma matriz de Rotacao
def orientacao(A):
    #Z-Y-Z Euler Angles
    if(A[2,2] < -1): A[2,2] = -1  
    if(A[0,2] != 0 or A[1,2] !=0):
        theta = atan2(sqrt(1 - (A[2,2]**2)),A[2,2])
        phi = atan2(A[1,2],A[0,2])
        psi = atan2(A[2,1],-A[2,0])
    else:
        if(A[2,2] == 1):
            theta = 0
            phi = 0
            psi = atan2(A[1,0],A[0,0])
        else:
            theta = pi
            phi = 0
            psi = - atan2(-A[0,1],-A[0,0]) 
    result = np.array([[phi,theta,psi]]).T
    return result

def matriz_B(v): #Matriz B(alpha) da equação 4.108 do livro Spong-RobotmodelingandControl
    L1 = np.array([round(cos(v[2])*sin(v[1]),5),round(-sin(v[2]),5),0])
    L2 = np.array([round(sin(v[2])*sin(v[1]),5),round(cos(v[2]),5),0])
    L3 = np.array([round(cos(v[1]),5),0,1])
    A = np.array([L1,L2,L3])
    return A

def matriz_B2(v): #para os angulos RPY
    L1 = np.array([0,sin(v[0]),-cos(v[0])*cos(v[1])])
    L2 = np.array([0,cos(v[0]),sin(v[0])*cos(v[1])])
    L3 = np.array([1,0,-sin(v[1])])
    A = np.array([L1,L2,L3])
    return A

#Calcula os angulos de RPY a partir de uma matriz de Rotacao
def orientacao2(A):
    #calcular os ângulos de orientação na conversão Z -> Y -> X
    R = atan2(A[1,0],A[0,0]) #Roll
    P = atan2(-A[2,0],sqrt((A[2,1]**2)+(A[2,2]**2))) #Pitch
    Y = atan2(A[2,1],A[2,2]) #Yaw
    result = np.array([[R,P,Y]]).T
    return result

def Ja(J,B):#Usando como entrada a matriz B(alpha) e a Jacobiana analitica eh calculada a matriz geometrica
    L1 = np.array([1,0,0,0,0,0])
    L2 = np.array([0,1,0,0,0,0])
    L3 = np.array([0,0,1,0,0,0])
    #C = np.linalg.inv(B.T@B + 0.01*np.eye(3))@B.T
    C = np.linalg.inv(B)#np.linalg.inv(B.T@B)@B.T
    L4 = np.array([0,0,0,C[0,0],C[0,1],C[0,2]])
    L5 = np.array([0,0,0,C[1,0],C[1,1],C[1,2]])
    L6 = np.array([0,0,0,C[2,0],C[2,1],C[2,2]])
    A = np.array([L1,L2,L3,L4,L5,L6])
    return A@J