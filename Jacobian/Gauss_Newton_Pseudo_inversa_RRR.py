#Este código é baseado na figura 1.8 do livro do Spong
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin, sqrt, pi, atan2 
import numpy as np
import math
import random 

#transforma uma matriz A m x n em uma matriz  3 x 3
def M_3x3(A):
    L1 = np.array([A[0,0],A[0,1],A[0,2]])
    L2 = np.array([A[1,0],A[1,1],A[1,2]])
    L3 = np.array([A[2,0],A[2,1],A[2,2]])
    H = np.array([L1,L2,L3])
    return H
                    
#transforma um vetor v n x 1 em um vetor 3 x 1
def v_3d(v):
    return np.array([v[0],v[1],v[2]])

#Retorna a Matriz de transformacao Homogeneadados  usando como entrada os parametros de DH
def matrix_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2      
    return sqrt(d)

#Calcula os angulos de Euler a partir de uma matriz de Rotacao
def orientacao(A):
    #Z-Y-Z Euler Angles
    if(A[0,2] != 0 or A[1,2] !=0):
        theta = atan2(sqrt(1 - A[2,2]**2),A[2,2])
        phi = atan2(A[1,2],A[0,2])
        psi = atan2(A[2,1],-A[2,0])
    else:
        if(A[2,2] == 1):
            theta = 0
            phi = 0
            psi = atan2(A[0,0],A[1,0])
        else:
            theta = pi
            phi = 0
            psi = - atan2(-A[0,0],-A[0,1]) 
    result = np.array([theta,phi,psi])
    return result

def matriz_B(v): #Matriz B(alpha) da equação 4.108 do livro Spong-RobotmodelingandControl
    L1 = np.array([round(cos(v[2])*sin(v[0]),4),round(-sin(v[2]),4),0])
    L2 = np.array([round(sin(v[2])*sin(v[0]),4),round(cos(v[2]),4),0])
    L3 = np.array([round(cos(v[0]),4),0,1])
    A = np.array([L1,L2,L3])
    return A

def Ja(J,B):#Usando como entrada a matriz B(alpha) e a Jacobiana analitica eh calculada a matriz geometrica
    L1 = np.array([1,0,0,0,0,0])
    L2 = np.array([0,1,0,0,0,0])
    L3 = np.array([0,0,1,0,0,0])
    C = np.linalg.inv(B)
    L4 = np.array([0,0,0,C[0,0],C[0,1],C[0,2]])
    L5 = np.array([0,0,0,C[1,0],C[1,1],C[1,2]])
    L6 = np.array([0,0,0,C[2,0],C[2,1],C[2,2]])
    A = np.array([L1,L2,L3,L4,L5,L6])
    return A@J

#Função que plota o manipulador, quando recebe os pontos de interesse
def plot(a,b,c,d,t):
    #Plotando Elos
    plt.plot([a[0,0],b[0,0]],[a[1,0],b[1,0]],[a[2,0],b[2,0]])
    plt.plot([b[0,0],c[0,0]],[b[1,0],c[1,0]],[b[2,0],c[2,0]])
    plt.plot([c[0,0],d[0,0]],[c[1,0],d[1,0]],[c[2,0],d[2,0]])

    #Plotando Juntas
    ax.scatter(a[0,0],a[1,0],a[2,0])
    ax.scatter(b[0,0],b[1,0],b[2,0])
    ax.scatter(c[0,0],c[1,0],c[2,0])

    #Plotando efetuador
    ax.scatter(d[0,0],d[1,0],d[2,0])

    ax.scatter(t[0],t[1],t[2])
    #Legendas
    plt.legend(['Elo 1','Elo 2','Elo 3','Junta 1 (Revolucao)','Junta 2 (Revolucao)'\
                ,'Junta 3 (Revolucao)','Efetuador','objetivo'])

    ax.set_xlabel('Eixo x')
    ax.set_ylabel('Eixo y')
    ax.set_zlabel('Eixo z')

    #titulo
    plt.title('Manipulador RRR')
    ax.set_xlim3d(-2,2)
    ax.set_ylim3d(-2,2)
    ax.set_zlim3d(0,2)
    fig.canvas.draw() #mostra o plot


#configurando o plot
plt.ion()
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')
#plt.pause(0.01)

#variaveis
cont = 0
alfa = 0.2 #tamanho do passo
destino = np.array([[1.2,1,1.2]]).T

#vetores colunas do sistema de coordenadas global
i = np.array([[1,0,0,1]]).T
j = np.array([[0,1,0,1]]).T
k = np.array([[0,0,1,1]]).T
o = np.array([[0,0,0,1]]).T #origem

##Parametros dos robos
b1 = 1
b2 = 1
b3 = 1

#angulos de juntas iniciais
q1 = 2*pi*random.random()
q2 = 2*pi*random.random()
q3 = 2*pi*random.random()
q = np.array([[q1,q2,q3]]).T
for v in range(1000):
    
    # parametros de DH
    d1 = b1 
    d2 = 0
    d3 = 0
    a1 = 0
    a2 = b2
    a3 = b3
    alfa1 = math.pi/2
    alfa2 = 0
    alfa3 = 0
    theta1 = math.pi/2 + q[0]
    theta2 = q[1] 
    theta3 = q[2]

    #Matrizes homogêneas
    A1 = matrix_homogenea(d1,a1,alfa1,theta1)
    A2 = matrix_homogenea(d2,a2,alfa2,theta2)
    A3 = matrix_homogenea(d3,a3,alfa3,theta3)
    A = (A1@A2@A3)
    #Encontrando os pontos de interesse no sistema Global
    o1_1 = o #origem do SC 1
    o2_2 = o #origem do SC 2
    o3_3 = o #origem do SC 3
    o1_0 = A1@o1_1
    o2_0 = A1@(A2@o2_2)
    o3_0 = A1@(A2@(A3@o3_3))
    
    #os vetores z serao transformados em vetores linhas no R^3, para poder ser usado a funcao np.cross
    z0_0 = v_3d(k).T
    z1_0 = v_3d(A1@k).T
    z2_0 = v_3d(A1@(A2@k)).T
    z3_0 = v_3d(A1@(A2@(A3@k))).T

    aux = (v_3d(o3_0) - v_3d(o)).T
    aux = np.cross(z0_0,aux)
    J = np.array([np.concatenate((aux,z0_0),axis = None)])

    aux = (v_3d(o3_0) - v_3d(o1_0)).T
    aux = np.cross(z1_0,aux)
    J2 = np.array([np.concatenate((aux,z1_0),axis = None)])
    J = np.concatenate((J,J2),axis = 0)

    aux = (v_3d(o3_0) - v_3d(o2_0)).T
    aux = np.cross(z2_0,aux)
    J3 = np.array([np.concatenate((aux,z2_0),axis = None)])
    J = np.concatenate((J,J3),axis = 0)
    J = J.T
    f = (v_3d(o3_0) - destino)
    J = M_3x3(J) #foi descartado a parte da velocidade angular, porque ela nao esta sendo usada
    #e caso estivesse sendo, teria que ser convertida em taxa de angulo.
  
    q = q - alfa*((J.T@np.linalg.inv(J@J.T))@f)
    #q = q - alfa*(np.linalg.inv(J.T@J)@J.T@f)
    erro = distancia(o3_0,destino,3)
    #print('erro: ',erro,'\n','iteracao: ',v,'\n') 
    ax.clear()
    plot(o,o1_0,o2_0,o3_0,destino)
    if(erro < 0.001):
        print('Solucao q: \n',q,'\nNumero de iteracoes:',v)
        break
    plt.pause(0.001)
    




    
