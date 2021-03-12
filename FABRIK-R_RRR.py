# %%
#Este código é baseado na figura 1.8 do livro do Spong
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin, sqrt, pi, atan2
import numpy as np
import random 

def matriz_antissimetrica(v):
    A = np.zeros((3,3))
    A[0,1] = -v[2,0]
    A[0,2] = v[1,0]
    A[1,2] = -v[0,0]
    A[1,0] = - A[0,1]
    A[2,0] = - A[0,2]
    A[2,1] = - A[1,2]
    return A
    
def norma(v):
    return v/(np.sqrt(np.sum(v*v)))

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

#Função que plota o manipulador, quando recebe os pontos de interesse
def plot(a,b,c,d,t):
    #Plotando Elos
    px = [a[0,0],b[0,0],c[0,0],d[0,0]]
    py = [a[1,0],b[1,0],c[1,0],d[1,0]]
    pz = [a[2,0],b[2,0],c[2,0],d[2,0]]
    plt.plot(px,py,pz,'blue')

    #Plotando Juntas
    ax.scatter(a[0,0],a[1,0],a[2,0])
    ax.scatter(b[0,0],b[1,0],b[2,0])
    ax.scatter(c[0,0],c[1,0],c[2,0])

    #Plotando efetuador
    ax.scatter(d[0,0],d[1,0],d[2,0])

    ax.scatter(t[0],t[1],t[2])
    #Legendas
    plt.legend(['Manipulador','P0','P1','P2','P3','objetivo'])

    ax.set_xlabel('Eixo x')
    ax.set_ylabel('Eixo y')
    ax.set_zlabel('Eixo z')

    #titulo
    plt.title('Manipulador RRR')
    ax.set_xlim3d(-2,2)
    ax.set_ylim3d(-2,2)
    ax.set_zlim3d(0,3)
    fig.canvas.draw() #mostra o plot
    plt.pause(0.001)
    #plt.pause(100)


#Função que plota o manipulador, quando recebe os pontos de interesse
def plot2(a,b,c,d,t):
    #Plotando Elos
    px = [a[0,0],b[0,0],c[0,0],d[0,0]]
    py = [a[1,0],b[1,0],c[1,0],d[1,0]]
    pz = [a[2,0],b[2,0],c[2,0],d[2,0]]
    plt.plot(px,py,pz,)
    
    #Plotando Juntas
    #px2 = [a[0,0],b[0,0],c[0,0]]
    #py2 = [a[1,0],b[1,0],c[1,0]]
    #pz2 = [a[2,0],b[2,0],c[2,0]]
    #ax.scatter(px2,py2,pz2,'blue')

    #Plotando efetuador
    #ax.scatter(d[0,0],d[1,0],d[2,0],'blue')
    #plotando destino
    #ax.scatter(t[0],t[1],t[2],'blue')
    #Legendas
    
    #plt.legend(['Elo 1','Elo 2','Elo 3','Junta 1 (Revolucao)','Junta 2 (Revolucao)'\
    #            ,'Junta 3 (Revolucao)','Efetuador','objetivo'])

    ax.set_xlabel('Eixo x')
    ax.set_ylabel('Eixo y')
    ax.set_zlabel('Eixo z')

    #titulo
    plt.title('Manipulador RRR')
    ax.set_xlim3d(-2,2)
    ax.set_ylim3d(-2,2)
    ax.set_zlim3d(0,3)
    fig.canvas.draw() #mostra o plot
    #plt.pause(0.001)


#configurando o plot
plt.ion()
fig = plt.figure('1')
ax =  fig.add_subplot(111, projection = '3d')

#variaveis
n = 3
destino = np.array([[1.5,1.5,1]]).T

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

# parametros de DH
d1 = b1 
d2 = 0
d3 = 0
a1 = 0
a2 = b2
a3 = b3
alfa1 = pi/2
alfa2 = 0
alfa3 = 0
theta1 = pi/2 + q[0]
theta2 = q[1] 
theta3 = q[2]

#Matrizes homogêneas
A1 = matrix_homogenea(d1,a1,alfa1,theta1)
A2 = matrix_homogenea(d2,a2,alfa2,theta2)
A3 = matrix_homogenea(d3,a3,alfa3,theta3)
A = (A1@A2@A3)

p0 = o # #origem do SC 0#Encontrando os pontos de interesse no sistema Global

p1_1 = o #origem do SC 1
p2_2 = o #origem do SC 2
p3_3 = o #origem do SC 3
p1_0 = v_3d(A1@p1_1)
p2_0 = v_3d(A1@(A2@p2_2))
p3_0 = v_3d(A1@(A2@(A3@p3_3)))
p0 = v_3d(p0)
plot(p0,p1_0,p2_0,p3_0,destino)
p = np.concatenate((p0.T,p1_0.T,p2_0.T,p3_0.T),axis = 0)

#primeira interação
v1 = np.array([p[3] - p[1]]).T#vetor 1 que define o plano, junta2 -> efetuador
v1 = norma(v1)

v2 = np.array(destino.T - p[1]).T#vetor 2 que define o plano, junta2 -> destino
v2 = norma(v2) 
n3 = matriz_antissimetrica(v1)@v2#vetor normal ao plano
n3 = norma(n3)

v3 = np.array(p[2] - destino.T).T#vetor do destino para junta 3
v3 = norma(v3)
p3_0 = destino

p2_0_new = p3_0 + v3

d = -p[1].dot(n3) #parâmetro d da equação do plano

d_pnew_plano = p2_0_new.T@n3 + d
p2_0_new2 = p2_0_new - d_pnew_plano*n3
v4 = p2_0_new2 - p3_0 
v4 = norma(v4)
p2_0_new3 = p3_0 + v4
#plot do plano
# create x,y
xx, yy = np.meshgrid(range(-2,2), range(-2,2))
# calculate corresponding z
z = (-n3[0] * xx - n3[1] * yy - d) * 1. /n3[2]

plt.pause(1)
fig = plt.figure('2')
ax =  fig.add_subplot(111, projection = '3d')
ax.plot_surface(xx, yy, z, alpha=0.2)
#ax.clear()
plot2(p0,p1_0,p2_0,p2_0,p2_0) #parte de baixo
plot2(p2_0_new,p3_0,p3_0,p3_0,p3_0) #parte de cima antes de projetar
plot2(p2_0_new3,p3_0,p3_0,p3_0,p3_0) #parte de cima depois de projetar
ax.scatter(p0[0,0],p0[1,0],p0[2,0])
ax.scatter(p1_0[0,0],p1_0[1,0],p1_0[2,0])
ax.scatter(p2_0[0,0],p2_0[1,0],p2_0[2,0])
ax.scatter(p2_0_new[0,0],p2_0_new[1,0],p2_0_new[2,0])
ax.scatter(p2_0_new3[0,0],p2_0_new3[1,0],p2_0_new3[2,0])
ax.scatter(p3_0[0,0],p3_0[1,0],p3_0[2,0])
plt.legend(['Manipulador','Manipulador','Manipulador','Plano','P0'\
            ,'P1','P2','P2^','P2*','P3'])
#fig.canvas.draw()
#plt.pause(1000)
   

# %%

