import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin 
import numpy as np
import math

def matrix_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

#configurando o plot
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')

#vetores colunas do sistema de coordenadas global
i = np.array([[1,0,0,1]]).T
j = np.array([[0,1,0,1]]).T
k = np.array([[0,0,1,1]]).T
o = np.array([[0,0,0,1]]).T #origem

##Parametros dos robos
b1 = 1
b2 = 1
b3 = 1
b4 = 1 

##parametros dinamicos
angulo1 = float(input('Digite o valor da junta 1 em Graus: ')) * math.pi/180 
angulo2 = float(input('Digite o valor da junta 2 em Graus: ')) * math.pi/180 
distancia1 = float(input('Digite o valor da junta 3: '))

### parametros de DH
d1 = b1 + b2
d2 = 0
d3 = b3 + distancia1
a1 = 0
a2 = 0
a3 = 0
alfa1 = math.pi/2
alfa2 = math.pi/2
alfa3 = 0
theta1 = math.pi/2 + angulo1 
theta2 = math.pi/2 + angulo2
theta3 = math.pi/2

#Matrizes homogêneas
A1 = matrix_homogenea(d1,a1,alfa1,theta1)
A2 = matrix_homogenea(d2,a2,alfa2,theta2)
A3 = matrix_homogenea(d3,a3,alfa3,theta3)

#Pontos de interesse
p1_0 = np.array([[0,0,b1,1]]).T
p2_2 = np.array([[0,0,b3,1]]).T
o1_1 = o
o3_3 = o
o2_2 = o

#Encontrando os pontos de interesse no sistema Global
p2_0 = A1@(A2@p2_2)
o1_0 = A1@o1_1
o2_0 = A1@(A2@o2_2)
o3_0 = A1@(A2@(A3@o3_3))

#Plotando Elos
plt.plot([o[0,0],p1_0[0,0]],[o[1,0],p1_0[1,0]],[o[2,0],p1_0[2,0]])
plt.plot([p1_0[0,0],o2_0[0,0]],[p1_0[1,0],o2_0[1,0]],[p1_0[2,0],o2_0[2,0]])
plt.plot([o2_0[0,0],p2_0[0,0]],[o2_0[1,0],p2_0[1,0]],[o2_0[2,0],p2_0[2,0]])


#Plotando Juntas
ax.scatter(p1_0[0,0],p1_0[1,0],p1_0[2,0])
ax.scatter(o2_0[0,0],o2_0[1,0],o2_0[2,0])
plt.plot([p2_0[0,0],o3_0[0,0]],[p2_0[1,0],o3_0[1,0]],[p2_0[2,0],o3_0[2,0]])

#Plotando efetuador
ax.scatter(o3_0[0,0],o3_0[1,0],o3_0[2,0])

#Legendas
plt.legend(['Elo 1','Elo 2','Elo 3','Junta 3 (Prismatica)',\
            'Junta 1 (Revolucao)', 'Junta 2 (Revolucao)','Efetuador'])
ax.set_xlabel('Eixo x')
ax.set_ylabel('Eixo y')
ax.set_zlabel('Eixo z')
#titulo
plt.title('Manipulador RRP')

ax.scatter(3,3,3) #ponto para melhor visualização do gráfico
plt.show()

