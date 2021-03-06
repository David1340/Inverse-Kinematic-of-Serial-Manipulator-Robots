#Este código é baseado na figura 1.8 do livro do Spong
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin, sqrt, pi
import numpy as np
import math

def matrix_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

def plot_junta_revolucao(A,p):
    #A matriz de Rotação, p origem da junta no seu sistema de coordenadas
    r = 0.05
    theta = np.arange(0,2*pi,0.1) #theta de 0 a 2pi com passos de 0.1
    z = np.linspace(-0.1,0.1,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
    z, theta = np.meshgrid(z, theta) #transforma em vetores 2D causa do plot de superficie
    #[x,y,z].T = A@([x,y,z].T + p)
    x = (r*np.cos(theta) + p[0,0])*A[0,0] + (r*np.sin(theta) + p[1,0])*A[0,1] + \
     (z + p[2,0])*A[0,2] + np.ones_like(z)*A[0,3]
    y = (r*np.cos(theta) + p[0,0])*A[1,0] + (r*np.sin(theta) + p[1,0])*A[1,1] + \
     (z + p[2,0])*A[1,2] + np.ones_like(z)*A[1,3]
    z = (r*np.cos(theta) + p[0,0])*A[2,0] + (r*np.sin(theta) + p[1,0])*A[2,1] + \
     (z + p[2,0])*A[2,2] + np.ones_like(z)*A[2,3]
    ax.plot_surface(x, y, z,color = 'blue', alpha = 1)

#configurando o plot
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')

#vetores colunas do sistema de coordenadas global
o = np.array([[0,0,0,1]]).T #origem

##Parametros dos robos
b1 = 1
b2 = 0.5
b3 = 0.5
##parametros dinamicos
angulo1 = float(input('Digite o valor da junta 1 em Graus: ')) * math.pi/180 
angulo2 = float(input('Digite o valor da junta 2 em Graus: ')) * math.pi/180 
angulo3 = float(input('Digite o valor da junta 3 em Graus: ')) * math.pi/180

### parametros de DH
d1 = b1 
d2 = 0
d3 = 0
a1 = 0
a2 = b2
a3 = b3
alfa1 = math.pi/2
alfa2 = 0
alfa3 = 0
theta1 = math.pi/2 + angulo1 
theta2 = angulo2 
theta3 = angulo3

#Matrizes homogêneas
A1 = matrix_homogenea(d1,a1,alfa1,theta1)
A2 = matrix_homogenea(d2,a2,alfa2,theta2)
A3 = matrix_homogenea(d3,a3,alfa3,theta3)

#Pontos de interesse
o1_1 = o #junta 2
o2_2 = o #junta 3
o3_3 = o #ponto do centro do efetuador

#Encontrando os pontos de interesse no sistema Global
o1_0 = A1@o1_1
o2_0 = A1@(A2@o2_2)
o3_0 = A1@(A2@(A3@o3_3))

#Plotando Elos
plt.plot([o[0,0],o1_0[0,0]],[o[1,0],o1_0[1,0]],[o[2,0],o1_0[2,0]],'blue')
plt.plot([o1_0[0,0],o2_0[0,0]],[o1_0[1,0],o2_0[1,0]],[o1_0[2,0],o2_0[2,0]],'blue')
plt.plot([o2_0[0,0],o3_0[0,0]],[o2_0[1,0],o3_0[1,0]],[o2_0[2,0],o3_0[2,0]],'blue')


#Plotando Juntas
plot_junta_revolucao(np.eye(4),o)
plot_junta_revolucao(A1,o)
plot_junta_revolucao(A1@A2,o)

#Plotando efetuador
ax.scatter(o3_0[0,0],o3_0[1,0],o3_0[2,0])
#Legendas
ax.set_xlabel('Eixo x')
ax.set_ylabel('Eixo y')
ax.set_zlabel('Eixo z')
#titulo
plt.title('Manipulador RRR')
ax.set_xlim3d(-1,1)
ax.set_ylim3d(-1,1)
ax.set_zlim3d(0,2)
plt.show()
plt.pause(500)

