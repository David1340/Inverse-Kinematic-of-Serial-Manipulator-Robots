#Este código é baseado na figura 1.8 do livro do Spong
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin,sqrt,atan2,pi 
import numpy as np
import math

def matrix_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),5), round(-sin(theta)*cos(alfa),5), round(sin(theta)*sin(alfa),5),round(a*cos(theta),5)])
    L2 = np.array([round(sin(theta),5), round(cos(theta)*cos(alfa),5),round(-cos(theta)*sin(alfa),5),round(a*sin(theta),5)])
    L3 = np.array([0, round(sin(alfa),5), round(cos(alfa),5), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a - b)**2        
    return sqrt(d)

def orientacao(A):
    R = atan2(A[1,0],A[0,0])*180/pi
    P = atan2(-A[2,0],distancia(A[2,1],A[2,2],1))*180/pi
    Y = atan2(A[2,1],A[2,2])*180/pi
    #Z - Y - X
    result = [Y,P,R]
    return result
    
#configurando o plot
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')

#vetores colunas do sistema de coordenadas global
i = np.array([[1,0,0,1]]).T
j = np.array([[0,1,0,1]]).T
k = np.array([[0,0,1,1]]).T
o = np.array([[0,0,0,1]]).T #origem

##Parametros dos robos
b1 = 0.2 #20 cm
b2 = 0.1
b3 = 0.2 
b4 = 0.1
b5 = 0.2
b6 = 0.1
b7 = 0.2
##parametros dinamicos
angulo1 = float(input('Digite o valor da junta 1 em Graus: ')) * math.pi/180 
angulo2 = float(input('Digite o valor da junta 2 em Graus: ')) * math.pi/180 
angulo3 = float(input('Digite o valor da junta 3 em Graus: ')) * math.pi/180
angulo4 = float(input('Digite o valor da junta 4 em Graus: ')) * math.pi/180 
angulo5 = float(input('Digite o valor da junta 5 em Graus: ')) * math.pi/180 
angulo6 = float(input('Digite o valor da junta 6 em Graus: ')) * math.pi/180
angulo7 = float(input('Digite o valor da junta 7 em Graus: ')) * math.pi/180
#parametros de DH
d1 = b1 
d2 = 0
d3 = b2 + b3
d4 = 0 
d5 = b4 + b5
d6 = 0
d7 = 0
a1 = 0
a2 = 0
a3 = 0
a4 = 0
a5 = 0
a6 = b6
a7 = 0
alfa1 = math.pi/2
alfa2 = -math.pi/2
alfa3 = math.pi/2
alfa4 = -math.pi/2
alfa5 = math.pi/2
alfa6 = math.pi/2
alfa7 = math.pi/2
theta1 = math.pi/2 + angulo1 
theta2 = angulo2
theta3 = angulo3
theta4 = angulo4
theta5 = angulo5
theta6 = math.pi/2 + angulo6
theta7 = math.pi/2 + angulo7

#Matrizes homogêneas
A1 = matrix_homogenea(d1,a1,alfa1,theta1)
A2 = matrix_homogenea(d2,a2,alfa2,theta2)
A3 = matrix_homogenea(d3,a3,alfa3,theta3)
A4 = matrix_homogenea(d4,a4,alfa4,theta4)
A5 = matrix_homogenea(d5,a5,alfa5,theta5)
A6 = matrix_homogenea(d6,a6,alfa6,theta6)
A7 = matrix_homogenea(d7,a7,alfa7,theta7)
A = A1@(A2@(A3@(A4@(A5@(A6@A7)))))
orient = orientacao(A)
print('orientacao: ',A,"\n",orient)
#Pontos de interesse
o1_1 = o #junta1
o2_2 = np.array([[0,0,b2,1]]).T #junta2
o3_3 = o #junta3
o4_4 =  np.array([[0,0,b4,1]]).T #junta4
o5_5 = o #junta5
o6_6 = o #junta6
o7_7 = np.array([[0,0,b7,1]]).T #centro do efetuador
w = 0.05 #largura do efetuador
h = 0.05 #altura do efetuador

#O efetuador será um u, para plotarmos um precisamos de 4 pontos
#topo esquerdo do u
p1_7 = np.array([[0,-w/2,b7 + h,1]]).T
#base esquerda do u
p2_7 = np.array([[0,-w/2,b7,1]]).T
#base direita do u
p3_7 = np.array([[0,w/2,b7,1]]).T
#topo direito do u
p4_7 = np.array([[0,w/2,b7 + h,1]]).T


#Encontrando os pontos de interesse no sistema Global
o1_0 = A1@o1_1
o2_0 = A1@(A2@o2_2)
o3_0 = A1@(A2@(A3@o3_3))
o4_0 = A1@(A2@(A3@(A4@o4_4)))
o5_0 = A1@(A2@(A3@(A4@(A5@o5_5))))
o6_0 = A1@(A2@(A3@(A4@(A5@(A6@o6_6)))))
o7_0 = A1@(A2@(A3@(A4@(A5@(A6@(A7@(o7_7)))))))
p1_0 = A1@(A2@(A3@(A4@(A5@(A6@(A7@(p1_7)))))))
p2_0 = A1@(A2@(A3@(A4@(A5@(A6@(A7@(p2_7)))))))
p3_0 = A1@(A2@(A3@(A4@(A5@(A6@(A7@(p3_7)))))))
p4_0 = A1@(A2@(A3@(A4@(A5@(A6@(A7@(p4_7)))))))
#Plotando Elos
plt.plot([o[0,0],o1_0[0,0]],[o[1,0],o1_0[1,0]],[o[2,0],o1_0[2,0]])
plt.plot([o1_0[0,0],o2_0[0,0]],[o1_0[1,0],o2_0[1,0]],[o1_0[2,0],o2_0[2,0]])
plt.plot([o2_0[0,0],o3_0[0,0]],[o2_0[1,0],o3_0[1,0]],[o2_0[2,0],o3_0[2,0]])
plt.plot([o3_0[0,0],o4_0[0,0]],[o3_0[1,0],o4_0[1,0]],[o3_0[2,0],o4_0[2,0]])
plt.plot([o4_0[0,0],o5_0[0,0]],[o4_0[1,0],o5_0[1,0]],[o4_0[2,0],o5_0[2,0]])
plt.plot([o5_0[0,0],o6_0[0,0]],[o5_0[1,0],o6_0[1,0]],[o5_0[2,0],o6_0[2,0]])
plt.plot([o6_0[0,0],o7_0[0,0]],[o6_0[1,0],o7_0[1,0]],[o6_0[2,0],o7_0[2,0]])
#Plotando efetuador
plt.plot([p1_0[0,0],p2_0[0,0],p3_0[0,0],p4_0[0,0]],[p1_0[1,0],p2_0[1,0],p3_0[1,0],p4_0[1,0]],[p1_0[2,0],p2_0[2,0],p3_0[2,0],p4_0[2,0]])
#ax.scatter(o7_0[0,0],o7_0[1,0],o7_0[2,0])
#Plotando Juntas
ax.scatter(o[0,0],o[1,0],o[2,0])
ax.scatter(o1_0[0,0],o1_0[1,0],o1_0[2,0])
ax.scatter(o2_0[0,0],o2_0[1,0],o2_0[2,0])
ax.scatter(o3_0[0,0],o3_0[1,0],o3_0[2,0])
ax.scatter(o4_0[0,0],o4_0[1,0],o4_0[2,0])
ax.scatter(o5_0[0,0],o5_0[1,0],o5_0[2,0])
ax.scatter(o6_0[0,0],o6_0[1,0],o6_0[2,0])

#Legendas
plt.legend(['Elo 1','Elo 2','Elo 3','Elo4', 'Elo5'\
            ,'Elo6','Elo7','Efetuador'])
ax.set_xlabel('Eixo x')
ax.set_ylabel('Eixo y')
ax.set_zlabel('Eixo z')
#titulo
plt.title('Manipulador 7DOF')
ax.set_xlim3d(-0.5,0.5)
ax.set_ylim3d(-0.5,0.5)
ax.set_zlim3d(0,1)
plt.show()


