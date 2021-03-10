#Este código é baseado na figura 1.8 do livro do Spong
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin, sqrt 
import numpy as np
import math

def matrix_homogenea(d,a,alfa,theta):
    L1 = np.array([round(cos(theta),4), round(-sin(theta)*cos(alfa),4), round(sin(theta)*sin(alfa),4),round(a*cos(theta),4)])
    L2 = np.array([round(sin(theta),4), round(cos(theta)*cos(alfa),4),round(-cos(theta)*sin(alfa),4),round(a*sin(theta),4)])
    L3 = np.array([0, round(sin(alfa),4), round(cos(alfa),4), d])
    L4 = np.array([0,0,0,1])
    A = np.array([L1,L2,L3,L4])
    return A

#vetores colunas do sistema de coordenadas global
i = np.array([[1,0,0,1]]).T
j = np.array([[0,1,0,1]]).T
k = np.array([[0,0,1,1]]).T
o = np.array([[0,0,0,1]]).T #origem

#Pontos de interesse
o1_1 = o
o2_2 = o
o3_3 = o


##Parametros dos robos
b1 = 1
b2 = 1
b3 = 1 

##parametros dinamicos
angulo = [0.0,0.0,0.0]
angulo[0] = float(input('Digite o valor da junta 1 em Graus: ')) * math.pi/180 
angulo[1] = float(input('Digite o valor da junta 2 em Graus: ')) * math.pi/180 
angulo[2] = float(input('Digite o valor da junta 3 em Graus: ')) * math.pi/180

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
theta1 = math.pi/2 + angulo[0] 
theta2 = angulo[1] 
theta3 = angulo[2]

#Matrizes homogêneas
A1 = matrix_homogenea(d1,a1,alfa1,theta1)
A2 = matrix_homogenea(d2,a2,alfa2,theta2)
A3 = matrix_homogenea(d3,a3,alfa3,theta3)

#Encontrando os pontos de interesse no sistema Global
o1_0 = A1@o1_1
o2_0 = A1@(A2@o2_2)
o3_0 = A1@(A2@(A3@o3_3))

#configurando o plot
plt.ion()
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')

#Plotando Elos
plt.plot([o[0,0],o1_0[0,0]],[o[1,0],o1_0[1,0]],[o[2,0],o1_0[2,0]])
plt.plot([o1_0[0,0],o2_0[0,0]],[o1_0[1,0],o2_0[1,0]],[o1_0[2,0],o2_0[2,0]])
plt.plot([o2_0[0,0],o3_0[0,0]],[o2_0[1,0],o3_0[1,0]],[o2_0[2,0],o3_0[2,0]])

#Plotando Juntas
ax.scatter(o[0,0],o[1,0],o[2,0])
ax.scatter(o1_0[0,0],o1_0[1,0],o1_0[2,0])
ax.scatter(o2_0[0,0],o2_0[1,0],o2_0[2,0])

#Plotando efetuador
ax.scatter(o3_0[0,0],o3_0[1,0],o3_0[2,0])

#Legendas
plt.legend(['Elo 1','Elo 2','Elo 3','Junta 1 (Revolucao)', 'Junta 2 (Revolucao)'\
            ,'Junta 3 (Revolucao)','Efetuador'])
ax.set_xlabel('Eixo x')
ax.set_ylabel('Eixo y')
ax.set_zlabel('Eixo z')

#titulo
plt.title('Manipulador RRR')
ax.scatter(2,2,2) #ponto para melhor visualização do gráfico
ax.scatter(-2,-2,2)
fig.canvas.draw()

while(True):
    ##Atualizar dinamicos
    opcao = input('Qual angulo você deseja atualizar: ')
    
    if(opcao == '1'):
        incremento = float(input('Digite o incremento da junta 1 em Graus: ')) * math.pi/180
        opcao2 = 0
    elif(opcao == '2'):
        incremento = float(input('Digite o incremento da junta 2 em Graus: ')) * math.pi/180
        opcao2 = 1
    elif(opcao == '3'):
        incremento = float(input('Digite o incremento da junta 3 em Graus: ')) * math.pi/180
        opcao2 = 2
    else:
        print('opção invalida')
        incremento  = 0
        opcao = 10
        erro = 0
    nangulo = angulo[opcao2] + incremento

    if(opcao != 10):
        erro = sqrt((nangulo - angulo[opcao2])**2)

    while(erro >= 0.1):
        print(angulo[opcao2])
        if(incremento >= 0):
            angulo[opcao2] = angulo[opcao2] + 0.1
            print(nangulo)
            print(opcao2)
            print(angulo[opcao2])
            erro = sqrt((nangulo - angulo[opcao2])**2)
        else:
            angulo[opcao2] = angulo[opcao2] - 0.1
            erro = sqrt((nangulo - angulo[opcao2])**2)

        ### parametros de DH que precisam ser atualizados
        if(opcao2 == 0):
            theta1 = math.pi/2 + angulo[0]
        elif(opcao2 == 1):
            theta2 = angulo[1]
        elif(opcao2 == 2):
            theta3 = angulo[2]
        else:
            break
        ax.clear()
        
        #Matrizes homogêneas
        A1 = matrix_homogenea(d1,a1,alfa1,theta1)
        A2 = matrix_homogenea(d2,a2,alfa2,theta2)
        A3 = matrix_homogenea(d3,a3,alfa3,theta3)

        #Encontrando os pontos de interesse no sistema Global
        o1_0 = A1@o1_1
        o2_0 = A1@(A2@o2_2)
        o3_0 = A1@(A2@(A3@o3_3))

        #Plotando Elos
        plt.plot([o[0,0],o1_0[0,0]],[o[1,0],o1_0[1,0]],[o[2,0],o1_0[2,0]])
        plt.plot([o1_0[0,0],o2_0[0,0]],[o1_0[1,0],o2_0[1,0]],[o1_0[2,0],o2_0[2,0]])
        plt.plot([o2_0[0,0],o3_0[0,0]],[o2_0[1,0],o3_0[1,0]],[o2_0[2,0],o3_0[2,0]])

        #Plotando Juntas
        ax.scatter(o[0,0],o[1,0],o[2,0])
        ax.scatter(o1_0[0,0],o1_0[1,0],o1_0[2,0])
        ax.scatter(o2_0[0,0],o2_0[1,0],o2_0[2,0])

        #Plotando efetuador
        ax.scatter(o3_0[0,0],o3_0[1,0],o3_0[2,0])
        
        #Legendas
        plt.legend(['Elo 1','Elo 2','Elo 3','Junta 1 (Revolucao)', 'Junta 2 (Revolucao)'\
                    ,'Junta 3 (Revolucao)','Efetuador'])
        ax.set_xlabel('Eixo x')
        ax.set_ylabel('Eixo y')
        ax.set_zlabel('Eixo z')
        
        #titulo
        plt.title('Manipulador RRR')
        ax.scatter(2,2,2) #ponto para melhor visualização do gráfico
        ax.scatter(-2,-2,2)
        fig.canvas.draw()
        plt.pause(0.00001)
