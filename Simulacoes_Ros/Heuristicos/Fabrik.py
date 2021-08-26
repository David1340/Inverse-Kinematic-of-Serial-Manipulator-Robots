#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Forward And Backward Reaching Inverse Kinematic 
#para encontrar uma configuação q dada uma posição (x,y,z) no espaço para o Pioneer 7DOF

#Import do modulo funcoes.py
import sys
sys.path.append('/home/david/Pibic2021/Inverse-Kinematic-of-Serial-Manipulator-Robots/Simulacoes_Ros')
from funcoes import Cinematica_Direta2, norm, distancia, random_pose, S \
                    , rotationar_vetor, projecao_ponto_plano

from math import pi,acos
import numpy as np
import random 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#criar um vetor coluna a partir de uma lista
def vetor(v):   
    return np.array([[v[0],v[1],v[2]]]).T

#Interação básica do Fabrik restringida no plano
def iteracao_Fabrik(p,p0,d,normal):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    normal = vetor(normal)
    p0 = vetor(p0)
    p = vetor(p)
    proj = projecao_ponto_plano(normal,p0,p)
    r = distancia(p0,proj,3)
    delta = d/r
    pnew = (1- delta)*p0 + delta*proj
    return pnew

#Calcula o acos de um angulo arrendondado em 10 casas decimais
def acosr(x):
    return acos(np.round(x,10))

#configurando o Rviz
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(100) # 100hz 
hello_str = JointState()
hello_str.header = Header()
hello_str.header.stamp = rospy.Time.now()
hello_str.name = ['One_joint', 'Two_joint', 'Three_joint', 'Four_joint'\
    ,'Five_joint','Six_joint','Seven_joint','L_F_joint','R_F_joint']
hello_str.position  = [0,0,0,0,0,0,0,0,0]    
hello_str.velocity = []
hello_str.effort = []

n = 7 #numero de juntas
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])

posicaod, orientd = random_pose()
destino = posicaod
#destino = vetor([0.004,0.039,0.446])
print('destino:',np.round(destino[:,0],3))

b = np.array([0.05,0.075,0.075,0.0725,0.0725,0.075,0.075]) #tamanho dos elos
qlim = [2.6179,1.5358,2.6179,1.6144,2.6179,1.8413,1.7889]  
q = np.zeros([7,1])
for i in range(np.size(q)):
    q[i] = random.uniform(-qlim[i],qlim[i])

p,D = Cinematica_Direta2(q)

pcte = p[:,0].copy()

pl = p.copy()
Dl = D.copy()

erro = distancia(p[:,n],destino,3)
print('erro inicial:', erro)

K = 100 #número máximo de iterações
k = 0
erromin = 10**-4

while(erro > erromin and k < K):
    #Forward
    for i in range(n-1,0,-1):
        if(i == 6): #Se for a junta 6 
            pl[:,i+1] = destino[:,0]#coloca o efetuador no destino          
            v1 = vetor(pl[:,i+1] - p[:,i])#p6 -> p7' (efetuador)
            v1 = v1/norm(v1)
            v2 = vetor(p[:,i-1] - p[:,i])#p6 -> p5
            v2 = v2/norm(v2)
            naux = (S(v1)@v2)#produto vetorial            
            if(norm(naux) > 0.00001):
                Dl[:,i] = naux[:,0]/norm(naux)
            else:
                Dl[:,i] = D[:,i].copy()

            pl[:,i] = iteracao_Fabrik(p[:,i],pl[:,i+1],b[i],Dl[:,i])[:,0] 

        elif(i == 1 or i == 3 or i == 5): #Se for junta Hinge (2,4 e 6)

            if(i == 1 or i == 3):#Se a junta prev for pivot (2 e 4)
                pl[:,i] = pl[:,i+1] - Dl[:,i+1]*b[i]
                paux = iteracao_Fabrik(p[:,i-1],pl[:,i],b[i],Dl[:,i+1])[:,0]
                v1 = vetor(paux - pl[:,i])                
                v1 = v1/norm(v1) #vetor de refência
                v2 = vetor(Dl[:,i+2])
                v2 = v2/norm(v2)
                th = np.real(acosr(v1.T@v2))

                #v3 é um vetor ortogonal ao vetor de referência (v1)
                v3 = rotationar_vetor(v1,vetor(Dl[:,i+1]),pi/2)[:,0]
                if(v3.T@v2 < 0):
                    th = -th

                Dl[:,i] = rotationar_vetor(v2,vetor(Dl[:,i+1]),(pi/2) - th)[:,0] 
                Dl[:,i] = Dl[:,i]/norm(D[:,i])

            else: #Se for a junta prev for hinge (6)
                pl[:,i] = iteracao_Fabrik(p[:,i-1],pl[:,i+1],b[i],Dl[:,i+1])[:,0]
                v1 = vetor(pl[:,i] - pl[:,i+1])
                v1 = v1/norm(v1) 
                Dl[:,i] = rotationar_vetor(vetor(Dl[:,i+1]),v1,pi/2)[:,0] 
                Dl[:,i] = Dl[:,i]/norm(D[:,i])
                
        elif(i == 2 or i == 4): #Se a junta for pivot (3 e 5)
            pl[:,i] = iteracao_Fabrik(p[:,i-1],pl[:,i+1],b[i],Dl[:,i+1])[:,0]
            v1 = pl[:,i] - pl[:,i+1]
            v1 = v1/norm(v1)
            Dl[:,i] = -v1
 
    Dl[:,0] = [0,0,1]
    pl[:,0] = pl[:,1] - b[0]*Dl[:,0] 
    D = Dl.copy()
    p = pl.copy()

    #Backward
    for i in range(0,n):
        if(i == 1 or i == 3 or i == 5 or i == 6): #Se for junta Hinge
            if(i == 1 or i == 3 or i == 5):#Se a junta prev for pivot (2,4 e 6)
                pl[:,i] = pl[:,i-1] + Dl[:,i-1]*b[i-1]
                paux = iteracao_Fabrik(p[:,i+1],pl[:,i],b[i],Dl[:,i-1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                if(i != 1): 
                    v2 = vetor(Dl[:,i-2])
                else: v2 = np.array([[1,0,0]]).T
                th = np.real(acosr(v1.T@v2))

                #v3 é um vetor ortogonal ao vetor de referência (v1)
                v3 = rotationar_vetor(v1,vetor(Dl[:,i-1]),pi/2)[:,0]
                if(v3.T@v2 < 0):
                    th = -th

                Dl[:,i] = rotationar_vetor(v2,vetor(Dl[:,i-1]),(pi/2) - th)[:,0] 
                Dl[:,i] = Dl[:,i]/norm(D[:,i])

            else: #Se a junta prev for Hinge (7)
                pl[:,i] = iteracao_Fabrik(p[:,i+1],pl[:,i-1],b[i-1],Dl[:,i-1])[:,0]
                paux = iteracao_Fabrik(p[:,i+1],pl[:,i],b[i],Dl[:,i-1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                Dl[:,i] = rotationar_vetor(vetor(Dl[:,i-1]),v1,pi/2)[:,0]
                Dl[:,i] = Dl[:,i]/norm(D[:,i])
                #efetuador
                pl[:,7]  = iteracao_Fabrik(p[:,7],pl[:,6],b[6],Dl[:,6])[:,0] 

        elif(i == 2 or i == 4): #Se a junta for pivot (3 e 5)
            pl[:,i] = iteracao_Fabrik(p[:,i+1],pl[:,i-1],b[i-1],Dl[:,i-1])[:,0]
            v1 = pl[:,i] - pl[:,i-1]
            v1 = v1/norm(v1)
            Dl[:,i] = v1
        elif(i == 0): #Primeira junta eixo de atuação e posição são fixos
            pl[:,0] = pcte #[0,0,0.075]
        #END-Backward
    
    p = pl.copy()
    D = Dl.copy()
    
    erro = distancia(p[:,n],destino,3)
    
    print('erro: ', erro)
    k = k +1

#Conversão da solução gráfica em um vetor de ângulo
for i in range(7):
    D[:,i] = D[:,i]/norm(D[:,i])
    
#v é o vetor ortogonal ao vetor de refência para o cálculo dos ângulos 
for i in range(7):
    if(i == 0):
        vref = x
    elif(i == 6):
        vref = vetor(p[:,6] - p[:,5])
        vref = vref/norm(vref)
    else:
        vref = vetor(D[:,i-1])

    #v é o vetor ortogonal ao vetor de refência para o cálculo dos ângulos 
    v = rotationar_vetor(vref,vetor(D[:,i]),pi/2) 
    
    #cálculo o ângulo
    if(i == 5):
        vaux = vetor(p[:,6] - p[:,5])
        vaux = vaux/norm(vaux)
        q[5] = acosr(vaux.T@vref)
        if(vaux.T@v < 0): q[5] = - q[5]
    elif(i == 6):
        vaux = vetor(p[:,7] - p[:,6])
        vaux = vaux/norm(vaux)
        q[6] = acosr(vaux.T@vref)
        if(vaux.T@v < 0): q[6] = - q[6]
    else:    
        q[i] = acosr(D[:,i+1].T@vref)
        if(D[:,i+1].T@v < 0): q[i] = - q[i]

print('erro: ', erro)
print('k: ',k)  
#print('q: ', q)

#atualiza os angulos do manipulador no Rviz
while not rospy.is_shutdown():
        
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0],0,0]
        pub.publish(hello_str)
        rate.sleep()  

