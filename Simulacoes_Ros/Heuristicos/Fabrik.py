#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Forward And Backward Reaching Inverse Kinematic 
#para encontrar uma configuação q dada uma posição (x,y,z) no espaço para o Pioneer 7DOF

#Import do modulo funcoes.py
import sys
sys.path.append('/home/david/Pibic2021/Inverse-Kinematic-of-Serial-Manipulator-Robots/Simulacoes_Ros')
from funcoes import Cinematica_Direta

from math import cos, sin, sqrt, pi,acos
import numpy as np
import random 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#criar um vetor coluna a partir de uma lista
def vetor(v):   
    return np.array([[v[0],v[1],v[2]]]).T

#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2     
    return sqrt(d)

#Retorna a Matriz de transformacao Homogeneadados  usando como entrada os parametros de DH
def matriz_homogenea(d,a,alfa,theta):
    L1 = np.array([cos(theta),-sin(theta)*cos(alfa),sin(theta)*sin(alfa),a*cos(theta)])
    L2 = np.array([sin(theta),cos(theta)*cos(alfa),-cos(theta)*sin(alfa),a*sin(theta)])
    L3 = np.array([0,sin(alfa),cos(alfa), d])
    L4 = np.array([0.0,0.0,0.0,1.0])
    A = np.array([L1,L2,L3,L4])
    return A

#Calcula a norma de um vetor 
def norm(v):
    return sqrt(v[[0]]**2 + v[[1]]**2 + v[[2]]**2)

#Projeta um ponto em um plano
def projecao_ponto_plano(normal,p0,p):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    #constante d da equação do 
    if(np.round(norm(normal),10) > 1): 
        normal = normal/norm(normal)
        
    d = -normal.T@p0 #produto escala 
    #distancia do ponto ao plano
    alpha = (-d - normal.T@p)/(normal[0,0]**2 +  normal[1,0]**2 + normal[2,0]**2)
    ponto_projetado = p + alpha*normal
    return ponto_projetado

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
    #r = distancia(p0[:,0],proj[:,0],3)
    delta = d/r
    pnew = (1- delta)*p0 + delta*proj
    return pnew

#realiza a operação de multiplicação de quaternios
def multiplicacao_quaternios(q,q2):
    resultado = np.zeros([4,1])
    resultado[0,0] = q[0,0]*q2[0,0] -q[1,0]*q2[1,0] -q[2,0]*q2[2,0] -q[3,0]*q2[3,0] 
    resultado[1,0] = q[0,0]*q2[1,0] +q[1,0]*q2[0,0] +q[2,0]*q2[3,0] -q[3,0]*q2[2,0] 
    resultado[2,0] = q[0,0]*q2[2,0] -q[1,0]*q2[3,0] +q[2,0]*q2[0,0] +q[3,0]*q2[1,0] 
    resultado[3,0] = q[0,0]*q2[3,0] +q[1,0]*q2[2,0] -q[2,0]*q2[1,0] +q[3,0]*q2[0,0] 
    return resultado

#gira p em torno de v em th rad
def rotationar_vetor(p,v,th):
    a = cos(th/2)
    v = v/norm(v)
    b = v[0,0]*sin(th/2)
    c = v[1,0]*sin(th/2)
    d = v[2,0]*sin(th/2)
    p_aumentado = np.zeros([4,1])
    p_aumentado[1:4,0] = p[:,0]
    h = np.array([[a,b,c,d]]).T
    hx = np.array([[a,-b,-c,-d]]).T
    p_r = multiplicacao_quaternios(h,p_aumentado)
    q_r = multiplicacao_quaternios(p_r,hx)
    return q_r[1:4]

#Matriz Antissimetrica
def S(a):
    #A = [0,-az,ay ; az,0,-ax ; -ay,ax,0]
    #Uso para calcular produto vetorial entre vetores u x v = S(u) * v
    A = np.zeros((3,3))
    A[0,1] = -a[2]
    A[0,2] = a[1]
    A[1,2] = -a[0]
    A[1,0] = - A[0,1]
    A[2,0] = - A[0,2]
    A[2,1] = - A[1,2]
    return A

#Calcula o acos de um angulo arrendondado em 10 casas decimais
def acosr(x):
    return acos(np.round(x,10))

#Gera uma pose alcançável 
def random_pose(): 
    #valor maximo que a junta pode assumir
    qlim = [2.6179,1.5358,2.6179,1.6144,2.6179,1.8413,1.7889]   
    #angulos de juntas iniciais
    q = np.zeros([7,1])
    for a in range(np.size(q)):
        q[a] = random.uniform(-qlim[a],qlim[a])

    #Parâmetros Físicos do manipulador [m]
    base = 0.05 #5 cm
    L = 0.075 #distância da ultima junta a extremidade do efetuador

    #parametros de DH constantes
    d1 = 0.075 + base
    d2 = 0
    d3 = 0.15
    d4 = 0 
    d5 = 0.145
    d6 = 0
    d7 = 0
    a1 = 0
    a2 = 0
    a3 = 0
    a4 = 0
    a5 = 0
    a6 = 0.075
    a7 = 0
    alpha1 = pi/2
    alpha2 = -pi/2
    alpha3 = pi/2
    alpha4 = -pi/2
    alpha5 = pi/2
    alpha6 = pi/2
    alpha7 = pi/2
    # parametros de DH variáveis
    theta1 = pi/2 + q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    theta5 = q[4]
    theta6 = pi/2 + q[5]
    theta7 = pi/2 + q[6]
    #Matrizes homogêneas
    A1 = matriz_homogenea(d1,a1,alpha1,theta1)
    A2 = matriz_homogenea(d2,a2,alpha2,theta2)
    A3 = matriz_homogenea(d3,a3,alpha3,theta3)
    A4 = matriz_homogenea(d4,a4,alpha4,theta4)
    A5 = matriz_homogenea(d5,a5,alpha5,theta5)
    A6 = matriz_homogenea(d6,a6,alpha6,theta6)
    A7 = matriz_homogenea(d7,a7,alpha7,theta7)
    #Calculando os pontos de interesse no sistema Global
    T1 = A1
    T2 = T1@A2
    T3 = T2@A3
    T4 = T3@A4
    T5 = T4@A5
    T6 = T5@A6
    T7 = T6@A7
    p_7 = np.array([[0,0,L,1]]).T
    p_0 = T7@p_7
    return p_0[0:3] , T7[0:3,0:3]

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
direcoes = [1,2,1,2,1,2,3] #1 - z, 2 - x, 3 -y  direcao iniciail do vetor de atuacao da junta
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])

posicaod, orientd = random_pose()
destino = posicaod
#destino = vetor([0.004,0.039,0.446])
print('destino:',np.round(destino[:,0],3))
dif_angular = [0,0,0,0,0,pi/2,0] #diferenca angular em relacao a junta anterior
b = np.array([0.05,0.075,0.075,0.0725,0.0725,0.075,0.075]) #tamanho dos elos
D = np.zeros([3,n]) #Vetores de atuação iniciais de cada junta
for cont in range(n):
    if(direcoes[cont] == 1): #Se z
        D[:,cont] = z[:,0]
    elif(direcoes[cont] == 2):#Se x
        D[:,cont] = x[:,0]
    elif(direcoes[cont] == 3):#Se y
        D[:,cont] = y[:,0] 

qlim = [2.6179,1.5358,2.6179,1.6144,2.6179,1.8413,1.7889]  
#for i in range(np.size(q)):
#    q[i] = random.uniform(-qlim[i],qlim[i])
q = np.zeros([7,1])

#px = np.zeros([1,8])  #gerando p manualmente
#py = np.zeros([1,8])
#pz = np.array([[0.075,0.125,0.2,0.275,0.3475,0.42,0.495,0.57]])
#p = np.zeros([3,n+1]) #posicao inicial das juntas
#p[0,:] = px
#p[1,:] = py
#p[2,:] = pz
p = Cinematica_Direta(q)
pcte = p[:,0].copy()
pl = p.copy()
Dl = D.copy()
erro = distancia(p[:,n],destino,3)
print('erro inicial:', erro)
K = 100 #número máximo de iterações
k = 0
erromin = 10**-3

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
                    v2 = v2/norm(v2)
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

        elif(i == 2 or i == 4): #Se a junta for pivot (3 e 5)
            pl[:,i] = iteracao_Fabrik(p[:,i+1],pl[:,i-1],b[i-1],Dl[:,i-1])[:,0]
            v1 = pl[:,i] - pl[:,i-1]
            v1 = v1/norm(v1)
            Dl[:,i] = v1
        elif(i == 0): #Primeira junta eixo de atuação e posição são fixos
            pl[:,0] = pcte #[0,0,0.075]
    pl[:,7]  = iteracao_Fabrik(p[:,7],pl[:,6],b[6],Dl[:,6])[:,0]
    p = pl.copy()
    D = Dl.copy()
    erro = distancia(p[:,n],destino,3)
    print('erro: ', erro)
    k = k +1


#Conversão da solução gráfica em um vetor de ângulo
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])
q = np.zeros([7,1])
#v é o vetor ortogonal ao vetor de refência para o cálculo dos ângulos 


for i in range(7):
    D[:,i] = D[:,i]/norm(D[:,i])

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
print('q: ', q)

#atualiza os angulos do manipulador no Rviz
while not rospy.is_shutdown():
        
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0],0,0]
        pub.publish(hello_str)
        rate.sleep()  
