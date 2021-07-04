#Autor David Oliveira
#Estudante de Engenharia Eletrônica da Universidade Federal de Sergipe-UFS
#Membro do Grupo de Pesquisa em Robotica da UFS-GPRUFS
#Implementação do Forward And Backward Reaching Inverse Kinematic 
#para encontrar uma configuação q dada uma posição (x,y,z) no espaço para o Pioneer 7DOF
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

#Calcula a norma de um vetor 
def norm(v):
    return sqrt(v[[0]]**2 + v[[1]]**2 + v[[2]]**2)

#Projeta um ponto em um plano
def projecao_ponto_plano(normal,p0,p):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    #constante d da equação do 
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
    #print(resultado)
    return resultado

#gira p em torno de v em th rad
def rotationar_vetor(p,v,th):
    
    a = cos(th/2)
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

destino = vetor([0.5*random.random(),0.5*random.random(),0.5*random.random()])
print('destino:',destino[:,0])
dif_angular = [0,0,0,0,0,pi/2,0] #diferenca angular em relacao a junta anterior
b = np.array([0.1,0.2,0.1,0.2,0.1,0.2,0.2]) #tamanho dos elos
D = np.zeros([3,n]) #Vetores de atuação iniciais de cada junta
for cont in range(n):
    if(direcoes[cont] == 1): #Se z
        D[:,cont] = z[:,0]
    elif(direcoes[cont] == 2):#Se x
        D[:,cont] = x[:,0]
    elif(direcoes[cont] == 3):#Se y
        D[:,cont] = y[:,0] 

px = np.zeros([1,8]) 
py = np.zeros([1,8])
#pz = 5*np.array([[0,2,3,5,6,8,9,11]])
pz = np.array([[0.2,0.3,0.5,0.6,0.8,0.9,1.1,1.3]])
p = np.zeros([3,n+1]) #posicao inicial das juntas
p[0,:] = px
p[1,:] = py
p[2,:] = pz
pcte = p[:,0].copy()
pl = p
Dl = D
erro = distancia(p[:,n],destino,3)
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

            if(norm(naux) > 0.1):
                Dl[:,i] = naux[:,0]/norm(naux)
            else:
                Dl[:,i] = D[:,i]

            pl[:,i] = iteracao_Fabrik(p[:,i],pl[:,i+1],b[i],Dl[:,i])[:,0] 

        elif(i == 1 or i == 3 or i == 5): #Se for junta Hinge (2,4 e 6)

            if(i == 1 or i == 3):#Se a junta prev for pivot (2 e 4)
                pl[:,i] = pl[:,i+1] - Dl[:,i+1]*b[i]
                paux = iteracao_Fabrik(p[:,i-1],pl[:,i],b[i],Dl[:,i+1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                v2 = vetor(Dl[:,i+2])
                th = np.real(acosr(v1.T@v2))
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
    D = Dl
    p = pl

    #Backward
    for i in range(0,n):
        if(i == 1 or i == 3 or i == 5 or i == 6): #Se for junta Hinge
            if(i == 1 or i == 3 or i == 5):#Se a junta prev for pivot (2,4 e 6)
                pl[:,i] = pl[:,i-1] + Dl[:,i-1]*b[i-1]
                paux = iteracao_Fabrik(p[:,i+1],pl[:,i],b[i],Dl[:,i-1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                if(i != 1): v2 = vetor(Dl[:,i-2])
                else: v2 = np.array([[1,0,0]]).T
                th = np.real(acosr(v1.T@v2))
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
            pl[:,0] = pcte #[0,0,0.2]
    pl[:,7]  = iteracao_Fabrik(p[:,7],pl[:,6],b[6],Dl[:,6])[:,0]
    p = pl
    D = Dl
    erro = distancia(p[:,n],destino,3)
    k = k +1


#Conversão da solução gráfica em um vetor de ângulo
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])
q = np.zeros([7,1])
#v é o vetor ortogonal ao vetor de refência para o cálculo dos ângulos 

v = rotationar_vetor(x,vetor(D[:,0]),pi/2)
q[0] = acosr(D[:,1].T@x)
if(D[:,1].T@v < 0): q[0] = -q[0]

v = rotationar_vetor(z,vetor(D[:,1]),pi/2)
q[1] = acosr(D[:,2].T@z)
if(D[:,2].T@v < 0): q[1] = -q[1]

v = rotationar_vetor(vetor(D[:,1]),vetor(D[:,2]),pi/2)
q[2] = acosr(D[:,3].T@D[:,1])
if(D[:,3].T@v < 0): q[2] = -q[2]

v = rotationar_vetor(vetor(D[:,2]),vetor(D[:,3]),pi/2)
q[3] = acosr(D[:,4].T@D[:,2])
if(D[:,4].T@v < 0): q[3] = -q[3]

v = rotationar_vetor(vetor(D[:,3]),vetor(D[:,4]),pi/2)
q[4] = acosr(D[:,5].T@D[:,3])
if(D[:,5].T@v < 0): q[4] = -q[4]

v_aux = vetor(p[:,6] - p[:,5])
v_aux = v_aux/norm(v_aux)
v = rotationar_vetor(vetor(D[:,4]),vetor(D[:,5]),pi/2)
q[5] = acosr(v_aux.T@D[:,4])
if(v_aux.T@v < 0): q[5] = -q[5]

v_aux = vetor(p[:,7] - p[:,6])
v_aux = v_aux/norm(v_aux)
v_aux2 = vetor(p[:,6] - p[:,5]) 
v_aux2 = v_aux2/norm(v_aux2)
v = rotationar_vetor(v_aux2,vetor(D[:,6]),pi/2)
q[6] = acosr(v_aux.T@v_aux2)
if(v_aux.T@v < 0): q[6] = -q[6]

print('erro: ', erro)
print('k: ',k)  
print('q: ', q)

#atualiaza os angulos do manipulador no Rviz
while not rospy.is_shutdown():
        
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [q[0,0],q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0],0,0]
        pub.publish(hello_str)
        rate.sleep()  
