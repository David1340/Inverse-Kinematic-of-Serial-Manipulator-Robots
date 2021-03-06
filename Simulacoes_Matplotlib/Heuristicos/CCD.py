import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from math import cos, sin, sqrt, pi, acos
import numpy as np
import random 

def multiplicacao_quaternios(q,q2):
    resultado = np.zeros([4,1])
    resultado[0,0] = q[0,0]*q2[0,0] -q[1,0]*q2[1,0] -q[2,0]*q2[2,0] -q[3,0]*q2[3,0] 
    resultado[1,0] = q[0,0]*q2[1,0] +q[1,0]*q2[0,0] +q[2,0]*q2[3,0] -q[3,0]*q2[2,0] 
    resultado[2,0] = q[0,0]*q2[2,0] -q[1,0]*q2[3,0] +q[2,0]*q2[0,0] +q[3,0]*q2[1,0] 
    resultado[3,0] = q[0,0]*q2[3,0] +q[1,0]*q2[2,0] -q[2,0]*q2[1,0] +q[3,0]*q2[0,0] 
    #print(resultado)
    return resultado

def rotationar_vetor(p,v,th):
    #gira p em torno de v em th rad
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

def acosr(x):
    return acos(np.round(x,10))

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

def norm(v):
    return sqrt(v[[0]]**2 + v[[1]]**2 + v[[2]]**2)

def vetor(v):
    #criar um vetor linha a partir de uma lista
    return np.array([[v[0],v[1],v[2]]]).T

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

#Retorna a Matriz de transformacao Homogeneadados  usando como entrada os parametros de DH
def matriz_homogenea(d,a,alfa,theta):
    L1 = np.array([cos(theta),-sin(theta)*cos(alfa),sin(theta)*sin(alfa),a*cos(theta)])
    L2 = np.array([sin(theta),cos(theta)*cos(alfa),-cos(theta)*sin(alfa),a*sin(theta)])
    L3 = np.array([0,sin(alfa),cos(alfa), d])
    L4 = np.array([0.0,0.0,0.0,1.0])
    A = np.array([L1,L2,L3,L4])
    return A

#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2      
    return sqrt(d)

def plot_junta_revolucao(A,p,c):
    #A matriz de Rotação, p origem da junta no seu sistema de coordenadas
    r = 0.025
    h = 0.05
    theta = np.arange(0,2*pi + 0.12,0.8) #theta de 0 a 2pi com passos de 0.1
    if(c == 'z'):  
        z = np.linspace(0,h,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        z, theta = np.meshgrid(z, theta) #transforma em vetores 2D por causa do plot de superficie
        #[x,y,z].T = A@([x,y,z].T + p)
        x = (r*np.cos(theta) + p[0,0])*A[0,0] + (r*np.sin(theta) + p[1,0])*A[0,1] + \
         (z + p[2,0])*A[0,2] + np.ones_like(z)*A[0,3]
        y = (r*np.cos(theta) + p[0,0])*A[1,0] + (r*np.sin(theta) + p[1,0])*A[1,1] + \
         (z + p[2,0])*A[1,2] + np.ones_like(z)*A[1,3]
        z = (r*np.cos(theta) + p[0,0])*A[2,0] + (r*np.sin(theta) + p[1,0])*A[2,1] + \
         (z + p[2,0])*A[2,2] + np.ones_like(z)*A[2,3]
        ax.plot_surface(x, y, z,color = 'blue', alpha = 1)
        
    elif(c == 'y'):
        y = np.linspace(-h,h,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        y, theta = np.meshgrid(y, theta) #transforma em vetores 2D por causa do plot de superficie
        #[x,y,z].T = A@([x,y,z].T + p)
        x = (r*np.cos(theta) + p[0,0])*A[0,0] + (r*np.sin(theta) + p[2,0])*A[0,2] + \
         (y + p[1,0])*A[0,1] + np.ones_like(y)*A[0,3]
        z = (r*np.cos(theta) + p[0,0])*A[2,0] + (r*np.sin(theta) + p[2,0])*A[2,2] + \
         (y + p[1,0])*A[2,1] + np.ones_like(y)*A[2,3]
        y = (r*np.cos(theta) + p[0,0])*A[1,0] + (r*np.sin(theta) + p[2,0])*A[1,2] + \
         (y + p[1,0])*A[1,1] + np.ones_like(y)*A[1,3]
        ax.plot_surface(x, y, z,color = 'blue', alpha = 1)
        
    elif(c == 'x'):
        x = np.linspace(-h,h,np.size(theta))#z de 0.1 a 0.1 com o numero de elementos iguais ao de theta
        x, theta = np.meshgrid(x, theta) #transforma em vetores 2D por causa do plot de superficie
        #[x,y,z].T = A@([x,y,z].T + p)
        y = (r*np.cos(theta) + p[2,0])*A[1,2] + (r*np.sin(theta) + p[1,0])*A[1,1] + \
         (x + p[0,0])*A[1,0] + np.ones_like(x)*A[1,3]
        z = (r*np.cos(theta) + p[2,0])*A[2,2] + (r*np.sin(theta) + p[1,0])*A[2,1] + \
         (x + p[0,0])*A[2,0] + np.ones_like(x)*A[2,3]
        x = (r*np.cos(theta) + p[2,0])*A[0,2] + (r*np.sin(theta) + p[1,0])*A[0,1] + \
         (x + p[0,0])*A[0,0] + np.ones_like(x)*A[0,3]
        ax.plot_surface(x, y, z,color = 'blue', alpha = 1)

#Função que plota o manipulador, quando recebe os pontos de interesse
def plot(q,t):
    #Comprimento dos elos do manipulador
    b1 = 0.2 #20 cm
    b2 = 0.1
    b3 = 0.2 
    b4 = 0.1
    b5 = 0.2
    b6 = 0.1
    b7 = 0.2
    
    w = 0.05 #largura do efetuador
    h = 0.05 #altura do efetuador
    L = 0.2 #comprimento do elo que liga a junta 7 ao efetuador

    #Pontos de interesse
    p = np.array([[0,0,0,1]]).T #Base
    p1_1 = np.array([[0,-b2,0,1]]).T #junta1
    p2_2 = p #junta2
    p3_3 = np.array([[0,-b4,0,1]]).T #junta3
    p4_4 = p #junta4
    p5_5 = np.array([[0,-b6,0,1]]).T #junta5
    p6_6 = np.array([[-b7,0,0,1]]).T #junta6
    p7_7 = p #junta7
    p8_7 = np.array([[0,0,L,1]]).T #contato de atuação do efetuador
    
    #O efetuador será um u, para plotarmos um precisamos de 4 pontos
    #topo esquerdo do u
    e1_7 = np.array([[0,-w/2,L + h,1]]).T
    #base esquerda do u
    e2_7 = np.array([[0,-w/2,L,1]]).T
    #base direita do u
    e3_7 = np.array([[0,w/2,L,1]]).T
    #topo direito do u
    e4_7 = np.array([[0,w/2,L+ h,1]]).T    
    ### parametros de DH
    d1 = b1 + b2
    d2 = 0
    d3 = b3 + b4
    d4 = 0 
    d5 = b5 + b6
    d6 = 0
    d7 = 0
    a1 = 0
    a2 = 0
    a3 = 0
    a4 = 0
    a5 = 0
    a6 = b7
    a7 = 0
    alpha1 = pi/2
    alpha2 = -pi/2
    alpha3 = pi/2
    alpha4 = -pi/2
    alpha5 = pi/2
    alpha6 = pi/2
    alpha7 = pi/2
    theta1 = pi/2 + q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    theta5 = q[4]
    theta6 = pi/2 + q[5]
    theta7 = pi/2 + q[6]
    #Calculando as matrizes homogêneas
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
    p1_0 = T1@p1_1
    p2_0 = T2@p2_2
    p3_0 = T3@p3_3
    p4_0 = T4@p4_4
    p5_0 = T5@p5_5
    p6_0 = T6@p6_6
    p7_0 = T7@p7_7
    p8_0 = T7@p8_7
    e1_0 = T7@e1_7
    e2_0 = T7@e2_7
    e3_0 = T7@e3_7
    e4_0 = T7@e4_7
    
    #Plotando Elos 
    ax.clear()   
    #Plotando Elos    
    plt.plot([p[0,0],p1_0[0,0]],[p[1,0],p1_0[1,0]],[p[2,0],p1_0[2,0]],'blue')
    plt.plot([p1_0[0,0],p2_0[0,0]],[p1_0[1,0],p2_0[1,0]],[p1_0[2,0],p2_0[2,0]],'blue')
    plt.plot([p2_0[0,0],p3_0[0,0]],[p2_0[1,0],p3_0[1,0]],[p2_0[2,0],p3_0[2,0]],'blue')
    plt.plot([p3_0[0,0],p4_0[0,0]],[p3_0[1,0],p4_0[1,0]],[p3_0[2,0],p4_0[2,0]],'blue')
    plt.plot([p4_0[0,0],p5_0[0,0]],[p4_0[1,0],p5_0[1,0]],[p4_0[2,0],p5_0[2,0]],'blue')
    plt.plot([p5_0[0,0],p6_0[0,0]],[p5_0[1,0],p6_0[1,0]],[p5_0[2,0],p6_0[2,0]],'blue')
    plt.plot([p6_0[0,0],p7_0[0,0]],[p6_0[1,0],p7_0[1,0]],[p6_0[2,0],p7_0[2,0]],'blue')
    
    #Plotando efetuador
    plt.plot([e1_0[0,0],e2_0[0,0],e3_0[0,0],e4_0[0,0],e3_0[0,0],p8_0[0,0],p7_0[0,0]]\
             ,[e1_0[1,0],e2_0[1,0],e3_0[1,0],e4_0[1,0],e3_0[1,0],p8_0[1,0],p7_0[1,0]]\
             ,[e1_0[2,0],e2_0[2,0],e3_0[2,0],e4_0[2,0],e3_0[2,0],p8_0[2,0],p7_0[2,0]],'blue')
    
    #Plotando objetivo    
    ax.scatter(t[0,0],t[1,0],t[2,0],'red')
    
    #Plotando Juntas e base    
    plot_junta_revolucao(np.eye(4),p,'z')
    plot_junta_revolucao(T1,p1_1,'y')
    plot_junta_revolucao(T2,p2_2,'y')
    plot_junta_revolucao(T3,p3_3,'y')
    plot_junta_revolucao(T4,p4_4,'y')
    plot_junta_revolucao(T5,p5_5,'y')
    plot_junta_revolucao(T6,p6_6,'y')
    plot_junta_revolucao(T7,p7_7,'y')

    ax.set_xlabel('Eixo x(m)')
    ax.set_ylabel('Eixo y(m)')
    ax.set_zlabel('Eixo z(m)')

    #titulo
    plt.title('Pioneer 7DOF')
    ax.set_xlim3d(-0.8,0.8)
    ax.set_ylim3d(-0.8,0.8)
    ax.set_zlim3d(0,0.8)
    fig.canvas.draw() #mostra o plot
    plt.pause(0.05)

def Cinematica_Direta(q):
    #Comprimento dos elos do manipulador
    b1 = 0.2 #20 cm
    b2 = 0.1
    b3 = 0.2 
    b4 = 0.1
    b5 = 0.2
    b6 = 0.1
    b7 = 0.2
    
    w = 0.05 #largura do efetuador
    h = 0.05 #altura do efetuador
    L = 0.2 #comprimento do elo que liga a junta 7 ao efetuador

    #Pontos de interesse
    p = np.array([[0,0,0,1]]).T #Base
    p1_1 = np.array([[0,-b2,0,1]]).T #junta1
    p2_2 = p #junta2
    p3_3 = np.array([[0,-b4,0,1]]).T #junta3
    p4_4 = p #junta4
    p5_5 = np.array([[0,-b6,0,1]]).T #junta5
    p6_6 = np.array([[-b7,0,0,1]]).T #junta6
    p7_7 = p #junta7
    p8_7 = np.array([[0,0,L,1]]).T #contato de atuação do efetuador

    #O efetuador será um u, para plotarmos um precisamos de 4 pontos
    #topo esquerdo do u
    e1_7 = np.array([[0,-w/2,L + h,1]]).T
    #base esquerda do u
    e2_7 = np.array([[0,-w/2,L,1]]).T
    #base direita do u
    e3_7 = np.array([[0,w/2,L,1]]).T
    #topo direito do u
    e4_7 = np.array([[0,w/2,L+ h,1]]).T    
    ### parametros de DH
    d1 = b1 + b2
    d2 = 0
    d3 = b3 + b4
    d4 = 0 
    d5 = b5 + b6
    d6 = 0
    d7 = 0
    a1 = 0
    a2 = 0
    a3 = 0
    a4 = 0
    a5 = 0
    a6 = b7
    a7 = 0
    alpha1 = pi/2
    alpha2 = -pi/2
    alpha3 = pi/2
    alpha4 = -pi/2
    alpha5 = pi/2
    alpha6 = pi/2
    alpha7 = pi/2
    theta1 = pi/2 + q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    theta5 = q[4]
    theta6 = pi/2 + q[5]
    theta7 = pi/2 + q[6]
    #Calculando as matrizes homogêneas
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
    p1_0 = T1@p1_1
    p2_0 = T2@p2_2
    p3_0 = T3@p3_3
    p4_0 = T4@p4_4
    p5_0 = T5@p5_5
    p6_0 = T6@p6_6
    p7_0 = T7@p7_7
    p8_0 = T7@p8_7
    e1_0 = T7@e1_7
    e2_0 = T7@e2_7
    e3_0 = T7@e3_7
    e4_0 = T7@e4_7
    
    pontos = np.array([p1_0[0:3,0],p2_0[0:3,0],p3_0[0:3,0],p4_0[0:3,0]\
                    ,p5_0[0:3,0],p6_0[0:3,0],p7_0[0:3,0],p8_0[0:3,0]]).T
    return pontos

#configurando o plot
plt.ion()
fig = plt.figure()
ax =  fig.add_subplot(111, projection = '3d')
#destino = np.array([[0.2,0.5,0.6]]).T
destino = np.array([[0.5*random.random(),0.5*random.random(),0.6*random.random()]]).T
q = np.array([[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]).T
n = 7 #número de juntas
#Comprimento dos elos do manipulador
b1 = 0.2 #20 cm
b2 = 0.1
b3 = 0.2 
b4 = 0.1
b5 = 0.2
b6 = 0.1
b7 = 0.2
L = 0.2 #comprimento do elo que liga a junta 7 ao efetuador

### parametros de DH constantes
d1 = b1 + b2
d2 = 0
d3 = b3 + b4
d4 = 0 
d5 = b5 + b6
d6 = 0
d7 = 0
a1 = 0
a2 = 0
a3 = 0
a4 = 0
a5 = 0
a6 = b7
a7 = 0
alpha1 = pi/2
alpha2 = -pi/2
alpha3 = pi/2
alpha4 = -pi/2
alpha5 = pi/2
alpha6 = pi/2
alpha7 = pi/2



for k in range(25):
    # parametros de DH variáveis
    theta1 = pi/2 + q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    theta5 = q[4]
    theta6 = pi/2 + q[5]
    theta7 = pi/2 + q[6]

    #Calculando as matrizes homogêneas
    A1 = matriz_homogenea(d1,a1,alpha1,theta1)
    A2 = matriz_homogenea(d2,a2,alpha2,theta2)
    A3 = matriz_homogenea(d3,a3,alpha3,theta3)
    A4 = matriz_homogenea(d4,a4,alpha4,theta4)
    A5 = matriz_homogenea(d5,a5,alpha5,theta5)
    A6 = matriz_homogenea(d6,a6,alpha6,theta6)
    A7 = matriz_homogenea(d7,a7,alpha7,theta7)

    #Calculando os pontos de interesse e vetores no sistema Global
    T1 = A1
    T2 = T1@A2
    T3 = T2@A3
    T4 = T3@A4
    T5 = T4@A5
    T6 = T5@A6
    T7 = T6@A7

    #vetor
    v1 = vetor(T1[0:3,1])
    v2 = vetor(T2[0:3,1])
    v3 = vetor(T3[0:3,1])
    v4 = vetor(T4[0:3,1])
    v5 = vetor(T5[0:3,1]) #y
    v6 = vetor(T6[0:3,1])
    v7 = vetor(T7[0:3,1]) #y
    Vy = np.array([T1[0:3,1],T2[0:3,1],T3[0:3,1],T4[0:3,1],T5[0:3,1],T6[0:3,1],T7[0:3,1]]).T
    pontos = Cinematica_Direta(q) 
    erro = distancia(pontos[:,7],destino,3) 
    for i in range(n-1,-1,-1):
        pontos = Cinematica_Direta(q) 
        proj = projecao_ponto_plano(vetor(Vy[:,i]),pontos[:,i],destino[:])
        va = proj - vetor(pontos[:,i])
        va = va/norm(va)
        proj = projecao_ponto_plano(vetor(Vy[:,i]),pontos[:,i],vetor(pontos[:,7]))
        vb = proj - vetor(pontos[:,i])
        vb = vb/norm(vb)
        th = acosr(va.T@vb)
        j = i + 1
        if(j == 4 or j ==2):
            v = rotationar_vetor(va,vetor(Vy[:,i]),pi/2)
        else:
           v = rotationar_vetor(va,vetor(Vy[:,i]),-pi/2) 
        if(vb.T@v < 0): th = -th
        q[i,0] = q[i,0] + th
        pontos = Cinematica_Direta(q)
        erro_anterior = erro
        erro = distancia(pontos[:,7],destino,3) 
        if(erro_anterior < erro):
            print('Deu ruim',j)
            break

    pontos = Cinematica_Direta(q)
    erro = distancia(pontos[:,7],destino,3) 
    print(erro,'\n','fim da interacao:',k,'\n')
    if(erro < 10**-5):
        break

#plot(q,destino)
#plt.pause(2)
