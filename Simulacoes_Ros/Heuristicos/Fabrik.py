from math import cos, sin, sqrt, pi, atan2
import numpy as np
import random 


def vetor(v):
    #criar um vetor linha a partir de uma lista
    return np.array([[v[0],v[1],v[2]]]).T
#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2     
    return sqrt(d)

def norm(v):
    return sqrt(v[[0]]**2 + v[[1]]**2 + v[[2]]**2)

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

def iteracao_Fabrik(p,p0,d,normal):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    normal = vetor(normal)
    p0 = vetor(p0)
    p = vetor(p)
    proj = projecao_ponto_plano(normal,p0,p)
    r = distancia(p0,proj,3)
    r = distancia(p0[:,0],proj[:,0],3)
    delta = d/r
    pnew = (1- delta)*p0 + delta*proj
    return pnew

def multiplicacao_quaternios(q,q2):
    resultado = np.array([[0,0,0,0]])
    resultado[0,0] = q[0,0]*q2[0,0] -q[1,0]*q2[1,0] -q[2,0]*q2[2,0] -q[3,0]*q2[3,0] 
    resultado[1,0] = q[0,0]*q2[1,0] -q[1,0]*q2[0,0] -q[2,0]*q2[3,0] -q[3,0]*q2[2,0] 
    resultado[2,0] = q[0,0]*q2[2,0] -q[1,0]*q2[3,0] -q[2,0]*q2[0,0] -q[3,0]*q2[1,0] 
    resultado[3,0] = q[0,0]*q2[3,0] -q[1,0]*q2[2,0] -q[2,0]*q2[1,0] -q[3,0]*q2[0,0] 
    return resultado

def rotationar_vetor(p,v,th):
    #gira p em torno de v em th rad
    a = cos(th/2)
    b = v[[0,0]]*sin(th/2)
    c = v[[1,0]]*sin(th/2)
    d = v[[2,0]]*sin(th/2)
    p_aumentado = np.zeros([4,1])
    p_aumentado[0:3,0] = p[:,0]
    h = np.array[[a,b,c,d]]
    hx = np.array[[a,-b,-c,-d]]
    p_r = multiplicacao_quaternios(h,p)
    q_r = multiplicacao_quaternios(p_r,hx)
    return q_r[0:3]

#matriz_antissimetrica
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

n = 7 #numero de juntas
direcoes = [1,2,1,2,1,2,3] #1 - z, 2 - x, 3 -y  direcao iniciail do vetor de atuacao da junta
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])
destino = vetor([20,20,20])
dif_angular = np.array([[0,0,0,0,0,pi/2,0]]) #diferenca angular em relacao a junta anterior
b = 5*np.array([2,1,2,1,2,1,2]) #comprimento dos elos 
D = np.zeros([3,n])
for cont in range(n):
    if(direcoes[cont] == 1): #Se z
        D[:,cont] = z[:,0]
    elif(direcoes[cont] == 2):#Se x
        D[:,cont] = x[:,0]
    elif(direcoes[cont] == 3):#Se y
        D[:,cont] = y[:,0] 

px = np.zeros([1,8])
py = np.zeros([1,8])
pz = 5*np.array([[0,2,3,5,6,8,9,11]])
p = np.zeros([3,n+1]) #posicao inicial das juntas
p[0,:] = px
p[1,:] = py
p[2,:] = pz
pl = p
Dl = D
erro = distancia(p[:,n],destino,3)
K = 100
k = 0
erromin = 10**-3
while(erro > erromin and k < K):
    for i in range(n-1,-1,-1):
        if(i == 6):
            if(k == 0):
                pl[:,i+1] = destino[:,0]#coloca o efetuador no destino 
                v1 = vetor(pl[:,i+1] - p[:,i])#p6 -> p7' (efetuador)
                v1 = v1/norm(v1)
                v2 = vetor(p[:,i-1] - p[:,i])#p6 -> p5
                Dl_cte = S(v1)@v2 #produto vetorial
                Dl[:,i] = Dl_cte[:,0]
                pl[:,i] = iteracao_Fabrik(p[:,i],pl[:,i+1],b[i],Dl[:,i])[:,0] #junta 6
            else:
                Dl[:,i] = Dl_cte[:,0]
                pl[:,i] = iteracao_Fabrik(p[:,i],pl[:,i+1],b[i],Dl[:,i])[:,0] #junta 6
        elif(i == 1 or i == 3 or i == 5): #Se for junta Hinge
            if(i == 1 or i == 3):#Se a junta prev for pivot
                #calculo da posicao da junta pi usando o Fabrik padrao
                v = vetor(pl[:,i+1] - pl[:,i+2]) #pl(i+2) -> pl(i+1) 
                v = v/norm(v)
                pl[:,i] = pl[:,i+1] + v*b(i)
            else: #Se for a junta prev for hinge
                pl[:,i] = iteracao_Fabrik(p[:,i-1],pl[:,i+1],b[i],Dl[:,i+1])[:,0]
            #Preciso investigar essa parte, ainda não compreendi bem
            paux = iteracao_Fabrik(p[:,i-1],pl[:,i+1],b[i],Dl[:,i+1])
            v1 = vetor(paux - pl[:,i+1])
        elif(i == 0 or i == 2 or i == 4):
            break
    break



n = np.array([[0,0,1]]).T
p = np.array([[5,4,3]]).T
p0 = np.array([[1,1,6]]).T

#x = iteracao_Fabrik(p,p0,1,n)
print('erro: ',erro)


