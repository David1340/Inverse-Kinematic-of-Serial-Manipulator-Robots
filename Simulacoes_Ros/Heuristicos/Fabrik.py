# %%
from math import cos, sin, sqrt, pi,acos
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
    #r = distancia(p0[:,0],proj[:,0],3)
    delta = d/r
    pnew = (1- delta)*p0 + delta*proj
    return pnew

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

def acosr(x):
    return acos(np.round(x,5))

n = 7 #numero de juntas
direcoes = [1,2,1,2,1,2,3] #1 - z, 2 - x, 3 -y  direcao iniciail do vetor de atuacao da junta
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])
#destino = 20*vetor([1,0.7,1])
destino = vetor([0.4,0.4,0.4])
dif_angular = [0,0,0,0,0,pi/2,0] #diferenca angular em relacao a junta anterior
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
#pz = 5*np.array([[0,2,3,5,6,8,9,11]])
pz = np.array([[0.2,0.1,0.2,0.1,0.2,0.1,0.2,0.2]])
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
    #Forward
    for i in range(n-1,0,-1):
        if(i == 6):
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
            pl[:,i] = iteracao_Fabrik(p[:,i],pl[:,i+1],b[i],Dl[:,i])[:,0] #junta 6
        elif(i == 1 or i == 3 or i == 5): #Se for junta Hinge
            if(i == 1 or i == 3):#Se a junta prev for pivot
                pl[:,i] = pl[:,i+1] - Dl[:,i+1]*b[i]
                paux = iteracao_Fabrik(p[:,i-1],pl[:,i],b[i],Dl[:,i+1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                v2 = vetor(Dl[:,i+2])
                th = np.real(acos(v1.T@v2))
                Dl[:,i] = rotationar_vetor(v2,vetor(Dl[:,i+1]),(pi/2) - th)[:,0] 
                Dl[:,i] = Dl[:,i]/norm(D[:,i])
            else: #Se for a junta prev for hinge
                pl[:,i] = iteracao_Fabrik(p[:,i-1],pl[:,i+1],b[i],Dl[:,i+1])[:,0]
                v1 = vetor(pl[:,i] - pl[:,i+1])
                v1 = v1/norm(v1) #esta certo
                Dl[:,i] = rotationar_vetor(vetor(Dl[:,i+1]),v1,pi/2)[:,0] #esta errado
                Dl[:,i] = Dl[:,i]/norm(D[:,i])
                
        elif(i == 2 or i == 4):
            pl[:,i] = iteracao_Fabrik(p[:,i-1],pl[:,i+1],b[i],Dl[:,i+1])[:,0]
            v1 = pl[:,i] - pl[:,i+1]
            v1 = v1/norm(v1)
            Dl[:,i] = -v1
    #if(k == 1): break
    Dl[:,0] = [0,0,1]
    pl[:,0] = pl[:,1] - b[0]*Dl[:,0] 
    D = Dl
    p = pl
    #até aqui está certo
    #Backward
    for i in range(0,n):
        if(i == 1 or i == 3 or i == 5 or i == 6): #Se for junta Hinge
            if(i == 1 or i == 3 or i == 5):#Se a junta prev for pivot
                pl[:,i] = pl[:,i-1] + Dl[:,i-1]*b[i-1]
                paux = iteracao_Fabrik(p[:,i+1],pl[:,i],b[i],Dl[:,i-1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                if(i != 1): v2 = vetor(Dl[:,i-2])
                else: v2 = np.array([[1,0,0]]).T
                th = np.real(acos(v1.T@v2))
                Dl[:,i] = rotationar_vetor(v2,vetor(Dl[:,i-1]),(pi/2) - th)[:,0] 
                Dl[:,i] = Dl[:,i]/norm(D[:,i])
            else: #Se i = 6
                pl[:,i] = iteracao_Fabrik(p[:,i+1],pl[:,i-1],b[i-1],Dl[:,i-1])[:,0]
                paux = iteracao_Fabrik(p[:,i+1],pl[:,i],b[i],Dl[:,i-1])[:,0]
                v1 = vetor(paux - pl[:,i])
                v1 = v1/norm(v1)
                Dl[:,i] = rotationar_vetor(vetor(Dl[:,i-1]),v1,-pi/2)[:,0]
                Dl[:,i] = Dl[:,i]/norm(D[:,i])
        elif(i == 2 or i == 4):
            pl[:,i] = iteracao_Fabrik(p[:,i+1],pl[:,i-1],b[i-1],Dl[:,i-1])[:,0]
            v1 = pl[:,i] - pl[:,i-1]
            v1 = v1/norm(v1)
            Dl[:,i] = v1
        elif(i == 0):
            pl[:,0] = [0,0,0]
    pl[:,7]  = iteracao_Fabrik(p[:,7],pl[:,6],b[6],Dl[:,6])[:,0]
    p = pl
    D = Dl
    erro = distancia(p[:,n],destino,3)
    k = k +1

print('erro: ', erro)
print('k: ',k)  
print(p,'\n\n\n')
print(D,'\n')

#Conversão da solução gráfica em um vetor de ângulo
x  = vetor([1,0,0])
y  = vetor([0,1,0])
z  = vetor([0,0,1])
q = np.zeros([7,1])
#v é o vetor ortogonal ao vetor de refência para o cálculo dos ângulps 
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
v = rotationar_vetor(v_aux2,vetor(D[:,5]),pi/2)
q[6] = acosr(v_aux.T@v_aux2)
if(v_aux.T@v < 0): q[6] = -q[6]

print(q*(180/pi))