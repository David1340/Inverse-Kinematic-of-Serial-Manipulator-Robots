from math import cos, sin, sqrt, pi, atan2
import numpy as np
import random 

#Calcula a distancia Euclidiana entre dois pontos no R^n
def distancia(a,b,n):
    d = 0
    for i in range(n):
        d = d + (a[i] - b[i])**2      
    return sqrt(d)

def projecao_ponto_plano(normal,p0,p):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    #constante d da equação do plano
    d = -normal.T@p0 #produto escala 
    #distancia do ponto ao plano
    alpha = (-d - normal.T@p)/(normal[0,0]**2 +  normal[1,0]**2 + normal[2,0]**2)
    ponto_projetado = p + alpha*normal
    return ponto_projetado

def iteracao_Fabrik(p,p0,d,normal):
    #normal -> vetor normal ao plano
    #p0 -> ponto pertecente ao plano
    #p -> ponto a ser projetado
    proj = projecao_ponto_plano(normal,p0,p)
    r = distancia(p0,proj,3)
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

n = np.array([[0,0,1]]).T
p = np.array([[5,4,3]]).T
p0 = np.array([[1,1,6]]).T

x = iteracao_Fabrik(p,p0,1,n)
print(x)


