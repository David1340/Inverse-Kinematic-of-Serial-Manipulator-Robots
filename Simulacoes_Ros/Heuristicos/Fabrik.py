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

n = np.array([[0,0,1]]).T
p = np.array([[5,4,3]]).T
p0 = np.array([[1,1,6]]).T

x = iteracao_Fabrik(p,p0,1,n)
print(x)


