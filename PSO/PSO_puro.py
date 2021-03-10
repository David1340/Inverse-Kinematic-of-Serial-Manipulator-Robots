import random
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt 
M = 10 #quantidade de particulas
L = 10 #largura do quadrado de pesquisa
q = np.array([np.zeros(M),np.zeros(M)])
k = 100
c1 = 1.4047
c2 =1.494

    

def distancia(a,b):
    d = sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    return d

def qbest(q,o):
    d = []
    for i in range(M):
        d.append(distancia([q[0,i],q[1,i]],o))
    maximo = min(d)
    for i in range(M):
       if(maximo == d[i]):
            p_maximo = i
    return p_maximo

print(q.size)
x = random.uniform(0,L)
y = random.uniform(0,L)
objetivo = [x,y]
print('Objetivo: ', objetivo)
v = np.array([np.zeros(M),np.zeros(M)])
print(v)
for i in range(M):
    x = random.random()
    y = random.random()
    #v[0,i] = x
    #v[1,i] = y
    
for i in range(M):
    x = random.uniform(0,L)
    y = random.uniform(0,L)    
    q[0,i] = x
    q[1,i] = y
    

pbest = qbest(q,objetivo)
qibest = q.copy()

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
plt.scatter(objetivo[0],objetivo[1])
plt.scatter([0,10],[0,10])
px = [q[0,0],q[0,1],q[0,2],q[0,3],q[0,4],q[0,5],q[0,6],q[0,7],q[0,8],q[0,9]]
py = [q[1,0],q[1,1],q[1,2],q[1,3],q[1,4],q[1,5],q[1,6],q[1,7],q[1,8],q[1,9]]
plt.scatter(px,py)
plt.xlabel('Eixo x')
plt.ylabel('Eixo y')
plt.legend(['Objetivo','Pontos de scala','Particulas'])
plt.title('PSO Inicialisado')
a = str(0)
plt.savefig(a, format='png')
fig.canvas.draw()
plt.clf()
for j in range(k):
    
#atualizar posição e velocidade
    for i in range(M):
        for i2 in range(2):
            #v[i2,i] = 0.1*v[i2,i] + c1*random.random()*(qibest[i2,i]-q[i2,i]) + c2*random.random()*(q[i2,pbest]-q[i2,i])
            v[i2,i] = random.random()*v[i2,i] + c1*random.random()*(qibest[i2,i]-q[i2,i]) + c2*random.random()*(q[i2,pbest]-q[i2,i])
            q[i2,i] = q[i2,i] + v[i2,i]
            if(q[i2,i] > L):
                q[i2,i] = L
            if(q[i2,i] < 0):
                q[i2,i] = 0
#ver se a particula melhorou
    for i in range(M):
        d1 = distancia([q[0,i],q[1,i]],objetivo)
        d2 = distancia([qibest[0,i],qibest[1,i]],objetivo)
        if(d1 < d2):
            qibest[0,i] = q[0,i]
            qibest[1,i] = q[1,i]
#calcular a melhor particula
    pbest = qbest(q,objetivo)
    if(distancia([q[0,pbest],q[1,pbest]],objetivo) <= 0.01):
        print('em ',j)
        break
    
    
    plt.scatter(objetivo[0],objetivo[1])
    plt.scatter([0,10],[0,10])
    px = [q[0,0],q[0,1],q[0,2],q[0,3],q[0,4],q[0,5],q[0,6],q[0,7],q[0,8],q[0,9]]
    py = [q[1,0],q[1,1],q[1,2],q[1,3],q[1,4],q[1,5],q[1,6],q[1,7],q[1,8],q[1,9]]
    plt.scatter(px,py)
    plt.xlabel('Teta1')
    plt.ylabel('Teta2')
    plt.legend(['Objetivo','Limites do espaço de busca','Particulas'])
    plt.title('PSO')
    a = str(j)
    plt.savefig(a, format='png')
    fig.canvas.draw()
    plt.pause(0.1)
    plt.clf()

plt.scatter(objetivo[0],objetivo[1])
plt.scatter([0,10],[0,10])
px = [q[0,0],q[0,1],q[0,2],q[0,3],q[0,4],q[0,5],q[0,6],q[0,7],q[0,8],q[0,9]]
py = [q[1,0],q[1,1],q[1,2],q[1,3],q[1,4],q[1,5],q[1,6],q[1,7],q[1,8],q[1,9]]
plt.scatter(px,py)
plt.xlabel('Teta1')
plt.ylabel('Teta2')
plt.legend(['Objetivo','Limites do espaço de busca','Particulas'])
plt.title('PSO')
fig.canvas.draw()


print('Solução: ',[q[0,pbest],q[1,pbest]])
print('F final: ',distancia([q[0,pbest],q[1,pbest]],objetivo))
print('Numero de iteração: ',j + 1)
