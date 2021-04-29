try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import cv2
import numpy as np
import math
import time

COPPELIA_SIM_IP_ADDRESS = '127.0.0.1'
COPPELIA_SIM_PORT = 19999
RAIO = 0.02
L = 0.1

def getPosition(clientID, objectHandle):    # [x,y,theta]

    ret, position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_oneshot_wait)
    if (ret > 0):
        print('Error reading object position')
        return -1

    ret, orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_oneshot_wait)
    if (ret > 0):
        print('Error reading object orientation')
        return -1

    position[2] = orientation[2]
    
    return position

def getDists(clientID, objectHandles):

    dists = []
    ## extraindo as distancias lidas pelos sensores ultrassônicos (b), se houve objeto detectado (a == True).
    for i in range(7):
        x, a, b, c, d = sim.simxReadProximitySensor(clientID, objectHandles[i], sim.simx_opmode_buffer)
        dists.append((a, b[2]))

    return dists

def toDeg(radians):

    return radians * (180.0 / math.pi)

def movementControl(pos, goal, clientID, usensors):

    ## aqui nada muda, a não ser os fatos de ignorarmos o beta, pois os alvos não têm orientação e
    ## de que o nosso robô não anda de ré.

    kp = 1.2
    ka = 5.0

    dx = goal[0] - pos[0]
    dy = goal[1] - pos[1]
    rho = math.sqrt(dx*dx + dy*dy)
    temp = math.atan2(dy, dx)

    if(pos[2]>=0):
        theta = pos[2]
    else:
        theta = 2 * math.pi + pos[2]

    alpha = -theta + temp

    if(alpha < -math.pi):
        alpha += 2 * math.pi
        
    v_robot = kp * rho
    w_robot = ka * alpha

    ## executamos a parte de evitar obstáculos com uma adaptação do algoritmo de Braitenberg.
    ## esse algoritmo diz que se o obstáculo está a esquerda do robô, então devemos girar a roda esquerda mais rápido que a direita.
    ## o mesmo vale para quando o obstáculo está a direita do robô.
    ## noDetection é a partir de qual distância já não importa se tem obstáculo ou não, pois se tiver obstáculo, ele estará demasiado longe.
    noDetectionDist = 0.5
    ## maxDetection é a mínima distância até o obstáculo, qualquer leitura menor, e.g., 0.15, será arredondada para 0.2 (todos os valores estão em metros).
    maxDetectionDist = 0.2
    ## detect vai ser o vetor de valores utilizados para o algoritmo. ele é um vetor de proporções de cada sensor, de 1 a 7 (nesta ordem).
    ## então se o detect[0] for um valor próximo de 1, significa que tem um obstáculo a aproximadamente 0.2 metros do sensor 1 do robô. quanto mais próximo de 0 o valor de detect[0],
    ## mais distante estaria o obstáculo do sensor 1. isso vale para todos os detect[i] (todos os sensores).
    detect = [0, 0, 0, 0, 0, 0, 0]
    ## esses são os vetores de braitenberg, o vetor L, é usado na roda esquerda, e o vetor R na direita. cada valor de cada vetor vai ser usado
    ## como fator multiplicativo nas velocidades.
    braitenbergL = [-0.2, -0.4, -0.6, 1.0, -0.8, -1.0, -1.2]
    braitenbergR = [-1.2, -1.0, -0.8, -3.0, -0.6, -0.4, -0.2]

    ## lê os sensores
    data = getDists(clientID, usensors)
    dists = [-1, -1, -1, -1, -1, -1, -1]

    ## transforma distâncias dos sensores em proporções como mencionado antes, se obstáculo perto do sensor, proporção perto de 1, se obstáculo longe proporção perto de 0.
    for i in range(7):
        if(data[i][0] == True and data[i][1] < noDetectionDist):
            if(data[i][1] < maxDetectionDist):
                dists[i] = maxDetectionDist
            else:
                dists[i] = data[i][1]
            detect[i] = 1 - ((dists[i] - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else:
            detect[i] = 0
    
    ## se existe algum sensor que detecta um obstáculo com proporção maior que 0.55 (valor obtido por testes), faça:
    if(sorted(detect, reverse=True)[0] >= 0.55):
        #print(sorted(detect, reverse=True)[0])
        ## algoritmo de braitenberg em efeito. basicamente atribui às velocidades um valor pequeno o suficiente pra manobrar sem estresse (valor obtido por teste) e igual.
        phiL = 2
        phiR = 2
        ## reduz o valor atribuido às velocidades de forma proporcional, ou seja, se o sensor 1 leu que tem um obstáculo bem perto (detect[0] == 1), e os outros sensores leram obstáculos longe,
        ## temos que girar a roda esquerda mais rápido que a direita, o quanto mais rápido? (vetores de braitenberg) 6 vezes mais rápido no nosso caso (valor obtido por testes). 
        ## perceba que cada sensor tem um peso diferente. assim quando só o sensor 1 detecta obstáculo, a roda esquerda gira 6 vezes mais rápido que a direita. quando só o sensor 3 detecta obstáculo
        ## a roda esquerda gira 0.75 vezes mais rápido que a direita. na maioria das vezes a mais de um sensor detecta obstáculos, então a velocidade é uma combinação da leitura de 2 ou até 3 sensores.
        ## atenção ao fato do sensor 3 fugir do padrão. ele gira faz a roda esquerda girar bem mais rápido que os outros sensores pois ele fica apontado bem a frente do robô, então quando o robõ vê um
        ## obstáculo bem a frente, ele sempre vira para a esquerda de forma bem rápida (valores obtidos por testes).
        for i in range(7):
            phiL += (braitenbergL[i] * detect[i])
            phiR += (braitenbergR[i] * detect[i])
    ## se não existe obstáculos a frente do robô siga a trajetória normal até o alvo.
    else:
        ## reduzi bastante a velocidade aqui pra dar tempo pro nosso programa reagir a leitura dos sensores. se não for assim o robô parte rápido demais e só lê os sensores quando já bateu nos obstáculos.
        #print(sorted(detect, reverse=True)[0])
        phiL = (v_robot - (L * w_robot)) / (55 * RAIO)
        phiR = (v_robot + (L * w_robot)) / (55 * RAIO)
    
    return phiL, phiR

def setTargetSpeed(clientID, phiL, phiR, revoluteJointSpeed, leftMotorHandle, rightMotorHandle, revoluteJointHandle):
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, revoluteJointHandle, revoluteJointSpeed, sim.simx_opmode_oneshot)   

print ('Program started')

sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart(COPPELIA_SIM_IP_ADDRESS, COPPELIA_SIM_PORT, True, True, 2000, 5) # Connect to CoppeliaSim

## lower e upper são vetores pra detecção da cor vermelha, cada elemento do vetor é um dos canais RGB (invertido por causa do opencv).
## então o limite inferior pra detecção seria o valor 0 para a cor azul, 0 para a cor verde e 100 para a cor vermelha.
## a análise é análoga para o vetor upper.
lower = np.uint8([0, 0, 100])
upper = np.uint8([60, 60, 255])

## nós temos 8 targets, portanto para pegar as suas referências (handles), é preciso iterar sobre eles, daí vem o targetIterator.
targetIterator = 1

## o imageItarator é usado para que se alguém quiser visualizar as imagens capturadas pelo vision sensor do robô, o incremento de imageItarator
## faz com que o nome das imagens vá mudando, i. e., image0.png, image1.png, etc.
imageItarator = 0

if (clientID != -1):

    print('Connection Successful')

    ## pega as referências de todos os elementos a serem manipulados.
    ## perceba que só pega a referência do primeiro alvo porque só pegamos a referência de um alvo por vez.
    err1, ddRobotHandle = sim.simxGetObjectHandle(clientID, 'RobotFrame#', sim.simx_opmode_oneshot_wait)
    err2, leftMotorHandle = sim.simxGetObjectHandle(clientID, 'LeftMotor#', sim.simx_opmode_oneshot_wait)
    err3, rightMotorHandle = sim.simxGetObjectHandle(clientID, 'RightMotor#', sim.simx_opmode_oneshot_wait)
    err4, revoluteJointHandle = sim.simxGetObjectHandle(clientID, 'RevoluteJoint#', sim.simx_opmode_oneshot_wait)
    err5, visionSensorHandle = sim.simxGetObjectHandle(clientID, 'VisionSensor#', sim.simx_opmode_oneshot_wait)
    err6, targetHandle = sim.simxGetObjectHandle(clientID, 'Target' + str(targetIterator) + '#', sim.simx_opmode_oneshot_wait)
    targetIterator += 1

    ## pega a referência dos sensores ultrassônicos (usamos 7 sensores).
    ## atenção: sensores ultrassônicos != vision sensor
    usensors = []
    for i in range(1, 8):
        err, usensor = sim.simxGetObjectHandle(clientID, 'Proximity_sensor' + str(i) + '#', sim.simx_opmode_oneshot_wait)
        usensors.append(usensor)
    
    ## a leitura dos sensores não pode bloquear o loop de execução do robô (while mais abaixo), portanto o coppelia oferece o opmode streaming,
    ## isto é, quando você manda a leitura começar a ser feita com streaming, ele mantém-se fazendo a leitura e colocando os resultados em um buffer
    ## exclusivo para aquele handle (7 handles 7 buffers). Portanto só precisamos a cada iteração ler do buffer, e aí não é preciso pedir que o simulador
    ## faça uma nova leitura, economizando tempo e não bloqueando o loop de execução do robô.
    for i in range(7):
        x, a, b, c, d = sim.simxReadProximitySensor(clientID, usensors[i], sim.simx_opmode_streaming)

    ## a mesma regra dos sensores ultrassônicos descrita acima se aplica ao vision sensor (e a qualquer sensor que poderia vir a ser usado).
    a, b, image = sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_streaming)

    print(f'RobotFrame: {ddRobotHandle}')
    print(f'LeftMotor: {leftMotorHandle}')
    print(f'RightMotor: {rightMotorHandle}')
    print(f'RevoluteJoint: {revoluteJointHandle}')
    print(f'VisionSensor: {visionSensorHandle}')

    ret = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    if (ret==-1):
        print('Connection Failed.')
        exit(-1)
    print('Simulation Started')

    while (sim.simxGetConnectionId(clientID) != -1):
        pos = getPosition(clientID, ddRobotHandle)
        #print(f'posX: {pos[0]:.3f} posY: {pos[1]:.3f}', end=' ')
        goal = getPosition(clientID, targetHandle)
        #print(f'goalX: {goal[0]:.3f} goalY: {goal[1]:.3f}', end=' ')
        
        ## observe o simx_opmode_buffer. nós não pedimos uma nova leitura do vision sensor, nós simplesmente pegamos a leitura do buffer.
        a, b, image = sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_buffer)
        ## operações feitas para organizar a imagem do vision sensor.
        newImage = np.uint8(image).reshape((b[0], b[0], 3))
        ## definitiveImage é autoexplicativa, mas atenção, pois ela se encontra com a codificação BGR.
        definitiveImage = cv2.cvtColor(np.flip(newImage, 0), cv2.COLOR_RGB2BGR)
        ## mask é simplesmente uma outra imagem com a mesma resolução de definitiveImage, porém só mostrando objetos vermelhos, isto é,
        ## valor 255 nos pixels correspondentes a pixels vermelhos na imagem original, e 0 em todos os outros pixels.
        mask = cv2.inRange(definitiveImage, lower, upper)

        ## distancia do alvo para o robô.
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        euclid = math.sqrt(dx*dx + dy*dy)
        #print(euclid)

        ## se o alvo está perto o suficiente faça:
        if(euclid <= 0.23):
            ## se na mask só tem pixels diferentes de 0, portanto só pixels vermelhos faça:
            if(0.0 not in mask):
                ## para o robô e manda a hélice girar no sentido anti-horário por aproximadamente três segundos.
                phiL, phiR, phiH = 0, 0, 2
                #print('sim vermelho')
                setTargetSpeed(clientID, phiL, phiR, phiH, leftMotorHandle, rightMotorHandle, revoluteJointHandle)
                time.sleep(3)
                setTargetSpeed(clientID, phiL, phiR, 0, leftMotorHandle, rightMotorHandle, revoluteJointHandle)
            ## se não, o alvo é azul.
            else:
                ## então para o robô e manda a hélice girar no sentido horário por aproximadamente três segundos.
                phiL, phiR, phiH = 0, 0, -2
                #print('sim azul')
                setTargetSpeed(clientID, phiL, phiR, phiH, leftMotorHandle, rightMotorHandle, revoluteJointHandle)
                time.sleep(3)
                setTargetSpeed(clientID, phiL, phiR, 0, leftMotorHandle, rightMotorHandle, revoluteJointHandle)

            ## pega a referência do próximo alvo, se há alvos que ainda não foram explorados.
            if(targetIterator <= 8):
                err6, targetHandle = sim.simxGetObjectHandle(clientID, 'Target' + str(targetIterator) + '#', sim.simx_opmode_oneshot_wait)
                targetIterator += 1
            ## se não encerra o programa e pausa a simulação.
            else:
                break
        ## se o alvo ainda está longe, continua indo até ele com a hélice parada.
        else:
            phiL, phiR = movementControl(pos, goal, clientID, usensors)
            phiH = 0
            #print('nao vermelho')
            setTargetSpeed(clientID, phiL, phiR, phiH, leftMotorHandle, rightMotorHandle, revoluteJointHandle)
            

        ## descomente estas duas linhas se você quiser salvar as imagens capturadas.
        #cv2.imwrite('capture' + str(imageIterator) + '.png', definitiveImage)
        #imageIterator += 1    
    
    setTargetSpeed(clientID, phiL, phiR, phiH, leftMotorHandle, rightMotorHandle, revoluteJointHandle)
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)
            
else:
    print('Connection failed.')
    exit(-2)