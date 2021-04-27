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
    for i in range(12):
        x, a, b, c, d = sim.simxReadProximitySensor(clientID, objectHandles[i], sim.simx_opmode_oneshot_wait)
        dists.append((a, b[2]))

    return dists

def toDeg(radians):

    return radians * (180.0 / math.pi)

def movementControl(pos, goal, clientID, usensors, rightMotorHandle, leftMotorHandle):

    ## OBS: No nosso caso beta não importa pois os nossos alvos não têm orientação.

    kp = 1.2
    ka = 5.0

    dx = goal[0] - pos[0]
    dy = goal[1] - pos[1]
    rho = math.sqrt(dx*dx + dy*dy)
    temp = math.atan2(dy, dx)

    if(pos[2]>=0):   # o livro não especifica se theta deve estar em algum limite, portanto coloco ele entre 0 e 2pi
        theta = pos[2]
    else:
        theta = 2 * math.pi + pos[2]

    alpha = -theta + temp

    if(alpha < -math.pi):  # o livro diz que alpha deve estar entre -pi e pi, aqui ajusto o alpha para estes limites
        alpha += 2 * math.pi
        
    v_robot = kp * rho
    w_robot = ka * alpha

    noDetectionDist = 0.5
    maxDetectionDist = 0.2
    detect = [0, 0, 0, 0, 0, 0]
    braitenbergL = [-0.2, -0.5, -0.8, -1.1, -1.4, -1.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    braitenbergR = [-1.7, -1.4, -1.1, -0.8, -0.5, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    data = getDists(clientID, usensors)
    dists = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]

    for i in range(6):
        if(data[i][0] == True and data[i][1] < noDetectionDist):
            if(data[i][1] < maxDetectionDist):
                dists[i] = maxDetectionDist
            else:
                dists[i] = data[i][1]
            detect[i] = 1 - ((dists[i] - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else:
            detect[i] = 0
    
    if(sorted(detect, reverse=True)[0] >= 0.9):

        print(sorted(detect, reverse=True)[0])
        phiL = 3
        phiR = 3
        for i in range(6):
            phiL += (braitenbergL[i] * detect[i])
            phiR += (braitenbergR[i] * detect[i])

    else:
        phiL = (v_robot - (L * w_robot)) / (20 * RAIO)
        phiR = (v_robot + (L * w_robot)) / (20 * RAIO)
    
    return phiL, phiR

def setTargetSpeed(clientID, phiL, phiR, leftMotorHandle, rightMotorHandle):
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, sim.simx_opmode_oneshot)   

print ('Program started')

sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart(COPPELIA_SIM_IP_ADDRESS, COPPELIA_SIM_PORT, True, True, 2000, 5) # Connect to CoppeliaSim

if (clientID != -1):

    print('Connection Successful')
    
    # Get handles for robot parts, actuators and sensors:
    err1, ddRobotHandle = sim.simxGetObjectHandle(clientID, 'RobotFrame#', sim.simx_opmode_oneshot_wait)
    err2, leftMotorHandle = sim.simxGetObjectHandle(clientID, 'LeftMotor#', sim.simx_opmode_oneshot_wait)
    err3, rightMotorHandle = sim.simxGetObjectHandle(clientID, 'RightMotor#', sim.simx_opmode_oneshot_wait)
    err4, targetHandle = sim.simxGetObjectHandle(clientID, 'RedBall01#', sim.simx_opmode_oneshot_wait)

    usensors = []
    for i in range(1, 13):
        err, usensor = sim.simxGetObjectHandle(clientID, 'Proximity_sensor' + str(i) + '#', sim.simx_opmode_oneshot_wait)
        usensors.append(usensor)

    print(f'RobotFrame: {ddRobotHandle}')
    print(f'LeftMotor: {leftMotorHandle}')
    print(f'RightMotor: {rightMotorHandle}')

    # start simulation
    ret = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    
    if (ret==-1):
        print('Connection Failed.')
        exit(-1)
    
    print('Simulation Started')

    # while is connected
    while (sim.simxGetConnectionId(clientID) != -1):
        
        # Read current position
        pos = getPosition(clientID, ddRobotHandle)

        # Read simulation time of the last command
        # Simulation time in ms or 0 if sim is not running
        # stop the loop if simulation is has been stopped
        if (sim.simxGetLastCmdTime(clientID) == 0):
            break

        #print(f'Posicao: [{pos[0]:.2f} {pos[1]:.2f} {toDeg(pos[2]):.2f}°] ', end = '')
        
        # Set new target speeds: robot going in a circle
        goal = getPosition(clientID, targetHandle)
        #print(f'Objetivo: [{goal[0]:.2f} {goal[1]:.2f} {toDeg(goal[2]):.2f}°] ', end = '')
        
        ERR_MARGIN = 0.05
        phiL = 0.0
        phiR = 0.0

        if ((abs(goal[1] - pos[1]) <= ERR_MARGIN) and (abs(goal[0] - pos[0]) <= ERR_MARGIN and (abs(goal[2] - pos[2]) <= ERR_MARGIN))):
            print('Fim!')
            phiL = 0.0
            phiR = 0.0
            #break
        else:
            phiL, phiR = movementControl(pos, goal, clientID, usensors, rightMotorHandle, leftMotorHandle)
        
        #print('Velocidades:', phiL, phiR)
        #setTargetSpeed(clientID, phiL, phiR, leftMotorHandle, rightMotorHandle);

        # Leave some time for CoppeliaSim do it's work
        setTargetSpeed(clientID, phiL, phiR, leftMotorHandle, rightMotorHandle)
        time.sleep(0.1)
    
    # Stop the robot and disconnect from CoppeliaSim
    setTargetSpeed(clientID, 0, 0, leftMotorHandle, rightMotorHandle)
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)
            
else:
    print('Connection failed.')
    exit(-2)