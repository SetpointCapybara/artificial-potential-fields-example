import sim
import time
import sys
import numpy as np

#----------------------------------------------------------------------------------Utility functions
def saturation(v, maxv):
    #Function that limits signals
    if v > maxv:
        v = maxv
    if v < -maxv:
        v = -maxv
    return v

def repulsion_force(angles, measurements):
    #Function that calculates the repulsion force vector
    frep_x = 0
    frep_y = 0
    for count, elem in enumerate(measurements):
        if measurements[count][0]:
             frep_x = frep_x + (measurements[count][1]**-1)*np.cos(np.deg2rad(angles[count]))
             frep_y = frep_y + (measurements[count][1]**-1)*np.sin(np.deg2rad(angles[count]))
    return 2*np.array([-frep_x, -frep_y])

def attraction_force(robot_position, goal_position):
    #Function tha calculates the attraction force vector
    fatt_x = 0
    fatt_y = 0    
    fatt_x = goal_position[0] - robot_position[0]
    fatt_y = goal_position[1] - robot_position[1] 
    return np.array([fatt_x, fatt_y])          
#----------------------------------------------------------Stablishing connections with CoppeliaSim - simRemoteApi.start(19999)
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:# If return code ia not an Error code
    print ('Connected to remote API server')
else:
    sys.exit('Failed connecting to remote API server')

#--------------------------------------------------------------------------------------Getting Handles
robotname = 'Pioneer_p3dx' # robot name
returnCode, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait) # getting robot handle   
returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait) # getting left wheel handle 
returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait) # getting right wheel handle 
sensors = [] # making a list for sensor handles
for i in range(1,17): #looping through all the sensors
    returnCode, temp = sim.simxGetObjectHandle(clientID, robotname + "_ultrasonicSensor" + str(i), sim.simx_opmode_oneshot_wait) # getting sensor handle 
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, temp, sim.simx_opmode_streaming) # starting sensor  communication
    sensors.append(temp) #adding sensor to list
returnCode, goalFrame = sim.simxGetObjectHandle(clientID, 'Goal', sim.simx_opmode_oneshot_wait)     

#-----------------------------------------------------------------------------------------Setup
#Data about the Pioneer robot
sensor_angles = [90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90]
L = 0.381
r = 0.0975
#max values for linear and angular speed
maxv = 1
maxw = np.deg2rad(45)

#--------------------------------------------------------------------------------------Control loop
t = 0
startTime = time.time() #Remember to start 'Real-time mode'
lastTime = startTime
while t < 60:
    now = time.time()
    dt = now - lastTime

    returnCode, robotPos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait) # Get robot position
    returnCode, robotOri = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait) # Get robot orientation   
    returnCode, goal_pos = sim.simxGetObjectPosition(clientID, goalFrame, -1, sim.simx_opmode_oneshot_wait) # Get Goal position

    #Reading sensors
    readings = []
    for i in range(16):
        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,sensors[i], sim.simx_opmode_buffer)
        readings.append([detectionState, detectedPoint[2]])
    
    frep = repulsion_force(sensor_angles, readings) # Calculating repulsion force
    fatt = attraction_force(robotPos, goal_pos) # Calculating attraction force
    ftotal = np.array([saturation(fatt[0], 1) + saturation(frep[0], 2), saturation(fatt[1], 1) + saturation(frep[1], 2)]) # Calculating total force

    kv = 1 # linear speed gain
    kw = 2 # angular speed gain

    # passing total force as error
    dx = ftotal[0] 
    dy = ftotal[1]

    # Calculating cinematic control
    v = kv*(dx*np.cos(robotOri[2]) + dy*np.sin(robotOri[2]))
    w = kw*(np.arctan2(dy,dx) - robotOri[2])
 
    # Limit v,w to +/- max
    v = saturation(v, maxv)
    w = saturation(w, maxw)

    # Calculating control signal with inverse kinematic
    vr = ((2.0*v) + (w*L))/(2.0*r)
    vl = ((2.0*v) - (w*L))/(2.0*r)
    
    # Sending control signal to robot
    sim.simxSetJointTargetVelocity(clientID, r_wheel, vr, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, l_wheel, vl, sim.simx_opmode_streaming)

    # Stop Control loop if this close to the
    if abs(dx) < 0.05 and abs(dy) < 0.05:
         break
    t = t + dt  
    lastTime = now

#Ending the whole process
#Stopping robot
sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

time.sleep(5)

#Stopping simulation
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)