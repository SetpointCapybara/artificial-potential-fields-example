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
way = [[-20.68, -21.97], [-20.68, -19.47], [-18.18, -19.47], [-15.68, -19.47], [-15.68, -16.97], [-15.68, -14.469999999999999], [-18.18, -14.469999999999999], [-20.68, -14.469999999999999], [-20.68, -11.969999999999999], [-20.68, -9.469999999999999], [-18.18, -9.469999999999999], [-15.68, -9.469999999999999], [-15.68, -6.969999999999999], [-15.68, -4.469999999999999], [-18.18, -4.469999999999999], [-20.68, -4.469999999999999], [-20.68, -1.9699999999999989], [-20.68, 0.5300000000000011], [-20.68, 3.030000000000001], [-20.68, 5.530000000000001], [-20.68, 8.030000000000001], [-18.18, 8.030000000000001], [-15.68, 8.030000000000001], [-13.18, 8.030000000000001], [-13.18, 5.530000000000001], [-10.68, 5.530000000000001], [-8.18, 5.530000000000001], [-8.18, 3.030000000000001], [-8.18, 0.5300000000000011], [-8.18, -1.9699999999999989], [-8.18, -4.469999999999999], [-8.18, -6.969999999999999], [-5.68, -6.969999999999999], [-3.1799999999999997, -6.969999999999999], [-0.6799999999999997, -6.969999999999999], [1.8200000000000003, -6.969999999999999], [4.32, -6.969999999999999], [6.82, -6.969999999999999], [9.32, -6.969999999999999], [9.32, -9.469999999999999], [9.32, -11.969999999999999], [9.32, -14.469999999999999], [9.32, -16.97], [9.32, -19.47], [11.82, -19.47], [14.32, -19.47], [16.82, -19.47], [19.32, -19.47], [19.32, -16.97], [19.32, -14.469999999999999], [16.82, -14.469999999999999], [14.32, -14.469999999999999], [14.32, -11.969999999999999], [14.32, -9.469999999999999], [14.32, -6.969999999999999], [14.32, -4.469999999999999], [14.32, -1.9699999999999989], [11.82, -1.9699999999999989], [9.32, -1.9699999999999989], [6.82, -1.9699999999999989], [4.32, -1.9699999999999989], [1.8200000000000003, -1.9699999999999989], [-0.6799999999999997, -1.9699999999999989], [-0.6799999999999997, 0.5300000000000011], [-0.6799999999999997, 3.030000000000001], [-0.6799999999999997, 5.530000000000001], [-0.6799999999999997, 8.030000000000001], [-0.6799999999999997, 10.530000000000001], [-0.6799999999999997, 13.030000000000001], [-0.6799999999999997, 15.530000000000001], [-0.6799999999999997, 18.03], [1.8200000000000003, 18.03], [4.32, 18.03], [6.82, 18.03], [9.32, 18.03], [11.82, 18.03], [14.32, 18.03], [14.32, 15.530000000000001], [14.32, 13.030000000000001], [16.82, 13.030000000000001], [19.32, 13.030000000000001], [19.32, 15.530000000000001], [19.32, 18.03], [19.32, 20.53]]
#--------------------------------------------------------------------------------------Control loop
t = 0
startTime = time.time() #Remember to start 'Real-time mode'
lastTime = startTime
step = 0
while t < 600:
    now = time.time()
    dt = now - lastTime
    returnCode, robotPos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait) # Get robot position
    returnCode, robotOri = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait) # Get robot orientation   
    #returnCode, goal_pos = sim.simxGetObjectPosition(clientID, goalFrame, -1, sim.simx_opmode_oneshot_wait) # Get Goal position
    
    goal_pos = [way[step][1], way[step][0]]

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
    dx = fatt[0] 
    dy = fatt[1]

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
    if abs(dx) < 1 and abs(dy) < 1:
         step = step + 1
         print(step)
         if step > len(way):
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