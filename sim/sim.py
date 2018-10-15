# -*- coding: utf-8 -*-

import zmq
import time
import struct
import numpy as np

def set_depth(dp):
    return dp

def set_yaw(x,y): 
    return x,y

def constrain(var,min,max):
    if var>max:
        var=max
    elif var<min:
        var=min
    return var

def main():

    context = zmq.Context()
 
    subscriber = context.socket (zmq.SUB)
    subscriber.connect ("tcp://127.0.0.1:3390")
    subscriber.setsockopt(zmq.SUBSCRIBE, b"")

    simulator = context.socket (zmq.PAIR)
    simulator.connect("tcp://127.0.0.1:3391")

    Kp=np.array([1,5])
    Ki=np.array([0.2,2]) #PID gain coeffiicients
    Kd=np.array([0.1,2])

    I_depth=0
    I_yaw=0
    lastYawError=0
    lastDepthError=0

    startTime = time.time()

    targetDepth=set_depth(40)
    Force,targetYaw=set_yaw(30,-35)


    while True:

        recvMessage = subscriber.recv()
    
        timeStep=time.time() - startTime 

        realYaw=struct.unpack('<f',recvMessage[16:20])
        realYaw=float(".".join(map(str,realYaw))) 
        realDepth=struct.unpack('<f',recvMessage[20:24])
        realDepth=float(".".join(map(str,realDepth)))

        if realYaw<=360 and realYaw>=180:   
           realYaw-=360

        if targetYaw<=360 and targetYaw>=180:
            targetYaw-=360
    
        depthError=targetDepth-realDepth
        yawError=targetYaw-realYaw
        
        P_yaw=Kp[0]*yawError
        I_yaw=I_yaw+Ki[0]*yawError*timeStep               #Yaw control PID loop
        D_yaw=Kd[0]*(yawError-lastYawError)/timeStep

        P_depth=Kp[1]*depthError
        I_depth=I_depth+Ki[1]*depthError*timeStep             #Depth control PID loop
        D_depth=Kd[1]*(depthError-lastDepthError)/timeStep

        U_yaw=P_yaw+D_yaw+I_yaw
        U_depth=P_depth+D_depth+I_depth

        U_yaw=constrain(U_yaw,-80,80)
        U_depth=constrain(U_depth,-100,100)
    
        lasrYawError=yawError
        lastDepthError=depthError
    
        trasterForces=np.array([-Force-U_yaw,-Force+U_yaw,U_depth,0])
        trasterForces=trasterForces.astype('uint8')
        startTime=time.time()

        simulator.send(trasterForces)
        
if __name__ == '__main__':
    main()

 
