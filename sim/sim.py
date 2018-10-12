import zmq
import time
import struct
import numpy as np

def setDepth(dp):
    return dp

def setYaw(x,y): 
    return x,y

context = zmq.Context()
 
subscriber = context.socket (zmq.SUB)
subscriber.connect ("tcp://127.0.0.1:3390")
subscriber.setsockopt(zmq.SUBSCRIBE, b"")

simulator = context.socket (zmq.PAIR)
simulator.connect("tcp://127.0.0.1:3391")

Kp=np.array([1,5])
Ki=np.array([0.2,2]) #PID gain coeffiicients
Kd=np.array([0.1,2])

I1=0
I0=0
yawErrLast=0
depthErrLast=0

startTime = time.time()
message = subscriber.recv()

targetDepth=setDepth(40)
V,targetYaw=setYaw(30,-35)

while True:

    message = subscriber.recv()
    
    ts=time.time() - startTime #time step

    yaw=struct.unpack('<f',message[16:20])
    yaw=float(".".join(map(str,yaw))) 
    depth=struct.unpack('<f',message[20:24])
    depth=float(".".join(map(str,depth)))

    if yaw<=360 and yaw>=180:   
        yaw=-(360-yaw)

    if targetYaw<0:
        targetYaw=360+targetYaw

    if targetYaw<=360 and targetYaw>=180:
        targetYaw=-(360-targetYaw)
    
          
   
    
    depthErr=targetDepth-depth
    yawErr=targetYaw-yaw
        
    P0=Kp[0]*yawErr
    I0=I0+Ki[0]*yawErr*ts               #Yaw control PID loop
    D0=Kd[0]*(yawErr-yawErrLast)/ts

    P1=Kp[1]*depthErr
    I1=I1+Ki[1]*depthErr*ts             #Depth control PID loop
    D1=Kd[1]*(depthErr-depthErrLast)/ts

    
    U0=P0+D0+I0
    U1=P1+D1+I1

    if U0>80:
        U0=80
    elif U0<-80:
        U0=-80
    
    if U1<-100:
        U1=-100
    elif U1>100:
        U1=100

    yawErrLast=yawErr
    depthErrLast=depthErr
    
    trast=np.array([-V-U0,-V+U0,U1,0])
    trast=trast.astype('uint8')
    startTime=time.time()

    simulator.send(trast)
   
 
