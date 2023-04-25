import cv2
import numpy as np
from opcua import  *
from time import sleep

cam1 = cv2.VideoCapture(1)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH,1280)

color1LowerBound=np.array([ 92, 75, 206])#Blue Color 
color1UpperBound=np.array([118,255,255])
color2LowerBound=np.array([71,59,80])#Green Color
color2UpperBound=np.array([88,255,255])

x1=0
y1=0
x0=0
y0=0

def establish_opc_conn():
    url = "opc.tcp://192.168.1.1:4840"
    client_conn = Client(url)

    while True:
        try:
            client_conn.connect()
            print('Connection established')
            break
        except: 
            print('Reconnecting after 5 secs...')
            sleep(5)
    client_conn.get_root_node()  
    return client_conn

def read_input_value(client_fn, node_id):
    client_node = Client.get_node(client_fn, node_id)  # get node
    client_node_value = client_node.get_value()  # read node value
    return client_node_value

def write_value_bool(node_id, value, client_fn):
    client_node = client_fn.get_node(node_id)  # get node
    client_node_value = value
    client_node_dv = ua.DataValue(ua.Variant(client_node_value, ua.VariantType.Boolean))
    client_node.set_value(client_node_dv)

def write_value_int(node_id, value, client_fn):
    client_node = Client.get_node(client_fn, node_id)  # get node
    client_node_value = value
    client_node_dv = ua.DataValue(ua.Variant(client_node_value, ua.VariantType.Int16))
    client_node.set_value(client_node_dv)

def calculateParameters(x0,y0,x1,y1):
    if x1==0 and y1==0 | x1==0 and y1==0:
        angle=720 
        distance=0
    else:
        angle = np.arctan2(y1 - y0, x1- x0)
        angle = np.degrees(angle)
        # ensure angle is positive and in the range [0, 360)
        if angle < 0:
            angle += 360
        distance=np.hypot(x1-x0,y1-y0)
    return (angle,distance)

def getAngle(mask1,mask2,frame):
    x0=0
    y0=0
    x1=0
    y1=0
    contours1,junk=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours1:
        area=cv2.contourArea(contour)
        if area>200:
            x,y,w,h=cv2.boundingRect(contour)
            x0=x+int(w/2)
            y0=y+int(h/2)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)          
    contours2,junk=cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for cntr in contours2:
        area2=cv2.contourArea(cntr)
        if area2>200:
            x2,y2,w2,h2=cv2.boundingRect(cntr)
            x1=x2+int(w2/2)
            y1=y2+int(h2/2)
            cv2.rectangle(frame,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)
    cameraParams=calculateParameters(x0,y0,x1,y1)
    return cameraParams
def mouseClick(event,xPos,yPos,flags,params):
    global evt
    global pt1
    global pt2
    if event==cv2.EVENT_LBUTTONDOWN:
        print(event)
        evt=event
        pt1=(xPos,yPos)
    if event==cv2.EVENT_LBUTTONUP:
        print(event)
        evt=event
        pt2=(xPos,yPos)
count=0 
evt=0
roi1Gotten=False
cv2.namedWindow('Frame1')
cv2.setMouseCallback('Frame1',mouseClick) 
while True:
    ignore,frame1=cam1.read()
    frame1=cv2.flip(frame1,0)
    frame1=cv2.flip(frame1,1)
    if count==0:
        print('Select your Region Of Interest For Camera 1')
        print()
        print('Make sure the workpiece is at the start position')
    if evt==4:
        cv2.rectangle(frame1,pt1,pt2,(0,255,0),2)
        ROI1=frame1[pt1[1]:pt2[1],pt1[0]:pt2[0]]
        roi1Gotten=True
    cv2.imshow('Frame1',frame1)
    cv2.moveWindow('Frame1',0,0)
    count=count+1
    if roi1Gotten==True:
        break
    if cv2.waitKey(1)&0xff==ord('q'):
        break
cam1.release()
cv2.destroyAllWindows()

cam1=cv2.VideoCapture(1)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
plc_client_1 = establish_opc_conn()
movement=False
steps=0
ratioCount=0
prevDistance=0
prevAngle=0
#Main program which gets the angles and distances:
while True:
    ignore, frame = cam1.read()
    frame=cv2.flip(frame,0)
    frame=cv2.flip(frame,1)
    frameROI=frame[pt1[1]:pt2[1],pt1[0]:pt2[0]]
    #frameROI = cv2.resize(frameROI, (0,0), fx=2, fy=2)
    frameHSV=cv2.cvtColor(frameROI,cv2.COLOR_BGR2HSV)

    myMask1=cv2.inRange(frameHSV,color1LowerBound,color1UpperBound)
    myMask2=cv2.inRange(frameHSV,color2LowerBound,color2UpperBound)
    camera1Params=getAngle(myMask1,myMask2,frameROI)
    camera1Angle=camera1Params[0]
    camera1Distance=camera1Params[1]
    if ratioCount==0:
        verifiedCamera1Distance=422
        ratio=camera1Distance/verifiedCamera1Distance
    camera1Distance=camera1Distance/ratio

    #print('Camera 1 Angle is: {}' .format(int(camera1Angle)))
    #print('Camera 1 distance is: {}' .format(int(camera1Distance)))
    if camera1Angle>=194 and camera1Angle<=198 and camera1Distance>=418 and camera1Distance<=424:
        #print('Start position')
        steps=1
    if camera1Angle>=221 and camera1Angle<=225 and camera1Distance>=395 and camera1Distance<=410:
        #print('Orientation check')
        steps=2
    if camera1Angle>=244 and camera1Angle<=248 and camera1Distance>=335 and camera1Distance<=345:
        #print('Flipping part 1')
        steps=3
    if camera1Angle>=244 and camera1Angle<=248 and camera1Distance>=356 and camera1Distance<=361:
        #print('Flipping part 2')
        steps=4
    if camera1Angle>=308 and camera1Angle<=312 and camera1Distance>=105 and camera1Distance<=114 :
        #print('Before next station')
        steps=5
    if camera1Angle>=323 and camera1Angle<=328 and camera1Distance>=126 and camera1Distance<=133:
        #print('Transfer to next station')
        steps=6
    if camera1Angle>=341 and camera1Angle<=345 and camera1Distance>=235 and camera1Distance<=247:
        #print('Differentiating position')
        steps=7
    if camera1Angle>=324 and camera1Angle<=330 and camera1Distance>=265 and camera1Distance<=280:
        #print('Metalic Sorter Rod Extended')
        steps=8
    if camera1Angle>=350 and camera1Angle<=354 and camera1Distance>=445 and camera1Distance<=455:
        #print('Release Acrylic')
        steps=9
    if camera1Angle>=352 and camera1Angle<=356 and camera1Distance>=535 and camera1Distance<=545:
        #print('End of acrylic partition')
        steps=10
    if camera1Angle>=341.5 and camera1Angle<=343.5 and camera1Distance>=450 and camera1Distance<=460:
        #print('Release Metalic')
        steps=11
    if camera1Angle>=344 and camera1Angle<=346 and camera1Distance>=535 and camera1Distance<=560:
        #print('End of metalic partition')
        steps=12
    if camera1Angle==720 or camera1Distance==0:
        print('One or more object(s) not detected')
    deltaDistance=abs(camera1Distance-prevDistance)
    deltaAngle=abs(camera1Angle-prevAngle)
    if prevAngle==720 or prevDistance==0 or prevAngle==0:
        pass
    else:
        if deltaDistance>=3 or deltaAngle>=3:
            movement=True
            print('Moving.......')
        else:
            movement=False
            print('Stationary')
    write_value_int('ns=3;s="pyOpcua"."steps"', steps, plc_client_1)
    write_value_bool('ns=3;s="pyOpcua"."movement"', movement, plc_client_1)
    ratioCount=ratioCount+1
    prevDistance=camera1Distance
    prevAngle=camera1Angle
    cv2.imshow('my ROI',frameROI)
    cv2.moveWindow('my ROI',0,0)

    if cv2.waitKey(1)&0xff==ord('q'):
        break
cam1.release()
cv2.destroyAllWindows()
#Getting the ROI of our second camera
cam2 = cv2.VideoCapture(2)
cam2.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cam2.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
evt=0
cv2.namedWindow('Frame2')
cv2.setMouseCallback('Frame2',mouseClick)
count2=0
roi2Gotten=False
while True:
    ignore,frame2=cam2.read()
    if count2==0:
        print('Select your Region Of Interest For Camera 2')
    if evt==4:
        cv2.rectangle(frame2,pt1,pt2,(0,255,0),2)
        ROI2=frame2[pt1[1]:pt2[1],pt1[0]:pt2[0]]
        roi2Gotten=True
    cv2.imshow('Frame2',frame2)
    cv2.moveWindow('Frame2',0,0)
    count2=count2+1
    if roi2Gotten==True:
        break
    if cv2.waitKey(1)&0xff==ord('q'):
        break
cam2.release()
cv2.destroyAllWindows()
cam2=cv2.VideoCapture(2)
cam2.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cam2.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
ratio2Count=0
while True:
    ignore, frame2 = cam2.read()
    #frame2=cv2.flip(frame2,0)
    frame2=cv2.flip(frame2,1)
    frameROI2=frame2[pt1[1]:pt2[1],pt1[0]:pt2[0]]
    frameROI2 = cv2.resize(frameROI2, (0,0), fx=2, fy=2)
    frameHSV2=cv2.cvtColor(frameROI2,cv2.COLOR_BGR2HSV)

    myMask3=cv2.inRange(frameHSV2,color1LowerBound,color1UpperBound)
    myMask4=cv2.inRange(frameHSV2,color2LowerBound,color2UpperBound)
    camera2Params=getAngle(myMask3,myMask4,frameROI2)
    camera2Angle=camera2Params[0]
    camera2Distance=camera2Params[1]
    #if ratio2Count ==0:
        #verifiedCamera12Distance=800
        #ratio2=camera1Distance/verifiedCamera1Distance
    #camera1Distance=camera1Distance/ratio2

    print('Camera 2 Angle is: {}' .format(int(camera2Angle)))
    print('Camera 2 distance is: {}' .format(int(camera2Distance)))
    ratio2Count=ratio2Count+1
    cv2.imshow('my ROI2',frameROI2)
    cv2.moveWindow('my ROI2',0,0)
    if cv2.waitKey(1)&0xff==ord('q'):
        break
cam1.release()
cv2.destroyAllWindows()






    




    

