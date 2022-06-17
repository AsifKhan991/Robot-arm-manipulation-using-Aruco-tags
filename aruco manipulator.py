from tkinter import *
import pickle
import time
import math
import serial

import numpy as np
import serial
import cv2
import cv2.aruco as aruco

recorded_data=[]

COM='COM7'
baud=57600

port=1
Ardu_stat=1 # make it 0 if you want to edit and test this gui only, so program won't waste time trying connecting to serial port

# try connecting to serial port if Ardu_stat=1
if Ardu_stat==1:
    try:
        arduino = serial.Serial(COM, baud, timeout=2)
        print(' Port found! ')
    except serial.serialutil.SerialException:
        print(' Port not found! ')
        
        port=0
        
WIDTH = 1920
HEIGHT = 1080

x=550  #initial end effector position
y=400

#setting dimensions of the arm
f=2
basex,basey=320,600
armlengths=[135.28*f,148.772*f,50*f]

window = Tk()
window.title('3 DOF Robotic Arm Simulator')
canvas = Canvas(window,width=WIDTH,height=HEIGHT,bg='black')
canvas.pack()

mode=False #end effector mode 

angles=[0,0,0,1500,0]
section=[0,0,0]
joint=[0,0,0]

#creating the arm in canvas

joint_color='gray55'

ground=canvas.create_line(0,696,WIDTH,696,fill='red',width='2',smooth=True)

section[0]=canvas.create_line(basex,basey,397,430,fill='orange',width='40',smooth=True)
joint[0]=canvas.create_oval(320+25,720+25,320-25,720-25,fill=joint_color)

section[1]=canvas.create_line(397,430,797,430,fill='DarkOrange1',width='30',smooth=True)
joint[1]=canvas.create_oval(320+25,720+25,320-25,720-25,fill=joint_color)

section[2]=canvas.create_line(600,430,867,430,fill='DarkOrange2',width='20',smooth=True)
joint[2]=canvas.create_oval(600,430+25,600-25,430-25,fill=joint_color)
base_plate=canvas.create_rectangle(basex-90,600,basex+50,696,fill='orange')

mode_text=canvas.create_text(120,30,fill='green2',font="15",text="End Effector unlocked")
grip_text=canvas.create_text(650,30,fill='green2',font="15",text="Gripper unlocked")

crcx=1200
crcy=100
rad=70
canvas.create_arc(crcx+rad,crcy+rad,crcx-rad,crcy-rad,start=0, extent=180,outline='green2',width='2')
lov=10
canvas.create_line(crcx+rad+lov,crcy,crcx-rad-lov,crcy,fill='green2',width='1')
canvas.create_line(crcx,crcy+lov,crcx,crcy-rad-lov,fill='green2',width='1',splinesteps=10)
base_pos=canvas.create_line(crcx,crcy,crcx,crcy-rad,fill='red',width='5')
base_text=canvas.create_text(1200,200,fill='green2',font="15",text="")
crcx=1400
canvas.create_oval(crcx+rad,crcy+rad,crcx-rad,crcy-rad,outline='green2',width='2')
lov=10
canvas.create_line(crcx+rad+lov,crcy,crcx-rad-lov,crcy,fill='green2',width='1')
canvas.create_line(crcx,crcy+rad+lov,crcx,crcy-rad-lov,fill='green2',width='1',splinesteps=10)
gripper_pos=canvas.create_line(crcx+rad,crcy,crcx-rad,crcy,fill='yellow',width='5')
griprot_text=canvas.create_text(1400,200,fill='green2',font="15",text="E.E Rotation")


def change_mode(event): #end effector mode changing functon
    global mode
    global x,y
    mode=not mode
    if mode==True:
        canvas.itemconfig(mode_text, text="End Effector locked")
        canvas.itemconfig(section[2],fill='green2')
    else:
        canvas.itemconfig(mode_text, text="End Effector unlocked")
        canvas.itemconfig(section[2],fill='DarkOrange2')
        
start=0    
def startsending(event): # function to start streaming data to serial
    global start
    start=1
    
grab=0
pressure=20

def toggle_grab(event): # gripper grab ungrab funtion
    global grab,angles
    if grab==0:
        grab=1
        angles[4]=pressure   
        canvas.itemconfig(grip_text,fill='red', text="Gripper locked")
    else:
        grab=0
        angles[4]=0
        canvas.itemconfig(grip_text,fill='green2', text="Gripper unlocked")

base=0
rotation=90
base_angle=90
gripper_angle=90

def toggle_base_gripper(event): # function to toggle between base and gripper rotation using scroll 
    global base, rotation, gripper_angle,base_angle  
    if base==0:
        base=1
        rotation=base_angle
        canvas.itemconfig(griprot_text, text='')
        canvas.itemconfig(base_text, text='Base Rotation')
    else:
        base=0
        rotation=gripper_angle
        canvas.itemconfig(griprot_text, text='E.E Rotation')
        canvas.itemconfig(base_text, text='')
        
def get_coords(event): # function for getting mouse curson coordinates
    global x
    global y
    x, y = event.x,event.y
    #print(x,y)
    #print(angles)
    

def MouseWheelHandler(event): # scroll amount calculation function 
    global rotation, gripper_angle,base_angle  
    rotation += int(event.delta/24)
    if rotation<0:
        rotation=0
    elif rotation>180:
        rotation=180
    if base==0:
        gripper_angle=rotation
    else:
        base_angle=rotation
    #print(rotation)
        

def record(event):# function for starting the sequence
    global sequence
    sequence = 1

def save_exit(event): # save and exit function
    arduino.close()
    if recorded_data:
        with open('pos.pkl', 'wb') as f:
            pickle.dump(recorded_data, f)
    window.destroy()
    cv2.destroyAllWindows()
    quit()

#tkinter mouse kb binding
window.bind('<r>', record)    
window.bind('<B1-Motion>', get_coords)
window.bind('<m>', change_mode)
window.bind('<s>', startsending)
window.bind('<b>', toggle_base_gripper)
window.bind('<g>', toggle_grab)
window.bind("<MouseWheel>",MouseWheelHandler)
window.bind("q",save_exit)


        
def save_angle(i,angle): #angle processing with limits as servo has rotation angle limit of 180  degree
    if i==0:
        temp=(180-math.degrees(angle))
        if (temp>=0 and temp<=180):
            angles[i]=temp
            return 0
        else:
            return 1
    elif i==1:
        temp=math.degrees(angle)
        temp=360-temp-angles[0]-((temp>180 * temp<360)*360)
        if (temp>=0 and temp<=180):
            angles[i]=temp
            return 0
        else:
            return 1
    elif i==2:
        temp=math.degrees(angle)
        temp=(180-temp-((temp>180 * temp<360)*360))+270-angles[0]-angles[1]
        angles[i]=temp
        if (temp>=0 and temp<=180):
            return 0
        else:
            return 1

preval=''
def sendangles(): # angle processing for servo nonlinearity correction and sending to serial
    global preval
    val=''
    val=str(int((base_angle*2000/180)+500))+','
    val=val+str(int((188-angles[0])*1900/170)+500)+','
    val=val+str(int(angles[1]*1900/174)+500)+','
    val=val+str(int((angles[2]+6+13)*2000/174)+500)+','
    val=val+str(int((gripper_angle*2000/180)+500))+','
    val=val+str(angles[4])+',\n'    
    #print(angles)
    if Ardu_stat==1 and val!=preval:
        arduino.write(val.encode())
    preval=val

def rotate_gripper():    
    theta=math.radians(gripper_angle-90)
    tx=1400+(70*math.cos(theta))
    ty=100+(70*math.sin(theta))
    sx=1400-(70*math.cos(theta))
    sy=100-(70*math.sin(theta))
    canvas.coords(gripper_pos,sx,sy,tx,ty)

def rotate_base():
    theta=math.radians(base_angle)
    tx=1200-(70*math.cos(theta))
    ty=100-(70*math.sin(theta))
    canvas.coords(base_pos,1200,100,tx,ty)

#inverse kinematics algorithm starts here  
def shift_to_base(xdiff,ydiff):
    #print(xdiff,ydiff)
    for j in range(3):
        ix,iy,ex,ey=canvas.coords(section[j])
        ix=ix-xdiff
        iy=iy-ydiff
        ex=ex-xdiff
        ey=ey-ydiff
        canvas.coords(section[j],ix,iy,ex,ey)           
        ix,iy,ex,ey=canvas.coords(joint[j])
        ix=ix-xdiff
        iy=iy-ydiff
        ex=ex-xdiff
        ey=ey-ydiff
        canvas.coords(joint[j],ix,iy,ex,ey)
        
def inverse_kin(i,r,mx,my):
    bx,by,m,n=canvas.coords(section[i])
    angle= math.atan2(by-my,bx-mx)
    st=save_angle(i,angle)
    if st==0:
        tx=bx-r*math.cos(angle)
        ty=by-r*math.sin(angle)
    if st==1 or (mode==True and i==2):
        tx=m
        ty=n
    bx=bx+(mx-tx)
    by=by+(my-ty)
    tx=mx
    ty=my
    canvas.coords(section[i], bx,by,tx,ty)
    canvas.coords(joint[i],bx-(35-i*10),by-(35-i*10),bx+(35-i*10),by+(35-i*10))    
    return bx,by

def follow_inverse():
    bx,by=inverse_kin(2,armlengths[2],x,y)
    bx,by=inverse_kin(1,armlengths[1],bx,by)
    bx,by=inverse_kin(0,armlengths[0],bx,by)    
    shift_to_base(bx-basex,by-basey)
#inverse kinematics algorithm ends here    


cap = cv2.VideoCapture(0)
timer=0
i=1
def dis(a,b):
    return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def theta(a,b):
    angle= math.atan2(a[1]-b[1],a[0]-b[0])
    return int(math.degrees(angle))


sequence=0
g=[1,2]
t=[1,1]
o=[1,2]
step=1

m_dis=9999999999   
d_t=-4

cor=[1,2]

def click_event(event, bcx, bcy, flags, param):
    global cor
    if event == cv2.EVENT_LBUTTONDOWN:
        cor=[bcx,bcy]   
 
def image_process():
    global x,y,base_angle,g,t,o,m_dis,sequence,angles,d_t
    
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
    
    if corners:
        for corner,id in zip(corners, ids):            
            timer = cv2.getTickCount()
            frame = aruco.drawDetectedMarkers(frame, [corner])
            cx=int((corner[0][0][0]+corner[0][2][0]))//2
            cy=int((corner[0][0][1]+corner[0][2][1]))//2            
            cv2.circle(frame, (cx,cy), 5,(255, 0, 0),2)
            if id==3:
                g=[cx,cy]
            if id==1:
                t=[cx,cy]
            if id==2:
                o=[cx,cy]
                
    cv2.setMouseCallback('Display',click_event)
    
    if sequence==1 and dis(g,o)>60:
        
        th=theta(cor,o)
        if th>100: 
            base_angle=th-7
        elif th<80:
            base_angle=th+7
        else:
            base_angle=th
            
        if g[1]<o[1]-2:
            x=x-1
        if g[1]>o[1]+2:
            x=x+1
    elif sequence==1:
        print('sequence 1')
        sequence=2
        

    if sequence==2:
        angles[4]=pressure
        d_t=time.time()
        sequence=3
        print('sequence 2')
            
    if sequence==3 and time.time()-d_t>4:
        if x!=498:
            x=x-1
        if y!=350:
            y=y-1
        if x==498 and y==350:
            sequence=4
            print('sequence 3')
           
    if sequence==4 and dis(o,t)>20:        
        th=theta(cor,t)
        
        if th>100: 
            base_angle=th-7
        elif th<80:
            base_angle=th+7
        else:
            base_angle=th
            
        if o[1]<t[1]:
            x=x-0.5
        if o[1]>t[1]:
            x=x+0.5
            
        if y>401:
            y=y-1
        elif y<399:
            y=y+1
            
    elif sequence==4:
        angles[4]=0
        sequence=5
        d_t=time.time()
        print('sequence 4')

    if sequence==5 and time.time()-d_t>4:
        if y>500:
            y=y-1
        elif y<500:
            y=y+1
            
        if x>498:
            x=x-1
        elif x<498:
            x=x+1
        
        if x>498 and x<502 and y>498 and y<502:
            sequence=0
            base_angle=90
            print('finished')
             
    cv2.imshow('Display', frame)
   


    cv2.waitKey(1) 
        
    
while True: #main loop
    #t=time.time()
    image_process()
        
    follow_inverse()
    if base==0:
        rotate_gripper()
    else:
        rotate_base()
    #rotate_base()
    window.update()
    if start==1:
        #time.sleep(0.05)
        sendangles()
    #print(time.time()-t)


        
window.mainloop()
