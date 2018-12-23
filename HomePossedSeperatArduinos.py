
# Learner Robot
import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import os
import time
from Tkinter import*
from visual import*
import matlab.engine

ser1= serial.Serial('COM4', baudrate=9600)
ser2= serial.Serial('COM5', baudrate=9600)
ser3= serial.Serial('COM7', baudrate=9600)

# To make desired robot links and algorithm for FABRIK for each link
class Link:
    a=vector(0, 0, 0)
    def __init__(self, x, y, z, length_, angle_, theta_):
        self.a= vector(x, y, z)
        self.length= length_
        self.angle= angle_
        self.theta= theta_

    def moveAngle(self, th1):
        dx= self.length*cos(th1)
        dy= self.length*sin(self.angle)
        dz= self.length*sin(th1)
        self.b= vector(self.a[0]+dx, self.a[1]+dy, self.a[2]+dz)
        
    def calculateB(self, th1):
        dx= self.length*cos(self.angle)*cos(th1)
        dy= self.length*sin(self.angle)
        dz= self.length*cos(self.angle)*sin(th1)
        self.b= vector(self.a.x+dx, self.a.y+dy, self.a.z+dz)        

    def show(self):
        dx= self.length*cos(self.angle)*cos(self.theta)
        dy= self.length*sin(self.angle)
        dz= self.length*cos(self.angle)*sin(self.theta)
        self.b= vector(self.a.x+dx, self.a.y+dy, self.a.z+dz)
        self.headed= arrow(pos=(self.a), axis=(self.b-self.a), shaftwidth=0.3, headwidth=0.3)
        

    def setTh(self, th):
        self.theta=th
        print(self.theta)
        dx= self.length*cos(self.angle)*cos(self.theta)
        dy= self.length*sin(self.angle)
        dz= self.length*cos(self.angle)*sin(self.theta)
        self.b= vector(self.a.x+dx, self.a.y+dy, self.a.z+dz)
        
    def follow(self, tx, ty, tz, th1, disp):
        self.b= vector(tx, ty, tz)
        direction = vector(self.b-self.a)
        self.angle= atan2(direction.y, direction.x)
        dx= self.length*cos(self.angle)*cos(th1)
        dy= self.length*sin(self.angle)
        dz= self.length*cos(self.angle)*sin(th1)
        self.a= vector(self.b.x-dx, self.b.y-dy, self.b.z-dz)
        if (disp==1):
            self.headed.pos= self.a
            self.headed.axis= self.b-self.a

    def setA(self, posi, th1, disp):
        self.a= vector(posi)
        self.headed.pos= vector(posi)
        direction= (self.b-self.a)                
        self.angle= atan2(direction.y, direction.x)
        self.calculateB(th1)
        if (disp==1):
            self.headed.pos= self.a
            self.headed.axis= self.b-self.a

    def calculatePhi(self):
        direction= self.b-self.a
        phi= atan2(direction.z, direction.x)
        return phi

    def calculateTh(self, initAng):
        direction= self.b-self.a
        ang= atan2(direction.y,direction.x)
        th= ang-initAng
        return th

# GUI to enter target coordinates or joint configurations
class Application(Frame):
    """A GUI application with a button. """
    x=0
    y=0
    z=0
    
    def __init__(self, master):
        """ Initialize the Frame"""
        Frame.__init__(self, master)
        self.grid()
        self.create_widgets()

    def create_widgets(self):
         """Create 3 buttons that do nothign"""
         #create first button, text and entry
         self.instruction= Label(self, text= "Enter the coordinate:")
         self.instruction.grid(row=1, column=0, columnspan=1, sticky=W)

         self.instruction= Label(self, text= "Enter the angle:")
         self.instruction.grid(row=3, column=0, columnspan=1, sticky=W)

         self.labelX= Label(self, text= "x")
         self.labelX.grid(row=0, column=1, columnspan=1, sticky=W)
         self.x1= Entry(self)
         self.x1.grid(row=1, column=1, sticky= W)

         self.labelY= Label(self, text= "y")
         self.labelY.grid(row=0, column=2, columnspan=1, sticky=W)
         self.y1= Entry(self)
         self.y1.grid(row=1, column=2, sticky= W)

         self.labelZ= Label(self, text= "z")
         self.labelZ.grid(row=0, column=3, columnspan=1, sticky=W)
         self.z1= Entry(self)
         self.z1.grid(row=1, column=3, sticky= W)
         
         self.button1= Button(self, text= "IKine", command= self.backward) # <= Chamith
         self.button1.grid(row=1, column=4, sticky=W)

         self.labelth1= Label(self, text= "th1")
         self.labelth1.grid(row=2, column=1, columnspan=1, sticky=W)
         self.th11= Entry(self)
         self.th11.grid(row=3, column=1, sticky= W)

         self.labelth2= Label(self, text= "th2")
         self.labelth2.grid(row=2, column=2, columnspan=1, sticky=W)
         self.th21= Entry(self)
         self.th21.grid(row=3, column=2, sticky= W)

         self.labelth3= Label(self, text= "th3")
         self.labelth3.grid(row=2, column=3, columnspan=1, sticky=W)
         self.th31= Entry(self)
         self.th31.grid(row=3, column=3, sticky= W)

         self.labelth4= Label(self, text= "th4")
         self.labelth4.grid(row=2, column=4, columnspan=1, sticky=W)
         self.th41= Entry(self)
         self.th41.grid(row=3, column=4, sticky= W)

         self.labelth5= Label(self, text= "th5")
         self.labelth5.grid(row=2, column=5, columnspan=1, sticky=W)
         self.th51= Entry(self)
         self.th51.grid(row=3, column=5, sticky= W)
         self.button2= Button(self, text= "FKine", command= self.forward) # <= Chamith
         self.button2.grid(row=3, column=6, sticky=W)

         self.button3= Button(self, text= "Mouse Click", command= self.mouseClick) # <= Chamith
         self.button3.grid(row=6, column=5, sticky=W)

         self.button4= Button(self, text= "Move Robot", command= self.moveBot) # <= Chamith
         self.button4.grid(row=6, column=2, sticky=W)

         self.button5= Button(self, text= "Home", command= self.home) # <= Chamith
         self.button5.grid(row=6, column=1, sticky=W)

         self.button6= Button(self, text= "Gripper Close", command= self.GripClose) # <= Chamith
         self.button6.grid(row=6, column=3, sticky=W)

         self.button7= Button(self, text= "Gripper Open", command= self.GripOpen) # <= Chamith
         self.button7.grid(row=6, column=4, sticky=W)        

    def backward(self): #<= Chamith
         """Display message based on the password typed in"""

         self.th11.delete(0, 'end')
         self.th21.delete(0, 'end')
         self.th31.delete(0, 'end')
         self.th41.delete(0, 'end')
         self.th51.delete(0, 'end')
             
         x = float(self.x1.get())
         y = float(self.y1.get())
         z = float(self.z1.get())

         d1= 14
         a2= 22
         d4= 22
         a5= 5
         
         scene = display(title='Robot movements', width=1024, height=1024, center=(8,4,0)) #set up the scene
         link= [0, 0, 0, 0]
         x_axis= arrow(pos=(0,0,0), axis=(60,0,0), shaftwidth=0.1, headwidth=0.3, color= color.red)
         y_axis= arrow(pos=(0,0,0), axis=(0,60,0), shaftwidth=0.1, headwidth=0.3, color= color.red)
         pos_z_axis= arrow(pos=(0,0,0), axis=(0,0,60), shaftwidth=0.1, headwidth=0.3, color= color.red)

         base= vector(0, 0, 0)
         link[0]= Link(0, 0, 0, 14, np.pi/2, 0)
         link[0].show()
         link[1]= Link(link[0].b.x, link[0].b.y, link[0].b.z, 22, np.pi/2, 0)
         link[1].show()
         link[2]= Link(link[1].b.x, link[1].b.y, link[1].b.z, 22, np.pi/2, 0)
         link[2].show()
         link[3]= Link(link[2].b.x, link[2].b.y, link[2].b.z, 5, np.pi/2, 0)
         link[3].show()

         total=3

         th1= atan2(z, x)

         if(scene.waitfor('click')):
             link[1].moveAngle(th1)
             link[2].a= link[1].b
             dy= link[2].length*sin(-np.pi/2)
             link[2].headed.pos= link[2].a
             link[2].b= (link[2].a.x, link[2].a.y+dy, link[2].a.z)
             link[3].a= link[2].b
             link[3].moveAngle(th1)
                                  
             for i in range(1):
                #link[total].follow(mouse.x, mouse.y, th1, 0)
                link[total].follow(x, y, z, th1, 0)
                
                for i in range (total, 1, -1):
                    th11=th1
                    if (i==3):
                        th11=0           
                    link[i-1].follow(link[i].a.x, link[i].a.y, link[i].a.z, th11, 0)
                        
                link[0].setA(base, 0, 0)

                for i in range (1, total):
                    th11=th1
                    if (i==2):
                        th11= 0            
                    link[i].setA(link[i-1].b, th1, 0)
                    link[3].setA(link[2].b, th1, 0)

             th1= link[1].calculatePhi()
             th2= link[1].calculateTh(0)
             th3= np.pi/2+link[2].calculateTh(0)-link[1].calculateTh(0)
             th4= 0
             th5= np.pi/2-(link[3].calculateTh(0)-link[2].calculateTh(0))

             eng = matlab.engine.start_matlab()
             th= eng.CapstoneRobot(x, y, z, th1, th2, th3, th4, th5)
     
             th1= th[0][0]
             th2= th[0][1]
             th3= th[0][2]
             th4= th[0][3]
             th5= th[0][4]

             link[1].headed.axis= (a2*cos(th1)*cos(th2), a2*sin(th2), a2*sin(th1)*cos(th2))
             link[2].headed.pos= (a2*cos(th1)*cos(th2), d1+a2*sin(th2), a2*sin(th1)*cos(th2))
             link[2].headed.axis= (d4*cos(th1)*sin(th2+th3), -d4*cos(th2+th3), d4*sin(th1)*sin(th2+th3))
             link[3].headed.pos= (a2*cos(th1)*cos(th2)+d4*cos(th1)*sin(th2+th3), a2*sin(th2)+d1-d4*cos(th2+th3), a2*sin(th1)*cos(th2)+d4*sin(th1)*sin(th2+th3))
             link[3].headed.axis= (5*cos(th5)*(sin(th1)*sin(th4) + cos(th4)*cos(th1)*cos(th2+th3))+ 5*cos(th1)*sin(th5)*sin(th2+th3), - 5*sin(th5)*cos(th2+th3)+ 5*cos(th4)*cos(th5)*sin(th2+th3),- 5*cos(th5)*(cos(th1)*sin(th4) - cos(th4)*sin(th1)*cos(th2+th3))+ 5*sin(th1)*sin(th5)*sin(th2+th3))    
             
             self.th11.insert(0, str(th1))
             self.th21.insert(0, str(th2-np.pi/2))
             self.th31.insert(0, str(th3-np.pi/2))
             self.th41.insert(0, str(th4))
             self.th51.insert(0, str(th5-np.pi/2))
             
             print(link[3].headed.axis+link[3].headed.pos) 
    def mouseClick(self):
        
             self.th11.delete(0, 'end')
             self.th21.delete(0, 'end')
             self.th31.delete(0, 'end')
             self.th41.delete(0, 'end')
             self.th51.delete(0, 'end')
             self.x1.delete(0, 'end')
             self.y1.delete(0, 'end')
             self.z1.delete(0, 'end')
             
             scene = display(title='Robot movements', width=1024, height=1024, center=(8,4,0)) #set up the scene
             link= [0, 0, 0, 0]
             x_axis= arrow(pos=(0,0,0), axis=(60,0,0), shaftwidth=0.1, headwidth=0.3, color= color.red)
             y_axis= arrow(pos=(0,0,0), axis=(0,60,0), shaftwidth=0.1, headwidth=0.3, color= color.red)
             pos_z_axis= arrow(pos=(0,0,0), axis=(0,0,60), shaftwidth=0.1, headwidth=0.3, color= color.red)

             d1= 14
             a2= 22
             d4= 22
             a5= 5

             num=3
             base= vector(0, 0, 0)
             link[0]= Link(0, 0, 0, 14, np.pi/2, 0)
             link[0].show()
             link[1]= Link(link[0].b.x, link[0].b.y, link[0].b.z, 22, np.pi/2, 0)
             link[1].show()
             link[2]= Link(link[1].b.x, link[1].b.y, link[1].b.z, 22, np.pi/2, 0)
             link[2].show()
             link[3]= Link(link[2].b.x, link[2].b.y, link[2].b.z, 5, np.pi/2, 0)
             link[3].show()


             scene.waitfor('click')
             mouse= vector(scene.mouse.pos)
             x= mouse.x
             y= mouse.y
             z= mouse.z
                          
             self.x1.insert(0, str(x))
             self.y1.insert(0, str(z))
             self.z1.insert(0, str(y))

             total=3
                                 
             for i in range(1600):
                #link[total].follow(mouse.x, mouse.y, 0, 0, 1)
                link[total].follow(x, y, 0, 0, 1)
                
                for i in range (total, 1, -1):          
                    link[i-1].follow(link[i].a.x, link[i].a.y, 0, 0, 1)
                        
                link[0].setA(base, 0, 1)

                for i in range (1, total):
          
                    link[i].setA(link[i-1].b, 0, 1)
                    link[3].setA(link[2].b, 0, 1)

             th1= 0
             th2= link[1].calculateTh(0)-np.pi/2
             th3= link[2].calculateTh(0)-link[1].calculateTh(0)
             th4= 0
             th5= -(link[3].calculateTh(0)-link[2].calculateTh(0))

             print(link[3].b)
             
             self.th11.insert(0, str(th1))
             self.th21.insert(0, str(th2))
             self.th31.insert(0, str(th3))
             self.th41.insert(0, str(th4))
             self.th51.insert(0, str(th5))

    
        
    def forward(self):
         self.x1.delete(0, 'end')
         self.y1.delete(0, 'end')
         self.z1.delete(0, 'end')
         th=[0, 0, 0, 0, 0]
         th1 = float(self.th11.get())
         th2 = float(self.th21.get())
         th3 = float(self.th31.get())
         th4 = float(self.th41.get())
         th5 = float(self.th51.get())

         scene = display(title='Robot movements', width=1024, height=1024) #set up the scene
         link= [0, 0, 0, 0]
         x_axis= arrow(pos=(0,0,0), axis=(60,0,0), shaftwidth=0.1, headwidth=0.3, color= color.red)
         y_axis= arrow(pos=(0,0,0), axis=(0,60,0), shaftwidth=0.1, headwidth=0.3, color= color.red)
         d1= 14
         a2= 22
         d4= 22
         a5= 5

         base= vector(0, 0, 0)
         link[0]= Link(0, 0, 0, 14, np.pi/2, 0)
         link[0].show()
         link[1]= Link(link[0].b.x, link[0].b.y, link[0].b.z, 22, np.pi/2, 0)
         link[1].show()
         link[2]= Link(link[1].b.x, link[1].b.y, link[1].b.z, 22, np.pi/2, 0)
         link[2].show()
         link[3]= Link(link[2].b.x, link[2].b.y, link[2].b.z, 5, np.pi/2, 0)
         link[3].show()

         th2= th2+np.pi/2
         th3= th3+np.pi/2
         th5= th5+np.pi/2
         if (scene.waitfor('click')):
           link[1].headed.axis= (a2*cos(th1)*cos(th2), a2*sin(th2), a2*sin(th1)*cos(th2))
           link[2].headed.pos= (a2*cos(th1)*cos(th2), d1+a2*sin(th2), a2*sin(th1)*cos(th2))
           link[2].headed.axis= (d4*cos(th1)*sin(th2+th3), -d4*cos(th2+th3), d4*sin(th1)*sin(th2+th3))
           link[3].headed.pos= (a2*cos(th1)*cos(th2)+d4*cos(th1)*sin(th2+th3), a2*sin(th2)+d1-d4*cos(th2+th3), a2*sin(th1)*cos(th2)+d4*sin(th1)*sin(th2+th3))
           link[3].headed.axis= (5*cos(th5)*(sin(th1)*sin(th4) + cos(th4)*cos(th1)*cos(th2+th3))+ 5*cos(th1)*sin(th5)*sin(th2+th3), - 5*sin(th5)*cos(th2+th3)+ 5*cos(th4)*cos(th5)*sin(th2+th3),- 5*cos(th5)*(cos(th1)*sin(th4) - cos(th4)*sin(th1)*cos(th2+th3))+ 5*sin(th1)*sin(th5)*sin(th2+th3))    

         self.x1.insert(0, (22*cos(th1)*cos(th2) + 5*cos(th5)*(sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2+th3)) + 22*cos(th1)*sin(th2+th3) + 5*cos(th1)*sin(th5)*sin(th2+th3)))
         self.z1.insert(0, (22*sin(th2) - 22*cos(th2+th3) - 5*sin(th5)*cos(th2+th3) + 5*cos(th4)*cos(th5)*sin(th2+th3) + 14))
         self.y1.insert(0,  22*cos(th2)*sin(th1) - 5*cos(th5)*(cos(th1)*sin(th4) - cos(th4)*sin(th1)*cos(th2+th3)) + 22*sin(th1)*sin(th2+th3) + 5*sin(th1)*sin(th5)*sin(th2+th3))

    def moveBot(self):
         Angles1= self.th11.get().encode()+"@"+self.th21.get().encode()
         ser1.write(Angles1)

         Angles2= self.th31.get().encode()+"$"+self.th41.get().encode()
         ser2.write(Angles2)

         Angles3= self.th51.get().encode()
         ser3.write(Angles3)

    def home(self):
        self.th11.delete(0, 'end')
        self.th21.delete(0, 'end')
        self.th31.delete(0, 'end')
        self.th41.delete(0, 'end')
        self.th51.delete(0, 'end')
        self.x1.delete(0, 'end')
        self.y1.delete(0, 'end')
        self.z1.delete(0, 'end')
             
        ser1.write("0@0")
        ser2.write("0$0")
        ser3.write("0")

        self.th11.insert(0, str(0))
        self.th21.insert(0, str(0))
        self.th31.insert(0, str(0))
        self.th41.insert(0, str(0))
        self.th51.insert(0, str(0))

        self.x1.insert(0, str(0))
        self.y1.insert(0, str(0))
        self.z1.insert(0, str(0))

    def GripClose(self):
        ser3.write("GC")

    def GripOpen(self):
        ser3.write("GO") 
       
        
    
while 1:
    root= Tk()
    root.title("Robot Learner")
    root.geometry("800x150")
               
    app= Application(root)
    root.mainloop()
    root.quit()


