#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Tkinter import *
from team4.srv import*
import rospy
import math, copy
import Image
from PIL import ImageTk
import time
from std_msgs.msg import Int32
import tkMessageBox

modeChange=False
MODES = [('KeyBoard_MODE','K') , ('Drawing_MODE','D')]
xy=[]
Cur_Mode = None
s1 = []
size=0
x=0
y=0
roll=0
pitch=0
yaw=0
throttle =0
MODE1 = None
MoveBtn = None
app2_count=0
drawing_state = False
origin_xy = None
imagePath="/home/team4/catkin_ws/src/pypyleepy/image/"

class App(Frame):
   def __init__(self, master):
      Frame.__init__(self, master, relief=SUNKEN, bd=2)
      global imagePath
      self.imagePath=imagePath
      self.app2 = None
      self.canvas2 = None
      self.drone_icon = None
      self.menubar = Menu(self)
      menu = Menu(self.menubar, tearoff=0)
      self.menubar.add_cascade(label="File", menu=menu)
      menu = Menu(self.menubar, tearoff=0)
      self.menubar.add_cascade(label="Edit", menu=menu)
      menu.add_command(label="Cut", underline=0, accelerator="Ctrl+S", command=menu_cut)
      menu.add_command(label="Copy", underline=0, accelerator="Ctrl+C", command=menu_copy)
      menu.add_command(label="Paste", underline=0, accelerator="Ctrl+D", command=menu_paste)

      try:
         self.master.config(menu=self.menubar)
      except AttributeError:
         self.master.tk.call(master, "config", "-menu", self.menubar)

      global frame
      frame = Frame(master, width=500, height=500)
      frame.grid()
      frame.pack()

      global img, panel, roll, pitch, yaw, throttle
      img = ImageTk.PhotoImage(Image.open(self.imagePath+"original.png"))
      panel = Label(frame, image=img)
      panel.pack()

      self.button = Button(frame, text="QUIT", fg="red", command = frame.quit)
      self.button.pack(side=LEFT,padx=20) #
      self.var1 = StringVar()
      self.var1.set('')
      self.start_index=1

      for cur_text, mode in MODES:
         b = Radiobutton(frame , indicatoron=0, text=cur_text, variable=self.var1, value=mode,activeforeground='gray',
                         activebackground='black',highlightcolor = 'white',command=self.cb)
         if self.start_index == 1:
            b.configure(state='active' )
         b.pack(anchor=W, fill='x', side=TOP,padx=100, pady=5,)
         self.start_index = self.start_index +1

      self.w = Label(frame, text=self.var1.get())
      self.w.pack(anchor=E, fill='x', side=TOP,padx=10, pady=1)
      self.icon = Image.open(self.imagePath+"mark.png")
      [imageSizeWidth, imageSizeHeight] = self.icon.size
      self.newImageSizeWidth = int(imageSizeWidth)
      self.newImageSizeHeight = int(imageSizeHeight)
      self.icon = self.icon.resize((250,50), Image.ANTIALIAS)
      self.icon_img = ImageTk.PhotoImage(self.icon)
      self.canvas1 = Canvas(frame)
      self.canvas1.create_image(70,25, image = self.icon_img, anchor=W)
      self.canvas1.config(bg='white', width = 20, height = 50)
      self.canvas1.pack(side=LEFT,expand=True,fill=BOTH)

   def cb(self):
      self.w.configure(text="Current Mode is" + " ' "+ self.var1.get() + "' Mode ")
      global MoveBtn, modeChange, app2_count
      a=copy.copy(modeChange)

      if self.var1.get() == 'D':
         modeChange=True
         if modeChange != a:
            self.service_mode()
         if app2_count == 0 or self.app2 == None:
            self.app2 = Toplevel()
            self.canvas2=Canvas(self.app2, bg='white', width=500, height= 500) #width =x ,  height= y
            self.canvas2.bind("<B1-Motion>", self.mouseClick)
            self.canvas2.pack()
            self.app2.protocol("WM_DELETE_WINDOW", self.on_closing)
            MoveBtn = Button(self.app2, width=10,height=2,text="MOVE",bg="gray",command=self.BtnClick)
            MoveBtn.pack(anchor=SE)
            ClearBtn =Button(self.app2,width=10,height=2,text="Clear",bg="gray",command=self.ClearCanvas)
            ClearBtn.pack(anchor=SE)
            app2_count = app2_count +1
            self.app2.mainloop()
      else:
         if(app2_count != 0):
            modeChange=False
            if modeChange != a:
               self.service_mode()
            self.app2.destroy()
            app2_count = app2_count - 1

   def service_RC(self):
      global roll, pitch, yaw, throttle
      print "Service RC"
      rospy.wait_for_service('srv_RC')
      try:
         getRC=rospy.ServiceProxy('srv_RC',srvRC)
         ret=getRC(roll,pitch,yaw,throttle)
         return ret.result
      except rospy.ServiceException,e:
         print "Service call failed: %s" % e

   def service_mode(self):
      modeTrue=True;
      print "Service Mode Change"
      rospy.wait_for_service('srv_ChangeMode')
      try:
         getMode =rospy.ServiceProxy('srv_ChangeMode',srvChangeMode)
         ret = getMode(modeTrue)
         return ret.result
      except rospy.ServiceException, e:
         print "Service call failed: %s"%e

   def send_XYInfo(self):
      global xy,size
      print "Service Drawing"
      rospy.wait_for_service('srv_Paint')
      points=[]
      for (x,y) in xy:
         points.append(x)
         points.append(y)
      try:
         xy_Info=rospy.ServiceProxy('srv_Paint',srvPaint)
         result = xy_Info(size,points)
         return result.result
      except rospy.ServiceException, e:
         print "Service call failed: %s"%e

   def getDegree(self,index): #Get degree between two lines
      global xy
      pntA=copy.copy(xy[index])
      pntB=copy.copy(xy[index+2])
      org= copy.copy(xy[index+1])
      radian=math.atan2((pntA[1] - org[1]),(pntA[0] - org[0])) - math.atan2((pntB[1] - org[1]),(pntB[0] -pntA[0] ))
      degree= math.degrees(radian)
      if degree<0 : degree= degree*-1
      #print degree
      return degree

   def make_staright(self): # make straight between two lines
      global xy
      for index, coord in enumerate(xy):
         if index+2>=len(xy) : break
         if self.getDegree(index)<=180:
            del(xy[index+1])
            index=index-1

   def mouseClick(self,event):
      global x, y
      if x==0 and y==0 :
         xy.append([event.x, event.y])
      else:
         xy.append([event.x, event.y])
         self.canvas2.create_line(x,y,event.x,event.y,width=10,fill="gray")
      x = event.x
      y = event.y

   def BtnClick(self):
      global size, xy, drawing_state,origin_xy
      self.drone_icon = ImageTk.PhotoImage(file = self.imagePath+'drone_icon.png')
      origin_xy= copy.copy(xy)
      print ("clicked MOVE button")      #send xy list to drone and initialize lisy xy[],x,y ,canvas
      self.make_staright()
      size= len(xy)*2
      self.send_XYInfo()
      self.start_index = 1
      drawing_state = True

      for index,coord in enumerate(origin_xy):
         if index == len(origin_xy)-1:
            break
         self.canvas2.after(50)
         self.line_event(index)
         self.canvas2.update()
         self.drone_icon = None
         self.canvas2.pack()
      drawing_state = False
      self.canvas2_init()

   def line_event(self, index):
      global size, xy,origin_xy
      self.drone_icon = ImageTk.PhotoImage(file = self.imagePath+'drone_icon.png')
      self.canvas2.create_line(origin_xy[index][0], origin_xy[index][1], origin_xy[index+1][0], origin_xy[index+1][1],width=10, fill="cyan")
      self.canvas2.create_image(origin_xy[index][0], origin_xy[index][1], image = self.drone_icon)
      self.canvas2.pack()

   def ClearCanvas(self):
      self.canvas2_init()
      print "initialize canvas"

   def canvas2_init(self):
      global xy, x, y
      xy = []
      x = 0
      y = 0
      self.canvas2.delete("all")

   def on_closing(self, evnet = None):
      global app2_count, Cur_Mode,modeChange
      app2_count = app2_count -1
      Cur_Mode = 'K'
      modeChange=False;
      self.service_RC()
      self.app2.destroy()

def MODE1_binding():
   MODE1.bind("<Control-s>", menu_cut)
   MODE1.bind("<Control-c>", menu_copy)
   MODE1.bind("<Control-p>", menu_paste)
   MODE1.bind("<i>", Throttle_Up)
   MODE1.bind("<k>", Throttle_Down)
   MODE1.bind("<a>", Yaw_Left)
   MODE1.bind("<d>", Yaw_Right)
   MODE1.bind("<w>", Pitch_Down)
   MODE1.bind("<s>", Pitch_Up)
   MODE1.bind("<j>", Roll_Left)
   MODE1.bind("<l>", Roll_Right)

def menu_cut(evnet=None):  #cut ???,
   print("menu_cut")

def menu_copy(evnet=None):  #copy ???,
   print("menu_copy")

def menu_paste(event=None):
   print("menu_paste")

def Throttle_Up(event=None):
   global throttle, panel, img2,roll,pitch,yaw,throttle , drawing_state,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'2-up.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if throttle >= 0 and throttle <= 0.9:  # throttle?? 0 ~ 10 ??? ?
      throttle = throttle + 0.01
   else:
      print("imvalid throttle value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Yaw_Left(event=None):
   global yaw, panel, img2, roll, pitch, yaw, throttle, drawing_state,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'1-left.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if yaw > -180  and yaw <= 180:  # roll?? -70 ~ 70 ??
      yaw = yaw - 1
   else:
      print("imvalid roll value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Throttle_Down(event=None):  # throttle ???
   global throttle,panel, img2, roll,pitch,yaw,throttle ,drawing_state,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'2-down.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if throttle >= 0.1 and throttle <= 1:  # throttle?? 0 ~ 10 ??? ?
      if (int)((throttle)*100) == 1:
         throttle = 0
      else:
         throttle = throttle - 0.01
   else:
      print("imvalid throttle value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Yaw_Right(event=None):
   global yaw,panel, img2,roll,pitch,yaw,throttle, drawing_state,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'1-right.png'))
   panel.configure(image = img2)

   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if yaw >= -180 and yaw < 180:  # roll?? -70 ~ 70 ??
      yaw = yaw + 1
   else:
      print("imvalid roll value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Roll_Left(event=None):
   global roll,panel, img2,pitch,yaw,throttle , drawing_state,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'2-left.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if roll > -70  and roll <= 70:  # roll?? -70 ~ 70 ??
      roll = roll - 1
   else:
      print("imvalid roll value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Roll_Right(event=None):
   global roll,panel, img2, pitch, yaw, throttle,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'2-right.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if roll >= -70  and roll < 70:  # roll?? -70 ~ 70 ??
      roll = roll + 1
   else:
      print("imvalid roll value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Pitch_Down(event=None):
   global pitch,panel, img2, roll, yaw, throttle, imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'1-up.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if pitch >= -70 and pitch < 70:  # pitch?? -70~70
      pitch = pitch + 1
   else:
      print("imvalid roll value")
   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

def Pitch_Up(event=None):
   global pitch, panel, img2, roll, yaw,throttle, drawing_state ,imagePath
   img2 = ImageTk.PhotoImage(Image.open(imagePath+'1-down.png'))
   panel.configure(image = img2)
   if drawing_state:
      print("can't controll drone during darwing mode")
      return
   if pitch > -70  and pitch <= 70:  # pitch?? -70~70
      pitch = pitch - 1
   else:
      print("imvalid roll value")

   app.service_RC()
   print("roll:"+str(roll) +"  pitch:"+str(pitch)+"  yaw:"+str(yaw)+"  throttle : "+str(throttle))

if __name__ == "__main__":
   MODE1 = Tk()
   MODE1.geometry('550x620')
   app = App(MODE1)
   s1.append(MODE1)
   MODE1_binding()
   MODE1.mainloop()
