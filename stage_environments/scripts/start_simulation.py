#!/usr/bin/env python

import Tkinter as tk
import tkMessageBox
import ConfigParser
import thread
from Tkinter import *
from ttk import *
import Image, tkFileDialog
import numpy as np
import sys, time, os, glob, shutil, math, datetime


Maps = ['DISB1', 'DISlabs', 'Rive1', 'Rockin2014']   

Default_initPoses = { 'DISB1': ['11.8 2 270', '2 2 0'],  'DISlabs': ['9.7 7.3 90', '10.7 7.3 0'], \
  'Rive1': ['10 23 0'], 'Rockin2014': ['-3.5 6.2 270', '0 0 0'] }

Robots = [ 'sapienzbot', 'diago' ]

Localization = ['none', 'amcl', 'glocalizer']

Navigation = ['none','move_base','gradient_based_nav']


LASER_TOPIC = 'scan' 
LASER_FRAME = 'laser_frame'


def run_simulation(map_name,robot,initpose,localization,navigation,joystick,keyboard,rviz):

  print 'Run simulation: map:%s robot:%s init:%s loc: %s nav: %s joy:%s keyb:%s rviz:%s' % (map_name,robot,initpose,localization,navigation,joystick,keyboard,rviz)
  
  output_worldfile="../maps/AUTOGEN_%s_%s.world" % (map_name,robot)

  no_localizer=False
  run_amcl=False
  run_glocalizer=False
  run_move_base=False
  run_gradient_based_navigation=False

  if (localization=='amcl'):
    run_amcl=True
  elif (localization=='glocalizer'):
    run_glocalizer=True
  else:
    no_localizer=True
    
  if (navigation=='move_base'):
    run_move_base=True
  elif (navigation=='gradient_based_nav'):
    run_gradient_based_navigation=True

  if (robot=='sapienzbot'):
    robottype='erratic'
  if (robot=='diago'):
    robottype='segway'

  pp = initpose.split()
  INITPOSE_X=float(pp[0])
  INITPOSE_Y=float(pp[1])
  INITPOSE_TH=float(pp[2]) # deg
  INITPOSE_TH_RAD=INITPOSE_TH*math.pi/180.0 # rad

  # Create the world
  cmd = './create_world.py %s %s %s %f %f %f %s' % (map_name,robottype,robot,INITPOSE_X,INITPOSE_Y,INITPOSE_TH,output_worldfile)
  #print cmd
  os.system(cmd)

  # Start the simulated environment

  cmd = 'xterm -e roslaunch stage_environments stage_map.launch world_file:=%s robot_type:=%s laser_topic:=%s laser_frame:=%s &' % (output_worldfile, robottype, LASER_TOPIC, LASER_FRAME)
  #print cmd
  os.system(cmd)
  os.system('sleep 5')
  
  
  # Start the robot

  amcl_str = ''
  if (run_amcl):
    amcl_str = "use_amcl:=true"

  glocalizer_str = ''
  if (run_glocalizer):
    glocalizer_str = "use_glocalizer:=true"

  no_localizer_str = ''
  if (no_localizer):
    no_localizer_str = "map_server:=false"

  gbn_str = ''
  if (run_gradient_based_navigation):
    gbn_str = "use_gradient_based_navigation:=true"

  move_base_str = ''
  if (run_move_base):
    move_base_str = "use_move_base:=true"

 

  cmd = 'xterm -e roslaunch stage_environments stage_robot.launch robot_name:=%s map_name:=%s initial_pose_x:=%f initial_pose_y:=%f initial_pose_a:=%f laser_topic:=%s laser_frame:=%s %s %s %s %s %s &' % (robot, map_name, INITPOSE_X, INITPOSE_Y, INITPOSE_TH_RAD, LASER_TOPIC, LASER_FRAME, amcl_str, glocalizer_str, gbn_str, move_base_str, no_localizer_str )
  #print cmd
  if (run_amcl or run_glocalizer or run_gradient_based_navigation):
    os.system(cmd)
    os.system('sleep 10')


  if (joystick==1):
    # Drive the robot with joystick 
    cmd = 'xterm -e "roslaunch stage_environments joystick.launch robot_name:=%s ' % (robot)
    if (run_gradient_based_navigation):
      cmd = cmd + 'cmd_vel_topic:=joystick_cmd_vel" &'
    else:
      cmd = cmd + 'cmd_vel_topic:=cmd_vel" &' 
    
  if (keyboard==1):
    # Drive the robot with keyboard
    cmd = 'xterm -e "rosrun gradient_based_navigation keyboard_control '
    if (run_gradient_based_navigation):
      cmd = cmd + 'joystick_cmd_vel:=/%s/joystick_cmd_vel" &' % (robot)
    else:
      cmd = cmd + 'joystick_cmd_vel:=/%s/cmd_vel" &' % (robot)

  if (joystick==1) or (keyboard==1):
    print cmd
    os.system(cmd)
    
  
  if (rviz==1):
    # Start rviz
    cmd = 'xterm -e rosrun rviz rviz -d `rospack find stage_environments`/config/%s/rviz/%s.rviz &' % (robot,robot)
    os.system(cmd)



class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.first_map_selected = True
        self.initUI()
        


    def map_selected(self, *args):
	current_map = self.map_name_ddm.get()
	#print 'Current map: ',current_map
	self.initpose_list = Default_initPoses[current_map]
	#print self.initpose_list
	if (self.first_map_selected):
	  self.first_map_selected = False
	  try:
	    last_init=self.oldConfigs["initpose"]
	  except:
	    last_init=self.initpose_list[0]
	else:
	    last_init=self.initpose_list[0]
	self.initpose_ddm.set(last_init)
	_row = 1
	_col = 2
	self.opt_poses = tk.OptionMenu(self, self.initpose_ddm, *self.initpose_list)
	self.opt_poses.grid(sticky=W, row=_row, column=_col, pady=4, padx=5)

	
    def initUI(self):
        self.loadOldConfig()

        self.parent.title("Simulation Launcher")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        
        #self.columnconfigure(1, weight=1)
        #self.columnconfigure(3, pad=7)
        #self.rowconfigure(3, weight=1)
        #self.rowconfigure(7, pad=7)
        
        self.map_name_ddm = StringVar(self)
        self.robot_ddm = StringVar(self)
        self.initpose_ddm = StringVar(self)


        _row = 0
        # Map
        lbl = Label(self, text="Map")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)                
        self.map_name_list = Maps
        self.map_name_ddm.trace("w", self.map_selected)
        try:
            lastmap_name=self.oldConfigs["map"]
        except:
            lastmap_name=self.map_name_list[0]
        self.map_name_ddm.set(lastmap_name)
        tk.OptionMenu(self, self.map_name_ddm, *self.map_name_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Robot
        lbl = Label(self, text="Robot & Pose")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)
        self.robot_list = Robots
        try:
            lastrobot=self.oldConfigs["robot"]
        except:
            lastrobot=self.robot_list[0]
        self.robot_ddm.set(lastrobot)
        tk.OptionMenu(self, self.robot_ddm, *self.robot_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Localization
        lbl = Label(self, text="Localization")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)
        self.loc_list = Localization
        self.loc_ddm = StringVar(self)
        try:
            lastlocmode=self.oldConfigs["localization"]
        except:
            lastlocmode=self.loc_list[0]
        self.loc_ddm.set(lastlocmode)
        tk.OptionMenu(self, self.loc_ddm, *self.loc_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Navigation
        lbl = Label(self, text="Navigation")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        self.nav_list = Navigation
        self.nav_ddm = StringVar(self)
        try:
            lastterm=self.oldConfigs["navigation"]
        except:
            lastterm=self.nav_list[0]
        self.nav_ddm.set(lastterm)
        tk.OptionMenu(self, self.nav_ddm, *self.nav_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Joystick
	self.joystick_ddm = IntVar(self)
	try:
	  last_joystick=self.oldConfigs["joystick"]
	except:
	  last_joystick=0
	self.joystick_ddm.set(last_joystick)
	self.Chk_joystick = tk.Checkbutton(self, text='Joystick', variable=self.joystick_ddm)      
	self.Chk_joystick.grid(sticky=W, row=_row, column=0, pady=10, padx=5)

	# Keyboard
	self.keyboard_ddm = IntVar(self)
	try:
	  last_keyboard=self.oldConfigs["keyboard"]
	except:
	  last_keyboard=0
	self.keyboard_ddm.set(last_keyboard)
	self.Chk_keyboard = tk.Checkbutton(self, text='Keyboard', variable=self.keyboard_ddm)      
	self.Chk_keyboard.grid(sticky=W, row=_row, column=1, pady=10, padx=5)
      
	# Rviz
	self.rviz_ddm = IntVar(self)
	try:
	  last_rviz=self.oldConfigs["rviz"]
	except:
	  last_rviz=0
	self.rviz_ddm.set(last_rviz)
	self.Chk_rviz = tk.Checkbutton(self, text='Rviz', variable=self.rviz_ddm)      
	self.Chk_rviz.grid(sticky=W, row=_row, column=2, pady=10, padx=5)
      
      
	_row = _row + 1
	# Buttons
        launchButton = Button(self, text="Start",command=self.launch_script)
        launchButton.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        
        launchButton = Button(self, text="Quit",command=self.kill_demo)
        launchButton.grid(sticky=W, row=_row, column=1, pady=4, padx=5)
        
    
    def launch_script(self):
        self.saveConfigFile();
        thread.start_new_thread( run_simulation, (self.map_name_ddm.get(), self.robot_ddm.get(), self.initpose_ddm.get(), self.loc_ddm.get(), self.nav_ddm.get(), self.joystick_ddm.get(), self.keyboard_ddm.get(), self.rviz_ddm.get()) )

    
    def quit(self):
      self.saveConfigFile()
      #self.parent.destroy()
      
    def kill_demo(self):
      os.system("./quit.sh &")
      
      
    def saveConfigFile(self):
      f = open('lastConfigUsed', 'w')
      f.write("[Config]\n")
      f.write("map: %s\n"%self.map_name_ddm.get())
      f.write("robot: %s\n"%self.robot_ddm.get())
      f.write("initpose: %s\n"%self.initpose_ddm.get())
      f.write("localization: %s\n"%self.loc_ddm.get())
      f.write("navigation: %s\n"%self.nav_ddm.get())
      f.write("joystick: %s\n"%self.joystick_ddm.get())
      f.write("keyboard: %s\n"%self.keyboard_ddm.get())
      f.write("rviz: %s\n"%self.rviz_ddm.get())
      f.close()


    def loadOldConfig(self):
      try:
        self.oldConfigs = {}
        self.Config = ConfigParser.ConfigParser()
        self.Config.read("lastConfigUsed")
        for option in self.Config.options("Config"):
          self.oldConfigs[option] = self.Config.get("Config", option)
      except:
        print "Could not load config file"


    


def main():

  if (len(sys.argv)==1):
    root = tk.Tk()
    f = DIP(root)
    root.geometry("360x320+0+0")
    root.mainloop()
    f.quit()

  elif (len(sys.argv)>7):
    map_name = sys.argv[1]
    robot_name = sys.argv[2]
    init_pose = sys.argv[3]+' '+sys.argv[4]+' '+sys.argv[5]
    localization = sys.argv[6]
    navigation = sys.argv[7]
    i=8
    joystick=0
    keyboard=0
    rviz=0
    while (i<len(sys.argv)):
      if (sys.argv[i]=='joystick'):
	joystick=1
      if (sys.argv[i]=='keyboard'):
	keyboard=1
      if (sys.argv[i]=='rviz'):
	rviz=1
      i=i+1
    print "Starting the simulation..."
    run_simulation(map_name, robot_name, init_pose, localization, navigation, joystick, keyboard, rviz)
    print "Use the following command to quit the simulation"
    print "  `rospack find stage_environments`/scripts/quit.sh"
    
  else:
    print "Use: ",sys.argv[0]
    print " or  ",sys.argv[0],' <map_name> <robot_name> <init_pose> <localization> <navigation> [joystick] [keyboard] [rviz]'
  


if __name__ == '__main__':
    main()

