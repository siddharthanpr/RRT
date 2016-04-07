#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import matplotlib.pyplot as plt
import numpy as np
draw=[]
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
	from openravepy import *
	from numpy import *

def waitrobot(robot):
	"""busy wait for robot completion"""
	while not robot.GetController().IsDone():
		time.sleep(0.01)

def tuckarms(env,robot):
	with env:
		jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
		robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
		robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
		robot.GetController().SetDesired(robot.GetDOFValues());
	waitrobot(robot)
	
def stringToFloatList(path):
	path = path.split('\n')
	for line in xrange(len(path)):
	  path[line] = path[line].split(',')
	  for i in xrange(len(path[line])):
		  path[line][i]=float(path[line][i])
	return path
	
def drawPath(path,robot,color):
	
	if type(path) is str: path = stringToFloatList(path)
	for i in path:
		robot.SetActiveDOFValues(i)
		draw.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.03,colors=color,drawstyle=1))
	
def stringToShortCutFormat(path,stepSize=0.2,ShortCutIters=200):
	path = path.split('\n')
	strPath = ''
	for ele in xrange(len(path)):
			if(ele!=len(path)-1):strPath+=(path[ele]+',')
			else:strPath+=(path[ele]+';')
	path = strPath;
	path+=str(stepSize);
	path+=';'
	path+=str(ShortCutIters);
	return path
	
if __name__ == "__main__":

	env = Environment()
	env.SetViewer('qtcoin')
	env.Reset()
	# load a scene from ProjectRoom environment XML file
	env.Load('hw3.env.xml')
	time.sleep(0.1)

	# 1) get the 1st robot that is inside the loaded scene
	# 2) assign it to the variable named 'robot'
	robot = env.GetRobots()[0]

	### INITIALIZE YOUR PLUGIN HERE ###
	RaveInitialize()
	RaveLoadPlugin('build/myplugin')
	RRTmodule = RaveCreateModule(env,'RRTmodule')
	### END INITIALIZING YOUR PLUGIN ###
   

	# tuck in the PR2's arms for driving
	tuckarms(env,robot);

	#set active joints
	jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
	robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

	# set start config
	startconfig = [-0.15,0.075,-1.008,0,0,0,0]
	robot.SetActiveDOFValues(startconfig);
	robot.GetController().SetDesired(robot.GetDOFValues());
	waitrobot(robot)

	with env:
		goalconfig = [0.449,-0.201,0,0,0,0,0]

		### YOUR CODE HERE ###
		#goalconfig = (-0.15,0.075,-1.008,8.88178e-16,0,-.3,0)s

		startPose = robot.GetActiveDOFValues()

		
		
		stepSize = 0.2;# using step size of 0 in RRT command does not smooth the path
		ShortCutIters = 200;
		startTime = time.time()
		#Please make your change here. Set the flag to 1.
		biRRTFlag =0; # 0 for unidirectional RRT. To run bidirectional RRT set this flag to 1

		path = RRTmodule.SendCommand('RRT goal %f,%f,%f,%f,%f,%f,%f; goalBias 0.15; step %f; metricWeight 900,800,60,25,0,13,0; shortCutStep 0; shortIters %f; biRRTFlag %f;'%tuple(goalconfig+[stepSize,ShortCutIters,biRRTFlag]))
		drawPath(path,robot,[1,0,0])
		RRTTime = time.time()-startTime;
		startTime = time.time()
		path = RRTmodule.SendCommand('shortCutSmooth '+stringToShortCutFormat(path,stepSize,ShortCutIters))
		smoothTime = time.time()-startTime;
		drawPath(path,robot,[0,0,1])
		print 'Time taken by RRT       ', RRTTime
		print 'Time taken for smoothing', smoothTime
		
		
		

		robot.SetActiveDOFValues(startPose)
		path = stringToFloatList(path)
		traj = RaveCreateTrajectory(env,'')
		traj.Init(robot.GetActiveConfigurationSpecification())

		for i in xrange(len(path)):
			traj.Insert(i,path[i])

		planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1)
		print 'duration of trajectory =',traj.GetDuration()
		
		

		robot.GetController().SetPath(traj)
		###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
		#print 'time', time.time()-start

		### END OF YOUR CODE ###
	waitrobot(robot)


	raw_input("Press enter to exit...")

