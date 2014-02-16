#!/usr/bin/python
min_factor = 0.08 #Minimum scaling factor
max_factor = 0.12 #Maximum scaling factor
leftmax_rotate_angle = -30.0 #Left-most maximum rotation angle, in degrees
rightmax_rotate_angle = 30.0 #Right-most maximum rotation angle, in degrees



import cv2
import os
from os import listdir
from os.path import isfile,join
import random
import time
import pp
import math,sys
import pdb
import numpy as np
from optparse import OptionParser
parser = OptionParser("createtrainingimages.py [options]")
parser.add_option("--Trials",dest="Trials",default="1")
parser.add_option("--Patterns",dest="Patterns",default="5")
parser.add_option("--UseParallel",dest="UseParallel",default='True')
(opts,args) = parser.parse_args()
Trials = int(opts.Trials)
if opts.UseParallel=='False':
	UseParallel=False
elif opts.UseParallel=='True':
	UseParallel=True
Patterns=int(opts.Patterns)
ppservers = ()
job_server = pp.Server(4,ppservers=ppservers)


masterimage_dir = os.getcwd()+'/../media/MasterImages/'
trainimage_dir = os.getcwd()+'/../media/SimulatedImages/'+time.strftime("%Y_%m_%d_%H_%M_%S")+"/"
os.mkdir(trainimage_dir)
environmentimage_dir = os.getcwd()+'/../media/EnvironmentImages/'
avg_seqtime = 0.0
avg_partime = 0.0
masterimage_names = [ f for f in listdir(masterimage_dir) if isfile(join(masterimage_dir,f)) ]
masterimage_paths = [''] * len(masterimage_names)


environmentimage_names = [f for f in listdir(environmentimage_dir) if isfile(join(environmentimage_dir,f))]
environmentimage_paths = [''] * len(environmentimage_names)
for i in range(len(environmentimage_names)):
	environmentimage_paths[i] = environmentimage_dir + environmentimage_names[i]
env_image = cv2.imread(environmentimage_paths[0])

for i in range(len(environmentimage_names)):
	environmentimage_paths[i] = environmentimage_dir + environmentimage_names[i]
	environmentimage_names[i] = environmentimage_names[i][0:environmentimage_names[i].find('.')]
for i in range(len(masterimage_names)):
		masterimage_paths[i] = masterimage_dir + masterimage_names[i]
		masterimage_names[i] = masterimage_names[i][0:masterimage_names[i].find('.')]

def deleteimages():
	files = listdir(trainimage_dir)
	for f in files:
		os.remove(trainimage_dir+f)	
def createtrainimages(masterimage_index,paths,names,patterns,minf,maxf,newdir,environmentdir,envpaths):
	master_images = []
	
	env_image = cv2.imread(envpaths[0])	
	for i in range(len(names)):
		master_images.append(cv2.imread(paths[i]))
		#height,width = env_image.shape[:2]
		#master_images[i] = cv2.resize(master_images[i],(width,height))

	for trainimage_index in range(patterns):

		#Load Environment Image
		pick = int(random.uniform(0,len(envpaths)-1))
		
		env_image = cv2.imread(envpaths[pick])
		#Drop/Scale Target Image into Environment Image 

		'''Resize Target Image'''
		factor_x = random.uniform(minf,maxf)
		factor_y = factor_x#random.uniform(minf,maxf)
		size_x,size_y = int(master_images[masterimage_index].shape[1]*factor_x),int(master_images[masterimage_index].shape[0]*factor_y)
		tempimage = cv2.resize(master_images[masterimage_index],(size_x,size_y))
		
		cur_size_x,cur_size_y = tempimage.shape[1],tempimage.shape[0]
		center_y = int(random.uniform(tempimage.shape[0]/2,env_image.shape[0]-tempimage.shape[0]/2))
		center_x = int(random.uniform(tempimage.shape[1]/2,env_image.shape[1]-tempimage.shape[1]/2))
		
		newimage = env_image
		y_min = center_y-math.floor(tempimage.shape[0]/2)
		y_max = center_y+math.floor(tempimage.shape[0]/2)
		x_min = center_x-math.floor(tempimage.shape[1]/2)
		x_max = center_x+math.floor(tempimage.shape[1]/2)
		tempsize = newimage[y_min:y_max,x_min:x_max,:]
		height,width = tempsize.shape[:2]
		tempimage = cv2.resize(tempimage,(width,height))
		newimage[y_min:y_max,x_min:x_max,:] = tempimage
		
		
		#rotateangle = random.uniform(leftmax_rotate_angle,rightmax_rotate_angle)
		#rotatematrix = cv2.getRotationMatrix2D((0,0),rotateangle,.3)
		#tempimage = cv2.warpAffine(tempimage,rotatematrix,(cur_size_y*3,cur_size_x*3))
		newpath = newdir+names[masterimage_index]+'_'+str(trainimage_index)+'_'+str(center_y)+'_'+str(center_x)+'.png'
		cv2.imwrite(newpath,newimage)
for t in range(Trials):	
	
	#Sequential Code
	if Trials > 1:
		print 'Trial: {}'.format(t+1)
	
	
	#Parallel Code
	if UseParallel:
		#deleteimages()
		start_time = time.time()

		masterimage_range = range(len(masterimage_names))
		jobs = [(index, job_server.submit(func=createtrainimages,args=(index,masterimage_paths,masterimage_names,Patterns,min_factor,max_factor,trainimage_dir,environmentimage_dir,environmentimage_paths), modules=("cv2","random","math",))) for index in masterimage_range]
		for index,job in jobs:
			job()
		elapsed_time = time.time() - start_time	
		print 'Elapsed Time (Parallel): {:.4f} Seconds'.format(elapsed_time)
		job_server.print_stats()
		avg_partime = avg_partime + elapsed_time
	else:
		#deleteimages()
		start_time = time.time()
		for index in range(len(masterimage_names)):
			createtrainimages(index,masterimage_paths,masterimage_names,Patterns,min_factor,max_factor,trainimage_dir,environmentimage_dir,environmentimage_paths)
		elapsed_time = time.time() - start_time	
		print 'Elapsed Time (Sequential): {:.4f} Seconds'.format(elapsed_time)
		avg_seqtime = avg_seqtime+elapsed_time
		
if UseParallel:
	print 'Average Elapsed Time for Parallel Execution: {:.4f} Seconds'.format(avg_partime/Trials)
else:
	print 'Average Elapsed Time for Sequential Execution: {:.4f} Seconds'.format(avg_seqtime/Trials)

	

	



