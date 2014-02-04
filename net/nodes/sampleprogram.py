#!/usr/bin/python
from icarus_helper import *
from array import array
import numpy as np
import xml.etree.ElementTree as ET
import pdb
import cv2
import os
from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.tools.validation import *
from pybrain.datasets.classification import *
from pybrain.utilities import *
#pdb.set_trace()
#tree = ET.parse('../trained_nets/classnet.xml')
#root = tree.getroot()

Answers = ['ChiefSecurity','MinistryTorture','None','SecurityCompound']
'''classnet = neuralnetwork(Answers)		
for i in range(0,len(root[0])):
  layername = root[0][i].text
  neurons = int(root[0][i][0].text)
  transferfunction = root[0][i][1].text
  biasweights_str = root[0][i][2].text
  weights_str = root[0][i][3].text
  biasweights = np.array(np.mat(biasweights_str))
  weights = np.array(np.mat(weights_str))
  
  
  classnet.addlayer(i,layername,neurons,weights,biasweights,transferfunction)
'''


#imagefile = 
#imagecontdir = "/home/dgitz/fuerte_workspace/sandbox/icarus_net/net/media/ModImages/"
imagecontdir = "/home/dgitz/catkin_ws/src/icarus_net/net/media/ModImages/"
index = 0
sumcorrect = 0
accuracy = 0
first = True

#Load Training Samples
imagefolders = os.listdir(imagecontdir)
for m in range(0,len(imagefolders)):
	
	imagefiles = os.listdir(imagecontdir + imagefolders[m])
	imagefiles.sort()
	for i in range(0,len(imagefiles)):
		img = cv2.imread(imagecontdir+imagefolders[m]+'/'+imagefiles[i],cv2.CV_LOAD_IMAGE_GRAYSCALE)
		inputvector = img.flatten()
		if first:
			first = False
			#classnet_allsets = SupervisedDataSet(len(inputvector),len(Answers))
			classnet_allsets = ClassificationDataSet(len(inputvector),class_labels = Answers)
		#classnet_allsets.addSample(inputvector,m)
		classnet_allsets.appendLinked(inputvector,[m])

#pdb.set_trace()	
classnet_allsets._convertToOneOfMany(bounds=[0,1])

classnet_testsets,classnet_trainsets = classnet_allsets.splitWithProportion(.25)

classnet = buildNetwork(len(inputvector),3,len(Answers),bias=True)
trainer = BackpropTrainer(classnet,dataset=classnet_trainsets,momentum=.25,verbose=True,weightdecay=.1)
for i in range(20):
	trainer.trainEpochs(1)
	#pdb.set_trace()
	train_error = 100*percentError(trainer.testOnClassData(),classnet_trainsets['class'])
	test_error = 100*percentError(trainer.testOnClassData(dataset=classnet_testsets),classnet_testsets['class'])
	
	print "epoch: {} train error: {:.2f}% test error: {:.2f}%".format(trainer.totalepochs,train_error,test_error) 		
pdb.set_trace()	
#Test Network		
'''for inputvector in range(0,len(imagefolders)):
	pdb.set_trace()
	#output = classnet.calcnet(inputvector)
	if output == imagefolders[m]:
		print 'Testing on Pattern: ' + str(index) + ' Succeeded.'
		sumcorrect = sumcorrect + 1
	else:
		print 'Testing on Pattern: ' + str(index) + ' Failed.'
	
	
	index = index + 1
	accuracy = 100*sumcorrect/index
	print 'Accuracy So Far: ' + str(accuracy) + '%'
'''





