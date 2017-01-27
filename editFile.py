#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from constants import *
import math
from time import time, sleep
from threading import Thread


import os
os.chdir("/home/pi/Documents/results")
print ("File open at ",os.getcwd())
my_file = open(time.strftime("%B_%d_%H_%M_%S"), 'w')
my_file.write("time(s)\tmpu6050\testimated_incl\tU_mot\tpos_way")

my_file.write("{0}\t{1}\t{2}\t{3}\t{4}\n".format(time.time()-t_begin_program, i2cdevice.get_gyro_x(), i2cdevice.get_estimated_incl(),  m/U_alim, loc$


class _FileWrite(Tread):
	def __init__(self, file, bufString):
		Tread.__init__(self)
		self.file = file
		self.buf = bufString

	def run(self):
		self.file.write(self.buf)

class FileEdit():
	def __init__(self, path = "/home/pi/Documents/results", name = time.strftime("%B_%d_%H_%M_%S"):
		self.path = path
		self.name = name + ".txt"
		self.buffer = ""
		self.file = open(time.strftime("%B_%d_%H_%M_%S"), 'w')
		self.threads = list()

	def saveString(self, instring):
		#si pas fini fill buffer
		#sinon go thread 
		self.threads.add(_FileWrite(self.file, self.buffer))
		#run
		self.buffer = ""
