#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import time
import matplotlib.pyplot as plt

t0_graph = time.time()
curves = list()

def new_curve(name):
	a_curve = list()
	ords = list()
	abss = list()
	a_curve.append(abss)
	a_curve.append(ords)
	a_curve.append(name)
	global curves
	curves.append(a_curve)
	return (len(curves)-1)

def set_t0(t = time.time()):
	global t0_graph
	t0_graph = t

def set_point(index_curve, val, t = time.time()):
	curves[index_curve][0].append(t)
	curves[index_curve][1].append(val)

def show():
	for a_curve in curves:
		plt.plot(a_curve[0], a_curve[1], label = a_curve[2])
	plt.legend()
	plt.show()

if __name__ == "__main__":
	#test there:
	from math import sin, pi
	idx = list()
	idx.append(new_curve("sin"))
	idx.append(new_curve("cos"))
	print('go go go')
	for i in range(0, 20001, 1):
		for j in idx:
			set_point(j, sin( i * pi / 1000 + j * pi / 2.0), i * pi / 500 )
	show()
