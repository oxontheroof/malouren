# Script that will contain the necessary global consts and funcs, and the conventions associated with them
# Angle convention : space is oriented clockwise (for angles especially)
# réécrire certaines fonctions pour accélérer éventuellement les calculs (sin, cos, exp, ...)
import math

import numpy as np

length = 10.5 
width = 4.5 # corresponds to radius, or width/2
speed = 0.5
h = 0.01 # this is not so much
dt = 0.08 # for the moment --> min 12 Hz
calculationLag = 0.001 # takes a little epsilon for sleep() ...

# CONVENTION DES TYPES :
"""
np.float : size de 24
np.array avec dtype de np.float
"""


def rotationAngle(vec1 : np.array, vec2 : np.array):
	assert vec1.size == vec2.size , "Error while using rotationAngle() in global_funcs"
	return math.acos(norm(vec1) * norm(vec2) / dot(vec1, vec2))


def dist(a : list, b : list):
	assert len(a) == len(b), "Error while using dist() in global funcs"
	return  norm([a[i] - b[i] for i in range(len(a))])


def norm(vector):
	return math.sqrt(sum(i**2 for i in vector))


def sayHello():
	# function of initialisation, may get instructions to do
	print("Welcome in the bot program execution. Display of this message means all scripts were compiled without errors.")
	inp = input("First instructions : ")
	return inp
# etc...
