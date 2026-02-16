#!/usr/bin/python3

# File that manages connection with robot using bluedot and gpiozero, and allows to control manually the movement 
# API pour faciliter la communication avec les moteurs, servos et capteurs

import numpy as np
import gpiozero
import time
import threading
from global_funcs import *
import RPi.GPIO as GPIO

# from adafruit_servokit import * # only using ServoKit ?
from adafruit_motorkit import MotorKit

# Other custom localisation scripts are to be imported ...
# funcs : kit.servo[i].angle = n
#         kit.servo[i].actuation_range = n
#         kit.servo[i].set_pulse_widht_range(min = 1000, max = 2000)
#         kit.continuous_servo[i].throttle = j (max 1, min 0)
# i is the channel the servo is connected to

#------------------ CONSTANTS  ------------------------

#------------------  PHYSICAL PINS ATTRIBUTION ----------------
from adafruit_motorkit import MotorKit


kit = MotorKit(0x40)
# motor1 = left motor, motor2 = rightmotor
# uses WaveshareHat
"""
# Forward at full throttle
kit.motor1.throttle = 1.0
kit.motor2.throttle = 1.0
# Stop & sleep for 1 sec.
kit.motor1.throttle = 0.0
kit.motor2.throttle = 0.0
# Right at half speed
kit.motor1.throttle = 0.5
kit.motor2.throttle = -0.5
"""

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


def getDistance():
	# set Trigger to HIGH
	GPIO.output(GPIO_TRIGGER, True)

	# set Trigger after 0.01ms to LOW
	time.sleep(0.00001)
	GPIO.output(GPIO_TRIGGER, False)

	StartTime = time.time()
	StopTime = time.time()

	# save StartTime
	while GPIO.input(GPIO_ECHO) == 0:
		StartTime = time.time()

	# save time of arrival
	while GPIO.input(GPIO_ECHO) == 1:
		StopTime = time.time()

	# time difference between start and arrival
	TimeElapsed = StopTime - StartTime
	# multiply with the sonic speed (34300 cm/s)
	# and divide by 2, because there and back
	distance = (TimeElapsed * 34300) / 2

	return distance


#------------------  MAIN CUSTOM CLASSES  ---------------------

def boolMoveThread(dt, s1, s2, mbool=None):
	if mbool != None : 
		kit.motor1.throttle = s1
		kit.motor2.throttle = s2
		time.sleep(dt - h) # little const to be taken into account because of multithreading
	else : 
		# means a condition is passed as argument, may not be useful (mbool doit être plutôt une fonction pour
		# être faiclement actualisée
		while mbool() :
			kit.motor1.throttle = s1
			kit.motor2.throttle = s2

#--------------  MAIN FUNCS  -----------------

def rotate(dtheta, ws=0.1)
	# fonction basique pour tourner d'un angle dtheta, avec une vitesse lente sur chaque roue (ws)
	# dtheta en radians, ws dans [0, 1]
	dx = dtheta * width
	dt = abs(dx / ws)
	th = boolMoveThread(, args=(dt, ws, -ws))
	th.start()


def forward(dx): 
	# fonction basique permettant d'aller de dx vers l'avant
	dt = dx / speed
	th = threading.Thread(target=boolMoveThread, args=(dx, speed, speed))
	th.start()

# Création d'une classe de mouvements, pour pouvoir etre réalisés en autonomie

class rectMove():
	# créer un standard de structure précis, pour optimiser les types des variables --> perfs
	def __init__(self, initialAngle : np.float, startPoint : np.array, target : np.array):
		self.startAngle = initialAngle
		self.startPoint = startPoint
		self.targetPoint = target


	def exec(self, X : np.array, angle : np.float):
		# méthode lancée depuis une pile de déplacements : d'abord corrige les erreurs avec approximation(),
		# puis lance un déplacement de A à B
		# but : ne pas être trop simple, intégrer le futur système de correction et détection d'obstacles
		self.approximation(X, angle)
		vector = self.targetPoint - self.startPoint
		length = norm(vector)
		forward(length)


	def __repr__(self):
		...

	def approximation(self, X : np.array, angle : np.float)
		# méthode lancée depuis la méthode exec, pour corriger les erreurs de déplacement et l'approximation
		if not self.startPoint is None :
			deltaVec = X - self.startPoint
			deltaTheta = angle - self.startAngle



			# then make a move to lower unprecision


		# puis : exécuter le mouvement

	# méthodes de classe à définir : exécution, représentation, évaluer avec la position actuelle.
	# un mouvement est un vecteur et on lui ajoute une localisation de départ : une méthode doit
	# déterminer (au moment où le mouvement doit être effectué) le décalage entre la prévision et la réalité
	# méthode de délétion aussi, et qui retourne la position finale (pour le prochain mouvement)
	# on peut mettre la valeur None à la position de départ, pour avoir un mouvement absolu dans l'espace


class rotation():
	# créer un standard de structure précis, pour optimiser les types des variables --> perfs
	def __init__(self, initialAngle : np.float, startPoint : np.array, angle : np.float):
		self.startAngle = initialAngle
		self.angle = angle
		self.startPoint = startPoint

	# cette classe reprend les mêmes méthodes que la classe précédente

	def __repr__(self):
		pass

	def exec(self):
		pass

	def approximation(self):
		# same that for rectMove()
		if not self.startPoint is None :
			deltaVec = X - self.startPoint
			deltaTheta = angle - self.startAngle

class movesStack():
	# regarder si déjà des libes python pour optimiser des stacks / files
	def __init__(self, moves : list):
		self.stack = moves

	def exec_stack(self):
		...



