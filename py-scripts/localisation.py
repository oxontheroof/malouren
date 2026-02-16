# Python script to get localisation in space around

########### SENSORS INPUTS MANAGEMENT ############

# public data : estimatePosition() : returns an estimation of the state when called
# estimateSpeed()
# estimateRotation() (of head)

from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from mpu6050 import mpu6050
from global_funcs import *
import numpy as np


# HC-SR04, distance sensor

X, Y = 0, 0

myFactory = PiGPIOFactory()
distanceSensor = DistanceSensor(max_range=4, pin_factory=myFactory, echo=..., trig=...)

# accel_sensor mpu6050
accelSensor = mpu6050(0x68)

def getAccelerationData() : 
    return accelSensor.get_accel_data()

def getGyroData() : 
    return accelSensor.get_gyro_data()

# création d'un thread pour intégrer les données du capteur inertiel, possible de faire plus tard un subprocess

###############  INTEGRATERS  ###################

def integrateAcceleration(previousSpeedVec : np.array, previousPositionvec : np.array) :
    # to be started and to work continuously in background ...
    # implémenter une correction de l'erreur sur la vitesse avec d'autres sources

    accelVec = getAccelerationData()
    speed = np.array(accelVec * dt + previousSpeedVec)
    position = np.array(speed * dt + previousPositionVec)
    
    # and now ??? sleep --> bad for CPU, subproc to be implemented ??
    return speed, position

# il faut créer une autre fonction pour estimer la position à partir des moteurs

# explication pour le thread suivant : doit être lancé dans le main, et tant qu'il fonctionne, lance régulièrement une intégration
# est lancé une fois, puis modifie à chaque intégration la valeur de X et dX, qui peuvent aussitôt être utilisées par main()
# --> quel est le statut des variables X et dX ? variables globales dans le main() ?
# --> comment le thread accède-t-il aux variables X et dX et comment retourne-t-i une nouvelle valeur ?

def integrationThread():
    pass

############  GETTING LOCALISATION : IN MAIN  ############