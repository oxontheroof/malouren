# Hello there

from moves import *
from map_drawing import *
from localisation import *
import threading
import time
import gpiozero


def detect_keys():
    ...
    # permet d'écouter une entrée standard, boutons sur le pi, etc ...


##############  MODES FUNCTIONS  #############

# functions here are to be launched within the main() function
# faire un brouillon un système de sycnhronisation des fonctions dans le main()
# chaque fonction doit occuper le thread principal jussqu'à la fin de son action
# les fonctions retournent dans le main() un signal qui indique l'état : (dresser une liste)

def scan():
    ...


def move():
    ...


###################  MAIN  ###################

def theBigMain():
    # DANS LES PREMIERES VERSIONS, COMMENCER AVEC UN PROCESSUS SIMPLIFIE SANS CARTES
    t0 = time.monotonic()
    theta = 0
    X, Y = 0, 0
    dX, dY = 0, 0

    # motor1speed, motor2speed = 0, 0
    def stop():
        # virtually sets the speed to 0
        # sert à reprendre à 0 les variables senssibles durant l'intégration
        # stoppe / est appelée à la fin d'un thread intégrateur
        dX, dY = 0, 0

    first_instructions = sayHello()  # --> commence à diverger ici
    # mode = "working" or "scanning"
    mode = "scanning"


# And ... ######################

# if __name__ == "main":
#     theBigMain()
