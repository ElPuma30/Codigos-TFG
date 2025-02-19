"""
-----------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA EL CÁLCULO DE LA CINEMÁTICA DIRECTA 
-----------------------------------------------------------------
"""
#------ PAQUETES IMPORTADOS ------#
import numpy as np
from graficar import *

#------ ABREVIATURAS ------#
arctan = np.arctan
arccos = np.arccos
arctan2 = np.arctan2
tan = np.tan
sin = np.sin
cos = np.cos
pi = np.pi
sqrt = np.sqrt


#------ FUNCIONES ------#
def calculo_cinematica_directa(theta, beta, alpha, l1, l2, l3):
    """
    Funcion:
    - Calcular la cinemática directa del brazo robótico

    Input:
    - theta: angulo de rotacion del hombro
    - beta: angulo de rotacion del codo
    - alpha: angulo de rotacion de la muñeca
    - l1: longitud del primer eslabon
    - l2: longitud del segundo eslabon
    - l3: longitud del tercer eslabon

    Retorna:
    - Grafica la configuracion angular del brazo robotico
    """
#------ Ecuaciones de posicion ------#
    # Hombro --> Codo
    x_1 = l1*cos(theta)
    y_1 = l1*sin(theta)
    # Codo --> Muñeca
    x_2 = x_1 + l2*cos(beta)
    y_2 = y_1 + l2*sin(beta)
    # Muñeca --> Falange
    x_3 = x_2 + l3*cos(alpha)
    y_3 = y_2 + l3*sin(alpha)
    print('Coordenadas del punto: ')
    print('x = ', x_3)
    print('y = ', y_3)
    print('Configuración angular para dicho punto -->','(', x_3, y_3, ')')
#------ Graficar ------#
    graficar_funcion(x_1,y_1,x_2,y_2,x_3,y_3,x_3, y_3)

    