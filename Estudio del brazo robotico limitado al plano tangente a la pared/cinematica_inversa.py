"""
-----------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA EL CÁLCULO DE LA CINEMÁTICA INVERSA 
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
def coordenadas(x,y,alpha_libre, l3):
    """
    Funcion:
    - Calcular las coordenadas a las que llegara el final del eslabon 2

    Input:
    - x: coordenada en el eje X
    - y: coordenada en el eje Y
    - l3: longitud del tercer eslabon

    Retorna:
    - x_dado: coordenada en el eje X reales
    - y_dado: coordenada en el eje Y reales
    """
    x_dado = x - l3*cos(alpha_libre)
    y_dado = y - l3*sin(alpha_libre)
    return x_dado, y_dado

def calcular_q2(x,y,l1, l2):
    """
    Funcion:
    - Calcular el angulo de la segunda articulacion

    Input:
    - x: coordenada en el eje X
    - y: coordenada en el eje Y
    - l1: longitud del primer eslabon
    - l2: longitud del segundo eslabon

    Parametros:
    - r: distancia entre el origen y el punto
    - d: variable auxiliar para calcular q2

    Retorna:
    - q2: angulo de la segunda articulacion
    """
    # Sacamos r
    r = sqrt(x**2 + y**2)

    # Sacamos d (teorema del coseno d = cos(q2))
    d = (-l1**2-l2**2+r**2)/(2*l1*l2)

    # Sacamos q2 con la expresion de la tangente
    # En este, la raiz cuadrada da +-, en funcion del que eligamos chocara o no con la pared
    q2 = arctan2(sqrt(1-d**2), d)
    return q2

# Angulo de la primera articulacion
def calcular_q1(x,y,q, l1, l2):
    """
    Funcion:
    - Calcular el angulo de la primera articulacion

    Input:
    - x: coordenada en el eje X
    - y: coordenada en el eje Y
    - l1: longitud del primer eslabon
    - l2: longitud del segundo eslabon
    - q: angulo de la segunda articulacion

    Parametros:
    - theta: angulo entre el eje X y r
    - psi: angulo entre L1 y r

    Retorna:
    - q1: angulo de la primera articulacion
    """
    # Donde la q = q2, ya que dependera del angulo e la segunda articulacion
    theta = arctan2(y,x)
    alpha = arctan2(l2*sin(q), l1+cos(q)*l2)
    q1 = theta - alpha
    return q1

def calculo_cinematica_inversa(x_punto, y_punto, alpha, l1, l2, l3):
    """
    Funcion:
    - Calcular la cinemática inversa del brazo robótico

    Input:
    - x_punto: coordenada en el eje X
    - y_punto: coordenada en el eje Y
    - alpha: angulo de la muñeca
    - l1: longitud del primer eslabon
    - l2: longitud del segundo eslabon
    - l3: longitud del tercer eslabon

    Retorna:
    - Grafica la configuracion angular del brazo robotico
    """
    x,y = coordenadas(x_punto,y_punto,alpha,l3)
    q2 = calcular_q2(x,y,l1, l2)
    q1 = calcular_q1(x,y,q2, l1, l2)
    q3 = alpha #Grado de libertad
    
    # Ecuaciones posicion
    # Hombro --> codo
    x_1 = l1*cos(q1)
    y_1 = l1*sin(q1)
    #Codo --> muñeca
    x_2 = x_1 + l2*cos(q1 + q2)
    y_2 = y_1 + l2*sin(q1 + q2)
    # Muñeca --> falange
    x_3 = x_2 + l3*np.cos(q3)
    y_3 = y_2 + l3*np.sin(q3)

    print('El angulo de la primera articulacion es = ',  q1)
    print('El angulo de la segunda articulacion es = ', q1 + q2)
    print('El angulo de la tercera articulacion es = ', q3)
    #---Graficar---#
    graficar_funcion(x_1,y_1,x_2,y_2,x_3,y_3,x_punto,y_punto)
