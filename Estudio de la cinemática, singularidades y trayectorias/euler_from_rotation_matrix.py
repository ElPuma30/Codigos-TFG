"""
--------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA HACER LA CONVERSION DE UNA MATRIZ DE ROTACION 
A ANGULOS DE EULER
--------------------------------------------------------------------------
"""
import numpy as np
from math import atan2, sqrt

def euler_from_rotation_matrix(R):
    """
    Funcion:
    - Convertir una matriz de rotacion a angulos de Euler (en grados)

    Input
    - R: matriz de rotacion

    Parametros:
    - sy: hace referencia al cos(theta), buscamos como de cerca estamos de la 
          singularidad (Gimbal Lock)

    Retorna:
    - orient_x, orient_y, orient_z: angulos de Euler en grados
    """
    # Calculamos la singularidad (como de cerca estamos de 0)
    sy = sqrt(R[0, 0]**2 + R[1, 0]**2)
    # Vemos si pasa la tolerancia
    singular = sy < 1e-6
    
    if not singular:
        orient_x = atan2(R[2,1], R[2,2])   
        orient_y = atan2(-R[2,0], sy)      
        orient_z = atan2(R[1,0], R[0,0])   
    else:
        orient_x = atan2(-R[1,2], R[1,1])
        orient_y = atan2(-R[2,0], sy)
        orient_z = 0
    return np.array([orient_z, orient_y, orient_x])

