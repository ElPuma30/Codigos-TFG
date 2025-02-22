"""
--------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LOS ANGULOS A TRAVES DE LA MATRIZ DE 
ROTACION Y VICEVERSA
--------------------------------------------------------------------------
"""
import numpy as np
from math import sin, cos

def rotation_matrix_to_axis_angle(R):
    """
    Función:
    - Calculo de los angulos a traves de la matriz de rotacion

    Input:
    - R: matriz de rotacion

    Parámetros:
    - arg: argumento del arccos 
    - angle: angulo de rotacion

    Retorna:
    - Las componentes del vector de rotacion
    """
    # Vemos si el argumento se encuentra entre -1 y 1 para evitar errores numericos
    arg = (np.trace(R) - 1) / 2
    arg = np.clip(arg, -1, 1)
    angle = np.arccos(arg)

    # Calculamos el vector de rotacion
    if angle == 0:
        return np.zeros(3)
    else:
        rx = (R[2,1] - R[1,2]) / (2 * np.sin(angle))
        ry = (R[0,2] - R[2,0]) / (2 * np.sin(angle))
        rz = (R[1,0] - R[0,1]) / (2 * np.sin(angle))
        return angle * np.array([rx, ry, rz])

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Función:
    - Calculo de la matriz de rotacion R_12 (ROTACION DESDE S1--> S2) segun convencion zyx

    Input:
    - roll, pitch, yaw: angulos de rotacion

    Parámetros:
    - R_x: matriz de rotacion alrededor del eje X
    - R_y: matriz de rotacion alrededor del eje Y
    - R_z: matriz de rotacion alrededor del eje Z

    Retorna:
    - La matriz de rotacion
    """
    R_x = np.array([
        [1,         0,                0],
        [0, cos(roll),      -sin(roll)],
        [0, sin(roll),       cos(roll)]
    ])
    R_y = np.array([
        [cos(pitch),    0, sin(pitch)],
        [0,             1,          0],
        [-sin(pitch),   0, cos(pitch)]
    ])
    R_z = np.array([
        [cos(yaw), -sin(yaw),    0],
        [sin(yaw),  cos(yaw),    0],
        [0,             0,       1]
    ])
    R = R_z @ R_y @ R_x
    return R


