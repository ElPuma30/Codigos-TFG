"""
---------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA MATRIZ DE TRANSFORMACION HOMOGENEA 
CON LOS PARAMETROS DE DENAVIT-HARTENBERG
---------------------------------------------------------------------------
"""
import numpy as np
from math import sin, cos


def compute_transform_DH(DH_params, n, theta_input):
    """
    Funcion:
    - Generar una matriz de transformacion homogenea para cada eslabon a partir de los parámetros DH

    Input
    - DH_params: matriz con los parámetros de DH
    - n: representa los eslabones del brazo robotico
    - theta_input: conjunto de angulos que las articulaciones deben tomar

    Parametros:
    - theta: angulo de rotacion alrededor del eje Z (en radianes)
    - alpha: angulo de torsion del eslabon (en radianes)
    - a: longitud del eslabon en el eje X
    - d: desplazamiento a lo largo del eje Z

    Retorna:
    - Matriz de transformacion 4x4 para cada eslabon
    """
    # Avanzamos en los eslabones (para acceder a las matrices en python comenzamos en 0)
    n = n - 1

    theta = theta_input[n]
    a = DH_params[n, 0]
    alpha = DH_params[n, 1]
    d = DH_params[n, 2]

    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)

    # Matriz de DH clasica
    transform_matrix = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,       ca, d],
        [0,        0,        0, 1]
    ], dtype=float)
    return transform_matrix

    # Matriz de DH modificada 
"""
    transform_matrix = np.array([
        [ct,      -st,       0,    a],
        [st*ca,  ct*ca,   -sa,  -d*sa],
        [st*sa,  ct*sa,    ca,   d*ca],
        [0,         0,       0,     1]
    ], dtype=float)
    return transform_matrix
"""


def transformation_matrix(R, t):
    """
    Funcion:
    - Genera la matriz de transformación homogénea 4x4

    Input:
    - R: matriz de rotacion
    - t: vector de traslacion

    Retorna:
    - T: matriz de transformacion
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

