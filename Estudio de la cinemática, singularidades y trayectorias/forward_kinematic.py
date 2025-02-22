"""
----------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA CINEMATICA DIRECTA (PARTE NUMERICO)
----------------------------------------------------------------------------
"""
import numpy as np
from DH_transform import compute_transform_DH

def compute_forward_kinematic(DH_params, theta_input):
    """
    Funcion:
    - Obtener la matriz de transformacion homogenea y las posiciones

    Input
    - DH_params: matriz de los parametros de DH
    - theta_input: conjunto de angulos que las articulaciones deben tomar

    Parámetros:
    - T: matriz de transformacion homogenea final
    - T_cumulative: matriz identidad de 4x4 que representa la transformación inicial (sin movimiento)
    - positions: lista que almacena las posiciones de los eslabones

    Retorna:
    - Matriz de transformación 4x4 y las posiciones de los eslabones
    """
    T = []                                                      # lista donde iremos almacenando las matrices de transformacion 
    T_cumulative = np.identity(4) 
    positions = [T_cumulative[0:3, 3]]                          # comienza en el origen (0,0,0)

    # Recorremos cada una de las 6 articulaciones
    for i in range(1, 7):                                       # (recordamos que las matrices empiezan en 0)
        # Calculamos la matriz de transformacion de cada eslabon
        T_i = compute_transform_DH(DH_params, i, theta_input)
        # Actualizamos la matriz de transformacion
        T_cumulative = T_cumulative @ T_i
        # La vamos añadiendo a la lista
        T.append(T_cumulative.copy())
        # Almacenamos las posiciones 
        positions.append(T_cumulative[0:3, 3])
    return T, positions

def compute_the_final_position_and_orientation(T):
    """
    Funcion:
    - Obtener la ultima matriz de rotacion y las posiciones finales

    Input
    - T: lista de matrices de transformacion acumuladas

    Parámetros:
    - T_end_effector: matriz de transformacion del ultimo eslabon
    - rotation_matrix: matriz 3x3 de rotacion del eslabon final
    - position: las posicion del ultimo eslabon

    Retorna:
    - Matriz de rotacion 3x3 y las ultimas posiciones
    """
    # Extraemos la ultima matriz de transformacion que corresponde al ultimo eslabon
    T_end_effector = T[-1]
    position = T_end_effector[0:3, 3]
    rotation_matrix = T_end_effector[0:3, 0:3]
    return position, rotation_matrix
