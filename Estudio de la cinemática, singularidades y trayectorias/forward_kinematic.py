"""
----------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA CINEMATICA DIRECTA (PARTE NUMERICO)
----------------------------------------------------------------------------
"""
import numpy as np
import matplotlib.pyplot as plt
from itertools import product
from math import pi
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from DH_transform import compute_transform_DH
from constants import *

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


def plot_ur_envelope(DH_params, samples=10, fixed_joints=[0, 0, 0]):
    """
    Calcula y grafica la envolvente del workspace del brazo UR de 6 GDL.
    
    Se varían las tres primeras articulaciones (q1, q2 y q3) y se fijan las articulaciones 4, 5 y 6.
    
    Parámetros:
      - DH_params: parámetros DH del robot.
      - samples: número de muestras por cada articulación (por defecto 10).
      - fixed_joints: lista con los valores (en radianes) para las articulaciones 4, 5 y 6.
    """
    # Definir rangos de variación para q1, q2 y q3
    q1_range = np.linspace(-pi, pi, samples)
    q2_range = np.linspace(-pi/2, pi/2, samples)
    q3_range = np.linspace(-pi, pi, samples)
    
    positions_list = []
    # Se generan todas las combinaciones posibles para q1, q2 y q3
    for q1, q2, q3 in product(q1_range, q2_range, q3_range):
        # Se forma el vector de 6 ángulos: los tres primeros varían y los últimos se fijan
        theta_vector = [q1, q2, q3] + fixed_joints
        # Se calcula la cinemática directa
        _, pos_list = compute_forward_kinematic(DH_params, theta_vector)
        final_pos = pos_list[-1]  # Posición del efector final
        positions_list.append(final_pos)
    
    positions_array = np.array(positions_list)
    
    # Crear el gráfico 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(positions_array[:, 0], positions_array[:, 1], positions_array[:, 2],
               s=1, color='blue', alpha=0.5)
    
    # Calcular y graficar la envolvente convexa si hay suficientes puntos
    if positions_array.shape[0] >= 4:
        hull = ConvexHull(positions_array)
        for simplex in hull.simplices:
            triangle = positions_array[simplex]
            poly = Poly3DCollection([triangle], alpha=0.2, facecolor='red')
            ax.add_collection3d(poly)
            
    ax.set_title("Envolvente del Workspace del brazo UR de 6 GDL")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()

# --- Ejecución ---
plot_ur_envelope(ur_params['DH_matrix_UR10'], samples=15, fixed_joints=[0, 0, 0])