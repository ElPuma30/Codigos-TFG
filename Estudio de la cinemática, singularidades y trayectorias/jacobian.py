"""
----------------------------------------
ESTE CODIGO CALCULA LA MATRIZ JACOBIANA 
----------------------------------------
"""
import numpy as np
from DH_transform import compute_transform_DH

def compute_jacobian(DH_params, theta_input):
    """
    Funcion:
    - Calcular la matriz jacobiana

    Input
    - DH_params: matriz de los parámetros de DH
    - theta_input: conjunto de angulos que las articulaciones deben tomar

    Parámetros:
    - n_joints: indica el numero de eslabones que tiene el brazo robotico
    - T_cumulative: matriz identidad de 4x4 que representa la transformación inicial (sin movimiento)
    - J: matriz jacobiana de 6 filas y n_joints columnas (jp: parte lineal, jo: parte angular)
    - z_vectors: lista que almacena los vectores z de cada articulación
    - positions: lista de las posiciones (iniciamos en el origen)

    Retorna:
    - J: matriz jacobiana de 6 filas y n_joints columnas
    """
    # Definimos las variables
    n_joints = len(theta_input)
    T_cumulative = np.identity(4)
    J = np.zeros((6, n_joints))
    z_vectors = [np.array([0, 0, 1])]               # inicializamos con el eje Z del sistema base
    positions = [T_cumulative[0:3, 3]]              # posicion inicial (origen)

    # Calculamos las matrices de transformacion acumuladas y extraemos los ejes Z y posiciones
    for i in range(n_joints):
        # Calculo la matriz de transformacion individual
        T_i = compute_transform_DH(DH_params, i + 1, theta_input)
        # Actualizamos la matriz de transformacion
        T_cumulative = T_cumulative @ T_i 
        # Añadimos a la lista el nuevo eje Z
        z_vectors.append(T_cumulative[0:3, 2])
        # Añadimos a la lista la nueva posicion
        positions.append(T_cumulative[0:3, 3])
    end_effector_pos = positions[-1]                # sacamos la psicion del efector final

    for i in range(n_joints):
        z_i = z_vectors[i]                          
        p_i = positions[i]                          
        Jp = np.cross(z_i, end_effector_pos - p_i)  
        Jo = z_i                                    
        J[:, i] = np.concatenate((Jp, Jo))          
    return J