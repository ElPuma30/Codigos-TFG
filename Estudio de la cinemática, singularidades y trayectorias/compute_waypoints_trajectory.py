"""
------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA CONFIGURACION ANGULAR DE 
CADA PUNTO DE LA TRAYECTORIA Y ESCRIBIR LA INFORMACION DE ESTE
------------------------------------------------------------------
"""
import numpy as np
from analitic_inverse_kinematic import inverse_kinematic_solution, filtrar_soluciones_unicas
from rotation_matrix import euler_to_rotation_matrix
from DH_transform import transformation_matrix
from inverse_kinematic import compute_inverse_kinematics


def waypoint_to_transform(waypoint):
    """
    Funcion:
    - Convierte un waypoint (x, y, z, roll, pitch, yaw) en una matriz de transformacion

    Input:
    - waypoint: punto al que se quiere ir

    Parametros:
    - x,y,z: coordenadas del punto
    - roll, pitch, yaw: orientaciones del punto (rx, ry, rz)
    - desired_pos: array de las coordenadas
    - desired_rot: matriz de rotacion

    Retorna:
    - T: matriz de transformacion 
    """
    x, y, z, roll, pitch, yaw = waypoint
    desired_pos = np.array([x, y, z])
    desired_rot = euler_to_rotation_matrix(yaw, pitch, roll)
    T = transformation_matrix(desired_rot, desired_pos)
    return T

def get_analitic_solutions_for_trajectory(trayectoria, dh_params, tolerancia=1e-10):
    """
    Funcion:
    - Para cada waypoint de la trayectoria, calcula la cinematica inversa y filtra las soluciones unicas

    Input:
    - trayectoria: lista de waypoints
    - dh_params: matriz de parametros DH
    - tolerancia: tolerancia para filtrar soluciones similares

    Parametros:
    - unique_solutions: lista de soluciones unicas
    - theta_solution: soluciones analiticas

    Retorna:
    - waypoint_solutions: lista de soluciones unicas
    """
    waypoint_solutions = []
    # El bucle recorre cada waypoint de la trayectoria
    for waypoint in trayectoria:
        T = waypoint_to_transform(waypoint)
        theta_solution = inverse_kinematic_solution(dh_params, T)
        unique_solutions = filtrar_soluciones_unicas(theta_solution, tolerancia)
        waypoint_solutions.append(unique_solutions)
    return waypoint_solutions

def refine_solutions_numerically(cartesian_waypoints, solutions_per_waypoint, DH_params):
    """
    Funcion:
    - Para cada waypoint, refina las soluciones analiticas con el metodo numerico

    Input:
    - cartesian_waypoints: lista de waypoints
    - solutions_per_waypoint: lista de soluciones analiticas
    - DH_params: matriz de parametros DH

    Parametros:
    - initial_guesses: lista de soluciones analiticas
    - refined_solutions: lista de soluciones refinadas
    - T_desired: matriz de transformacion deseada
    - desired_pos: array de las coordenadas
    - desired_rot: matriz de rotacion
    
    Retorna:
    - refined_solutions_list: lista (por waypoint) de listas de soluciones numericas refinadas.
    """
    refined_solutions_list = []
    for i, waypoint in enumerate(cartesian_waypoints):
        # Convertir el waypoint a su transformacion deseada
        T_desired = waypoint_to_transform(waypoint)
        desired_pos = np.array(T_desired[:3, 3]).flatten()
        desired_rot = np.array(T_desired[:3, :3])
        
        initial_guesses = solutions_per_waypoint[i]
        
        refined_solutions = compute_inverse_kinematics(DH_params, desired_pos, desired_rot, initial_guesses)
        
        # Si no se obtuvo ninguna solucion refinada, se usa la solucion analitica
        if not refined_solutions:
            print(f"Refinamiento numérico falló para el waypoint {i+1}. Se mantiene la solución analítica.")
            refined_solutions = initial_guesses
        
        refined_solutions_list.append(refined_solutions)
        print(f"Waypoint {i+1} - Soluciones refinadas (numéricas):")
        for sol in refined_solutions:
            print("    ", sol)
    return refined_solutions_list

def write_waypoint_information(trayectoria, waypoint_solutions):
    """
    Funcion:
    - Para cada waypoint se imprime la posición deseada y la matriz de rotación deseada
    
    Input:
    - trayectoria: lista de waypoints
    - waypoint_solutions: lista de soluciones

    Parametros:
    - theta_solutions: soluciones analiticas
    - T_desired: matriz de transformacion deseada
    - desired_pos: array de las coordenadas
    - desired_rot: matriz de rotacion

    Retorna:
      - La información de cada waypoint
    """
    for i, waypoint in enumerate(trayectoria):
        # Convertir el waypoint a su transformacion deseada
        T_desired = waypoint_to_transform(waypoint)
        desired_pos = np.array(T_desired[:3, 3]).flatten()
        desired_rot = np.array(T_desired[:3, :3])
        
        theta_solutions = waypoint_solutions[i]
        
        print(f"\n> WAYPOINT {i+1}:")
        print("  \n-Posicion deseada:", desired_pos)
        print("  \n-Matriz de rotacion deseada:\n", desired_rot)
        print("  \n-Este punto presenta", len(theta_solutions), "soluciones analiticas")
        for sol in theta_solutions:
            print("    ", sol)
        print("\n")
