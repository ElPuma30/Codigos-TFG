"""
--------------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA VERIFICAR QUE LA MATRIZ DE TRANSFORMACION HOMOGENEA
ES IGUAL CALCULADA POR EL METODO DIRECTO Y LO INDIRECTO
--------------------------------------------------------------------------------
"""
import numpy as np
from forward_kinematic import compute_forward_kinematic, compute_the_final_position_and_orientation
from plot import plot_robot

def verificar_y_graficar_soluciones(Dh_params, theta_solutions, desired_pos, desired_rot, positions_target):
    """
    Funcion:
    - Verificar que con la cinematica directa e indirecta dan el mismo resultado y graficar las soluciones

    Input:
    - DH_params: matriz de los parametros de DH
    - theta_solutions: los angulos que obtenemos con la cinematica inversa
    - desired_pos, desired_rot: la posicion y la matriz de rotacion deseada del efector final
    - positions_target: la posicion de todos los eslabones a los que quiero ir

    Parámetros:
    - solutions_info: informacion de las soluciones que estamos sacando
    - T_solution,positions_solution: posiciones y matrices de transformacion de los distintos eslabones obtenidas 
                                    con los angulos calculados con la cinematica inversa
    - final_pos, final_rot: posicion y matriz de rotacion del efector final calculada con la cinematica inversa
    - z_vectors: lista que almacena los vectores z de cada articulación
    - pos_error, rot_error : errores de orientacion y posicion en el efector final
    """
    solutions_info = []

    for idx, theta_sol in enumerate(theta_solutions):
        # Sacamos la posicion y la matriz de rotacion con las distintas soluciones que nos dan la cinematica inversa
        T_solution, positions_solution = compute_forward_kinematic(Dh_params, theta_sol)
        final_pos, final_rot = compute_the_final_position_and_orientation(T_solution)
        # Vemos el error entre lo deseado y lo obtenido
        pos_error = np.linalg.norm(desired_pos - final_pos)
        rot_error = np.linalg.norm(desired_rot - final_rot)

        print(f"\n> Verificación de la Solución {idx+1}:")
        print(f"- Posición obtenida: x = {final_pos[0]:.6f}, y = {final_pos[1]:.6f}, z = {final_pos[2]:.6f}")
        print(f"- Error de posición: {pos_error:.6e}")
        print(f"- Matriz de orientacion obtenida:\n", final_rot)
        print(f"- Error de orientación: {rot_error:.6e}")

        # Almacenar informacion de la solucion
        solutions_info.append({
            'index': idx,
            'theta_sol': theta_sol,
            'positions_solution': positions_solution,
            'pos_error': pos_error,
            'rot_error': rot_error
        })

    ##--- Control de graficacion ---##
    graficar_todas_las_soluciones = True                # Cambia a False para graficar solo una solucion
    solucion_elegida = 0                                # Indice de la solucion a graficar (hay que poner una menos de la que queremos)

    positions_list = []
    labels = []
    markers = []
    colors = []
    alphas = []

    if graficar_todas_las_soluciones:
        # Graficar todas las soluciones
        for sol_info in solutions_info:
            positions_list.append(sol_info['positions_solution'])
            labels.append(f"Solución {sol_info['index']+1}")
            markers.append('o')
            colors.append(None)
            alphas.append(1.0)
    else:
        # Graficar solo la solucion elegida
        if solucion_elegida < 0 or solucion_elegida >= len(solutions_info):
            print(f"\nÍndice de solución inválido. Por favor, elige un valor entre 0 y {len(solutions_info)-1}.")
        else:
            selected_solution = solutions_info[solucion_elegida]
            positions_list.append(selected_solution['positions_solution'])
            labels.append(f"Solución {selected_solution['index']+1} Elegida")
            markers.append('o')
            colors.append(None)
            alphas.append(1.0)

    # Graficar las soluciones y la posicion deseada
    plot_robot(positions_list, labels, markers, colors, alphas, desired_pos)

"""
    # Agregar la configuración objetivo (opcional)
    positions_list.append(positions_target)
    labels.append('Configuración Objetivo')
    markers.append('s')
    colors.append('green')
    alphas.append(0.5)
"""
