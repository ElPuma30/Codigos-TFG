"""
----------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA CINEMATICA INVERSA (PARTE NUMERICO)
----------------------------------------------------------------------------
"""
import numpy as np
from forward_kinematic import compute_forward_kinematic, compute_the_final_position_and_orientation
from jacobian import compute_jacobian
from rotation_matrix import rotation_matrix_to_axis_angle

def compute_inverse_kinematics(DH_params, desired_pos, desired_rot, initial_guesses=True, max_iters=10000, tol=1e-7):
    """
    Funcion:
    - Calculo de la cinematica inversa

    Input
    - DH_params: matriz de los parametros de DH
    - desired_pos, desired_rot: posicion y orientacion deseada
    - initial guess: condiciones iniciales con las que iniciamos las iteraciones 
    - max_iters: numero maximo de iteraciones que hara la funcion
    - tol: tolerancia permitida para la convergencia

    Parametros:
    - pos_error: error en la posicion
    - rot_error_mat: error en la matriz de rotacion
    - rot_error_vec: vector de error de rotación convertido de la matriz a un vector de angulo-eje
    - error: vector concatenado que incluye el error de posición y orientación

    Retorna:
    - solutions: lista de configuraciones de angulos que alcanzan la pose objetivo
    """
    # Inicializa una lista para almacenar las soluciones encontradas
    solutions = []
    # Vemos si tenemos condiciones iniciales dadas o las genera el codigo
    if initial_guesses is None:
        initial_guesses = [np.zeros(6)]
    # Iteramos sobre la condicion inicial para obtener la theta deseada
    for initial_guess in initial_guesses:
        theta = initial_guess.copy() 
        for i in range(max_iters):
            # Calculamos la posicion y la orientacion con la theta obtenida
            T, _ = compute_forward_kinematic(DH_params, theta)
            current_pos, current_rot = compute_the_final_position_and_orientation(T)
            # Calculamos la diferencia entre la posicion deseada y la calculada
            pos_error = desired_pos - current_pos
            # Calculamos la diferencia entre la matriz de rot deseada y la calculada
            rot_error_mat = desired_rot @ current_rot.T
            rot_error_vec = rotation_matrix_to_axis_angle(rot_error_mat)
            # Concatenamos el error de posicion y de orientacion
            error = np.concatenate((pos_error, rot_error_vec))

            # Vemos si el error supera la tolerancia
            if np.linalg.norm(error) < tol:
                print(f"Convergencia alcanzada en la iteración {i} para la estimación inicial {initial_guess}")
                # Ajustar theta para que esté dentro del rango -2π a 2π
                theta = (theta + 2 * np.pi) % (4 * np.pi) - 2 * np.pi
                solutions.append(theta)
                break
            
            # Calculamos el jacobiano
            J = compute_jacobian(DH_params, theta)

            delta_theta = np.linalg.pinv(J) @ error
            # Actualizamos el valor de theta
            theta += delta_theta

        else:
            print(f"No se alcanzó la convergencia para la estimación inicial {initial_guess}")
    return solutions