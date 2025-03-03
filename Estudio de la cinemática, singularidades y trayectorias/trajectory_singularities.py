"""
------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA DETECTAR LAS POSIBLES SINGULARIDADES
A LO LARGO DE LA TRAYECTORIA
------------------------------------------------------------------
"""
import numpy as np
from jacobian import compute_jacobian
from build_trajectory import interpolate_trajectory

def is_singular(theta_input, DH_params, tol=1e-4):
    """
    Funcion:
    - Verifica si una configuracion (theta_input) se encuentra cerca de una singularidad
    
    Input:
    - theta_input: configuracion articular
    - DH_params: matriz de parametros DH
    - tol: umbral para el determinante del Jacobiano

    Parametros:
    - J: matriz Jacobiana
    - detJ: determinante del Jacobiano

    Retorna:
    - True si la configuracion es singular, False en caso contrario
    """
    J = compute_jacobian(DH_params, theta_input)
    detJ = np.linalg.det(J)
    return np.abs(detJ) < tol


def check_trajectory_singularity(trajectory, DH_params, tol=1e-4):
    """
    Funcion:
    - Recorre la trayectoria (lista o array de configuraciones) y verifica si en cada punto
    se detecta una singularidad (usando el determinante del Jacobiano).
    
    Input:
    - trajectory: lista o array de configuraciones articulares
    - DH_params: matriz de parametros DH
    - tol: umbral para el determinante del Jacobiano

    Retorna:
    - singular_points: lista de indices en los que se detecto singularidad.
    """
    singular_points = []
    for i, theta_input in enumerate(trajectory):
        if is_singular(theta_input, DH_params, tol):
            print(f"Punto {i}: configuración {theta_input} -> SINGULAR")
            singular_points.append(i)
        else:
            print(f"Punto {i}: configuración {theta_input} -> No singular")
    return singular_points


def plot_jacobian_variation(dh_params, num_steps, best_by_start, global_best_chain, plot_all_trajectories=True):
    """
    Funcion:
    - Grafica la variacion del determinante del Jacobiano a lo largo de las trayectorias interpoladas.
    """
    import matplotlib.pyplot as plt
    
    plt.figure(figsize=(10, 6))
    available_colors = ["blue", "red", "green", "orange", "purple", "brown", "cyan", "magenta"]
    
    if plot_all_trajectories:
        # Graficar la variacion del determinante del Jacobiano para cada cadena candidata
        candidate_keys = sorted(best_by_start.keys())
        for idx, key in enumerate(candidate_keys):
            chain = best_by_start[key][0]
            # Interpolar la cadena para obtener mas puntos y asi una curva mas suave
            interpolated_traj = interpolate_trajectory(chain, num_steps)
            det_jacobian = []
            for theta in interpolated_traj:
                J = compute_jacobian(dh_params, theta)
                det_jacobian.append(np.linalg.det(J))
            plt.plot(det_jacobian, marker='o', linestyle='-', 
                     color=available_colors[idx % len(available_colors)], 
                     label=f"Chain {key+1}")
    else:
        # Graficar solo la trayectoria global optima
        chain = global_best_chain
        interpolated_traj = interpolate_trajectory(chain, num_steps)
        det_jacobian = []
        for theta in interpolated_traj:
            J = compute_jacobian(dh_params, theta)
            det_jacobian.append(np.linalg.det(J))
        plt.plot(det_jacobian, marker='o', linestyle='-', 
                 color='blue', label="Trayectoria óptima")
    
    plt.xlabel("Índice del waypoint interpolado")
    plt.ylabel("Determinante del Jacobiano")
    plt.title("Variación del determinante del Jacobiano")
    plt.grid(True)
    plt.legend()
    plt.show()