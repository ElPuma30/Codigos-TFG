"""
------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA DIBUJAR Y VERIFICAR LAS TRAYECTORIAS
------------------------------------------------------------------
"""
import numpy as np
from forward_kinematic import compute_forward_kinematic, compute_the_final_position_and_orientation
from jacobian import compute_jacobian
from interpolar import interpolar_angulos
from plot import plot_robot, plot_joint_space_2D
from compute_waypoints_trajectory import *
from trajectory_singularities import *
from build_trajectory import *

def plot_and_verify_trajectories(dh_params, num_steps, best_by_start, global_best_chain, plot_all_trajectories):

    if plot_all_trajectories:
        # Dibujar todas las cadenas candidatas
        candidate_chains = []
        end_effector_positions_all = []
        labels_all = []
        colors_all = []
        available_colors = ["blue", "red", "green", "orange", "purple", "brown", "cyan", "magenta"]
        
        # Ordenamos las claves de best_by_start
        candidate_keys = sorted(best_by_start.keys())
        for idx, key in enumerate(candidate_keys):
            chain = best_by_start[key][0]
            candidate_chains.append(chain)
            
            # Calcula la posiciones SOLO de los waypoints (sin interpolar) para la gráfica 3D
            chain_positions = []
            for theta_input in chain:
                T, _ = compute_forward_kinematic(dh_params, theta_input)
                pos, _ = compute_the_final_position_and_orientation(T)
                chain_positions.append(pos)
            
            # Calcular la interpolación para la gráfica 2D
            interpolated_chain = interpolate_trajectory(chain, num_steps)
            
            # Guardar SOLO los waypoints en la lista de posiciones 3D
            end_effector_positions_all.append(chain_positions)
            labels_all.append(f"Chain {key+1}")
            colors_all.append(available_colors[idx % len(available_colors)])
            
            # Graficar la trayectoria en 2D con TODOS los puntos interpolados
            label_2d = f"Trayectoria {idx}"
            plot_joint_space_2D(interpolated_chain, label_2d)

        plot_robot(
            positions_list=end_effector_positions_all,
            labels=labels_all,
            markers=["o"] * len(end_effector_positions_all),
            colors=colors_all,
            desired_pos=end_effector_positions_all[-1][-1],   
        )
        
        # Verificación de singularidades en cada cadena candidata
        for idx, positions in enumerate(end_effector_positions_all):
            print(f"\nVerificando singularidades para Chain {candidate_keys[idx]+1}:")
            # Reconstruimos la trayectoria interpolada (joint space) para verificar
            chain = candidate_chains[idx]
            interpolated_chain = interpolate_trajectory(chain, num_steps)
            singular_points = check_trajectory_singularity(interpolated_chain, dh_params)
            if singular_points:
                print(f"Se detectaron singularidades en los puntos: {singular_points}")
            else:
                print("No se detectaron singularidades en la trayectoria interpolada.")
        
        # Simulación de movimiento y cálculo del Jacobiano para cada cadena
        for chain_idx, chain in enumerate(candidate_chains):
            print(f"\nSimulación para Chain {candidate_keys[chain_idx]+1}:")
            interpolated_chain = interpolate_trajectory(chain, num_steps)
            for idx, theta_input in enumerate(interpolated_chain):
                T, _ = compute_forward_kinematic(dh_params, theta_input)
                end_effector_pos, _ = compute_the_final_position_and_orientation(T)
                print(f"  Waypoint {idx}:")
                print(f"    Configuración articular: {theta_input}")
                print(f"    Posición del efector final: {end_effector_pos}")
                J = compute_jacobian(dh_params, theta_input)
                det_J = np.linalg.det(J)
                print(f"    Determinante del Jacobiano: {det_J:.6f}\n")    
    
    else:
        # Si no se grafican todas, se elige una sola cadena (opción manual u óptima)
        opcion = None  # Trayectoria a usar (None = optima), esto es a modo de ejemplo
        if opcion in best_by_start:
            cadena_elegida = best_by_start[opcion][0]
            print(f"\nTrayectoria elegida: opción {opcion}")
        else:
            print("\nSe usará la trayectoria más óptima.")
            cadena_elegida = global_best_chain
        

        chain_positions = []
        for theta_input in cadena_elegida:
            T, _ = compute_forward_kinematic(dh_params, theta_input)
            pos, _ = compute_the_final_position_and_orientation(T)
            chain_positions.append(pos)
        
        # Calcular la interpolación para la gráfica 2D
        interpolated_trajectory = interpolate_trajectory(cadena_elegida, num_steps)

        plot_robot(
            positions_list=[chain_positions],
            labels=["Trayectoria Elegida"],
            markers=["o"],
            colors=["blue"],
            desired_pos=chain_positions[-1],
        )

        plot_joint_space_2D(interpolated_trajectory, "Trayectoria Elegida")

        # Verificación de singularidades en la trayectoria elegida
        print("\n--- Verificación de Singularidades en la Trayectoria Elegida ---")
        singular_points = check_trajectory_singularity(interpolated_trajectory, dh_params)
        if singular_points:
            print(f"Se detectaron singularidades en los puntos: {singular_points}")
        else:
            print("No se detectaron singularidades en la trayectoria interpolada.")

        # Simulación de Movimiento y Cálculo del Jacobiano para la cadena elegida
        print("\n--- Simulación de Movimiento y Cálculo del Jacobiano ---")
        for idx, theta_input in enumerate(interpolated_trajectory):
            T, _ = compute_forward_kinematic(dh_params, theta_input)
            end_effector_pos, _ = compute_the_final_position_and_orientation(T)
            print(f"Waypoint {idx}:")
            print(f"  Configuración articular: {theta_input}")
            print(f"  Posición del efector final: {end_effector_pos}")
            J = compute_jacobian(dh_params, theta_input)
            det_J = np.linalg.det(J)
            print(f"  Determinante del Jacobiano: {det_J:.6f}\n")
