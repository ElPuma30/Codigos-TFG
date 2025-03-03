from trajectory import *
from constants import dh_params
from trajectories import trajectory_TEST
from compute_waypoints_trajectory import *
from build_trajectory import *
from trajectory_singularities import *
import sys 

with open("salida.txt", "w") as archivo:
    # Redirige la salida estándar al archivo
    sys.stdout = archivo  
    # Fijamos la trayectoria a analizar
    trajectory = trajectory_TEST

    # Obtenemos las soluciones analíticas para cada waypoint
    waypoint_solutions = get_analitic_solutions_for_trajectory(trajectory, dh_params)
    write_waypoint_information(trajectory, waypoint_solutions)

    # Refinamos las soluciones numéricamente (True/False)
    refine_numerically = True # 
    if refine_numerically:
        solutions = refine_solutions_numerically(trajectory, waypoint_solutions, dh_params)
    else:
        solutions = waypoint_solutions

    # Determinamos la mejor cadena para cada solución inicial
    best_by_start, global_best_trajectory, global_best_cost = find_best_chains_by_start(solutions)
    write_best_trajectory(best_by_start, global_best_trajectory, global_best_cost)

    # Ploteamos las trayectorias y verificamos singularidades
    num_steps = 5
    plot_all_trajectories = True # Plotear todas las trayectorias candidatas (False/True)
    plot_and_verify_trajectories(dh_params, num_steps, best_by_start, global_best_trajectory, plot_all_trajectories)
    plot_jacobian_variation(dh_params, num_steps, best_by_start, global_best_trajectory, plot_all_trajectories)

