import numpy as np
from inverse_kinematic import compute_inverse_kinematics
from verificar_graficar_soluciones import verificar_y_graficar_soluciones
from analitic_inverse_kinematic import inverse_kinematic_solution, forward_kinematic_analitic_solution, filtrar_soluciones_unicas
from singularities import check_singularity_type
from constants import *


#--------------------------------------------------#
####################################################
##---- CALCULO CINEMATICA DIRECTA E INVERSA   ----##
####################################################
#--------------------------------------------------#

#---------------------------------------------------#                                                   
#     Calculo de la cinematica directa numerico     #
#---------------------------------------------------# 
##--- Imprimir posicion y orientacion deseadas ---##
print("\n************************************")
print("************************************")
print("**                                **")
print("** POSICION Y ORIENTACION DESEADA **")
print("**                                **")
print("************************************")
print("************************************")
print("\n1. Posición deseada del efector final:")
print(f"x = {desired_pos[0]:.6f}")
print(f"y = {desired_pos[1]:.6f}")
print(f"z = {desired_pos[2]:.6f}")
print("\n2. Matriz de rotación deseada del efector final:")
print(desired_rot)
print("\n")

#---------------------------------------------------#                                                   
#     Calculo de la cinematica inversa numerico     #
#---------------------------------------------------#             
print("\n******************************************************************************")
print("******************************************************************************")
print("**                                                                          **")
print("** RESULTADO DE LOS ANGULOS CALCULADOS CON LA CINEMATICA INVERSA (NUMERICO) **")
print("**                                                                          **")
print("******************************************************************************")
print("******************************************************************************")
print("\n1. Soluciones encontradas con la cinemática inversa numérica")
print("\n")
theta_solutions = compute_inverse_kinematics(dh_params, desired_pos, desired_rot, initial_guesses)
# Imprimir las soluciones encontradas
print("\n2. Soluciones encontradas con la cinemática inversa numérica")
for idx, theta_sol in enumerate(theta_solutions):
    print(f"\n> Solución {idx+1}:")
    print("Radianes:",theta_sol)
    print("Grados:",np.rad2deg(theta_sol))

#-------------------------------------------------------------#                                                   
#     Calculo de la cinematica directa e inversa numerico     #
#-------------------------------------------------------------#  
print("\n*****************************************************************************************")
print("*****************************************************************************************")
print("**                                                                                     **")
print("** RESULTADO DE LOS ANGULOS CALCULADOS CON LA CINEMATICA DIRECTA E INVERSA (ANALITICA) **")
print("**                                                                                     **")
print("*****************************************************************************************")
print("*****************************************************************************************")
# Calcula la transformacion directa analiticamente
print("\n1. Matriz de transformacion cinematica directa ")
if usar_pose_predefinida:
    transform = desired_transform
else:
    transform = forward_kinematic_analitic_solution(dh_params, theta_wanted)
print("\n",transform)
print("\n")
print("\n2. Solucion cinematica inversa")
IKS = inverse_kinematic_solution(dh_params, transform)
IKS1 = np.array(IKS)
num_solutions = IKS1.shape[1]
# Extraemos cada solucion en un formato similar a theta_wanted (6x1)
solutions_list = []
for i in range(num_solutions):
    # IKS[:, i] es la i-esima solucion de 6 elementos
    solution = IKS1[:, i].flatten()                                      # Convierte en array unidimensional
    solutions_list.append(solution)

num_solutions = IKS.shape[1]                                             # Obtiene el número de soluciones
print(f"\nHay {num_solutions} soluciones:")
for i in range(0, num_solutions, 3):
    end_index = min(i + 3, num_solutions)
    print(f"Soluciones {i+1} a {end_index}:")
    soluciones_a_imprimir = IKS[:, i:end_index]
    # Transponer para que cada solucion se muestre como una columna
    print(soluciones_a_imprimir)
    print()
# Filtrar soluciones unicas
print("\n3. Filtrar soluciones únicas")
soluciones_unicas = filtrar_soluciones_unicas(IKS, tolerancia=tol)
for idx, sol in enumerate(soluciones_unicas):
    print(f"\n> Solución {idx+1}:")
    print("Radianes:",sol)
    print("Grados:",np.rad2deg(sol))

#---------------------------------------#
#########################################
##---- VERIFICACION SINGULARIDADES ----##
#########################################
#---------------------------------------#
print("\n************************************************")
print("***************************************************")
print("**                                               **")
print("**  DETECCION DE LAS SINGULARIDADES (NUMERICO)   **")
print("**                                               **")
print("***************************************************")
print("***************************************************")
# Ahora 'solutions_list' contiene cada solucion como un vector numpy de 6 elementos
num_solutions2 = len(theta_solutions)
print(f"\nHay {num_solutions2} soluciones unicas:")
for idx, sol in enumerate(theta_solutions):
    print(f"\n> Solución {idx+1}:")
    print("\n",check_singularity_type(dh_params, sol))

print("\n************************************************")
print("***************************************************")
print("**                                               **")
print("**  DETECCION DE LAS SINGULARIDADES (ANALITICO)  **")
print("**                                               **")
print("***************************************************")
print("***************************************************")
# Ahora 'solutions_list' contiene cada solucion como un vector numpy de 6 elementos
print(f"\nHay {len(soluciones_unicas)} soluciones unicas:")
for idx, sol in enumerate(soluciones_unicas):
    print(f"\n> Solución {idx+1}:")
    print("\n",check_singularity_type(dh_params, sol))


#########################################################
##---- VERIFICACION Y GRFICACION DE LAS SOLUCIONES ----##
#########################################################
print("\n****************************************************")
print("****************************************************")
print("**                                                **")
print("** VERIFICAR Y GRAFICAR LAS SOLUCIONES (NUMERICO) **")
print("**                                                **")
print("****************************************************")
print("****************************************************")
verificar_y_graficar_soluciones(dh_params, theta_solutions, desired_pos, desired_rot, positions_target)
print("\n")

print("\n****************************************************")
print("****************************************************")
print("**                                                **")
print("** VERIFICAR Y GRAFICAR LAS SOLUCIONES (ANALITICO)**")
print("**                                                **")
print("****************************************************")
print("****************************************************")
verificar_y_graficar_soluciones(dh_params, soluciones_unicas, desired_pos, desired_rot, positions_target)
print("\n")

