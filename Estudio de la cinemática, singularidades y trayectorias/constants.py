import numpy as np
from math import pi
from forward_kinematic import compute_forward_kinematic, compute_the_final_position_and_orientation
from rotation_matrix import euler_to_rotation_matrix
from DH_transform import transformation_matrix

#####################################
##---- CONSTANTES DEL PROBLEMA ----##
#####################################

##---- Parámetros DH ----##
ur_params = {
    "DH_matrix_UR5e": np.array([
        [0,           pi / 2.0, 0.1625],
        [-0.425,      0,        0],
        [-0.3922,     0,        0],
        [0,           pi / 2.0, 0.1333],
        [0,          -pi / 2.0, 0.0997],
        [0,           0,        0.0996]
    ], dtype=float),

    "DH_matrix_UR3": np.array([
        [0,           pi / 2.0, 0.1519],
        [-0.24365,    0,        0],
        [-0.21325,    0,        0],
        [0,           pi / 2.0, 0.11235],
        [0,          -pi / 2.0, 0.08535],
        [0,           0,        0.0819]
    ], dtype=float),

    "DH_matrix_UR10": np.array([
        [0,           pi / 2.0, 0.1807],
        [-0.6127,    0,        0],
        [-0.57155,    0,        0],
        [0,           pi / 2.0, 0.17415],
        [0,          -pi / 2.0, 0.11985],
        [0,           0,        0.11655]
    ], dtype=float),

    "DH_matrix_UR30": np.array([
        [0,           pi / 2.0, 0.2363],
        [-0.6370,    0,        0],
        [-0.5037,    0,        0],
        [0,           pi / 2.0, 0.2010],
        [0,          -pi / 2.0, 0.1593],
        [0,           0,        0.1543]
    ], dtype=float),

#### EL CODIGO DEL CALCULO ANALITICO NO COGE ESTOS ANGULOS ####
    "DH_matrix_Puma_260": np.array([
        [0,           -pi / 2.0, 0],
        [203.2,      0,        125.4],
        [-7.9,     pi / 2.0,        0],
        [0,           -pi / 2.0, 203.2],
        [0,          pi / 2.0, 0],
        [0,           0,        63.5]
    ], dtype=float),

    "DH_matrix_Kuka_robot": np.array([
        [750,           -pi / 2.0, 700],
        [1250,      0,        0],
        [-55,     -pi / 2.0,        0],
        [0,           pi / 2.0, 1500],
        [0,          pi / 2.0, 0],
        [0,           180,        -230]
    ], dtype=float)
}
dh_params = ur_params['DH_matrix_UR10']  # Definimos el UR que vamos a usar

##--- Angulos de articulaciones iniciales ---##
theta = {
# TEST HODEI
    "ed" : np.matrix([[1.572584629058838], [-1.566467599277832], [-0.0026149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]]),
    "edd" : np.matrix([[1.672584629058838], [-1.456467599277832], [-0.0036149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]]),
    "edp" : np.matrix([[1.672584629058838], [-1.456467599277832], [-0.0036149749755859375], [-1.568673924808838],
                    [pi], [0.007950782775878906]]),               

#### EL CODIGO DEL CALCULO ANALITICO NO COGE ESTOS ANGULOS ####
# TEST PUMA 260
    "ed1": np.array(np.deg2rad([158.74, -69.84, 41.48, -123.18, -112.69, 23.54]), dtype=float),
    "ed2": np.array(np.deg2rad([135.43, 40.96, 56.52, 52.10, -46.65, -97.70]), dtype=float),
    "ed3": np.array(np.deg2rad([39.41, -6.27, 72.10, 31.87, -92.32, 6.41]), dtype=float),
    "ed4": np.array(np.deg2rad([5.89, -117.66, -30.60, 70.26, -171.14, -135.09]), dtype=float),

# TEST KUKA ROBOT
    "ed5": np.array(np.deg2rad([-63.26, -110.13, 143.95, -59.48, -91.88, 129.90]), dtype=float),
    "ed6": np.array(np.deg2rad([101.30, -9.75, 74.46, -120.66, -86.31, 116.04]), dtype=float),
    "ed7": np.array(np.deg2rad([26.52, 32.09, 13.26, -146.12, -44.21, -125.02]), dtype=float),
    "ed8": np.array(np.deg2rad([-48.42, 112.80, 105.97, -54.26, -139.97, 171.98]), dtype=float),

# PRUEBAS DE SINGULARIDAD
    "posible_sing_codo" : np.matrix([[0], [-pi], [0], [-2.568673924808838], [-1.29446446095601857], [2.47950782775878906]]),
    "sing2" : np.matrix([[0], [-pi], [0], [0], [-1.29446446095601857], [0]]),
    "posible_sing_muneca" : np.matrix([[0], [-pi/2], [pi/2], [0], [0], [0]]),
    "sing4" : np.matrix([[0.5236], [-0.7854], [0.0509], [0], [0], [0]]),
    "sing5" : np.matrix([[2.14198399], [-1.86670222], [-2.95168103], [-2.47475487], [1.14205871], [-0.68441141]]),
    "posible_sing_codo1" : np.matrix([[0], [-pi/2], [0], [0], [pi/2], [0]]),
    "sing_muneca" : np.matrix([[1.572584629058838], [-1.566467599277832], [-0.226149749755859375], [-1.568673924808838], [pi], [0]]),  
    "sing7": np.array(np.deg2rad([45, 90, 0, 30, 0, 0]), dtype=float),       
    "sing8": np.array(np.deg2rad([90, 0, 0, 0, 0, 0]), dtype=float), 
    "sing9": np.array(np.deg2rad([45, 90, 0, 30, 0, 0]), dtype=float),  #Puede ser sing de hombro                          

}
theta_wanted = theta["posible_sing_codo1"]  # Definimos la theta que vamos a usar


##--- Defincion de las distintas condiciones iniciales ---##
initial_guesses = [

    np.array([ 0.00000000e+00, -1.57079633e+00,  0.00000000e+00,  3.14159265e+00, -1.57079633e+00, 6.12323400e-17]),
    np.array([-1.29933461, -1.92834318,  0,          -0.35754686,  1.57079633,  1.29933461]),
    np.array([-1.29933461, -1.92834318,  0,         -2.7840458,  -1.57079633,  1.29933461]),
]

"""
    np.array([ 2.77053565, -0.93681756, 0, 1.43353827, 1.96681153, -2.73074215]),
    np.array([ 2.77053565, -0.80700907, 0, -1.83786288, -1.96681153, -2.73074215]),
    np.array([ 0.31960948, -2.51997845, 0.71435916, -2.1947096, 0.61969062, 0.96088189]),
    np.array([ 0.31960948, -1.85526924, -0.71435916, -1.43070049,  0.61969062,  0.96088189]),
    np.array([ 0.31960948, -1.96405971,  0, -2.03626918, -0.61969062,  0.96088189]),
    np.pi * np.ones(6),
    np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0]),
    np.array([np.pi/2, -np.pi/2, 0, np.pi/2, 0, np.pi/2]),
    np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, 0, np.pi/2]),
"""

"""
Elegimos si bien queremos introducir nosotros la posicion y orientacion del punto deseado o
si queremos elegir los angulos y cojamos el punto que sale de la directa.
    - False: utiliza el resultado de la cinematica directa
    - True: utiliza el punto y orientacion que nosotros le demos
"""
usar_pose_predefinida = False

if usar_pose_predefinida:
    # Definir posición y orientación deseadas directamente
    desired_pos = np.array([-1.06160374,  0.1975718,   0.79163423])    # Modifica estos valores segun las posiciones que queramos
    # Utiliza angulos de Euler para definir la orientacion deseada
    roll = np.deg2rad(0)                                                   # Rotacion alrededor del eje X
    pitch = np.deg2rad(90)                                                    # Rotacion alrededor del eje Y
    yaw = np.deg2rad(0)                                                      # Rotacion alrededor del eje Z
    desired_rot = euler_to_rotation_matrix(yaw, pitch, roll)                 # Matriz de rotacion (puesto yaw, pitch, roll)
    desired_transform = transformation_matrix(desired_rot, desired_pos)      # Matriz de transformacion
    print("\n Matriz de transformacion deseada")
    print(desired_transform)
    _, positions_target = compute_forward_kinematic(dh_params, theta_wanted) # Para poder graficar la real (solo pruebas)
else:
    T_target, positions_target = compute_forward_kinematic(dh_params, theta_wanted)
    desired_pos, desired_rot = compute_the_final_position_and_orientation(T_target)



##--- Tolerancia para considerar soluciones únicas ---##
tol = 1e-3
