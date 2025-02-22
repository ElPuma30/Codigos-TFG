"""
---------------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA CINEMATICA DIRECTA E INVERSA (ANALITICA)
Codigo de la documentacion https://github.com/japersik/UR_kinematics
---------------------------------------------------------------------------------
"""
import numpy as np
from numpy import linalg
from math import pi, asin, cos, sin, atan2, acos, sqrt
from DH_transform import compute_transform_DH

def mat_transtorm_DH(DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    n = n - 1
    t_z_theta = np.matrix([[cos(edges[n]), -sin(edges[n]), 0, 0],
                           [sin(edges[n]), cos(edges[n]), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], copy=False)
    t_zd = np.matrix(np.identity(4), copy=False)
    t_zd[2, 3] = DH_matrix[n, 2]
    t_xa = np.matrix(np.identity(4), copy=False)
    t_xa[0, 3] = DH_matrix[n, 0]
    t_x_alpha = np.matrix([[1, 0, 0, 0],
                           [0, cos(DH_matrix[n, 1]), -sin(DH_matrix[n, 1]), 0],
                           [0, sin(DH_matrix[n, 1]), cos(DH_matrix[n, 1]), 0],
                           [0, 0, 0, 1]], copy=False)
    transform = t_z_theta * t_zd * t_xa * t_x_alpha
    return transform


def forward_kinematic_analitic_solution(DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    t01 = mat_transtorm_DH(DH_matrix, 1, edges)
    t12 = mat_transtorm_DH(DH_matrix, 2, edges)
    t23 = mat_transtorm_DH(DH_matrix, 3, edges)
    t34 = mat_transtorm_DH(DH_matrix, 4, edges)
    t45 = mat_transtorm_DH(DH_matrix, 5, edges)
    t56 = mat_transtorm_DH(DH_matrix, 6, edges)
    answer = t01 * t12 * t23 * t34 * t45 * t56
    return answer


def inverse_kinematic_solution(DH_matrix, transform_matrix,):

    theta = np.matrix(np.zeros((6, 8)))
    # theta 1
    T06 = transform_matrix

    P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])
    psi = atan2(P05[1], P05[0])
    
    # Obtenemos la distancia en el plano xy
    dist_xy = sqrt(P05[0]**2 + P05[1]**2)
    # Evitamos división por cero o muy pequeña
    eps = 1e-12
    if dist_xy < eps:
        # Manejar el caso especial en que P05 está en (0,0) en XY
        # Por ejemplo, asignar phi = 0 o descartar esta solución.
        phi = 0
    else:
        # Calculamos el valor para pasar a acos
        numerador = (DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2])
        valor_acos = numerador / dist_xy
        
        # Clampeamos para evitar dominio fuera de [-1, 1]
        if valor_acos > 1.0:
            valor_acos = 1.0
        elif valor_acos < -1.0:
            valor_acos = -1.0
        
        phi = acos(valor_acos)

    theta[0, 0:4] = psi + phi + pi / 2
    theta[0, 4:8] = psi - phi + pi / 2

    # theta 5
    for i in {0, 4}:
            th5cos = (T06[0, 3] * sin(theta[0, i]) - T06[1, 3] * cos(theta[0, i]) - (
                    DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2])) / DH_matrix[5, 2]
            if 1 >= th5cos >= -1:
                th5 = acos(th5cos)
            else:
                th5 = 0
            theta[4, i:i + 2] = th5
            theta[4, i + 2:i + 4] = -th5
    # theta 6
    for i in {0, 2, 4, 6}:
        # if sin(theta[4, i]) == 0:
        #     theta[5, i:i + 1] = 0 # any angle
        #     break
        T60 = linalg.inv(T06)
        th = atan2((-T60[1, 0] * sin(theta[0, i]) + T60[1, 1] * cos(theta[0, i])),
                   (T60[0, 0] * sin(theta[0, i]) - T60[0, 1] * cos(theta[0, i])))
        theta[5, i:i + 2] = th

    # theta 3
    for i in {0, 2, 4, 6}:
        T01 = mat_transtorm_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transtorm_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transtorm_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])
        costh3 = ((P13[0] ** 2 + P13[1] ** 2 - DH_matrix[1, 0] ** 2 - DH_matrix[2, 0] ** 2) /
                  (2 * DH_matrix[1, 0] * DH_matrix[2, 0]))
        if 1 >= costh3 >= -1:
            th3 = acos(costh3)
        else:
            th3 = 0
        theta[2, i] = th3
        theta[2, i + 1] = -th3

    # theta 2,4
    for i in range(8):
        T01 = mat_transtorm_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transtorm_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transtorm_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
            -DH_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2)
        )
        T32 = linalg.inv(mat_transtorm_DH(DH_matrix, 3, theta[:, i]))
        T21 = linalg.inv(mat_transtorm_DH(DH_matrix, 2, theta[:, i]))
        T34 = T32 * T21 * T14
        theta[3, i] = atan2(T34[1, 0], T34[0, 0])
    return theta

def filtrar_soluciones_unicas(theta_solutions, tolerancia=1e-3):
    """
    Dada la matriz de ángulos (6x8) con hasta 8 posibles soluciones,
    filtra las que sean únicas según una tolerancia 'tolerancia'.

    Retorna una lista de arrays (cada uno con 6 ángulos).
    """
    # Lista para ir acumulando las soluciones "únicas"
    soluciones_unicas = []

    # Recorremos cada columna (cada posible solución) de la matriz
    for i in range(theta_solutions.shape[1]):
        candidata = np.array(theta_solutions[:, i]).flatten()  # (6,) en vez de (6,1)
        es_unica = True
        # Comparamos con las que ya tenemos en "soluciones_unicas"
        for sol_existente in soluciones_unicas:
            # Usamos np.allclose con la tolerancia dada
            if np.allclose(candidata, sol_existente, atol=tolerancia, rtol=0):
                es_unica = False
                break
        if es_unica:
            soluciones_unicas.append(candidata)

    return soluciones_unicas
