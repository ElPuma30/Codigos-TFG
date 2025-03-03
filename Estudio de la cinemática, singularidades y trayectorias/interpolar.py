"""
----------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA INTERPOLAR VALORES ENTRE DOS PUNTOS
----------------------------------------------------------------
"""
import numpy as np

def interpolar_angulos(q_inicial, q_final, num_puntos):
    """
    Funcion:
    - Interpola los angulos entre dos puntos

    Input:
    - q_inicial: angulos iniciales
    - q_final: angulos finales
    - num_puntos: numero de puntos a interpolar

    Parametros:
    - delta_q: diferencia entre los angulos iniciales y finales
    - paso: paso a dar en cada iteracion

    Retorna:
    - angulos_interpolados: lista de angulos interpolados
    """
    delta_q = np.array(q_final) - np.array(q_inicial)
    paso = delta_q / num_puntos
    angulos_interpolados = [q_inicial + i * paso for i in range(num_puntos + 1)]
    return angulos_interpolados

