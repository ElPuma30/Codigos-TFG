"""
------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CONSTRUIR LAS TRAYECTORIAS ASI COMO
PARA CALCULAR LOS CAMINOS OPTIMOS PARA RECORRERLAS
------------------------------------------------------------------
"""
import numpy as np
from interpolar import interpolar_angulos


def waypoints_distance(theta_n, theta_m):
    """
    Funcion:
    - Calcula la distancia entre dos vectores de 6 angulos

    Input:
    - theta_n: configuracion angular n
    - theta_m: configuracion angular n + 1

    Retorna:
    - Distancia angular (norma euclidea)
    """
    diff = np.abs((theta_n - theta_m + np.pi) % (2 * np.pi) - np.pi)
    return np.linalg.norm(diff)


def find_best_chains_by_start(waypoint_solutions):
    """
    Funcion:
    - Dada una trayectoria determina el camino que menos distancia supone en el espacio angular para el brazo robotico

    Input:
    - waypoint_solutions: lista de n_waypoints, donde cada elemento es una lista de soluciones IK (array de 6 angulos)

    Parametros:
    - possible_solutions: numero de soluciones para cada waypoint
    - num_waypoints: cantidad total de waypoints
    - cost_min: matriz con el coste minimo para alcanzar cada solucion
    - prev: matriz que almacena el inidice de la solucion anterior
    - start_chain: rastrea la solucion inicial para cada solucion

    Retorna:
    - best_by_start: diccionario con la mejor cadena para cada solucion inicial
    - global_best_chain: cadena completa con el menos coste
    - global_best_cost: coste total mininimo 
    """
    # Calcula el numero total de waypoints en la trayectoria
    num_waypoints = len(waypoint_solutions)
    if num_waypoints == 0:
        return {}, None, None

    # Obtiene el numero de soluciones disponibles en el primer waypoint (habra tantos caminos como soluciones tenga el primer waypoint)
    possible_solutions = len(waypoint_solutions[0])
    cost_min = []
    prev = []
    start_chain = []
    # Para el primer waypoint el coste es 0 y cada solucion inicia su propia cadena
    cost_min.append([0.0 for _ in range(possible_solutions)])
    prev.append([None for _ in range(possible_solutions)])
    start_chain.append([j for j in range(possible_solutions)]) 

    # Itera desde el segundo waypoint hasta el final
    for i in range(1, num_waypoints):
        num_sol_i = len(waypoint_solutions[i]) # Numero de soluciones en el waypoint i
        cost_min_i = [float('inf')] * num_sol_i
        prev_i = [None] * num_sol_i
        start_i = [None] * num_sol_i
        # Recorremos todas las soluciones del waypoint i
        for j in range(num_sol_i):
            # Evaluamos la transicion desde cada solucion del waypoint anterior
            for k in range(len(waypoint_solutions[i-1])):
                # Calculamos el coste de la transicion
                cost = waypoints_distance(waypoint_solutions[i-1][k], waypoint_solutions[i][j])
                new_cost = cost_min[i-1][k] + cost
                # Actualizamos el coste minimo si es necesario
                if new_cost < cost_min_i[j]:
                    cost_min_i[j] = new_cost
                    prev_i[j] = k
                    start_i[j] = start_chain[i-1][k]
        cost_min.append(cost_min_i)
        prev.append(prev_i)
        start_chain.append(start_i)

    # Extraemos la mejor cadena para cada solucion inicial del primer waypoint
    best_by_start = {}
    global_best_cost = float('inf')
    global_best_chain = None
    m_last = len(waypoint_solutions[-1])
    # Para cada solucion inicial del primer waypoint buscamos la mejor cadena
    for s in range(possible_solutions):
        best_cost_for_s = float('inf')
        best_end_index = None 
        for j in range(m_last):
            if start_chain[-1][j] == s and cost_min[-1][j] < best_cost_for_s:
                best_cost_for_s = cost_min[-1][j]
                best_end_index = j
        if best_end_index is not None:
            # Retroceder para reconstruir la cadena para este candidato inicial
            chain = [None] * num_waypoints
            chain[-1] = waypoint_solutions[-1][best_end_index]
            current = best_end_index
            for i in range(num_waypoints - 1, 0, -1):
                current = prev[i][current]
                chain[i-1] = waypoint_solutions[i-1][current]
            best_by_start[s] = (chain, best_cost_for_s)
            # Actualizamos la mejor cadena global si es necesario
            if best_cost_for_s < global_best_cost:
                global_best_cost = best_cost_for_s
                global_best_chain = chain

    return best_by_start, global_best_chain, global_best_cost


def interpolate_trajectory(puntos, num_waypoints_segment):
    """
    Funcion:
    - Interpola la trayectoria entre multiples puntos articulares
    
    Input:
    - puntos: lista de configuraciones articulares
    - num_waypoints_segment: numero de puntos interpolados entre cada par consecutivo
    
    Parametros:
    - segment: lista de configuraciones articulares entre dos puntos  

    Retorna:
    - trayectoria_total: array (en joint space) de configuraciones resultante de la interpolacion
    """
    trayectoria_total = []
    for i in range(len(puntos) - 1):
        segment = interpolar_angulos(puntos[i], puntos[i+1], num_waypoints_segment)
        if i > 0:
            segment = segment[1:]  # evitar duplicar el extremo compartido
        trayectoria_total.extend(segment)
    return np.array(trayectoria_total)


def write_best_trajectory(best_by_start, global_best_chain, global_best_cost):
    """
    Funcion:
    - Imprime la mejor cadena global y las mejores cadenas para cada solución del primer waypoint
    
    Input:
    - best_by_start: diccionario con las mejores cadenas para cada solucion del primer waypoint
    - global_best_chain: mejor cadena global
    - global_best_cost: costo total de la mejor cadena global

    Retorna:
    - Informacion de las mejores cadenas
    """
    print("\n--- Mejores cadenas según cada solución del primer waypoint ---")
    for start_idx, (chain, cost) in best_by_start.items():                       #recorremos el diccionario best_by_start
        print(f"\nDesde la solución {start_idx + 1} del primer waypoint:")
        for i, sol in enumerate(chain):
            print(f"  Waypoint {i+1} solución: {sol}")
        print(f"  Costo total: {cost}")
    
    print("\n--- Cadena global óptima ---")
    for i, sol in enumerate(global_best_chain):
        print(f"Waypoint {i+1} solución: {sol}")
    print("Con una distancia total de:", global_best_cost) 
