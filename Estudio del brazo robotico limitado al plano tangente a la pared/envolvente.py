"""
---------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA ENVOLVENTE DEL BRAZO ROBOTICO
---------------------------------------------------------------------------
"""

# ------ PAQUETES IMPORTADOS ------ #
import numpy as np
import matplotlib.pyplot as plt


# ------ FUNCIONES ------ #
def envolvente_eslabon1(l1, num_puntos=300):
    """
    Funcion:
    - Calcular la envolvente del primer eslabon del brazo robotico

    Input:
    - l1: longitud del primer eslabon
    - num_puntos: número de puntos a calcular en la envolvente

    Parametros:
    - theta: angulo de rotacion alrededor del eje Z (en radianes)

    Retorna:
    - x: coordenada en el eje X
    - y: coordenada en el eje Y
    """
    theta = np.linspace(0, 2*np.pi, num_puntos)
    x = l1 * np.cos(theta)
    y = l1 * np.sin(theta)
    return x, y

def envolvente_eslabon2(l1, l2, num_puntos=100):
    """
    Funcion:
    - Calcular la envolvente del segundo eslabón del brazo robotico

    Input
    - l1: longitud del primer eslabon
    - l2: longitud del segundo eslabon
    - num_puntos: numero de puntos a calcular en la envolvente

    Parametros:
    - q1: angulo de rotación del hombro
    - q2: angulo de rotación del codo

    Retorna:
    - x: coordenada en el eje X
    - y: coordenada en el eje Y
    """
    q1 = np.linspace(0, 2*np.pi, num_puntos)
    q2 = np.linspace(0, 2*np.pi, num_puntos)
    Q1, Q2 = np.meshgrid(q1, q2)
    Q1 = Q1.flatten()
    Q2 = Q2.flatten()
    
    x = l1 * np.cos(Q1) + l2 * np.cos(Q2)
    y = l1 * np.sin(Q1) + l2 * np.sin(Q2)
    return x, y

def envolvente_eslabon3(l1, l2, l3, num_puntos=5000):
    """
    Funcion:
    - Calcular la envolvente del tercer eslabón del brazo robótico

    Input
    - l1: longitud del primer eslabon
    - l2: longitud del segundo eslabon
    - l3: longitud del tercer eslabon
    - num_puntos: numero de puntos a calcular en la envolvente

    Parametros:
    - q1: angulo de rotación del hombro
    - q2: angulo de rotación del codo
    - q3: angulo de rotación del eslabón final

    Retorna:
    - x: coordenada en el eje X
    - y: coordenada en el eje Y
    """
    q1 = np.random.uniform(0, 2*np.pi, num_puntos)
    q2 = np.random.uniform(0, 2*np.pi, num_puntos)
    q3 = np.random.uniform(0, 2*np.pi, num_puntos)
    
    x = l1 * np.cos(q1) + l2 * np.cos(q2) + l3 * np.cos(q3)
    y = l1 * np.sin(q1) + l2 * np.sin(q2) + l3 * np.sin(q3)
    return x, y

def plot_envolventes_independientes(l1, l2, l3):
    # Obtener las coordenadas para cada eslabon
    x1, y1 = envolvente_eslabon1(l1)
    x2, y2 = envolvente_eslabon2(l1, l2)
    x3, y3 = envolvente_eslabon3(l1, l2, l3)
    
    # Graficar
    plt.figure(figsize=(8,8))
    
    # Eslabon 1: se grafica como linea continua (circulo)
    plt.plot(x1, y1, label='Eslabón 1', color='red', linewidth=2)
    
    # Eslabon 2: se grafica con puntos pequeños
    plt.scatter(x2, y2, s=1, label='Eslabón 2', color='blue', alpha=0.5)
    
    # Eslabon 3: se grafica con puntos pequeños
    plt.scatter(x3, y3, s=1, label='Eslabón 3', color='green', alpha=0.5)
    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Envolvente del brazo robótio')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

