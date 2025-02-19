"""
----------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA GRAFICAR LOS RESULTADOS DE LA CINEMÁTICA 
----------------------------------------------------------------------
"""
#------ PAQUETES IMPORTADOS ------#
import matplotlib.pyplot as plt

#------ FUNCIONES ------#
def graficar_funcion(x_1,y_1,x_2,y_2,x_3,y_3,x,y):
    #Lineas
    plt.plot([0, x_1], [0, y_1], 'r-', label = 'L1') #Linea L1 - L2
    plt.plot([x_1, x_2], [y_1, y_2], 'b-', label = 'L2') #Linea L2 - L3
    plt.plot([x_2, x_3], [y_2, y_3], 'g--', label = 'L3') #Linea L3
    #Puntos
    plt.plot(0, 0, 'ko')
    plt.plot(x_1, y_1, 'ko')
    plt.plot(x_2, y_2, 'ko')
    plt.plot(x_3, y_3, 'ko')
    plt.plot(x, y, marker='o', color = 'red') # Graficamos el punto escogido
    #Formato de la grafica
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Brazo robótico')
    plt.grid(True) #Pone el mallado
    plt.legend()
    plt.show()
