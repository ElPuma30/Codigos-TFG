"""
------------
CODIGO MAIN
------------
"""

#------ PAQUETES IMPORTADOS ------#
import numpy as np
from cinematica_directa import *
from cinematica_inversa import *
from envolvente import *

#------ ABREVIATURAS ------#
deg2rad = np.deg2rad


#------ CONSTANTES ------#
# Angulos de los brazos 
theta = 1.928367430440407
beta = 0.8556541195038759
alpha = deg2rad(0)
# Puntos a los que queremos ir en metodo indirecto
x_punto = 0.6090073528822821
y_punto = 1.4652496997597595
# Longitudes del brazo
l1 = 1
l2 = 0.7
l3 = 0.5


#------ LLAMADA DE LAS FUNCIONES ------#
# Función método directo
calculo_cinematica_directa(theta, beta, alpha, l1, l2, l3)  
# Función método idirecto
calculo_cinematica_inversa(x_punto, y_punto, alpha, l1, l2, l3)
# Funcion de la envolvente
plot_envolventes_independientes(l1, l2, l3)