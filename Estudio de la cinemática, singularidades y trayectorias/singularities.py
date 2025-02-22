"""
-----------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA CALCULAR LA POSIBILIDAD DE QUE HAYA SINGULARIDAD
-----------------------------------------------------------------------------
"""
from jacobian import compute_jacobian
import numpy as np

def check_singularity_type(DH_matrix, thetas):
    """
    Funcion:
    - Ver si hay o no singularidad en la configuracion actual

    Input
    - DH_params: matriz de los parametros de DH
    - thetas: los angulos que obtenemos con la cinematica inversa

    Parámetros:
    - J: matriz jacobiana
    - det(J): determinante de la matriz jacobiana
    """
    tol = 1e-4
    J = compute_jacobian(DH_matrix,thetas)
    detJ = np.linalg.det(J)
    print(thetas)
    print("- Determinante del Jacobiano:", detJ)
    sing = np.abs(detJ) < tol
    if sing:
        print("- ¡Atencion! Esta configuracion es potencialmente singular.")
        return "Singularidad"
    else:
        print("- Esta configuracion no es singular.")
        return "No singularidad"

