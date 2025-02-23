"""
----------------------------------------------------------------------------
ESTE CODIGO LO USAREMOS PARA GRAFICAR LAS CONFIGURACIONES DEL BRAZO ROBOTICO
----------------------------------------------------------------------------
"""
import numpy as np
import matplotlib.pyplot as plt

def plot_robot(positions_list, labels, markers, colors=None, alphas=None, desired_pos=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if colors is None:
        colors = [None] * len(positions_list)
    if alphas is None:
        alphas = [1.0] * len(positions_list)

    for positions, label, marker, color, alpha in zip(positions_list, labels, markers, colors, alphas):
        x_coords = [pos[0] for pos in positions]
        y_coords = [pos[1] for pos in positions]
        z_coords = [pos[2] for pos in positions]
        ax.plot(x_coords, y_coords, z_coords, linestyle='-', marker=marker, color=color, linewidth=2, markersize=6, label=label, alpha=alpha)

    if desired_pos is not None:
        ax.scatter(desired_pos[0], desired_pos[1], desired_pos[2], color='red', s=100, label='Posicion deseada')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("Configuraciones del brazo robotico")
    ax.legend()

    all_x_coords = np.concatenate([[pos[0] for pos in positions] for positions in positions_list])
    all_y_coords = np.concatenate([[pos[1] for pos in positions] for positions in positions_list])
    all_z_coords = np.concatenate([[pos[2] for pos in positions] for positions in positions_list])

    if desired_pos is not None:
        all_x_coords = np.append(all_x_coords, desired_pos[0])
        all_y_coords = np.append(all_y_coords, desired_pos[1])
        all_z_coords = np.append(all_z_coords, desired_pos[2])

    max_range = np.array([all_x_coords.max() - all_x_coords.min(),
                          all_y_coords.max() - all_y_coords.min(),
                          all_z_coords.max() - all_z_coords.min()]).max() / 2.0

    mid_x = (all_x_coords.max() + all_x_coords.min()) / 2.0
    mid_y = (all_y_coords.max() + all_y_coords.min()) / 2.0
    mid_z = (all_z_coords.max() + all_z_coords.min()) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

def plot_joint_space_2D(trayectoria, label):
    """
    Dibuja la trayectoria en el espacio de los ángulos (cada ángulo en función del waypoint).
    
    Parámetros:
      - trayectoria: array de forma (n_waypoints, n_joints)
    """
    trayectoria = np.array(trayectoria)  # Asegurarse de que es un array de NumPy
    n_waypoints, n_joints = trayectoria.shape
    
    plt.figure(figsize=(10, 6))
    for i in range(n_joints):
        plt.plot(range(n_waypoints), trayectoria[:, i], marker='o', label=f'Ángulo {i+1}')
    
    plt.xlabel("Waypoint")
    plt.ylabel("Ángulo (rad)")
    plt.title(f"Espacio Articular - {label}")
    plt.legend()
    plt.grid(True)
    plt.show()