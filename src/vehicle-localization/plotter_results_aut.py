import matplotlib.pyplot as plt
import numpy as np

# Cargar los datos del archivo
data = np.genfromtxt('localization_output/data.txt', delimiter=',', skip_header=1)

# Extraer las columnas relevantes
time = data[:, 0]
x_accel = data[:, 1]
y_accel = data[:, 2]
z_accel = data[:, 3]
speed = data[:, 4]
steer = data[:, 5]
roll = data[:, 6]
pitch = data[:, 7]
yaw = data[:, 8]
x_gps = data[:, 9] 
y_gps = data[:, 10] 
x_odom = data[:, 11]
y_odom = data[:,12]
x_real = data[:, 13]
y_real = data[:, 14]
z_real = data[:, 15]
x_est = data[:, 16]
y_est = data[:, 17]


plt.rcParams.update({
            "text.usetex": True,
            "font.family": "DejaVu Sans"
        })



def plot_trajectory():
    # Crear la gráfica
    plt.plot(x_real, y_real, color='green', label=r"Trayectoria real", zorder=1)
    plt.plot(x_est, y_est, color='orange', label=r"Trayectoria estimada", zorder=1)


    # Marcadores de inicio y fin
    plt.scatter(x_real[0], y_real[0], color='red', s=100, label='Inicio', zorder=2)
    plt.scatter(x_real[-1], y_real[-1], color='blue', s=100, label='Fin', zorder=2)


    # Configurar el aspecto de la gráfica
    plt.xlabel(r"posici\'on x (m)",fontsize=12)
    plt.ylabel(r"posici\'on y (m)",fontsize=12)


    # plt.title('Ruta Real y Estimada')

    # Mostrar la leyenda
    plt.legend()
    # Mostrar la gráfica
    plt.show()


def plot_trajectory_full():
    # Crear la gráfica

    
    # plt.plot(x_est, y_est, color='orange', label='Trayectoria estimada', zorder=1)
    plt.plot(x_odom, y_odom, color='red', label=r'Trayectoria por odometr\'ia', zorder=1)
    plt.plot(x_gps, y_gps, color='blue', label='Trayectoria por GPS', zorder=1)
    plt.plot(x_real, y_real, color='green', label='Trayectoria real', zorder=1, linewidth="2")


    # Marcadores de inicio y fin
    plt.scatter(x_real[0], y_real[0], color='red', s=100, label='Inicio', zorder=2)
    plt.scatter(x_real[-1], y_real[-1], color='blue', s=100, label='Fin', zorder=2)


    # Configurar el aspecto de la gráfica
    plt.xlabel(r"posici\'on x (m)",fontsize=12)
    plt.ylabel(r"posici\'on y (m)",fontsize=12)


    # plt.title('Ruta Real y Estimada')

    # Mostrar la leyenda
    plt.legend()
    # Mostrar la gráfica
    plt.show()



def plot_error():
    # Calcular el error de la trayectoria
    distances1 = np.sqrt((x_real - x_est) ** 2 + (y_real - y_est) ** 2)
    distances2 = np.sqrt((x_real - x_gps) ** 2 + (y_real - y_gps) ** 2)
    distances3 = np.sqrt((x_real - x_odom) ** 2 + (y_real - y_odom) ** 2)
    
   # Calcular los promedios de error para cada conjunto de distancias
    avg_error1 = np.mean(distances1)
    avg_error2 = np.mean(distances2)
    avg_error3 = np.mean(distances3)

    print(avg_error1, avg_error2, avg_error3)

    # Crear la figura y los subgráficos
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

    # Plotear el primer conjunto de distancias
    ax1.set_title('Error de filtro de Kalman')
    ax1.plot(time, distances1, color='blue')
    ax1.set_ylabel(r"Error (m)")

    # Plotear el segundo conjunto de distancias
    ax2.set_title('Error de GPS')
    ax2.plot(time, distances2, color='blue')
    ax2.set_ylabel(r"Error (m)")

    # Plotear el tercer conjunto de distancias
    ax3.set_title(r'Error de Odometr\'ia')
    ax3.plot(time, distances3, color='blue')
    ax3.set_xlabel('Tiempo (s)')
    ax3.set_ylabel(r"Error (m)")

    # Ajustar el espaciado entre subgráficos
    plt.tight_layout()

    # Mostrar la gráfica
    plt.show()

def plot_accels():
    # Calcular el error de la trayectoria
    accel1 = x_accel
    accel2 = y_accel
    accel3 = z_accel
    
   # Calcular los promedios de error para cada conjunto de distancias
    avg_accel1 = np.mean(accel1)
    avg_accel2 = np.mean(accel2)
    avg_accel3 = np.mean(accel3)

    print(avg_accel1, avg_accel2, avg_accel3)

    # Crear la figura y los subgráficos
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

    # Plotear el primer conjunto de distancias
    ax1.set_title('Ruido en el eje X')
    ax1.plot(time, accel1, color='blue')
    ax1.set_ylabel(r"Aceleraci\'on $(m/s^2)$")
    ax1.grid()

    # Plotear el segundo conjunto de distancias
    ax2.set_title('Ruido en el eje Y')
    ax2.plot(time, accel2, color='blue')
    ax2.set_ylabel(r"Aceleraci\'on $(m/s^2)$")
    ax2.grid()

    # Plotear el tercer conjunto de distancias
    ax3.set_title('Ruido en el eje Z')
    ax3.plot(time, accel3, color='blue')
    ax3.set_xlabel('Tiempo (s)')
    ax3.set_ylabel(r"Aceleraci\'on $(m/s^2)$")
    ax3.grid()

    # Ajustar el espaciado entre subgráficos
    plt.tight_layout()
    
    # Mostrar la gráfica
    plt.show()


# plot_error()

# plot_trajectory_full()

plot_accels()