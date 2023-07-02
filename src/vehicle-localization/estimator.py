import numpy as np

from kalman import ExtendedFilter

class PositionSpeedFilter:
    def __init__(self, dt, x_init, y_init, vx_init, vy_init, var_model):
        # Frecuencia de muestreo
        self.dt = dt
        # Valores iniciales
        xk = np.array([[x_init, y_init, vx_init, vy_init]]).T   
        uk = np.array([[0.0, 0.0]]).T  
        Pk = var_model*np.eye(4)

        # Matrices caracteristicas (estas son constantes, pero Hk no)

        self.F = np.eye(4)
        self.F[:2, 2:4] = dt * np.eye(2)
        self.L = np.zeros((4,2))
        self.L[2:,:] = np.eye(2)
        self.Q = var_model * dt**2 * np.eye(2)


        self.H = np.zeros((2,4))
        self.H[:2,:2] = np.eye(2)
        I = np.eye(2)
        self.M = I

        # Inicializando filtro de kalman
        self.filter = ExtendedFilter(xk, uk, Pk)


    def prediction_update(self, x_accel, y_accel):
        # En esta funcion se haran las predicciones
        # Primero hace la predicción con lo que ya tenemosf
        p_est = self.filter.xk[:2]
        v_est = self.filter.xk[2:]

        C_ns = np.eye(2) # El simulador CARLA ya realiza las transformaciones con respecto al sistema que queremos 
        accels = np.array([[x_accel, y_accel]]).T

        p_est = p_est + self.dt*v_est + 0.5*(self.dt**2)*(C_ns @ accels)
        v_est = v_est + self.dt*(C_ns@accels)

        f = np.vstack((p_est, v_est))
    
        self.filter.prediction_step(f,self.F,self.L,self.Q)

        
        # Retorna las posiciones x e y
        return self.filter.xk[0][0], self.filter.xk[1][0]

        


    def measurement_update(self, x_pos, y_pos, sensor_var):
        # En esta funcion se haran las correcciones
        I = np.eye(2)
        self.R = I * sensor_var

        y_k = np.array([[x_pos, y_pos]]).T
        p_check = self.filter.xk[:2]

        # print(y_k)

        self.filter.correction_step(yk=y_k,h=p_check, Hk=self.H,Mk=self.M, R=self.R)

        # Retorna las posiciones x e y del filtro
        return self.filter.xk[0][0], self.filter.xk[1][0]


