import numpy as np

from kalman import ExtendedFilter

class PositionFilter:
    def __init__(self, dt, x_init, y_init, var_speed, var_yaw):
        # Frecuencia de muestreo
        self.dt = dt
        # Valores iniciales
        xk = np.array([[x_init, y_init]]).T   
        uk = np.array([[0.0, 0.0]]).T  
        Pk = np.diag([var_speed, var_yaw])

        # Matrices caracteristicas (estas son constantes, pero Hk no)

        self.F = np.eye(2)
        self.Q = np.diag([var_speed, var_yaw])


        self.H = np.eye(2)
        I = np.eye(2)
        self.M = I

        # Inicializando filtro de kalman
        self.filter = ExtendedFilter(xk, uk, Pk)


    def prediction_update(self, vel, yaw):
        # En esta funcion se haran las predicciones
        # Primero hace la predicción con lo que ya tenemosf
        p_est = self.filter.xk
        # print(p_est)
        f_v = np.array([[np.cos(yaw)*self.dt,
                    np.sin(yaw)*self.dt]]).T
        p_est = p_est + f_v*vel

        f = p_est

        self.L = np.array([[np.cos(yaw)*self.dt, np.sin(yaw)*self.dt],
                    [-vel*self.dt*np.sin(yaw), vel*self.dt*np.cos(yaw)]]).T
        
        self.filter.prediction_step(f,self.F,self.L,self.Q)

        
        # Retorna las posiciones x e y
        # print(self.filter.xk[0][0], self.filter.xk[1][0])
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


