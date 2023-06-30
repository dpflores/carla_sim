from kalman import ExtendedFilter

class PositionSpeedFilter:
    def __init__(self, dt, x_init, y_init, vx_init, vy_init, var_model):
        # Frecuencia de muestreo

        # Valores iniciales
        self.xk = np.array([[x_init, y_init, vx_init, vy_init]]).T   
        self.uk = np.array([[0.0, 0.0]]).T  
        self.Pk = var_model*np.eye(3)

        # Matrices caracteristicas (estas son constantes, pero Hk no)

        self.F = np.eye(4)
        self.F[:2, 2:4] = dt * np.eye(4)
        self.L = np.zeros((4,2))
        self.L[2:,:] = np.eye(2)
        self.Q = var_imu_f * dt**2 * np.eye(2)


        self.H = np.zeros((3,6))
        self.H[:3,:3] = np.eye(3)
        I = np.eye(3)
        self.M = I
        


        # Inicializando filtro de kalman
        self.filter = kalman.ExtendedFilter(self.xk, self.uk, self.Pk)


    def prediction_update(self, x_accel, y_accel):
        # En esta funcion se haran las predicciones
        # Primero hace la predicción con lo que ya tenemosf
        C_ns = np.eye(2) # El simulador CARLA ya realiza las transformaciones con respecto al sistema que queremos


        p_est = p_est[k-1] + delta_t*v_est[k-1] + 0.5*(delta_t**2)*(C_ns@imu_f["data"][k-1])

        v_est[k] = v_est[k-1] + delta_t*(C_ns@imu_f["data"][k-1])


        f = np.hstack((p_est[k], v_est[k])).T
    
        filter.prediction_step(f,F,L,Q)
        return x_est, y_est

        


    def measurement_update(self, x_gps, y_gps):
        # En esta funcion se haran las correcciones
        self.R = I * sensor_var


        vx = self.speed_filter.xk[0,0]
        vy = self.speed_filter.xk[1,0]
        vz = self.speed_filter.xk[2,0]

        # if vx<0.0001:
        #     vx = 0.0001
        va = (vx**2 + vy**2 + vz**2)**(-1/2)    # constante solo para facilitar el trabajo

        h = np.sqrt(vx**2 + vy**2 + vz**2)

        self.Hk = np.array([[vx*va, vy*va, vz*va]])

        # Primero hace la correción con lo que ya tenemos
        self.filter.correction_step(h, self.Hk,self.Mk, self.Rk)

        return x_est, y_est


