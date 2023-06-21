Para poder instalar todas las dependencias de python correr:

```
pip install -r requirements.txt
```

Para el grupo de robótica autónoma, en este folder hay una carpeta llamada [vehicle-control](./vehicle-control/), en la cual se ha implementado el control longitudinal y lateral de un vehículo en un racetrack, de modo que pueda seguir una trayectoria definida. Este control es realizado mediante los valores reales de posicionamiento del vehículo; sin embargo, esto es muy lejano de la realidad pues normalmente estos valores se tienen que estimar. Para ello se planea hacer una fusión de sensores utilizando el filtro de Kalman, en el cual diría que se requiere utilizar el filtro de Kalman extendido. Los sensores que se podrían utilizar son IMU y GPS, combinado con el modelo de odometría, el modelo de ahí les paso. Toda la implementación se debería hacer en la carpeta [vehicle-localization](./vehicle-localization/) . Cualquier cosa me avisan xd.