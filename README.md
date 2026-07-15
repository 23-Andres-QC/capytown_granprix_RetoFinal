# CapyTown G0 Gran Prix

Paquete ROS 2 para navegación autónoma de un laberinto con LiDAR, detección
de señales por cámara, registro de métricas y visualización web.

## Reglas del sistema

- `maze_solver.py` contiene la única lógica de recorrido y es el único nodo
  autorizado para publicar en `/cmd_vel`.
- El robot sigue la pared izquierda.
- Los giros son fijos de 90°, cerrados por odometría. Antes de girar, el robot
  se detiene y verifica el lado.
- La colisión frontal detiene el robot hasta que el camino vuelva a estar libre.
- `visualizador_web.py` solo observa y publica datos HTTP.
- `pare_detector.py` solo publica las detecciones de PARE y META.
- Deben conservarse los tres comandos oficiales de ejecución.
- Los defaults de `maze_solver.py` y `config/navegacion_params.yaml` deben
  permanecer sincronizados.

## Compilación

```bash
cd ~/yahboomcar_ws
colcon build --packages-select capytown_g0_granprix
source install/setup.bash
```

## Ejecución

El sistema utiliza tres terminales permanentes y una cuarta terminal para
calcular e iniciar la ruta corta.

Terminal 1, navegación:

```bash
cd ~/yahboomcar_ws
source install/setup.bash
ros2 run capytown_g0_granprix maze_solver
```

Terminal 2, cámara USB:

```bash
cd ~/yahboomcar_ws
source install/setup.bash
ros2 launch capytown_g0_granprix solo_camara.launch.py
```

Terminal 3, detección, métricas y visualización:

```bash
cd ~/yahboomcar_ws
source install/setup.bash
ros2 launch capytown_g0_granprix visualizacion.launch.py
```

Solo `maze_solver` publica en `/cmd_vel`.

Terminal 4, calcular y mostrar la ruta corta:

```bash
cd ~/yahboomcar_ws
source install/setup.bash
ros2 topic pub --once /maze/calcular_ruta std_msgs/msg/Bool "{data: true}"
```

Este comando carga la ruta fija, la dibuja en amarillo y deja el carrito
detenido. Luego se debe colocar el carrito en el inicio mirando al norte.

En la misma terminal, iniciar el recorrido de la ruta corta:

```bash
ros2 topic pub --once /maze/iniciar_ruta std_msgs/msg/Bool "{data: true}"
```

El segundo comando puede repetirse cada vez que el carrito vuelva a colocarse
en el inicio.

## Componentes

| Archivo | Responsabilidad |
|---|---|
| `maze_solver.py` | Máquina de estados y procesamiento LiDAR. |
| `motion_lidar.py` | Ventanas, distancias y ajuste de pared. |
| `motion_geometry.py` | Operaciones de yaw y ángulos. |
| `motion_grid.py` | Seguimiento lógico de celda. |
| `motion_ruta.py` | Proyección de la ruta fija en la grilla web. |
| `pare_detector.py` | Detecta rojo, amarillo y verde en `/image_raw`. |
| `metrics_logger.py` | Guarda en CSV las métricas de `maze_solver`. |
| `visualizador_web.py` | Expone cámara, LiDAR, odometría y estados por HTTP. |
| `web/index.html` | Dibuja el mapa, cámara, LiDAR, trayectoria y señales. |
| `config/navegacion_params.yaml` | Parámetros de navegación. |
| `config/pare_params.yaml` | Calibración HSV y filtros de forma. |
| `config/metricas_params.yaml` | Parámetros del registro de métricas. |

El comando directo de `maze_solver` usa sus valores por defecto. Deben ser
idénticos a los de `navegacion_params.yaml`.

## Seguimiento de pared izquierda

El LiDAR utiliza `front_offset_deg=180`.

- Frente general: `[-15°, 15°]`.
- Frente angosto para giros: `[-5°, 5°]`.
- Lado izquierdo: `[70°, 110°]`.

El avance recto ajusta una línea por mínimos cuadrados sobre los puntos de la
pared izquierda. Usa lecturas hasta `left_wall_max_range_m=0.50`, rechaza
outliers y suma correcciones de ángulo y distancia hacia
`distancia_objetivo_m=0.12`. Las ganancias son
`ganancia_angulo_recta=2.0` y `ganancia_distancia_recta=2.0`. Sin una línea
válida, el robot avanza recto sin corrección.

## Máquina de estados

Todos los giros son fijos de 90° y se cierran con el yaw de odometría.

```text
AVANZAR_PARALELO
  cada 0.12 m
    -> PAUSA_CHEQUEO_PARED durante 0.5 s
  obstáculo frontal menor a 0.30 m durante 3 ciclos
    -> PAUSA_CHEQUEO_PARED

PAUSA_CHEQUEO_PARED
  lado izquierdo libre mayor a 0.30 m durante 5 ciclos
    -> GIRAR 90° IZQUIERDA
    -> AVANCE_GIRO_VACIO de 0.10 m
    -> repetir hasta 2 veces si el lado continúa libre
  lado ocupado y obstáculo frontal
    -> GIRAR 90° DERECHA
  lado ocupado durante chequeo periódico
    -> AVANZAR_PARALELO
```

El giro utiliza:

- `velocidad_giro_lineal_mps=0.06`.
- `velocidad_giro_angular_radps=0.6`.
- `angulo_giro_deg=90`.
- `factor_ang_odom=0.9899`.

## Seguridad

- La anticolisión frontal se activa a `umbral_colision_m=0.15`. Funciona en
  cualquier estado y mantiene el robot detenido mientras exista el obstáculo.
- La protección lateral se activa a `umbral_lateral_min_m=0.07` y aleja el
  robot de la pared izquierda.
- Si el robot acumula más de 360° girando, ejecuta una corrección de 10° en el
  sentido contrario y vuelve al avance.
- Antes de cada giro, el robot se detiene y verifica el lado.
- `angulo_maximo_giro_deg` limita los giros si la odometría falla.

## Detección de colores

El detector aplica HSV, morfología, área, forma y confirmación temporal.

- Una nueva detección roja activa `FRENO_PARE` durante 5 segundos. Después, la
  navegación continúa desde el mismo estado interno.
- El mapa guarda un marcador `P` por evento.
- El rojo se busca en toda la cámara y no necesita estar centrado.
- El verde se busca en los dos tercios superiores con
  `franja_verde=0.6667`.
- El visualizador registra un solo marcador META cuando su posición estimada
  está en `J1–L2`.
- Los componentes pequeños se eliminan antes de dibujarse.

## Ruta rápida

La segunda fase utiliza un guion fijo definido por `ruta_fija_giros` y
`ruta_fija_distancias_m`. No modifica el mapeo y no depende de haber visto el
verde. Con `ruta_activa=false`, la navegación normal no cambia.

El carrito nunca inicia la ruta solo. Se utilizan dos comandos.

Primero se carga y dibuja la ruta, dejando el carrito detenido:

```bash
cd ~/yahboomcar_ws && source install/setup.bash && \
  ros2 topic pub --once /maze/calcular_ruta std_msgs/msg/Bool "{data: true}"
```

Después se coloca el carrito en el inicio mirando al norte y se inicia:

```bash
ros2 topic pub --once /maze/iniciar_ruta std_msgs/msg/Bool "{data: true}"
```

`calcular_ruta` carga el guion aunque todavía no se haya visto la meta. El
comando `iniciar_ruta` puede repetirse; al terminar, el carrito vuelve a
`ESPERA_RUTA`.

El guion predeterminado es:

1. Avanzar 1.02 m.
2. Girar 90° a la derecha y avanzar 1.02 m.
3. Girar 90° a la izquierda y avanzar 0.55 m.
4. Girar 90° a la derecha y avanzar hasta detectar verde o alcanzar 1.85 m.

El primer y último tramo usan seguimiento de pared. Los tramos centrales usan
odometría. La detección PARE y la anticolisión permanecen activas durante toda
la ruta.

Parámetros relacionados:

- `ruta_activa`.
- `ruta_fija_giros`.
- `ruta_fija_distancias_m`.
- `tamano_celda_m=0.30`.
- `ruta_celda_inicio=A7`.
- `ruta_heading_inicial`.
- `verde_topic`.
- `ruta_topic`.
- `calcular_ruta_topic`.
- `iniciar_ruta_topic`.

La ruta es open-loop y depende de `factor_dist_odom` y `factor_ang_odom`. La
anticolisión evita impactos, pero no corrige toda la deriva lateral.

## Visualizador web

El visualizador muestra:

- Estado de la máquina y descripción del movimiento.
- Velocidad lineal y angular.
- Distancias frontal y laterales.
- Puntos LiDAR en el marco del robot.
- Recorrido calculado desde `/odom_raw`.
- Posiciones PARE y META.
- Ruta rápida en amarillo.

En la laptop, abrir `capytown_G0_granprix/web/index.html` y conectar con:

```text
http://10.42.0.1:8080/data
```

Después de actualizar el frontend, usar `Ctrl+Shift+R`.

## Detención manual

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## Diagnóstico

```bash
ros2 topic hz /scan
ros2 topic hz /image_raw
ros2 topic echo /verde_detectado
ros2 topic echo /pare_detectado
```

## Validación antes de entregar

```bash
python3 -m compileall -q capytown_g0_granprix launch
git diff --check
```
