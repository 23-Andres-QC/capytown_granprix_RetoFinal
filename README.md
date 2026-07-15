# CapyTown G0 Gran Prix

Paquete ROS 2 para navegación autónoma de un laberinto con LiDAR, detección
de señales por cámara, registro de métricas y visualización web.

## Reglas del sistema

- `maze_solver.py` contiene la única lógica de recorrido y es el único nodo
  autorizado para publicar en `/cmd_vel`.
- El robot sigue la pared izquierda.
- Los giros son fijos de 90°, cerrados por odometría. Antes de girar, el robot
  se detiene y verifica el lado.
- La protección frontal ejecuta una maniobra de recuperación y mantiene el
  robot detenido si el obstáculo continúa presente.
- `visualizador_web.py` solo observa y publica datos HTTP.
- `pare_detector.py` solo publica las detecciones de PARE, META y amarillo.
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

## Arquitectura, librerías y flujo de datos

El paquete está escrito en Python para ROS 2 y separa percepción, control,
registro y presentación. Las librerías principales son:

| Librería o paquete | Uso dentro del proyecto |
|---|---|
| `rclpy` | Nodos ROS 2, parámetros, reloj, timers, publishers y subscribers. |
| `sensor_msgs` | Recibe `LaserScan`, `Image` e `Imu`. |
| `nav_msgs` | Recibe `Odometry` desde `/odom_raw`. |
| `geometry_msgs` | Publica comandos `Twist` en `/cmd_vel`. |
| `std_msgs` | Señales booleanas, eventos JSON, métricas y orden del beep. |
| `numpy` | Vectores LiDAR, máscaras y cálculo numérico. |
| `OpenCV` (`cv2`) | Conversión BGR/HSV, morfología, contornos y JPEG. |
| `cv_bridge` | Convierte imágenes ROS a OpenCV y viceversa. |
| `ament_index_python` | Encuentra la carpeta instalada del paquete en los launch. |
| `launch` y `launch_ros` | Arranque de cámara, detector, métricas y visualizador. |
| `http.server` | Expone la instantánea del visualizador mediante `GET /data`. |
| `json`, `csv` | Intercambio de telemetría y persistencia de métricas. |

Flujo principal:

```text
/scan ---------> maze_solver ---------> /cmd_vel
                    |  |  |
/odom_raw ----------+  |  +-----------> /maze/estado
/imu ------------------+---------------> /maze/metricas
/pare_detectado --------+--------------> /robot_event
/verde_detectado -------+--------------> /maze/ruta_corta

/image_raw ----> pare_detector ----> /pare_detectado
                      |  |  +-------> /verde_detectado
                      |  +----------> /amarillo_detectado
                      +-------------> /pare/debug_image y /beep

sensores y tópicos ---> visualizador_web ---> HTTP :8080/data
HTTP :8080/data ------> web/index.html ------> mapa y paneles
/maze/metricas -------> metrics_logger -----> ~/metricas_granprix.csv
```

La navegación se evalúa a `control_rate_hz=20.0`, es decir, cada 50 ms cuando
ya existen odometría y una lectura LiDAR válida.

## Cámara y detección de colores

### Captura

`solo_camara.launch.py` inicia `usb_cam_node_exe` con `/dev/video0`, resolución
`640x480`, formato `yuyv`, 30 FPS y brillo 50. La imagen se remapea de
`/usb_cam/image_raw` a `/image_raw`. Tras dos segundos también publica tres
veces las posiciones de servo `S1=0` y `S2=-5`.

El detector convierte cada cuadro de BGR a HSV. HSV separa tono (`H`),
saturación (`S`) y brillo (`V`), por lo que los umbrales de color son más
estables que comparaciones directas en RGB. En OpenCV, `H` usa el rango
0–179 y `S`/`V` usan 0–255.

### Configuración HSV exacta

| Color | Rango HSV | Región de imagen | Propósito |
|---|---|---|---|
| Rojo | `H=0..10` o `170..179`, `S=100..255`, `V=70..255` | `franja_inferior=1.0`: cuadro completo | Señal PARE. El tono se divide porque el rojo cruza el extremo de la rueda HSV. |
| Verde | `H=35..95`, `S=40..255`, `V=60..255` | `franja_verde=0.6667`: dos tercios superiores | Meta verde. |
| Amarillo | `H=20..35`, `S=100..255`, `V=80..255` | cuadro completo | Indicador publicado para integración externa y aviso acústico. |

Todos estos valores se cambian en `config/pare_params.yaml`.

### Algoritmo de visión

Para cada color se ejecutan estas etapas:

1. Recortar la región configurada y convertirla con `cv2.cvtColor(...,
   COLOR_BGR2HSV)`.
2. Crear la máscara binaria con `cv2.inRange`; para rojo se unen los dos
   intervalos con `cv2.bitwise_or`.
3. Aplicar apertura y cierre morfológico con un kernel `5x5`. La apertura
   elimina ruido aislado y el cierre rellena pequeños huecos.
4. Eliminar componentes conectados menores al área mínima.
5. Extraer contornos externos y validar área, relación ancho/alto y solidez
   `área_contorno / área_convexHull`.
6. Elegir el contorno válido de mayor área.
7. Exigir `frames_confirmacion=3` cuadros consecutivos antes de publicar una
   detección verdadera. Al perderla, el contador vuelve a cero.

Filtros de forma exactos:

| Color | Área válida | Aspecto `ancho/alto` | Solidez mínima | ¿Debe estar centrado? |
|---|---:|---:|---:|---|
| Rojo | 600–60,000 px² | 0.5–2.0 | 0.75 | No |
| Verde | 600–150,000 px² | 0.05–20.0 | 0.35 | No |
| Amarillo | desde 600 px², máximo general 60,000 px² | 0.5–2.0 | 0.75 | Sí, dentro de ±20% del ancho respecto al centro |

Cuando aparece por primera vez rojo confirmado o amarillo confirmado, se
publica un beep de 150 ms en `/beep`. `beep_cooldown_s=5.0` evita repetirlo
continuamente. `/pare/area` publica el área del mejor contorno rojo.

Si `publish_debug=true`, `/pare/debug_image` muestra las máscaras como una
capa semitransparente: rojo `(BGR 0,0,255)`, verde `(0,200,0)` y amarillo
`(0,220,255)`. El contorno pasa a verde brillante cuando queda confirmado y
se dibujan su caja, nombre y área.

### Efecto de cada color

- Rojo: `/pare_detectado=true`; `maze_solver` frena cinco segundos y luego
  ignora nuevas activaciones hasta avanzar 0.60 m desde ese PARE.
- Verde: `/verde_detectado=true`; registra la meta y, en el último tramo de la
  ruta fija, puede terminar el avance antes de llegar a 1.85 m.
- Amarillo: `/amarillo_detectado=true`; no cambia directamente `/cmd_vel` en
  este paquete. El cálculo real se ordena por `/maze/calcular_ruta`.

## Seguimiento de pared izquierda

El sensor publica `sensor_msgs/LaserScan` en `/scan`. El código calcula el
ángulo de cada muestra como `angle_min + índice * angle_increment`, aplica
`front_offset_deg=180` para hacer coincidir el cero con el frente físico y
normaliza el resultado a `[-π, π)`. `invert_left_right=false`; se puede activar
si el montaje invierte los lados. Solo se usan distancias finitas dentro de
`range_min`, `min(range_max, 4.0 m)`.

### Ventanas angulares exactas

Los ángulos negativos corresponden al lado derecho y los positivos al
izquierdo:

| Zona | Ventana | Uso |
|---|---:|---|
| Frente | -15° a 15° | Obstáculo frontal general. |
| Frente angosto | -5° a 5° | Confirmación frontal durante la lógica de dos reglas. |
| Frente-derecha | -75° a -45° | Observación diagonal derecha. |
| Derecha | -110° a -70° | Distancia lateral derecha y ajuste de línea. |
| Trasera-derecha | -135° a -105° | Alineación en el modo convencional. |
| Izquierda | 70° a 110° | Distancia lateral y ajuste de la pared seguida. |
| Frente-izquierda | 65° a 85° | Detección temprana de una apertura izquierda. |

La distancia de una zona es el mínimo de sus lecturas válidas. Si no existe
ninguna, se devuelve infinito junto con `valid=false`, para no confundir
“sin lectura” con “camino libre”.

### Algoritmo de detección de pared para control

Para las ventanas laterales se conservan puntos hasta 0.50 m y se convierten
de polar a cartesiano:

```text
x = rango * cos(ángulo)
y = rango * sin(ángulo)
```

Con al menos seis puntos se ajusta `y = m*x + b` mediante mínimos cuadrados
(`numpy.polyfit`). Se calcula la distancia perpendicular de cada punto a la
recta y se descartan outliers con residuo mayor o igual a 0.03 m. El ajuste se
repite como máximo tres veces. El resultado usado por el controlador es:

```text
ángulo_pared    = atan(m)
distancia_pared = abs(b) / sqrt(m² + 1)
```

El seguimiento busca mantener la pared izquierda a 0.12 m. La corrección
angular es:

```text
error_distancia = 0.12 - distancia_pared
corrección = 2.0 * ángulo_pared - 2.0 * error_distancia
```

La corrección se limita a ±0.6 rad/s y el avance normal es 0.15 m/s. Si la
distancia lateral baja de 0.07 m, se ordena la corrección máxima para alejarse.
Si no hay línea válida, se avanza recto a 0.15 m/s.

### Detección y dibujo LiDAR en la web

El visualizador toma hasta unas 360 muestras distribuidas uniformemente,
descarta medidas inválidas o mayores de 3.5 m y transforma cada punto al marco
del robot. Para el panel polar muestra solo el campo configurado de 180° y
para detectar paredes conserva puntos hasta 1.4 m.

Las paredes dibujadas no usan el `polyfit` del controlador. El visualizador
aplica un detector robusto alineado con los ejes:

1. Separa regiones izquierda, derecha, frontal y trasera.
2. Agrupa distancias en bins de 0.025 m.
3. Busca vecinos dentro de ±0.040 m del centro del bin.
4. Calcula la mediana, vuelve a filtrar inliers a ±0.040 m y separa grupos
   continuos; el salto máximo permitido entre puntos es 0.09 m.
5. Exige al menos ocho candidatos, seis vecinos y cinco puntos en el grupo.
6. Exige una extensión mínima de 0.16 m para izquierda, derecha y frente, y
   0.12 m para atrás.
7. Recorta 15% de las distancias en cada extremo, calcula la media y elige la
   pared con mayor puntuación `n_puntos * extensión / (error + 0.01)`.

El backend entrega como máximo 240 puntos de pared y ocho segmentos de hasta
120 puntos. `index.html` transforma metros a coordenadas del lienzo, dibuja el
LiDAR alrededor del carrito, superpone los segmentos detectados, la
trayectoria de odometría, los PARE, la meta y la ruta amarilla.

La grilla web representa celdas de 0.30 m. El recorrido inicia visualmente en
`A8`, en el centro `(0.5, 7.5)`. La trayectoria agrega un punto cada 0.02 m y
conserva como máximo 4,000 puntos. Por defecto la calibración inicial con
LiDAR está desactivada porque `track_lidar_init_m=0.0` y
`track_lidar_blend=0.0`.

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

### Estados y funciones principales

| Estado o función | Responsabilidad |
|---|---|
| `INICIAR` | Registra tiempo, celda `A4` y rumbo norte; prepara el avance. |
| `AVANZAR_PARALELO` | Sigue la pared y evalúa apertura lateral, distancia recorrida y frente. |
| `PAUSA_CHEQUEO_PARED` | Publica velocidad cero durante al menos 0.5 s y confirma el lado. |
| `GIRAR` | Ejecuta el giro físico y compara yaw de odometría con IMU. |
| `AVANCE_GIRO_VACIO` | Avanza 0.10 m después de girar hacia un espacio abierto. |
| `DETECTAR_CRUCE` | En el modo alternativo toma cinco muestras y exige consenso 4/5. |
| `BUSCAR_PARE` | Espera hasta 0.5 s por la cámara antes de decidir. |
| `DECIDIR` | Prioriza derecha, frente, izquierda y finalmente giro de 180°. |
| `PAUSA_GIRO` | Mantiene velocidad cero durante 1.0 s antes del giro convencional. |
| `ALINEAR` | Compara sectores frontal/trasero derechos; tolerancia 0.02 m y timeout 4 s. |
| `VERIFICAR_META` | Compara la celda lógica con `F1`. |
| `CORREGIR_GIRO` | Compensa 10° si la rotación acumulada supera 360°. |
| `ESPERA_RUTA` | Mantiene el robot detenido y republica la ruta una vez por segundo. |
| `SEGUIR_RUTA` | Ejecuta los cuatro tramos fijos mediante yaw y odometría. |
| `META` / `DETENIDO` | Publica `Twist` cero. |

Funciones auxiliares relevantes:

- `compute_zone_distance`: mínimo válido dentro de una ventana LiDAR.
- `fit_wall_line`: recta lateral robusta y distancia perpendicular.
- `yaw_from_quaternion`, `normalize_angle`, `angle_diff`: conversión y
  diferencias angulares sin errores al cruzar ±π.
- `GridTracker`: celda y orientación lógica de la primera fase.
- `RouteExplorer`: simula la ruta fija para dibujar sus celdas.
- `_twist_wall_follow`: combina error angular y error de distancia.
- `_handle_obstaculo_frente`: recuperación frontal independiente del flujo.
- `_metricas_actuales`: genera tiempo, longitud, colisiones y PARE.

### Distancias, tiempos y velocidades exactas

| Parámetro | Valor | Efecto |
|---|---:|---|
| `velocidad_recta_mps` | 0.15 m/s | Avance normal. |
| `distancia_chequeo_pared_m` | 0.12 m | Intervalo entre verificaciones laterales. |
| `tiempo_chequeo_pared_s` | 0.5 s | Pausa antes de decidir con el lado. |
| `umbral_frente_pared_m` | 0.30 m | Frente considerado bloqueado para decidir giro. |
| `umbral_frente_libre_m` | 0.35 m | Frente considerado libre en el modo convencional. |
| `umbral_lado_libre_m` | 0.30 m | Apertura lateral. |
| `avance_giro_vacio_m` | 0.10 m | Avance entre giros hacia una apertura. |
| `giro_vacio_max_repeticiones` | 2 | Giros adicionales máximos si sigue abierto. |
| `velocidad_giro_lineal_mps` | 0.06 m/s | Componente lineal del giro. |
| `velocidad_giro_angular_radps` | 0.60 rad/s | Componente angular del giro. |
| `angulo_giro_deg` | 90° | Giro nominal. |
| `angulo_maximo_giro_deg` | 150° | Tope defensivo en giro dinámico. |
| `tolerancia_giro_deg` | 4° | Tolerancia del giro convencional. |
| `umbral_patinaje_deg` | 8° | Diferencia odom/IMU que genera evento. |
| `tiempo_pausa_antes_girar_s` | 1.0 s | Pausa previa en modo convencional. |
| `factor_dist_odom` | 0.9474 | Calibración multiplicativa de distancia. |
| `factor_ang_odom` | 0.9899 | Calibración multiplicativa del yaw. |
| `max_celdas_recorridas` | 60 | Límite lógico de exploración. |

`distancia_celda_m=5.0` y `margen_avance_m=0.05` pertenecen al modo
convencional (`logica_dos_reglas=false`). No son el tamaño gráfico de celda;
la ruta web usa `tamano_celda_m=0.30`.

## Seguridad

- La anticolisión frontal se activa si el sector frontal o el sector angosto
  marcan menos de 0.15 m durante tres ciclos (150 ms a 20 Hz). Ejecuta un
  retroceso de 0.10 m a -0.06 m/s y -0.9 rad/s, luego avanza 0.10 m a
  0.15 m/s. Después espera; si el frente continúa bloqueado, permanece con
  `Twist` cero. El temporizador interno de espera se reinicia cada 2 s.
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

## Tópicos ROS 2

| Tópico | Tipo | Nodo productor | Uso |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR | Zonas, paredes, aperturas y colisión. |
| `/odom_raw` | `nav_msgs/Odometry` | Base | Distancia, yaw, trayectoria y cierre de giros. |
| `/imu` | `sensor_msgs/Imu` | IMU | Comparación de giro y posible patinaje. |
| `/image_raw` | `sensor_msgs/Image` | `usb_cam` | Entrada del detector. |
| `/cmd_vel` | `geometry_msgs/Twist` | `maze_solver` | Única salida de movimiento del paquete. |
| `/pare_detectado` | `std_msgs/Bool` | `pare_detector` | Freno PARE. |
| `/verde_detectado` | `std_msgs/Bool` | `pare_detector` | Meta y fin anticipado del último tramo. |
| `/amarillo_detectado` | `std_msgs/Bool` | `pare_detector` | Indicador amarillo para integración externa. |
| `/beep` | `std_msgs/UInt16` | `pare_detector` | Duración del beep en milisegundos. |
| `/pare/area` | `std_msgs/Float32` | `pare_detector` | Área del mejor contorno rojo. |
| `/pare/debug_image` | `sensor_msgs/Image` | `pare_detector` | Imagen anotada para diagnóstico/web. |
| `/maze/estado` | `std_msgs/String` | `maze_solver` | Estado actual. |
| `/maze/metricas` | `std_msgs/String` JSON | `maze_solver` | Comando, distancias y métricas acumuladas. |
| `/robot_event` | `std_msgs/String` JSON | `maze_solver` | Inicio, giro, PARE, colisión, meta y timeout. |
| `/maze/ruta_corta` | `std_msgs/String` JSON | `maze_solver` | Lista de celdas proyectadas. |
| `/maze/calcular_ruta` | `std_msgs/Bool` | operador | Carga y dibuja el guion fijo. |
| `/maze/iniciar_ruta` | `std_msgs/Bool` | operador | Ejecuta el guion fijo. |

## Métricas y archivo CSV

`maze_solver` acumula distancia euclidiana entre muestras consecutivas de
odometría ya calibrada, tiempo desde `INICIAR`, colisiones y PARE detectados y
respetados. Publica esos datos en JSON junto con `v`, `w`, distancias LiDAR,
estado y cantidad de giros físicos.

`metrics_logger` escucha `/maze/metricas` y guarda una fila cuando
`llego_meta=true`; al apagar también intenta guardar la última instantánea si
todavía no escribió. El archivo predeterminado es
`~/metricas_granprix.csv`. Sus columnas son ronda, fecha, llegada, tiempo,
longitud real, longitud óptima, eficiencia, colisiones, PARE y dead ends. La
eficiencia se calcula como `long_optima_cm / long_ruta_cm`, con longitud óptima
predeterminada de 520 cm.

## Visualizador web

El visualizador muestra:

- Estado de la máquina y descripción del movimiento.
- Velocidad lineal y angular.
- Distancias frontal y laterales.
- Puntos LiDAR en el marco del robot.
- Recorrido calculado desde `/odom_raw`.
- Posiciones PARE y META.
- Ruta rápida en amarillo.

El backend se enlaza a `0.0.0.0:8080`, habilita CORS y responde JSON sin caché
en `/data`. La cámara se limita por defecto a 360 px de ancho, 15 FPS y calidad
JPEG 45 para reducir tráfico. El frontend consulta periódicamente la URL que
se indique, guarda el endpoint en `localStorage` y actualiza los textos con
`textContent`.

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
