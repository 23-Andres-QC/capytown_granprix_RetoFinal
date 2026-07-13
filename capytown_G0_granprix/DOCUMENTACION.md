# CapyTown G0 Gran Prix

## Ejecución

Compilar dentro del contenedor:

```bash
cd ~/yahboomcar_ws
colcon build --packages-select capytown_g0_granprix
source install/setup.bash
```

El sistema se ejecuta con tres terminales:

```bash
cd ~/yahboomcar_ws && source install/setup.bash && ros2 run capytown_g0_granprix maze_solver
```

```bash
cd ~/yahboomcar_ws && source install/setup.bash && ros2 launch capytown_g0_granprix solo_camara.launch.py
```

```bash
cd ~/yahboomcar_ws && source install/setup.bash && ros2 launch capytown_g0_granprix visualizacion.launch.py
```

Solo `maze_solver` publica `/cmd_vel`.

Para la **ruta más corta** (fase 2) el carrito nunca parte solo: son **dos
comandos** (una cuarta terminal). Primero calcular (pinta el amarillo y deja el
carrito detenido); después, ya con el carrito colocado en el inicio mirando
NORTE, partir:

```bash
# 1) calcular la ruta (la pinta de amarillo, el carrito queda DETENIDO)
ros2 topic pub --once /maze/calcular_ruta std_msgs/msg/Bool "{data: true}"
# 2) partir (recién aquí arranca y maneja la ruta)
ros2 topic pub --once /maze/iniciar_ruta std_msgs/msg/Bool "{data: true}"
```

Ver la sección «Fase 2: ruta más corta» más abajo.

## Componentes

| Archivo | Responsabilidad |
|---|---|
| `maze_solver.py` | FSM de referencia + procesamiento de zonas LiDAR integrado. |
| `motion_lidar.py` | Ventanas, distancias y ajuste de recta copiados de la referencia. |
| `motion_geometry.py` / `motion_grid.py` | Yaw, ángulos y seguimiento lógico de celda. |
| `pare_detector.py` | Detecta rojo y verde en `/image_raw`; publica imagen de depuración. |
| `visualizador_web.py` | Publica por HTTP cámara, LiDAR, odometría, estados y recorrido. |
| `web/index.html` | Dibuja cámara, puntos LiDAR, mapa, trayectoria, PARE y META. |
| `config/navegacion_params.yaml` | Copia documentada de los parámetros incorporados en `maze_solver`. |
| `config/pare_params.yaml` | Calibración HSV y filtros de forma. |

El comando directo `ros2 run ... maze_solver` usa los valores por defecto de
`maze_solver.py`; deben mantenerse iguales a `navegacion_params.yaml`.

## Seguimiento de pared izquierda

El LiDAR se corrige con `front_offset_deg=180`. La navegación es un port
COMPLETO de la referencia (Reto-Final-ROBOTICA-Yahboom-ROSMASTER), con todos
sus parámetros y su procesamiento por AJUSTE DE LÍNEA:

- **frente** (colisión / general): cono `[-15°, 15°]`.
- **frente angosto** (disparo de giro): cono `[-5°, 5°]`.
- **lado seguido** (izquierda): ventana `[70°, 110°]`, para la distancia
  PUNTUAL (ocupado/vacío) y para el ajuste de recta.

El seguimiento recto ajusta una RECTA por mínimos cuadrados a los puntos del
lado izquierdo (rango propio hasta `left_wall_max_range_m = 0.50 m`, con
rechazo iterativo de outliers) y corrige ÁNGULO + DISTANCIA SUMADOS hacia
`distancia_objetivo_m = 0.12 m` (`ganancia_angulo_recta=2.0`,
`ganancia_distancia_recta=2.0`). Sin
recta válida, avanza recto sin corregir.

## Máquina de estados (lógica de la referencia)

Todos los giros son FIJOS de 90° cerrados por odometría (no reactivos), así no
se pasan ni se encadenan.

```text
AVANZAR_PARALELO (recto a 0.15 m/s, corrigiendo pared izquierda)
  cada distancia_chequeo_pared_m (0.12 m) de avance recto
    -> PAUSA_CHEQUEO_PARED (detenido 0.5 s)
  obstáculo al frente < umbral_frente_pared_m (0.30 m) sostenido
  frente_confirmaciones_ciclos (3)
    -> CHEQUEO_PARED (chequeo_por_frente)

PAUSA_CHEQUEO_PARED verifica la distancia lateral del lado seguido:
  lado VACÍO (> umbral_lado_libre_m, 0.30 m) confirmado
  chequeo_pared_confirmaciones_ciclos (5)
    -> GIRAR 90° IZQUIERDA (entra al hueco)
    -> AVANCE_GIRO_VACIO (avanza 0.12 m)
    -> si sigue vacío repite giro+avance (máximo 4 repeticiones)
    -> AVANZAR_PARALELO cuando recupera pared
  lado OCUPADO y venía de obstáculo al frente
    -> GIRAR 90° DERECHA (se aleja de la pared seguida)
    -> AVANZAR_PARALELO
  lado OCUPADO y venía del chequeo periódico
    -> AVANZAR_PARALELO (retoma, mismo rumbo)
```

El giro usa exactamente `velocidad_giro_lineal_mps=0.06`,
`velocidad_giro_angular_radps=0.6`, `angulo_giro_deg=90` y el yaw corregido
por `factor_ang_odom=0.9899`. Tras un hueco, repite giro+avance mientras el
lado seguido continúe vacío, igual que el código de referencia.

## Seguridad

- **Anti-choque frontal (LiDAR):** `umbral_colision_m` (0.15 m): en CUALQUIER
  estado, si el cono **ancho** (`front`) o el **angosto** (`front_narrow`) ven
  algo más cerca que esto, el robot se detiene y ESPERA hasta que se libere;
  nunca avanza contra la pared. El cono angosto atrapa paredes de frente que el
  ancho promedia y no detecta a tiempo.
- **Anti-choque lateral (LiDAR):** si la pared seguida (izquierda) está más
  cerca que `umbral_lateral_min_m` (0.07 m), fuerza el giro máximo alejándose
  para no rozarla (además del ajuste de línea normal).
- **Anti-vuelta-completa:** si el robot acumula **más de 360°** girando (se puso
  a dar vueltas), hace un giro de `correccion_giro_grados` (10°) al lado
  contrario para reorientarse y retoma el avance (estado `CORREGIR_GIRO`).
- Antes de cada giro el robot se detiene y verifica el lado (`PAUSA_CHEQUEO_PARED`),
  así no gira a ciegas.
- Los giros son de 90° fijos por odometría (tope de seguridad
  `angulo_maximo_giro_deg`), no avanzan indefinidamente.

## Colores

El detector aplica HSV, morfología, área, forma y confirmación temporal.

- Cada nueva detección PARE confirmada activa `FRENO_PARE`: el carrito solo
  publica velocidad cero durante 5 segundos. La lógica queda congelada y luego
  continúa exactamente desde el estado interno donde se había quedado.
- El mapa registra un único marcador rojo `P` por evento.
- El **rojo** (PARE) se detecta en **toda la cámara** y en **cualquier posición**
  (no exige que el cartel esté centrado). El **verde** (meta) solo en los **2/3
  superiores** (`franja_verde=0.6667`). El verde no frena: solo marca la META en
  el mapa.
- El verde muestra un rectángulo grande y registra un solo marcador `META`
  durante toda la corrida, exclusivamente si el carrito está en `J1–L2`.
  Fuera de esos seis bloques el verde se ignora. Es exclusivamente visual: no
  frena, no cambia de estado y el carrito continúa con la misma lógica.
- Los componentes pequeños se eliminan antes de dibujarse.

## Fase 2: ruta más corta (speed-run)

Es una segunda fase ADITIVA que **no altera el mapeo**: mientras el carrito
sigue la pared izquierda, `maze_solver` graba de forma pasiva el grafo de
celdas que recorrió (grilla 12×8 cardinal de `motion_ruta.py`, igual que el
mapa web) y marca la celda donde se confirmó el verde como META. Nada de esto
cambia una sola decisión de `/cmd_vel` del mapeo. Con `ruta_activa=false` el
comportamiento es idéntico al mapeo puro.

El carrito **NUNCA frena ni parte solo**. El verde solo marca la META en el mapa
(no frena). La ruta se maneja en **dos comandos**:

- **Calcular** (`/maze/calcular_ruta`): **frena el mapeo y calcula** la ruta más
  corta inicio→meta con **BFS** sobre las aristas que sí recorrió (nunca cruza
  una pared y **elimina callejones sin salida y vueltas redundantes**), la
  publica en `/maze/ruta_corta` para pintarla de **amarillo** y deja el carrito
  **DETENIDO** (`ESPERA_RUTA`). **Solo funciona si ya vio el verde** (si no, no
  frena y sigue mapeando). Es la única forma de frenar sin matar el nodo — no
  uses Ctrl+C, que borraría el mapeo.
- **Partir** (`/maze/iniciar_ruta`): **recién con este comando** el carrito
  arranca y entra a `SEGUIR_RUTA`. Es **repetible**: al terminar la ruta vuelve a
  `ESPERA_RUTA` (detenido, amarillo pintado), así podés colocarlo de nuevo en el
  inicio y reenviar el comando cuantas veces quieras para probar.

Como el punto de partida es siempre el inicio, colocá el carrito ahí mirando
NORTE (`ruta_asume_rumbo_inicial`) antes de dar el comando de partir.

En `SEGUIR_RUTA` maneja la ruta como un guion de movimientos (giros fijos de 90°
cerrados por yaw —un 180° se hace como dos de 90°— + avances rectos por
odometría), **obedeciendo solo la ruta + la anticolisión** (sin seguir pared ni
ajustar línea). Al agotar el guion pasa a `META`.

Parámetros (sincronizados con `navegacion_params.yaml`): `ruta_activa`,
`tamano_celda_m` (0.30, debe coincidir con `CELDA_M` del visualizador),
`ruta_celda_inicio` (A7), `ruta_heading_inicial`, `ruta_asume_rumbo_inicial`
(True: el manejo asume que el carrito se recoloca en el inicio mirando NORTE),
`verde_topic`, `ruta_topic`, `calcular_ruta_topic`, `iniciar_ruta_topic`.

Riesgo a calibrar en pista: la grilla y el manejo son *open-loop* (odometría),
así que dependen de `factor_dist_odom`/`factor_ang_odom` y de `tamano_celda_m`;
la anticolisión evita choques pero no corrige deriva lateral.

## Visualizador

El visualizador muestra:

- estado FSM y descripción del movimiento;
- comandos lineal `v` y angular `w`;
- distancias frontal, laterales, LF y LT;
- puntos LiDAR en marco del robot;
- recorrido calculado desde `/odom_raw`;
- posiciones PARE y META;
- ruta más corta en **amarillo** (`/maze/ruta_corta`, fase 2).

Abrir `web/index.html`, conectar a `http://10.42.0.1:8080/data` y usar
`Ctrl+Shift+R` después de actualizar el frontend.

## Validación

```bash
python3 -m compileall -q capytown_g0_granprix launch
git diff --check
```
