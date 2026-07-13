# Comandos

## Compilar

```bash
cd ~/yahboomcar_ws
colcon build --packages-select capytown_g0_granprix
source install/setup.bash
```

## Ejecutar (tres terminales)

```bash
cd ~/yahboomcar_ws && source install/setup.bash && ros2 run capytown_g0_granprix maze_solver
```

```bash
cd ~/yahboomcar_ws && source install/setup.bash && ros2 launch capytown_g0_granprix solo_camara.launch.py
```

```bash
cd ~/yahboomcar_ws && source install/setup.bash && ros2 launch capytown_g0_granprix visualizacion.launch.py
```

## Ruta más corta (fase 2) — dos comandos

El carrito NUNCA parte solo. Cuando ya vio la meta (verde):

```bash
# 1) CALCULAR: pinta la ruta de amarillo y deja el carrito DETENIDO
cd ~/yahboomcar_ws && source install/setup.bash && \
  ros2 topic pub --once /maze/calcular_ruta std_msgs/msg/Bool "{data: true}"

# (colocá el carrito en el inicio mirando NORTE)

# 2) PARTIR: recién aquí arranca y maneja la ruta (solo ruta + anticolisión)
ros2 topic pub --once /maze/iniciar_ruta std_msgs/msg/Bool "{data: true}"
```

`calcular_ruta` **solo funciona si ya se marcó la meta** (se vio el verde); si
no, no frena y sigue mapeando.

El comando 2 (`iniciar_ruta`) es **repetible**: al terminar la ruta el carrito
vuelve a estar detenido con el amarillo pintado, así podés recolocarlo en el
inicio (mirando NORTE) y reenviarlo las veces que quieras para probar.

## Detener

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
