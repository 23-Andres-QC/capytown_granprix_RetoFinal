# Guía del repositorio

La documentación vigente está en `DOCUMENTACION.md` y los comandos en
`docs/COMANDOS.md`.

Reglas del sistema:

- `capytown_g0_granprix/maze_solver.py` es la única lógica de recorrido y el
  único publicador autorizado de `/cmd_vel`.
- La pared seguida es la izquierda.
- Todos los giros son FIJOS de 90° cerrados por odometría (yaw), no reactivos:
  no se pasan ni se encadenan. Antes de cada giro el robot se detiene y
  verifica el lado (`CHEQUEO_PARED`).
- La parada de colisión (`umbral_colision_m`) es la única que frena en seco:
  se detiene y ESPERA a que se libere; nunca avanza contra la pared.
- `visualizador_web.py` no controla el robot: solo observa y publica datos HTTP.
- `pare_detector.py` no mueve el robot: publica `/pare_detectado` y
  `/verde_detectado`.
- Deben conservarse los tres comandos descritos en `docs/COMANDOS.md`.
- Los defaults de `maze_solver.py` y `config/navegacion_params.yaml` deben
  permanecer sincronizados.

Antes de entregar:

```bash
python3 -m compileall -q capytown_g0_granprix launch
git diff --check
```
