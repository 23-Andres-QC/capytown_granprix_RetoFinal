#!/bin/bash
# Script de inicio rápido para CapyTown Gran Prix
# Uso: ./iniciar.sh

cd ~/yahboomcar_ws
source install/setup.bash

echo "=========================================="
echo "CapyTown Gran Prix - Sistema Completo"
echo "=========================================="
echo ""
echo "Iniciando:"
echo "  1. maze_solver (navegación)"
echo "  2. usb_cam (cámara USB)"
echo "  3. visualizador_web (emisor JSON para web/index.html)"
echo ""

# Lanzar los 3 nodos en paralelo
ros2 run capytown_g0_granprix maze_solver &
PID_MAZE=$!

sleep 1

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p pixel_format:=mjpeg2rgb &
PID_CAMARA=$!

sleep 1

ros2 run capytown_g0_granprix visualizador_web &
PID_VIZ=$!

echo ""
echo "✅ Sistema iniciado (PID: maze=$PID_MAZE cam=$PID_CAMARA viz=$PID_VIZ)"
echo ""
echo "Para detener: Ctrl+C"
echo ""

# Esperar a que se termine alguno (si lo hacen)
wait
