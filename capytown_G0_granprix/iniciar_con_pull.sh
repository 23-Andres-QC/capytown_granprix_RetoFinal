#!/bin/bash
# Iniciar CapyTown con git pull force + compilar + ejecutar

set -e  # Exit si hay error

echo "=========================================="
echo "CapyTown Gran Prix - INICIAR CON GIT PULL"
echo "=========================================="
echo ""

# 1. CD al workspace
cd ~/yahboomcar_ws
echo "[1/5] Directorio: $(pwd)"
echo ""

# 2. GIT PULL FORCE
echo "[2/5] GIT PULL FORCE..."
cd src/capytown_granprix_RetoFinal
git fetch origin main
git reset --hard origin/main
echo "✓ Git sincronizado con origin/main"
cd ~/yahboomcar_ws
echo ""

# 3. COMPILAR
echo "[3/5] COMPILANDO..."
colcon build --packages-select capytown_g0_granprix 2>&1 | tail -5
echo "✓ Compilación completa"
echo ""

# 4. SOURCE
echo "[4/5] SOURCING setup.bash..."
source install/setup.bash
echo "✓ Ambiente listo"
echo ""

# 5. EJECUTAR
echo "[5/5] LANZANDO SISTEMA COMPLETO..."
echo ""
echo "=========================================="
echo "maze_solver + usb_cam + visualizador_web"
echo "=========================================="
echo ""

ros2 launch capytown_g0_granprix movimiento.launch.py &
PID_MOV=$!

sleep 2

ros2 launch capytown_g0_granprix visualizacion.launch.py &
PID_VIZ=$!

echo ""
echo "✅ Sistema corriendo"
echo "   maze_solver (PID: $PID_MOV)"
echo "   visualizador_web (PID: $PID_VIZ)"
echo ""
echo "Para detener: Ctrl+C"
echo ""

# Esperar
wait
