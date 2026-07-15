#!/usr/bin/env python3
"""Módulo maze_solver."""

import json
import math
from types import SimpleNamespace

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from capytown_g0_granprix import motion_events as EV
from capytown_g0_granprix.motion_geometry import angle_diff, normalize_angle, yaw_from_quaternion
from capytown_g0_granprix.motion_grid import GridTracker
from capytown_g0_granprix.motion_ruta import RouteExplorer
from capytown_g0_granprix.motion_lidar import (
    ZoneWindow, compute_robot_frame_angles, compute_zone_distance, fit_wall_line)


class MazeSolverNode(Node):

    def __init__(self):
        """Inicializa el componente."""
        super().__init__('maze_solver')
        self._declare_parameters()
        self._read_parameters()

        self._grid = GridTracker.from_cell_name(self._celda_inicio, self._heading_inicial)


        self._state = 'INICIAR'
        self._terminado = False


        self._zones = None
        self._zones_ready = False
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0
        self._odom_ready = False
        self._pare_activo = False
        self._pare_anterior = False
        self._pare_pendiente = False
        self._pare_ignorar_xy = None
        self._freno_pare_start = None


        self._cell_start_xy = (0.0, 0.0)
        self._avance_chequeo_start_xy = (0.0, 0.0)
        self._num_celdas = 0
        self._cruce_muestras = None
        self._derecha_libre = False
        self._frente_libre = False
        self._izquierda_libre = False
        self._buscar_pare_start = None
        self._pare_hold_start = None
        self._celdas_pare_respetadas = set()
        self._decision_actual = 'NINGUNO'
        self._giro_objetivo = 0.0
        self._alinear_start = None
        self._pausa_giro_start = None

        self._esperando_obstaculo = False
        self._espera_obstaculo_inicio = None
        self._contador_frente_colision = 0
        self._retrocediendo_obstaculo = False
        self._retroceso_obstaculo_xy0 = (0.0, 0.0)
        self._avanzando_post_retroceso = False
        self._avance_post_retroceso_xy0 = (0.0, 0.0)
        self._contador_frente_dos_reglas = 0
        self._contador_lado_libre_temprano = 0
        self._yaw_inicio_giro = 0.0
        self._imu_acum_giro = 0.0
        self._imu_t_prev = None
        self._pausa_chequeo_start = None
        self._contador_derecha_libre = 0
        self._chequeo_por_frente = False
        self._giro_vacio_fase = 0
        self._giro_vacio_repeticiones = 0
        self._avance_fijo_inicio_xy = (0.0, 0.0)


        self._verde_anterior = False
        self._meta_detectada = False
        self._ruta_movimientos = None
        self._ruta_idx = 0
        self._ruta_fase = 'GIRO'
        self._ruta_giro_yaw0 = 0.0
        self._ruta_giro_restante = None
        self._ruta_avance_xy0 = (0.0, 0.0)
        self._ruta_json = None
        self._ruta_pub_counter = 0


        self._yaw_prev_360 = None
        self._rot_acum = 0.0
        self._correccion_signo = 1.0
        self._yaw_correccion0 = 0.0


        self._tiempo_inicio = None
        self._distancia_total_m = 0.0
        self._odom_prev_xy = None
        self._contador_colisiones = 0
        self._contador_pare_detectados = 0
        self._contador_pare_respetados = 0
        self._metricas_meta = None


        self._contador_giros_fisicos = 0

        self._STATE_HANDLERS = {
            'INICIAR': self._handle_iniciar,
            'AVANZAR_PARALELO': self._handle_avanzar_paralelo,
            'DETECTAR_CRUCE': self._handle_detectar_cruce,
            'BUSCAR_PARE': self._handle_buscar_pare,
            'DECIDIR': self._handle_decidir,
            'PAUSA_GIRO': self._handle_pausa_giro,
            'PAUSA_CHEQUEO_PARED': self._handle_pausa_chequeo_pared,
            'GIRAR': self._handle_girar,
            'AVANCE_GIRO_VACIO': self._handle_avance_giro_vacio,
            'ALINEAR': self._handle_alinear,
            'VERIFICAR_META': self._handle_verificar_meta,
            'ESPERA_RUTA': self._handle_espera_ruta,
            'SEGUIR_RUTA': self._handle_seguir_ruta,
            'CORREGIR_GIRO': self._handle_corregir_giro,
            'META': self._handle_meta,
            'DETENIDO': self._handle_detenido,
        }

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._event_pub = self.create_publisher(String, self._event_topic, 10)
        self._state_pub = self.create_publisher(String, self._robot_state_topic, 10)
        self._metrics_pub = self.create_publisher(String, '/maze/metricas', 10)
        self._ruta_pub = self.create_publisher(String, self._ruta_topic, 10)

        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(
            Imu, self._imu_topic, self._on_imu, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Bool, self._pare_topic, self._on_pare, 10)


        self.create_subscription(Bool, self._verde_topic, self._on_verde, 10)

        self.create_subscription(
            Bool, self._calcular_ruta_topic, self._on_calcular_ruta, 10)
        self.create_subscription(
            Bool, self._iniciar_ruta_topic, self._on_iniciar_ruta, 10)

        self.create_timer(1.0 / self._control_rate_hz, self._on_timer)

        self.get_logger().info(
            f'maze_solver referencia listo: inicio={self._celda_inicio} meta={self._celda_meta} '
            f'heading_inicial={self._heading_inicial}'
        )


    def _declare_parameters(self):
        """Ejecuta declare parameters."""
        defaults = {
            'scan_topic': '/scan',
            'front_offset_deg': 180.0,
            'invert_left_right': False,
            'max_range_use_m': 4.0,
            'front_window_deg': [-15.0, 15.0],
            'front_narrow_window_deg': [-5.0, 5.0],
            'right_front_window_deg': [-75.0, -45.0],
            'right_window_deg': [-110.0, -70.0],
            'right_rear_window_deg': [-135.0, -105.0],
            'left_window_deg': [70.0, 110.0],


            'left_front_window_deg': [65.0, 85.0],
            'right_side_window_deg': [-110.0, -70.0],
            'left_side_window_deg': [70.0, 110.0],
            'min_puntos_linea': 6,
            'right_wall_max_range_m': 0.50,
            'left_wall_max_range_m': 0.50,
            'outlier_max_iter': 3,
            'outlier_residuo_m': 0.03,
            'odom_topic': '/odom_raw',


            'imu_topic': '/imu',
            'umbral_patinaje_deg': 8.0,
            'cmd_vel_topic': '/cmd_vel',
            'pare_topic': '/pare_detectado',
            'event_topic': '/robot_event',
            'robot_state_topic': '/maze/estado',
            'usar_camara': True,
            'control_rate_hz': 20.0,


            'modo_simplificado': True,


            'logica_dos_reglas': True,


            'seguir_pared_izquierda': True,
            'velocidad_recta_mps': 0.15,


            'distancia_objetivo_m': 0.12,


            'ganancia_angulo_recta': 2.0,
            'ganancia_distancia_recta': 2.0,
            'angular_max_recta_radps': 0.6,


            'frente_confirmaciones_ciclos': 3,


            'distancia_chequeo_pared_m': 0.12,
            'tiempo_chequeo_pared_s': 0.5,


            'chequeo_pared_confirmaciones_ciclos': 5,


            'avance_giro_vacio_m': 0.10,
            'giro_vacio_max_repeticiones': 2,


            'angulo_maximo_giro_deg': 150.0,
            'umbral_frente_pared_m': 0.30,
            'umbral_frente_libre_m': 0.35,
            'umbral_lado_libre_m': 0.30,


            'umbral_colision_m': 0.15,


            'retroceso_obstaculo_m': 0.10,
            'velocidad_retroceso_obstaculo_mps': 0.06,
            'velocidad_retroceso_obstaculo_angular_radps': 0.9,


            'avance_post_retroceso_m': 0.10,


            'umbral_lateral_min_m': 0.07,


            'correccion_giro_360': True,
            'correccion_giro_grados': 10.0,
            'tiempo_espera_obstaculo_s': 2.0,
            'distancia_celda_m': 5.00,
            'margen_avance_m': 0.05,
            'muestras_confirmacion': 5,
            'consenso_minimo': 4,
            'velocidad_giro_lineal_mps': 0.06,
            'velocidad_giro_angular_radps': 0.6,
            'tolerancia_giro_deg': 4.0,


            'angulo_giro_deg': 90.0,


            'tiempo_pausa_antes_girar_s': 1.0,
            'tolerancia_alineacion_m': 0.02,
            'tiempo_max_alinear_s': 4.0,
            'velocidad_alineacion_lineal_mps': 0.06,
            'velocidad_alineacion_angular_radps': 0.3,


            'tiempo_pare_s': 5.0,


            'distancia_ignorar_pare_m': 0.60,
            'tiempo_espera_camara_s': 0.5,
            'celda_inicio': 'A4',
            'celda_meta': 'F1',
            'heading_inicial': 'NORTE',
            'max_celdas_recorridas': 60,


            'factor_dist_odom': 0.9474,
            'factor_ang_odom': 0.9899,


            'ruta_activa': True,
            'verde_topic': '/verde_detectado',
            'ruta_topic': '/maze/ruta_corta',


            'calcular_ruta_topic': '/maze/calcular_ruta',
            'iniciar_ruta_topic': '/maze/iniciar_ruta',


            'tamano_celda_m': 0.30,


            'ruta_celda_inicio': 'A7',
            'ruta_heading_inicial': 'NORTE',


            'ruta_fija_giros': ['NINGUNO', 'DERECHA', 'IZQUIERDA', 'DERECHA'],
            'ruta_fija_distancias_m': [1.02, 1.02, 0.55, 1.85],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        """Ejecuta read parameters."""
        g = lambda name: self.get_parameter(name).value

        self._scan_topic = g('scan_topic')
        self._front_offset_rad = math.radians(float(g('front_offset_deg')))
        self._lidar_sign = -1 if bool(g('invert_left_right')) else 1
        self._max_range_use = float(g('max_range_use_m'))
        self._lidar_windows = {
            'front': ZoneWindow(*g('front_window_deg')),
            'front_narrow': ZoneWindow(*g('front_narrow_window_deg')),
            'right_front': ZoneWindow(*g('right_front_window_deg')),
            'right': ZoneWindow(*g('right_window_deg')),
            'right_rear': ZoneWindow(*g('right_rear_window_deg')),
            'left': ZoneWindow(*g('left_window_deg')),
            'left_front': ZoneWindow(*g('left_front_window_deg')),
        }
        self._right_side_window = ZoneWindow(*g('right_side_window_deg'))
        self._left_side_window = ZoneWindow(*g('left_side_window_deg'))
        self._min_puntos_linea = int(g('min_puntos_linea'))
        self._right_wall_max_range = float(g('right_wall_max_range_m'))
        self._left_wall_max_range = float(g('left_wall_max_range_m'))
        self._outlier_max_iter = int(g('outlier_max_iter'))
        self._outlier_residuo = float(g('outlier_residuo_m'))

        self._odom_topic = g('odom_topic')
        self._imu_topic = g('imu_topic')
        self._umbral_patinaje = float(g('umbral_patinaje_deg'))
        self._cmd_vel_topic = g('cmd_vel_topic')
        self._pare_topic = g('pare_topic')
        self._event_topic = g('event_topic')
        self._robot_state_topic = g('robot_state_topic')

        self._usar_camara = bool(g('usar_camara'))
        self._control_rate_hz = float(g('control_rate_hz'))
        self._modo_simplificado = bool(g('modo_simplificado'))
        self._logica_dos_reglas = bool(g('logica_dos_reglas'))
        self._seguir_izquierda = bool(g('seguir_pared_izquierda'))
        self._velocidad_recta = float(g('velocidad_recta_mps'))
        self._distancia_objetivo_recta = float(g('distancia_objetivo_m'))
        self._ganancia_angulo_recta = float(g('ganancia_angulo_recta'))
        self._ganancia_distancia_recta = float(g('ganancia_distancia_recta'))
        self._angular_max_recta = float(g('angular_max_recta_radps'))
        self._frente_confirmaciones_ciclos = int(g('frente_confirmaciones_ciclos'))
        self._distancia_chequeo_pared = float(g('distancia_chequeo_pared_m'))
        self._chequeo_pared_confirmaciones_ciclos = int(g('chequeo_pared_confirmaciones_ciclos'))
        self._avance_giro_vacio = float(g('avance_giro_vacio_m'))
        self._giro_vacio_max_repeticiones = int(g('giro_vacio_max_repeticiones'))
        self._tiempo_chequeo_pared = float(g('tiempo_chequeo_pared_s'))
        self._contador_frente_dos_reglas = 0
        self._angulo_maximo_giro_rad = math.radians(float(g('angulo_maximo_giro_deg')))

        self._umbral_frente_pared = float(g('umbral_frente_pared_m'))
        self._umbral_frente_libre = float(g('umbral_frente_libre_m'))
        self._umbral_lado_libre = float(g('umbral_lado_libre_m'))
        self._umbral_colision = float(g('umbral_colision_m'))
        self._retroceso_obstaculo = float(g('retroceso_obstaculo_m'))
        self._v_retroceso_obstaculo = float(g('velocidad_retroceso_obstaculo_mps'))
        self._w_retroceso_obstaculo = float(g('velocidad_retroceso_obstaculo_angular_radps'))
        self._avance_post_retroceso = float(g('avance_post_retroceso_m'))
        self._umbral_lateral_min = float(g('umbral_lateral_min_m'))
        self._correccion_giro_360 = bool(g('correccion_giro_360'))
        self._correccion_giro_rad = math.radians(float(g('correccion_giro_grados')))
        self._distancia_celda = float(g('distancia_celda_m'))
        self._margen_avance = float(g('margen_avance_m'))

        self._muestras_confirmacion = int(g('muestras_confirmacion'))
        self._consenso_minimo = int(g('consenso_minimo'))

        self._v_giro_lineal = float(g('velocidad_giro_lineal_mps'))
        self._v_giro_angular = float(g('velocidad_giro_angular_radps'))
        self._tolerancia_giro_rad = math.radians(float(g('tolerancia_giro_deg')))
        self._angulo_giro_rad = math.radians(float(g('angulo_giro_deg')))
        self._tiempo_pausa_antes_girar = float(g('tiempo_pausa_antes_girar_s'))

        self._tolerancia_alineacion = float(g('tolerancia_alineacion_m'))
        self._tiempo_max_alinear = float(g('tiempo_max_alinear_s'))
        self._v_alinear_lineal = float(g('velocidad_alineacion_lineal_mps'))
        self._v_alinear_angular = float(g('velocidad_alineacion_angular_radps'))

        self._tiempo_pare = float(g('tiempo_pare_s'))
        self._distancia_ignorar_pare = float(g('distancia_ignorar_pare_m'))
        self._tiempo_espera_camara = float(g('tiempo_espera_camara_s'))

        self._tiempo_espera_obstaculo = float(g('tiempo_espera_obstaculo_s'))

        self._celda_inicio = str(g('celda_inicio'))
        self._celda_meta = str(g('celda_meta'))
        self._heading_inicial = str(g('heading_inicial'))
        self._max_celdas = int(g('max_celdas_recorridas'))

        self._factor_dist_odom = float(g('factor_dist_odom'))
        self._factor_ang_odom = float(g('factor_ang_odom'))

        self._ruta_activa = bool(g('ruta_activa'))
        self._verde_topic = g('verde_topic')
        self._ruta_topic = g('ruta_topic')
        self._calcular_ruta_topic = g('calcular_ruta_topic')
        self._iniciar_ruta_topic = g('iniciar_ruta_topic')
        self._tamano_celda = float(g('tamano_celda_m'))
        self._ruta_celda_inicio = str(g('ruta_celda_inicio'))
        self._ruta_heading_inicial = str(g('ruta_heading_inicial'))
        self._ruta_fija_giros = list(g('ruta_fija_giros'))
        self._ruta_fija_distancias = [float(d) for d in g('ruta_fija_distancias_m')]


    def _on_scan(self, msg: LaserScan):
        """Procesa on scan."""
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = compute_robot_frame_angles(
            ranges, msg.angle_min, msg.angle_increment,
            self._front_offset_rad, self._lidar_sign)
        max_use = min(float(msg.range_max), self._max_range_use)
        z = SimpleNamespace()
        for name in ('front', 'front_narrow', 'right_front', 'right', 'right_rear',
                     'left', 'left_front'):
            distance, valid = compute_zone_distance(
                ranges, angles, msg.range_min, max_use, self._lidar_windows[name])
            setattr(z, name, distance)
            setattr(z, f'{name}_valid', valid)
        for side, window, max_wall in (
                ('right', self._right_side_window, self._right_wall_max_range),
                ('left', self._left_side_window, self._left_wall_max_range)):
            angle, distance, valid = fit_wall_line(
                ranges, angles, msg.range_min, min(max_use, max_wall), window,
                self._min_puntos_linea, self._outlier_max_iter, self._outlier_residuo)
            setattr(z, f'{side}_line_angle_rad', angle)
            setattr(z, f'{side}_line_distance_m', distance)
            setattr(z, f'{side}_line_valid', valid)
        self._zones = z
        self._zones_ready = True

    def _on_odom(self, msg: Odometry):


        """Procesa on odom."""
        self._odom_x = msg.pose.pose.position.x * self._factor_dist_odom
        self._odom_y = msg.pose.pose.position.y * self._factor_dist_odom
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation) * self._factor_ang_odom
        self._odom_ready = True


        if self._odom_prev_xy is not None:
            dx = self._odom_x - self._odom_prev_xy[0]
            dy = self._odom_y - self._odom_prev_xy[1]
            self._distancia_total_m += math.hypot(dx, dy)
        self._odom_prev_xy = (self._odom_x, self._odom_y)

    def _on_imu(self, msg: Imu):
        """Procesa on imu."""
        ahora = self.get_clock().now()
        if self._state == 'GIRAR' and self._imu_t_prev is not None:
            dt = (ahora - self._imu_t_prev).nanoseconds / 1e9
            self._imu_acum_giro += msg.angular_velocity.z * dt
        self._imu_t_prev = ahora

    def _on_pare(self, msg: Bool):
        """Procesa on pare."""
        detectado = bool(msg.data)
        en_cooldown = False
        if self._pare_ignorar_xy is not None:
            dx = self._odom_x - self._pare_ignorar_xy[0]
            dy = self._odom_y - self._pare_ignorar_xy[1]
            en_cooldown = math.hypot(dx, dy) < self._distancia_ignorar_pare


        if detectado and not self._pare_anterior:
            if en_cooldown:


                dx = self._odom_x - self._pare_ignorar_xy[0]
                dy = self._odom_y - self._pare_ignorar_xy[1]
                self._publish_event(
                    EV.PARE_FALSO,
                    f'rojo ignorado (cooldown): avanzo {math.hypot(dx, dy):.2f}m '
                    f'de {self._distancia_ignorar_pare:.2f}m requeridos'
                )
            else:
                self._pare_pendiente = True
                self._contador_pare_detectados += 1
        self._pare_activo = detectado
        self._pare_anterior = detectado

    def _on_verde(self, msg: Bool):
        """Procesa on verde."""
        activo = bool(msg.data)
        if activo and not self._verde_anterior and not self._meta_detectada:
            self._meta_detectada = True


            self._metricas_meta = self._metricas_actuales()
            self._publish_event(
                EV.META, 'meta (verde) registrada'
            )
        self._verde_anterior = activo

    def _on_calcular_ruta(self, msg: Bool):
        """Procesa on calcular ruta."""
        if not bool(msg.data):
            return
        if not self._ruta_activa:
            self._publish_event(EV.TIMEOUT, 'calcular_ruta ignorado: ruta_activa=false')
            return
        if self._terminado or self._state in ('ESPERA_RUTA', 'SEGUIR_RUTA', 'META'):
            return
        if not self._calcular_ruta():
            return
        self._publish_twist(Twist())
        self._publish_event(
            EV.META,
            'DETENIDO + ruta calculada (amarillo). '
            'Envia /maze/iniciar_ruta para PARTIR.'
        )
        self._set_state('ESPERA_RUTA')

    def _on_iniciar_ruta(self, msg: Bool):
        """Procesa on iniciar ruta."""
        if not bool(msg.data):
            return
        if not self._ruta_activa:
            self._publish_event(EV.TIMEOUT, 'iniciar_ruta ignorado: ruta_activa=false')
            return
        if self._terminado or self._state == 'SEGUIR_RUTA':
            return
        if self._ruta_movimientos is None and not self._calcular_ruta():
            self._publish_event(
                EV.TIMEOUT,
                'iniciar_ruta ignorado: no hay ruta calculada '
                '(usa /maze/calcular_ruta)'
            )
            return
        self._ruta_idx = 0
        self._ruta_fase = 'GIRO'
        self._ruta_giro_restante = None
        self._publish_event(EV.INICIO, 'PARTIR: manejando la ruta corta')
        self._set_state('SEGUIR_RUTA')


    def _on_timer(self):
        """Procesa on timer."""
        if not (self._odom_ready and self._zones_ready):
            return


        if self._freno_pare_start is not None:
            elapsed = (self.get_clock().now() - self._freno_pare_start).nanoseconds / 1e9
            if elapsed < self._tiempo_pare:
                self._state_pub.publish(String(data='FRENO_PARE'))
                self._publish_twist(Twist())
                return

            self._congelar_relojes_durante(elapsed)
            self._freno_pare_start = None


            self._pare_pendiente = False
            self._pare_ignorar_xy = (self._odom_x, self._odom_y)
            self._celdas_pare_respetadas.add(self._grid.cell)
            self._contador_pare_respetados += 1
            self._publish_event(
                EV.PARE_RESPETADO,
                f'PARE respetado {elapsed:.1f}s; continúa en {self._state}',
            )
            self._state_pub.publish(String(data=self._state))
        elif self._pare_pendiente and self._state not in ('META', 'DETENIDO'):
            self._pare_pendiente = False
            self._freno_pare_start = self.get_clock().now()
            self._publish_event(
                EV.PARE_DETECTADO,
                f'rojo detectado: FRENO_PARE {self._tiempo_pare:.1f}s '
                f'sin abandonar {self._state}',
            )
            self._state_pub.publish(String(data='FRENO_PARE'))
            self._publish_twist(Twist())
            return
        elif self._pare_pendiente:

            self._pare_pendiente = False

        if self._handle_obstaculo_frente():
            return

        self._STATE_HANDLERS[self._state]()


        self._monitor_rotacion()

    def _congelar_relojes_durante(self, segundos: float):
        """Ejecuta congelar relojes durante."""
        pausa = Duration(nanoseconds=int(segundos * 1e9))
        for nombre in (
                '_buscar_pare_start', '_pare_hold_start', '_alinear_start',
                '_pausa_giro_start', '_espera_obstaculo_inicio',
                '_pausa_chequeo_start'):
            instante = getattr(self, nombre, None)
            if instante is not None:
                setattr(self, nombre, instante + pausa)

    def _handle_obstaculo_frente(self) -> bool:
        """Gestiona obstaculo frente."""
        if self._terminado:
            return False

        z = self._zones


        frente_bloqueado = (
            (z.front_valid and z.front < self._umbral_colision) or
            (z.front_narrow_valid and z.front_narrow < self._umbral_colision))

        if self._retrocediendo_obstaculo:
            dx = self._odom_x - self._retroceso_obstaculo_xy0[0]
            dy = self._odom_y - self._retroceso_obstaculo_xy0[1]
            if math.hypot(dx, dy) >= self._retroceso_obstaculo:
                self._publish_twist(Twist())
                self._retrocediendo_obstaculo = False
                self._avanzando_post_retroceso = True
                self._avance_post_retroceso_xy0 = (self._odom_x, self._odom_y)
                self._publish_event(
                    EV.COLISION,
                    f'retroceso de {self._retroceso_obstaculo * 100:.0f}cm completado'
                )
                return True
            cmd = Twist()
            cmd.linear.x = -self._v_retroceso_obstaculo


            cmd.angular.z = -self._w_retroceso_obstaculo
            self._publish_twist(cmd)
            return True

        if self._avanzando_post_retroceso:
            dx = self._odom_x - self._avance_post_retroceso_xy0[0]
            dy = self._odom_y - self._avance_post_retroceso_xy0[1]
            if math.hypot(dx, dy) >= self._avance_post_retroceso:
                self._publish_twist(Twist())
                self._avanzando_post_retroceso = False
                self._esperando_obstaculo = True
                self._espera_obstaculo_inicio = self.get_clock().now()
                self._publish_event(
                    EV.COLISION,
                    f'avance de {self._avance_post_retroceso * 100:.0f}cm '
                    f'post-retroceso completado; analizando entorno'
                )
                return True
            cmd = Twist()
            cmd.linear.x = self._velocidad_recta
            self._publish_twist(cmd)
            return True

        if self._esperando_obstaculo:
            if frente_bloqueado:
                self._publish_twist(Twist())
                elapsed = (
                    self.get_clock().now() - self._espera_obstaculo_inicio
                ).nanoseconds / 1e9
                if elapsed >= self._tiempo_espera_obstaculo:


                    self._espera_obstaculo_inicio = self.get_clock().now()
                return True
            self._esperando_obstaculo = False
            return False

        self._contador_frente_colision = (
            self._contador_frente_colision + 1 if frente_bloqueado else 0)
        if self._contador_frente_colision >= self._frente_confirmaciones_ciclos:
            self._contador_frente_colision = 0
            d_frente = z.front if z.front_valid else z.front_narrow
            self._contador_colisiones += 1
            self._publish_event(
                EV.COLISION,
                f'obstaculo a {d_frente:.2f} m cerca de {self._grid.cell}; '
                f'retrocediendo {self._retroceso_obstaculo * 100:.0f}cm'
            )
            self._retrocediendo_obstaculo = True
            self._retroceso_obstaculo_xy0 = (self._odom_x, self._odom_y)
            return True
        return False

    def _monitor_rotacion(self):
        """Ejecuta monitor rotacion."""
        if not self._correccion_giro_360 or self._terminado:
            return
        if self._state in ('ESPERA_RUTA', 'SEGUIR_RUTA', 'META', 'DETENIDO'):
            self._yaw_prev_360 = self._yaw
            self._rot_acum = 0.0
            return
        if self._yaw_prev_360 is None:
            self._yaw_prev_360 = self._yaw
            return

        if self._state == 'AVANZAR_PARALELO':
            self._yaw_prev_360 = self._yaw
            self._rot_acum = 0.0
            return
        self._rot_acum += angle_diff(self._yaw, self._yaw_prev_360)
        self._yaw_prev_360 = self._yaw
        if self._state != 'CORREGIR_GIRO' and abs(self._rot_acum) > 2.0 * math.pi:
            self._correccion_signo = -1.0 if self._rot_acum > 0.0 else 1.0
            self._yaw_correccion0 = self._yaw
            self._rot_acum = 0.0
            self._publish_event(
                EV.GIRO,
                f'giro >360 deg detectado -> corrige '
                f'{math.degrees(self._correccion_giro_rad):.0f} deg al contrario'
            )
            self._set_state('CORREGIR_GIRO')

    def _handle_corregir_giro(self):
        """Gestiona corregir giro."""
        girado = abs(angle_diff(self._yaw, self._yaw_correccion0))
        if girado >= self._correccion_giro_rad:
            self._publish_twist(Twist())
            self._begin_avanzar_paralelo()
            self._set_state('AVANZAR_PARALELO')
            return
        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        cmd.angular.z = self._correccion_signo * self._v_giro_angular
        self._publish_twist(cmd)

        return False


    def _line_valid(self, z) -> bool:
        """Ejecuta line valid."""
        return bool(z.left_line_valid if self._seguir_izquierda else z.right_line_valid)

    def _line_angle(self, z) -> float:
        """Ejecuta line angle."""
        return z.left_line_angle_rad if self._seguir_izquierda else z.right_line_angle_rad

    def _line_distance(self, z) -> float:
        """Ejecuta line distance."""
        return z.left_line_distance_m if self._seguir_izquierda else z.right_line_distance_m

    def _lado_valid(self, z) -> bool:
        """Ejecuta lado valid."""
        return bool(z.left_valid if self._seguir_izquierda else z.right_valid)

    def _lado_distancia(self, z) -> float:
        """Ejecuta lado distancia."""
        return z.left if self._seguir_izquierda else z.right

    def _lado_frente_valid(self, z) -> bool:
        """Ejecuta lado frente valid."""
        return bool(z.left_front_valid if self._seguir_izquierda else z.right_front_valid)

    def _lado_frente_distancia(self, z) -> float:
        """Ejecuta lado frente distancia."""
        return z.left_front if self._seguir_izquierda else z.right_front

    def _twist_wall_follow(self) -> Twist:
        """Ejecuta twist wall follow."""
        z = self._zones
        cmd = Twist()
        if not self._line_valid(z):


            cmd.linear.x = self._velocidad_recta
            return cmd


        signo_distancia = -1.0 if self._seguir_izquierda else 1.0
        error_distancia = self._distancia_objetivo_recta - self._line_distance(z)
        correccion = (self._ganancia_angulo_recta * self._line_angle(z)
                      + signo_distancia * self._ganancia_distancia_recta * error_distancia)


        if self._lado_valid(z) and self._lado_distancia(z) < self._umbral_lateral_min:
            correccion = signo_distancia * self._angular_max_recta
        cmd.linear.x = self._velocidad_recta
        cmd.angular.z = max(-self._angular_max_recta, min(self._angular_max_recta, correccion))
        return cmd

    def _direccion_obstaculo(self) -> str:
        """Ejecuta direccion obstaculo."""
        return 'DERECHA' if self._seguir_izquierda else 'IZQUIERDA'

    def _direccion_vacio(self) -> str:
        """Ejecuta direccion vacio."""
        return 'IZQUIERDA' if self._seguir_izquierda else 'DERECHA'


    def _handle_iniciar(self):
        """Gestiona iniciar."""
        self._publish_event(
            EV.INICIO, f'inicio en {self._grid.cell}, heading {self._grid.heading}'
        )
        self._tiempo_inicio = self.get_clock().now()
        self._begin_avanzar_paralelo()
        self._set_state('AVANZAR_PARALELO')

    def _begin_avanzar_paralelo(self):
        """Ejecuta begin avanzar paralelo."""
        self._cell_start_xy = (self._odom_x, self._odom_y)
        self._avance_chequeo_start_xy = (self._odom_x, self._odom_y)

    def _handle_avanzar_paralelo(self):
        """Gestiona avanzar paralelo."""
        if self._logica_dos_reglas:
            self._handle_avanzar_paralelo_dos_reglas()
            return

        dx = self._odom_x - self._cell_start_xy[0]
        dy = self._odom_y - self._cell_start_xy[1]
        avance = math.hypot(dx, dy)

        z = self._zones
        frente_cerca = z.front_valid and z.front < self._umbral_frente_pared

        if avance >= (self._distancia_celda - self._margen_avance) or frente_cerca:
            self._publish_twist(Twist())
            self._num_celdas += 1
            self._grid.advance_cell()
            self._publish_event(
                EV.CELDA_AVANZADA, f'celda {self._grid.cell} (#{self._num_celdas})'
            )

            if self._num_celdas > self._max_celdas:
                self._publish_event(
                    EV.TIMEOUT, 'limite de celdas recorridas alcanzado sin llegar a la meta'
                )
                self._terminado = True
                self._set_state('DETENIDO')
                return

            if self._modo_simplificado:


                self._derecha_libre = bool(z.right_valid and z.right > self._umbral_lado_libre)
                self._frente_libre = bool(z.front_valid and z.front > self._umbral_frente_libre)
                self._izquierda_libre = bool(z.left_valid and z.left > self._umbral_lado_libre)
                self._set_state('DECIDIR')
            else:
                self._set_state('DETECTAR_CRUCE')
            return

        self._publish_twist(self._twist_wall_follow())

    def _handle_avanzar_paralelo_dos_reglas(self):
        """Gestiona avanzar paralelo dos reglas."""
        z = self._zones

        lado_libre_temprano = bool(
            self._lado_frente_valid(z)
            and self._lado_frente_distancia(z) > self._umbral_lado_libre)
        self._contador_lado_libre_temprano = (
            self._contador_lado_libre_temprano + 1 if lado_libre_temprano else 0)
        if self._contador_lado_libre_temprano >= self._chequeo_pared_confirmaciones_ciclos:
            self._contador_lado_libre_temprano = 0
            self._publish_twist(Twist())
            self._decision_actual = self._direccion_vacio()
            self._yaw_inicio_giro = self._yaw
            self._giro_vacio_fase = 1
            self._giro_vacio_repeticiones = 0
            self._contador_giros_fisicos += 1
            self._publish_event(
                EV.GIRO,
                f'lado seguido vacio en movimiento '
                f'({self._lado_frente_distancia(z):.2f}m diagonal) '
                f'-> {self._decision_actual}'
            )
            self._set_state('GIRAR')
            return

        dx = self._odom_x - self._avance_chequeo_start_xy[0]
        dy = self._odom_y - self._avance_chequeo_start_xy[1]
        avance_chequeo = math.hypot(dx, dy)

        if avance_chequeo >= self._distancia_chequeo_pared:
            self._publish_event(
                EV.GIRO, f'avanzo {avance_chequeo:.2f}m -> detenido a verificar pared'
            )
            self._chequeo_por_frente = False
            self._publish_twist(Twist())
            self._pausa_chequeo_start = self.get_clock().now()
            self._set_state('PAUSA_CHEQUEO_PARED')
            return

        frente_cerca_1_ciclo = z.front_narrow_valid and z.front_narrow < self._umbral_frente_pared
        self._contador_frente_dos_reglas = (
            self._contador_frente_dos_reglas + 1 if frente_cerca_1_ciclo else 0
        )

        if self._contador_frente_dos_reglas >= self._frente_confirmaciones_ciclos:
            self._contador_frente_dos_reglas = 0
            self._publish_event(
                EV.GIRO, f'obstaculo al frente ({z.front_narrow:.2f}m) -> detenido a verificar pared'
            )
            self._chequeo_por_frente = True
            self._publish_twist(Twist())
            self._pausa_chequeo_start = self.get_clock().now()
            self._set_state('PAUSA_CHEQUEO_PARED')
            return

        self._publish_twist(self._twist_wall_follow())

    def _handle_pausa_chequeo_pared(self):
        """Gestiona pausa chequeo pared."""
        self._publish_twist(Twist())
        elapsed = (self.get_clock().now() - self._pausa_chequeo_start).nanoseconds / 1e9
        if elapsed < self._tiempo_chequeo_pared:
            return

        z = self._zones
        lado_libre = bool(self._lado_valid(z) and self._lado_distancia(z) > self._umbral_lado_libre)

        if lado_libre:
            self._contador_derecha_libre += 1
            if self._contador_derecha_libre < self._chequeo_pared_confirmaciones_ciclos:
                return
            self._contador_derecha_libre = 0
            self._decision_actual = self._direccion_vacio()
            self._yaw_inicio_giro = self._yaw
            self._giro_vacio_fase = 1
            self._giro_vacio_repeticiones = 0
            self._contador_giros_fisicos += 1
            self._publish_event(
                EV.GIRO, f'lado seguido vacio ({self._lado_distancia(z):.2f}m) -> {self._decision_actual}'
            )
            self._set_state('GIRAR')
            return

        self._contador_derecha_libre = 0

        if self._chequeo_por_frente:
            self._decision_actual = self._direccion_obstaculo()
            self._yaw_inicio_giro = self._yaw
            self._giro_vacio_fase = 0
            self._contador_giros_fisicos += 1
            self._publish_event(
                EV.GIRO, f'lado seguido ocupado, frente bloqueado -> {self._decision_actual}'
            )
            self._set_state('GIRAR')
            return


        self._publish_event(EV.GIRO, 'lado seguido ocupado -> retoma avance')
        self._avance_chequeo_start_xy = (self._odom_x, self._odom_y)
        self._set_state('AVANZAR_PARALELO')

    def _handle_detectar_cruce(self):
        """Gestiona detectar cruce."""
        self._publish_twist(Twist())

        if self._cruce_muestras is None:
            self._cruce_muestras = {'right': [], 'front': [], 'left': []}

        z = self._zones
        self._cruce_muestras['right'].append(
            bool(z.right_valid and z.right > self._umbral_lado_libre)
        )
        self._cruce_muestras['front'].append(
            bool(z.front_valid and z.front > self._umbral_frente_libre)
        )
        self._cruce_muestras['left'].append(
            bool(z.left_valid and z.left > self._umbral_lado_libre)
        )

        if len(self._cruce_muestras['right']) < self._muestras_confirmacion:
            return

        def consenso(muestras):
            """Ejecuta consenso."""
            return sum(muestras) >= self._consenso_minimo

        self._derecha_libre = consenso(self._cruce_muestras['right'])
        self._frente_libre = consenso(self._cruce_muestras['front'])
        self._izquierda_libre = consenso(self._cruce_muestras['left'])
        self._cruce_muestras = None

        self._publish_event(
            EV.CRUCE,
            f'derecha={self._derecha_libre} frente={self._frente_libre} '
            f'izquierda={self._izquierda_libre}',
        )

        self._buscar_pare_start = self.get_clock().now()
        self._pare_hold_start = None
        self._set_state('BUSCAR_PARE')

    def _handle_buscar_pare(self):
        """Gestiona buscar pare."""
        self._publish_twist(Twist())

        if not self._usar_camara:
            self._set_state('DECIDIR')
            return

        cell = self._grid.cell


        if self._pare_hold_start is not None:
            elapsed = (self.get_clock().now() - self._pare_hold_start).nanoseconds / 1e9
            if elapsed >= self._tiempo_pare:
                self._celdas_pare_respetadas.add(cell)
                self._publish_event(EV.PARE_RESPETADO, f'PARE respetado en {cell}')
                self._set_state('DECIDIR')
            return

        if self._pare_activo and cell not in self._celdas_pare_respetadas:
            self._publish_event(EV.PARE_DETECTADO, f'senal PARE detectada en {cell}')
            self._pare_hold_start = self.get_clock().now()
            return

        elapsed_settle = (self.get_clock().now() - self._buscar_pare_start).nanoseconds / 1e9
        if elapsed_settle >= self._tiempo_espera_camara:
            self._set_state('DECIDIR')

    def _handle_decidir(self):
        """Gestiona decidir."""
        if self._derecha_libre:
            direction = 'DERECHA'
        elif self._frente_libre:
            direction = 'NINGUNO'
        elif self._izquierda_libre:
            direction = 'IZQUIERDA'
        else:
            direction = 'ATRAS'
            self._publish_event(EV.DEAD_END, f'callejon sin salida en {self._grid.cell}')

        self._decision_actual = direction

        if direction == 'NINGUNO':
            if self._modo_simplificado:
                self._begin_avanzar_paralelo()
                self._set_state('AVANZAR_PARALELO')
            else:
                self._alinear_start = None
                self._set_state('ALINEAR')
            return

        self._giro_objetivo = self._compute_turn_target(self._yaw, direction)
        self._publish_event(EV.GIRO, f'{direction} desde {self._grid.cell}')
        self._publish_twist(Twist())
        self._pausa_giro_start = self.get_clock().now()
        self._set_state('PAUSA_GIRO')

    def _handle_pausa_giro(self):
        """Gestiona pausa giro."""
        self._publish_twist(Twist())
        elapsed = (self.get_clock().now() - self._pausa_giro_start).nanoseconds / 1e9
        if elapsed >= self._tiempo_pausa_antes_girar:
            self._set_state('GIRAR')

    def _compute_turn_target(self, yaw: float, direction: str) -> float:
        """Calcula compute turn target."""
        if direction == 'DERECHA':
            delta = -self._angulo_giro_rad
        elif direction == 'IZQUIERDA':
            delta = self._angulo_giro_rad
        elif direction == 'ATRAS':
            delta = math.pi
        else:
            delta = 0.0
        return normalize_angle(yaw + delta)

    def _handle_girar(self):
        """Gestiona girar."""
        if self._logica_dos_reglas:
            self._handle_girar_dinamico()
            return

        error = angle_diff(self._giro_objetivo, self._yaw)

        if abs(error) <= self._tolerancia_giro_rad:
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)


            self._alinear_start = None
            self._set_state('ALINEAR')
            return


        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        cmd.angular.z = self._v_giro_angular if error > 0.0 else -self._v_giro_angular
        self._publish_twist(cmd)

    def _handle_girar_dinamico(self):
        """Gestiona girar dinamico."""
        angulo_girado = abs(angle_diff(self._yaw, self._yaw_inicio_giro))

        if angulo_girado >= self._angulo_giro_rad or angulo_girado >= self._angulo_maximo_giro_rad:
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)
            self.get_logger().info(
                f'GIRO TERMINADO (90 fijo): girado={math.degrees(angulo_girado):.0f} deg'
            )


            odom_deg = math.degrees(angulo_girado)
            imu_deg = abs(math.degrees(self._imu_acum_giro))
            diff_deg = abs(odom_deg - imu_deg)
            if diff_deg > self._umbral_patinaje:
                self._publish_event(
                    EV.PATINAJE,
                    f'posible patinaje: odom={odom_deg:.0f}° imu={imu_deg:.0f}° '
                    f'diff={diff_deg:.0f}°'
                )
            if self._giro_vacio_fase == 1:
                self._avance_fijo_inicio_xy = (self._odom_x, self._odom_y)
                self._set_state('AVANCE_GIRO_VACIO')
                return
            self._begin_avanzar_paralelo()
            self._set_state('AVANZAR_PARALELO')
            return

        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        cmd.angular.z = self._v_giro_angular if self._decision_actual == 'IZQUIERDA' else -self._v_giro_angular
        self._publish_twist(cmd)

    def _handle_avance_giro_vacio(self):
        """Gestiona avance giro vacio."""
        dx = self._odom_x - self._avance_fijo_inicio_xy[0]
        dy = self._odom_y - self._avance_fijo_inicio_xy[1]
        avance = math.hypot(dx, dy)

        if avance < self._avance_giro_vacio:
            cmd = Twist()
            cmd.linear.x = self._velocidad_recta
            self._publish_twist(cmd)
            return

        self._publish_twist(Twist())
        z = self._zones
        lado_libre = bool(self._lado_valid(z) and self._lado_distancia(z) > self._umbral_lado_libre)

        if lado_libre and self._giro_vacio_repeticiones < self._giro_vacio_max_repeticiones:
            self._giro_vacio_repeticiones += 1
            self._yaw_inicio_giro = self._yaw
            self._contador_giros_fisicos += 1
            self._publish_event(
                EV.GIRO,
                f'avanzo {avance:.2f}m, lado seguido sigue vacio '
                f'-> otro giro (rep {self._giro_vacio_repeticiones})'
            )
            self._set_state('GIRAR')
            return

        self._giro_vacio_fase = 0
        motivo = 'lado seguido ocupado' if not lado_libre else 'tope de repeticiones'
        self._publish_event(EV.GIRO, f'avanzo {avance:.2f}m, {motivo} -> retoma avance')
        self._begin_avanzar_paralelo()
        self._set_state('AVANZAR_PARALELO')

    def _handle_alinear(self):
        """Gestiona alinear."""
        if self._alinear_start is None:
            self._alinear_start = self.get_clock().now()

        z = self._zones
        if not (z.right_front_valid and z.right_rear_valid):


            self._alinear_start = None
            self._set_state('VERIFICAR_META')
            return

        error_angulo = z.right_front - z.right_rear
        elapsed = (self.get_clock().now() - self._alinear_start).nanoseconds / 1e9

        if abs(error_angulo) <= self._tolerancia_alineacion or elapsed >= self._tiempo_max_alinear:
            self._publish_twist(Twist())
            self._alinear_start = None
            self._set_state('VERIFICAR_META')
            return

        cmd = Twist()
        cmd.linear.x = self._v_alinear_lineal
        cmd.angular.z = -self._v_alinear_angular if error_angulo > 0.0 else self._v_alinear_angular
        self._publish_twist(cmd)

    def _handle_verificar_meta(self):
        """Gestiona verificar meta."""
        if self._grid.cell == self._celda_meta:
            self._publish_twist(Twist())
            self._publish_event(EV.META, f'meta alcanzada en {self._grid.cell}')
            self._terminado = True
            self._set_state('META')
            return

        self._begin_avanzar_paralelo()
        self._set_state('AVANZAR_PARALELO')


    def _calcular_ruta(self) -> bool:
        """Calcula calcular ruta."""
        n = len(self._ruta_fija_giros)
        self._ruta_movimientos = [
            {
                'giro': giro,
                'distancia_m': self._ruta_fija_distancias[i],
                'wall_follow': i == 0 or i == n - 1,
                'hasta_verde': i == n - 1,
            }
            for i, giro in enumerate(self._ruta_fija_giros)
        ]
        self._ruta_idx = 0
        self._ruta_fase = 'GIRO'
        self._ruta_giro_restante = None
        self._publicar_ruta()
        self._publish_event(
            EV.META, f'ruta FIJA cargada: {len(self._ruta_movimientos)} tramos')
        return True

    def _publicar_ruta(self):
        """Publica publicar ruta."""
        sim = RouteExplorer.from_cell_name(
            self._ruta_celda_inicio, self._ruta_heading_inicial)
        celdas = [sim.cell]
        for mov in self._ruta_movimientos:
            sim.girar(mov['giro'])
            n_celdas = max(1, round(mov['distancia_m'] / self._tamano_celda))
            for _ in range(n_celdas):
                celdas.append(sim.avanzar())
        self._ruta_json = json.dumps({
            'listo': True,
            'fija': True,
            'celdas': [[int(c), int(r)] for (c, r) in celdas],
        })
        self._ruta_pub.publish(String(data=self._ruta_json))

    def _republicar_ruta(self):
        """Publica republicar ruta."""
        if self._ruta_json is None:
            return
        self._ruta_pub_counter += 1
        if self._ruta_pub_counter >= int(max(1.0, self._control_rate_hz)):
            self._ruta_pub_counter = 0
            self._ruta_pub.publish(String(data=self._ruta_json))

    def _handle_espera_ruta(self):
        """Gestiona espera ruta."""
        self._publish_twist(Twist())
        self._republicar_ruta()

    def _handle_seguir_ruta(self):
        """Gestiona seguir ruta."""
        self._republicar_ruta()
        if (self._ruta_movimientos is None
                or self._ruta_idx >= len(self._ruta_movimientos)):
            self._publish_twist(Twist())
            self._publish_event(
                EV.META,
                'ruta corta completada; DETENIDO. Reenvia /maze/iniciar_ruta '
                'para repetir (colocalo en el inicio mirando NORTE).'
            )
            self._set_state('ESPERA_RUTA')
            return

        mov = self._ruta_movimientos[self._ruta_idx]
        if self._ruta_fase == 'GIRO':
            self._ejecutar_giro_ruta(mov['giro'])
        else:
            self._ejecutar_avance_ruta(
                mov['distancia_m'], mov.get('wall_follow', False),
                mov.get('hasta_verde', False))

    def _iniciar_avance_ruta(self):
        """Ejecuta iniciar avance ruta."""
        self._ruta_fase = 'AVANCE'
        self._ruta_avance_xy0 = (self._odom_x, self._odom_y)

    def _ejecutar_giro_ruta(self, direccion: str):
        """Ejecuta ejecutar giro ruta."""
        if direccion == 'NINGUNO':
            self._iniciar_avance_ruta()
            return
        if self._ruta_giro_restante is None:
            self._ruta_giro_restante = 2 if direccion == 'ATRAS' else 1
            self._ruta_giro_yaw0 = self._yaw
        girado = abs(angle_diff(self._yaw, self._ruta_giro_yaw0))
        if girado >= self._angulo_giro_rad:
            self._publish_twist(Twist())
            self._ruta_giro_restante -= 1
            if self._ruta_giro_restante > 0:
                self._ruta_giro_yaw0 = self._yaw
                return
            self._ruta_giro_restante = None
            self._iniciar_avance_ruta()
            return
        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        izquierda = direccion in ('IZQUIERDA', 'ATRAS')
        cmd.angular.z = self._v_giro_angular if izquierda else -self._v_giro_angular
        self._publish_twist(cmd)

    def _ejecutar_avance_ruta(self, distancia_m, wall_follow=False, hasta_verde=False):
        """Ejecuta ejecutar avance ruta."""
        if hasta_verde and self._verde_anterior:
            self._publish_twist(Twist())
            self._ruta_idx += 1
            self._ruta_fase = 'GIRO'
            self._ruta_giro_restante = None
            return

        dx = self._odom_x - self._ruta_avance_xy0[0]
        dy = self._odom_y - self._ruta_avance_xy0[1]
        if math.hypot(dx, dy) >= distancia_m:
            self._publish_twist(Twist())
            self._ruta_idx += 1
            self._ruta_fase = 'GIRO'
            self._ruta_giro_restante = None
            return

        if wall_follow:
            self._publish_twist(self._twist_wall_follow())
            return
        cmd = Twist()
        cmd.linear.x = self._velocidad_recta
        self._publish_twist(cmd)

    def _handle_meta(self):
        """Gestiona meta."""
        self._publish_twist(Twist())

    def _handle_detenido(self):
        """Gestiona detenido."""
        self._publish_twist(Twist())


    def _metricas_actuales(self) -> dict:
        """Ejecuta metricas actuales."""
        if self._tiempo_inicio is not None:
            tiempo_s = (self.get_clock().now() - self._tiempo_inicio).nanoseconds / 1e9
        else:
            tiempo_s = 0.0
        return {
            'llego_meta': self._meta_detectada,
            'tiempo_s': round(tiempo_s, 1),
            'long_ruta_cm': round(self._distancia_total_m * 100.0, 1),
            'colisiones': self._contador_colisiones,
            'pare_detectados': self._contador_pare_detectados,
            'pare_respetados': self._contador_pare_respetados,
        }

    def _publish_twist(self, cmd: Twist):
        """Publica publish twist."""
        self._cmd_pub.publish(cmd)
        z = self._zones
        if z is not None:
            followed = z.left_line_distance_m if self._seguir_izquierda else z.right_line_distance_m
            followed_valid = z.left_line_valid if self._seguir_izquierda else z.right_line_valid
            rear = z.left if self._seguir_izquierda else z.right
            rear_valid = z.left_valid if self._seguir_izquierda else z.right_valid
            payload = {
                'estado': self._state,
                'v': float(cmd.linear.x),
                'w': float(cmd.angular.z),
                'd_frente': float(z.front) if z.front_valid else None,
                'd_atras': None,
                'd_izq': float(z.left) if z.left_valid else None,
                'd_der': float(z.right) if z.right_valid else None,
                'd_lado_frontal': float(followed) if followed_valid else None,
                'd_lado_trasera': float(rear) if rear_valid else None,


                'giros_fisicos': self._contador_giros_fisicos,
            }


            payload.update(self._metricas_meta or self._metricas_actuales())
            self._metrics_pub.publish(String(data=json.dumps(payload)))

    def _publish_event(self, tipo: str, detalle: str):
        """Publica publish event."""
        self._event_pub.publish(String(data=json.dumps({'tipo': tipo, 'detalle': detalle})))
        self.get_logger().info(f'[{tipo}] {detalle}')

    def _set_state(self, new_state: str):


        """Ejecuta set state."""
        if new_state == 'GIRAR' and self._state != 'GIRAR':
            self._imu_acum_giro = 0.0
            self._imu_t_prev = self.get_clock().now()
        self._state = new_state
        self._state_pub.publish(String(data=self._state))


def main(args=None):
    """Inicia el nodo."""
    rclpy.init(args=args)
    node = MazeSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
