#!/usr/bin/env python3
"""Módulo metrics_logger."""

import csv
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


CAMPOS = [
    'ronda', 'fecha', 'llego_meta', 'tiempo_s', 'long_ruta_cm',
    'long_optima_cm', 'eficiencia', 'colisiones', 'pare_reales',
    'pare_detectados', 'pare_respetados', 'pare_falsos',
    'dead_ends_visitados',
]


class MetricsLogger(Node):
    def __init__(self):
        """Inicializa el componente."""
        super().__init__('metrics_logger')

        self.declare_parameter('ronda', 1)
        self.declare_parameter('pare_reales', 2)
        self.declare_parameter('long_optima_cm', 520.0)
        self.declare_parameter('pare_falsos', 0)
        self.declare_parameter('archivo_csv', '~/metricas_granprix.csv')

        g = lambda n: self.get_parameter(n).value
        self.ronda = int(g('ronda'))
        self.pare_reales = int(g('pare_reales'))
        self.long_optima_cm = float(g('long_optima_cm'))
        self.pare_falsos = int(g('pare_falsos'))
        self.archivo = os.path.expanduser(g('archivo_csv'))

        self.ultimas = None
        self.guardado = False
        self.create_subscription(String, '/maze/metricas', self.cb_metricas, 10)

        self.get_logger().info(
            f'metrics_logger | ronda={self.ronda} '
            f'pare_reales={self.pare_reales} '
            f'optima={self.long_optima_cm:.0f} cm | CSV → {self.archivo}')

    def cb_metricas(self, msg: String):
        """Procesa cb metricas."""
        try:
            self.ultimas = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if self.ultimas.get('llego_meta') and not self.guardado:
            self.guardar_csv()
            self.guardado = True

    def guardar_csv(self):
        """Ejecuta guardar csv."""
        if self.ultimas is None:
            self.get_logger().warn('sin datos de /maze/metricas — no se guarda')
            return
        m = self.ultimas
        ruta = float(m.get('long_ruta_cm', 0.0))
        eficiencia = round(self.long_optima_cm / ruta, 3) if ruta > 0 else 0.0

        existe = os.path.exists(self.archivo)
        with open(self.archivo, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=CAMPOS)
            if not existe:
                w.writeheader()
            w.writerow({
                'ronda':               self.ronda,
                'fecha':               datetime.now().strftime('%Y-%m-%d %H:%M'),
                'llego_meta':          'Si' if m.get('llego_meta') else 'No',
                'tiempo_s':            m.get('tiempo_s', 0.0),
                'long_ruta_cm':        ruta,
                'long_optima_cm':      self.long_optima_cm,
                'eficiencia':          eficiencia,
                'colisiones':          m.get('colisiones', 0),
                'pare_reales':         self.pare_reales,
                'pare_detectados':     m.get('pare_detectados', 0),
                'pare_respetados':     m.get('pare_respetados', 0),
                'pare_falsos':         self.pare_falsos,
                'dead_ends_visitados': m.get('dead_ends_visitados', 0),
            })
        self.get_logger().info(
            f"[ronda {self.ronda}] CSV → {self.archivo} | "
            f"meta={m.get('llego_meta')} t={m.get('tiempo_s')}s "
            f"ruta={ruta:.0f}cm ef={eficiencia:.2f} "
            f"pare={m.get('pare_respetados')}/{self.pare_reales}")


def main(args=None):
    """Inicia el nodo."""
    rclpy.init(args=args)
    nodo = MetricsLogger()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if not nodo.guardado:
            nodo.guardar_csv()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
