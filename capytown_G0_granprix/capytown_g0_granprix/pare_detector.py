#!/usr/bin/env python3
"""Módulo pare_detector."""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, UInt16
from cv_bridge import CvBridge


class PareDetector(Node):
    def __init__(self):
        """Inicializa el componente."""
        super().__init__('pare_detector')
        self.bridge = CvBridge()

        self.declare_parameters('', [

            ('rojo_h_bajo_max',   10),
            ('rojo_h_alto_min',  170),
            ('rojo_s_min',       100),
            ('rojo_v_min',        70),

            ('amarillo_h_min',    20),
            ('amarillo_h_max',    35),
            ('amarillo_s_min',   100),
            ('amarillo_v_min',    80),
            ('amarillo_area_min', 600),


            ('verde_h_min',       35),
            ('verde_h_max',       95),
            ('verde_s_min',       40),
            ('verde_s_max',      255),
            ('verde_v_min',       60),
            ('verde_v_max',      255),

            ('verde_area_min',    600),
            ('verde_aspecto_min', 0.05),
            ('verde_aspecto_max', 20.0),
            ('verde_area_max',  150000),
            ('verde_solidez_min', 0.35),

            ('area_min',         600),
            ('area_max',       60000),
            ('aspecto_min',      0.5),
            ('aspecto_max',      2.0),
            ('solidez_min',      0.75),


            ('franja_inferior',  1.0),


            ('franja_verde',     0.6667),


            ('centro_tol_frac',  0.20),
            ('frames_confirmacion', 3),


            ('beep_cooldown_s',  5.0),
            ('publish_debug',    True),
            ('topic_camera',      '/image_raw'),
        ])

        gp = lambda n: self.get_parameter(n).value
        self.h_bajo = int(gp('rojo_h_bajo_max'))
        self.h_alto = int(gp('rojo_h_alto_min'))
        self.s_min = int(gp('rojo_s_min'))
        self.v_min = int(gp('rojo_v_min'))
        self.amarillo_h_min = int(gp('amarillo_h_min'))
        self.amarillo_h_max = int(gp('amarillo_h_max'))
        self.amarillo_s_min = int(gp('amarillo_s_min'))
        self.amarillo_v_min = int(gp('amarillo_v_min'))
        self.amarillo_area_min = float(gp('amarillo_area_min'))
        self.verde_h_min = int(gp('verde_h_min'))
        self.verde_h_max = int(gp('verde_h_max'))
        self.verde_s_min = int(gp('verde_s_min'))
        self.verde_s_max = int(gp('verde_s_max'))
        self.verde_v_min = int(gp('verde_v_min'))
        self.verde_v_max = int(gp('verde_v_max'))
        self.verde_area_min = float(gp('verde_area_min'))
        self.verde_aspecto_min = float(gp('verde_aspecto_min'))
        self.verde_aspecto_max = float(gp('verde_aspecto_max'))
        self.verde_area_max = float(gp('verde_area_max'))
        self.verde_solidez_min = float(gp('verde_solidez_min'))
        self.area_min = float(gp('area_min'))
        self.area_max = float(gp('area_max'))
        self.aspecto_min = float(gp('aspecto_min'))
        self.aspecto_max = float(gp('aspecto_max'))
        self.solidez_min = float(gp('solidez_min'))
        self.franja_inferior = float(gp('franja_inferior'))
        self.franja_verde = float(gp('franja_verde'))
        self.centro_tol_frac = float(gp('centro_tol_frac'))
        self.frames_confirmacion = int(gp('frames_confirmacion'))
        self.beep_cooldown_s = float(gp('beep_cooldown_s'))
        self.publish_debug = bool(gp('publish_debug'))
        self.topic_camera = str(gp('topic_camera'))

        self.frames_seguidos = 0
        self.frames_seguidos_verde = 0
        self.frames_seguidos_amarillo = 0
        self._rojo_anterior = False
        self._amarillo_anterior = False
        self._ultimo_beep = None

        self.create_subscription(Image, self.topic_camera, self.on_image, 10)
        self.pub_pare = self.create_publisher(Bool, '/pare_detectado', 10)
        self.pub_verde = self.create_publisher(Bool, '/verde_detectado', 10)
        self.pub_amarillo = self.create_publisher(Bool, '/amarillo_detectado', 10)
        self.pub_beep = self.create_publisher(UInt16, '/beep', 10)
        self.pub_area = self.create_publisher(Float32, '/pare/area', 10)
        self.pub_dbg = self.create_publisher(Image, '/pare/debug_image', 10)

        self.get_logger().info(
            f'pare_detector listo en {self.topic_camera} | '
            f'rojo H<= {self.h_bajo} o H>={self.h_alto}, '
            f'S>={self.s_min}, V>={self.v_min} | area>={self.area_min}')


    def on_image(self, msg: Image):
        """Procesa on image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame[:int(self.franja_inferior * h), :],
                           cv2.COLOR_BGR2HSV)

        hsv_verde = cv2.cvtColor(frame[:max(1, int(self.franja_verde * h)), :],
                                 cv2.COLOR_BGR2HSV)

        area_min = self.area_min


        mask_rojo = self._mascara_rojo(hsv)
        mask_rojo = self._quitar_componentes_pequenos(mask_rojo, area_min)
        det_r, mejor_r = self._validar_forma(mask_rojo, area_min,
                                             exigir_centro=False)
        self.frames_seguidos = self.frames_seguidos + 1 if det_r else 0
        conf_r = self.frames_seguidos >= self.frames_confirmacion


        mask_verde = self._mascara_verde(hsv_verde)
        mask_verde = self._quitar_componentes_pequenos(
            mask_verde, self.verde_area_min)
        det_v, mejor_v = self._validar_forma(
            mask_verde, self.verde_area_min,
            aspecto_min=self.verde_aspecto_min,
            aspecto_max=self.verde_aspecto_max,
            area_max=self.verde_area_max,
            solidez_min=self.verde_solidez_min,
            exigir_centro=False)
        self.frames_seguidos_verde = self.frames_seguidos_verde + 1 if det_v else 0
        conf_v = self.frames_seguidos_verde >= self.frames_confirmacion


        mask_amarillo = self._mascara_amarillo(hsv)
        mask_amarillo = self._quitar_componentes_pequenos(
            mask_amarillo, self.amarillo_area_min)
        det_a, mejor_a = self._validar_forma(
            mask_amarillo, self.amarillo_area_min)
        self.frames_seguidos_amarillo = (
            self.frames_seguidos_amarillo + 1 if det_a else 0)
        conf_a = self.frames_seguidos_amarillo >= self.frames_confirmacion

        self.pub_pare.publish(Bool(data=bool(conf_r)))
        self.pub_verde.publish(Bool(data=bool(conf_v)))
        self.pub_amarillo.publish(Bool(data=bool(conf_a)))
        self.pub_area.publish(Float32(
            data=float(mejor_r['area']) if mejor_r else 0.0))


        ahora = self.get_clock().now()
        en_cooldown = (self._ultimo_beep is not None and
                       (ahora - self._ultimo_beep).nanoseconds / 1e9
                       < self.beep_cooldown_s)
        if not en_cooldown and (
                (conf_r and not self._rojo_anterior) or
                (conf_a and not self._amarillo_anterior)):
            self._iniciar_beep()
            self._ultimo_beep = ahora
        self._rojo_anterior = conf_r
        self._amarillo_anterior = conf_a

        if self.publish_debug:
            self._publicar_debug(frame, mask_rojo, mask_verde, mask_amarillo,
                                 mejor_r, mejor_v, mejor_a,
                                 conf_r, conf_v, conf_a, msg)


    def _limpiar(self, mask):
        """Ejecuta limpiar."""
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _mascara_rojo(self, hsv):

        """Ejecuta mascara rojo."""
        lo1 = np.array([0, self.s_min, self.v_min], dtype=np.uint8)
        hi1 = np.array([self.h_bajo, 255, 255], dtype=np.uint8)
        lo2 = np.array([self.h_alto, self.s_min, self.v_min], dtype=np.uint8)
        hi2 = np.array([179, 255, 255], dtype=np.uint8)
        return self._limpiar(cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1),
                                            cv2.inRange(hsv, lo2, hi2)))

    def _mascara_verde(self, hsv):
        """Ejecuta mascara verde."""
        lo = np.array([self.verde_h_min, self.verde_s_min, self.verde_v_min], dtype=np.uint8)
        hi = np.array([self.verde_h_max, self.verde_s_max, self.verde_v_max], dtype=np.uint8)
        return self._limpiar(cv2.inRange(hsv, lo, hi))

    def _mascara_amarillo(self, hsv):
        """Ejecuta mascara amarillo."""
        lo = np.array([self.amarillo_h_min, self.amarillo_s_min,
                       self.amarillo_v_min], dtype=np.uint8)
        hi = np.array([self.amarillo_h_max, 255, 255], dtype=np.uint8)
        return self._limpiar(cv2.inRange(hsv, lo, hi))


    _BEEP_DURACION_MS = 150

    def _iniciar_beep(self):
        """Ejecuta iniciar beep."""
        self.pub_beep.publish(UInt16(data=self._BEEP_DURACION_MS))

    @staticmethod
    def _quitar_componentes_pequenos(mask, area_min):
        """Ejecuta quitar componentes pequenos."""
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        limpia = np.zeros_like(mask)
        for etiqueta in range(1, n):
            if stats[etiqueta, cv2.CC_STAT_AREA] >= area_min:
                limpia[labels == etiqueta] = 255
        return limpia


    def _validar_forma(self, mask, area_min, aspecto_min=None,
                       aspecto_max=None, area_max=None, solidez_min=None,
                       exigir_centro=True):
        """Ejecuta validar forma."""
        aspecto_min = self.aspecto_min if aspecto_min is None else aspecto_min
        aspecto_max = self.aspecto_max if aspecto_max is None else aspecto_max
        area_max = self.area_max if area_max is None else area_max
        solidez_min = self.solidez_min if solidez_min is None else solidez_min
        cx_img = mask.shape[1] / 2.0
        tol_px = self.centro_tol_frac * mask.shape[1]
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        mejor = None
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area < area_min or area > area_max:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if exigir_centro and abs((x + bw / 2.0) - cx_img) > tol_px:
                continue
            aspecto = bw / float(bh)
            if not (aspecto_min <= aspecto <= aspecto_max):
                continue
            hull = cv2.convexHull(cnt)
            area_hull = cv2.contourArea(hull)
            if area_hull < 1e-3 or area / area_hull < solidez_min:
                continue
            if mejor is None or area > mejor['area']:
                mejor = {'area': area, 'bbox': (x, y, bw, bh)}
        return mejor is not None, mejor


    def _publicar_debug(self, frame, mask_rojo, mask_verde, mask_amarillo,
                        mejor_r, mejor_v, mejor_a,
                        conf_r, conf_v, conf_a, header_msg):
        """Publica publicar debug."""
        dbg = frame.copy()
        overlay = dbg.copy()


        overlay[:mask_rojo.shape[0]][mask_rojo > 0] = (0, 0, 255)
        overlay[:mask_verde.shape[0]][mask_verde > 0] = (0, 200, 0)
        overlay[:mask_amarillo.shape[0]][mask_amarillo > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.4, dbg, 0.6, 0, dbg)
        Hf, Wf = dbg.shape[:2]


        yv = int(self.franja_verde * Hf)
        cv2.line(dbg, (0, yv), (Wf, yv), (0, 200, 0), 1)
        cv2.putText(dbg, 'VERDE arriba de esta linea', (5, max(14, yv - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)
        for mejor, conf, txt, col in ((mejor_r, conf_r, 'PARE', (0, 0, 255)),
                                      (mejor_v, conf_v, 'VERDE', (0, 200, 0)),
                                      (mejor_a, conf_a, 'AMARILLO', (0, 220, 255))):
            if mejor is None:
                continue
            x, y, bw, bh = mejor['bbox']
            color = (0, 255, 0) if conf else col
            cv2.rectangle(dbg, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(dbg, f'{txt} {mejor["area"]:.0f}px', (x, max(15, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        etq = []
        if conf_r:
            etq.append('PARE!')
        if conf_v:
            etq.append('VERDE!')
        if conf_a:
            etq.append('AMARILLO!')
        estado = ' '.join(etq) if etq else '-'
        cv2.putText(dbg, estado, (5, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = header_msg.header
        self.pub_dbg.publish(out)


def main(args=None):
    """Inicia el nodo."""
    rclpy.init(args=args)
    node = PareDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
