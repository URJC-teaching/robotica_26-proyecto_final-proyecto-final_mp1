#!/usr/bin/env python3
# Convierte detecciones 2D de YOLO (yolo_msgs/DetectionArray) en
# vision_msgs/Detection3DArray para mission_manager.
#
# Sigue el patrón de p6 (sensors_p6/camera + vff_control_p6):
#  - QoS: qos_profile_sensor_data (BEST_EFFORT) en todos los topics
#  - Calcula ángulo con la intrínseca fx/cx (idéntico a yolo_class_detector_node_2d)
#  - Estima distancia con la altura del bbox y altura típica de persona
#    (apaño 2D para evitar el detect_3d_node de yolo_ros, que es pesado en VM)
#
# Topics (ya están en el namespace global porque se hace remap en launch):
#   in : 'detections'   (yolo_msgs/DetectionArray)   — /yolo/detections
#   in : 'camera_info'  (sensor_msgs/CameraInfo)     — /rgbd_camera/... | /camera/...
#   out: 'detections_3d'(vision_msgs/Detection3DArray) — /detections_3d
#
# Frames: la salida está en frame 'camera_link' o 'camera_rgb_frame' (parámetro
# camera_frame). El mission_manager hace TF lookup desde ese frame a 'map'.
# Se usa convención del frame óptico ROS: x=derecha, y=abajo, z=adelante,
# pero EXPRESADA en camera_link convertida a (x=adelante, y=izquierda) para que
# el TF lookup en mission_manager funcione (asume camera_link x-forward).

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    Detection3DArray, Detection3D, ObjectHypothesisWithPose
)
from yolo_msgs.msg import DetectionArray


class YoloDepthNode(Node):

    def __init__(self):
        super().__init__('yolo_depth_node')

        # Parámetros (mismos nombres que p6 + altura de persona para apaño 2D)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('person_height_m', 1.7)
        self.declare_parameter('min_bbox_height_px', 20)
        self.declare_parameter('max_distance_m', 8.0)

        self._target_class    = self.get_parameter('target_class').value
        self._camera_frame    = self.get_parameter('camera_frame').value
        self._person_height_m = float(self.get_parameter('person_height_m').value)
        self._min_bbox_h      = float(self.get_parameter('min_bbox_height_px').value)
        self._max_distance_m  = float(self.get_parameter('max_distance_m').value)

        # Intrínsecas de cámara
        self._fx = self._fy = self._cx = self._cy = None
        self._configured = False

        # camera_info (BEST_EFFORT — gz_bridge y openni2_camera publican así)
        self._info_sub = self.create_subscription(
            CameraInfo, 'camera_info',
            self._info_cb, qos_profile_sensor_data,
        )

        # Detecciones 2D de YOLO
        self.create_subscription(
            DetectionArray, 'detections',
            self._det_cb, qos_profile_sensor_data,
        )

        self._pub = self.create_publisher(
            Detection3DArray, 'detections_3d', qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'YoloDepthNode listo — clase="{self._target_class}", '
            f'camera_frame="{self._camera_frame}", '
            f'person_height={self._person_height_m:.2f} m'
        )

    # ---------------------------------------------------------------- #
    def _info_cb(self, msg: CameraInfo):
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        if not self._configured:
            self._configured = True
            self.get_logger().info(
                f'CameraInfo recibida: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f} '
                f'size={msg.width}x{msg.height}'
            )
            # Una sola lectura es suficiente — los parámetros no cambian
            self.destroy_subscription(self._info_sub)

    # ---------------------------------------------------------------- #
    def _det_cb(self, msg: DetectionArray):
        if not self._configured:
            self.get_logger().warn(
                'Detección recibida pero CameraInfo aún no disponible',
                throttle_duration_sec=5.0,
            )
            return

        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._camera_frame

        for det in msg.detections:
            if det.class_name != self._target_class:
                continue

            bbox_h = float(det.bbox.size.y)
            if bbox_h < self._min_bbox_h:
                continue  # bbox demasiado pequeño → estimación poco fiable

            # 1) Distancia con apaño: persona ~1.7 m, h_pix = (h_real * fy) / d
            distance = (self._person_height_m * self._fy) / bbox_h
            if distance > self._max_distance_m:
                continue

            # 2) Ángulo con la intrínseca (igual que p6 yolo_class_detector_node_2d)
            #    pixel_offset_x>0 → persona a la derecha de la imagen
            #    en camera_link (x adelante, y izquierda) eso es y NEGATIVO
            u = float(det.bbox.center.position.x)
            angle_yaw = -math.atan((u - self._cx) / self._fx)

            x_camlink = distance * math.cos(angle_yaw)   # adelante
            y_camlink = distance * math.sin(angle_yaw)   # izquierda(+) / derecha(-)
            z_camlink = 0.0                              # ignoramos altura

            d3 = Detection3D()
            d3.header = out.header
            d3.bbox.center.position.x = x_camlink
            d3.bbox.center.position.y = y_camlink
            d3.bbox.center.position.z = z_camlink
            d3.bbox.size.x = d3.bbox.size.y = d3.bbox.size.z = 0.5

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self._target_class
            hyp.hypothesis.score = float(det.score)
            hyp.pose.pose.position.x = x_camlink
            hyp.pose.pose.position.y = y_camlink
            hyp.pose.pose.position.z = z_camlink
            d3.results.append(hyp)

            out.detections.append(d3)

            self.get_logger().info(
                f'{self._target_class}: dist={distance:.2f} m, '
                f'angle={math.degrees(angle_yaw):+.1f}°, '
                f'bbox_h={bbox_h:.0f}px, score={det.score:.2f}',
                throttle_duration_sec=1.0,
            )

        if out.detections:
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
