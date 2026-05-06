#!/usr/bin/env python3
# Nodo de prueba autónomo: busca una persona con YOLO girando en el sitio,
# navega a 1.5 m de ella con Nav2 y para.
#
# Pipeline esperado (lanzar con find_person.launch.py):
#   yolo_node → /yolo/detections (yolo_msgs/DetectionArray)
#   yolo_depth_node → /detections_3d (vision_msgs/Detection3DArray)
#   find_person_test_node ← /detections_3d  + Nav2 action server

import math
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray


class State(IntEnum):
    WAIT_NAV   = 0   # esperando a que Nav2 esté disponible
    SEARCHING  = 1   # girando buscando persona
    NAVIGATING = 2   # navegando al punto objetivo
    DONE       = 3   # llegó (o falló), se detiene


class FindPersonTestNode(Node):

    def __init__(self):
        super().__init__('find_person_test_node')

        self.declare_parameter('detections_topic',   '/detections_3d')
        self.declare_parameter('person_class',       'person')
        self.declare_parameter('hold_distance',      1.5)
        self.declare_parameter('search_angular_spd', 0.5)
        self.declare_parameter('search_timeout_sec', 60.0)
        self.declare_parameter('goal_timeout_sec',   90.0)
        self.declare_parameter('base_frame',         'base_link')
        self.declare_parameter('map_frame',          'map')

        self._person_class   = self.get_parameter('person_class').value
        self._hold_dist      = float(self.get_parameter('hold_distance').value)
        self._search_spd     = float(self.get_parameter('search_angular_spd').value)
        self._search_timeout = float(self.get_parameter('search_timeout_sec').value)
        self._goal_timeout   = float(self.get_parameter('goal_timeout_sec').value)
        self._base_frame     = self.get_parameter('base_frame').value
        self._map_frame      = self.get_parameter('map_frame').value

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_pub    = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(
            Detection3DArray,
            self.get_parameter('detections_topic').value,
            self._det_cb,
            qos_profile_sensor_data,
        )

        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._last_det     = None
        self._state        = State.WAIT_NAV
        self._state_ts     = self.get_clock().now().nanoseconds / 1e9
        self._goal_handle  = None
        self._goal_done    = False
        self._goal_success = False

        self.create_timer(0.1, self._tick)
        self.get_logger().info('FindPersonTestNode iniciado. Esperando Nav2...')

    # ------------------------------------------------------------------ #
    def _det_cb(self, msg: Detection3DArray):
        for det in msg.detections:
            if not det.results:
                continue
            if det.results[0].hypothesis.class_id != self._person_class:
                continue
            x, y, z = (det.bbox.center.position.x,
                       det.bbox.center.position.y,
                       det.bbox.center.position.z)
            self._last_det = {
                'stamp':    msg.header.stamp,
                'frame_id': det.header.frame_id or msg.header.frame_id,
                'x': x, 'y': y, 'z': z,
                'dist': math.sqrt(x*x + y*y + z*z),
            }
            return

    def _person_fresh(self, max_age: float = 1.5) -> bool:
        if self._last_det is None:
            return False
        now   = self.get_clock().now().nanoseconds / 1e9
        stamp = self._last_det['stamp'].sec + self._last_det['stamp'].nanosec / 1e9
        return (now - stamp) <= max_age

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self) -> float:
        return self._now() - self._state_ts

    def _set_state(self, s: State):
        self.get_logger().info(f'Estado: {State(self._state).name} → {State(s).name}')
        self._state    = s
        self._state_ts = self._now()

    # ------------------------------------------------------------------ #
    def _tick(self):
        if self._state == State.WAIT_NAV:
            if self._nav_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().info('Nav2 disponible — empezando búsqueda')
                self._set_state(State.SEARCHING)

        elif self._state == State.SEARCHING:
            if self._elapsed() > self._search_timeout:
                self._stop()
                self.get_logger().error('Timeout: no se encontró ninguna persona.')
                self._set_state(State.DONE)
                return

            if self._person_fresh():
                self._stop()
                det  = self._last_det
                self.get_logger().info(
                    f'Persona encontrada a {det["dist"]:.2f} m — calculando goal...'
                )
                pose = self._build_goal_pose(det)
                if pose is None:
                    return  # TF no disponible, reintenta en el siguiente ciclo
                self._send_nav_goal(pose)
                self._set_state(State.NAVIGATING)
                return

            # Girar en el sitio
            spin = Twist()
            spin.angular.z = self._search_spd
            self._cmd_pub.publish(spin)

        elif self._state == State.NAVIGATING:
            if self._elapsed() > self._goal_timeout:
                self.get_logger().warn('Timeout de navegación — cancelando.')
                if self._goal_handle:
                    self._goal_handle.cancel_goal_async()
                self._set_state(State.DONE)
                return
            if self._goal_done:
                if self._goal_success:
                    self.get_logger().info('¡Llegué! Parado a ~1.5 m de la persona.')
                else:
                    self.get_logger().warn('Navegación fallida.')
                self._set_state(State.DONE)

        elif self._state == State.DONE:
            self._stop()

    # ------------------------------------------------------------------ #
    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _send_nav_goal(self, pose: PoseStamped):
        self._goal_done    = False
        self._goal_success = False
        self._goal_handle  = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(
            f'Navegando... dist restante = {fb.distance_remaining:.2f} m',
            throttle_duration_sec=3.0,
        )

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rechazado por Nav2.')
            self._goal_done    = True
            self._goal_success = False
            return
        self.get_logger().info('Goal aceptado.')
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._goal_done    = True
        self._goal_success = (future.result().status == GoalStatus.STATUS_SUCCEEDED)

    # ------------------------------------------------------------------ #
    def _build_goal_pose(self, det) -> PoseStamped:
        try:
            tr = self._tf_buf.lookup_transform(
                self._map_frame, det['frame_id'], rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF persona: {e}')
            return None

        # Posición de la persona en map
        q   = tr.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cy, sy = math.cos(yaw), math.sin(yaw)
        px = tr.transform.translation.x + det['x'] * cy - det['y'] * sy
        py = tr.transform.translation.y + det['x'] * sy + det['y'] * cy

        # Posición del robot en map
        try:
            tr_robot = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF robot: {e}')
            return None

        rx = tr_robot.transform.translation.x
        ry = tr_robot.transform.translation.y

        dx, dy = px - rx, py - ry
        d = math.sqrt(dx * dx + dy * dy)
        if d < 0.05:
            self.get_logger().info('Robot ya muy cerca — sin moverse.')
            return None

        theta    = math.atan2(dy, dx)
        target_d = max(0.0, d - self._hold_dist)

        pose = PoseStamped()
        pose.header.frame_id    = self._map_frame
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = rx + (dx / d) * target_d
        pose.pose.position.y    = ry + (dy / d) * target_d
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        self.get_logger().info(
            f'Goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) '
            f'— {target_d:.2f} m avanzando hacia persona en map'
        )
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = FindPersonTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
