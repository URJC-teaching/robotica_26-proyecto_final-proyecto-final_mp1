#!/usr/bin/env python3
# Proyecto Final ROS2 Jazzy
# Orquestador FSM: Nav2 + HRI (simple_hri: stt_service_local + tts_service_local) + YOLO
#
# Menú por voz (dí un número o una palabra clave):
#   1 -> ir a "papelera"
#   2 -> ir a "punto_a"
#   3 -> buscar persona con YOLO y acercarse (<= 1.5 m durante 2 s)
#   4 -> ir a "punto_b" y volver al inicio

import math
import os
import random
from enum import IntEnum

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from hri_client.hri_client import HRIClient
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray


class State(IntEnum):
    INIT        = 0
    ASK         = 1
    LISTEN      = 2
    DECIDE      = 3
    NAV         = 4
    FOLLOW      = 5
    RANDOM_OUT  = 6
    RANDOM_BACK = 7
    ARRIVED     = 8
    DONE        = 9


MENU_PROMPT = (
    "Dime un número del uno al cuatro. "
    "Uno: ir a la puerta. "
    "Dos: ir al centro. "
    "Tres: buscar a una persona. "
    "Cuatro: ir a un punto y volver."
)

ARRIVAL_PHRASES = [
    "He llegado al destino.",
    "Objetivo alcanzado.",
    "Listo, estoy en el punto indicado.",
]

OPTION_LABELS = {
    1: 'la puerta',
    2: 'el centro',
    3: 'buscar una persona',
    4: 'un paseo',
}

WORD_TO_OPTION = {
    'uno': 1, 'una': 1, 'puerta': 1, 'primero': 1,
    'dos': 2, 'segundo': 2, 'centro': 2,
    'tres': 3, 'persona': 3, 'sigue': 3, 'sígueme': 3, 'tercero': 3,
    'cuatro': 4, 'aleatorio': 4, 'random': 4, 'cuarto': 4, 'paseo': 4,
}

POST_TTS_BUFFER_SEC = 1.5  # extra espera tras is_speaking_done() para VM/latencia soundplay


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('detections_topic', '/detections_3d')
        self.declare_parameter('person_class', 'person')
        self.declare_parameter('person_hold_distance', 1.5)
        self.declare_parameter('person_hold_time', 2.0)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('post_goal_wait_sec', 2.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('random_xmin', -3.0)
        self.declare_parameter('random_xmax', 3.0)
        self.declare_parameter('random_ymin', -3.0)
        self.declare_parameter('random_ymax', 3.0)

        self.hold_distance = float(self.get_parameter('person_hold_distance').value)
        self.hold_time      = float(self.get_parameter('person_hold_time').value)
        self.goal_timeout   = float(self.get_parameter('goal_timeout_sec').value)
        self.post_goal_wait = float(self.get_parameter('post_goal_wait_sec').value)
        self.base_frame     = self.get_parameter('base_frame').value
        self.map_frame      = self.get_parameter('map_frame').value
        self.person_class   = self.get_parameter('person_class').value

        # HRI client (simple_hri stack: /tts_service + /stt_service)
        self.hri = HRIClient(self)

        # Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # YOLO detections — sensor_data QoS para casar con yolo_depth_node
        self.create_subscription(
            Detection3DArray,
            self.get_parameter('detections_topic').value,
            self._detections_cb,
            qos_profile_sensor_data,
        )

        # Debug state publisher
        self.state_pub = self.create_publisher(String, '~/state', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Waypoints
        self.waypoints = {}
        self._load_waypoints(self.get_parameter('waypoints_file').value)

        # FSM
        self.state    = State.INIT
        self.state_ts = self.get_clock().now()

        # Nav flags
        self._nav_goal_handle = None
        self._nav_active      = False
        self._nav_done        = False
        self._nav_success     = False

        # YOLO
        self._last_detection  = None
        self._close_since     = None

        # Estado general
        self._chosen_option     = 0
        self._return_pose       = None
        self._post_goal_ts      = None
        self._stt_text          = ''

        # Flags de un solo disparo por estado
        self._ask_started       = False
        self._ask_done_ts       = None   # para buffer post-TTS antes de STT
        self._listen_started    = False
        self._arrived_announced = False
        self._arrived_done_ts   = None   # para buffer post-TTS en ARRIVED

        # Follow: timestamps independientes del state_ts
        self._follow_entered_ts   = None
        self._follow_last_send_ts = 0.0

        self.create_timer(0.1, self._control_cycle)
        self.get_logger().info('MissionManager iniciado.')

    # =========================================================
    #   Waypoints
    # =========================================================
    def _load_waypoints(self, filename: str):
        try:
            pkg = get_package_share_directory('pfinal_nav2_hri_manager')
            path = os.path.join(pkg, 'config', filename)
        except Exception:
            path = filename
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            self.waypoints = dict(data.get('waypoints', {}))
            self.get_logger().info(f'Waypoints: {list(self.waypoints.keys())}')
        except Exception as e:
            self.get_logger().error(f'No se pudo cargar waypoints: {e}')
            self.waypoints = {
                'home':     {'x': 0.0,  'y': 0.0,  'theta': 0.0},
                'papelera': {'x': 2.5,  'y': 1.0,  'theta': 0.0},
                'punto_a':  {'x': -1.5, 'y': 2.0,  'theta': 1.5707},
                'punto_b':  {'x': 3.0,  'y': -1.5, 'theta': 3.1415},
            }

    # =========================================================
    #   YOLO
    # =========================================================
    def _detections_cb(self, msg: Detection3DArray):
        for det in msg.detections:
            if not det.results:
                continue
            if det.results[0].hypothesis.class_id != self.person_class:
                continue
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            self._last_detection = {
                'stamp':    msg.header.stamp,
                'frame_id': det.header.frame_id or msg.header.frame_id,
                'x': x, 'y': y, 'z': z,
                'dist': math.sqrt(x*x + y*y + z*z),
            }
            return

    def _person_seen_recently(self, max_age: float = 2.0) -> bool:
        det = self._last_detection
        if det is None:
            return False
        now   = self.get_clock().now().nanoseconds / 1e9
        stamp = det['stamp'].sec + det['stamp'].nanosec / 1e9
        return (now - stamp) <= max_age

    # =========================================================
    #   FSM
    # =========================================================
    def _control_cycle(self):
        msg = String()
        msg.data = State(self.state).name
        self.state_pub.publish(msg)

        {
            State.INIT:        self._h_init,
            State.ASK:         self._h_ask,
            State.LISTEN:      self._h_listen,
            State.DECIDE:      self._h_decide,
            State.NAV:         self._h_nav,
            State.FOLLOW:      self._h_follow,
            State.RANDOM_OUT:  self._h_random_out,
            State.RANDOM_BACK: self._h_random_back,
            State.ARRIVED:     self._h_arrived,
            State.DONE:        self._h_done,
        }[self.state]()

    def _go(self, new_state: State):
        self.get_logger().info(
            f'FSM: {State(self.state).name} -> {State(new_state).name}'
        )
        self.state    = new_state
        self.state_ts = self.get_clock().now()

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_ts).nanoseconds / 1e9

    # ---- INIT ----
    def _h_init(self):
        if self._elapsed() > 1.0:
            if not self.hri.wait_for_services(timeout_sec=10.0):
                self.get_logger().warn('Servicios HRI no disponibles, reintentando...')
                self.state_ts = self.get_clock().now()
                return
            self._go(State.ASK)

    # ---- ASK ----
    def _h_ask(self):
        if not self._ask_started:
            self.hri.start_speaking(MENU_PROMPT)
            self._ask_started = True
            self._ask_done_ts = None
        elif self.hri.is_speaking_done():
            if self._ask_done_ts is None:
                self._ask_done_ts = self.get_clock().now()
            elif (self.get_clock().now() - self._ask_done_ts).nanoseconds / 1e9 >= POST_TTS_BUFFER_SEC:
                self._ask_started = False
                self._ask_done_ts = None
                self._go(State.LISTEN)

    # ---- LISTEN ----
    def _h_listen(self):
        if not self._listen_started:
            self.hri.start_listen()
            self._listen_started = True
        elif self.hri.is_listen_done():
            self._stt_text = self.hri.get_listened_text()
            self._listen_started = False
            self._go(State.DECIDE)

    # ---- DECIDE ----
    def _h_decide(self):
        text = (self._stt_text or '').lower().strip()
        opt  = self._parse_option(text)

        if opt is None:
            opt = random.randint(1, 4)
            self.hri.start_speaking(f'No te entendí. Voy a {OPTION_LABELS[opt]}.')
            self.get_logger().warn(f'STT="{text}" no entendido -> aleatoria: {opt}')
        else:
            self.get_logger().info(f'STT="{text}" -> opción {opt}')

        self._chosen_option = opt
        if opt == 1:
            self._send_waypoint_goal('puerta')
            self._go(State.NAV)
        elif opt == 2:
            self._send_waypoint_goal('centro')
            self._go(State.NAV)
        elif opt == 3:
            self._close_since         = None
            self._follow_entered_ts   = None
            self._follow_last_send_ts = 0.0
            self._go(State.FOLLOW)
        else:
            self._return_pose = self._get_current_pose()
            self._send_waypoint_goal('punto_b')
            self._go(State.RANDOM_OUT)

    def _parse_option(self, text: str):
        for ch in text:
            if ch in '1234':
                return int(ch)
        for word, opt in WORD_TO_OPTION.items():
            if word in text:
                return opt
        return None

    # ---- NAV ----
    def _h_nav(self):
        if not self._nav_done:
            if self._elapsed() > self.goal_timeout:
                self.get_logger().warn('Timeout Nav2')
                self._cancel_nav()
                self._nav_done = True
            return
        self._go(State.ARRIVED)

    # ---- FOLLOW ----
    def _h_follow(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._follow_entered_ts is None:
            self._follow_entered_ts = now

        if not self._person_seen_recently(4.0):
            if (now - self._follow_entered_ts) > 20.0:
                self.hri.start_speaking('No veo a nadie. Vuelvo al menú.')
                self._cancel_nav()
                self._follow_entered_ts = None
                self._go(State.ARRIVED)
            return

        det  = self._last_detection
        dist = det['dist']

        if dist <= self.hold_distance:
            if self._close_since is None:
                self._close_since = now
            elif (now - self._close_since) >= self.hold_time:
                self.get_logger().info(
                    f'Persona a {dist:.2f} m durante {self.hold_time:.1f} s -> OK'
                )
                self._cancel_nav()
                self._follow_entered_ts = None
                self._go(State.ARRIVED)
                return
        else:
            self._close_since = None

        if (now - self._follow_last_send_ts) < 1.0 and self._nav_active:
            return
        pose = self._person_goal_pose(det)
        if pose:
            self._send_pose_goal(pose, label='persona')
            self._follow_last_send_ts = now

    # ---- RANDOM_OUT (opción 4: ir a punto_b) ----
    def _h_random_out(self):
        if not self._nav_done:
            if self._elapsed() > self.goal_timeout:
                self.get_logger().warn('Timeout punto_b')
                self._cancel_nav()
                self._nav_done = True
            return
        if self._return_pose:
            self._send_pose_goal(self._return_pose, label='regreso')
        else:
            self._send_waypoint_goal('home')
        self._go(State.RANDOM_BACK)

    # ---- RANDOM_BACK (opción 4: volver) ----
    def _h_random_back(self):
        if not self._nav_done:
            if self._elapsed() > self.goal_timeout:
                self.get_logger().warn('Timeout regreso')
                self._cancel_nav()
                self._nav_done = True
            return
        self._go(State.ARRIVED)

    # ---- ARRIVED ----
    def _h_arrived(self):
        if not self._arrived_announced:
            phrase = random.choice(ARRIVAL_PHRASES)
            if self._person_seen_recently(2.0):
                phrase += ' Veo a una persona delante.'
            self.hri.start_speaking(phrase)
            self._arrived_announced = True
            self._arrived_done_ts = None
            return

        if not self.hri.is_speaking_done():
            return

        # Buffer post-TTS + espera mínima post_goal_wait desde que TTS terminó
        if self._arrived_done_ts is None:
            self._arrived_done_ts = self.get_clock().now()

        elapsed = (self.get_clock().now() - self._arrived_done_ts).nanoseconds / 1e9
        if elapsed >= max(POST_TTS_BUFFER_SEC, self.post_goal_wait):
            self._arrived_announced = False
            self._arrived_done_ts   = None
            self._post_goal_ts      = None
            self._go(State.DONE)

    # ---- DONE ----
    def _h_done(self):
        self._go(State.ASK)

    # =========================================================
    #   Nav2
    # =========================================================
    def _send_waypoint_goal(self, name: str):
        if name not in self.waypoints:
            self.get_logger().error(f'Waypoint desconocido: {name}')
            self._nav_done = True
            return
        c = self.waypoints[name]
        self._send_xy_goal(
            float(c['x']), float(c['y']), float(c.get('theta', 0.0)), label=name
        )

    def _send_xy_goal(self, x, y, theta, label):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        self._send_pose_goal(pose, label)

    def _send_pose_goal(self, pose: PoseStamped, label: str):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 no disponible')
            self._nav_done = True
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_done    = False
        self._nav_success = False
        self._nav_active  = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_resp_cb)
        self.get_logger().info(
            f'Nav2: "{label}" -> ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

    def _nav_resp_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2: goal rechazado')
            self._nav_done   = True
            self._nav_active = False
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        self._nav_success = (result.status == GoalStatus.STATUS_SUCCEEDED)
        self._nav_done    = True
        self._nav_active  = False
        self.get_logger().info(f'Nav2: {"OK" if self._nav_success else "FAIL"}')

    def _cancel_nav(self):
        if self._nav_goal_handle and self._nav_active:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_active = False

    # =========================================================
    #   TF / Pose helpers
    # =========================================================
    def _get_current_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            p = PoseStamped()
            p.header.frame_id = self.map_frame
            p.header.stamp    = self.get_clock().now().to_msg()
            p.pose.position.x = tr.transform.translation.x
            p.pose.position.y = tr.transform.translation.y
            p.pose.orientation = tr.transform.rotation
            return p
        except Exception as e:
            self.get_logger().warn(f'TF actual: {e}')
            return None

    def _person_goal_pose(self, det) -> PoseStamped:
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, det['frame_id'], rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF persona: {e}')
            return None

        q = tr.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        cy, sy = math.cos(yaw), math.sin(yaw)
        px = tr.transform.translation.x + det['x'] * cy - det['y'] * sy
        py = tr.transform.translation.y + det['x'] * sy + det['y'] * cy

        robot = self._get_current_pose()
        if robot is None:
            return None
        rx, ry = robot.pose.position.x, robot.pose.position.y

        dx, dy = px - rx, py - ry
        d = math.sqrt(dx*dx + dy*dy)
        if d < 0.05:
            return robot
        ux, uy = dx / d, dy / d
        target_d = max(0.0, d - self.hold_distance)
        theta = math.atan2(dy, dx)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = rx + ux * target_d
        pose.pose.position.y = ry + uy * target_d
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
