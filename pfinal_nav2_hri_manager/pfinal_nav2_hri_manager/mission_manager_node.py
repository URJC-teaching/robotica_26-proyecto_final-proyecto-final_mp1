# Copyright 2025 - Final Project ROS2 Jazzy
# Nav2 + HRI + YOLO integration
# Orquestador principal con FSM

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from audio_common_msgs.action import TTS
from whisper_msgs.action import STT
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import rclpy.duration

import math
import random
import yaml
import os
from enum import IntEnum
from ament_index_python.packages import get_package_share_directory


class State(IntEnum):
    IDLE          = 0  # Esperando, puede hablar
    WAIT_COMMAND  = 1  # HRI: escuchando al usuario
    NAVIGATE      = 2  # Nav2: yendo a un waypoint
    FOLLOW_PERSON = 3  # Siguiendo persona detectada por YOLO
    DONE_GOAL     = 4  # Goal completado, anuncia y espera 2 s antes de pedir nuevo


# Frases de anuncio al completar goal (variedad para que no repita siempre lo mismo)
ARRIVAL_PHRASES = [
    "He llegado al destino.",
    "Objetivo alcanzado.",
    "Listo, estoy en el punto indicado.",
]

# Lo que dice si no entiende el comando
FALLBACK_PHRASES = [
    "No he entendido bien. Voy a un punto aleatorio.",
    "No te he podido entender. Me movo a un sitio al azar.",
]

LISTEN_PROMPT = "Dime a dónde quieres que vaya. Puedo ir a casa, a la papelera, seguirte, o a un punto aleatorio."


class MissionManagerNode(Node):
    """
    Nodo orquestador principal.
    Gestiona la FSM de misión integrando Nav2, HRI (TTS/STT) y YOLO.

    Waypoints predeterminados se cargan desde config/waypoints.yaml.
    Si el comando HRI no se reconoce, se elige un waypoint aleatorio.
    """

    def __init__(self):
        super().__init__('mission_manager_node')

        # --- Parámetros ---
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('follow_topic', '/yolo/target_pose')
        self.declare_parameter('goal_timeout_sec', 60.0)
        self.declare_parameter('post_goal_wait_sec', 2.0)

        waypoints_file   = self.get_parameter('waypoints_file').value
        follow_topic     = self.get_parameter('follow_topic').value
        self.goal_timeout   = self.get_parameter('goal_timeout_sec').value
        self.post_goal_wait = self.get_parameter('post_goal_wait_sec').value

        # --- Clientes de acción ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tts_client = ActionClient(self, TTS, 'say')
        self.stt_client = ActionClient(self, STT, 'whisper/listen')

        # --- Suscriptor YOLO ---
        # El nodo YOLO publica la pose del objetivo en este topic
        self.yolo_sub = self.create_subscription(
            PoseStamped,
            follow_topic,
            self._yolo_callback,
            10
        )
        self.yolo_pose = None  # Última pose recibida de YOLO

        # --- Publicador de estado (útil para debug/visualización) ---
        self.state_pub = self.create_publisher(String, '~/state', 10)

        # --- Waypoints ---
        self.waypoints = {}
        self._load_waypoints(waypoints_file)

        # --- FSM ---
        self.state = State.IDLE
        self.state_ts = self.get_clock().now()

        # Resultado pendiente de acción
        self._nav_goal_handle = None
        self._nav_done = False
        self._nav_success = False
        self._tts_done = False
        self._stt_done = False
        self._transcribed_text = ''

        # Control del ciclo post-goal
        self._post_goal_ts = None

        # Ciclo de control a 10 Hz
        self.timer = self.create_timer(0.1, self._control_cycle)

        self.get_logger().info('MissionManager listo. Iniciando...')

    # =========================================================
    #   Carga de waypoints
    # =========================================================

    def _load_waypoints(self, filename: str):
        try:
            pkg_share = get_package_share_directory('final_project')
            path = os.path.join(pkg_share, 'config', filename)
        except Exception:
            path = filename  # Path absoluto como fallback

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            for name, coords in data.get('waypoints', {}).items():
                self.waypoints[name] = coords
            self.get_logger().info(f'Waypoints cargados: {list(self.waypoints.keys())}')
        except Exception as e:
            self.get_logger().error(f'No se pudo cargar waypoints: {e}')
            # Waypoints de emergencia hardcoded
            self.waypoints = {
                'home':     {'x': 0.0,  'y': 0.0,  'theta': 0.0},
                'papelera': {'x': 2.0,  'y': 1.0,  'theta': 0.0},
                'punto_a':  {'x': -1.0, 'y': 2.0,  'theta': 1.57},
                'punto_b':  {'x': 3.0,  'y': -1.5, 'theta': 0.0},
            }
            self.get_logger().warn('Usando waypoints de emergencia hardcoded')

    # =========================================================
    #   YOLO callback
    # =========================================================

    def _yolo_callback(self, msg: PoseStamped):
        self.yolo_pose = msg
        self.get_logger().debug(
            f'YOLO: pose recibida x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )

    # =========================================================
    #   FSM principal
    # =========================================================

    def _control_cycle(self):
        # Publica el estado actual para debug
        state_msg = String()
        state_msg.data = State(self.state).name
        self.state_pub.publish(state_msg)

        if self.state == State.IDLE:
            self._handle_idle()

        elif self.state == State.WAIT_COMMAND:
            self._handle_wait_command()

        elif self.state == State.NAVIGATE:
            self._handle_navigate()

        elif self.state == State.FOLLOW_PERSON:
            self._handle_follow()

        elif self.state == State.DONE_GOAL:
            self._handle_done_goal()

    # ----- IDLE -----
    def _handle_idle(self):
        # En IDLE esperamos un instante y pasamos a pedir comando
        elapsed = (self.get_clock().now() - self.state_ts).nanoseconds / 1e9
        if elapsed > 1.0:
            self._transition_to(State.WAIT_COMMAND)

    # ----- WAIT_COMMAND -----
    def _handle_wait_command(self):
        # 1) Primero TTS para decir el prompt
        if not self._tts_done:
            if not hasattr(self, '_tts_started') or not self._tts_started:
                self._tts_started = True
                self._say(LISTEN_PROMPT)
            return  # Esperar a que TTS termine

        # 2) Luego STT para escuchar
        if not self._stt_done:
            if not hasattr(self, '_stt_started') or not self._stt_started:
                self._stt_started = True
                self._listen()
            return  # Esperar transcripción

        # 3) Procesar transcripción
        self._tts_started = False
        self._stt_started = False
        cmd = self._transcribed_text.lower().strip()
        self.get_logger().info(f'Comando recibido: "{cmd}"')

        target = self._parse_command(cmd)

        if target == 'follow':
            self._transition_to(State.FOLLOW_PERSON)
        elif target is not None:
            self._current_waypoint = target
            self._send_nav_goal(target)
            self._transition_to(State.NAVIGATE)
        else:
            # Comando no entendido → fallback aleatorio
            phrase = random.choice(FALLBACK_PHRASES)
            self._say(phrase)
            random_wp = self._pick_random_waypoint()
            self._current_waypoint = random_wp
            self._send_nav_goal(random_wp)
            self._transition_to(State.NAVIGATE)

    # ----- NAVIGATE -----
    def _handle_navigate(self):
        if not self._nav_done:
            # Comprobar timeout
            elapsed = (self.get_clock().now() - self.state_ts).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                self.get_logger().warn('Timeout de navegación, cancelando goal')
                if self._nav_goal_handle:
                    self._nav_goal_handle.cancel_goal_async()
                self._nav_done = True
                self._nav_success = False
            return

        # Goal completado
        if self._nav_success:
            self.get_logger().info(f'Llegado a waypoint: {self._current_waypoint}')
        else:
            self.get_logger().warn(f'Fallo al llegar a: {self._current_waypoint}')

        self._transition_to(State.DONE_GOAL)

    # ----- FOLLOW_PERSON -----
    def _handle_follow(self):
        if self.yolo_pose is None:
            self.get_logger().info_throttle(2.0, 'Esperando pose de YOLO...')  # type: ignore
            return

        # Enviamos continuamente la pose de YOLO como goal de Nav2
        # (Nav2 con el goal reemplaza el anterior automáticamente)
        pose = self.yolo_pose
        self.yolo_pose = None  # Consumir para no re-enviar si no hay nueva

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.nav_client.send_goal_async(goal)
        self.get_logger().debug('Follow: nuevo goal enviado a Nav2')

        # Para salir de FOLLOW, el usuario tendría que decir "para" o "stop"
        # (simplificación: se queda en FOLLOW hasta que no haya YOLO por 5 s)
        self.state_ts = self.get_clock().now()

    # ----- DONE_GOAL -----
    def _handle_done_goal(self):
        if self._post_goal_ts is None:
            phrase = random.choice(ARRIVAL_PHRASES)
            self._say(phrase)
            self._post_goal_ts = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() - self._post_goal_ts).nanoseconds / 1e9
        if elapsed >= self.post_goal_wait:
            self._post_goal_ts = None
            self._transition_to(State.WAIT_COMMAND)

    # =========================================================
    #   Transiciones
    # =========================================================

    def _transition_to(self, new_state: State):
        self.get_logger().info(f'FSM: {State(self.state).name} -> {State(new_state).name}')
        self.state = new_state
        self.state_ts = self.get_clock().now()
        # Reset de flags al entrar en WAIT_COMMAND
        if new_state == State.WAIT_COMMAND:
            self._tts_done = False
            self._stt_done = False
            self._transcribed_text = ''

    # =========================================================
    #   Parseo de comandos HRI
    # =========================================================

    def _parse_command(self, text: str):
        """
        Parseo simple por palabras clave.
        Retorna: nombre de waypoint, 'follow', o None si no reconoce.
        """
        # Follow / seguir
        if any(k in text for k in ['sigue', 'sígueme', 'follow', 'ven conmigo']):
            return 'follow'

        # Waypoints por nombre
        for name in self.waypoints:
            if name.lower() in text:
                return name

        # Palabras clave adicionales mapeadas a waypoints
        keyword_map = {
            'casa':     'home',
            'inicio':   'home',
            'papelera': 'papelera',
            'basura':   'papelera',
            'aleatorio': None,  # Fuerza fallback
            'random':   None,
        }
        for kw, wp in keyword_map.items():
            if kw in text:
                return wp  # None dispara fallback

        return None  # No reconocido

    def _pick_random_waypoint(self) -> str:
        return random.choice(list(self.waypoints.keys()))

    # =========================================================
    #   Nav2
    # =========================================================

    def _send_nav_goal(self, waypoint_name: str):
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'Waypoint desconocido: {waypoint_name}')
            return

        coords = self.waypoints[waypoint_name]
        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(coords['x'])
        pose.pose.position.y = float(coords['y'])
        theta = float(coords.get('theta', 0.0))
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)

        goal.pose = pose
        self._nav_done = False
        self._nav_success = False

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor Nav2 no disponible')
            self._nav_done = True
            return

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response_cb)
        self.get_logger().info(
            f'Nav2: goal enviado hacia "{waypoint_name}" '
            f'({coords["x"]}, {coords["y"]})'
        )

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2: goal rechazado')
            self._nav_done = True
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        from action_msgs.msg import GoalStatus
        result = future.result()
        self._nav_success = (result.status == GoalStatus.STATUS_SUCCEEDED)
        self._nav_done = True
        self.get_logger().info(
            f'Nav2: resultado = {"EXITO" if self._nav_success else "FALLO"}'
        )

    # =========================================================
    #   TTS / STT  (mismo estilo que repeat_node y generate_response)
    # =========================================================

    def _say(self, text: str):
        self._tts_done = False
        if not self.tts_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('TTS no disponible')
            self._tts_done = True
            return
        goal = TTS.Goal()
        goal.text = text
        self.get_logger().info(f'TTS: "{text}"')
        self.tts_client.send_goal_async(goal).add_done_callback(self._tts_response_cb)

    def _tts_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._tts_done = True
            return
        handle.get_result_async().add_done_callback(self._tts_result_cb)

    def _tts_result_cb(self, _future):
        self._tts_done = True

    def _listen(self):
        self._stt_done = False
        if not self.stt_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('STT no disponible')
            self._stt_done = True
            return
        goal = STT.Goal()
        self.get_logger().info('STT: escuchando...')
        self.stt_client.send_goal_async(goal).add_done_callback(self._stt_response_cb)

    def _stt_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._stt_done = True
            return
        handle.get_result_async().add_done_callback(self._stt_result_cb)

    def _stt_result_cb(self, future):
        result = future.result().result
        text = ''
        if hasattr(result, 'transcription') and hasattr(result.transcription, 'text'):
            text = result.transcription.text
        self._transcribed_text = text
        self._stt_done = True
        self.get_logger().info(f'STT: "{text}"')


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
