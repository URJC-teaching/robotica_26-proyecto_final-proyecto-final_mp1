#!/usr/bin/env python3

"""
mission_coordinator.py: Nodo orquestador principal del sistema de navegación y HRI.

Coordina:
  - PersonTracker: detecciones de YOLO
  - GoalManager: objetivos nombrados
  - InteractionManager: máquina de voz
  - ControlCycle: lógica de transiciones y modos
  - NavigationClient: servidor Nav2
  - HRIClient: servicios de voz

El nodo se lanza desde el archivo .launch.py del paquete.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from yolo_msgs.msg import DetectionArray

from .control_cycle import ControlCycle
from .goal_manager import GoalManager
from hri_client.hri_client import HRIClient
from .interaction_manager import InteractionManager
from navigation_client.navigation_client import NavigationClient
from .person_tracker import PersonTracker


class MissionCoordinator(Node):
    """
    Nodo principal que orquesta la misión del robot.
    Inicializa todos los módulos y coordina su ejecución.
    """

    def __init__(self) -> None:
        super().__init__('mission_coordinator')

        # 1. Declarar y cargar parámetros
        self._declare_parameters()
        self._load_parameters()

        # 2. Inicializar clientes de servicios
        self.hri = HRIClient(self)
        self.nav = NavigationClient(self)

        # 3. Configurar transformaciones (TF2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4. Inicializar módulos especializados
        self.person_tracker = PersonTracker(
            person_class_name=self.person_class_name,
            min_detection_score=self.min_detection_score,
        )

        self.goal_manager = GoalManager(logger=self.get_logger())
        self.goal_manager.load_goals_from_file(self.named_goals_file)

        self.interaction = InteractionManager(
            hri_client=self.hri,
            command_processor=self._process_command_wrapper,
            logger=self.get_logger(),
        )

        self.control = ControlCycle(
            person_tracker=self.person_tracker,
            goal_manager=self.goal_manager,
            interaction_manager=self.interaction,
            nav_client=self.nav,
            logger=self.get_logger(),
            get_clock_now=lambda: self.get_clock().now(),
        )

        # Copiar parámetros de seguimiento al control_cycle
        self.control.desired_follow_distance = self.desired_follow_distance
        self.control.follow_distance_tolerance = self.follow_distance_tolerance
        self.control.follow_update_period = self.follow_update_period
        self.control.follow_max_step = self.follow_max_step
        self.control.lost_person_timeout_sec = self.lost_person_timeout_sec
        self.control.nav_goal_timeout_sec = self.nav_goal_timeout_sec

        # 5. Suscribirse a detecciones YOLO
        self.create_subscription(
            DetectionArray,
            self.detection_topic,
            self._detections_cb,
            10,
        )

        # 6. Variables de control de dependencias
        self._hri_ready = False
        self._nav_ready = False
        self._dependencies_ready = False
        self._next_dependency_check = self.get_clock().now()
        self._last_tf_warn_time = self.get_clock().now()

        # 7. Iniciar bucle de control principal (5 Hz -> 0.2 s)
        self.create_timer(0.2, self._control_loop)

        self.get_logger().info('Mission Coordinator iniciado')
        self.get_logger().info(f'Escuchando detecciones en: {self.detection_topic}')
        self.get_logger().info(f'Objetivos cargados: {len(self.goal_manager.get_all_goal_names())}')


    def _declare_parameters(self) -> None:
        """Declara parámetros del nodo."""
        self.declare_parameter('desired_follow_distance', 1.5)
        self.declare_parameter('follow_distance_tolerance', 0.2)
        self.declare_parameter('follow_update_period', 1.0)
        self.declare_parameter('follow_max_step', 1.0)
        self.declare_parameter('lost_person_timeout_sec', 2.5)
        self.declare_parameter('min_detection_score', 0.5)
        self.declare_parameter('person_class_name', 'person')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('detection_topic', '/yolo/detections_3d')
        self.declare_parameter('nav_goal_timeout_sec', 180.0)
        self.declare_parameter('named_goals_file', '')
        self.declare_parameter('startup_prompt', 'Sistema listo. Dime una instrucción.')
        self.declare_parameter('command_prompt', 'Di: seguir persona, ir a un punto, o ir aleatorio.')

    def _load_parameters(self) -> None:
        """Carga parámetros declarados."""
        self.desired_follow_distance = float(self.get_parameter('desired_follow_distance').value)
        self.follow_distance_tolerance = float(self.get_parameter('follow_distance_tolerance').value)
        self.follow_update_period = float(self.get_parameter('follow_update_period').value)
        self.follow_max_step = float(self.get_parameter('follow_max_step').value)
        self.lost_person_timeout_sec = float(self.get_parameter('lost_person_timeout_sec').value)
        self.min_detection_score = float(self.get_parameter('min_detection_score').value)
        self.person_class_name = str(self.get_parameter('person_class_name').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.detection_topic = str(self.get_parameter('detection_topic').value)
        self.nav_goal_timeout_sec = float(self.get_parameter('nav_goal_timeout_sec').value)
        self.named_goals_file = str(self.get_parameter('named_goals_file').value)
        self.startup_prompt = str(self.get_parameter('startup_prompt').value)
        self.command_prompt = str(self.get_parameter('command_prompt').value)

        # Pasar parámetros a navigation_client
        self.nav.set_frames(self.base_frame, self.map_frame)
        self.nav.tf_buffer = self.tf_buffer


    def _control_loop(self) -> None:
        """Bucle principal de control (ejecutado periódicamente)."""
        now = self.get_clock().now()

        # Verificar disponibilidad de dependencias
        self._check_dependencies(now)

        if not self._dependencies_ready:
            return

        # Ejecutar ciclo de control
        self.control.update(now)

    def _check_dependencies(self, now: Time) -> None:
        """Verifica si todos los servicios necesarios están listos."""
        if now < self._next_dependency_check:
            return

        if not self._hri_ready:
            stt = self.hri._stt_client.service_is_ready()
            tts = self.hri._tts_client.service_is_ready()
            ext = self.hri._extract_client.service_is_ready()
            yn = self.hri._yesno_client.service_is_ready()

            if stt and tts and ext and yn:
                self._hri_ready = True
            else:
                self.get_logger().info('Esperando servicios HRI...', throttle_duration_sec=5.0)

        if not self._nav_ready:
            if self.nav.nav_client_.server_is_ready():
                self._nav_ready = True
            else:
                self.get_logger().info('Esperando servidor Nav2...', throttle_duration_sec=5.0)

        self._next_dependency_check = now + Duration(seconds=2.0)

        if self._hri_ready and self._nav_ready and not self._dependencies_ready:
            self._dependencies_ready = True
            self.get_logger().info('Servicios disponibles. Sistema listo.')
            self.interaction.enqueue_speech(self.startup_prompt, listen_after=True)

    def _process_command_wrapper(self, text: str) -> None:
        """
        Wrapper que redirige comandos al control_cycle.
        Esto permite que InteractionManager no tenga dependencia fuerte.

        Args:
            text: Comando de voz recibido
        """
        self.control.process_command(text)

    def _detections_cb(self, msg: DetectionArray) -> None:
        """
        Callback de detecciones YOLO.

        Args:
            msg: Mensaje con detecciones 3D
        """
        self.person_tracker.process_detections(msg)

        # Actualizar TF warning timestamp
        if self.person_tracker.has_people():
            try:
                self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                )
            except TransformException:
                now = self.get_clock().now()
                if now - self._last_tf_warn_time > Duration(seconds=2.0):
                    self.get_logger().warn(
                        f'TF no disponible: {self.map_frame} -> {self.base_frame}'
                    )
                    self._last_tf_warn_time = now


def main(args=None) -> None:
    """Función principal para lanzar el nodo."""
    rclpy.init(args=args)
    node = MissionCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

