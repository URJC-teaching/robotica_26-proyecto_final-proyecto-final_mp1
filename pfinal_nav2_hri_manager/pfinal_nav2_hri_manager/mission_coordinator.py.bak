#!/usr/bin/env python3

import math
import random
import re
import unicodedata
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml
from yolo_msgs.msg import DetectionArray

from hri_client.hri_client import HRIClient
from navigation_client.navigation_client import NavigationClient


class RobotMode(Enum):
    WAITING_COMMAND = auto()
    FOLLOW_PERSON = auto()
    NAVIGATING_TO_GOAL = auto()


class InteractionState(Enum):
    IDLE = auto()
    SPEAKING = auto()
    LISTENING = auto()


@dataclass
class PersonDetection:
    track_id: str
    score: float
    x: float
    y: float
    z: float


class MissionCoordinator(Node):
    """
    Nodo principal que orquesta la mision: Gestiona el estado del robot (Seguir, Navegar, Esperar)
    e interactua con los clientes de HRI (Voz) y Navegacion.
    """
    def __init__(self) -> None:
        super().__init__('mission_coordinator')

        # 1. Configuracion inicial y carga de parametros
        self._declare_parameters()
        self._load_parameters()

        # 2. Inicializacion de clientes (Abstracciones de acciones y servicios)
        self.hri = HRIClient(self)
        self.nav = NavigationClient(self)

        # 3. Configuracion de transformaciones (TF2) para localizacion
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4. Suscripcion a las detecciones de personas (YOLO 3D)
        self.create_subscription(
            DetectionArray,
            self.detection_topic,
            self._detections_cb,
            10,
        )

        # Variables de estado interno para detecciones y seguimiento
        self._latest_people: List[PersonDetection] = []
        now = self.get_clock().now()
        self._last_detections_time = now
        self._last_person_seen_time = now
        self._last_follow_goal_time = now

        # Estado de la maquina de estados (FSM) e interaccion
        self._mode = RobotMode.WAITING_COMMAND
        self._interaction_state = InteractionState.IDLE
        self._speech_queue: List[Tuple[str, bool]] = []
        self._listen_after_speech = False
        self._waiting_prompt_active = False

        self._target_track_id = ''
        self._active_goal_label = ''
        self._navigation_deadline: Optional[Time] = None

        # Control de disponibilidad de servicios externos
        self._hri_ready = False
        self._nav_ready = False
        self._dependencies_ready = False
        self._next_dependency_check = now

        # Inicializa sin restar para evitar tiempo negativo en sim
        self._last_tf_warn_time = now

        # Bucle de control principal (5 Hz)
        self.create_timer(0.2, self._control_cycle)

        self.get_logger().info('Mission coordinator iniciado')
        self.get_logger().info(f'Escuchando detecciones en: {self.detection_topic}')
        self.get_logger().info(f'Objetivos nombrados cargados: {len(self._named_goals)}')

    def _declare_parameters(self) -> None:
        """Define los parametros del nodo con sus valores por defecto."""
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
        self.declare_parameter('random_goal_names', ['puerta_a', 'puerta_b', 'puerta_c'])
        self.declare_parameter('startup_prompt', 'Sistema listo. Dime una instruccion.')
        self.declare_parameter(
            'command_prompt',
            'Di: seguir persona, ir a un punto, o ir aleatorio.',
        )

    def _load_parameters(self) -> None:
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
        self.random_goal_names = [
            str(name) for name in self.get_parameter('random_goal_names').value
        ]
        self.startup_prompt = str(self.get_parameter('startup_prompt').value)
        self.command_prompt = str(self.get_parameter('command_prompt').value)

        self._named_goals: Dict[str, Tuple[float, float, float]] = {}
        self._goal_display_names: Dict[str, str] = {}
        self._goal_alias_to_name: Dict[str, str] = {}
        self._load_named_goals_from_file(self.named_goals_file)

    def _control_cycle(self) -> None:
        """Bucle principal que se ejecuta periodicamente."""
        now = self.get_clock().now()

        # Verifica si los servicios de voz y nav2 estan listos
        self._check_dependencies(now)
        # Gestiona la cola de voz (hablar/escuchar)
        self._process_interaction()

        if not self._dependencies_ready:
            return

        # Logica segun el modo de mision activo
        if self._mode == RobotMode.FOLLOW_PERSON:
            self._update_follow_mode(now)
        elif self._mode == RobotMode.NAVIGATING_TO_GOAL:
            self._update_navigation_mode(now)

    def _check_dependencies(self, now: Time) -> None:
        """Verifica la conexion con los servidores de acciones y servicios."""
        if now < self._next_dependency_check:
            return

        if not self._hri_ready:
            # Comprobacion no bloqueante para evitar spam de logs de error
            stt = self.hri._stt_client.service_is_ready()
            tts = self.hri._tts_client.service_is_ready()
            ext = self.hri._extract_client.service_is_ready()
            yn = self.hri._yesno_client.service_is_ready()
            
            if stt and tts and ext and yn:
                self._hri_ready = True
            else:
                self.get_logger().info('Esperando a que los servicios HRI se inicien...', throttle_duration_sec=5.0)

        if not self._nav_ready:
            # Comprobacion no bloqueante del servidor de accion
            if self.nav.nav_client_.server_is_ready():
                self._nav_ready = True
            else:
                self.get_logger().info('Esperando al servidor de Nav2...', throttle_duration_sec=5.0)

        self._next_dependency_check = now + Duration(seconds=2.0)

        if self._hri_ready and self._nav_ready and not self._dependencies_ready:
            self._dependencies_ready = True
            self.get_logger().info('Servicios HRI y servidor Nav2 disponibles')
            self._enqueue_speech(self.startup_prompt, listen_after=True)

    def _process_interaction(self) -> None:
        """
        Gestiona la maquina de estados de la interaccion (Hablar -> Escuchar).
        Asegura que el robot no intente escuchar mientras esta hablando.
        """
        if self._interaction_state == InteractionState.SPEAKING:
            if self.hri.is_speaking_done():
                self._interaction_state = InteractionState.IDLE
                if self._listen_after_speech:
                    self.hri.start_listen()
                    self._interaction_state = InteractionState.LISTENING
                    self._listen_after_speech = False
            return

        if self._interaction_state == InteractionState.LISTENING:
            if self.hri.is_listen_done():
                self._interaction_state = InteractionState.IDLE
                self._waiting_prompt_active = False

                text = self.hri.get_listened_text().strip()
                if not text:
                    text = self.hri.get_last_listened_text().strip()

                self.get_logger().info(f'Comando recibido: {text}')
                self._process_command(text)
            return

        # Si hay algo en la cola de mensajes, empieza a hablar
        if self._speech_queue:
            speech_text, listen_after = self._speech_queue.pop(0)
            self.hri.start_speaking(speech_text)
            self._interaction_state = InteractionState.SPEAKING
            self._listen_after_speech = listen_after

    def _enqueue_speech(self, text: str, listen_after: bool = False) -> None:
        """Añade un mensaje a la cola de reproduccion de voz."""
        if not text:
            return
        self._speech_queue.append((text, listen_after))
        if listen_after:
            self._waiting_prompt_active = True

    def _process_command(self, text: str) -> None:
        """
        Analiza el texto recibido por voz para determinar la accion a tomar.
        Soporta: seguir persona, ir a puntos prefijados, coordenadas x/y y cancelaciones.
        """
        normalized = self._normalize_text(text)

        if not normalized:
            self._enter_waiting_mode('No te he entendido.')
            return

        # Comando de CANCELACION
        if any(word in normalized for word in ['parar', 'detente', 'cancela', 'cancelar']):
            self._cancel_navigation()
            self._enter_waiting_mode('Orden cancelada.')
            return

        # Comando SEGUIR PERSONA
        if ('seguir' in normalized and 'persona' in normalized) or normalized.startswith('seguir'):
            self._start_follow_person()
            return

        # Comando IR A PUNTO AZAR
        if 'aleatorio' in normalized or 'azar' in normalized:
            self._start_random_goal()
            return

        # Comando COORDENADAS EXPLICITAS (ej: "ve a x 1 y 2")
        coords = self._extract_coordinates(normalized)
        if coords is not None:
            self._start_coordinate_goal(coords[0], coords[1], coords[2])
            return

        # Comando NOMBRE DE PUERTA/LUGAR (ej: "ve a la puerta a")
        named_goal = self._extract_named_goal(normalized)
        if named_goal is not None:
            self._start_named_goal(named_goal)
            return

        # Comando de AYUDA
        if any(word in normalized for word in ['ayuda', 'opciones', 'menu', 'instruccion', 'instrucciones']):
            self._enqueue_speech(
                'Puedes decir seguir persona, ir a puerta A, ir a coordenadas x e y, o ir aleatorio.',
                listen_after=True,
            )
            return

        self._enter_waiting_mode('No reconozco la instruccion.')

    def _start_follow_person(self) -> None:
        person = self._select_person('')
        if person is None:
            self._enter_waiting_mode('No veo ninguna persona para seguir.')
            return

        self._mode = RobotMode.FOLLOW_PERSON
        self._target_track_id = person.track_id
        self._last_person_seen_time = self.get_clock().now()
        self._last_follow_goal_time = self.get_clock().now() - Duration(
            seconds=self.follow_update_period
        )

        target_text = 'la persona detectada'
        if person.track_id:
            target_text = f'la persona con ID {person.track_id}'
        self._enqueue_speech(f'Comienzo a seguir a {target_text}.')

    def _start_named_goal(self, canonical_name: str) -> None:
        if canonical_name not in self._named_goals:
            self._enter_waiting_mode('Ese punto no esta definido.')
            return

        x, y, yaw = self._named_goals[canonical_name]
        pose = self.nav.create_pose_stamped(x, y, yaw)
        label = self._goal_display_names.get(canonical_name, canonical_name)
        self._start_navigation_goal(pose, label)

    def _start_random_goal(self) -> None:
        candidates: List[str] = []

        if self.random_goal_names:
            for goal_name in self.random_goal_names:
                canonical_name = self._canonical_goal_name(goal_name)
                if canonical_name in self._named_goals:
                    candidates.append(canonical_name)

        if not candidates:
            candidates = list(self._named_goals.keys())

        if not candidates:
            self._enter_waiting_mode('No hay puntos configurados para modo aleatorio.')
            return

        selected = random.choice(candidates)
        self._start_named_goal(selected)

    def _start_coordinate_goal(self, x: float, y: float, yaw: float) -> None:
        pose = self.nav.create_pose_stamped(x, y, yaw)
        label = f'coordenada ({x:.2f}, {y:.2f})'
        self._start_navigation_goal(pose, label)

    def _start_navigation_goal(self, pose, label: str) -> None:
        self._cancel_navigation()

        self.nav.send_goal(pose)
        self._mode = RobotMode.NAVIGATING_TO_GOAL
        self._active_goal_label = label
        self._navigation_deadline = self.get_clock().now() + Duration(
            seconds=self.nav_goal_timeout_sec
        )

        self._enqueue_speech(f'Voy hacia {label}.')

    def _update_follow_mode(self, now: Time) -> None:
        """Logica para mantener la distancia de 1.5m respecto a la persona."""
        person = self._select_person(self._target_track_id)

        if person is None:
            # Gestion de perdida de la persona (timeout)
            if now - self._last_person_seen_time > Duration(
                seconds=self.lost_person_timeout_sec
            ):
                self._cancel_navigation()
                self._enter_waiting_mode('He perdido a la persona.')
            return

        self._last_person_seen_time = now

        # Calculo de distancia actual basada en 3D (x, y)
        distance = math.hypot(person.x, person.y)

        # Si ya estamos a la distancia deseada (con tolerancia), nos paramos
        if abs(distance - self.desired_follow_distance) <= self.follow_distance_tolerance:
            self._cancel_navigation()
            self._enter_waiting_mode('Distancia objetivo alcanzada.')
            return

        # Solo enviamos nuevos objetivos a Nav2 cada X segundos para no saturar
        if now - self._last_follow_goal_time < Duration(seconds=self.follow_update_period):
            return

        # Calculamos donde deberia estar el robot para estar a 1.5m
        relative_goal = self._compute_relative_goal(person.x, person.y)
        if relative_goal is None:
            return

        # Transformamos esa posicion relativa a coordenadas globales (mapa)
        map_goal = self._relative_goal_to_map(relative_goal, person)
        if map_goal is None:
            return

        # Cancelamos el anterior y enviamos el nuevo objetivo
        if self.nav.is_goal_active():
            self.nav.cancel_goal()

        self.nav.send_goal(map_goal)
        self._last_follow_goal_time = now

    def _update_navigation_mode(self, now: Time) -> None:
        if self.nav.is_goal_done():
            label = self._active_goal_label or 'el objetivo'
            if self.nav.was_goal_successful():
                self._enter_waiting_mode(f'He llegado a {label}.')
            else:
                self._enter_waiting_mode(f'No he podido llegar a {label}.')
            return

        if self._navigation_deadline is not None and now > self._navigation_deadline:
            self._cancel_navigation()
            self._enter_waiting_mode('Tiempo de navegacion agotado.')

    def _cancel_navigation(self) -> None:
        if self.nav.is_goal_active():
            self.nav.cancel_goal()

        self._active_goal_label = ''
        self._navigation_deadline = None

    def _enter_waiting_mode(self, message: Optional[str] = None) -> None:
        self._mode = RobotMode.WAITING_COMMAND
        self._target_track_id = ''

        if message:
            self._enqueue_speech(message)

        self._enqueue_speech(self.command_prompt, listen_after=True)

    def _compute_relative_goal(
        self,
        person_x: float,
        person_y: float,
    ) -> Optional[Tuple[float, float]]:
        """
        Calcula el desplazamiento (x, y) relativo al robot necesario para
        quedarse a 1.5m de la persona.
        """
        distance = math.hypot(person_x, person_y)
        if distance < 1e-3:
            return None

        # Cuanto nos falta para los 1.5m (positivo = acercarse, negativo = alejarse)
        displacement = distance - self.desired_follow_distance
        if abs(displacement) < 1e-2:
            return None

        # Proyectamos el desplazamiento sobre la linea robot-persona
        scale = displacement / distance
        rel_x = person_x * scale
        rel_y = person_y * scale

        # Limitamos el paso maximo para mayor seguridad/suavidad
        step = math.hypot(rel_x, rel_y)
        if step > self.follow_max_step:
            limiter = self.follow_max_step / step
            rel_x *= limiter
            rel_y *= limiter

        return rel_x, rel_y

    def _relative_goal_to_map(
        self,
        relative_goal: Tuple[float, float],
        person: PersonDetection,
    ):
        """
        Convierte una posicion (relativa al robot) en una posicion global (mapa).
        Utiliza TF2 para obtener la posicion actual del robot.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            now = self.get_clock().now()
            if now - self._last_tf_warn_time > Duration(seconds=2.0):
                self.get_logger().warn(
                    f'No hay TF {self.map_frame}->{self.base_frame}: {exc}'
                )
                self._last_tf_warn_time = now
            return None

        # Posicion y orientacion actual del robot segun el mapa
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Calculo de las coordenadas del mapa para el objetivo relativo
        rel_x, rel_y = relative_goal
        goal_x = tx + math.cos(yaw) * rel_x - math.sin(yaw) * rel_y
        goal_y = ty + math.sin(yaw) * rel_x + math.cos(yaw) * rel_y

        # Calculo de la posicion de la persona en el mapa para orientar el robot hacia ella
        person_gx = tx + math.cos(yaw) * person.x - math.sin(yaw) * person.y
        person_gy = ty + math.sin(yaw) * person.x + math.cos(yaw) * person.y

        # El robot mirara hacia donde esta la persona al llegar al punto
        goal_yaw = math.atan2(person_gy - goal_y, person_gx - goal_x)

        return self.nav.create_pose_stamped(goal_x, goal_y, goal_yaw)

    def _detections_cb(self, msg: DetectionArray) -> None:
        """Callback que procesa las detecciones 3D de YOLO."""
        people: List[PersonDetection] = []

        for detection in msg.detections:
            # Filtramos solo por 'person' y por puntuacion minima
            if detection.class_name.lower() != self.person_class_name.lower():
                continue
            if detection.score < self.min_detection_score:
                continue
            if not detection.bbox3d.frame_id:
                continue

            px = float(detection.bbox3d.center.position.x)
            py = float(detection.bbox3d.center.position.y)
            pz = float(detection.bbox3d.center.position.z)

            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue

            people.append(
                PersonDetection(
                    track_id=detection.id,
                    score=float(detection.score),
                    x=px,
                    y=py,
                    z=pz,
                )
            )

        self._latest_people = people
        self._last_detections_time = self.get_clock().now()

        if people:
            self._last_person_seen_time = self._last_detections_time

    def _select_person(self, preferred_track_id: str) -> Optional[PersonDetection]:
        """Selecciona la persona a seguir (por ID o la mas cercana)."""
        if not self._latest_people:
            return None

        # Si tenemos un ID fijado, lo buscamos
        if preferred_track_id:
            for person in self._latest_people:
                if person.track_id == preferred_track_id:
                    return person

        # Si no, elegimos a la persona mas cercana por defecto
        return min(self._latest_people, key=lambda person: math.hypot(person.x, person.y))

    def _load_named_goals_from_file(self, file_path: str) -> None:
        if not file_path:
            self.get_logger().warn('No se ha definido named_goals_file')
            return

        path = Path(file_path)
        if not path.exists():
            self.get_logger().warn(f'Archivo de puntos no encontrado: {file_path}')
            return

        try:
            with open(path, 'r', encoding='utf-8') as handle:
                yaml_data = yaml.safe_load(handle) or {}
        except Exception as exc:
            self.get_logger().error(f'No se pudo leer {file_path}: {exc}')
            return

        goals_data = yaml_data.get('goals', yaml_data)
        if not isinstance(goals_data, dict):
            self.get_logger().error('El archivo de puntos no tiene formato valido')
            return

        for raw_name, goal_data in goals_data.items():
            if not isinstance(goal_data, dict):
                continue

            try:
                x = float(goal_data['x'])
                y = float(goal_data['y'])
                yaw = float(goal_data.get('yaw', 0.0))
            except (KeyError, ValueError, TypeError):
                self.get_logger().warn(f'Punto omitido por formato invalido: {raw_name}')
                continue

            canonical_name = self._canonical_goal_name(str(raw_name))
            self._named_goals[canonical_name] = (x, y, yaw)
            self._goal_display_names[canonical_name] = str(raw_name)

            for alias in self._build_goal_aliases(str(raw_name)):
                self._goal_alias_to_name[alias] = canonical_name

    def _extract_named_goal(self, command: str) -> Optional[str]:
        for alias, canonical_name in self._goal_alias_to_name.items():
            if alias and alias in command:
                return canonical_name
        return None

    def _extract_coordinates(self, command: str) -> Optional[Tuple[float, float, float]]:
        explicit_xy = re.search(
            r'x\s*[:=]?\s*(-?\d+(?:[\.,]\d+)?)\D+y\s*[:=]?\s*(-?\d+(?:[\.,]\d+)?)',
            command,
        )

        if explicit_xy:
            x = self._to_float(explicit_xy.group(1))
            y = self._to_float(explicit_xy.group(2))
            yaw = 0.0
            yaw_match = re.search(r'yaw\s*[:=]?\s*(-?\d+(?:[\.,]\d+)?)', command)
            if yaw_match:
                yaw = self._to_float(yaw_match.group(1))
            return x, y, yaw

        if not any(word in command for word in ['ir', 'punto', 'coordenada', 'coordenadas']):
            return None

        values = re.findall(r'-?\d+(?:[\.,]\d+)?', command)
        if len(values) < 2:
            return None

        x = self._to_float(values[0])
        y = self._to_float(values[1])
        yaw = self._to_float(values[2]) if len(values) >= 3 else 0.0
        return x, y, yaw

    def _build_goal_aliases(self, goal_name: str) -> List[str]:
        normalized = self._normalize_text(goal_name)
        normalized_spaced = normalized.replace('_', ' ')
        compact = normalized_spaced.replace(' ', '')

        aliases = [normalized, normalized_spaced, compact]
        aliases = [alias for alias in aliases if alias]

        # remove duplicates while preserving order
        seen = set()
        unique_aliases: List[str] = []
        for alias in aliases:
            if alias not in seen:
                seen.add(alias)
                unique_aliases.append(alias)

        return unique_aliases

    def _canonical_goal_name(self, goal_name: str) -> str:
        return self._normalize_text(goal_name).replace(' ', '_')

    @staticmethod
    def _to_float(raw_value: str) -> float:
        return float(raw_value.replace(',', '.'))

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_text(text: str) -> str:
        lower_text = text.lower().strip()
        no_accents = ''.join(
            char
            for char in unicodedata.normalize('NFD', lower_text)
            if unicodedata.category(char) != 'Mn'
        )
        clean = re.sub(r'[^a-z0-9_\.,\- ]+', ' ', no_accents)
        return re.sub(r'\s+', ' ', clean).strip()


def main(args=None) -> None:
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
