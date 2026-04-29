"""
control_cycle.py: Máquina de estados principal del robot.
Orquesta los modos WAITING_COMMAND, FOLLOW_PERSON, y NAVIGATING_TO_GOAL.
"""

import math
import random
from enum import Enum, auto
from typing import Optional, Tuple

from rclpy.duration import Duration
from rclpy.time import Time

from .goal_manager import GoalManager
from .interaction_manager import InteractionManager
from navigation_client.navigation_client import NavigationClient
from .person_tracker import PersonDetection, PersonTracker


class RobotMode(Enum):
    """Modos operacionales del robot."""
    WAITING_COMMAND = auto()     # Espera comando por voz
    FOLLOW_PERSON = auto()        # Sigue una persona
    NAVIGATING_TO_GOAL = auto()   # Navega hacia un objetivo


class ControlCycle:
    """
    Implementa la máquina de estados principal del robot.
    Gestiona transiciones entre modos y lógica de cada estado.
    """

    def __init__(
        self,
        person_tracker: PersonTracker,
        goal_manager: GoalManager,
        interaction_manager: InteractionManager,
        nav_client: NavigationClient,
        logger,
        get_clock_now,
    ) -> None:
        """
        Args:
            person_tracker: Gestor de detecciones de personas
            goal_manager: Gestor de objetivos nombrados
            interaction_manager: Gestor de interacción por voz
            nav_client: Cliente de navegación Nav2
            logger: Logger de ROS2
            get_clock_now: Función para obtener el tiempo actual
        """
        self.person_tracker = person_tracker
        self.goal_manager = goal_manager
        self.interaction = interaction_manager
        self.nav = nav_client
        self.logger = logger
        self.get_clock_now = get_clock_now

        # Estado de la máquina
        self._mode = RobotMode.WAITING_COMMAND
        self._target_track_id = ''
        self._active_goal_label = ''
        self._navigation_deadline: Optional[Time] = None

        # Control de seguimiento de personas
        self._last_person_seen_time = get_clock_now()
        self._last_follow_goal_time = get_clock_now()

        # Parámetros de seguimiento (se establecen desde mission_coordinator)
        self.desired_follow_distance = 1.5
        self.follow_distance_tolerance = 0.2
        self.follow_update_period = 1.0
        self.follow_max_step = 1.0
        self.lost_person_timeout_sec = 2.5
        self.nav_goal_timeout_sec = 180.0

    def update(self, now: Time) -> None:
        """
        Ejecuta un ciclo de la máquina de estados.

        Args:
            now: Tiempo actual del sistema
        """
        # Procesa la interacción de voz
        self.interaction.process_interaction()

        # Ejecuta lógica según el modo actual
        if self._mode == RobotMode.FOLLOW_PERSON:
            self._update_follow_mode(now)
        elif self._mode == RobotMode.NAVIGATING_TO_GOAL:
            self._update_navigation_mode(now)

    def process_command(self, text: str) -> None:
        """
        Procesa un comando de voz y realiza la acción correspondiente.

        Args:
            text: Comando de voz recibido
        """
        normalized = self.goal_manager._normalize_text(text)

        if not normalized:
            self._enter_waiting_mode('No te he entendido.')
            return

        # Comando de CANCELACIÓN
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

        # Comando COORDENADAS EXPLÍCITAS (ej: "ve a x 1 y 2")
        coords = self.goal_manager.extract_coordinates(normalized)
        if coords is not None:
            self._start_coordinate_goal(coords[0], coords[1], coords[2])
            return

        # Comando NOMBRE DE PUERTA/LUGAR (ej: "ve a la puerta a")
        named_goal = self.goal_manager.extract_named_goal(normalized)
        if named_goal is not None:
            self._start_named_goal(named_goal)
            return

        # Comando de AYUDA
        if any(word in normalized for word in ['ayuda', 'opciones', 'menu', 'instruccion', 'instrucciones']):
            self.interaction.enqueue_speech(
                'Puedes decir seguir persona, ir a puerta A, ir a coordenadas x e y, o ir aleatorio.',
                listen_after=True,
            )
            return

        self._enter_waiting_mode('No reconozco la instrucción.')

    def _start_follow_person(self) -> None:
        """Inicia modo de seguimiento de personas."""
        person = self.person_tracker.select_person('')
        if person is None:
            self._enter_waiting_mode('No veo ninguna persona para seguir.')
            return

        self._mode = RobotMode.FOLLOW_PERSON
        self._target_track_id = person.track_id
        self._last_person_seen_time = self.get_clock_now()
        self._last_follow_goal_time = self.get_clock_now() - Duration(
            seconds=self.follow_update_period
        )

        target_text = 'la persona detectada'
        if person.track_id:
            target_text = f'la persona con ID {person.track_id}'
        self.interaction.enqueue_speech(f'Comienzo a seguir a {target_text}.')

    def _start_named_goal(self, canonical_name: str) -> None:
        """Inicia navegación hacia un objetivo nombrado."""
        coords = self.goal_manager.get_goal_by_name(canonical_name)
        if coords is None:
            self._enter_waiting_mode('Ese punto no está definido.')
            return

        x, y, yaw = coords
        pose = self.nav.create_pose_stamped(x, y, yaw)
        label = self.goal_manager._goal_display_names.get(canonical_name, canonical_name)
        self._start_navigation_goal(pose, label)

    def _start_random_goal(self) -> None:
        """Inicia navegación hacia un objetivo aleatorio."""
        candidates = self.goal_manager.get_all_goal_names()

        if not candidates:
            self._enter_waiting_mode('No hay puntos configurados para modo aleatorio.')
            return

        selected = random.choice(candidates)
        self._start_named_goal(selected)

    def _start_coordinate_goal(self, x: float, y: float, yaw: float) -> None:
        """Inicia navegación hacia coordenadas específicas."""
        pose = self.nav.create_pose_stamped(x, y, yaw)
        label = f'coordenada ({x:.2f}, {y:.2f})'
        self._start_navigation_goal(pose, label)

    def _start_navigation_goal(self, pose, label: str) -> None:
        """Inicia navegación hacia una pose."""
        self._cancel_navigation()

        self.nav.send_goal(pose)
        self._mode = RobotMode.NAVIGATING_TO_GOAL
        self._active_goal_label = label
        self._navigation_deadline = self.get_clock_now() + Duration(
            seconds=self.nav_goal_timeout_sec
        )

        self.interaction.enqueue_speech(f'Voy hacia {label}.')

    def _update_follow_mode(self, now: Time) -> None:
        """Actualiza lógica del modo seguimiento de personas."""
        person = self.person_tracker.select_person(self._target_track_id)

        if person is None:
            # Gestión de pérdida de persona (timeout)
            if now - self._last_person_seen_time > Duration(
                seconds=self.lost_person_timeout_sec
            ):
                self._cancel_navigation()
                self._enter_waiting_mode('He perdido a la persona.')
            return

        self._last_person_seen_time = now

        # Cálculo de distancia actual
        distance = math.hypot(person.x, person.y)

        # Si estamos a distancia deseada, nos paramos
        if abs(distance - self.desired_follow_distance) <= self.follow_distance_tolerance:
            self._cancel_navigation()
            self._enter_waiting_mode('Distancia objetivo alcanzada.')
            return

        # Solo enviamos nuevos objetivos cada X segundos
        if now - self._last_follow_goal_time < Duration(seconds=self.follow_update_period):
            return

        # Calcula objetivo relativo para mantener distancia
        relative_goal = self._compute_relative_goal(person.x, person.y)
        if relative_goal is None:
            return

        # Transforma a coordenadas globales
        map_goal = self.nav._relative_goal_to_map(relative_goal, person)
        if map_goal is None:
            return

        # Envía nuevo objetivo a Nav2
        if self.nav.is_goal_active():
            self.nav.cancel_goal()

        self.nav.send_goal(map_goal)
        self._last_follow_goal_time = now

    def _update_navigation_mode(self, now: Time) -> None:
        """Actualiza lógica del modo navegación."""
        if self.nav.is_goal_done():
            label = self._active_goal_label or 'el objetivo'
            if self.nav.was_goal_successful():
                self._enter_waiting_mode(f'He llegado a {label}.')
            else:
                self._enter_waiting_mode(f'No he podido llegar a {label}.')
            return

        if self._navigation_deadline is not None and now > self._navigation_deadline:
            self._cancel_navigation()
            self._enter_waiting_mode('Tiempo de navegación agotado.')

    def _cancel_navigation(self) -> None:
        """Cancela la navegación actual."""
        if self.nav.is_goal_active():
            self.nav.cancel_goal()

        self._active_goal_label = ''
        self._navigation_deadline = None

    def _enter_waiting_mode(self, message: Optional[str] = None) -> None:
        """
        Transiciona al modo espera y opcionalmente enuncia un mensaje.

        Args:
            message: Mensaje opcional a reproducir
        """
        self._mode = RobotMode.WAITING_COMMAND
        self._target_track_id = ''

        if message:
            self.interaction.enqueue_speech(message)

        # Que espere por comando
        command_prompt = 'Di: seguir persona, ir a un punto, o ir aleatorio.'
        self.interaction.enqueue_speech(command_prompt, listen_after=True)

    def _compute_relative_goal(
        self,
        person_x: float,
        person_y: float,
    ) -> Optional[Tuple[float, float]]:
        """
        Calcula desplazamiento relativo al robot para mantener distancia.

        Args:
            person_x: Posición X de la persona (relativa al robot)
            person_y: Posición Y de la persona (relativa al robot)

        Returns:
            Tupla (dx, dy) relativa al robot o None
        """
        distance = math.hypot(person_x, person_y)
        if distance < 1e-3:
            return None

        # Cuánto falta para los 1.5m
        displacement = distance - self.desired_follow_distance
        if abs(displacement) < 1e-2:
            return None

        # Proyecta desplazamiento sobre línea robot-persona
        scale = displacement / distance
        rel_x = person_x * scale
        rel_y = person_y * scale

        # Limita el paso máximo
        step = math.hypot(rel_x, rel_y)
        if step > self.follow_max_step:
            limiter = self.follow_max_step / step
            rel_x *= limiter
            rel_y *= limiter

        return rel_x, rel_y

    def get_current_mode(self) -> RobotMode:
        """Retorna el modo actual."""
        return self._mode
