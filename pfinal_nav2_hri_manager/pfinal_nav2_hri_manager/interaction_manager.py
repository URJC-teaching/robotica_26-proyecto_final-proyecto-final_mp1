"""
interaction_manager.py: Gestión de la máquina de estados de interacción por voz.
Controla el flujo: hablar -> esperar -> escuchar -> procesar comando.
"""

from enum import Enum, auto
from typing import Callable, List, Optional, Tuple

from hri_client.hri_client import HRIClient


class InteractionState(Enum):
    """Estados de la máquina de interacción de voz."""
    IDLE = auto()           # Esperando para hablar
    SPEAKING = auto()       # Reproduciendo audio
    LISTENING = auto()      # Capturando voz del usuario


class InteractionManager:
    """
    Gestiona la máquina de estados de interacción de voz.
    Asegura que el robot no intente hablar y escuchar simultáneamente.
    """

    def __init__(
        self,
        hri_client: HRIClient,
        command_processor: Callable[[str], None],
        logger,
    ) -> None:
        """
        Args:
            hri_client: Cliente HRI (incluye TTS, STT, etc.)
            command_processor: Función que procesa comandos recibidos
            logger: Logger de ROS2
        """
        self.hri = hri_client
        self.command_processor = command_processor
        self.logger = logger

        self._interaction_state = InteractionState.IDLE
        self._speech_queue: List[Tuple[str, bool]] = []
        self._listen_after_speech = False
        self._waiting_prompt_active = False

    def process_interaction(self) -> None:
        """
        Procesa un paso de la máquina de interacción.
        Debe llamarse periódicamente desde el control_cycle.
        """
        if self._interaction_state == InteractionState.SPEAKING:
            self._handle_speaking_state()
            return

        if self._interaction_state == InteractionState.LISTENING:
            self._handle_listening_state()
            return

        # Estado IDLE: si hay mensajes en cola, comienza a hablar
        if self._speech_queue:
            speech_text, listen_after = self._speech_queue.pop(0)
            self.hri.start_speaking(speech_text)
            self._interaction_state = InteractionState.SPEAKING
            self._listen_after_speech = listen_after

    def _handle_speaking_state(self) -> None:
        """Maneja el estado SPEAKING."""
        if self.hri.is_speaking_done():
            self._interaction_state = InteractionState.IDLE
            if self._listen_after_speech:
                self.hri.start_listen()
                self._interaction_state = InteractionState.LISTENING
                self._listen_after_speech = False

    def _handle_listening_state(self) -> None:
        """Maneja el estado LISTENING."""
        if self.hri.is_listen_done():
            self._interaction_state = InteractionState.IDLE
            self._waiting_prompt_active = False

            text = self.hri.get_listened_text().strip()
            if not text:
                text = self.hri.get_last_listened_text().strip()

            self.logger.info(f'Comando recibido: {text}')
            self.command_processor(text)

    def enqueue_speech(self, text: str, listen_after: bool = False) -> None:
        """
        Añade un mensaje a la cola de reproducción de voz.

        Args:
            text: Texto a reproducir
            listen_after: Si True, escuchará después de hablar
        """
        if not text:
            return
        self._speech_queue.append((text, listen_after))
        if listen_after:
            self._waiting_prompt_active = True

    def is_idle(self) -> bool:
        """Retorna True si no hay interacción en curso."""
        return self._interaction_state == InteractionState.IDLE

    def is_listening(self) -> bool:
        """Retorna True si está escuchando."""
        return self._interaction_state == InteractionState.LISTENING

    def is_speaking(self) -> bool:
        """Retorna True si está hablando."""
        return self._interaction_state == InteractionState.SPEAKING

    def is_waiting_prompt_active(self) -> bool:
        """Retorna True si está esperando un comando por voz."""
        return self._waiting_prompt_active

    def clear_queue(self) -> None:
        """Limpia la cola de mensajes pendientes."""
        self._speech_queue.clear()
        self._listen_after_speech = False
        self._waiting_prompt_active = False
