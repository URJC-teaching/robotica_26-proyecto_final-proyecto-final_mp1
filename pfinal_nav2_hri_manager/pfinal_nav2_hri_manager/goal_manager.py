"""
goal_manager.py: Gestión de objetivos y puntos de navegación.
Responsable de cargar objetivos desde archivo, parsear comandos de voz y crear poses.
"""

import re
import unicodedata
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


class GoalManager:
    """
    Gestiona los objetivos nombrados del sistema.
    Carga puntos desde YAML, parsea comandos de voz y crea poses para navegación.
    """

    def __init__(self, logger) -> None:
        """
        Args:
            logger: Logger de ROS2 para mensajes
        """
        self.logger = logger
        self._named_goals: Dict[str, Tuple[float, float, float]] = {}
        self._goal_display_names: Dict[str, str] = {}
        self._goal_alias_to_name: Dict[str, str] = {}

    def load_goals_from_file(self, file_path: str) -> None:
        """
        Carga objetivos desde un archivo YAML.

        Formato esperado:
        ```
        goals:
          puerta_a:
            x: 1.0
            y: 2.0
            yaw: 0.0
          cocina:
            x: -1.5
            y: 3.2
            yaw: 1.57
        ```

        Args:
            file_path: Ruta al archivo YAML
        """
        if not file_path:
            self.logger.warn('No se ha definido named_goals_file')
            return

        path = Path(file_path)
        if not path.exists():
            self.logger.warn(f'Archivo de puntos no encontrado: {file_path}')
            return

        try:
            with open(path, 'r', encoding='utf-8') as handle:
                yaml_data = yaml.safe_load(handle) or {}
        except Exception as exc:
            self.logger.error(f'No se pudo leer {file_path}: {exc}')
            return

        goals_data = yaml_data.get('goals', yaml_data)
        if not isinstance(goals_data, dict):
            self.logger.error('El archivo de puntos no tiene formato válido')
            return

        for raw_name, goal_data in goals_data.items():
            if not isinstance(goal_data, dict):
                continue

            try:
                x = float(goal_data['x'])
                y = float(goal_data['y'])
                yaw = float(goal_data.get('yaw', 0.0))
            except (KeyError, ValueError, TypeError):
                self.logger.warn(f'Punto omitido por formato inválido: {raw_name}')
                continue

            canonical_name = self._canonical_goal_name(str(raw_name))
            self._named_goals[canonical_name] = (x, y, yaw)
            self._goal_display_names[canonical_name] = str(raw_name)

            # Registra aliases para facilitar búsqueda por voz
            for alias in self._build_goal_aliases(str(raw_name)):
                self._goal_alias_to_name[alias] = canonical_name

        self.logger.info(f'Cargados {len(self._named_goals)} objetivos nombrados')

    def get_goal_by_name(self, canonical_name: str) -> Optional[Tuple[float, float, float]]:
        """
        Obtiene las coordenadas de un objetivo por nombre canónico.

        Args:
            canonical_name: Nombre canónico (normalizado)

        Returns:
            Tupla (x, y, yaw) o None si no existe
        """
        return self._named_goals.get(canonical_name)

    def get_all_goal_names(self) -> List[str]:
        """Retorna lista de nombres canónicos de todos los objetivos."""
        return list(self._named_goals.keys())

    def extract_named_goal(self, command: str) -> Optional[str]:
        """
        Extrae un objetivo nombrado de un comando de voz.

        Busca aliases que coincidan en el comando y retorna el nombre canónico.

        Args:
            command: Texto del comando de voz

        Returns:
            Nombre canónico del objetivo o None
        """
        for alias, canonical_name in self._goal_alias_to_name.items():
            if alias and alias in command:
                return canonical_name
        return None

    def extract_coordinates(self, command: str) -> Optional[Tuple[float, float, float]]:
        """
        Extrae coordenadas x, y (y opcionalmente yaw) de un comando de voz.

        Soporta formatos como:
         - "ve a x 1 y 2"
         - "ve a x=1 y=2 yaw=0.5"
         - "ir a 5, 10"

        Args:
            command: Texto del comando

        Returns:
            Tupla (x, y, yaw) o None si no encuentra coordenadas
        """
        # Primer intento: búsqueda explícita "x=valor y=valor"
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

        # Segundo intento: si dice "ir a" o similar, busca números
        if not any(word in command for word in ['ir', 'punto', 'coordenada', 'coordenadas']):
            return None

        values = re.findall(r'-?\d+(?:[\.,]\d+)?', command)
        if len(values) < 2:
            return None

        x = self._to_float(values[0])
        y = self._to_float(values[1])
        yaw = self._to_float(values[2]) if len(values) >= 3 else 0.0
        return x, y, yaw

    @staticmethod
    def _build_goal_aliases(goal_name: str) -> List[str]:
        """
        Crea variantes del nombre para búsqueda flexible.

        De "puerta_a" genera: ["puertak", "puerta k", "puertak", ...]

        Args:
            goal_name: Nombre original del objetivo

        Returns:
            Lista de aliases normalizados
        """
        normalized = GoalManager._normalize_text(goal_name)
        normalized_spaced = normalized.replace('_', ' ')
        compact = normalized_spaced.replace(' ', '')

        aliases = [normalized, normalized_spaced, compact]
        aliases = [alias for alias in aliases if alias]

        # Elimina duplicados preservando orden
        seen = set()
        unique_aliases: List[str] = []
        for alias in aliases:
            if alias not in seen:
                seen.add(alias)
                unique_aliases.append(alias)

        return unique_aliases

    @staticmethod
    def _canonical_goal_name(goal_name: str) -> str:
        """
        Normaliza un nombre de objetivo a formato canónico.

        Args:
            goal_name: Nombre original

        Returns:
            Nombre normalizado y con espacios reemplazados por guiones bajos
        """
        return GoalManager._normalize_text(goal_name).replace(' ', '_')

    @staticmethod
    def _to_float(raw_value: str) -> float:
        """Convierte string a float, soportando comas como separador decimal."""
        return float(raw_value.replace(',', '.'))

    @staticmethod
    def _normalize_text(text: str) -> str:
        """
        Normaliza texto: minúsculas, elimina acentos, remueve caracteres especiales.

        Args:
            text: Texto a normalizar

        Returns:
            Texto normalizado
        """
        lower_text = text.lower().strip()
        # Elimina acentos (NFD descompone caracteres acentuados)
        no_accents = ''.join(
            char
            for char in unicodedata.normalize('NFD', lower_text)
            if unicodedata.category(char) != 'Mn'
        )
        # Reemplaza caracteres especiales por espacios
        clean = re.sub(r'[^a-z0-9_\.,\- ]+', ' ', no_accents)
        # Limpia espacios múltiples
        return re.sub(r'\s+', ' ', clean).strip()
