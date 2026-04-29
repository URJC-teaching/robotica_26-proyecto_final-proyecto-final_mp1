"""
person_tracker.py: Gestión de detecciones y seguimiento de personas.
Responsable de mantener actualizado el estado de personas detectadas por YOLO.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

from yolo_msgs.msg import DetectionArray


@dataclass
class PersonDetection:
    """Estructura que representa una detección de persona."""
    track_id: str
    score: float
    x: float
    y: float
    z: float


class PersonTracker:
    """
    Gestiona las detecciones de personas del sistema YOLO 3D.
    Filtra por clase, puntuación mínima y mantiene un registro de personas vistas.
    """

    def __init__(
        self,
        person_class_name: str,
        min_detection_score: float,
    ) -> None:
        """
        Args:
            person_class_name: Nombre de la clase a filtrar (ej: 'person')
            min_detection_score: Puntuación mínima para considerar una detección
        """
        self.person_class_name = person_class_name
        self.min_detection_score = min_detection_score
        self._latest_people: List[PersonDetection] = []

    def process_detections(self, msg: DetectionArray) -> None:
        """
        Procesa un mensaje de detecciones 3D de YOLO.

        Args:
            msg: Mensaje DetectionArray de yolo_msgs
        """
        people: List[PersonDetection] = []

        for detection in msg.detections:
            # Filtrar solo por clase 'person' y puntuación mínima
            if detection.class_name.lower() != self.person_class_name.lower():
                continue
            if detection.score < self.min_detection_score:
                continue
            if not detection.bbox3d.frame_id:
                continue

            px = float(detection.bbox3d.center.position.x)
            py = float(detection.bbox3d.center.position.y)
            pz = float(detection.bbox3d.center.position.z)

            # Validar que los valores sean números válidos
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

    def get_latest_people(self) -> List[PersonDetection]:
        """Retorna la lista de personas detectadas más reciente."""
        return self._latest_people

    def select_person(self, preferred_track_id: str = '') -> Optional[PersonDetection]:
        """
        Selecciona una persona según criterios.

        Args:
            preferred_track_id: Si se especifica, busca esta persona. Si no existe
                               o está vacío, retorna la más cercana.

        Returns:
            PersonDetection o None si no hay personas detectadas
        """
        if not self._latest_people:
            return None

        # Si tenemos un ID fijado, lo buscamos
        if preferred_track_id:
            for person in self._latest_people:
                if person.track_id == preferred_track_id:
                    return person

        # Sino, retorna la persona más cercana (por distancia euclideana en x,y)
        return min(
            self._latest_people,
            key=lambda person: math.hypot(person.x, person.y)
        )

    def has_people(self) -> bool:
        """Retorna True si hay personas detectadas."""
        return len(self._latest_people) > 0
