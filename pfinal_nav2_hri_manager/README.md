# Proyecto Final — ROS2 Jazzy

Robot autónomo con tres pilares: **Nav2** (navegación), **HRI** (voz) y **YOLO** (detección).

---

## Estructura del paquete

```
final_project/
├── final_project/
│   ├── mission_manager_node.py   ← Orquestador FSM principal
│   ├── nav_test_node.py          ← Prueba standalone Nav2
│   └── hri_test_node.py          ← Prueba standalone HRI
├── launch/
│   └── final_project.launch.py   ← Launch completo
├── config/
│   └── waypoints.yaml            ← Coordenadas de destinos
├── package.xml
└── setup.py
```

---

## Los tres pilares

### 1. Nav2 — Navegación autónoma
El robot navega a **waypoints predeterminados** definidos en `config/waypoints.yaml`.
Cada waypoint tiene coordenadas `(x, y)` en metros en el frame `map` y una orientación `theta` en radianes.

**Cómo obtener coordenadas de tu mapa PGM:**
1. Abre `map.yaml` y anota `resolution` y `origin: [ox, oy, 0]`
2. Abre el PGM en GIMP → mueve el ratón al punto deseado → anota `(px, py)`
3. Convierte:
   ```
   x = ox + px * resolution
   y = oy + (altura_imagen - py) * resolution
   ```
4. O más fácil: lanza el robot en RViz, usa **2D Nav Goal** y lee las coordenadas del log

### 2. HRI — Interacción por voz
Usa **TTS** (`audio_common_msgs/TTS`) para hablar y **STT** (`whisper_msgs/STT`) para escuchar.

**Comandos reconocidos:**
| Lo que dices | Acción |
|---|---|
| "ve a casa" / "inicio" | Navega al waypoint `home` |
| "ve a la papelera" / "basura" | Navega al waypoint `papelera` |
| "ve al punto a/b" | Navega al waypoint correspondiente |
| "sígüeme" / "follow" / "ven conmigo" | Entra en modo seguimiento YOLO |
| Cualquier otra cosa | Waypoint aleatorio (fallback) |

### 3. YOLO — Seguimiento de personas
Cuando el modo es `FOLLOW_PERSON`, el orquestador consume poses publicadas en `/yolo/target_pose` (`geometry_msgs/PoseStamped`) y las envía continuamente como goals a Nav2.

Tu nodo YOLO debe publicar en ese topic la pose de la persona detectada en el frame `map`.

---

## FSM del orquestador

```
IDLE ──► WAIT_COMMAND ──► NAVIGATE ──► DONE_GOAL ──► WAIT_COMMAND
                    └──► FOLLOW_PERSON ──────────────────────┘
```

- **IDLE**: Espera 1 s al arrancar
- **WAIT_COMMAND**: Dice el prompt → escucha → parsea → decide
- **NAVIGATE**: Envía goal a Nav2, espera resultado con timeout de 60 s
- **FOLLOW_PERSON**: Reenvía poses YOLO a Nav2 continuamente
- **DONE_GOAL**: Anuncia llegada → espera 2 s → vuelve a `WAIT_COMMAND`

---

## Instalación y compilación

```bash
# Copiar el paquete a tu workspace
cp -r final_project ~/ros2_ws/src/

# Compilar
cd ~/ros2_ws
colcon build --packages-select final_project
source install/setup.bash
```

---

## Ejecución

### Prueba solo Nav2
```bash
ros2 launch final_project final_project.launch.py test_nav:=true
# O con goal personalizado:
ros2 run final_project nav_test --ros-args -p goal_x:=3.0 -p goal_y:=-1.5
```

### Prueba solo HRI
```bash
ros2 launch final_project final_project.launch.py test_hri:=true
```

### Proyecto completo
Asegúrate de que Nav2, el servidor TTS (`say`) y el servidor STT (`whisper/listen`) están activos, luego:
```bash
ros2 launch final_project final_project.launch.py
```

Con simulación Gazebo/Webots:
```bash
ros2 launch final_project final_project.launch.py use_sim_time:=true
```

---

## Personalizar waypoints

Edita `config/waypoints.yaml`:
```yaml
waypoints:
  home:
    x: 0.0
    y: 0.0
    theta: 0.0
  mi_punto_nuevo:
    x: 4.5
    y: 2.0
    theta: 1.5707
```

Añade el nombre al parseo de comandos HRI en `mission_manager_node.py` en `_parse_command()` si quieres activarlo por voz.

---

## Dependencias

- `nav2_msgs` — acción `NavigateToPose`
- `audio_common_msgs` — acción TTS `say`
- `whisper_msgs` — acción STT `whisper/listen`
- `geometry_msgs`, `std_msgs`, `rclpy`

---

## Reutilización de paquetes del curso

| Paquete del curso | Uso en este proyecto |
|---|---|
| `vff_control` | Puede lanzarse en paralelo para evitación de obstáculos local (publica en `vel`) |
| `navigation_client` (simple_navigation_app) | Patrón copiado en `nav_test_node.py` |
| `hri_examples` (repeat_node, hri_client) | Patrón copiado en `hri_test_node.py` y `mission_manager_node.py` |
