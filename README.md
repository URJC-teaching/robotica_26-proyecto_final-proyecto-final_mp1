# Proyecto Final — ROS2 Jazzy

Robot autónomo (Kobuki) con **Nav2** (navegación), **HRI** (voz local
Whisper + MMS-TTS) y **YOLO 2D** (detección y seguimiento de personas
estimando la distancia con la altura del bbox), orquestado por una FSM
en `mission_manager_node.py`.

Funciona en **simulación** (Gazebo Harmonic) y en el **robot real**
(openni2 + scan + AMCL); el modo se selecciona con `mode:=sim` /
`mode:=real` y el launch ajusta automáticamente topics y frames.

---

## Qué hace

Ciclo de la FSM (10 Hz):

```
INIT → ASK (TTS menú) → LISTEN (STT) → DECIDE → …
       │
       ├─ opt 1: NAV → "puerta"
       ├─ opt 2: NAV → "centro"
       ├─ opt 3: FOLLOW (YOLO) → se acerca a una persona ≤ 1.5 m
       └─ opt 4: NAV → "punto_b" → vuelve al inicio
       (sin reconocimiento → elige 1–4 al azar y lo anuncia por TTS)
       │
       └─ ARRIVED ("He llegado…", añade "Veo a una persona delante" si
                    YOLO ha visto a alguien en los últimos 2 s) → DONE → ASK
```

Menú de voz (palabras clave reconocidas por `extract_service`):

| Lo que dice el usuario              | Opción |
|-------------------------------------|--------|
| "uno", "1", "puerta"                | 1      |
| "dos", "2", "centro"                | 2      |
| "tres", "3", "persona", "sígueme"   | 3      |
| "cuatro", "4", "aleatorio", "paseo" | 4      |

---

## Estructura del paquete

```
pfinal_nav2_hri_manager/
├── pfinal_nav2_hri_manager/
│   ├── mission_manager_node.py   ← FSM (Nav2 + HRI + YOLO)
│   ├── yolo_depth_node.py        ← YOLO 2D → Detection3DArray (apaño 2D)
│   ├── hri_test_node.py          ← prueba aislada HRI
│   └── nav_test_node.py          ← prueba aislada Nav2
├── launch/
│   ├── pfinal_nav2_hri_manager.launch.py  ← launch principal
│   └── hri_bringup.launch.py             ← solo stack HRI
└── config/
    └── waypoints.yaml            ← coordenadas en frame map
```

### El "apaño 2D" de `yolo_depth_node`

`yolo_ros` trae `detect_3d_node` (LifecycleNode) que proyecta detecciones
2D a 3D usando la imagen de profundidad. En la VM sin GPU se queda
colgado y satura RAM. En su lugar usamos `yolo_depth_node`, que sigue la
idea de `p6/vff_control_p6/yolo_class_detector_node_2d`:

- subscribe `/yolo/detections` (yolo_msgs/DetectionArray, 2D) y `camera_info`
- ángulo: `θ = atan((u − cx) / fx)` (intrínseca de la cámara)
- distancia: asume persona de 1.7 m → `d = (1.7 · fy) / bbox_h_pixeles`
- publica `vision_msgs/Detection3DArray` en `/detections_3d`, en frame
  `camera_link` (sim) o `camera_rgb_frame` (real), con
  `(x = d·cos θ, y = d·sin θ, z = 0)`

`mission_manager` solo lee `/detections_3d` y hace TF lookup a `map`.

---

## Dependencias

| Paquete                                                             | Para qué                                              |
|---------------------------------------------------------------------|-------------------------------------------------------|
| `kobuki`                                                            | bringup robot/sim, Nav2 (`navigation_sim.launch.py`)  |
| `simple_hri` + `hri_client` + `simple_hri_interfaces`               | servicios STT/TTS/extract/yesno                       |
| `sound_play`                                                        | reproducción de TTS                                   |
| `yolo_bringup`, `yolo_ros`, `yolo_msgs` (en `thirdparty/yolo_ros/`) | YOLO 2D                                               |
| `vision_msgs`, `nav2_msgs`, `tf2_ros`                               | tipos estándar                                        |
| **`venv_asr/`** en la raíz del workspace                            | torch, ultralytics, whisper, transformers (lo añade el launch al `PYTHONPATH`) |

Token HuggingFace: `~/.hf_token` (fuera del repo, `chmod 600`).

Mapa: el launch detecta `mp3_ws/map_edited.yaml` y lo pasa a Nav2; si no
existe usa el por defecto del bringup.

---

## Configuración (parámetros del launch)

Todos los argumentos tienen `default_value` y son opcionales.

| Argumento            | Default | Significado                                                      |
|----------------------|---------|------------------------------------------------------------------|
| `mode`               | `real`  | `real` ⇒ topics/frames del Kobuki; `sim` ⇒ topics de Gazebo      |
| `include_nav2`       | `false` | Lanza `kobuki/navigation_sim.launch.py` con `map_edited.yaml`    |
| `include_hri`        | `true`  | Lanza el stack HRI (STT + TTS + extract + yesno)                 |
| `use_yolo`           | `true`  | Lanza `yolo_bringup` (2D, CPU, nano) + `yolo_depth_node`         |
| `run_mission`        | `true`  | Lanza `mission_manager` (FSM)                                    |
| `mission_delay_sec`  | `12.0`  | Segundos antes de lanzar `mission_manager`. Da tiempo a YOLO/HRI a cargar modelos |
| `test_hri`           | `false` | Lanza `hri_test_node` aislado                                    |
| `test_nav`           | `false` | Lanza `nav_test_node` aislado                                    |

Topics y frames (según `mode`):

|                | `sim`                       | `real`                          |
|----------------|-----------------------------|---------------------------------|
| imagen RGB     | `/rgbd_camera/image`        | `/camera/rgb/image_raw`         |
| `camera_info`  | `/rgbd_camera/camera_info`  | `/camera/depth_raw/camera_info` |
| `camera_frame` | `camera_link`               | `camera_rgb_frame`              |
| `base_frame`   | `base_link`                 | `base_footprint`                |

YOLO se lanza con `model:=yolov8n.pt`, `device:=cpu`,
`use_tracking:=False`, `use_3d:=False`, `imgsz:=320×320` (perfil
VM-friendly).

`yolo_depth_node` admite además: `target_class` (`person`),
`person_height_m` (`1.7`), `min_bbox_height_px` (`20`),
`max_distance_m` (`6.0`).

---

## Cómo lanzar

### Prerequisitos en cada terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/ROBOTICA/mp3_ws/install/setup.bash
```

### Modo simulación (todo)

```bash
# Terminal 1 — Gazebo
ros2 launch kobuki simulation.launch.py

# Terminal 2 — Nav2 + HRI + YOLO + FSM (con el mapa editado)
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
    mode:=sim include_nav2:=true
```

Tras `mission_delay_sec` (12 s) la FSM arranca y empieza a hablar.
Whisper y MMS-TTS pueden tardar 20–30 s en cargar la primera vez.

### Modo robot real (todo)

Asume que el bringup del Kobuki ya está corriendo (cámara openni2,
scan, odometría, AMCL):

```bash
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py mode:=real
```

(`include_nav2` se queda en `false`: en el robot real Nav2 ya está
lanzado por el bringup).

### Componentes por separado

```bash
# Solo HRI
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
    mode:=sim include_nav2:=false use_yolo:=false run_mission:=false test_hri:=true

# Solo YOLO 2D + yolo_depth_node (sim ya corriendo)
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
    mode:=sim include_nav2:=false include_hri:=false run_mission:=false use_yolo:=true

# Solo mission_manager (Nav2 + HRI + YOLO ya vivos)
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
    mode:=sim include_nav2:=false include_hri:=false use_yolo:=false run_mission:=true

# Test Nav2 aislado
ros2 run pfinal_nav2_hri_manager nav_test \
    --ros-args -p goal_x:=1.37 -p goal_y:=7.01 -p use_sim_time:=true
```

---

## Waypoints (frame `map`, alineados con `map_edited.pgm`)

| Nombre     | x (m) | y (m) | θ (rad) | Uso       |
|------------|-------|-------|---------|-----------|
| `puerta`   | 1.37  | 7.01  | -0.745  | Opción 1  |
| `centro`   | 1.627 | 4.287 |  0.604  | Opción 2  |
| `punto_b`  | 3.263 | 8.449 |  2.154  | Opción 4  |
| `papelera` | 1.734 | 5.891 | -0.746  | extra     |

---

## Notas de VM

- **Arranque lento de HRI** (~20–30 s la primera vez); `mission_delay_sec`
  cubre eso. La FSM además reintenta hasta que `wait_for_services()` pasa.
- **YOLO en CPU** (`yolov8n.pt`, 320×320). Si la VM va muy justa,
  baja la resolución o súbele `mission_delay_sec`.
- **POST_TTS_BUFFER_SEC = 1.5 s** evita que el STT empiece mientras
  soundplay aún reproduce (típico en VM con latencia de audio).
- **Empty world ↔ persona**: para probar la opción 3 hace falta un
  modelo de persona en la escena (Gazebo Fuel `Standing Person`); en
  `empty.world` el pipeline funciona pero sin detecciones.
