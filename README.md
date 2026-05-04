# Proyecto Final — ROS2 Jazzy

Robot autónomo con **Nav2** (navegación), **HRI** (voz local) y **YOLO** (detección 3D de personas),
orquestado por una FSM de 10 Hz en `mission_manager_node.py`.

---

## Estructura del código

```
pfinal_nav2_hri_manager/
├── pfinal_nav2_hri_manager/
│   ├── mission_manager_node.py   ← FSM principal (Nav2 + HRI + YOLO)
│   ├── hri_test_node.py          ← prueba aislada HRI (TTS→STT→repite)
│   └── nav_test_node.py          ← prueba aislada Nav2 (envía un goal)
├── launch/
│   ├── pfinal_nav2_hri_manager.launch.py  ← launch principal
│   └── hri_bringup.launch.py             ← solo stack HRI
└── config/
    └── waypoints.yaml            ← coordenadas en frame map (map_edited.pgm)
```

### FSM del mission_manager

```
INIT → ASK(TTS menú) → LISTEN(STT) → DECIDE →
  opt 1: NAV(puerta) → ARRIVED → DONE → ASK
  opt 2: NAV(centro) → ARRIVED → DONE → ASK
  opt 3: FOLLOW(YOLO) → ARRIVED → DONE → ASK
  opt 4: RANDOM_OUT(punto_b) → RANDOM_BACK(inicio) → ARRIVED → DONE → ASK
  sin reconocer: aleatorio 1-4 (anuncia la elección)
```

### Menú de voz

| Lo que dice el usuario       | Opción | Acción                          |
|------------------------------|--------|---------------------------------|
| "uno", "1", "puerta"         | 1      | Ir al waypoint **puerta**       |
| "dos", "2", "centro"         | 2      | Ir al waypoint **centro**       |
| "tres", "3", "persona", "sígueme" | 3 | Buscar persona con YOLO        |
| "cuatro", "4", "aleatorio", "paseo" | 4 | Ir a punto_b y volver       |
| (no reconocido)              | aleatorio | Elige 1–4 al azar y lo anuncia |

### Stack HRI (simple_hri)

| Nodo                  | Servicio ROS2      | Tecnología              |
|-----------------------|--------------------|-------------------------|
| `stt_service_local`   | `/stt_service`     | Whisper tiny (CPU)      |
| `tts_service_local`   | `/tts_service`     | HF MMS-TTS spa (CPU)    |
| `extract_service_hugg`| `/extract_service` | HF Inference API (cloud)|
| `yesno_service_local` | `/yesno_service`   | Patrones locales        |

Token HuggingFace: `~/.hf_token` (fuera del repo, `chmod 600`).

---

## Prerequisitos de entorno

```bash
# Terminal 1 — zenoh router (siempre primero)
rmw_zenohd

# Source del workspace (en cada terminal nueva)
source /opt/ros/jazzy/setup.bash
source ~/Documents/ROBOTICA/mp3_ws/install/setup.bash
```

---

## Lanzar todo junto (modo simulador)

```bash
# Terminal 2 — simulador Gazebo
ros2 launch kobuki simulation.launch.py

# Terminal 3 — Nav2 + mapa editado + HRI + YOLO + FSM
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim include_nav2:=true

# Nav2 necesita ~15 s para activarse; HRI necesita ~20-30 s para cargar modelos.
# El robot empieza a hablar cuando todos los servicios estén listos.
```

> **Mapa**: el launch detecta automáticamente `mp3_ws/map_edited.yaml` y lo pasa a Nav2.
> Si no existe, usará `aws_house.yaml` (los waypoints no coincidirán).

---

## Lanzar componentes por separado

### Solo Nav2 (sim corriendo)
```bash
ros2 launch kobuki navigation_sim.launch.py \
  map:=$HOME/Documents/ROBOTICA/mp3_ws/map_edited.yaml
```

### Solo HRI (en terminal con modelos ya cargados)
```bash
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim include_nav2:=false use_yolo:=false run_mission:=false \
  include_hri:=true test_hri:=true
```

### Solo mission manager (Nav2 + HRI ya corriendo)
```bash
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim include_nav2:=false include_hri:=false use_yolo:=false run_mission:=true
```

### Test Nav2 aislado (Nav2 ya corriendo)
```bash
ros2 run pfinal_nav2_hri_manager nav_test \
  --ros-args -p goal_x:=1.37 -p goal_y:=7.01 -p use_sim_time:=true
```

### Test HRI aislado
```bash
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim run_mission:=false include_nav2:=false use_yolo:=false test_hri:=true
```

---

## Waypoints (frame map, mapa map_edited.pgm)

| Nombre     | x (m)   | y (m)   | theta (rad) | Descripción    |
|------------|---------|---------|-------------|----------------|
| `puerta`   | 1.37    | 7.01    | -0.745      | Opción 1       |
| `centro`   | 1.627   | 4.287   | 0.604       | Opción 2       |
| `punto_b`  | 3.263   | 8.449   | 2.154       | Destino opt. 4 |
| `papelera` | 1.734   | 5.891   | -0.746      | Extra          |

---

## Notas de VM / rendimiento

- **HRI lento al inicio**: Whisper y MMS-TTS cargan los modelos en el primer arranque
  (~20-30 s). Después es más rápido. El mission_manager reintenta hasta que estén listos.
- **Buffer post-TTS** (`POST_TTS_BUFFER_SEC = 1.5 s`): evita que STT empiece mientras
  soundplay aún reproduce. Aumentar si el audio sigue solapándose.
- **YOLO en CPU**: usa `yolov8n.pt` (nano). Si la VM no tiene recursos, YOLO puede
  caerse; el mission_manager funciona igualmente (opt. 3 no encontrará persona).
- **Zenoh**: siempre lanzar `rmw_zenohd` antes que cualquier nodo ROS2.
