[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/7bjYLsjm)
# Proyecto final

## Objetivo

Desarrollar una aplicación robótica que:

- Navegue de forma autónoma por al menos dos waypoints.
- Interactúe con el humano mediante el paquete HRI (Human-Robot Interaction).
- Utilice YOLO para la detección de objetos o personas.

## Configuración del sistema HRI

Consulta las instrucciones completas en el [repositorio de simple_hri](https://github.com/rodperex/simple_hri#launch-local-services).

Se recomienda usar el **modelo local** para evitar el consumo de tokens externos, para ello:

Lanzar los servicios locales:

```bash
ros2 launch simple_hri local_simple_hri.launch.py
```

Probar los servicios:

```bash
ros2 run simple_hri test_services
```

## Creación del mapa con Nav2

Si en algún momento necesitas rehacer el mapa o explorar la navegación por separado sin el manager del proyecto:

**Para crear, capturar y guardar un mapa:**
```bash
# 1. Levantar el robot (simulación o real)
ros2 launch kobuki simulation.launch.py
# 2. Lanzar SLAM / Mapeo
ros2 launch kobuki mapping.launch.py
# 3. Explorar con teleoperación (en otra terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 4. Guardar el mapa en la carpeta actual (generará map_edited.yaml y map_edited.pgm)
ros2 run nav2_map_server map_saver_cli -f map_edited
```

**Para levantar solo la navegación de Nav2 con la interfaz RViz:**
```bash
ros2 launch kobuki navigation_sim.launch.py use_sim_time:=true map:=$(pwd)/map_edited.yaml rviz:=True
```


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

## Notas finales: Arquitectura y Orquestación del Launch

El archivo principal `pfinal_nav2_hri_manager.launch.py` actúa como un **meta-launch integrador**. Está diseñado para encapsular y levantar todas las dependencias del proyecto de forma automática, evitando que el usuario tenga que usar múltiples terminales o recordar configuraciones complejas. Este launch se encarga de lo siguiente:

1. **Navegación Autónoma (`kobuki` y `nav2`):**  
   Llama dinámicamente a la navegación del robot base (`navigation_sim.launch.py` o `navigation.launch.py`) e inyecta la ruta de nuestro mapa editado (`map_edited.yaml`) de forma automática.

2. **Interacción Humano-Robot (`simple_hri`):**  
   Incluye `hri_bringup.launch.py`, el cual arranca los servicios locales de extracción de entidades, STT (Whisper) y TTS (MMS). Además, el launch del proyecto modifica la variable de entorno `PYTHONPATH` para incluir automáticamente `venv_asr`, asegurándose de que todas las librerías de IA se encuentren sin tener que activar el entorno en nuevas terminales.

3. **Visión Computacional (`yolo_bringup` y `yolo_depth_node`):**  
   Llama a la capa base de ejecución de `yolov8n.pt` para lograr bounding boxes en 2D optimizados para CPU. A continuación, invoca nuestro traductor `yolo_depth_node.py` interno (ubicado en `pfinal_nav2_hri_manager`), empleando las propiedades intrínsecas de la cámara para calcular la profundidad de la persona a partir de los píxeles; una estrategia mucho más ligera computacionalmente que procesar nubes de puntos 3D completas.

4. **El Cerebro de la Misión (`mission_manager_node.py`):**  
   Finalmente, levanta el gestor orquestador en un *TimerAction* (que retrasa su arranque unos 12 segundos para dar tiempo a que los modelos pesados de HRI y YOLO carguen del todo en RAM y memoria). Este Manager funciona como una Máquina de Estados Finita (FSM) que aglutina:
   * **Cliente de HRI** (`HRIClient`) para interactuar con los nodos de escucha/habla.
   * **Cliente Action de Navegación** (`NavigationClient`) para lanzar los waypoints o coordenadas calculadas frente a las personas.
   * Lógica propia acoplada al **nodo YOLO 3D** procesando los giros manuales de búsqueda (`/cmd_vel`) hasta encontrar un humano.

### Comandos individuales equivalentes (que de otra forma obligarían a abrir 5-6 terminales)

Para lograr el mismo comportamiento del `mode:=sim` sin el launch principal, tendrías que ejecutar todo esto en múltiples terminales asegurándote de tener `venv_asr` activado en las de HRI/YOLO:

```bash
# 1. Navegación con mapa editado
ros2 launch kobuki navigation_sim.launch.py use_sim_time:=true map:=/home/alumno/Documents/ROBOTICA/mp3_ws/map_edited.yaml rviz:=False

# 2. Servicios de HRI (Escucha y Habla local)
# El paquete tiene su propio hri_bringup que automatiza la inyección del Token de HuggingFace (~/.hf_token) 
# y llama internamente a la dependencia externa `simple_hri` (específicamente a `free_simple_hri.launch.py` o `local_simple_hri.launch.py`).
# En lugar de configurar tokens a mano y llamar a la dependencia externa, simplemente usaríamos:
ros2 launch pfinal_nav2_hri_manager hri_bringup.launch.py

# 3. Nodo principal de Detección YOLOv8n
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/rgbd_camera/image model:=yolov8n.pt device:=cpu use_tracking:=False use_3d:=False imgsz_height:=320 imgsz_width:=320 use_sim_time:=true

# 4. Nodo Custom YOLO Depth (proyección 2D->3D)
ros2 run pfinal_nav2_hri_manager yolo_depth --ros-args -p use_sim_time:=true -r detections:=/yolo/detections -r camera_info:=/rgbd_camera/camera_info -r detections_3d:=/detections_3d

# 5. Finalmente, el Mission Manager (La Máquina de Estados)
ros2 run pfinal_nav2_hri_manager mission_manager --ros-args -p use_sim_time:=true -p waypoints_file:=/home/alumno/Documents/ROBOTICA/mp3_ws/src/pfinal_nav2_hri_yolop6/pfinal_nav2_hri_manager/config/waypoints.yaml
```

### Explicación rápida de partes clave del código en el FSM (`mission_manager_node.py`)
- **`_control_cycle(self)`**: Es el corazón del manager. Salta a la función correspondiente según el estado de la FSM (INIT, ASK, LISTEN, DECIDE, NAV... etc) a 10Hz.
- **`_h_decide(self)`**: Procesa el texto que haya interceptado en `_h_listen()`, pasándolo por un filtro flexible `WORD_TO_OPTION` (si dices "persona" en vez de "tres", igual detecta la Opción 3).
- **`_h_follow(self)`**: La lógica persecutoria. Si no detecta una persona (`_last_detection`), publica velocidad rotacional en `/cmd_vel` haciendo que el robot dé vueltas hasta localizar a alguien. Si la encuentra, usa `_person_goal_pose` para recalcular la posición en TF2 a la que el robot debe moverse (dejando 1.5 metros de saludo).



