# Proyecto Final - Sistema Robótico Simplificado

## Objetivo

Desarrollar una aplicación robótica que:

- **Navegue de forma autónoma por al menos dos waypoints.**
- **Interactúe con el humano mediante el paquete HRI (Human-Robot Interaction).**
- **Utilice YOLO para la detección de objetos o personas.**

---

# Proyecto Final — ROS2 Jazzy

Robot autónomo con tres pilares: **Nav2** (navegación), **HRI** (voz local sin internet) y
**YOLO** (detección 3D de personas). Toda la lógica está orquestada por una FSM de 10 Hz.

---

## Estructura del repositorio

```
pfinal_nav2_hri_yolop6/
├── pfinal_nav2_hri_manager/        ← paquete principal (FSM + launches + waypoints)
│   ├── pfinal_nav2_hri_manager/
│   │   ├── mission_manager_node.py ← orquestador FSM principal
│   │   ├── hri_test_node.py        ← prueba aislada de HRI
│   │   └── nav_test_node.py        ← prueba aislada de Nav2
│   ├── launch/
│   │   ├── pfinal_nav2_hri_manager.launch.py  ← launch principal (todo + Nav2 opcional)
│   │   └── hri_bringup.launch.py              ← solo el stack HRI local
│   ├── config/
│   │   └── waypoints.yaml          ← coordenadas de los waypoints en frame map
│   ├── package.xml
│   └── setup.py
├── hri_examples/                   ← ejemplos del curso de HRI
├── nav2_example/                   ← ejemplo del curso de Nav2
└── README.md                       ← este archivo
```

---

## Cómo funciona

El robot escucha al usuario por voz, interpreta una opción del menú (1–4), ejecuta la
misión correspondiente y vuelve a preguntar al terminar. La FSM coordina:

```
                  		┌────────────────────────────────────┐
                  		│      mission_manager_node          │
                  		│                                    │
  /tts_service     ◄──	┤  FSM (10 Hz)                       │
  /stt_service     ──►	│  INIT → ASK → LISTEN → DECIDE      │
                  		│       → NAV / FOLLOW /             │
  /detections_3d    ──►	│         RANDOM_OUT / RANDOM_BACK   │
                  		│       → ARRIVED → DONE → ASK       │
  /navigate_to_pose	──►	│                                  	 │
  (Nav2 action)   		└────────────────────────────────────┘
```

### Estados de la FSM

| Estado | Función |
|---|---|
| `INIT` | Espera 1 s y comprueba que los servicios HRI están vivos (timeout 10 s, reintenta) |
| `ASK` | Reproduce el menú de voz por TTS |
| `LISTEN` | Activa el micrófono y espera la transcripción de Whisper |
| `DECIDE` | Parsea el texto a opción 1–4; si no entiende elige aleatoria |
| `NAV` | Envía un waypoint a Nav2 y espera resultado (timeout 90 s) |
| `FOLLOW` | YOLO: localiza persona, navega hasta `≤ 1.5 m`, mantiene 2 s |
| `RANDOM_OUT` | Navega a `punto_b` (opción 4) |
| `RANDOM_BACK` | Vuelve a la pose guardada antes de salir (TF en el momento de decidir) |
| `ARRIVED` | Anuncia "He llegado" + comenta si ve persona; espera `post_goal_wait` |
| `DONE` | Vuelve inmediatamente a `ASK` |

### Menú de voz

| Lo que dice el usuario | Opción | Acción |
|---|---|---|
| "uno", "1", "papelera" | 1 | Ir al waypoint `papelera` |
| "dos", "2", "punto a" | 2 | Ir al waypoint `punto_a` |
| "tres", "3", "persona", "sígueme" | 3 | Buscar persona con YOLO y acercarse |
| "cuatro", "4", "aleatorio", "paseo" | 4 | Ir a `punto_b` y volver al inicio |
| *(no reconocido)* | aleatoria | Elige 1–4 al azar y lo anuncia |

---

## Dependencias y nodos externos

| Nodo / Stack | Paquete | Provee | Necesario para |
|---|---|---|---|
| Bringup robot+sim | `kobuki/simulation.launch.py` | TF, sensores, `/cmd_vel`, cámara RGB-D | Sim |
| Nav2 + mapa | `kobuki/navigation_sim.launch.py` | `/navigate_to_pose` action, costmaps, AMCL | Sim navegación |
| `stt_service_local` | `simple_hri` | `/stt_service` (`std_srvs/SetBool`) — Whisper tiny local | HRI |
| `tts_service_local` | `simple_hri` | `/tts_service` (`simple_hri_interfaces/Speech`) — MMS-TTS local | HRI |
| `extract_service_hugg` | `simple_hri` | `/extract_service` — LLM HF cloud (opcional) | HRI extract |
| `yesno_service_local` | `simple_hri` | `/yesno_service` — patrones sí/no | HRI yesno |
| `soundplay_node` | `sound_play` | Reproducción de WAV en altavoz | TTS audio |
| `yolo_ros` | `yolo_bringup` | `yolo_msgs/DetectionArray` (2D + depth) | YOLO 2D |
| `yolo_detection_node_3d` | `camera` | `vision_msgs/Detection3DArray` en `/detections_3d` | YOLO 3D |

### Dependencias Python (en `venv_asr`)

El stack HRI necesita Whisper, transformers y huggingface_hub. Se instalaron en el
entorno virtual `~/Documents/ROBOTICA/mp3_ws/venv_asr`:

```bash
source venv_asr/bin/activate
pip list | grep -E "whisper|torch|transformers|huggingface_hub"
```

> **IMPORTANTE**: cualquier terminal que lance `hri_bringup.launch.py` o el launch
> principal con HRI debe activar el venv primero, si no `stt_service_local` y
> `yesno_service_local` morirán con `ModuleNotFoundError`.

### Token HuggingFace

El servicio `extract_service_hugg` usa la HF Inference API (LLM en la nube).
El token se lee de `~/.hf_token` (fuera del repo, nunca se sube a git):

```bash
echo "hf_xxxxxxxxxxxxxxxxxxxx" > ~/.hf_token
chmod 600 ~/.hf_token
```

Si no existe el archivo, intenta la variable `HF_TOKEN` del entorno.

---

## Compilar

```bash
cd ~/Documents/ROBOTICA/mp3_ws
colcon build --symlink-install --packages-select camera pfinal_nav2_hri_manager
source install/setup.bash
```

---

## Cómo lanzar — escenarios

> **Setup mínimo en cada terminal:**
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/Documents/ROBOTICA/mp3_ws/install/setup.bash
> source ~/Documents/ROBOTICA/mp3_ws/venv_asr/bin/activate   # solo si lanza HRI
> export RMW_IMPLEMENTATION=rmw_zenoh_cpp
> ```
>
> Y un router zenoh corriendo aparte (una vez por sesión):
> ```bash
> ros2 run rmw_zenoh_cpp rmw_zenohd
> ```

### Escenario 1 — Proyecto completo en simulador (lo más común)

```bash
# Terminal 1: simulador (Gazebo + robot Kobuki)
ros2 launch kobuki simulation.launch.py

# Terminal 2: TODO el proyecto (Nav2 + HRI + YOLO + mission manager)
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim include_nav2:=true
```

### Escenario 2 — Proyecto completo en robot real

```bash
# Terminal 1: bringup del robot real (tu launch propio)
# Terminal 2: Nav2 con tu mapa
ros2 launch kobuki navigation.launch.py

# Terminal 3: TODO el proyecto
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py
```

### Escenario 3 — Probar solo Nav2 (sin HRI ni YOLO)

```bash
# Sim + Nav2
ros2 launch kobuki simulation.launch.py &
ros2 launch kobuki navigation_sim.launch.py rviz:=False &

# Lanzar nav_test
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim run_mission:=false include_hri:=false use_yolo:=false test_nav:=true
```

O directamente:

```bash
ros2 run pfinal_nav2_hri_manager nav_test \
  --ros-args -p goal_x:=0.5 -p goal_y:=0.0 -p goal_theta:=0.0 -p use_sim_time:=true
```

### Escenario 4 — Probar solo HRI (sin Nav2, sin YOLO)

```bash
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  run_mission:=false use_yolo:=false test_hri:=true
```

El nodo dice "Hola, voy a escucharte y repetir lo que digas", graba 5 s y reproduce
lo que ha entendido.

### Escenario 5 — Solo el stack HRI (servicios sin lógica)

```bash
ros2 launch pfinal_nav2_hri_manager hri_bringup.launch.py
```

Útil para probar manualmente con `ros2 service call`:

```bash
ros2 service call /tts_service simple_hri_interfaces/srv/Speech "{text: 'Hola mundo'}"
ros2 service call /stt_service std_srvs/srv/SetBool "{data: true}"
```

### Escenario 6 — Solo YOLO 3D

```bash
ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py \
  mode:=sim run_mission:=false include_hri:=false use_yolo:=true

# Ver detecciones
ros2 topic echo /detections_3d --once
```

---

## Argumentos del launch principal

| Argumento | Default | Descripción |
|---|---|---|
| `mode` | `real` | `real` o `sim` — auto-selecciona topics y frames |
| `include_nav2` | `false` | Lanza Nav2 (`kobuki/navigation_sim.launch.py` o `navigation.launch.py`) |
| `include_hri` | `true` | Lanza el stack HRI local |
| `use_yolo` | `true` | Lanza el pipeline YOLO 3D |
| `run_mission` | `true` | Lanza el `mission_manager_node` (FSM) |
| `test_hri` | `false` | Lanza `hri_test` en vez del mission manager |
| `test_nav` | `false` | Lanza `nav_test` en vez del mission manager |

### Topics y frames según `mode`

| Recurso | `real` | `sim` |
|---|---|---|
| imagen RGB | `/camera/rgb/image_raw` | `/rgbd_camera/image` |
| profundidad | `/camera/depth_raw/image` | `/rgbd_camera/depth_image` |
| camera info | `/camera/depth_raw/camera_info` | `/rgbd_camera/camera_info` |
| `base_frame` | `base_footprint` | `base_link` |
| `camera_frame` | `camera_rgb_frame` | `camera_link` |

---

## Configurar los waypoints

Coordenadas en `frame map` (las mismas que muestra RViz):

```yaml
# config/waypoints.yaml
waypoints:
  home:        # punto de retorno tras opción 4
    x: 0.0
    y: 0.0
    theta: 0.0   # radianes

  papelera:    # opción 1
    x: 2.5
    y: 1.0
    theta: 0.0

  punto_a:     # opción 2
    x: -1.5
    y: 2.0
    theta: 1.5707  # 90°

  punto_b:     # opción 4 (ida; vuelta = pose guardada en el momento de decidir)
    x: 3.0
    y: -1.5
    theta: 3.1415  # 180°
```

### Cómo obtener las coordenadas reales desde RViz

1. Lanza el robot + Nav2 con el mapa cargado.
2. En RViz haz clic en **"2D Goal Pose"** sobre el punto deseado.
3. En la terminal verás algo como:
   ```
   [bt_navigator]: Goal accepted by server, waiting for result.
   ```
   Para leer la posición exacta:
   ```bash
   ros2 topic echo /goal_pose
   ```
4. Copia `x`, `y`, y `theta` (extraído del cuaternión).

### Configurar la zona de exploración aleatoria (parámetros del launch)

```python
'random_xmin': -3.0,   # m, frame map
'random_xmax':  8.0,
'random_ymin': -3.0,
'random_ymax':  8.0,
```

Cambia estos valores en
[pfinal_nav2_hri_manager.launch.py](pfinal_nav2_hri_manager/launch/pfinal_nav2_hri_manager.launch.py)
para limitar la zona accesible (evita que pruebe puntos fuera del mapa).

---

## Crear y guardar un mapa con Nav2 (SLAM)

Si necesitas generar tu propio mapa:

```bash
# Terminal 1: robot + sensores (sim o real)
ros2 launch kobuki simulation.launch.py

# Terminal 2: SLAM Toolbox (mapeo en tiempo real)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3: teleoperación
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: cuando el mapa esté completo
ros2 run nav2_map_server map_saver_cli -f ~/maps/mi_mapa
```

Para usarlo después, lanza Nav2 apuntando a ese mapa:

```bash
ros2 launch kobuki navigation_sim.launch.py map:=/home/alumno/maps/mi_mapa.yaml
```

---

## Partes importantes del código

### FSM — ciclo de control

Timer a 10 Hz que despacha al handler del estado activo. Cada `_h_*` es **no
bloqueante** y llama a `_go(nuevo_estado)` para transitar.

```python
def _control_cycle(self):
    {
        State.INIT:        self._h_init,
        State.ASK:         self._h_ask,
        State.LISTEN:      self._h_listen,
        State.DECIDE:      self._h_decide,
        State.NAV:         self._h_nav,
        State.FOLLOW:      self._h_follow,
        State.RANDOM_OUT:  self._h_random_out,
        State.RANDOM_BACK: self._h_random_back,
        State.ARRIVED:     self._h_arrived,
        State.DONE:        self._h_done,
    }[self.state]()
```

### HRI — API asíncrona con HRIClient

Nunca bloquea el ciclo. Las llamadas devuelven inmediatamente y el resultado se
consulta en el siguiente tick:

```python
# En _h_ask:
if not self._ask_started:
    self.hri.start_speaking(MENU_PROMPT)
    self._ask_started = True
elif self.hri.is_speaking_done():
    self._go(State.LISTEN)

# En _h_listen:
if self.hri.is_listen_done():
    self._stt_text = self.hri.get_listened_text()
    self._go(State.DECIDE)
```

### Nav2 — envío de goals con callbacks

```python
def _send_pose_goal(self, pose, label):
    if not self.nav_client.wait_for_server(timeout_sec=3.0):
        return
    goal = NavigateToPose.Goal()
    goal.pose = pose
    self._nav_active = True
    self.nav_client.send_goal_async(goal).add_done_callback(self._nav_resp_cb)

def _nav_result_cb(self, future):
    self._nav_success = (future.result().status == GoalStatus.STATUS_SUCCEEDED)
    self._nav_done = True
```

### YOLO — acercarse a una persona

Las detecciones llegan en el frame de la cámara. Se transforman a `map` con TF y
se calcula un punto a `hold_distance` antes de la persona:

```python
def _person_goal_pose(self, det):
    tr = self.tf_buffer.lookup_transform(self.map_frame, det['frame_id'], ...)
    # ... rotación + traslación con cuaternión a yaw ...
    target_d = max(0.0, d - self.hold_distance)
    pose.pose.position.x = rx + ux * target_d
    pose.pose.position.y = ry + uy * target_d
```

El goal se refresca cada 1 s mientras se persigue. Cuando la distancia es
`≤ 1.5 m` durante `≥ 2 s`, la misión se da por completada.

### Parseo de voz robusto

Acepta dígitos y palabras clave en español. Si no reconoce nada, elige una opción
aleatoria y lo anuncia:

```python
WORD_TO_OPTION = {
    'uno': 1, 'papelera': 1,
    'dos': 2, 'punto a': 2,
    'tres': 3, 'persona': 3, 'sígueme': 3,
    'cuatro': 4, 'aleatorio': 4, 'paseo': 4,
}

def _parse_option(self, text):
    for ch in text:
        if ch in '1234':
            return int(ch)
    for word, opt in WORD_TO_OPTION.items():
        if word in text:
            return opt
    return None   # → opción aleatoria
```

### Opción 4 — guardar pose de retorno con TF

Antes de enviar el goal a `punto_b` se guarda la pose actual del robot en `map`:

```python
self._return_pose = self._get_current_pose()   # via TF map<-base_frame
self._send_waypoint_goal('punto_b')
self._go(State.RANDOM_OUT)
# ... al llegar:
self._send_pose_goal(self._return_pose, label='regreso')
```

---

## Troubleshooting

| Síntoma | Causa | Solución |
|---|---|---|
| `Servicio STT/TTS no disponible` | HRI no está corriendo o venv no activado | Activa `venv_asr` y lanza `hri_bringup.launch.py` |
| `ModuleNotFoundError: huggingface_hub` | Lanzaste sin `venv_asr` | `source venv_asr/bin/activate` antes del launch |
| `coverage.types.Tracer` AttributeError | Bug numba+coverage | Ya corregido en `numba/misc/coverage_support.py` (try/except) |
| `Unable to connect to a Zenoh router` | Falta el router zenoh | `ros2 run rmw_zenoh_cpp rmw_zenohd &` |
| Topics del sim no aparecen | Sesiones zenoh aisladas | Asegúrate de que todas usan el mismo router (mismo host) |
| Nav2 no acepta goals | Falta pose inicial AMCL | RViz → `2D Pose Estimate`, o `ros2 topic pub /initialpose ...` |
| `wait_for_action_server` AttributeError | API antigua | Ya corregido a `wait_for_server()` |

---

## Pruebas verificadas

| Componente | Test ejecutado | Resultado |
|---|---|---|
| Nav2 (sim Kobuki) | `nav_test goal_x:=0.5 goal_y:=0.0` | ✓ "Navegacion completada con exito" |
| HRI servicios | `ros2 service list | grep -E "stt|tts|extract|yesno"` | ✓ los 4 visibles |
| TTS HF MMS-TTS | `ros2 service call /tts_service ...` | ✓ `success=True`, WAV generado |
| STT Whisper tiny | Carga modelo en stt_service_local | ✓ "STTService inicializado y listo" |
| Mission FSM | `ros2 run pfinal_nav2_hri_manager mission_manager` | ✓ INIT→ASK→TTS menú |
