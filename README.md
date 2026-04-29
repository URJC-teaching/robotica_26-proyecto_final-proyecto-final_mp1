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



# Proyecto final Nav2 + HRI + YOLO (Kobuki)

Este directorio contiene el paquete de integracion del proyecto final para ejecutar un flujo completo:

1. El robot arranca en espera de orden por voz (HRI).
2. Puede seguir a una persona con YOLO 3D manteniendo una distancia objetivo.
3. Puede navegar con Nav2 a puntos con nombre, aleatorios o coordenadas directas.
4. Al terminar cada accion vuelve a preguntar orden.

La integracion se ha hecho con cambios minimos y reutilizando paquetes que ya existen en el workspace.

## Estructura de este directorio

- `pfinal_nav2_hri_manager/`: paquete nuevo de orquestacion.
- `hri_examples/`: ejemplos de HRI (no necesario para ejecutar la solucion final).
- `nav2_example/`: ejemplos de Nav2 (no necesario para ejecutar la solucion final).

## Paquete nuevo: pfinal_nav2_hri_manager

Contenido principal:

- `pfinal_nav2_hri_manager/pfinal_nav2_hri_manager/mission_coordinator.py`
	- Nodo principal con maquina de estados.
	- Estados: espera de orden, seguimiento de persona, navegacion a objetivo.
	- Reutiliza `hri_client` y `navigation_client`.
	- Lee detecciones `yolo_msgs/DetectionArray` en 3D para seguimiento.

- `pfinal_nav2_hri_manager/launch/pfinal_system.launch.py`
	- Launch maestro para modo simulacion y modo real.
	- Incluye Kobuki, Nav2, simple_hri, yolo_bringup y el manager.

- `pfinal_nav2_hri_manager/config/mission_params.yaml`
	- Parametros del comportamiento (distancia de seguimiento, timeouts, topic de deteccion, etc).

- `pfinal_nav2_hri_manager/config/named_goals.yaml`
	- Puntos con nombre (puerta_a, puerta_b, puerta_c).

## Dependencias reutilizadas del workspace

El paquete nuevo no sustituye paquetes existentes. Los integra:

- `kobuki` para robot/simulacion y Nav2 bringup.
- `simple_hri` para STT, TTS, extract y yes/no.
- `hri_client` para cliente HRI asincrono.
- `navigation_client` para envio de objetivos Nav2.
- `yolo_bringup` + `yolo_ros` para deteccion 3D y tracking.

## Comandos de voz soportados

El manager interpreta instrucciones en lenguaje natural simple. Ejemplos:

- `seguir persona`
- `ir a puerta a`
- `ir a puerta_b`
- `ir aleatorio`
- `ir a x 1.2 y -0.8`
- `ir a x 1.2 y -0.8 yaw 1.57`
- `parar` o `cancela`

## Logica de seguimiento de persona

- Se filtran detecciones de clase `person` en topic 3D.
- Si hay tracking ID, el robot mantiene el mismo objetivo.
- Calcula una meta relativa para mantener la distancia deseada (por defecto 1.5 m).
- Convierte esa meta al frame `map` con TF y la envia a Nav2.
- Si alcanza la distancia objetivo, vuelve a modo espera y pregunta nueva orden.
- Si pierde a la persona durante el timeout, cancela seguimiento y vuelve a espera.

## Configurar puntos con nombre

Editar:

- `pfinal_nav2_hri_manager/config/named_goals.yaml`

Formato:

```yaml
goals:
	puerta_a:
		x: 1.0
		y: 0.5
		yaw: 0.0
	puerta_b:
		x: 2.5
		y: -0.7
		yaw: 1.57
```

## Parametros relevantes

Editar:

- `pfinal_nav2_hri_manager/config/mission_params.yaml`

Parametros recomendados:

- `desired_follow_distance`: distancia objetivo a persona (m).
- `follow_distance_tolerance`: tolerancia alrededor de la distancia objetivo.
- `follow_update_period`: cada cuanto actualiza meta de seguimiento.
- `follow_max_step`: salto maximo por actualizacion.
- `lost_person_timeout_sec`: tiempo para declarar perdida de objetivo.
- `min_detection_score`: score minimo de deteccion YOLO.
- `detection_topic`: topic de deteccion 3D usado por el manager.
- `nav_goal_timeout_sec`: timeout de navegacion para metas puntuales.

## Build

Desde la raiz del workspace:

```bash
cd /home/alumno/Documents/ROBOTICA/mp3_ws
colcon build --packages-select pfinal_nav2_hri_manager
source install/setup.bash
```

## Ejecucion en simulacion

```bash
cd /home/alumno/Documents/ROBOTICA/mp3_ws
source install/setup.bash

ros2 launch pfinal_nav2_hri_manager pfinal_system.launch.py \
	mode:=sim \
	hri_stack:=free \
	model:=yolov8n.pt \
	device:=cpu
```

Notas:

- `hri_stack:=free` usa `free_simple_hri.launch.py` (STT/TTS locales + extract hugg).
- Ajusta topicos de camara/depth si tu simulador publica en otros nombres.

## Ejecucion en robot real

```bash
cd /home/alumno/Documents/ROBOTICA/mp3_ws
source install/setup.bash

ros2 launch pfinal_nav2_hri_manager pfinal_system.launch.py \
	mode:=real \
	hri_stack:=free \
	map:=/ruta/a/tu_mapa.yaml \
	astra:=true \
	lidar_a2:=true \
	model:=yolov8n.pt \
	device:=cpu
```

Notas:

- En real puedes usar `hri_stack:=local` o `hri_stack:=cloud`.
- Verifica que los topics de imagen/depth coincidan con la camara real.
- Si cambias namespace de YOLO, actualiza `detection_topic` o el argumento del launch.

## Argumentos principales del launch maestro

- `mode`: `sim` o `real`.
- `start_kobuki`: habilita arranque de base/simulador.
- `start_nav2`: habilita Nav2.
- `start_hri`: habilita servicios HRI.
- `start_yolo`: habilita pila YOLO.
- `start_manager`: habilita nodo coordinador.
- `hri_stack`: `free`, `local`, `cloud`.
- `map`: mapa para localizacion/navegacion.
- `model`, `device`, `threshold`: parametros de YOLO.
- `input_image_topic`, `input_depth_topic`, `input_depth_info_topic`: topics de camara.
- `detection_topic`: topic que consume el manager (por defecto `/yolo/detections_3d`).

## Flujo esperado

1. Arranca el sistema.
2. El robot anuncia que esta listo y pide instruccion.
3. Usuario dice una orden:
	 - seguir persona
	 - ir a puerta_a
	 - ir aleatorio
	 - ir a coordenadas
4. Al terminar, el robot vuelve a preguntar y queda preparado para la siguiente orden.

## Estado actual y extension recomendada

Implementado:

- Conmutacion sim/real desde un unico launch.
- Integracion HRI + Nav2 + YOLO tracking 3D.
- Seguimiento por distancia y retorno automatico a espera.
- Navegacion por punto nombrado, coordenadas y aleatorio.

Siguiente mejora recomendada:

- Añadir una fase de confirmacion por voz antes de ejecutar orden critica.
- Guardar historico de objetivos en un topic de diagnostico.
- Añadir tests de integracion para parsing de comandos.
