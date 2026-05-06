# launch/pfinal_nav2_hri_manager.launch.py
#
# Launch principal del proyecto. Estructura derivada de p6 (full_vff_2d.launch.py):
#   - Nav2 + mapa (kobuki/navigation_sim.launch.py)     [include_nav2:=true]
#   - HRI stack local                                    [include_hri:=true]
#   - YOLO 2D (yolo_bringup) + yolo_depth_node          [use_yolo:=true]
#   - mission_manager (FSM)                              [run_mission:=true]
#   - Nodos de prueba aislados                           [test_hri / test_nav]
#
# Modos:
#   ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py mode:=sim
#   ros2 launch pfinal_nav2_hri_manager pfinal_nav2_hri_manager.launch.py            # mode:=real
#
# YOLO arranca antes que mission_manager (TimerAction de mission_delay_sec, 12 s
# por defecto) para que el modelo termine de cargar y publique camera_info antes
# de que la FSM empiece a leer detecciones.

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = (mode == 'sim')

    # Topics y frames según modo (mismas convenciones que p6/full_vff_3d.launch.py)
    if mode == 'sim':
        image_topic  = '/rgbd_camera/image'
        info_topic   = '/rgbd_camera/camera_info'
        camera_frame = 'camera_link'
        base_frame   = 'base_link'
    else:
        image_topic  = '/camera/rgb/image_raw'
        info_topic   = '/camera/rgb/camera_info'   # intrínsecas de la cámara RGB (misma que YOLO usa)
        camera_frame = 'camera_rgb_frame'
        base_frame   = 'base_footprint'

    pkg_share = get_package_share_directory('pfinal_nav2_hri_manager')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    # mp3_ws/ — para localizar map_edited.yaml y venv_asr/
    ws_root    = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    map_edited = os.path.join(ws_root, 'map_edited.yaml')

    # venv_asr contiene torch / ultralytics / whisper / transformers.
    # Sin esto los nodos hijos (yolo_node, stt_*, tts_*) no encuentran sus deps.
    venv_site = os.path.join(ws_root, 'venv_asr', 'lib', 'python3.12', 'site-packages')
    pythonpath = (
        (venv_site + ':' + os.environ.get('PYTHONPATH', '')).rstrip(':')
        if os.path.isdir(venv_site)
        else os.environ.get('PYTHONPATH', '')
    )

    include_nav2 = LaunchConfiguration('include_nav2').perform(context).lower() == 'true'
    include_hri  = LaunchConfiguration('include_hri').perform(context).lower() == 'true'
    use_yolo     = LaunchConfiguration('use_yolo').perform(context).lower() == 'true'
    mission_delay = float(
        LaunchConfiguration('mission_delay_sec').perform(context) or '12.0'
    )

    actions = [SetEnvironmentVariable('PYTHONPATH', pythonpath)]

    # ---- Nav2 (kobuki bringup, opcional) ----
    if include_nav2:
        try:
            kobuki_share = get_package_share_directory('kobuki')
            nav_launch = (
                'navigation_sim.launch.py' if mode == 'sim' else 'navigation.launch.py'
            )
            nav_launch_path = os.path.join(kobuki_share, 'launch', nav_launch)
            if os.path.isfile(nav_launch_path):
                nav_args = {
                    'use_sim_time': str(use_sim_time).lower(),
                    'rviz':         'False',
                }
                if os.path.isfile(map_edited):
                    nav_args['map'] = map_edited
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch_path),
                    launch_arguments=nav_args.items(),
                ))
        except PackageNotFoundError:
            pass

    # ---- HRI stack (stt_service_local + tts_service_local + extract + yesno) ----
    if include_hri:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'hri_bringup.launch.py')
            ),
        ))

    # ---- YOLO 2D ligero ----
    # Solo yolo_node (sin tracking_node ni detect_3d_node de yolo_ros, ambos
    # LifecycleNodes pesados que se quedan colgados en VM sin GPU).
    # La conversión 2D→distancia la hace nuestro yolo_depth_node con la
    # altura del bbox + intrínseca (apaño tipo p6 yolo_class_detector_node_2d).
    if use_yolo:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yolo_bringup'),
                    'launch', 'yolo.launch.py',
                )
            ),
            launch_arguments={
                'input_image_topic': image_topic,
                'model':             'yolov8n.pt',
                'device':            'cpu',
                'use_tracking':      'False',
                'use_3d':            'False',
                'use_debug':         'False',
                'imgsz_height':      '320',
                'imgsz_width':       '320',
                'threshold':         '0.4',
                'image_reliability': '2',   # BEST_EFFORT (sensor_data)
                'use_sim_time':      str(use_sim_time).lower(),
            }.items(),
        ))

        actions.append(Node(
            package='pfinal_nav2_hri_manager',
            executable='yolo_depth',
            name='yolo_depth_node',
            output='screen',
            parameters=[{
                'use_sim_time':       use_sim_time,
                'camera_frame':       camera_frame,
                'target_class':       'person',
                'person_height_m':    1.7,
                'min_bbox_height_px': 20,
                'max_distance_m':     6.0,
            }],
            remappings=[
                ('detections', '/yolo/detections'),
                ('camera_info', info_topic),
                ('detections_3d', '/detections_3d'),
            ],
        ))

    # ---- Mission manager (retrasado para que YOLO/HRI terminen de cargar) ----
    mission_node = Node(
        package='pfinal_nav2_hri_manager',
        executable='mission_manager',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'waypoints_file':       waypoints_file,
            'detections_topic':     '/detections_3d',
            'person_class':         'person',
            'person_hold_distance': 1.5,
            'person_hold_time':     2.0,
            'goal_timeout_sec':     110.0,
            'post_goal_wait_sec':   2.0,
            'base_frame':           base_frame,
            'map_frame':            'map',
            'random_xmin':           0.05,
            'random_xmax':           1.45,
            'random_ymin':           2.6,
            'random_ymax':           5.5,
        }],
        condition=IfCondition(LaunchConfiguration('run_mission')),
    )
    actions.append(TimerAction(period=mission_delay, actions=[mission_node]))

    # ---- Nodos de prueba opcionales (sin retraso) ----
    actions.append(Node(
        package='pfinal_nav2_hri_manager',
        executable='hri_test',
        name='hri_test_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('test_hri')),
    ))
    actions.append(Node(
        package='pfinal_nav2_hri_manager',
        executable='nav_test',
        name='nav_test_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_x':       0.5,
            'goal_y':       0.0,
            'goal_theta':   0.0,
        }],
        condition=IfCondition(LaunchConfiguration('test_nav')),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='real',
            description='real | sim — selecciona topics y frames'),
        DeclareLaunchArgument(
            'include_nav2', default_value='false',
            description='Lanzar Nav2 + mapa (kobuki bringup)'),
        DeclareLaunchArgument(
            'include_hri', default_value='true',
            description='Lanzar el stack HRI local (STT + TTS + extract + yesno)'),
        DeclareLaunchArgument(
            'use_yolo', default_value='true',
            description='Lanzar YOLO 2D + yolo_depth_node'),
        DeclareLaunchArgument(
            'run_mission', default_value='true',
            description='Lanzar el mission manager (FSM)'),
        DeclareLaunchArgument(
            'mission_delay_sec', default_value='12.0',
            description='Segundos a esperar antes de lanzar mission_manager '
                        '(deja tiempo a YOLO/HRI a cargar modelos)'),
        DeclareLaunchArgument(
            'test_hri', default_value='false',
            description='Lanzar el nodo de prueba HRI'),
        DeclareLaunchArgument(
            'test_nav', default_value='false',
            description='Lanzar el nodo de prueba Nav2'),
        OpaqueFunction(function=launch_setup),
    ])
