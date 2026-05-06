# launch/find_person.launch.py
#
# Prueba autónoma YOLO + Nav2: gira buscando persona, navega a 1.5 m de ella y para.
#
# Modo simulación (Nav2 ya debe estar corriendo aparte o con include_nav2:=true):
#   ros2 launch pfinal_nav2_hri_manager find_person.launch.py mode:=sim
#
# Modo real (Nav2 ya corriendo):
#   ros2 launch pfinal_nav2_hri_manager find_person.launch.py
#
# Pipeline de topics:
#   image → yolo_node → /yolo/detections
#   camera_info + /yolo/detections → yolo_depth_node → /detections_3d
#   /detections_3d → find_person_test_node → navigate_to_pose

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
    mode        = LaunchConfiguration('mode').perform(context)
    use_sim_time = (mode == 'sim')

    if mode == 'sim':
        image_topic  = '/rgbd_camera/image'
        info_topic   = '/rgbd_camera/camera_info'
        camera_frame = 'camera_link'
        base_frame   = 'base_link'
    else:
        image_topic  = '/camera/rgb/image_raw'
        info_topic   = '/camera/rgb/camera_info'   # intrínsecas de la cámara RGB
        camera_frame = 'camera_rgb_frame'
        base_frame   = 'base_footprint'

    ws_root  = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(
            get_package_share_directory('pfinal_nav2_hri_manager')
        )))
    )
    venv_site = os.path.join(ws_root, 'venv_asr', 'lib', 'python3.12', 'site-packages')
    pythonpath = (
        (venv_site + ':' + os.environ.get('PYTHONPATH', '')).rstrip(':')
        if os.path.isdir(venv_site)
        else os.environ.get('PYTHONPATH', '')
    )

    include_nav2 = LaunchConfiguration('include_nav2').perform(context).lower() == 'true'
    use_yolo     = LaunchConfiguration('use_yolo').perform(context).lower() == 'true'
    test_delay   = float(LaunchConfiguration('test_delay_sec').perform(context) or '12.0')

    actions = [SetEnvironmentVariable('PYTHONPATH', pythonpath)]

    # ---- Nav2 opcional ----
    if include_nav2:
        try:
            kobuki_share = get_package_share_directory('kobuki')
            nav_launch   = (
                'navigation_sim.launch.py' if mode == 'sim' else 'navigation.launch.py'
            )
            nav_launch_path = os.path.join(kobuki_share, 'launch', nav_launch)
            if os.path.isfile(nav_launch_path):
                map_edited = os.path.join(ws_root, 'map_edited.yaml')
                nav_args   = {'use_sim_time': str(use_sim_time).lower(), 'rviz': 'False'}
                if os.path.isfile(map_edited):
                    nav_args['map'] = map_edited
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch_path),
                    launch_arguments=nav_args.items(),
                ))
        except PackageNotFoundError:
            pass

    # ---- YOLO ----
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
                'image_reliability': '2',
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
                ('detections',   '/yolo/detections'),
                ('camera_info',  info_topic),
                ('detections_3d', '/detections_3d'),
            ],
        ))

    # ---- Nodo de prueba (retrasado para que YOLO cargue el modelo) ----
    test_node = Node(
        package='pfinal_nav2_hri_manager',
        executable='find_person_test',
        name='find_person_test_node',
        output='screen',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'detections_topic':   '/detections_3d',
            'person_class':       'person',
            'hold_distance':      1.5,
            'search_angular_spd': 0.5,
            'search_timeout_sec': 60.0,
            'goal_timeout_sec':   90.0,
            'base_frame':         base_frame,
            'map_frame':          'map',
        }],
    )
    actions.append(
        TimerAction(period=test_delay, actions=[test_node])
        if use_yolo else test_node
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='real',
            description='real | sim'),
        DeclareLaunchArgument(
            'include_nav2', default_value='false',
            description='Lanzar Nav2 + mapa (kobuki bringup)'),
        DeclareLaunchArgument(
            'use_yolo', default_value='true',
            description='Lanzar YOLO 2D + yolo_depth_node'),
        DeclareLaunchArgument(
            'test_delay_sec', default_value='12.0',
            description='Segundos de espera antes de arrancar el nodo de prueba '
                        '(para que YOLO termine de cargar el modelo)'),
        OpaqueFunction(function=launch_setup),
    ])
